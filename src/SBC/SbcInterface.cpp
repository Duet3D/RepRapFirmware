/*
 * SbcInterface.cpp
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */

#include "SbcInterface.h"
#include "DataTransfer.h"

#if HAS_SBC_INTERFACE

#include <GCodes/GCodeBuffer/ExpressionParser.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Heating/Heat.h>
#include <Movement/Move.h>
#include <Platform/Platform.h>
#include <PrintMonitor/PrintMonitor.h>
#include <Tools/Filament.h>
#include <Platform/RepRap.h>
#include <RepRapFirmware.h>
#include <Platform/Tasks.h>
#include <Hardware/SoftwareReset.h>
#include <Hardware/ExceptionHandlers.h>
#include <Platform/TaskPriorities.h>
#include <AppNotifyIndices.h>

extern char _estack;		// defined by the linker

// This function is not used in this class
const ObjectModelClassDescriptor *SbcInterface::GetObjectModelClassDescriptor() const noexcept { return nullptr; }

// The SBC task's stack size needs to be enough to support rr_model and expression evaluation
// In RRF 3.3beta3, 744 is only just enough for simple expression evaluation in a release build when using globals
// In 3.3beta3.1 we have saved ~151 bytes (37 words) of stack compared to 3.3beta3
// In 3.5.2, the stack size is increased again to allow for nested functions to be properly evaluated (up to 7 nested max calls e.g.)
#if defined(DEBUG)
constexpr size_t SBCTaskStackWords = 1600;			// debug builds use more stack
#else
constexpr size_t SBCTaskStackWords = 1400;
#endif

constexpr uint32_t SbcYieldTimeout = 10;

static Task<SBCTaskStackWords> *sbcTask;

extern "C" [[noreturn]] void SBCTaskStart(void * pvParameters) noexcept
{
	reprap.GetSbcInterface().TaskLoop();
}

SbcInterface::SbcInterface() noexcept : isConnected(false), numDisconnects(0), numTimeouts(0), numSbcTimeouts(0), lastTransferTime(0),
	maxDelayBetweenTransfers(SpiTransferDelay), maxFileOpenDelay(SpiFileOpenDelay), numMaxEvents(SpiEventsRequired),
	delaying(false), numEvents(0), reportPause(false), reportPauseWritten(false), printAborted(false),
	codeBuffer(nullptr), rxPointer(0), txPointer(0), txEnd(0), sendBufferUpdate(true),
	fileMutex(), numOpenFiles(0), fileSemaphore(), fileOperation(FileOperation::none), fileOperationPending(false),
	gcodeReply(), gcodeReplyMutex()
#ifdef TRACK_FILE_CODES
	, fileCodesRead(0), fileCodesHandled(0), fileMacrosRunning(0), fileMacrosClosing(0)
#endif
{
}

void SbcInterface::Init() noexcept
{
	fileMutex.Create("SBCFile");
	gcodeReplyMutex.Create("SBCReply");
	codeBuffer = (char *)new uint32_t[(SpiCodeBufferSize + 3)/4];
	transfer.Init();
	sbcTask = new Task<SBCTaskStackWords>();
	sbcTask->Create(SBCTaskStart, "SBC", nullptr, TaskPriority::SbcPriority);
	iapRamAvailable = (const char*)&_estack - Tasks::GetHeapTop();
}

[[noreturn]] void SbcInterface::TaskLoop() noexcept
{
	transfer.InitFromTask();
	transfer.StartNextTransfer();

	bool busy = false, transferComplete = false, hadTimeout = false, hadSbcTimeout = false, hadReset = false;
	for (;;)
	{
		// Try to exchange data with the SBC
		transferComplete = hadTimeout = hadReset = false;
		do
		{
			busy = false;
			state = transfer.DoTransfer();
			const uint32_t transferStartTime = millis();
			switch (state)
			{
			case TransferState::doingFullTransfer:
				hadTimeout = !TaskBase::TakeIndexed(NotifyIndices::SbcInterface, isConnected ? SpiConnectionTimeout : TaskBase::TimeoutUnlimited);
				hadSbcTimeout = hadTimeout && millis() - transferStartTime < SpiConnectionTimeout + SbcYieldTimeout;
				break;
			case TransferState::doingPartialTransfer:
				hadTimeout = !TaskBase::TakeIndexed(NotifyIndices::SbcInterface, SpiTransferTimeout);
				hadSbcTimeout = hadTimeout && millis() - transferStartTime < SpiTransferTimeout + SbcYieldTimeout;
				break;
			case TransferState::finishingTransfer:
				busy = true;
				break;
			case TransferState::connectionTimeout:
				hadTimeout = hadSbcTimeout = true;
				break;
			case TransferState::connectionReset:
				hadReset = true;
				break;
			case TransferState::finished:
				transferComplete = true;
				break;
			}
		} while (busy);

		// Handle connection errors
		if (isConnected && (hadReset || hadTimeout))
		{
			isConnected = false;
			numDisconnects++;
			if (hadTimeout)
			{
				numTimeouts++;
				if (hadSbcTimeout)
				{
					numSbcTimeouts++;
				}
				reprap.GetPlatform().MessageF(NetworkInfoMessage, "Lost connection to SBC due to %s timeout\n", hadSbcTimeout ? "remote" : "local");
			}
			else
			{
				reprap.GetPlatform().Message(NetworkInfoMessage, "Lost connection to SBC due to connection reset\n");
			}

			// Invalidate local resources
			InvalidateResources();
			if (hadReset)
			{
				// Let the main task invalidate resources before processing new data
				TaskBase::TakeIndexed(NotifyIndices::SbcInterface, SbcYieldTimeout);
			}
		}

		// Deal with received data
		if (transferComplete)
		{
			if (!isConnected)
			{
				isConnected = true;
				reprap.GetPlatform().Message(NetworkInfoMessage, "Connection to SBC established!\n");
			}

			// Handle exchanged data and kick off the next transfer
			ExchangeData();
			transfer.StartNextTransfer();
		}
		else if (hadTimeout || hadReset)
		{
			// Reset the SPI connection if no data could be exchanged
			transfer.ResetConnection(hadTimeout);
		}
	}
}

void SbcInterface::ExchangeData() noexcept
{
	// Process incoming packets
	bool codeBufferAvailable = true;
	for (size_t i = 0; i < transfer.PacketsToRead(); i++)
	{
		const PacketHeader * const packet = transfer.ReadPacket();
		if (packet == nullptr)
		{
			if (reprap.Debug(Module::SbcInterface))
			{
				debugPrintf("Error trying to read next SPI packet\n");
			}
			break;
		}

		if (packet->request >= (uint16_t)SbcRequest::InvalidRequest)
		{
			REPORT_INTERNAL_ERROR;
			break;
		}

		bool packetAcknowledged = true;
		switch ((SbcRequest)packet->request)
		{
		// Perform an emergency stop
		case SbcRequest::EmergencyStop:
			reprap.EmergencyStop();
			break;

		// Reset the controller
		case SbcRequest::Reset:
			reprap.EmergencyStop();							// turn off heaters and motors, tell expansion boards to reset
			SoftwareReset(SoftwareResetReason::userFromSbc);
			break;

		// Perform a G/M/T-code
		case SbcRequest::Code:
		{
			// Read the next code
			if (packet->length == 0)
			{
				reprap.GetPlatform().Message(WarningMessage, "Received empty binary code, discarding\n");
				break;
			}

			const CodeHeader *code = reinterpret_cast<const CodeHeader*>(transfer.ReadData(packet->length));
			const GCodeChannel channel(code->channel);
			if (channel.IsValid())
			{
				GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
				if (gb == nullptr)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				if (gb->IsInvalidated())
				{
					// Don't deal with codes that will be thrown away
					break;
				}

				// Check if a GB is waiting for a macro file to be started
				if (gb->IsWaitingForMacro() && !gb->IsMacroRequestPending())
				{
					gb->ResolveMacroRequest(false, false);
#ifdef TRACK_FILE_CODES
					if (gb->IsFileChannel())
					{
						fileMacrosRunning++;
					}
#endif
				}

				// Don't process any more codes if we failed to store them last time...
				if (!codeBufferAvailable)
				{
					packetAcknowledged = false;
					break;
				}

				TaskCriticalSectionLocker locker;

				// Make sure no existing codes are overwritten
				uint16_t bufferedCodeSize = sizeof(BufferedCodeHeader) + packet->length;
				if ((txEnd == 0 && bufferedCodeSize > max<uint16_t>(rxPointer, SpiCodeBufferSize - txPointer)) ||
						(txEnd != 0 && bufferedCodeSize > rxPointer - txPointer))
				{
#if false
					// This isn't enabled because the debug call plus critical section would lead to software resets
					debugPrintf("Failed to store code, RX/TX %d/%d-%d\n", rxPointer, txPointer, txEnd);
#endif
					packetAcknowledged = codeBufferAvailable = false;
					break;
				}

				// Overlap if necessary
				if (txPointer + bufferedCodeSize > SpiCodeBufferSize)
				{
					txEnd = txPointer;
					txPointer = 0;
				}

				// Store the buffer header
				BufferedCodeHeader *bufHeader = reinterpret_cast<BufferedCodeHeader *>(codeBuffer + txPointer);
				bufHeader->isPending = true;
				bufHeader->length = packet->length;

				// Store the corresponding code. Binary codes are always aligned on a 4-byte boundary
				uint32_t *dst = reinterpret_cast<uint32_t *>(codeBuffer + txPointer + sizeof(BufferedCodeHeader));
				const uint32_t *src = reinterpret_cast<const uint32_t *>(code);
				memcpyu32(dst, src, packet->length / sizeof(uint32_t));
				txPointer += bufferedCodeSize;
				sendBufferUpdate = true;
			}
			else
			{
				REPORT_INTERNAL_ERROR;
			}
			break;
		}

		// Get the object model
		case SbcRequest::GetObjectModel:
		{
			String<StringLength100> key;
			String<StringLength20> flags;
			transfer.ReadGetObjectModel(packet->length, key.GetRef(), flags.GetRef());

			try
			{
				OutputBuffer *outBuf = reprap.GetModelResponse(nullptr, key.c_str(), flags.c_str());
				if (outBuf != nullptr && outBuf->Length() > SbcTransferBufferSize - sizeof(PacketHeader) - sizeof(StringHeader))
				{
					if (!transfer.WriteObjectModel(nullptr))
					{
						// Cannot store this object model response even if we wanted to
						reprap.GetPlatform().MessageF(ErrorMessage, "Cannot store excessively long object model response, discarding request (total length %d, key %s, flags %s)", outBuf->Length(), key.c_str(), flags.c_str());
					}
					else
					{
						// Failed to write an empty object model response, try again later
						packetAcknowledged = false;
					}
					OutputBuffer::ReleaseAll(outBuf);
				}
				else if (outBuf == nullptr || !transfer.WriteObjectModel(outBuf))
				{
					// Failed to write the whole object model, try again later
					packetAcknowledged = false;
					OutputBuffer::ReleaseAll(outBuf);
				}
			}
			catch (const GCodeException& e)
			{
				// Get the error message and send it back to DSF
				OutputBuffer *buf;
				if (OutputBuffer::Allocate(buf))
				{
					String<StringLength100> errorMessage;
					e.GetMessage(errorMessage.GetRef(), nullptr);
					buf->cat(errorMessage.c_str());
					if (!transfer.WriteObjectModel(buf))
					{
						OutputBuffer::ReleaseAll(buf);
						packetAcknowledged = false;
					}
				}
				else
				{
					packetAcknowledged = false;
				}
			}
			break;
		}

		// Print is about to be started, set file print info
		case SbcRequest::SetPrintFileInfo:
		{
			String<MaxFilenameLength> filename;
			transfer.ReadPrintStartedInfo(packet->length, filename.GetRef(), fileInfo);
			reprap.GetPrintMonitor().SetPrintingFileInfo(filename.c_str(), fileInfo);
			break;
		}

		// Print has been stopped
		case SbcRequest::PrintStopped:
		{
			const PrintStoppedReason reason = transfer.ReadPrintStoppedInfo();
			if (reason == PrintStoppedReason::abort)
			{
				// Stop the print with the given reason
				printAborted = true;
				InvalidateBufferedCodes(GCodeChannel::File);
#if SUPPORT_ASYNC_MOVES
				InvalidateBufferedCodes(GCodeChannel::File2);
#endif
			}
			else
			{
				// Just mark the print files as finished
				GCodeBuffer * const fileGb = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::File);
				MutexLocker fileLocker(fileGb->mutex, SbcYieldTimeout);
#if SUPPORT_ASYNC_MOVES
				GCodeBuffer * const file2Gb = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::File2);
				MutexLocker file2Locker(file2Gb->mutex, SbcYieldTimeout);
				if (fileLocker.IsAcquired() && file2Locker.IsAcquired())
#else
				if (fileLocker.IsAcquired())
#endif
				{
					fileGb->SetPrintFinished();
#if SUPPORT_ASYNC_MOVES
					file2Gb->SetPrintFinished();
#endif
				}
				else
				{
					packetAcknowledged = false;
				}
			}
			break;
		}

		// Macro file has been finished
		case SbcRequest::MacroCompleted:
		{
			bool error;
			const GCodeChannel channel = transfer.ReadMacroCompleteInfo(error);
			if (channel.IsValid())
			{
				GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
				if (gb == nullptr)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				if (gb->IsWaitingForMacro() && !gb->IsMacroRequestPending())
				{
					gb->ResolveMacroRequest(error, true);
					if (reprap.Debug(Module::SbcInterface))
					{
						debugPrintf("Waiting macro completed on channel %u\n", channel.ToBaseType());
					}
				}
				else
				{
					MutexLocker locker(gb->mutex, SbcYieldTimeout);
					if (locker.IsAcquired())
					{
						if (error)
						{
							gb->CurrentFileMachineState().CloseFile();
							gb->PopState(false);
							gb->Init();
						}
						else
						{
#ifdef TRACK_FILE_CODES
							if (gb->IsFileChannel())
							{
								fileMacrosClosing++;
							}
#endif
							gb->SetFileFinished();
						}

						if (reprap.Debug(Module::SbcInterface))
						{
							debugPrintf("Macro completed on channel %u\n", channel.ToBaseType());
						}
					}
					else
					{
						packetAcknowledged = false;
					}
				}
			}
			else
			{
				REPORT_INTERNAL_ERROR;
			}
			break;
		}

		// Lock movement and wait for standstill. Currently this is used only by M505, so we lock all movement systems.
		case SbcRequest::LockMovementAndWaitForStandstill:
		{
			const GCodeChannel channel = transfer.ReadCodeChannel();
			if (channel.IsValid())
			{
				GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
				if (gb == nullptr)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				MutexLocker locker(gb->mutex, SbcYieldTimeout);
				if (locker.IsAcquired() && reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(*gb))
				{
					transfer.WriteLocked(channel);
				}
				else
				{
					packetAcknowledged = false;
				}
			}
			else
			{
				REPORT_INTERNAL_ERROR;
			}
			break;
		}

		// Unlock everything
		case SbcRequest::Unlock:
		{
			const GCodeChannel channel = transfer.ReadCodeChannel();
			if (channel.IsValid())
			{
				GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
				if (gb == nullptr)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				MutexLocker locker(gb->mutex, SbcYieldTimeout);
				if (locker.IsAcquired())
				{
					reprap.GetGCodes().UnlockAll(*gb);
				}
				else
				{
					packetAcknowledged = false;
				}
			}
			else
			{
				REPORT_INTERNAL_ERROR;
			}
			break;
		}

		// Write the first chunk of the IAP binary
		case SbcRequest::WriteIap:
		{
			reprap.PrepareToLoadIap();
			ReceiveAndStartIap(transfer.ReadData(packet->length), packet->length);
			break;
		}

		// Assign filament (deprecated)
		// Return a file chunk (deprecated)
		case SbcRequest::AssignFilament_deprecated:
		case SbcRequest::FileChunk_deprecated:
			(void)transfer.ReadData(packet->length);		// skip the packet content
			break;

		// Evaluate an expression
		case SbcRequest::EvaluateExpression:
		{
			String<MaxGCodeLength> expression;
			const GCodeChannel channel = transfer.ReadEvaluateExpression(packet->length, expression.GetRef());
			if (channel.IsValid())
			{
				GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
				if (gb == nullptr)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				// If there is a macro file waiting, the first instruction must be conditional. Don't block any longer...
				if (gb->IsWaitingForMacro())
				{
					gb->ResolveMacroRequest(false, false);
#ifdef TRACK_FILE_CODES
					if (gb->IsFileChannel())
					{
						fileMacrosRunning++;
					}
#endif
				}

				try
				{
					// Evaluate the expression and send the result to DSF
					MutexLocker lock(gb->mutex, SbcYieldTimeout);
					if (lock.IsAcquired())
					{
						ExpressionParser parser(gb, expression.c_str(), expression.c_str() + expression.strlen());
						const ExpressionValue val = parser.Parse();
						if (val.GetType() == TypeCode::HeapArray)
						{
							// Write heap arrays as JSON
							OutputBuffer *json;
							if (OutputBuffer::Allocate(json))
							{
								ObjectExplorationContext context;
								ReportHeapArrayAsJson(json, context, nullptr, val.ahVal, "");
								packetAcknowledged = transfer.WriteEvaluationResult(expression.c_str(), json);
							}
							else
							{
								packetAcknowledged = false;
							}
						}
						else
						{
							// Write plain result
							packetAcknowledged = transfer.WriteEvaluationResult(expression.c_str(), val);
						}
					}
					else
					{
						packetAcknowledged = false;
					}
				}
				catch (const GCodeException& e)
				{
					// Get the error message and send it back to DSF
					String<StringLength100> errorMessage;
					e.GetMessage(errorMessage.GetRef(), nullptr);
					packetAcknowledged = transfer.WriteEvaluationError(expression.c_str(), errorMessage.c_str());
				}
			}
			else
			{
				REPORT_INTERNAL_ERROR;
			}
			break;
		}

		// Send a firmware message, typically a response to a command that has been passed to DSF.
		// These responses can get quite long (e.g. responses to M20) so receive it into an OutputBuffer.
		case SbcRequest::Message:
		{
			OutputBuffer *buf;
			if (OutputBuffer::Allocate(buf))
			{
				MessageType type;
				if (transfer.ReadMessage(type, buf))
				{
					// FIXME Push flag is not supported yet
					reprap.GetPlatform().Message(type, buf);
				}
				else
				{
					// Not enough memory for reading the whole message, try again later
					OutputBuffer::ReleaseAll(buf);
					packetAcknowledged = false;
				}
			}
			break;
		}

		// Macro file has been started
		case SbcRequest::MacroStarted:
		{
			const GCodeChannel channel = transfer.ReadCodeChannel();
			if (channel.IsValid())
			{
				GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
				if (gb == nullptr)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				if (gb->IsWaitingForMacro() && !gb->IsMacroRequestPending())
				{
					// File exists and is open, but no code has arrived yet
					gb->ResolveMacroRequest(false, false);
#ifdef TRACK_FILE_CODES
					if (gb->IsFileChannel())
					{
						fileMacrosRunning++;
					}
#endif
				}
				else if (channel != GCodeChannel::Daemon)
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Macro file has been started on channel %s but none was requested\n", channel.ToString());
				}
				else
				{
					// dameon.g is running, now the OM may report the file is being executed
					reprap.InputsUpdated();
				}
			}
			else
			{
				REPORT_INTERNAL_ERROR;
			}
			break;
		}

		// Invalidate all files and codes on a given channel
		case SbcRequest::InvalidateChannel:
		{
			const GCodeChannel channel = transfer.ReadCodeChannel();
			if (channel.IsValid())
			{
				GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
				if (gb == nullptr)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				if (gb->IsWaitingForMacro())
				{
					gb->ResolveMacroRequest(true, false);
				}

				MutexLocker locker(gb->mutex, SbcYieldTimeout);
				if (locker.IsAcquired())
				{
					// Note that we do not call StopPrint here or set any other variables; DSF already does that
					gb->AbortFile(true);
					gb->FileAbortSent();	// don't notify the SBC
					InvalidateBufferedCodes(channel);
				}
				else
				{
					packetAcknowledged = false;
				}
			}
			else
			{
				REPORT_INTERNAL_ERROR;
			}
			break;
		}

		// Set the content of a variable
		case SbcRequest::SetVariable:
		{
			bool createVariable;
			String<MaxVariableNameLength> varName;
			String<MaxGCodeLength> expression;
			const GCodeChannel channel = transfer.ReadSetVariable(createVariable, varName.GetRef(), expression.GetRef());

			// Make sure we can access the gb safely...
			if (!channel.IsValid())
			{
				REPORT_INTERNAL_ERROR;
				break;
			}

			GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
			if (gb == nullptr)
			{
				REPORT_INTERNAL_ERROR;
				break;
			}

			MutexLocker lock(gb->mutex, SbcYieldTimeout);
			if (!lock.IsAcquired())
			{
				packetAcknowledged = false;
				break;
			}

			// Get the variable set
			const bool isGlobal = StringStartsWith(varName.c_str(), "global.");
			if (!isGlobal && !StringStartsWith(varName.c_str(), "var."))
			{
				packetAcknowledged = transfer.WriteSetVariableError(varName.c_str(), "expected a global or local variable");
				break;
			}
			WriteLockedPointer<VariableSet> vset = (isGlobal) ? reprap.GetGlobalVariablesForWriting() : WriteLockedPointer<VariableSet>(nullptr, &gb->GetVariables());

			// Make a copy of the variable name excluding prefix so that we can terminate the name at the first '[' if necessary
			String<MaxVariableNameLength> shortVarName;
			shortVarName.copy(varName.c_str() + strlen(isGlobal ? "global." : "var."));

			// Check for index expressions after the variable name. DSF will have stripped out any spaces except for those within index expressions.
			uint32_t indices[MaxExpressionArrayIndices];
			size_t numIndices = 0;
			if (!createVariable)
			{
				const char* indexStart = strchr(shortVarName.c_str(), '[');
				if (indexStart != nullptr)
				{
					const size_t firstIndexOffset = indexStart - shortVarName.c_str();
					bool hadError = false;
					do
					{
						if (numIndices == MaxExpressionArrayIndices)
						{
							expression.printf("too many array indices in '%s'", varName.c_str());
							hadError = true;
							break;
						}

						try
						{
							ExpressionParser indexParser(gb, indexStart + 1, shortVarName.c_str() + shortVarName.strlen());
							const uint32_t indexExpr = indexParser.ParseUnsigned();
							indexStart = indexParser.GetEndptr();
							if (*indexStart != ']')
							{
								expression.printf("missing ']' in '%s'", varName.c_str());
								hadError = true;
								break;
							}

							indices[numIndices++] = indexExpr;
							++indexStart;								// skip the ']'
						}
						catch (const GCodeException& e)
						{
							e.GetMessage(expression.GetRef(), nullptr);
							hadError = true;
							break;
						}
					} while (*indexStart == '[');

					if (hadError)
					{
						packetAcknowledged = transfer.WriteSetVariableError(varName.c_str(), expression.c_str());
						break;
					}

					shortVarName[firstIndexOffset] = 0;					// terminate the short variable name at the first '['
				}
			}

			// Check if the variable is valid
			Variable * const v = vset->Lookup(shortVarName.c_str(), false);
			if (createVariable && v != nullptr)
			{
				// For now we don't allow an existing variable to be reassigned using a 'var' or 'global' statement. We may need to allow it for 'global' statements.
				// Save memory by re-using 'expression' to capture the error message
				expression.printf("variable '%s' already exists", varName.c_str());
				packetAcknowledged = transfer.WriteSetVariableError(varName.c_str(), expression.c_str());
				break;
			}
			if (!createVariable && v == nullptr)
			{
				// Save memory by re-using 'expression' to capture the error message
				expression.printf("unknown variable '%s'", varName.c_str());
				packetAcknowledged = transfer.WriteSetVariableError(varName.c_str(), expression.c_str());
				break;
			}

			// Evaluate the expression and assign it
			try
			{
				ExpressionParser parser(gb, expression.c_str(), expression.c_str() + expression.strlen());
				ExpressionValue ev = parser.Parse();
				if (v == nullptr)
				{
					// DSF doesn't provide indent values but instructs RRF to delete local variables when the current block ends
					vset->InsertNew(shortVarName.c_str(), ev, 0);
				}
				else if (numIndices == 0)
				{
					v->Assign(ev);
				}
				else
				{
					v->AssignIndexed(ev, numIndices, indices);
				}

				if (ev.GetType() == TypeCode::HeapArray)
				{
					// Write heap arrays as JSON
					OutputBuffer *json;
					if (OutputBuffer::Allocate(json))
					{
						ObjectExplorationContext context;
						ReportHeapArrayAsJson(json, context, nullptr, ev.ahVal, "");
						packetAcknowledged = transfer.WriteSetVariableResult(varName.c_str(), json);
					}
					else
					{
						packetAcknowledged = false;
					}
				}
				else
				{
					// Write plain result
					packetAcknowledged = transfer.WriteSetVariableResult(varName.c_str(), ev);
				}

				if (isGlobal)
				{
					reprap.GlobalUpdated();
				}
			}
			catch (const GCodeException& e)
			{
				// Get the error message and send it back to DSF
				// Save memory by re-using 'expression' to capture the error message
				e.GetMessage(expression.GetRef(), nullptr);
				packetAcknowledged = transfer.WriteSetVariableError(varName.c_str(), expression.c_str());
			}
			break;
		}

		// Delete a local variable
		case SbcRequest::DeleteLocalVariable:
		{
			String<MaxVariableNameLength> varName;
			const GCodeChannel channel = transfer.ReadDeleteLocalVariable(varName.GetRef());

			// Make sure we can access the gb safely...
			if (!channel.IsValid())
			{
				REPORT_INTERNAL_ERROR;
				break;
			}

			GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
			if (gb == nullptr)
			{
				REPORT_INTERNAL_ERROR;
				break;
			}

			MutexLocker lock(gb->mutex, SbcYieldTimeout);
			if (!lock.IsAcquired())
			{
				packetAcknowledged = false;
				break;
			}

			// Try to delete the variable again
			WriteLockedPointer<VariableSet> vset = WriteLockedPointer<VariableSet>(nullptr, &gb->GetVariables());
			vset.Ptr()->Delete(varName.c_str());
			break;
		}

		// Result of a file exists check
		case SbcRequest::CheckFileExistsResult:
			if (fileOperation == FileOperation::checkFileExists)
			{
				fileSuccess = transfer.ReadBoolean();
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;

		// Result of a deletion request
		case SbcRequest::FileDeleteResult:
			if (fileOperation == FileOperation::deleteFileOrDirectory || fileOperation == FileOperation::deleteFileOrDirectoryRecursively)
			{
				fileSuccess = transfer.ReadBoolean();
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;

		// Result of a file open request
		case SbcRequest::OpenFileResult:
			if (fileOperation == FileOperation::openRead ||
				fileOperation == FileOperation::openWrite ||
				fileOperation == FileOperation::openAppend)
			{
				fileHandle = transfer.ReadOpenFileResult(fileOffset);
				fileSuccess = (fileHandle != noFileHandle);
				fileOperation = FileOperation::none;
				if (fileSuccess)
				{
					numOpenFiles++;
				}
				fileSemaphore.Give();
			}
			break;

		// Result of a file read request
		case SbcRequest::FileReadResult:
			if (fileOperation == FileOperation::read)
			{
				int bytesRead = transfer.ReadFileData(fileReadBuffer, fileBufferLength);
				fileSuccess = bytesRead >= 0;
				fileOffset = fileSuccess ? bytesRead : 0;
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;

		// Result of a file write request
		case SbcRequest::FileWriteResult:
			if (fileOperation == FileOperation::write)
			{
				fileSuccess = transfer.ReadBoolean();
				if (!fileSuccess || fileBufferLength == 0)
				{
					fileOperationPending = false;
					fileOperation = FileOperation::none;
					fileSemaphore.Give();
				}
			}
			break;

		// Result of a file seek request
		case SbcRequest::FileSeekResult:
			if (fileOperation == FileOperation::seek)
			{
				fileSuccess = transfer.ReadBoolean();
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;

		// Result of a file seek request
		case SbcRequest::FileTruncateResult:
			if (fileOperation == FileOperation::truncate)
			{
				fileSuccess = transfer.ReadBoolean();
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;

		// Invalid request
		default:
#ifdef DEBUG
			// Report this error only in debug builds. We may get here when the SBC sends a file response but the connection was reset
			REPORT_INTERNAL_ERROR;
#endif
			break;
		}

		// Request the packet again if no response could be sent back
		if (!packetAcknowledged)
		{
			transfer.ResendPacket(packet);
		}
	}

	// Check if we can wait a short moment to reduce CPU load on the SBC
	if (!skipNextDelay && numEvents < numMaxEvents &&
		!fileOperationPending && fileOperation == FileOperation::none)
	{
		delaying = true;
		if (!TaskBase::TakeIndexed(NotifyIndices::SbcInterface, (numOpenFiles != 0) ? maxFileOpenDelay : maxDelayBetweenTransfers))
		{
			delaying = false;
		}
	}
	numEvents = 0;
	skipNextDelay = false;

	// Send code replies and generic messages
	if (!gcodeReply.IsEmpty())
	{
		MutexLocker lock(gcodeReplyMutex);
		while (!gcodeReply.IsEmpty())
		{
			const MessageType type = gcodeReply.GetFirstItemType();
			OutputBuffer *buffer = gcodeReply.GetFirstItem();			// this may be null
			if (!transfer.WriteCodeReply(type, buffer))					// this handles the null case too
			{
				break;
			}
			gcodeReply.SetFirstItem(buffer);							// this does a pop if buffer is null
		}
	}

	// Notify DSF about the available buffer space
	DefragmentBufferedCodes();
	{
		TaskCriticalSectionLocker locker;
		if (!codeBufferAvailable || sendBufferUpdate)
		{
			const uint16_t bufferSpace = (txEnd == 0) ? max<uint16_t>(rxPointer, SpiCodeBufferSize - txPointer) : rxPointer - txPointer;
			sendBufferUpdate = !transfer.WriteCodeBufferUpdate(bufferSpace);
		}
	}

	// Perform the next file operation if requested
	if (fileOperationPending.load(std::memory_order_acquire))
	{
		switch (fileOperation)
		{
		case FileOperation::checkFileExists:
			fileOperationPending = !transfer.WriteCheckFileExists(filePath);
			break;

		case FileOperation::deleteFileOrDirectory:
			fileOperationPending = !transfer.WriteDeleteFileOrDirectory(filePath);
			break;
		case FileOperation::deleteFileOrDirectoryRecursively:
			fileOperationPending = !transfer.WriteDeleteFileOrDirectory(filePath, true);
			break;

		case FileOperation::openRead:
		case FileOperation::openWrite:
		case FileOperation::openAppend:
			fileOperationPending = !transfer.WriteOpenFile(filePath, fileOperation == FileOperation::openWrite || fileOperation == FileOperation::openAppend, fileOperation == FileOperation::openAppend, filePreAllocSize);
			break;

		case FileOperation::read:
			fileOperationPending = !transfer.WriteReadFile(fileHandle, fileBufferLength);
			break;

		case FileOperation::write:
		{
			const size_t bytesNotWritten = fileBufferLength;
			if (transfer.WriteFileData(fileHandle, fileWriteBuffer, fileBufferLength))
			{
				fileWriteBuffer += bytesNotWritten - fileBufferLength;
				if (fileBufferLength == 0)
				{
					fileOperationPending = false;
				}
			}
			break;
		}

		case FileOperation::seek:
			fileOperationPending = !transfer.WriteSeekFile(fileHandle, fileOffset);
			break;

		case FileOperation::truncate:
			fileOperationPending = !transfer.WriteTruncateFile(fileHandle);
			break;

		case FileOperation::close:
			fileOperationPending = !transfer.WriteCloseFile(fileHandle);
			if (!fileOperationPending)
			{
				// Close requests don't get a result back, so they can be resolved as soon as they are sent to the SBC
				fileOperation = FileOperation::none;
				numOpenFiles--;
				fileSemaphore.Give();
			}
			break;

		default:
			fileOperationPending = false;
			REPORT_INTERNAL_ERROR;
			break;
		}
	}

	// Deal with code channel requests
	for (size_t i = 0; i < NumGCodeChannels; i++)
	{
		const GCodeChannel channel(i);
		GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
		if (gb == nullptr)
		{
			// Skip GBs that are not available due to the build configuration
			continue;
		}

		// Invalidate buffered codes if required
		if (gb->IsInvalidated())
		{
			InvalidateBufferedCodes(gb->GetChannel());
			gb->Invalidate(false);
		}

		// Deal with macro files being closed
		if (gb->IsMacroFileClosed() && transfer.WriteMacroFileClosed(channel))
		{
			// Note this is only sent when a macro file has finished successfully
			gb->MacroFileClosedSent();
		}

		// Handle blocking macro requests
		if (gb->IsWaitingForMacro() && gb->IsMacroRequestPending())
		{
			const char * const requestedMacroFile = gb->GetRequestedMacroFile();
			bool fromCode = gb->IsMacroStartedByCode();
			if (transfer.WriteMacroRequest(channel, requestedMacroFile, fromCode))
			{
				if (reprap.Debug(Module::SbcInterface))
				{
					debugPrintf("Requesting macro file '%s' (fromCode: %s)\n", requestedMacroFile, fromCode ? "true" : "false");
				}
				gb->MacroRequestSent();
				gb->Invalidate();
			}
		}

		// Deal with other requests unless we are still waiting in a semaphore
		if (!gb->IsWaitingForMacro())
		{
			MutexLocker gbLock(gb->mutex, SbcYieldTimeout);
			if (gbLock.IsAcquired())
			{
				if (gb->GetChannel() != GCodeChannel::Daemon)
				{
					skipNextDelay |= gb->IsMacroRequestPending() || gb->HasJustStartedMacro();
				}

				// Handle file abort requests
				if (gb->IsAbortRequested() && transfer.WriteAbortFileRequest(channel, gb->IsAbortAllRequested()))
				{
#ifdef TRACK_FILE_CODES
					if (gb->IsFileChannel())
					{
						if (gb->IsAbortAllRequested())
						{
							fileCodesRead = fileCodesHandled = fileMacrosRunning = fileMacrosClosing = 0;
						}
						else
						{
							fileMacrosClosing++;
						}
					}
#endif
					// Send back a final code reply for codes that started the last macros, else the corresponding code will never finish
					if (!gb->IsAbortAllRequested() && gb->GetState() == GCodeState::normal && (!gb->LatestMachineState().lastCodeFromSbc || gb->LatestMachineState().macroStartedByCode))
					{
						OutputBuffer *dummy = nullptr;
						if (!transfer.WriteCodeReply(gb->GetResponseMessageType(), dummy))
						{
							// Cannot send an empty code reply now, do it later
							HandleGCodeReply(gb->GetResponseMessageType(), dummy);
						}
					}
					gb->FileAbortSent();
					gb->Invalidate();
				}

				// Handle blocking messages and their results
				if (gb->LatestMachineState().waitingForAcknowledgement && gb->IsMessagePromptPending() &&
					transfer.WriteWaitForAcknowledgement(channel))
				{
					gb->MessagePromptSent();
					gb->Invalidate();
				}
				else if (gb->IsMessageAcknowledged() && transfer.WriteMessageAcknowledged(channel))
				{
					// Note this is only sent when a message was acknowledged in a regular way (i.e. by M292)
					gb->MessageAcknowledgementSent();
				}

				// Handle non-blocking macro requests (e.g. daemon.g)
				if (gb->IsMacroRequestPending())
				{
					const char * const requestedMacroFile = gb->GetRequestedMacroFile();
					bool fromCode = gb->IsMacroStartedByCode();
					if (transfer.WriteMacroRequest(channel, requestedMacroFile, fromCode))
					{
						if (reprap.Debug(Module::SbcInterface))
						{
							debugPrintf("Requesting non-blocking macro file '%s' (fromCode: %s)\n", requestedMacroFile, fromCode ? "true" : "false");
						}
						gb->MacroRequestSent();
						gb->Invalidate();
					}
				}

				// Send pending firmware codes
				if (gb->IsSendRequested() && transfer.WriteDoCode(channel, gb->DataStart(), gb->DataLength()))
				{
					gb->SetFinished(true);
				}
			}
		}
	}

	// Send pause notification on demand
	if (reportPause && transfer.WritePrintPaused(pauseFilePosition, pauseReason))
	{
		reportPause = false;
	}
}

[[noreturn]] void SbcInterface::ReceiveAndStartIap(const char *iapChunk, size_t length) noexcept
{
	char *iapWritePointer = reinterpret_cast<char *>(IAP_IMAGE_START);
	for(;;)
	{
		// Write the next IAP chunk
		if (iapChunk != nullptr)
		{
			uint32_t *dst = reinterpret_cast<uint32_t *>(iapWritePointer);
			const uint32_t *src = reinterpret_cast<const uint32_t *>(iapChunk);
			memcpyu32(dst, src, length / sizeof(uint32_t));
			iapWritePointer += length;
			iapChunk = nullptr;
		}

		// Get the next IAP chunk
		transfer.StartNextTransfer();
		bool transferComplete = false;
		do
		{
			switch (transfer.DoTransfer())
			{
#if SAME5x
			case TransferState::connectionTimeout:
#endif
			case TransferState::connectionReset:
				// Perform a firmware reset, we're in an unsafe state to resume regular operation
				SoftwareReset(SoftwareResetReason::user);
				break;
			case TransferState::finished:
				transferComplete = true;
				break;
			default:
				// do nothing
				break;
			}
		}
		while (!transferComplete);

		// Process only IAP-related packets
		for (size_t i = 0; i < transfer.PacketsToRead(); i++)
		{
			const PacketHeader * const packet = transfer.ReadPacket();
			switch ((SbcRequest)packet->request)
			{
			case SbcRequest::WriteIap:	// Write another IAP chunk. It's always bound on a 4-byte boundary
			{
				iapChunk = transfer.ReadData(packet->length);
				length = packet->length;
				break;
			}
			case SbcRequest::StartIap:	// Start the IAP binary
				reprap.StartIap(nullptr);
				break;
			default:						// Other packet types are not supported while IAP is being written
				// do nothing
				break;
			}
		}
	}
}

void SbcInterface::InvalidateResources() noexcept
{
	rxPointer = txPointer = txEnd = 0;
	sendBufferUpdate = true;

	if (fileOperation != FileOperation::none)
	{
		fileOperationPending = false;
		fileOperation = FileOperation::none;
		fileSemaphore.Give();
	}
	MassStorage::InvalidateAllFiles();
	numOpenFiles = 0;

	// Don't cache any messages if they cannot be sent
	{
		MutexLocker lock(gcodeReplyMutex);
		gcodeReply.ReleaseAll();
	}

	// Close all open G-code files
	for (size_t i = 0; i < NumGCodeChannels; i++)
	{
		GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel(i));
		if (gb == nullptr)
		{
			// Skip GBs that are not available due to the build configuration
			break;
		}

		if (gb->IsWaitingForMacro())
		{
			gb->ResolveMacroRequest(true, false);
		}

		MutexLocker locker(gb->mutex);
		if (gb->IsMacroRequestPending())
		{
			gb->MacroRequestSent();
		}
		gb->AbortFile(true);
		gb->FileAbortSent();	// don't notify the SBC
		gb->MessageAcknowledged(true, 0, ExpressionValue());
	}

	// Abort the print (if applicable)
	printAborted = true;

	// Turn off all the heaters
	reprap.GetHeat().SwitchOffAll(true);
}

void SbcInterface::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().Message(mtype, "=== SBC interface ===\n");
	transfer.Diagnostics(mtype);
	reprap.GetPlatform().MessageF(mtype, "State: %d, disconnects: %" PRIu32 ", timeouts: %" PRIu32 " total, %" PRIu32 " by SBC, IAP RAM available 0x%05" PRIx32 "\n", (int)state, numDisconnects, numTimeouts, numSbcTimeouts, iapRamAvailable);
	reprap.GetPlatform().MessageF(mtype, "Buffer RX/TX: %d/%d-%d, open files: %u\n", (int)rxPointer, (int)txPointer, (int)txEnd, numOpenFiles);
#ifdef TRACK_FILE_CODES
	reprap.GetPlatform().MessageF(mtype, "File codes read/handled: %d/%d, file macros open/closing: %d %d\n", (int)fileCodesRead, (int)fileCodesHandled, (int)fileMacrosRunning, (int)fileMacrosClosing);
#endif
}

GCodeResult SbcInterface::HandleM576(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	bool seen = false;

	if (gb.Seen('S'))
	{
		uint32_t sParam = gb.GetUIValue();
		if (sParam > SpiConnectionTimeout)
		{
			reply.printf("SPI transfer delay must not exceed %" PRIu32 "ms", SpiConnectionTimeout);
			return GCodeResult::error;
		}
		maxDelayBetweenTransfers = sParam;
		seen = true;
	}

	if (gb.Seen('F'))
	{
		uint32_t fParam = gb.GetUIValue();
		if (fParam > SpiConnectionTimeout)
		{
			reply.printf("SPI transfer delay must not exceed %" PRIu32 "ms", SpiConnectionTimeout);
			return GCodeResult::error;
		}
		maxFileOpenDelay = fParam;
		seen = true;
	}

	if (gb.Seen('P'))
	{
		numMaxEvents = gb.GetUIValue();
		seen = true;
	}

	if (!seen)
	{
		reply.printf("Max delay between full SBC transfers %" PRIu32 "ms (%" PRIu32 "ms during file IO), max number of events before a delay is skipped: %" PRIu32, maxDelayBetweenTransfers, maxFileOpenDelay, numMaxEvents);
	}
	return GCodeResult::ok;
}

bool SbcInterface::FillBuffer(GCodeBuffer &gb) noexcept
{
	if (gb.IsInvalidated() || gb.IsMacroFileClosed() || gb.IsMessageAcknowledged() ||
		gb.IsAbortRequested() || (reportPause && gb.IsFileChannel()) ||
		(gb.LatestMachineState().waitingForAcknowledgement && gb.IsMessagePromptPending()))
	{
		// Don't process codes that are supposed to be suspended...
		return false;
	}

	bool gotCommand = false;
	{
		//TODO can we take the lock inside the loop body instead, if we re-read readPointer and writePointer after taking it?
		TaskCriticalSectionLocker locker;
		if (rxPointer != txPointer || txEnd != 0)
		{
			bool updateRxPointer = true;
			uint16_t readPointer = rxPointer;
			do
			{
				BufferedCodeHeader *bufHeader = reinterpret_cast<BufferedCodeHeader*>(codeBuffer + readPointer);
				readPointer += sizeof(BufferedCodeHeader);
				const CodeHeader *codeHeader = reinterpret_cast<const CodeHeader*>(codeBuffer + readPointer);
				readPointer += bufHeader->length;

				RRF_ASSERT(bufHeader->length > 0);
				RRF_ASSERT(readPointer <= SpiCodeBufferSize);

				if (bufHeader->isPending)
				{
					if (gb.GetChannel().RawValue() == codeHeader->channel)
					{
#ifdef TRACK_FILE_CODES
						if (gb.IsFileChannel() && gb.GetCommandLetter() != 'Q')
						{
							fileMacrosRunning -= fileMacrosClosing;
							fileMacrosClosing = 0;
							if (fileCodesRead > fileCodesHandled + fileMacrosRunning)
							{
								// Note that we cannot use MessageF here because the task scheduler is suspended
								OutputBuffer *buf;
								if (OutputBuffer::Allocate(buf))
								{
									String<StringLength100> codeString;
									gb.PrintCommand(codeString.GetRef());
									buf->printf("Code %s did not return a code result, delta %d, running macros %d\n", codeString.c_str(), fileCodesRead - fileCodesHandled - fileMacrosRunning, fileMacrosRunning);
									gcodeReply.Push(buf, WarningMessage);
								}
								fileCodesRead = fileCodesHandled - fileMacrosRunning;
							}
							fileCodesRead++;
						}
#endif

						// Process the next binary G-code
						gb.PutBinary(reinterpret_cast<const uint32_t *>(codeHeader), bufHeader->length / sizeof(uint32_t));
						bufHeader->isPending = false;

						// Check if we can reset the ring buffer pointers
						if (updateRxPointer)
						{
							sendBufferUpdate = true;
							if (readPointer == txPointer && txEnd == 0)
							{
								// Buffer completely read, reset RX/TX pointers
								rxPointer = txPointer = 0;
							}
							else if (readPointer == txEnd)
							{
								// Read last code before overlapping, restart from the beginning
								rxPointer = txEnd = 0;
							}
							else
							{
								// Code has been read, move on to the next one
								rxPointer = readPointer;
							}
						}

						gotCommand = true;
						break;
					}
					updateRxPointer = false;
				}

				if (readPointer == txEnd)
				{
					if (updateRxPointer)
					{
						// Skipped non-pending codes, restart from the beginning
						rxPointer = txEnd = 0;
						sendBufferUpdate = true;
					}

					// About to overlap, continue from the start
					readPointer = 0;
				}
			} while (readPointer != txPointer);
		}
	}

	if (gotCommand)
	{
		gb.DecodeCommand();
		return true;
	}
	return false;
}

bool SbcInterface::FileExists(const char *filename) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return false;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	filePath = filename;
	if (!DoFileOperation(FileOperation::checkFileExists))
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Timeout while trying to check if file %s exists\n", filename);
		return false;
	}

	// Return the result
	return fileSuccess;
}

bool SbcInterface::DeleteFileOrDirectory(const char *fileOrDirectory, bool recursive) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return false;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	filePath = fileOrDirectory;
	if (!DoFileOperation(recursive ? FileOperation::deleteFileOrDirectoryRecursively : FileOperation::deleteFileOrDirectory))
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Timeout while trying to delete %s\n", fileOrDirectory);
		return false;
	}

	// Return the result
	return fileSuccess;
}

FileHandle SbcInterface::OpenFile(const char *filename, OpenMode mode, FilePosition& fileLength, uint32_t preAllocSize) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return false;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	filePath = filename;
	filePreAllocSize = preAllocSize;
	FileOperation op;
	switch (mode)
	{
	case OpenMode::read:
		op = FileOperation::openRead;
		break;

	case OpenMode::write:
	case OpenMode::writeWithCrc:
		op = FileOperation::openWrite;
		break;

	case OpenMode::append:
		op = FileOperation::openAppend;
		break;

	default:
		filePath = nullptr;
		REPORT_INTERNAL_ERROR;
		return noFileHandle;
	}

	if (!DoFileOperation(op))
	{
		fileLength = 0;
		reprap.GetPlatform().MessageF(ErrorMessage, "Timeout while trying to open file %s\n", filename);
		return noFileHandle;
	}

	// Update the file length and return the handle
	fileLength = fileOffset;
	return fileHandle;
}

int SbcInterface::ReadFile(FileHandle handle, char *buffer, size_t bufferLength) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return false;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	fileHandle = handle;
	fileReadBuffer = buffer;
	fileBufferLength = bufferLength;
	if (!DoFileOperation(FileOperation::read))
	{
		reprap.GetPlatform().Message(ErrorMessage, "Timeout while trying to read from file\n");
		return -1;
	}

	// Return the number of bytes read
	return fileSuccess ? (int)fileOffset : -1;
}

bool SbcInterface::WriteFile(FileHandle handle, const char *buffer, size_t bufferLength) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return false;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	fileHandle = handle;
	fileWriteBuffer = buffer;
	fileBufferLength = bufferLength;
	if (!DoFileOperation(FileOperation::write))
	{
		reprap.GetPlatform().Message(ErrorMessage, "Timeout while trying to write to file\n");
		return false;
	}

	// Return the result
	return fileSuccess;
}

bool SbcInterface::SeekFile(FileHandle handle, FilePosition offset) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return false;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	fileHandle = handle;
	fileOffset = offset;
	if (!DoFileOperation(FileOperation::seek))
	{
		reprap.GetPlatform().Message(ErrorMessage, "Timeout while trying to seek in file\n");
		return false;
	}

	// Return the result
	return fileSuccess;
}

bool SbcInterface::TruncateFile(FileHandle handle) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return false;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	fileHandle = handle;
	if (!DoFileOperation(FileOperation::truncate))
	{
		reprap.GetPlatform().Message(ErrorMessage, "Timeout while trying to truncate file\n");
		return false;
	}

	// Return the result
	return fileSuccess;
}

void SbcInterface::CloseFile(FileHandle handle) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);
	fileHandle = handle;

	if (!DoFileOperation(FileOperation::close))
	{
		reprap.GetPlatform().Message(ErrorMessage, "Timeout while trying to close file\n");
	}
}

// Ask the SBC task to do a file operation
// Return true if the SBC task gave us a response, false if we timed out waiting for it
// Caller must own fileMutex and set up the appropriate parameters before calling this
bool SbcInterface::DoFileOperation(FileOperation f) noexcept
{
	fileOperation = f;
	TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::SbcInterface);
	fileOperationPending.store(true, std::memory_order_release);

	// Let the SBC task process this request as quickly as possible
	const bool isDelaying = delaying.exchange(false);
	if (isDelaying)
	{
		sbcTask->Give(NotifyIndices::SbcInterface);
	}

	const bool rslt = fileSemaphore.Take(SpiMaxRequestTime);
	if (!rslt)
	{
		fileOperation = FileOperation::none;
		fileOperationPending.store(false, std::memory_order_release);
	}
	return rslt;
}

void SbcInterface::HandleGCodeReply(MessageType mt, const char *reply) noexcept
{
	if (!IsConnected())
	{
		return;
	}

#ifdef TRACK_FILE_CODES
	if ((mt & ((1u << GCodeChannel::File) | (1u << GCodeChannel::File2))) != 0)
	{
		fileCodesHandled++;
	}
#endif

	MutexLocker lock(gcodeReplyMutex);
	OutputBuffer *buffer = gcodeReply.GetLastItem();
	if (buffer != nullptr && mt == gcodeReply.GetLastItemType() && (mt & PushFlag) != 0 && !buffer->IsReferenced())
	{
		// Try to save some space by combining segments that have the Push flag set
		buffer->cat(reply);
	}
	else if (reply[0] != 0 && OutputBuffer::Allocate(buffer))
	{
		// Attempt to allocate one G-code buffer per non-empty output message
		buffer->cat(reply);
		gcodeReply.Push(buffer, mt);
	}
	else
	{
		// Store nullptr to indicate an empty response. This way many OutputBuffer references can be saved
		gcodeReply.Push(nullptr, mt);
	}
	EventOccurred();
}

void SbcInterface::HandleGCodeReply(MessageType mt, OutputBuffer *buffer) noexcept
{
	if (!IsConnected())
	{
		OutputBuffer::ReleaseAll(buffer);
		return;
	}

#ifdef TRACK_FILE_CODES
	if ((mt & ((1u << GCodeChannel::File) | (1u << GCodeChannel::File2))) != 0)
	{
		fileCodesHandled++;
	}
#endif

	MutexLocker lock(gcodeReplyMutex);
	gcodeReply.Push(buffer, mt);
	EventOccurred();
}

void SbcInterface::EventOccurred(bool timeCritical) noexcept
{
	if (!IsConnected())
	{
		return;
	}

	// Increment the number of events
	if (timeCritical)
	{
		numEvents = numMaxEvents;
	}
	else
	{
		numEvents++;
	}

	// Stop delaying if the next transfer is time-critical
	if (numEvents >= numMaxEvents)
	{
		const bool isDelaying = delaying.exchange(false);
		if (isDelaying)
		{
			sbcTask->Give(NotifyIndices::SbcInterface);
		}
	}
}

void SbcInterface::DefragmentBufferedCodes() noexcept
{
	TaskCriticalSectionLocker locker;
	if (rxPointer != txPointer || txEnd != 0)
	{
		const uint16_t bufferSpace = (txEnd == 0) ? max<uint16_t>(rxPointer, SpiCodeBufferSize - txPointer) : rxPointer - txPointer;
		if (bufferSpace > MaxCodeBufferSize)
		{
			// There is still enough space left for at least one more code, don't worry about fragmentation yet
			return;
		}

		if (txEnd == 0)
		{
			// Ring buffer data is sequential (rxPointer..txPointer, txEnd=0)
			(void)DefragmentCodeBlock(rxPointer, txPointer);
		}
		else
		{
			// Ring buffer overlapped (rxPointer..txEnd, 0..txPointer)
			if (!DefragmentCodeBlock(rxPointer, txEnd) &&
				!DefragmentCodeBlock(0, txPointer) &&
				SpiCodeBufferSize - (size_t)txEnd > MaxCodeBufferSize)
			{
				size_t endBufferSize = txEnd - rxPointer;
				memmoveu32(reinterpret_cast<uint32_t*>(codeBuffer + SpiCodeBufferSize - endBufferSize), reinterpret_cast<uint32_t*>(codeBuffer + rxPointer), endBufferSize / sizeof(uint32_t));
				rxPointer = SpiCodeBufferSize - endBufferSize;
				txEnd = SpiCodeBufferSize;
				sendBufferUpdate = true;
			}
		}
	}
}

// Defragment a specific block of the code buffer and update the end of it
bool SbcInterface::DefragmentCodeBlock(uint16_t start, volatile uint16_t &end) noexcept
{
	char *gapStart = nullptr;
	for (uint16_t readPointer = start; readPointer != end;)
	{
		BufferedCodeHeader *bufHeader = reinterpret_cast<BufferedCodeHeader *>(codeBuffer + readPointer);
		size_t bufSize = sizeof(BufferedCodeHeader) + bufHeader->length;
		readPointer += bufSize;

		if (bufHeader->isPending)
		{
			if (gapStart != nullptr)
			{
				size_t gapSize = reinterpret_cast<const char *>(bufHeader) - gapStart;
				if (gapSize >= bufSize)
				{
					// Gap size is big enough to accommodate the next code
					memcpyu32(reinterpret_cast<uint32_t*>(gapStart), reinterpret_cast<uint32_t *>(bufHeader), bufSize / sizeof(uint32_t));		// requires incrementing copy order
					gapStart += bufSize;
				}
				else
				{
					// Gap size is too small. Move the remaining buffer but only once per run
					memcpyu32(reinterpret_cast<uint32_t*>(gapStart), reinterpret_cast<uint32_t *>(bufHeader), (codeBuffer + end - gapStart) / sizeof(uint32_t));
					readPointer = (uint16_t)(gapStart - codeBuffer + bufSize);
					gapStart = nullptr;
					end -= gapSize;
					sendBufferUpdate = true;
					return true;
				}
			}
		}
		else if (gapStart == nullptr)
		{
			gapStart = reinterpret_cast<char *>(bufHeader);
		}
	}

	if (gapStart != nullptr)
	{
		end = (uint16_t)(gapStart - codeBuffer);
		sendBufferUpdate = true;
		return true;
	}
	return false;
}

void SbcInterface::InvalidateBufferedCodes(GCodeChannel channel) noexcept
{
	TaskCriticalSectionLocker locker;
	if (rxPointer != txPointer || txEnd != 0)
	{
		bool updateRxPointer = true;
		uint16_t readPointer = rxPointer;
		do
		{
			BufferedCodeHeader *bufHeader = reinterpret_cast<BufferedCodeHeader *>(codeBuffer + readPointer);
			if (bufHeader->isPending)
			{
				const CodeHeader *codeHeader = reinterpret_cast<const CodeHeader*>(codeBuffer + readPointer + sizeof(BufferedCodeHeader));
				if (codeHeader->channel == channel.RawValue())
				{
					bufHeader->isPending = false;
				}
				else
				{
					updateRxPointer = false;
				}
			}
			readPointer += sizeof(BufferedCodeHeader) + bufHeader->length;

			if (updateRxPointer)
			{
				sendBufferUpdate = true;
				if (readPointer == txPointer && txEnd == 0)
				{
					// Buffer is empty again, reset the pointers
					rxPointer = txPointer = 0;
					break;
				}
				else if (readPointer == txEnd)
				{
					// Invalidated last code before overlapping, continue from the beginning
					readPointer = 0;
					rxPointer = txEnd = 0;
				}
				else
				{
					// Invalidated next code
					rxPointer = readPointer;
				}
			}
			else if (readPointer == txEnd)
			{
				// About to overlap, continue from the start
				readPointer = 0;
			}
		} while (readPointer != txPointer);
	}
}

#endif
