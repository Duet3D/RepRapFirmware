/*
 * LinuxInterface.cpp
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */

#include "LinuxInterface.h"
#include "DataTransfer.h"

#if HAS_LINUX_INTERFACE

#include "GCodes/GCodeBuffer/ExpressionParser.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Heating/Heat.h"
#include "Movement/Move.h"
#include "Platform.h"
#include "PrintMonitor.h"
#include "Tools/Filament.h"
#include "RepRap.h"
#include "RepRapFirmware.h"
#include <Tasks.h>
#include <Hardware/SoftwareReset.h>
#include <Hardware/ExceptionHandlers.h>
#include <Cache.h>
#include <TaskPriorities.h>

extern char _estack;		// defined by the linker

volatile OutputStack LinuxInterface::gcodeReply;
Mutex LinuxInterface::gcodeReplyMutex;

#ifdef __LPC17xx__
constexpr size_t LinuxTaskStackWords = 375;
#elif defined(DEBUG)
constexpr size_t LinuxTaskStackWords = 1000;			// needs to be enough to support rr_model
#else
constexpr size_t LinuxTaskStackWords = 600;				// needs to be enough to support rr_model
#endif

static Task<LinuxTaskStackWords> *linuxTask;

extern "C" [[noreturn]] void LinuxTaskStart(void * pvParameters) noexcept
{
	reprap.GetLinuxInterface().TaskLoop();
}

LinuxInterface::LinuxInterface() noexcept : isConnected(false), numDisconnects(0),
	reportPause(false), reportPauseWritten(false), printStarted(false), printStopped(false),
	rxPointer(0), txPointer(0), txLength(0), sendBufferUpdate(true),
	iapWritePointer(IAP_IMAGE_START), waitingForFileChunk(false)
#ifdef TRACK_FILE_CODES
	, fileCodesRead(0), fileCodesHandled(0), fileMacrosRunning(0), fileMacrosClosing(0)
#endif
{
}

void LinuxInterface::Init() noexcept
{
	gcodeReplyMutex.Create("LinuxReply");

#if defined(DUET_NG)
	// Make sure that the Wifi module if present is disabled. The ESP Reset pin is already forced low in Platform::Init();
	pinMode(EspEnablePin, OUTPUT_LOW);
#endif

	transfer.Init();
	linuxTask = new Task<LinuxTaskStackWords>;
	linuxTask->Create(LinuxTaskStart, "Linux", nullptr, TaskPriority::SpinPriority);
	transfer.SetLinuxTask(linuxTask);
	transfer.StartNextTransfer();
	iapRamAvailable = &_estack - Tasks::GetHeapTop();
}

[[noreturn]] void LinuxInterface::TaskLoop() noexcept
{
	bool writingIap = false;
	for (;;)
	{
		if (transfer.IsReady())
		{
			// Check if the connection state has changed
			if (!isConnected && !writingIap)
			{
				isConnected = true;
				reprap.GetPlatform().Message(NetworkInfoMessage, "Connection to Linux established!\n");
			}

			// Process incoming packets
			for (size_t i = 0; i < transfer.PacketsToRead(); i++)
			{
				const PacketHeader * const packet = transfer.ReadPacket();
				if (packet == nullptr)
				{
					if (reprap.Debug(moduleLinuxInterface))
					{
						debugPrintf("Error trying to read next SPI packet\n");
					}
					break;
				}

				if (packet->request >= (uint16_t)LinuxRequest::InvalidRequest)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				bool packetAcknowledged = true;
				switch ((LinuxRequest)packet->request)
				{
				// Perform an emergency stop
				case LinuxRequest::EmergencyStop:
					reprap.EmergencyStop();
					break;

				// Reset the controller
				case LinuxRequest::Reset:
					SoftwareReset(SoftwareResetReason::user);
					break;

				// Perform a G/M/T-code
				case LinuxRequest::Code:
				{
					// Read the next code
					const CodeHeader *code = reinterpret_cast<const CodeHeader*>(transfer.ReadData(packet->length));
					const GCodeChannel channel(code->channel);
					GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
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
						if (channel == GCodeChannel::File)
						{
							fileMacrosRunning++;
						}
#endif
					}

					// Check if the next code overlaps. If yes, restart from the beginning
					TaskCriticalSectionLocker locker;
					if (txPointer + sizeof(BufferedCodeHeader) + packet->length > SpiCodeBufferSize)
					{
						if (rxPointer == txPointer)
						{
							rxPointer = 0;
						}
						else
						{
							// Check to see if we will overwrite existing buffer contents.
							if (rxPointer < sizeof(BufferedCodeHeader) + packet->length)
							{
								// debugPrintf("Buffer overwrite txPointer %d rxPointer %d packet len %d\n", txPointer, rxPointer, sizeof(BufferedCodeHeader) + packet->length);
								// reject this packet so it will be resent later
								packetAcknowledged = false;
								break;
							}
						}

						txLength = txPointer;
						txPointer = 0;
						sendBufferUpdate = true;
					}

					// Check to see if we are about to overwrite older packets.
					if (txPointer < rxPointer && txPointer + sizeof(BufferedCodeHeader) + packet->length > rxPointer)
					{
						// debugPrintf("Buffer overwrite txPointer %d rxPointer %d packet len %d\n", txPointer, rxPointer, sizeof(BufferedCodeHeader) + packet->length);
						// reject this packet so it will be resent later
						packetAcknowledged = false;
						break;
					}

					// Store the buffer header
					BufferedCodeHeader *bufHeader = reinterpret_cast<BufferedCodeHeader*>(codeBuffer + txPointer);
					bufHeader->isPending = true;
					bufHeader->length = packet->length;
					txPointer += sizeof(BufferedCodeHeader);

					// Store the corresponding code. Binary codes are always aligned on a 4-byte boundary
					uint32_t *dst = reinterpret_cast<uint32_t *>(codeBuffer + txPointer);
					const uint32_t *src = reinterpret_cast<const uint32_t *>(code);
					memcpyu32(dst, src, packet->length / sizeof(uint32_t));
					txPointer += packet->length;
					break;
				}

				// Get the object model
				case LinuxRequest::GetObjectModel:
				{
					String<StringLength100> key;
					StringRef keyRef = key.GetRef();
					String<StringLength20> flags;
					StringRef flagsRef = flags.GetRef();
					transfer.ReadGetObjectModel(packet->length, keyRef, flagsRef);

					try
					{
						OutputBuffer *outBuf = reprap.GetModelResponse(key.c_str(), flags.c_str());
						if (outBuf == nullptr || !transfer.WriteObjectModel(outBuf))
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

				// Set value in the object model
				case LinuxRequest::SetObjectModel:
				{
					const size_t dataLength = packet->length;
					const char * const data = transfer.ReadData(dataLength);
					// TODO implement this
					(void)data;
					break;
				}

				// Print has been started, set file print info
				case LinuxRequest::PrintStarted:
				{
					String<MaxFilenameLength> filename;
					StringRef filenameRef = filename.GetRef();
					transfer.ReadPrintStartedInfo(packet->length, filenameRef, fileInfo);
					reprap.GetPrintMonitor().SetPrintingFileInfo(filename.c_str(), fileInfo);
					printStarted = true;
					break;
				}

				// Print has been stopped
				case LinuxRequest::PrintStopped:
				{
					const PrintStoppedReason reason = transfer.ReadPrintStoppedInfo();
					if (reason == PrintStoppedReason::normalCompletion)
					{
						// Just mark the print file as finished
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::File);
						MutexLocker locker(gb->mutex, 10);
						if (locker)
						{
							gb->SetPrintFinished();
						}
						else
						{
							packetAcknowledged = false;
						}
					}
					else
					{
						// Stop the print with the given reason
						printStopReason = (StopPrintReason)reason;
						printStopped = true;
						InvalidateBufferChannel(GCodeChannel::File);
					}
					break;
				}

				// Macro file has been finished
				case LinuxRequest::MacroCompleted:
				{
					bool error;
					const GCodeChannel channel = transfer.ReadMacroCompleteInfo(error);
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
						if (gb->IsWaitingForMacro() && !gb->IsMacroRequestPending())
						{
							gb->ResolveMacroRequest(error, true);
							if (reprap.Debug(moduleLinuxInterface))
							{
								debugPrintf("Waiting macro completed on channel %u\n", channel.ToBaseType());
							}
						}
						else
						{
							MutexLocker locker(gb->mutex, 10);
							if (locker)
							{
								if (error)
								{
									gb->MachineState().CloseFile();
									gb->PopState(false);
									gb->Init();
								}
								else
								{
#ifdef TRACK_FILE_CODES
									if (channel == GCodeChannel::File)
									{
										fileMacrosClosing++;
									}
#endif
									gb->SetFileFinished();
								}

								if (reprap.Debug(moduleLinuxInterface))
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

				// Return heightmap as generated by G29 S0
				case LinuxRequest::GetHeightMap:
				{
					if (!reprap.GetMove().heightMapLock.IsLocked())
					{
						ReadLocker locker(reprap.GetMove().heightMapLock);
						packetAcknowledged = transfer.WriteHeightMap();
					}
					else
					{
						packetAcknowledged = false;
					}
					break;
				}

				// Set heightmap via G29 S1
				case LinuxRequest::SetHeightMap:
				{
					if (!reprap.GetMove().heightMapLock.IsLocked())
					{
						WriteLocker locker(reprap.GetMove().heightMapLock);
						if (!transfer.ReadHeightMap())
						{
							reprap.GetPlatform().Message(ErrorMessage, "Failed to set height map - bad data?\n");
						}
					}
					else
					{
						packetAcknowledged = false;
					}
					break;
				}

				// Lock movement and wait for standstill
				case LinuxRequest::LockMovementAndWaitForStandstill:
				{
					const GCodeChannel channel = transfer.ReadCodeChannel();
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
						MutexLocker locker(gb->mutex, 10);
						if (locker && reprap.GetGCodes().LockMovementAndWaitForStandstill(*gb))
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
				case LinuxRequest::Unlock:
				{
					const GCodeChannel channel = transfer.ReadCodeChannel();
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
						MutexLocker locker(gb->mutex, 10);
						if (locker)
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

				// Write another chunk of the IAP binary to the designated Flash area
				case LinuxRequest::WriteIap:
				{
					if (iapWritePointer == IAP_IMAGE_START)			// if start of IAP write
					{
						reprap.PrepareToLoadIap();
						writingIap = true;
					}

					// Copy another IAP chunk. It's always bound on a 4-byte boundary
					uint32_t *dst = reinterpret_cast<uint32_t *>(iapWritePointer);
					const uint32_t *src = reinterpret_cast<const uint32_t *>(transfer.ReadData(packet->length));
					memcpyu32(dst, src, packet->length / sizeof(uint32_t));
					iapWritePointer += packet->length;
					break;
				}

				// Launch the IAP binary
				case LinuxRequest::StartIap:
					reprap.StartIap();
					break;

				// Assign filament
				case LinuxRequest::AssignFilament:
				{
					int extruder;
					String<FilamentNameLength> filamentName;
					StringRef filamentRef = filamentName.GetRef();
					transfer.ReadAssignFilament(extruder, filamentRef);

					Filament *filament = Filament::GetFilamentByExtruder(extruder);
					if (filament != nullptr)
					{
						if (filamentName.IsEmpty())
						{
							filament->Unload();
						}
						else
						{
							filament->Load(filamentName.c_str());
						}

						TaskCriticalSectionLocker locker;
						reprap.MoveUpdated();
					}
					break;
				}

				// Return a file chunk
				case LinuxRequest::FileChunk:
					transfer.ReadFileChunk(requestedFileBuffer, requestedFileDataLength, requestedFileLength);
					requestedFileSemaphore.Give();
					break;

				// Evaluate an expression
				case LinuxRequest::EvaluateExpression:
				{
					String<StringLength100> expression;
					StringRef expressionRef = expression.GetRef();
					GCodeChannel channel = transfer.ReadEvaluateExpression(packet->length, expressionRef);
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);

						// If there is a macro file waiting, the first instruction must be conditional. Don't block any longer...
						if (gb->IsWaitingForMacro())
						{
#ifdef TRACK_FILE_CODES
							if (channel == GCodeChannel::File)
							{
								fileMacrosRunning++;
							}
#endif
							gb->ResolveMacroRequest(false, false);
						}

						try
						{
							// Evaluate the expression and send the result to DSF
							MutexLocker lock(gb->mutex, 10);
							if (lock)
							{
								ExpressionParser parser(*gb, expression.c_str(), expression.c_str() + expression.strlen());
								const ExpressionValue val = parser.Parse();
								packetAcknowledged = transfer.WriteEvaluationResult(expression.c_str(), val);
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
				case LinuxRequest::Message:
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
				case LinuxRequest::MacroStarted:
				{
					const GCodeChannel channel = transfer.ReadCodeChannel();
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
						if (gb->IsWaitingForMacro() && !gb->IsMacroRequestPending())
						{
#ifdef TRACK_FILE_CODES
							if (channel == GCodeChannel::File)
							{
								fileMacrosRunning++;
							}
#endif
							gb->ResolveMacroRequest(false, false);
						}
						else if (channel != GCodeChannel::Daemon)
						{
							reprap.GetPlatform().MessageF(WarningMessage, "Macro file has been started on channel %s but none was requested\n", channel.ToString());
						}
					}
					else
					{
						REPORT_INTERNAL_ERROR;
					}
					break;
				}

				// All the files have been aborted on the given channel
				case LinuxRequest::FilesAborted:
				{
					const GCodeChannel channel = transfer.ReadCodeChannel();
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
						MutexLocker locker(gb->mutex, 10);
						if (locker)
						{
							// Note that we do not call StopPrint here or set any other variables; DSF already does that
							gb->AbortFile(true, false);
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

				// Invalid request
				default:
					REPORT_INTERNAL_ERROR;
					break;
				}

				// Request the packet again if no response could be sent back
				if (!packetAcknowledged)
				{
					transfer.ResendPacket(packet);
				}
			}

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
			if (sendBufferUpdate || transfer.LinuxHadReset())
			{
				TaskCriticalSectionLocker locker;

				const uint16_t bufferSpace = (txLength == 0) ? max<uint16_t>(rxPointer, SpiCodeBufferSize - txPointer) : rxPointer - txPointer;
				sendBufferUpdate = !transfer.WriteCodeBufferUpdate(bufferSpace);
			}

			if (!writingIap)					// it's not safe to access GCodes once we have started writing the IAP
			{
				// Get another chunk of the file being requested
				if (waitingForFileChunk &&
					!fileChunkRequestSent && transfer.WriteFileChunkRequest(requestedFileName.c_str(), requestedFileOffset, requestedFileLength))
				{
					fileChunkRequestSent = true;
				}

				// Deal with code channel requests
				for (size_t i = 0; i < NumGCodeChannels; i++)
				{
					const GCodeChannel channel(i);
					GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);

					// Invalidate buffered codes if required
					if (gb->IsInvalidated())
					{
						InvalidateBufferChannel(gb->GetChannel());
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
							if (reprap.Debug(moduleLinuxInterface))
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
						MutexLocker gbLock(gb->mutex, 10);
						if (gbLock)
						{
							// Handle file abort requests
							if (gb->IsAbortRequested() && transfer.WriteAbortFileRequest(channel, gb->IsAbortAllRequested()))
							{
#ifdef TRACK_FILE_CODES
								if (channel == GCodeChannel::File)
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
								gb->FileAbortSent();
								gb->Invalidate();
							}

							// Handle blocking messages and their results
							if (gb->MachineState().waitingForAcknowledgement && gb->IsMessagePromptPending() &&
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
									if (reprap.Debug(moduleLinuxInterface))
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
				if (reportPause)
				{
					GCodeBuffer * const fileGCode = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::File);
					MutexLocker locker(fileGCode->mutex, 10);
					if (locker && transfer.WritePrintPaused(pauseFilePosition, pauseReason))
					{
						fileGCode->Invalidate();
						reportPause = false;
					}
				}
			}

			// Start the next transfer
			transfer.StartNextTransfer();

			// Wait for the next SPI transaction to complete or for a timeout to occur
			TaskBase::Take(SpiConnectionTimeout);
		}
		else if (isConnected && !writingIap && !transfer.IsConnected())
		{
			isConnected = false;
			numDisconnects++;
			reprap.GetPlatform().Message(NetworkInfoMessage, "Lost connection to Linux\n");

			rxPointer = txPointer = txLength = 0;
			sendBufferUpdate = true;
			iapWritePointer = IAP_IMAGE_START;

#if SUPPORT_CAN_EXPANSION
			if (!requestedFileName.IsEmpty())
			{
				requestedFileDataLength = -1;
				requestedFileSemaphore.Give();
			}
#endif

			// Don't cache any messages if they cannot be sent
			{
				MutexLocker lock(gcodeReplyMutex);
				gcodeReply.ReleaseAll();
			}

			// Close all open G-code files
			for (size_t i = 0; i < NumGCodeChannels; i++)
			{
				GCodeBuffer *gb = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel(i));
				if (gb->IsWaitingForMacro())
				{
					gb->ResolveMacroRequest(true, false);
				}

				MutexLocker locker(gb->mutex);
				if (gb->IsMacroRequestPending())
				{
					gb->MacroRequestSent();
				}
				gb->AbortFile(true, false);
				gb->MessageAcknowledged(true);
			}

			// Stop the print (if applicable)
			printStopReason = StopPrintReason::abort;
			printStopped = true;

			// Turn off all the heaters
			reprap.GetHeat().SwitchOffAll(true);

			// Wait indefinitely for the next transfer
			TaskBase::Take();
		}
		else if (!writingIap)
		{
			// A transfer is being performed but it has not finished yet
			RTOSIface::Yield();
		}
	}
}

void LinuxInterface::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().Message(mtype, "=== SBC interface ===\n");
	transfer.Diagnostics(mtype);
	reprap.GetPlatform().MessageF(mtype, "Number of disconnects: %" PRIu32 ", IAP RAM available 0x%05" PRIx32 "\n", numDisconnects, iapRamAvailable);
	reprap.GetPlatform().MessageF(mtype, "Buffer RX/TX: %d/%d-%d\n", (int)rxPointer, (int)txPointer, (int)txLength);
#ifdef TRACK_FILE_CODES
	reprap.GetPlatform().MessageF(mtype, "File codes read/handled: %d/%d, file macros open/closing: %d %d\n", (int)fileCodesRead, (int)fileCodesHandled, (int)fileMacrosRunning, (int)fileMacrosClosing);
#endif
}

bool LinuxInterface::FillBuffer(GCodeBuffer &gb) noexcept
{
	if (gb.IsInvalidated() || gb.IsMacroFileClosed() || gb.IsMessageAcknowledged() ||
		gb.IsAbortRequested() || (reportPause && gb.GetChannel() == GCodeChannel::File) ||
		(gb.MachineState().waitingForAcknowledgement && gb.IsMessagePromptPending()))
	{
		// Don't process codes that are supposed to be suspended...
		return false;
	}

	bool gotCommand = false;
	{
		//TODO can we take the lock inside the loop body instead, if we re-read readPointer and writePointer after taking it?
		TaskCriticalSectionLocker locker;
		if (rxPointer != txPointer || txLength != 0)
		{
			bool updateRxPointer = true;
			uint16_t readPointer = rxPointer;
			do
			{
				BufferedCodeHeader * const bufHeader = reinterpret_cast<BufferedCodeHeader*>(codeBuffer + readPointer);
				readPointer += sizeof(BufferedCodeHeader);
				const CodeHeader * const header = reinterpret_cast<const CodeHeader*>(codeBuffer + readPointer);
				readPointer += bufHeader->length;

				if (bufHeader->isPending)
				{
					if (gb.GetChannel().RawValue() == header->channel)
					{
#ifdef TRACK_FILE_CODES
						if (gb.GetChannel() == GCodeChannel::File && gb.GetCommandLetter() != 'Q')
						{
							fileMacrosRunning -= fileMacrosClosing;
							fileMacrosClosing = 0;
							if (fileCodesRead > fileCodesHandled + fileMacrosRunning)
							{
								// Note that we cannot use MessageF here because the task scheduler is suspended
								OutputBuffer *buf;
								if (OutputBuffer::Allocate(buf))
								{
									String<SHORT_GCODE_LENGTH> codeString;
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
						gb.PutBinary(reinterpret_cast<const uint32_t *>(header), bufHeader->length / sizeof(uint32_t));
						bufHeader->isPending = false;

						// Check if we can reset the ring buffer pointers
						if (updateRxPointer)
						{
							sendBufferUpdate = true;

							rxPointer = readPointer;
							if (rxPointer == txLength)
							{
								rxPointer = txLength = 0;
							}
							else if (rxPointer == txPointer && txLength == 0)
							{
								rxPointer = txPointer = 0;
							}
						}

						gotCommand = true;
						break;
					}
					else
					{
						updateRxPointer = false;
					}
				}

				if (readPointer == txLength)
				{
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

// Read a file chunk from the SBC. When a response has been received, the current task is woken up again.
// It changes bufferLength to the number of received bytes
// This method returns true on success and false if an error occurred (e.g. file not found)
bool LinuxInterface::GetFileChunk(const char *filename, uint32_t offset, char *buffer, uint32_t& bufferLength, uint32_t& fileLength) noexcept
{
	if (waitingForFileChunk)
	{
		reprap.GetPlatform().Message(ErrorMessage, "Trying to request a file chunk from two independent tasks\n");
		bufferLength = fileLength = 0;
		return false;
	}

	fileChunkRequestSent = false;
	requestedFileName.copy(filename);
	requestedFileLength = bufferLength;
	requestedFileOffset = offset;
	requestedFileBuffer = buffer;

	waitingForFileChunk = true;
	requestedFileSemaphore.Take();

	waitingForFileChunk = false;
	if (requestedFileDataLength < 0)
	{
		bufferLength = fileLength = 0;
		return false;
	}
	bufferLength = requestedFileDataLength;
	fileLength = requestedFileLength;
	return true;
}

void LinuxInterface::HandleGCodeReply(MessageType mt, const char *reply) noexcept
{
	if (!IsConnected())
	{
		return;
	}

#ifdef TRACK_FILE_CODES
	if ((mt & (1 << GCodeChannel::File)) != 0)
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
}

void LinuxInterface::HandleGCodeReply(MessageType mt, OutputBuffer *buffer) noexcept
{
	if (!IsConnected())
	{
		OutputBuffer::ReleaseAll(buffer);
		return;
	}

#ifdef TRACK_FILE_CODES
	if ((mt & (1 << GCodeChannel::File)) != 0)
	{
		fileCodesHandled++;
	}
#endif

	MutexLocker lock(gcodeReplyMutex);
	gcodeReply.Push(buffer, mt);
}

void LinuxInterface::InvalidateBufferChannel(GCodeChannel channel) noexcept
{
	TaskCriticalSectionLocker locker;
	if (rxPointer != txPointer || txLength != 0)
	{
		bool updateRxPointer = true;
		uint16_t readPointer = rxPointer;
		do
		{
			BufferedCodeHeader *bufHeader = reinterpret_cast<BufferedCodeHeader*>(codeBuffer + readPointer);
			readPointer += sizeof(BufferedCodeHeader);

			if (bufHeader->isPending)
			{
				const CodeHeader *header = reinterpret_cast<const CodeHeader*>(codeBuffer + readPointer);
				if (header->channel == channel.RawValue())
				{
					bufHeader->isPending = false;
				}
				else
				{
					updateRxPointer = false;
				}
			}
			readPointer += bufHeader->length;

			if (readPointer == txLength)
			{
				readPointer = 0;
			}

			if (updateRxPointer)
			{
				sendBufferUpdate = true;
				rxPointer = readPointer;
				if (rxPointer == 0)
				{
					txLength = 0;
				}
				else if (rxPointer == txPointer && txLength == 0)
				{
					rxPointer = txPointer = 0;
					break;
				}
			}
		} while (readPointer != txPointer);

		// TODO It might make sense to reorder out-of-order blocks here to create a larger chunk of free buffer space.
		// An alternative could be to limit the buffered codes per size per channel in DCS - yet we must avoid segmentation as much as possible
		// Segmentation could become a problem if a lot of codes from different channels keep coming in and one or more codes cannot be put into GB(s)
	}
}

#endif
