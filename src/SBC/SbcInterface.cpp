/*
 * SbcInterface.cpp
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */

#include "SbcInterface.h"
#include "DataTransfer.h"

#if HAS_SBC_INTERFACE

#if SUPPORTS_SBC_OVER_USB
# include <Devices.h>
#endif

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
	maxDelayBetweenTransfers(SbcTransferDelay), maxFileOpenDelay(SbcFileOpenDelay), numMaxEvents(SbcEventsRequired),
	burstModeWindow(SbcBurstModeWindow), burstModeDelay(SbcBurstModeDelay), burstModeStartTime(0),
	delaying(false), numEvents(0), reportPause(false), reportPauseWritten(false), printAborted(false),
	codeBuffer(nullptr), rxPointer(0), txPointer(0), txEnd(0), sendBufferUpdate(true),
#if SUPPORTS_SBC_OVER_USB
	pendingUsbDevice(nullptr), usbDeviceIndex(0),
#endif
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
	codeBuffer = (char *)new uint32_t[(SbcCodeBufferSize + 3)/4];
	transfer.Init();
	sbcTask = new Task<SBCTaskStackWords>();
	sbcTask->Create(SBCTaskStart, "SBC", nullptr, TaskPriority::SbcPriority);
	iapRamAvailable = (const char*)&_estack - Tasks::GetHeapTop();
}

#if SUPPORTS_SBC_OVER_USB

// Blocking write that retries until all bytes are sent or timeout
static bool ReliableUsbWrite(SerialCDC *dev, const uint8_t *data, size_t length, uint32_t timeoutMs) noexcept
{
	const uint32_t startTime = millis();
	while (length > 0)
	{
		const size_t written = dev->write(data, length);
		data += written;
		length -= written;
		if (length > 0)
		{
			if (millis() - startTime >= timeoutMs)
			{
				return false;
			}
			delay(1);
		}
	}
	dev->flush();
	return true;
}

// Send the USB SBC init response message reliably
static void SendUsbInitMessage(SerialCDC *dev) noexcept
{
	char buf[128];
	SafeSnprintf(buf, sizeof(buf),
		"Switching to binary SBC mode\n"
		"{\"protocol\":%u,\"rxBuffer\":%u,\"txBuffer\":%u}\n",
		(unsigned)SbcProtocolVersion, (unsigned)SbcTransferBufferSize, (unsigned)SbcTransferBufferSize);
	ReliableUsbWrite(dev, reinterpret_cast<const uint8_t *>(buf), strlen(buf), 2000);
}

#endif // SUPPORTS_SBC_OVER_USB

[[noreturn]] void SbcInterface::TaskLoop() noexcept
{
	transfer.InitFromTask();
	transfer.StartNextTransfer();

	bool busy = false, transferComplete = false, hadTimeout = false, hadSbcTimeout = false, hadReset = false;
	for (;;)
	{
#if SUPPORTS_SBC_OVER_USB
		// Check for pending USB transport switch (requested by GCode system after M576.1)
		if (pendingUsbDevice != nullptr)
		{
			SerialCDC *dev = pendingUsbDevice;
			pendingUsbDevice = nullptr;

			// Switch DataTransfer from SPI to USB
			transfer.SwitchToUsb(dev, usbDeviceIndex);

			// Send init message via standard CDC I/O (before direct mode)
			SendUsbInitMessage(dev);
			dev->WaitForTxEmpty(SbcTxDrainTimeout);

			// Handover: host sends first packet to complete CDC stream's pending OUT transfer
			dev->BeginDirectMode();
			continue;		// restart the task loop
		}
#endif

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
#if SUPPORTS_SBC_OVER_USB
				// When USB SBC is supported but not connected over SPI, use a short timeout so we can poll for M576.1
				if (!isConnected && transfer.GetTransportType() == SbcTransportType::spi)
				{
					hadTimeout = !TaskBase::TakeIndexed(NotifyIndices::SbcInterface, SbcConnectionTimeout);
					hadSbcTimeout = false;
					break;
				}
#endif
				hadTimeout = !TaskBase::TakeIndexed(NotifyIndices::SbcInterface, isConnected ? SbcConnectionTimeout : TaskBase::TimeoutUnlimited);
				hadSbcTimeout = hadTimeout && millis() - transferStartTime < SbcConnectionTimeout + SbcYieldTimeout;
				break;
			case TransferState::doingPartialTransfer:
				hadTimeout = !TaskBase::TakeIndexed(NotifyIndices::SbcInterface, SbcTransferTimeout);
				hadSbcTimeout = hadTimeout && millis() - transferStartTime < SbcTransferTimeout + SbcYieldTimeout;
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
#if SUPPORTS_SBC_OVER_USB
				if (transfer.GetTransportType() == SbcTransportType::usb)
				{
					SerialCDC *dev = transfer.GetUsbDevice();
					if (dev != nullptr && !dev->IsConnected())
					{
						reprap.GetPlatform().Message(NetworkInfoMessage, "Lost connection to SBC (port closed)\n");
					}
					else
					{
						reprap.GetPlatform().Message(NetworkInfoMessage, "Lost connection to SBC (timeout)\n");
					}
				}
				else
#endif
				{
					reprap.GetPlatform().Message(NetworkInfoMessage, "Lost connection to SBC (timeout)\n");
				}
			}
			else
			{
				reprap.GetPlatform().Message(NetworkInfoMessage, "Lost connection to SBC (connection reset)\n");
			}

			// Invalidate local resources
			InvalidateResources();
			if (hadReset)
			{
				// Let the main task invalidate resources before processing new data
				TaskBase::TakeIndexed(NotifyIndices::SbcInterface, SbcYieldTimeout);
			}

#if SUPPORTS_SBC_OVER_USB
			// On USB disconnect, exit direct mode and reinit the USB GCode device
			if (transfer.GetTransportType() == SbcTransportType::usb)
			{
				if (SerialCDC *dev = transfer.GetUsbDevice())
				{
					dev->EndDirectMode();
				}
				reprap.GetPlatform().ReinitUsbDevice(usbDeviceIndex);
				transfer.ResetConnection(true);

				continue;		// restart the task loop
			}
#endif
		}

		// Deal with received data
		if (transferComplete)
		{
			if (!isConnected)
			{
				isConnected = true;
				reprap.GetPlatform().MessageF(NetworkInfoMessage, "Connection to SBC established over %s!\n",
					transfer.GetTransportType() == SbcTransportType::usb ? "USB" : "SPI");
			}

			// Handle exchanged data and kick off the next transfer
			ExchangeData();
			transfer.StartNextTransfer();
		}
		else if (hadTimeout || hadReset)
		{
#if SUPPORTS_SBC_OVER_USB
			// If USB transport failed, always exit direct mode and reset to SPI
			if (transfer.GetTransportType() == SbcTransportType::usb)
			{
				if (SerialCDC *dev = transfer.GetUsbDevice())
				{
					dev->EndDirectMode();
				}
				reprap.GetPlatform().ReinitUsbDevice(usbDeviceIndex);
				transfer.ResetConnection(true);

				continue;
			}
#endif
			// SPI: reset the connection if no data could be exchanged
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
				debugPrintf("Error trying to read next packet\n");
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

			// Refuse codes with invalid lengths, else an over-long code would overrun GCodeBuffer::buffer in PutBinary later.
			// This check must come after ReadData so the payload is consumed and the next packet is read from the correct offset
			if (packet->length < sizeof(CodeHeader) || packet->length > MaxGCodeBinaryLength || (packet->length % sizeof(uint32_t)) != 0)
			{
				packetAcknowledged = codeBufferAvailable = false;
				break;
			}

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
				if ((txEnd == 0 && bufferedCodeSize > max<uint16_t>(rxPointer, SbcCodeBufferSize - txPointer)) ||
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
				if (txPointer + bufferedCodeSize > SbcCodeBufferSize)
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
					if (transfer.WriteObjectModel(nullptr))
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
			String<MaxSbcExpressionLength> expression;
			const GCodeChannel channel = transfer.ReadEvaluateExpression(packet->length, expression.GetRef());
			if (channel.IsValid())
			{
				GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
				if (gb == nullptr)
				{
					REPORT_INTERNAL_ERROR;
					break;
				}

				try
				{
					// Evaluate the expression and send the result to DSF
					MutexLocker lock(gb->mutex, SbcYieldTimeout);
					if (lock.IsAcquired())
					{
						ExpressionParser parser(gb, expression.c_str());
						const ExpressionValue val = parser.Parse();
						parser.CheckForExtraCharacters();
						if (val.GetType() == TypeCode::HeapArray)
						{
							// Write heap arrays as JSON
							OutputBuffer *json;
							if (OutputBuffer::Allocate(json))
							{
								ObjectExplorationContext context;
								ReportHeapArrayAsJson(json, context, nullptr, val.ahVal, "");
								packetAcknowledged = transfer.WriteEvaluationResult(channel, expression.c_str(), json);
							}
							else
							{
								packetAcknowledged = false;
							}
						}
						else if (val.GetType() == TypeCode::ObjectModelArray)
						{
							// Object model arrays need to be output differently
							OutputBuffer *json;
							if (OutputBuffer::Allocate(json))
							{
								ObjectExplorationContext context;
								context.AddIndex(val.param >> 8);
								val.omVal->ReportItemAsJsonFull(json, context, nullptr, val, "");
								packetAcknowledged = transfer.WriteEvaluationResult(channel, expression.c_str(), json);
							}
							else
							{
								packetAcknowledged = false;
							}
						}
						else
						{
							// Write plain result
							packetAcknowledged = transfer.WriteEvaluationResult(channel, expression.c_str(), val);
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
					packetAcknowledged = transfer.WriteEvaluationError(channel, expression.c_str(), errorMessage.c_str());
				}
				burstModeStartTime = millis();
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
					// Check if this is a targeted reply to a single channel whose code is executing on the SBC
					// (e.g. a response to M121 retransmitted by DSF). If so, mark the GB as finished.
					if ((type & PushFlag) == 0)
					{
						const Bitmap<uint32_t> destBits((uint32_t)type & (uint32_t)DestinationsMask);
						for (size_t channel = 0; channel < NumGCodeChannels; channel++)
						{
							if (destBits.IsOnlyBitSet(channel))
							{
								GCodeBuffer *const gb = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel(channel));
								if (gb != nullptr && gb->IsExecutingOnSbc())
								{
									gb->SetFinished(true);
								}
								break;
							}
						}
					}

					// Output message to the target
					reprap.GetPlatform().Message(type, buf);
				}
				else
				{
					// Not enough memory for reading the whole message, try again later
					OutputBuffer::ReleaseAll(buf);
					packetAcknowledged = false;
				}
			}
			else
			{
				// No output memory available, skip the packet content and try again later
				(void)transfer.ReadData(packet->length);
				packetAcknowledged = false;
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
					reprap.GetGCodes().PauseSequenceAborted(*gb);	// don't get stuck in "pausing" if this abort tears down the pause sequence
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
			String<MaxGCodeStringLength> expression;
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
				packetAcknowledged = transfer.WriteSetVariableError(channel, varName.c_str(), "expected a global or local variable");
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
						packetAcknowledged = transfer.WriteSetVariableError(channel, varName.c_str(), expression.c_str());
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
				packetAcknowledged = transfer.WriteSetVariableError(channel, varName.c_str(), expression.c_str());
				break;
			}
			if (!createVariable && v == nullptr)
			{
				// Save memory by re-using 'expression' to capture the error message
				expression.printf("unknown variable '%s'", varName.c_str());
				packetAcknowledged = transfer.WriteSetVariableError(channel, varName.c_str(), expression.c_str());
				break;
			}

			// Evaluate the expression and assign it
			try
			{
				ExpressionParser parser(gb, expression.c_str());
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
						packetAcknowledged = transfer.WriteSetVariableResult(channel, varName.c_str(), json);
					}
					else
					{
						packetAcknowledged = false;
					}
				}
				else if (ev.GetType() == TypeCode::ObjectModelArray)
				{
					// Object model arrays need to be output differently
					OutputBuffer *json;
					if (OutputBuffer::Allocate(json))
					{
						ObjectExplorationContext context;
						context.AddIndex(ev.param >> 8);
						ev.omVal->ReportItemAsJsonFull(json, context, nullptr, ev, "");
						packetAcknowledged = transfer.WriteSetVariableResult(channel, varName.c_str(), json);
					}
					else
					{
						packetAcknowledged = false;
					}
				}
				else
				{
					// Write plain result
					packetAcknowledged = transfer.WriteSetVariableResult(channel, varName.c_str(), ev);
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
				packetAcknowledged = transfer.WriteSetVariableError(channel, varName.c_str(), expression.c_str());
			}
			burstModeStartTime = millis();
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
			burstModeStartTime = millis();
			break;
		}

		// Result of a file exists check
		case SbcRequest::CheckFileExistsResult:
		{
			bool success = transfer.ReadBoolean();
			if (fileOperation == FileOperation::checkFileExists)
			{
				fileSuccess = success;
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;
		}

		// Result of a deletion request
		case SbcRequest::FileDeleteResult:
		{
			bool success = transfer.ReadBoolean();
			if (fileOperation == FileOperation::deleteFileOrDirectory
				|| fileOperation == FileOperation::deleteFileOrDirectoryRecursively
				|| fileOperation == FileOperation::secureDeleteFile)
			{
				fileSuccess = success;
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;
		}

		// Result of a file open request
		case SbcRequest::OpenFileResult:
		{
			FileHandle handle = transfer.ReadOpenFileResult(fileOffset);
			if (fileOperation == FileOperation::openRead ||
				fileOperation == FileOperation::openWrite ||
				fileOperation == FileOperation::openAppend)
			{
				fileHandle = handle;
				fileSuccess = (fileHandle != noFileHandle);
				fileOperation = FileOperation::none;
				if (fileSuccess)
				{
					numOpenFiles++;
				}
				fileSemaphore.Give();
			}
			break;
		}

		// Result of a file read request
		case SbcRequest::FileReadResult:
		{
			int bytesRead = transfer.ReadFileData(fileReadBuffer, fileBufferLength);
			if (fileOperation == FileOperation::read)
			{
				fileSuccess = bytesRead >= 0;
				fileOffset = fileSuccess ? bytesRead : 0;
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;
		}

		// Result of a directory listing request
		case SbcRequest::FileListResult:
		{
			bool endOfList;
			const size_t bytesRead = transfer.ReadFileList(fileReadBuffer, fileBufferLength, endOfList);
			if (fileOperation == FileOperation::getFileList)
			{
				fileSuccess = true;
				fileBufferLength = bytesRead;
				fileListEndOfList = endOfList;
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;
		}

		// Result of a file write request
		case SbcRequest::FileWriteResult:
		{
			bool success = transfer.ReadBoolean();
			if (fileOperation == FileOperation::write)
			{
				fileSuccess = success;
				if (!success || fileBufferLength == 0)
				{
					fileOperationPending = fileBufferLength != 0;
					fileOperation = FileOperation::none;
					fileSemaphore.Give();
				}
				else
				{
					fileOperationPending = true;
				}
			}
			break;
		}

		// Result of a file seek request
		case SbcRequest::FileSeekResult:
		{
			bool success = transfer.ReadBoolean();
			if (fileOperation == FileOperation::seek)
			{
				fileSuccess = success;
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;
		}

		// Result of a file seek request
		case SbcRequest::FileTruncateResult:
		{
			bool success = transfer.ReadBoolean();
			if (fileOperation == FileOperation::truncate)
			{
				fileSuccess = success;
				fileOperation = FileOperation::none;
				fileSemaphore.Give();
			}
			break;
		}

		// Request to update the last G-code result
		case SbcRequest::SetLastCodeResult:
		{
			GCodeResult result;
			const GCodeChannel channel = transfer.ReadSetLastCodeResult(result);

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

			gb->SetLastResult(result);
			break;
		}

		// Called when keys provided exclusively by DSF in SBC mode have changed
		case SbcRequest::ObjectModelKeyChanged:
		{
			const char *key = transfer.ReadData(packet->length);
			if (StringEqualsIgnoreCase(key, "network"))
			{
				reprap.NetworkUpdated();
			}
			else if (StringEqualsIgnoreCase(key, "volumes"))
			{
				reprap.VolumesUpdated();
			}
			break;
		}

		// Invalid request
		default:
			(void)transfer.ReadData(packet->length);		// skip the packet content
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
	const bool inBurstMode = (burstModeStartTime != 0 && millis() - burstModeStartTime < burstModeWindow);
	if (!inBurstMode && numEvents < numMaxEvents && !fileOperationPending && fileOperation == FileOperation::none)
	{
		// Normal mode: wait the full delay
		delaying = true;
		if (!TaskBase::TakeIndexed(NotifyIndices::SbcInterface, (numOpenFiles != 0) ? maxFileOpenDelay : maxDelayBetweenTransfers))
		{
			delaying = false;
		}
	}
	else if (inBurstMode && numEvents < numMaxEvents && !fileOperationPending && fileOperation == FileOperation::none)
	{
		// Burst mode with no pending events: use a short delay to avoid busy-waiting
		delaying = true;
		if (!TaskBase::TakeIndexed(NotifyIndices::SbcInterface, burstModeDelay))
		{
			delaying = false;
		}
	}
	// else: events pending or file operation in progress, proceed immediately
	numEvents = 0;
	if (burstModeStartTime != 0 && millis() - burstModeStartTime >= burstModeWindow)
	{
		burstModeStartTime = 0;
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
	DefragmentBufferedCodes();
	{
		TaskCriticalSectionLocker locker;
		if (!codeBufferAvailable || sendBufferUpdate)
		{
			const uint16_t bufferSpace = (txEnd == 0) ? max<uint16_t>(rxPointer, SbcCodeBufferSize - txPointer) : rxPointer - txPointer;
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
		case FileOperation::secureDeleteFile:
			fileOperationPending = !transfer.WriteSecureDeleteFile(filePath);
			break;

		case FileOperation::getFileList:
			fileOperationPending = !transfer.WriteGetFileList(filePath, fileOffset, fileBufferLength);
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
			fileOperationPending = !transfer.WriteFileData(fileHandle, fileWriteBuffer, fileBufferLength);
			if (!fileOperationPending)
			{
				// File data sent, move forwards in the buffer
				fileWriteBuffer += bytesNotWritten - fileBufferLength;
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

		// Invalidate buffered codes if required. It may take multiple transfers before a
		// print is actually paused, so make sure no more job codes are accepted until then
		if (gb->IsInvalidated())
		{
			InvalidateBufferedCodes(channel);
			if (!reportPause || (channel != GCodeChannel::File && channel != GCodeChannel::File2))
			{
				gb->Invalidate(false);
			}
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
					debugPrintf("Requesting macro file '%s' (channel %d, fromCode %s)\n", requestedMacroFile, (int)i, fromCode ? "true" : "false");
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
				if (gb->GetChannel() != GCodeChannel::Daemon && (gb->IsMacroRequestPending() || gb->HasJustStartedMacro()))
				{
					burstModeStartTime = millis();
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
							debugPrintf("Requesting non-blocking macro file '%s' (channel %i, fromCode %s)\n", requestedMacroFile, (int)i, fromCode ? "true" : "false");
						}
						gb->MacroRequestSent();
						gb->Invalidate();
					}
				}

				// Send pending firmware codes
				if (gb->IsSendRequested())
				{
					if (gb->HadExplicitLineNumber())
					{
						// Unfortunately, the explicit line number is stripped from the G-code data when we get here.
						// That means we need to prepend it again before the full code is sent over to the SBC
						String<MaxGCodeStringLength> code;
						code.printf("N%" PRIu32 " %s", gb->GetExplicitLineNumber(), gb->DataStart());
						if (transfer.WriteDoCode(channel, code.c_str(), code.strlen()))
						{
							gb->SentToSbc();
						}
					}
					else
					{
						if (transfer.WriteDoCode(channel, gb->DataStart(), gb->DataLength()))
						{
							gb->SentToSbc();
						}
					}
				}
			}
		}
	}

	// Send pause notification on demand
	if (reportPause && transfer.WritePrintPaused(pauseFilePosition, pauseFilePosition2, pauseReason))
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
#if SUPPORTS_SBC_OVER_USB
				// Cleanly disconnect USB before IAP brings up its own bare-metal stack. All IAP chunks
				// have arrived by now, so the link is no longer needed and the host gets a clean detach
				if (transfer.GetTransportType() == SbcTransportType::usb)
				{
					reprap.GetPlatform().DisconnectUsb();
					StopUsbTask();
				}
#endif
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
	txEnd = 0;
	txPointer = 0;
	rxPointer = 0;
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

		if (gb->IsExecutingOnSbc())
		{
			gb->SetFinished(true);
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
		gb->MessageAcknowledged(true, true, 0, ExpressionValue());
	}

	// Abort the print (if applicable)
	printAborted = true;

	// Turn off all the heaters
	reprap.GetHeat().SwitchOffAll(true);
}

void SbcInterface::Diagnostics(const StringRef& reply) noexcept
{
	reply.copy( "=== SBC interface ===");
	if (isConnected)
	{
		transfer.Diagnostics(reply);
	}
	else
	{
		reply.lcat("Not connected");
	}
	reply.lcatf("State: %d, disconnects: %" PRIu32 ", timeouts: %" PRIu32 " total, %" PRIu32 " by SBC, IAP RAM available 0x%05" PRIx32, (int)state, numDisconnects, numTimeouts, numSbcTimeouts, iapRamAvailable);
	reply.lcatf("Buffer RX/TX: %d/%d-%d, open files: %u", (int)rxPointer, (int)txPointer, (int)txEnd, numOpenFiles);
#ifdef TRACK_FILE_CODES
	reply.lcatf("File codes read/handled: %d/%d, file macros open/closing: %d %d", (int)fileCodesRead, (int)fileCodesHandled, (int)fileMacrosRunning, (int)fileMacrosClosing);
#endif
}

GCodeResult SbcInterface::HandleM576(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	if (gb.GetCommandFraction() == 1)
	{
#if SUPPORTS_SBC_OVER_USB
		if (IsConnected())
		{
			reply.copy("SBC interface already connected");
			return GCodeResult::error;
		}
		if (!gb.Seen('P'))
		{
			reply.copy("Protocol version parameter P required");
			return GCodeResult::error;
		}
		const uint16_t protocolVersion = (uint16_t)gb.GetUIValue();
		if (protocolVersion != SbcProtocolVersion)
		{
			reply.printf("Unsupported protocol version %u (expected %u)", protocolVersion, SbcProtocolVersion);
			return GCodeResult::error;
		}
		return reprap.SwitchToUsbSbcMode(gb, reply);
#else
		reply.copy("USB SBC mode not supported on this board");
		return GCodeResult::error;
#endif
	}

	bool seen = false;

	if (gb.Seen('S'))
	{
		uint32_t sParam = gb.GetUIValue();
		if (sParam > SbcConnectionTimeout)
		{
			reply.printf("SBC transfer delay must not exceed %" PRIu32 "ms", SbcConnectionTimeout);
			return GCodeResult::error;
		}
		maxDelayBetweenTransfers = sParam;
		seen = true;
	}

	if (gb.Seen('F'))
	{
		uint32_t fParam = gb.GetUIValue();
		if (fParam > SbcConnectionTimeout)
		{
			reply.printf("SBC transfer delay must not exceed %" PRIu32 "ms", SbcConnectionTimeout);
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

	if (gb.Seen('B'))
	{
		burstModeWindow = gb.GetUIValue();
		seen = true;
	}

	if (gb.Seen('D'))
	{
		burstModeDelay = gb.GetUIValue();
		seen = true;
	}

	if (!seen)
	{
		reply.printf("Max delay between full SBC transfers %" PRIu32 "ms (%" PRIu32 "ms during file IO), max events before skip: %" PRIu32 ", burst window: %" PRIu32 "ms, burst delay: %" PRIu32 "ms",
			maxDelayBetweenTransfers, maxFileOpenDelay, numMaxEvents, burstModeWindow, burstModeDelay);
	}
	return GCodeResult::ok;
}

#if SUPPORTS_SBC_OVER_USB

void SbcInterface::RequestUsbSwitch(SerialCDC *dev, unsigned int usbDevIndex) noexcept
{
	pendingUsbDevice = dev;
	usbDeviceIndex = usbDevIndex;
	sbcTask->Give(NotifyIndices::SbcInterface);		// wake the SBC task directly, bypassing IsConnected() check in EventOccurred
}

// Freeze the SBC task so a deliberate USB teardown isn't undone by its connection-loss recovery,
// which would EndDirectMode and ReinitUsbDevice and so re-attach the device. No-op when the SBC
// task itself is the caller, as it cannot suspend itself
void SbcInterface::Suspend() noexcept
{
	if (sbcTask != nullptr && TaskBase::GetCallerTaskHandle() != sbcTask)
	{
		sbcTask->Suspend();
	}
}

#endif

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
		// The whole walk must be atomic: readPointer is a cursor into variable-length records, so a concurrent defragment would invalidate it
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
				RRF_ASSERT(readPointer <= SbcCodeBufferSize);

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
								txPointer = 0;
								rxPointer = 0;
							}
							else if (readPointer == txEnd)
							{
								// Read last code before overlapping, restart from the beginning
								txEnd = 0;
								rxPointer = 0;
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
						txEnd = 0;
						rxPointer = 0;
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

bool SbcInterface::SecureDeleteFile(const char *filename) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		return false;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	filePath = filename;
	if (!DoFileOperation(FileOperation::secureDeleteFile))
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Timeout while trying to securely delete %s\n", filename);
		return false;
	}

	// Return the result
	return fileSuccess;
}

// Read part of a directory listing into the given buffer, returning the number of bytes read.
// endOfList is set when the returned data includes the final entry of the directory
size_t SbcInterface::GetFileList(const char *directory, uint32_t startIndex, char *buffer, size_t bufferLength, bool& endOfList) noexcept
{
	// Don't do anything if the SBC is not connected
	if (!IsConnected())
	{
		endOfList = true;
		return 0;
	}

	// Set up the request content
	MutexLocker locker(fileMutex);

	filePath = directory;
	fileOffset = startIndex;
	fileReadBuffer = buffer;
	fileBufferLength = bufferLength;
	if (!DoFileOperation(FileOperation::getFileList))
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Timeout while trying to list %s\n", directory);
		endOfList = true;
		return 0;
	}

	// Return the result
	endOfList = fileListEndOfList;
	return fileSuccess ? fileBufferLength : 0;
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
	fileOperationPending.store(true, std::memory_order_release);

	// Let the SBC task process this request as quickly as possible
	const bool isDelaying = delaying.exchange(false);
	if (isDelaying)
	{
		sbcTask->Give(NotifyIndices::SbcInterface);
	}

	const bool rslt = fileSemaphore.Take(SbcMaxRequestTime);
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
		const uint16_t bufferSpace = (txEnd == 0) ? max<uint16_t>(rxPointer, SbcCodeBufferSize - txPointer) : rxPointer - txPointer;
		if (bufferSpace > MaxGCodeBinaryLength)
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
			const bool tailDefragmented = DefragmentCodeBlock(rxPointer, txEnd);
			if (txEnd == rxPointer)
			{
				// The tail block contained no pending codes so DefragmentCodeBlock left txEnd == rxPointer, which is not a valid encoding.
				// Return to sequential mode, else the buffer walks in FillBuffer and InvalidateBufferedCodes would run off the end of the buffer
				rxPointer = 0;
				txEnd = 0;
				sendBufferUpdate = true;
			}
			else if (!tailDefragmented &&
				!DefragmentCodeBlock(0, txPointer) &&
				SbcCodeBufferSize - (size_t)txEnd > MaxGCodeBinaryLength)
			{
				size_t endBufferSize = txEnd - rxPointer;
				memmoveu32(reinterpret_cast<uint32_t*>(codeBuffer + SbcCodeBufferSize - endBufferSize), reinterpret_cast<uint32_t*>(codeBuffer + rxPointer), endBufferSize / sizeof(uint32_t));
				rxPointer = SbcCodeBufferSize - endBufferSize;
				txEnd = SbcCodeBufferSize;
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
					memcpyu32(reinterpret_cast<uint32_t*>(gapStart), reinterpret_cast<uint32_t *>(bufHeader), (codeBuffer + end - reinterpret_cast<const char *>(bufHeader)) / sizeof(uint32_t));
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
					txPointer = 0;
					rxPointer = 0;
					break;
				}
				else if (readPointer == txEnd)
				{
					// Invalidated last code before overlapping, continue from the beginning
					readPointer = 0;
					txEnd = 0;
					rxPointer = 0;
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
