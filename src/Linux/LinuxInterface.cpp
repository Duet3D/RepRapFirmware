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
#include "GCodes/GCodes.h"
#include "Platform.h"
#include "PrintMonitor.h"
#include "Tools/Filament.h"
#include "RepRap.h"
#include "RepRapFirmware.h"
#include <Hardware/Cache.h>

LinuxInterface::LinuxInterface() : transfer(new DataTransfer()), wasConnected(false), numDisconnects(0),
	reportPause(false), rxPointer(0), txPointer(0), txLength(0), sendBufferUpdate(true),
	iapWritePointer(IAP_IMAGE_START), gcodeReply(new OutputStack())
{
}

void LinuxInterface::Init()
{
	gcodeReplyMutex.Create("LinuxReply");
	transfer->Init();
	transfer->StartNextTransfer();
}

void LinuxInterface::Spin()
{
	bool writingIap = false;
	do
	{
		if (transfer->IsReady())
		{
			// Process incoming packets
			for (size_t i = 0; i < transfer->PacketsToRead(); i++)
			{
				const PacketHeader * const packet = transfer->ReadPacket();
				if (packet == nullptr)
				{
					if (reprap.Debug(moduleLinuxInterface))
					{
						reprap.GetPlatform().Message(DebugMessage, "Error trying to read next SPI packet\n");
					}
					break;
				}

				if (packet->request >= (uint16_t)LinuxRequest::InvalidRequest)
				{
					REPORT_INTERNAL_ERROR;
					return;
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
					reprap.SoftwareReset((uint16_t)SoftwareResetReason::user);
					return;

				// Perform a G/M/T-code
				case LinuxRequest::Code:
				{
					// Check if the code overlaps. If so, restart from the beginning
					if (txPointer + sizeof(BufferedCodeHeader) + packet->length > SpiCodeBufferSize)
					{
						if (rxPointer == txPointer)
						{
							rxPointer = 0;
						}
						txLength = txPointer;
						txPointer = 0;
						sendBufferUpdate = true;
					}

					// Store the buffer header
					BufferedCodeHeader *bufHeader = reinterpret_cast<BufferedCodeHeader*>(codeBuffer + txPointer);
					bufHeader->isPending = true;
					bufHeader->length = packet->length;
					txPointer += sizeof(BufferedCodeHeader);

					// Store the code content
					size_t dataLength = packet->length;
					memcpy(codeBuffer + txPointer, transfer->ReadData(packet->length), dataLength);
					txPointer += dataLength;
					break;
				}

				// Get the object model
				case LinuxRequest::GetObjectModel:
				{
					String<StringLength100> key;
					StringRef keyRef = key.GetRef();
					String<StringLength20> flags;
					StringRef flagsRef = flags.GetRef();
					transfer->ReadGetObjectModel(packet->length, keyRef, flagsRef);

					try
					{
						OutputBuffer *outBuf = reprap.GetModelResponse(key.c_str(), flags.c_str());
						if (outBuf == nullptr || !transfer->WriteObjectModel(outBuf))
						{
							// Failed to write the whole object model, try again later
							packetAcknowledged = false;
							OutputBuffer::ReleaseAll(outBuf);
						}
					}
					catch (GCodeException& e)
					{
						// Get the error message and send it back to DSF
						OutputBuffer *buf;
						if (OutputBuffer::Allocate(buf))
						{
							String<StringLength100> errorMessage;
							e.GetMessage(errorMessage.GetRef(), nullptr);
							buf->cat(errorMessage.c_str());
							if (!transfer->WriteObjectModel(buf))
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
					const char * const data = transfer->ReadData(dataLength);
					// TODO implement this
					(void)data;
					break;
				}

				// Print has been started, set file print info
				case LinuxRequest::PrintStarted:
				{
					String<MaxFilenameLength> filename;
					StringRef filenameRef = filename.GetRef();
					transfer->ReadPrintStartedInfo(packet->length, filenameRef, fileInfo);
					reprap.GetPrintMonitor().SetPrintingFileInfo(filename.c_str(), fileInfo);
					reprap.GetGCodes().StartPrinting(true);
					break;
				}

				// Print has been stopped
				case LinuxRequest::PrintStopped:
				{
					const PrintStoppedReason reason = transfer->ReadPrintStoppedInfo();
					if (reason == PrintStoppedReason::normalCompletion)
					{
						// Just mark the print file as finished
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::File);
						gb->SetPrintFinished();
					}
					else
					{
						// Stop the print with the given reason
						reprap.GetGCodes().StopPrint((StopPrintReason)reason);
						InvalidateBufferChannel(GCodeChannel::File);
					}
					break;
				}

				// Macro file has been finished
				case LinuxRequest::MacroCompleted:
				{
					bool error;
					const GCodeChannel channel = transfer->ReadMacroCompleteInfo(error);
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
						gb->MachineState().SetFileFinished(error);

						if (reprap.Debug(moduleLinuxInterface))
						{
							reprap.GetPlatform().MessageF(DebugMessage, "Macro completed on channel %u\n", channel.ToBaseType());
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
					packetAcknowledged = transfer->WriteHeightMap();
					break;

				// Set heightmap via G29 S1
				case LinuxRequest::SetHeightMap:
					transfer->ReadHeightMap();
					break;

				// Lock movement and wait for standstill
				case LinuxRequest::LockMovementAndWaitForStandstill:
				{
					const GCodeChannel channel = transfer->ReadCodeChannel();
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
						if (reprap.GetGCodes().LockMovementAndWaitForStandstill(*gb))
						{
							transfer->WriteLocked(channel);
						}
						else
						{
							transfer->ResendPacket(packet);
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
					const GCodeChannel channel = transfer->ReadCodeChannel();
					if (channel.IsValid())
					{
						GCodeBuffer * const gb = reprap.GetGCodes().GetGCodeBuffer(channel);
						reprap.GetGCodes().UnlockAll(*gb);
					}
					else
					{
						REPORT_INTERNAL_ERROR;
					}
					break;
				}

				// Write another chunk of the IAP binary to the designated Flash area
				case LinuxRequest::WriteIap:
					if (iapWritePointer == IAP_IMAGE_START)			// if start of IAP write
					{
						reprap.PrepareToLoadIap();
						writingIap = true;
					}
					memcpy(reinterpret_cast<char *>(iapWritePointer), transfer->ReadData(packet->length), packet->length);
					iapWritePointer += packet->length;
					break;

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
					transfer->ReadAssignFilament(extruder, filamentRef);

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
						reprap.MoveUpdated();
					}
					break;
				}

				// Return a file chunk
				case LinuxRequest::FileChunk:
					transfer->ReadFileChunk(requestedFileChunk, requestedFileDataLength, requestedFileLength);
					requestedFileSemaphore.Give();
					break;

				// Evaluate an expression
				case LinuxRequest::EvaluateExpression:
				{
					String<StringLength100> expression;
					StringRef expressionRef = expression.GetRef();
					GCodeChannel channel = transfer->ReadEvaluateExpression(packet->length, expressionRef);
					if (channel.IsValid())
					{
						try
						{
							// Evaluate the expression and send the result to DSF
							const GCodeBuffer *gb = reprap.GetGCodes().GetInput(channel);
							ExpressionParser parser(*gb, expression.c_str(), expression.c_str() + expression.strlen());
							const ExpressionValue val = parser.Parse();
							packetAcknowledged = transfer->WriteEvaluationResult(expression.c_str(), val);
						}
						catch (GCodeException& e)
						{
							// Get the error message and send it back to DSF
							String<StringLength100> errorMessage;
							e.GetMessage(errorMessage.GetRef(), nullptr);
							packetAcknowledged = transfer->WriteEvaluationError(expression.c_str(), errorMessage.c_str());
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
						if (transfer->ReadMessage(type, buf))
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

				// Invalid request
				default:
					REPORT_INTERNAL_ERROR;
					break;
				}

				// Request the packet again if no response could be sent back
				if (!packetAcknowledged)
				{
					transfer->ResendPacket(packet);
				}
			}

			// Send code replies and generic messages
			if (!gcodeReply->IsEmpty())
			{
				MutexLocker lock(gcodeReplyMutex);
				while (!gcodeReply->IsEmpty())
				{
					const MessageType type = gcodeReply->GetFirstItemType();
					OutputBuffer *buffer = gcodeReply->GetFirstItem();			// this may be null
					if (!transfer->WriteCodeReply(type, buffer))				// this handles the null case too
					{
						break;
					}
					gcodeReply->SetFirstItem(buffer);							// this does a pop if buffer is null
				}
			}

			// Notify DSF about the available buffer space
			if (sendBufferUpdate || transfer->LinuxHadReset())
			{
				const uint16_t bufferSpace = (txLength == 0) ? max<uint16_t>(rxPointer, SpiCodeBufferSize - txPointer) : rxPointer - txPointer;
				sendBufferUpdate = !transfer->WriteCodeBufferUpdate(bufferSpace);
			}

			if (!writingIap)					// it's not safe to access GCodes once we have started writing the IAP
			{
				// Get another chunk of the file being requested
				if (!requestedFileName.IsEmpty() && !reprap.GetGCodes().IsFlashing() &&
					transfer->WriteFileChunkRequest(requestedFileName.c_str(), requestedFileOffset, requestedFileLength))
				{
					requestedFileName.Clear();
				}

				// Deal with code channel requests
				bool reportMissing, fromCode;
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

					// Handle macro start requests
					if (gb->IsMacroRequested())
					{
						const char * const requestedMacroFile = gb->GetRequestedMacroFile(reportMissing, fromCode);
						if (transfer->WriteMacroRequest(channel, requestedMacroFile, reportMissing, fromCode))
						{
							if (reprap.Debug(moduleLinuxInterface))
							{
								reprap.GetPlatform().MessageF(DebugMessage, "Requesting macro file '%s' (reportMissing: %s fromCode: %s)\n", requestedMacroFile, reportMissing ? "true" : "false", fromCode ? "true" : "false");
							}
							gb->MacroRequestSent();
							gb->Invalidate();
						}
					}

					// Handle file abort requests
					if (gb->IsAbortRequested() && transfer->WriteAbortFileRequest(channel, gb->IsAbortAllRequested()))
					{
						gb->AcknowledgeAbort();
						gb->Invalidate();
					}

					// Handle blocking messages
					if (gb->MachineState().waitingForAcknowledgement && !gb->MachineState().waitingForAcknowledgementSent &&
						transfer->WriteWaitForAcknowledgement(channel))
					{
						gb->MachineState().waitingForAcknowledgementSent = true;
						gb->Invalidate();
					}

					// Send pending firmware codes
					if (gb->IsSendRequested() && transfer->WriteDoCode(channel, gb->DataStart(), gb->DataLength()))
					{
						gb->SetFinished(true);
					}
				}

				// Send pause notification on demand
				if (reportPause && transfer->WritePrintPaused(pauseFilePosition, pauseReason))
				{
					reportPause = false;
					reprap.GetGCodes().GetGCodeBuffer(GCodeChannel::File)->Invalidate();
				}
			}

			// Start the next transfer
			transfer->StartNextTransfer();
			if (!wasConnected && !writingIap)
			{
				reprap.GetPlatform().Message(NetworkInfoMessage, "Connection to Linux established!\n");
			}
			wasConnected = true;
		}
		else if (!transfer->IsConnected() && wasConnected && !writingIap)
		{
			reprap.GetPlatform().Message(NetworkInfoMessage, "Lost connection to Linux\n");

			wasConnected = false;
			numDisconnects++;

			rxPointer = txPointer = txLength = 0;
			sendBufferUpdate = true;
			iapWritePointer = IAP_IMAGE_START;

			if (!requestedFileName.IsEmpty())
			{
				requestedFileDataLength = -1;
				requestedFileSemaphore.Give();
			}

			// Don't cache any messages if they cannot be sent
			{
				MutexLocker lock(gcodeReplyMutex);
				gcodeReply->ReleaseAll();
			}

			// Close all open G-code files
			for (size_t i = 0; i < NumGCodeChannels; i++)
			{
				GCodeBuffer *gb = reprap.GetGCodes().GetGCodeBuffer((GCodeChannel)i);
				gb->AbortFile(true, false);
				gb->MessageAcknowledged(true);
			}
			reprap.GetGCodes().StopPrint(StopPrintReason::abort);

			// Invalidate the G-code buffers holding binary data (if applicable)
			for (size_t i = 0; i < NumGCodeChannels; i++)
			{
				GCodeBuffer *gb = reprap.GetGCodes().GetGCodeBuffer((GCodeChannel)i);
				if (gb->IsBinary() && gb->IsCompletelyIdle())
				{
					gb->Reset();
				}
			}
		}
	} while (writingIap);
}

void LinuxInterface::Diagnostics(MessageType mtype)
{
	reprap.GetPlatform().Message(mtype, "=== Linux interface ===\n");
	transfer->Diagnostics(mtype);
	reprap.GetPlatform().MessageF(mtype, "Number of disconnects: %" PRIu32 "\n", numDisconnects);
	reprap.GetPlatform().MessageF(mtype, "Buffer RX/TX: %d/%d-%d\n", (int)rxPointer, (int)txPointer, (int)txLength);
}

bool LinuxInterface::IsConnected() const
{
	return transfer->IsConnected();
}

bool LinuxInterface::FillBuffer(GCodeBuffer &gb)
{
	if (gb.IsInvalidated() ||
		gb.IsMacroRequested() || gb.IsAbortRequested() || (reportPause && gb.GetChannel() == GCodeChannel::File) ||
		(gb.MachineState().waitingForAcknowledgement && !gb.MachineState().waitingForAcknowledgementSent))
	{
		// Don't process codes that are supposed to be suspended...
		return false;
	}

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
					gb.PutAndDecode(reinterpret_cast<const char *>(header), bufHeader->length, true);
					bufHeader->isPending = false;

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

					return true;
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
	return false;
}

// Read a file chunk from the SBC. When a response has been received, the current thread is woken up again.
// If an error occurred, the number of bytes read is -1
const char *LinuxInterface::GetFileChunk(const char *filename, uint32_t offset, uint32_t maxLength, int32_t& dataLength, uint32_t& fileLength)
{
	requestedFileName.copy(filename);
	requestedFileLength = min<uint32_t>(maxLength, MaxFileChunkSize);
	requestedFileOffset = offset;

	requestedFileSemaphore.Take();

	dataLength = requestedFileDataLength;
	fileLength = requestedFileLength;
	return requestedFileChunk;
}

void LinuxInterface::HandleGCodeReply(MessageType mt, const char *reply)
{
	if (!transfer->IsConnected())
	{
		return;
	}

	MutexLocker lock(gcodeReplyMutex);
	OutputBuffer *buffer = gcodeReply->GetLastItem();
	if (buffer != nullptr && mt == gcodeReply->GetLastItemType() && (mt & PushFlag) != 0 && !buffer->IsReferenced())
	{
		// Try to save some space by combining segments that have the Push flag set
		buffer->cat(reply);
	}
	else if (reply[0] != 0 && OutputBuffer::Allocate(buffer))
	{
		// Attempt to allocate one G-code buffer per non-empty output message
		buffer->cat(reply);
		gcodeReply->Push(buffer, mt);
	}
	else
	{
		// Store nullptr to indicate an empty response. This way many OutputBuffer references can be saved
		gcodeReply->Push(nullptr, mt);
	}
}

void LinuxInterface::HandleGCodeReply(MessageType mt, OutputBuffer *buffer)
{
	if (!transfer->IsConnected())
	{
		OutputBuffer::ReleaseAll(buffer);
		return;
	}

	MutexLocker lock(gcodeReplyMutex);
	gcodeReply->Push(buffer, mt);
}

void LinuxInterface::InvalidateBufferChannel(GCodeChannel channel)
{
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
