/*
 * CommandProcessor.cpp
 *
 *  Created on: 12 Aug 2019
 *      Author: David
 */

#include "CommandProcessor.h"

#if SUPPORT_CAN_EXPANSION

#include "CanInterface.h"
#include <CanMessageBuffer.h>
#include "RepRap.h"
#include "Platform.h"
#include "Heating/Heat.h"
#include "ExpansionManager.h"

#if HAS_LINUX_INTERFACE
# include "Linux/LinuxInterface.h"
#endif


// Handle a firmware update request and free the buffer
static void HandleFirmwareBlockRequest(CanMessageBuffer *buf)
pre(buf->id.MsgType() == CanMessageType::FirmwareBlockRequest)
{
	const CanMessageFirmwareUpdateRequest& msg = buf->msg.firmwareUpdateRequest;
	const CanAddress src = buf->id.Src();
	if (msg.bootloaderVersion == CanMessageFirmwareUpdateRequest::BootloaderVersion0)		// we only understand bootloader version 0
	{
		String<MaxFilenameLength> fname;
		fname.copy("Duet3Firmware_");
		fname.catn(msg.boardType, msg.GetBoardTypeLength(buf->dataLength));
		fname.cat(".bin");

		uint32_t fileOffset = msg.fileOffset, fileLength = 0;
		if (fileOffset == 0)
		{
// DC disabled this because it is buggy
//			CanInterface::UpdateStarting();
		}

		uint32_t lreq = msg.lengthRequested;

#if HAS_LINUX_INTERFACE
		if (reprap.UsingLinuxInterface())
		{
			// Fetch the firmware file from the SBC
			int32_t bytesRead;
			const char *data = reprap.GetLinuxInterface().GetFileChunk(fname.c_str(), fileOffset, lreq, bytesRead, fileLength);
			if (bytesRead > 0)
			{
				if (fileOffset >= fileLength)
				{
					CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
					msgp->dataLength = 0;
					msgp->err = CanMessageFirmwareUpdateResponse::ErrBadOffset;
					msgp->fileLength = fileLength;
					msgp->fileOffset = 0;
					buf->dataLength = msgp->GetActualDataLength();
					CanInterface::SendResponse(buf);

					reprap.GetPlatform().MessageF(ErrorMessage, "Received firmware update request with bad file offset, actual %" PRIu32 " max %" PRIu32 "\n", fileOffset, fileLength);
				}
				else
				{
					if (fileLength - fileOffset < lreq)
					{
						lreq = fileLength - fileOffset;
					}

//debugPrintf("Sending %" PRIu32 " bytes at offset %" PRIu32 "\n", lreq, fileOffset);

					size_t bytesSent = 0;
					for (;;)
					{
						CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
						const size_t lengthToSend = min<size_t>(bytesRead - bytesSent, sizeof(msgp->data));
						memcpy(msgp->data, data + bytesSent, lengthToSend);
						msgp->dataLength = lengthToSend;
						msgp->err = CanMessageFirmwareUpdateResponse::ErrNone;
						msgp->fileLength = fileLength;
						msgp->fileOffset = fileOffset;
						buf->dataLength = msgp->GetActualDataLength();
						CanInterface::SendResponse(buf);

						bytesSent += lengthToSend;
						fileOffset += lengthToSend;
						lreq -= lengthToSend;
						if (lreq == 0)
						{
							break;
						}

						while ((buf = CanMessageBuffer::Allocate()) == nullptr)
						{
							delay(1);
						}

						if (bytesSent == (size_t)bytesRead)
						{
							data = reprap.GetLinuxInterface().GetFileChunk(fname.c_str(), fileOffset, lreq, bytesRead, fileLength);
							if (bytesRead < 0)
							{
								CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
								msgp->dataLength = 0;
								msgp->err = CanMessageFirmwareUpdateResponse::ErrOther;
								msgp->fileLength = fileLength;
								msgp->fileOffset = 0;
								buf->dataLength = msgp->GetActualDataLength();
								CanInterface::SendResponse(buf);

								reprap.GetPlatform().MessageF(ErrorMessage, "Error reading firmware update file '%s'\n", fname.c_str());
								reprap.GetExpansion().UpdateFailed(src);
								return;
							}
							bytesSent = 0;
						}
					}
				}
			}
			else
			{
				CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
				msgp->dataLength = 0;
				msgp->err = CanMessageFirmwareUpdateResponse::ErrNoFile;
				msgp->fileLength = fileLength;
				msgp->fileOffset = 0;
				buf->dataLength = msgp->GetActualDataLength();
				CanInterface::SendResponse(buf);
				reprap.GetPlatform().MessageF(ErrorMessage, "Firmware file %s not found", fname.c_str());
				reprap.GetExpansion().UpdateFailed(src);
				return;
			}
		}
		else
#endif
		{
#if HAS_MASS_STORAGE
			// Fetch the firmware file from the local SD card
			FileStore * const f = reprap.GetPlatform().OpenFile(DEFAULT_SYS_DIR, fname.c_str(), OpenMode::read);
			if (f != nullptr)
			{
				fileLength = f->Length();
				if (fileOffset >= fileLength)
				{
					CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
					msgp->dataLength = 0;
					msgp->err = CanMessageFirmwareUpdateResponse::ErrBadOffset;
					msgp->fileLength = fileLength;
					msgp->fileOffset = 0;
					buf->dataLength = msgp->GetActualDataLength();
					CanInterface::SendResponse(buf);

					reprap.GetPlatform().MessageF(ErrorMessage, "Received firmware update request with bad file offset, actual %" PRIu32 " max %" PRIu32 "\n", fileOffset, fileLength);
				}
				else
				{
					f->Seek(fileOffset);
					if (fileLength - fileOffset < lreq)
					{
						lreq = fileLength - fileOffset;
					}

//debugPrintf("Sending %" PRIu32 " bytes at offset %" PRIu32 "\n", lreq, fileOffset);

					for (;;)
					{
						CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
						const size_t lengthToSend = min<size_t>(lreq, sizeof(msgp->data));
						if (f->Read(msgp->data, lengthToSend) != (int)lengthToSend)
						{
							msgp->dataLength = 0;
							msgp->err = CanMessageFirmwareUpdateResponse::ErrOther;
							msgp->fileLength = fileLength;
							msgp->fileOffset = 0;
							buf->dataLength = msgp->GetActualDataLength();
							CanInterface::SendResponse(buf);

							reprap.GetPlatform().MessageF(ErrorMessage, "Error reading firmware update file '%s'\n", fname.c_str());
							reprap.GetExpansion().UpdateFailed(src);
							return;
						}
						msgp->dataLength = lengthToSend;
						msgp->err = CanMessageFirmwareUpdateResponse::ErrNone;
						msgp->fileLength = fileLength;
						msgp->fileOffset = fileOffset;
						buf->dataLength = msgp->GetActualDataLength();
						CanInterface::SendResponse(buf);
						fileOffset += lengthToSend;
						lreq -= lengthToSend;
						if (lreq == 0)
						{
							break;
						}
						while ((buf = CanMessageBuffer::Allocate()) == nullptr)
						{
							delay(1);
						}
					}
				}
				f->Close();
			}
#endif
		}

		if (lreq != 0)			// if we didn't complete the request
		{
			CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
			msgp->dataLength = 0;
			msgp->err = CanMessageFirmwareUpdateResponse::ErrNoFile;
			msgp->fileLength = 0;
			msgp->fileOffset = 0;
			buf->dataLength = msgp->GetActualDataLength();
			CanInterface::SendResponse(buf);

			reprap.GetPlatform().MessageF(ErrorMessage, "Received firmware update request for missing file '%s'\n", fname.c_str());
			reprap.GetExpansion().UpdateFailed(src);
		}
		else if (fileOffset == fileLength)
		{
			reprap.GetExpansion().UpdateFinished(src);
		}
	}
	else
	{
		const uint32_t bootloaderVersion = msg.bootloaderVersion;
		CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
		msgp->dataLength = 0;
		msgp->err = CanMessageFirmwareUpdateResponse::ErrOther;
		msgp->fileLength = 0;
		msgp->fileOffset = 0;
		buf->dataLength = msgp->GetActualDataLength();
		CanInterface::SendResponse(buf);
		reprap.GetPlatform().MessageF(ErrorMessage, "Received firmware update request from unknown bootloader version %" PRIu32 "\n", bootloaderVersion);
	}
}

// Handle an input state change message
static void HandleInputStateChanged(const CanMessageInputChanged& msg, CanAddress src)
{
	bool endstopStatesChanged = false;
	for (unsigned int i = 0; i < msg.numHandles; ++i)
	{
		const RemoteInputHandle handle(msg.handles[i]);
		const bool state = (msg.states & (1 << i)) != 0;
		switch (handle.u.parts.type)
		{
		case RemoteInputHandle::typeEndstop:
			reprap.GetPlatform().GetEndstops().HandleRemoteInputChange(src, handle.u.parts.major, handle.u.parts.minor, state);
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeTrigger:
			//TODO see if any triggers are waiting for this state change
		default:
			break;
		}
	}

	if (endstopStatesChanged)
	{
		reprap.GetPlatform().GetEndstops().OnEndstopStatesChanged();
	}
}

// Process a received broadcast or request message and free the message buffer
void CommandProcessor::ProcessReceivedMessage(CanMessageBuffer *buf) noexcept
{
	if (buf->id.Src() == CanId::MasterAddress)
	{
		// I don't think we should receive our own broadcasts, but in case we do...
		CanMessageBuffer::Free(buf);
	}
	else
	{
		// In the following switch, each case must release the message buffer, either directly or by re-using it to send a response
		switch (buf->id.MsgType())
		{
		case CanMessageType::inputStateChanged:
			//TODO we should preferably handle this one using a separate high-priority queue or buffer
			HandleInputStateChanged(buf->msg.inputChanged, buf->id.Src());
			CanMessageBuffer::Free(buf);
			break;

		case CanMessageType::firmwareBlockRequest:
			HandleFirmwareBlockRequest(buf);					// this one reuses or frees the buffer
			break;

		case CanMessageType::sensorTemperaturesReport:
			reprap.GetHeat().ProcessRemoteSensorsReport(buf->id.Src(), buf->msg.sensorTemperaturesBroadcast);
			CanMessageBuffer::Free(buf);
			break;

		case CanMessageType::heatersStatusReport:
			reprap.GetHeat().ProcessRemoteHeatersReport(buf->id.Src(), buf->msg.heatersStatusBroadcast);
			CanMessageBuffer::Free(buf);
			break;

		case CanMessageType::fanRpmReport:
			reprap.GetFansManager().ProcessRemoteFanRpms(buf->id.Src(), buf->msg.fanRpms);
			CanMessageBuffer::Free(buf);
			break;

		case CanMessageType::announce:
			reprap.GetExpansion().ProcessAnnouncement(buf);		// this one reuses or frees the buffer
			break;

		case CanMessageType::statusReport:
		default:
//			buf->DebugPrint("Rec: ");
			CanMessageBuffer::Free(buf);
			break;
		}
	}
}

#endif

// End
