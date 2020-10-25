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


// Handle a firmware update request
static void HandleFirmwareBlockRequest(CanMessageBuffer *buf)
pre(buf->id.MsgType() == CanMessageType::firmwareBlockRequest)
{
	const CanMessageFirmwareUpdateRequest& msg = buf->msg.firmwareUpdateRequest;
	const CanAddress src = buf->id.Src();
	if (   msg.bootloaderVersion == CanMessageFirmwareUpdateRequest::BootloaderVersion0
		&& (msg.fileWanted == (unsigned int)FirmwareModule::main || msg.fileWanted == (unsigned int)FirmwareModule::bootloader)
	   )																	// we only understand bootloader version 0 and files requests for main firmware and bootloader
	{
		String<MaxFilenameLength> fname;
		fname.copy((msg.fileWanted == (unsigned int)FirmwareModule::bootloader) ? "Duet3Bootloader-" : "Duet3Firmware_");
		fname.catn(msg.boardType, msg.GetBoardTypeLength(buf->dataLength));
		fname.cat(".bin");

		uint32_t fileOffset = msg.fileOffset, fileLength = 0;
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
					CanInterface::SendResponseNoFree(buf);

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
						CanMessageFirmwareUpdateResponse * msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
						const size_t lengthToSend = min<size_t>(bytesRead - bytesSent, sizeof(msgp->data));
						memcpy(msgp->data, data + bytesSent, lengthToSend);
						msgp->dataLength = lengthToSend;
						msgp->err = CanMessageFirmwareUpdateResponse::ErrNone;
						msgp->fileLength = fileLength;
						msgp->fileOffset = fileOffset;
						buf->dataLength = msgp->GetActualDataLength();
						CanInterface::SendResponseNoFree(buf);

						bytesSent += lengthToSend;
						fileOffset += lengthToSend;
						lreq -= lengthToSend;
						if (lreq == 0)
						{
							break;
						}

						if (bytesSent == (size_t)bytesRead)
						{
							data = reprap.GetLinuxInterface().GetFileChunk(fname.c_str(), fileOffset, lreq, bytesRead, fileLength);
							if (bytesRead < 0)
							{
								msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
								msgp->dataLength = 0;
								msgp->err = CanMessageFirmwareUpdateResponse::ErrOther;
								msgp->fileLength = fileLength;
								msgp->fileOffset = 0;
								buf->dataLength = msgp->GetActualDataLength();
								CanInterface::SendResponseNoFree(buf);

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
				CanInterface::SendResponseNoFree(buf);
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
					CanInterface::SendResponseNoFree(buf);

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
							CanInterface::SendResponseNoFree(buf);

							reprap.GetPlatform().MessageF(ErrorMessage, "Error reading firmware update file '%s'\n", fname.c_str());
							reprap.GetExpansion().UpdateFailed(src);
							return;
						}

						msgp->dataLength = lengthToSend;
						msgp->err = CanMessageFirmwareUpdateResponse::ErrNone;
						msgp->fileLength = fileLength;
						msgp->fileOffset = fileOffset;
						buf->dataLength = msgp->GetActualDataLength();
						CanInterface::SendResponseNoFree(buf);
						fileOffset += lengthToSend;
						lreq -= lengthToSend;
						if (lreq == 0)
						{
							break;
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
			CanInterface::SendResponseNoFree(buf);

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
		const unsigned int bootloaderVersion = msg.bootloaderVersion;
		const unsigned int fileWanted = msg.fileWanted;
		CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanId::MasterAddress, src);
		msgp->dataLength = 0;
		msgp->err = CanMessageFirmwareUpdateResponse::ErrOther;
		msgp->fileLength = 0;
		msgp->fileOffset = 0;
		buf->dataLength = msgp->GetActualDataLength();
		CanInterface::SendResponseNoFree(buf);
		reprap.GetPlatform().MessageF(ErrorMessage, "Can't satisfy request for firmware file %u from bootloader version %u\n", fileWanted, bootloaderVersion);
	}
}

// Handle an input state change message
static void HandleInputStateChanged(const CanMessageInputChanged& msg, CanAddress src)
{
	bool endstopStatesChanged = false;
	for (unsigned int i = 0; i < msg.numHandles; ++i)
	{
		const RemoteInputHandle handle(msg.handles[i]);
		const bool state = (msg.states & (1u << i)) != 0;
		switch (handle.u.parts.type)
		{
		case RemoteInputHandle::typeEndstop:
			reprap.GetPlatform().GetEndstops().HandleRemoteEndstopChange(src, handle.u.parts.major, handle.u.parts.minor, state);
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeZprobe:
			reprap.GetPlatform().GetEndstops().HandleRemoteZProbeChange(src, handle.u.parts.major, handle.u.parts.minor, state);
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeGpIn:
			reprap.GetPlatform().HandleRemoteGpInChange(src, handle.u.parts.major, handle.u.parts.minor, state);
			break;

		default:
			break;
		}
	}

	if (endstopStatesChanged)
	{
		reprap.GetPlatform().GetEndstops().OnEndstopOrZProbeStatesChanged();
	}
}

// Process a received broadcast or request message. Don't free the message buffer
void CommandProcessor::ProcessReceivedMessage(CanMessageBuffer *buf) noexcept
{
	if (buf->id.Src() != CanId::MasterAddress)								// I don't think we should receive our own broadcasts, but in case we do...
	{
		switch (buf->id.MsgType())
		{
		case CanMessageType::inputStateChanged:
			//TODO we should preferably handle this one using a separate high-priority queue or buffer
			HandleInputStateChanged(buf->msg.inputChanged, buf->id.Src());
			break;

		case CanMessageType::firmwareBlockRequest:
			HandleFirmwareBlockRequest(buf);
			break;

		case CanMessageType::sensorTemperaturesReport:
			reprap.GetHeat().ProcessRemoteSensorsReport(buf->id.Src(), buf->msg.sensorTemperaturesBroadcast);
			break;

		case CanMessageType::heatersStatusReport:
			reprap.GetHeat().ProcessRemoteHeatersReport(buf->id.Src(), buf->msg.heatersStatusBroadcast);
			break;

		case CanMessageType::fansReport:
			reprap.GetFansManager().ProcessRemoteFanRpms(buf->id.Src(), buf->msg.fansReport);
			break;

		case CanMessageType::announce:
			reprap.GetExpansion().ProcessAnnouncement(buf);
			break;

		case CanMessageType::filamentMonitorsStatusReport:
			FilamentMonitor::UpdateRemoteFilamentStatus(buf->id.Src(), buf->msg.filamentMonitorsStatus);
			break;

		case CanMessageType::driversStatusReport:	// not handled yet
		default:
			if (reprap.Debug(moduleCan))
			{
				buf->DebugPrint("Rec: ");
			}
			break;
		}
	}
}

#endif

// End
