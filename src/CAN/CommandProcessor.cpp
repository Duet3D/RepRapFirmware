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

		uint32_t fileOffset = msg.fileOffset;
		uint32_t lreq = msg.lengthRequested;

#if HAS_LINUX_INTERFACE
		if (reprap.UsingLinuxInterface())
		{
			// Request another chunk of data from the given file
			while (lreq > 0)
			{
				reprap.GetLinuxInterface().RequestFileChunk(fname.c_str(), fileOffset, lreq, RTOSIface::GetCurrentTask());
				TaskBase::Take();

				int bytesRead;
				uint32_t fileLength = 0;
				const char *data = reprap.GetLinuxInterface().GetFileChunk(bytesRead, fileLength);
				if (bytesRead < 0)
				{
					break;
				}

				size_t bytesSent = 0;
				for (;;)
				{
					CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(CanId::MasterAddress, src);
					const size_t lengthToSend = min<size_t>(lreq, sizeof(msgp->data));
					memcpy(msgp->data, data + bytesSent, lengthToSend);
					bytesSent += lengthToSend;
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
		}
		else
#endif
		{
#if HAS_MASS_STORAGE
			// Fetch the firmware file from the local SD card
			FileStore * const f = reprap.GetPlatform().OpenSysFile(fname.c_str(), OpenMode::read);
			if (f != nullptr)
			{
				const uint32_t fileLength = f->Length();
				if (fileOffset >= fileLength)
				{
					CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(CanId::MasterAddress, src);
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
						CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(CanId::MasterAddress, src);
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
							break;
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
			CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(CanId::MasterAddress, src);
			msgp->dataLength = 0;
			msgp->err = CanMessageFirmwareUpdateResponse::ErrNoFile;
			msgp->fileLength = 0;
			msgp->fileOffset = 0;
			buf->dataLength = msgp->GetActualDataLength();
			CanInterface::SendResponse(buf);
			reprap.GetPlatform().MessageF(ErrorMessage, "Received firmware update request for missing file '%s'\n", fname.c_str());
		}
	}
	else
	{
		const uint32_t bootloaderVersion = msg.bootloaderVersion;
		CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(CanId::MasterAddress, src);
		msgp->dataLength = 0;
		msgp->err = CanMessageFirmwareUpdateResponse::ErrOther;
		msgp->fileLength = 0;
		msgp->fileOffset = 0;
		buf->dataLength = msgp->GetActualDataLength();
		CanInterface::SendResponse(buf);
		reprap.GetPlatform().MessageF(ErrorMessage, "Received firmware update request from unknown bootloader version %" PRIu32 "\n", bootloaderVersion);
	}
}

// Handle a temperature report and free the buffer
static void HandleTemperatureReport(CanMessageBuffer *buf)
pre(buf->id.MsgType() == CanMessageType::temperatureReport)
{
	const CanMessageSensorTemperatures& msg = buf->msg.sensorTemperaturesBroadcast;
	uint64_t sensorsReported = msg.whichSensors;
	size_t index = 0;
	for (unsigned int sensor = 0; sensor < 64 && sensorsReported != 0; ++sensor)
	{
		if (((uint8_t)sensorsReported & 1u) != 0)
		{
			if (index < ARRAY_SIZE(msg.temperatureReports))
			{
				reprap.GetHeat().UpdateRemoteSensorTemperature(sensor, msg.temperatureReports[index]);
			}
			++index;
		}
		sensorsReported >>= 1;
	}
	CanMessageBuffer::Free(buf);
}

// Process a received broadcast or request message amd free the message buffer
void CommandProcessor::ProcessReceivedMessage(CanMessageBuffer *buf)
{
	// In the following switch, each case must release the message buffer, either directly or by re-using it to send a response
	switch (buf->id.MsgType())
	{
	case CanMessageType::FirmwareBlockRequest:
		HandleFirmwareBlockRequest(buf);
		break;

	case CanMessageType::sensorTemperaturesReport:
		HandleTemperatureReport(buf);
		break;

	case CanMessageType::statusReport:
	default:
//		buf->DebugPrint("Rec: ");
		CanMessageBuffer::Free(buf);
		break;
	}
}

#endif

// End
