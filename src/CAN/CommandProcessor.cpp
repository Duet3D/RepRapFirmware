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

#ifndef DUET3_ATE
# include <Movement/Move.h>
# include <Version.h>

# if SUPPORT_TMC2660
#  include "Movement/StepperDrivers/TMC2660.h"
# endif
# if SUPPORT_TMC22xx
#  include "Movement/StepperDrivers/TMC22xx.h"
# endif
# if SUPPORT_TMC51xx
#  include "Movement/StepperDrivers/TMC51xx.h"
# endif
#endif

#if HAS_LINUX_INTERFACE
# include "Linux/LinuxInterface.h"

constexpr size_t MaxFileChunkSize = 448;	// Maximum size of file chunks for reading files from the SBC. Should be a multiple of sizeof(CanMessageFirmwareUpdateResponse::data) for best CAN performance
char sbcFirmwareChunk[MaxFileChunkSize];
#endif

// Enter test mode
static void EnterTestMode(const CanMessageEnterTestMode& msg) noexcept
{
	if (msg.passwd == CanMessageEnterTestMode::Passwd)
	{
		CanInterface::EnterTestMode(msg.parameter);
	}
}

// Handle a firmware update request
static void HandleFirmwareBlockRequest(CanMessageBuffer *buf) noexcept
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
			uint32_t bytesRead = min<uint32_t>(lreq, MaxFileChunkSize);
			if (reprap.GetLinuxInterface().GetFileChunk(fname.c_str(), fileOffset, sbcFirmwareChunk, bytesRead, fileLength))
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
						memcpy(msgp->data, sbcFirmwareChunk + bytesSent, lengthToSend);
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
							bytesRead = min<uint32_t>(lreq, MaxFileChunkSize);
							if (!reprap.GetLinuxInterface().GetFileChunk(fname.c_str(), fileOffset, sbcFirmwareChunk, bytesRead, fileLength))
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
static void HandleInputStateChanged(const CanMessageInputChanged& msg, CanAddress src) noexcept
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

#ifndef DUET3_ATE

static GCodeResult EutGetInfo(const CanMessageReturnInfo& msg, const StringRef& reply, uint8_t& extra)
{
	static constexpr uint8_t LastDiagnosticsPart = 2;				// the last diagnostics part is typeDiagnosticsPart0 + 7

	switch (msg.type)
	{
	case CanMessageReturnInfo::typeFirmwareVersion:
	default:
		reply.printf("%s (%s)", VERSION, DATE);
		break;

	case CanMessageReturnInfo::typeBoardName:
		reply.copy(BOARD_NAME);
		break;

	case CanMessageReturnInfo::typeBootloaderName:
		reply.copy("(n/a)");
		break;

	case CanMessageReturnInfo::typeM408:
		// For now we ignore the parameter and always return the same set of info
		// This command is currently only used by the ATE, which needs the board type and the voltages
		reply.copy("{\"firmwareElectronics\":\"Duet 3 ");
		reply.cat(BOARD_NAME);
		reply.cat("\"");
#if HAS_VOLTAGE_MONITOR
		{
			const MinMaxCurrent voltages = reprap.GetPlatform().GetPowerVoltages();
			reply.catf(",\"vin\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}",
					(double)voltages.min, (double)voltages.current, (double)voltages.max);
		}
#endif
#if HAS_12V_MONITOR
		{
			const MinMaxCurrent voltages = reprap.GetPlatform().GetV12Voltages();
			reply.catf(",\"v12\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}",
					(double)voltages.min, (double)voltages.current, (double)voltages.max);
		}
#endif
		reply.cat('}');
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0:
		extra = LastDiagnosticsPart;
		reply.lcatf("%s (%s)", VERSION, DATE);
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 1:
		extra = LastDiagnosticsPart;
		for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
		{

			reply.lcatf("Driver %u: position %" PRIi32 ", %.1f steps/mm"
#if HAS_SMART_DRIVERS
				", "
#endif
				, driver, reprap.GetMove().GetEndPoint(driver), (double)reprap.GetPlatform().DriveStepsPerUnit(driver));
#if HAS_SMART_DRIVERS
			SmartDrivers::AppendDriverStatus(driver, reply);
#endif
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 2:
		extra = LastDiagnosticsPart;
		{
#if HAS_VOLTAGE_MONITOR && HAS_12V_MONITOR
			reply.catf("VIN: %.1fV, V12: %.1fV\n", (double)reprap.GetPlatform().GetCurrentPowerVoltage(), (double)reprap.GetPlatform().GetCurrentV12Voltage());
#elif HAS_VOLTAGE_MONITOR
			reply.catf("VIN: %.1fV\n", (double)reprap.GetPlatform().GetCurrentPowerVoltage());
#elif HAS_12V_MONITOR
			reply.catf("V12: %.1fV\n", (double)reprap.GetPlatform().GetCurrentV12Voltage());
#endif
#if HAS_CPU_TEMP_SENSOR
			const MinMaxCurrent temps = reprap.GetPlatform().GetMcuTemperatures();
			reply.catf("MCU temperature: min %.1fC, current %.1fC, max %.1fC\n", (double)temps.min, (double)temps.current, (double)temps.max);
#endif
		}
		break;
	}
	return GCodeResult::ok;
}

#endif

// Process a received broadcast or request message. Don't free the message buffer
void CommandProcessor::ProcessReceivedMessage(CanMessageBuffer *buf) noexcept
{
	if (buf->id.Src() != CanInterface::GetCanAddress())								// I don't think we should receive our own broadcasts, but in case we do...
	{
		const CanMessageType id = buf->id.MsgType();
#ifndef DUET3_ATE
		if (CanInterface::InEutMode())
		{
			String<StringLength500> reply;
			const StringRef& replyRef = reply.GetRef();
			GCodeResult rslt;
			CanRequestId requestId;
			uint8_t extra = 0;

			switch (id)
			{
			case CanMessageType::movement:
				reprap.GetMove().AddMoveFromRemote(buf->msg.move);
				return;							// no reply needed

			case CanMessageType::returnInfo:
				requestId = buf->msg.getInfo.requestId;
				rslt = EutGetInfo(buf->msg.getInfo, replyRef, extra);
				break;

			case CanMessageType::m308New:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetHeat().EutProcessM308(buf->msg.generic, replyRef);
				break;

			case CanMessageType::m950Gpio:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetPlatform().EutHandleM950Gpio(buf->msg.generic, replyRef);
				break;

			case CanMessageType::writeGpio:
				requestId = buf->msg.writeGpio.requestId;
				rslt = reprap.GetPlatform().EutHandleGpioWrite(buf->msg.writeGpio, replyRef);
				break;

			case CanMessageType::setMotorCurrents:
				requestId = buf->msg.multipleDrivesRequestFloat.requestId;
				rslt = reprap.GetPlatform().EutSetMotorCurrents(buf->msg.multipleDrivesRequestFloat, buf->dataLength, replyRef);
				break;

			case CanMessageType::setStepsPerMmAndMicrostepping:
				requestId = buf->msg.multipleDrivesStepsPerUnitAndMicrostepping.requestId;
				rslt = reprap.GetPlatform().EutSetStepsPerMmAndMicrostepping(buf->msg.multipleDrivesStepsPerUnitAndMicrostepping, buf->dataLength, replyRef);
				break;

			case CanMessageType::setDriverStates:
				requestId = buf->msg.multipleDrivesRequestUint16.requestId;
				rslt = reprap.GetPlatform().EutHandleSetDriverStates(buf->msg.multipleDrivesRequestDriverState, replyRef);
				break;

			case CanMessageType::createInputMonitor:
				requestId = buf->msg.createInputMonitor.requestId;
				rslt = InputMonitor::EutCreate(buf->msg.createInputMonitor, buf->dataLength, replyRef, extra);
				break;

			case CanMessageType::changeInputMonitor:
				requestId = buf->msg.changeInputMonitor.requestId;
				rslt = InputMonitor::EutChange(buf->msg.changeInputMonitor, replyRef, extra);
				break;

			case CanMessageType::readInputsRequest:
				// This one has its own reply message type
				InputMonitor::EutReadInputs(buf);
				CanInterface::SendResponseNoFree(buf);
				return;

			default:
				if (reprap.Debug(moduleCan))
				{
					buf->DebugPrint("Rec: ");
				}
				break;
			}

			// Re-use the message buffer to send a standard reply
			const CanAddress srcAddress = buf->id.Src();
			CanMessageStandardReply *msg = buf->SetupResponseMessage<CanMessageStandardReply>(requestId, CanInterface::GetCanAddress(), srcAddress);
			msg->resultCode = (uint16_t)rslt;
			msg->extra = extra;
			const size_t totalLength = reply.strlen();
			size_t lengthDone = 0;
			uint8_t fragmentNumber = 0;
			for (;;)
			{
				const size_t fragmentLength = min<size_t>(totalLength - lengthDone, CanMessageStandardReply::MaxTextLength);
				memcpy(msg->text, reply.c_str() + lengthDone, fragmentLength);
				lengthDone += fragmentLength;
				buf->dataLength = msg->GetActualDataLength(fragmentLength);
				msg->fragmentNumber = fragmentNumber;
				if (lengthDone == totalLength)
				{
					msg->moreFollows = false;
					CanInterface::SendResponseNoFree(buf);
					break;
				}
				msg->moreFollows = true;
				CanInterface::SendResponseNoFree(buf);
				++fragmentNumber;
			}
		}
		else
#endif
		{
			// Handle messages received in normal operation mode
			switch (id)
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

			case CanMessageType::enterTestMode:
				EnterTestMode(buf->msg.enterTestMode);
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
}

#endif

// End
