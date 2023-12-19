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
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Platform/Event.h>
#include <Heating/Heat.h>
#include "ExpansionManager.h"
# include <ClosedLoop/ClosedLoop.h>

#ifndef DUET3_ATE
# include <Movement/Move.h>
# include <InputMonitors/InputMonitor.h>
# include <Version.h>

# if SUPPORT_TMC2660
#  include <Movement/StepperDrivers/TMC2660.h>
# endif
# if SUPPORT_TMC22xx
#  include <Movement/StepperDrivers/TMC22xx.h>
# endif
# if SUPPORT_TMC51xx
#  include <Movement/StepperDrivers/TMC51xx.h>
# endif
#endif

#if SUPPORT_ACCELEROMETERS
# include <Accelerometers/Accelerometers.h>
#endif

#if HAS_SBC_INTERFACE
# include "SBC/SbcInterface.h"

constexpr size_t MaxFileChunkSize = 448;	// Maximum size of file chunks for reading files from the SBC. Should be a multiple of sizeof(CanMessageFirmwareUpdateResponse::data) for best CAN performance
char sbcFirmwareChunk[MaxFileChunkSize];
#endif

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
		fname.cat((msg.uf2Format) ? ".uf2" : ".bin");

		uint32_t fileOffset = msg.fileOffset, fileLength = 0;
		uint32_t lreq = msg.lengthRequested;

#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			// Fetch the firmware file from the SBC
			uint32_t bytesRead = min<uint32_t>(lreq, MaxFileChunkSize);
			if (reprap.GetSbcInterface().GetFileChunk(fname.c_str(), fileOffset, sbcFirmwareChunk, bytesRead, fileLength))
			{
				if (fileOffset >= fileLength)
				{
					CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanInterface::GetCurrentMasterAddress(), src);
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
						CanMessageFirmwareUpdateResponse * msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanInterface::GetCurrentMasterAddress(), src);
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
							if (!reprap.GetSbcInterface().GetFileChunk(fname.c_str(), fileOffset, sbcFirmwareChunk, bytesRead, fileLength))
							{
								msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanInterface::GetCurrentMasterAddress(), src);
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
				CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanInterface::GetCurrentMasterAddress(), src);
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
			FileStore * const f = reprap.GetPlatform().OpenFile(FIRMWARE_DIRECTORY, fname.c_str(), OpenMode::read);
			if (f != nullptr)
			{
				fileLength = f->Length();
				if (fileOffset >= fileLength)
				{
					CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanInterface::GetCurrentMasterAddress(), src);
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
						CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanInterface::GetCurrentMasterAddress(), src);
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
			CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanInterface::GetCurrentMasterAddress(), src);
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
		CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessage<CanMessageFirmwareUpdateResponse>(0, CanInterface::GetCurrentMasterAddress(), src);
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
		reprap.GetMove().OnEndstopOrZProbeStatesChanged();
	}
}

#if SUPPORT_REMOTE_COMMANDS

static GCodeResult EutGetInfo(const CanMessageReturnInfo& msg, const StringRef& reply, uint8_t& extra)
{
	static constexpr uint8_t LastDiagnosticsPart = 9;				// the last diagnostics part is typeDiagnosticsPart0 + 9

	switch (msg.type)
	{
	case CanMessageReturnInfo::typeFirmwareVersion:
	default:
		// This must be formatted in a specific way for the ATE, starting with the electronics string
		reply.printf("%s firmware version " VERSION " (%s%s)", reprap.GetPlatform().GetElectronicsString(), DATE, TIME_SUFFIX);
		break;

	case CanMessageReturnInfo::typeBoardName:
		reply.copy(BOARD_SHORT_NAME);
#if BOARD_USES_UF2_BINARY
		extra |= 0x01;
#endif
		break;

	case CanMessageReturnInfo::typeBootloaderName:
		reply.copy("(n/a)");
		break;

	case CanMessageReturnInfo::typeM408:
		// For now we ignore the parameter and always return the same set of info
		// This command is only used by the old ATE, which needs the board type and the voltages
		reply.printf("{\"firmwareElectronics\":\"Duet 3 %.0s\"", BOARD_NAME);
#if HAS_VOLTAGE_MONITOR
		{
			const MinCurMax voltages = reprap.GetPlatform().GetPowerVoltages();
			reply.catf(",\"vin\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}",
					(double)voltages.minimum, (double)voltages.current, (double)voltages.maximum);
		}
#endif
#if HAS_12V_MONITOR
		{
			const MinCurMax voltages = reprap.GetPlatform().GetV12Voltages();
			reply.catf(",\"v12\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}",
					(double)voltages.minimum, (double)voltages.current, (double)voltages.maximum);
		}
#endif
		reply.cat('}');
		break;

	case CanMessageReturnInfo::typeBoardUniqueId:
		reprap.GetPlatform().GetUniqueId().AppendCharsToString(reply);
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0:
		extra = LastDiagnosticsPart;
		// Report the firmware version and board type
		reply.lcatf("%s version %s (%s%s) running on %s", FIRMWARE_NAME, VERSION, DATE, TIME_SUFFIX, reprap.GetPlatform().GetElectronicsString());
		// Show the up time and reason for the last reset
		{
			const uint32_t now = (uint32_t)(millis64()/1000u);		// get up time in seconds
			reply.lcatf("Last reset %02d:%02d:%02d ago, cause: %s", (unsigned int)(now/3600), (unsigned int)((now % 3600)/60), (unsigned int)(now % 60), Platform::GetResetReasonText());
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 1:
	case CanMessageReturnInfo::typeDiagnosticsPart0 + 2:
	case CanMessageReturnInfo::typeDiagnosticsPart0 + 3:
	case CanMessageReturnInfo::typeDiagnosticsPart0 + 4:
	case CanMessageReturnInfo::typeDiagnosticsPart0 + 5:
	case CanMessageReturnInfo::typeDiagnosticsPart0 + 6:
	case CanMessageReturnInfo::typeDiagnosticsPart0 + 7:
		// We send each driver status in a separate message because we can't fit more than three in one reply
		extra = LastDiagnosticsPart;
		{
			const size_t driver = msg.type - (CanMessageReturnInfo::typeDiagnosticsPart0 + 1);
			if (driver < NumDirectDrivers)			// we have up to 7 drivers on the Duet 3 Mini but only 6 on the 6HC and 6XD
			{
				reply.lcatf("Driver %u: %.1f steps/mm"
#if HAS_SMART_DRIVERS
					","
#endif
					, driver, (double)reprap.GetPlatform().DriveStepsPerUnit(driver));
#if HAS_SMART_DRIVERS
				SmartDrivers::AppendDriverStatus(driver, reply);
#endif
			}
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 8:
		extra = LastDiagnosticsPart;
		{
#if HAS_VOLTAGE_MONITOR && HAS_12V_MONITOR
			reply.catf("VIN: %.1fV, V12: %.1fV", (double)reprap.GetPlatform().GetCurrentPowerVoltage(), (double)reprap.GetPlatform().GetCurrentV12Voltage());
#elif HAS_VOLTAGE_MONITOR
			reply.catf("VIN: %.1fV", (double)reprap.GetPlatform().GetCurrentPowerVoltage());
#elif HAS_12V_MONITOR
			reply.catf("V12: %.1fV", (double)reprap.GetPlatform().GetCurrentV12Voltage());
#endif
#if HAS_CPU_TEMP_SENSOR
			const MinCurMax temps = reprap.GetPlatform().GetMcuTemperatures();
			reply.catf(
# if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
				", "
# endif
				"MCU temperature: min %.1fC, current %.1fC, max %.1fC", (double)temps.minimum, (double)temps.current, (double)temps.maximum);
#endif
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 9:
		extra = LastDiagnosticsPart;
		StepTimer::Diagnostics(reply);
#if 0	// We don't currently support accelerometers on main boards used as expansion boards
//#if SUPPORT_ACCELEROMETERS
		Accelerometers::Diagnostics(reply);
#endif
		FilamentMonitor::GetDiagnostics(reply);
		break;
	}
	return GCodeResult::ok;
}

static GCodeResult InitiateFirmwareUpdate(const CanMessageUpdateYourFirmware& msg, const StringRef& reply)
{
	if (msg.boardId != CanInterface::GetCanAddress() || msg.invertedBoardId != (uint8_t)~CanInterface::GetCanAddress() || (msg.module != 0 && msg.module != 3))
	{
		reply.printf("Invalid firmware update command received");
		return GCodeResult::error;
	}

	if (msg.module == 0)
	{
		if (!reprap.GetPlatform().FileExists(FIRMWARE_DIRECTORY, IAP_CAN_LOADER_FILE))
		{
			reply.printf("In-application programming binary \"%s\" not found on board %u", FIRMWARE_DIRECTORY IAP_CAN_LOADER_FILE, CanInterface::GetCanAddress());
			return GCodeResult::error;
		}
		reply.printf("Board %u starting firmware update", CanInterface::GetCanAddress());
		reprap.ScheduleFirmwareUpdateOverCan();
		return GCodeResult::ok;
	}
	reply.copy("unknown firmware module number");
	return GCodeResult::error;
}

#endif	// SUPPORT_REMOTE_COMMANDS

// Process a received broadcast or request message. Don't free the message buffer
void CommandProcessor::ProcessReceivedMessage(CanMessageBuffer *buf) noexcept
{
	if (buf->id.Src() != CanInterface::GetCanAddress())								// I don't think we should receive our own broadcasts, but in case we do...
	{
		if (   buf->id.Dst() != CanId::BroadcastAddress
			&& buf->id.MsgType() != CanMessageType::fanTachoReport					// don't flash whenever we receive a regular status message
			&& buf->id.MsgType() != CanMessageType::heatersStatusReport
			&& buf->id.MsgType() != CanMessageType::boardStatusReport
			&& buf->id.MsgType() != CanMessageType::driversStatusReport
		   )
		{
			reprap.GetPlatform().OnProcessingCanMessage();
		}

		const CanMessageType id = buf->id.MsgType();
#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			String<StringLength500> reply;
			const StringRef& replyRef = reply.GetRef();
			GCodeResult rslt;
			CanRequestId requestId;
			uint8_t extra = 0;

			switch (id)
			{
			case CanMessageType::timeSync:
				StepTimer::ProcessTimeSyncMessage(buf->msg.sync, buf->dataLength, buf->timeStamp);
				return;							// no reply needed

			case CanMessageType::emergencyStop:
				reprap.EmergencyStop();
				reprap.ScheduleReset();
				return;							// no reply needed

			case CanMessageType::movementLinearShaped:
				//TODO check seq
				reprap.GetMove().AddMoveFromRemote(buf->msg.moveLinearShaped);
				return;							// no reply needed

			case CanMessageType::stopMovement:
				reprap.GetMove().StopDrivers(buf->msg.stopMovement.whichDrives);
				return;							// no reply needed

			case CanMessageType::revertPosition:
				reprap.GetMove().RevertPosition(buf->msg.revertPosition);
				return;							// no reply needed

			case CanMessageType::acknowledgeAnnounce:
				CanInterface::MainBoardAcknowledgedAnnounce();
				return;

			case CanMessageType::updateFirmware:
				requestId = buf->msg.updateYourFirmware.requestId;
				rslt = InitiateFirmwareUpdate(buf->msg.updateYourFirmware, replyRef);
				break;

			case CanMessageType::reset:
				requestId = buf->msg.reset.requestId;
				reply.printf("Board %u resetting", CanInterface::GetCanAddress());
				reprap.EmergencyStop();
				reprap.ScheduleReset();
				rslt = GCodeResult::ok;
				break;

			case CanMessageType::returnInfo:
				requestId = buf->msg.getInfo.requestId;
				rslt = EutGetInfo(buf->msg.getInfo, replyRef, extra);
				break;

			// Heater commands
			case CanMessageType::m950Heater:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetHeat().ConfigureHeater(buf->msg.generic, replyRef);
				break;

			case CanMessageType::heaterFeedForward:
				requestId = buf->msg.heaterFeedForward.requestId;
				rslt = reprap.GetHeat().FeedForward(buf->msg.heaterFeedForward, replyRef);
				break;

			case CanMessageType::heaterModelNewNew:
				requestId = buf->msg.heaterModelNewNew.requestId;
				rslt = reprap.GetHeat().ProcessM307New(buf->msg.heaterModelNewNew, replyRef);
				break;

			case CanMessageType::setHeaterTemperature:
				requestId = buf->msg.setTemp.requestId;
				rslt = reprap.GetHeat().SetTemperature(buf->msg.setTemp, replyRef);
				break;

			case CanMessageType::heaterTuningCommand:
				requestId = buf->msg.heaterTuningCommand.requestId;
				rslt = reprap.GetHeat().TuningCommand(buf->msg.heaterTuningCommand, replyRef);
				break;

			case CanMessageType::setHeaterFaultDetection:
				requestId = buf->msg.setHeaterFaultDetection.requestId;
				rslt = reprap.GetHeat().SetFaultDetection(buf->msg.setHeaterFaultDetection, replyRef);
				break;

			case CanMessageType::setHeaterMonitors:
				requestId = buf->msg.setHeaterMonitors.requestId;
				rslt = reprap.GetHeat().SetHeaterMonitors(buf->msg.setHeaterMonitors, replyRef);
				break;

			case CanMessageType::m308New:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetHeat().ProcessM308(buf->msg.generic, replyRef);
				break;

			// Fan commands
			case CanMessageType::m950Fan:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetFansManager().ConfigureFanPort(buf->msg.generic, replyRef);
				break;

			case CanMessageType::fanParameters:
				requestId = buf->msg.fanParameters.requestId;
				rslt = reprap.GetFansManager().ConfigureFan(buf->msg.fanParameters, replyRef);
				break;

			case CanMessageType::setFanSpeed:
				requestId = buf->msg.setFanSpeed.requestId;
				rslt = reprap.GetFansManager().SetFanSpeed(buf->msg.setFanSpeed, replyRef);
				break;

			// GPIO commands
			case CanMessageType::m950Gpio:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetPlatform().EutHandleM950Gpio(buf->msg.generic, replyRef);
				break;

			case CanMessageType::writeGpio:
				requestId = buf->msg.writeGpio.requestId;
				rslt = reprap.GetPlatform().EutHandleGpioWrite(buf->msg.writeGpio, replyRef);
				break;

			// LED strip commands
			case CanMessageType::m950Led:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetPlatform().GetLedStripManager().HandleM950Led(buf->msg.generic, replyRef, extra);
				break;

			case CanMessageType::writeLedStrip:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetPlatform().GetLedStripManager().HandleLedSetColours(buf->msg.generic, replyRef);
				break;

			// Driver commands
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

			case CanMessageType::m915:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetPlatform().EutProcessM915(buf->msg.generic, replyRef);
				break;

			case CanMessageType::setPressureAdvance:
				requestId = buf->msg.multipleDrivesRequestFloat.requestId;
				rslt = reprap.GetMove().EutSetRemotePressureAdvance(buf->msg.multipleDrivesRequestFloat, buf->dataLength, replyRef);
				break;

			case CanMessageType::setInputShaping:
				requestId = buf->msg.setInputShaping.requestId;
				rslt = reprap.GetMove().EutSetInputShaping(buf->msg.setInputShaping, buf->dataLength, replyRef);
				break;

			case CanMessageType::m569:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetPlatform().EutProcessM569(buf->msg.generic, replyRef);
				break;

			case CanMessageType::m569p2:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetPlatform().EutProcessM569Point2(buf->msg.generic, replyRef);
				break;

			case CanMessageType::m569p7:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetPlatform().EutProcessM569Point7(buf->msg.generic, replyRef);
				break;

			case CanMessageType::createInputMonitorNew:
				requestId = buf->msg.createInputMonitorNew.requestId;
				rslt = InputMonitor::Create(buf->msg.createInputMonitorNew, buf->dataLength, replyRef, extra);
				break;

			case CanMessageType::changeInputMonitorNew:
				requestId = buf->msg.changeInputMonitorNew.requestId;
				rslt = InputMonitor::Change(buf->msg.changeInputMonitorNew, replyRef, extra);
				break;

			case CanMessageType::readInputsRequest:
				// This one has its own reply message type
				InputMonitor::ReadInputs(buf);
				CanInterface::SendResponseNoFree(buf);
				return;

			// Filament monitor commands
			case CanMessageType::createFilamentMonitor:
				requestId = buf->msg.createFilamentMonitor.requestId;
				rslt = FilamentMonitor::Create(buf->msg.createFilamentMonitor, replyRef);
				break;

			case CanMessageType::deleteFilamentMonitor:
				requestId = buf->msg.deleteFilamentMonitor.requestId;
				rslt = FilamentMonitor::Delete(buf->msg.deleteFilamentMonitor, replyRef);
				break;

			case CanMessageType::configureFilamentMonitor:
				requestId = buf->msg.generic.requestId;
				rslt = FilamentMonitor::Configure(buf->msg.generic, replyRef);
				break;

			default:
				// We received a message type that we don't recognise. If it's a broadcast, ignore it. If it's addressed to us, send a reply.
				if (buf->id.Src() != CanInterface::GetCanAddress())
				{
					return;
				}
				requestId = CanRequestIdAcceptAlways;
				reply.printf("Board %u received unknown msg type %u", CanInterface::GetCanAddress(), (unsigned int)buf->id.MsgType());
				rslt = GCodeResult::error;
			}

			if (requestId != CanRequestIdNoReplyNeeded)				// if a reply is needed
			{
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

			case CanMessageType::driversStatusReport:
				reprap.GetExpansion().ProcessDriveStatusReport(buf);
				break;

			case CanMessageType::boardStatusReport:
				reprap.GetExpansion().ProcessBoardStatusReport(buf);
				break;

			case CanMessageType::heaterTuningReport:
				reprap.GetHeat().ProcessRemoteHeaterTuningReport(buf->id.Src(), buf->msg.heaterTuningReport);
				break;

			case CanMessageType::fansReport:
				reprap.GetFansManager().ProcessRemoteFanRpms(buf->id.Src(), buf->msg.fansReport);
				break;

			case CanMessageType::announceOld:
				reprap.GetExpansion().ProcessAnnouncement(buf, false);
				break;

			case CanMessageType::announceNew:
				reprap.GetExpansion().ProcessAnnouncement(buf, true);
				break;

			case CanMessageType::filamentMonitorsStatusReportNew:
				FilamentMonitor::UpdateRemoteFilamentStatus(buf->id.Src(), buf->msg.filamentMonitorsStatusNew);
				break;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
			case CanMessageType::closedLoopData:
				ClosedLoop::ProcessReceivedData(buf->id.Src(), buf->msg.closedLoopData, buf->dataLength);
				break;
#endif

			case CanMessageType::event:
				Event::Add(buf->msg.event, buf->id.Src(), buf->dataLength);
				break;

			case CanMessageType::debugText:
				reprap.GetPlatform().MessageF(GenericMessage, "Debug from %u: %.*s\n", buf->id.Src(), buf->msg.debugText.GetMaxTextLength(buf->dataLength), buf->msg.debugText.text);
				break;

#if SUPPORT_ACCELEROMETERS
			case CanMessageType::accelerometerData:
				Accelerometers::ProcessReceivedData(buf->id.Src(), buf->msg.accelerometerData, buf->dataLength);
				break;
#endif

#if SUPPORT_REMOTE_COMMANDS
			case CanMessageType::enterTestMode:
				if (buf->msg.enterTestMode.passwd == CanMessageEnterTestMode::Passwd)
				{
					const CanAddress newAddress = buf->msg.enterTestMode.address;

					// Send a standard response before we switch
					const CanAddress srcAddress = buf->id.Src();
					const CanRequestId requestId = buf->msg.enterTestMode.requestId;

					CanMessageStandardReply * const msg = buf->SetupResponseMessage<CanMessageStandardReply>(requestId, CanInterface::GetCanAddress(), srcAddress);
					msg->resultCode = (uint16_t)GCodeResult::ok;
					msg->extra = 0;
					msg->text[0] = 0;
					buf->dataLength = msg->GetActualDataLength(0);
					msg->fragmentNumber = 0;
					msg->moreFollows = false;
					CanInterface::SendResponseNoFree(buf);

					delay(25);							// allow time for the response to be sent before we re-initialise CAN
					CanInterface::SwitchToExpansionMode(newAddress, true);
				}
				break;
#endif

			default:
				if (reprap.Debug(Module::CAN))
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
