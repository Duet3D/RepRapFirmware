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
#include <ClosedLoop/ClosedLoop.h>
#include <Movement/Move.h>
#include <FilamentMonitors/FilamentMonitor.h>

#ifndef DUET3_ATE
# include <InputMonitors/InputMonitor.h>
# include <Version.h>
# include <Movement/StepperDrivers/SmartDrivers.h>
#endif

#if SUPPORT_ACCELEROMETERS
# include <Accelerometers/Accelerometers.h>
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <Hardware/NonVolatileMemory.h>

static uint8_t expectedSeq = 0xFF;
#endif

static unsigned int duplicateMotionMessages = 0;
static unsigned int oosMessages1Ahead = 0, oosMessages2Ahead = 0, oosMessages2Behind = 0, oosMessagesOther = 0;

// Append diagnostics relating to bad motion messages
void CommandProcessor::AppendBadMotionStats(const StringRef& reply) noexcept
{
	reply.lcatf("Motion dup %u, oos %u/%u/%u/%u", duplicateMotionMessages, oosMessages1Ahead, oosMessages2Ahead, oosMessages2Behind, oosMessagesOther);
	duplicateMotionMessages = oosMessages1Ahead = oosMessages2Ahead = oosMessages2Behind = oosMessagesOther = 0;
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
		fname.cat((msg.uf2Format) ? ".uf2" : ".bin");

		uint32_t fileOffset = msg.fileOffset, fileLength = 0;
		uint32_t lreq = msg.lengthRequested;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
		// Fetch the firmware file from the local SD card or SBC
		FileStore *_ecv_null const f = reprap.GetPlatform().OpenFile(FIRMWARE_DIRECTORY, fname.c_str(), OpenMode::read);
		if (f != nullptr)
		{
			fileLength = f->Length();
			if (fileOffset >= fileLength)
			{
				CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessageNoRid<CanMessageFirmwareUpdateResponse>(CanInterface::GetCurrentMasterAddress(), src);
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
					CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessageNoRid<CanMessageFirmwareUpdateResponse>(CanInterface::GetCurrentMasterAddress(), src);
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

		if (lreq != 0)			// if we didn't complete the request
		{
			CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessageNoRid<CanMessageFirmwareUpdateResponse>(CanInterface::GetCurrentMasterAddress(), src);
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
		CanMessageFirmwareUpdateResponse * const msgp = buf->SetupResponseMessageNoRid<CanMessageFirmwareUpdateResponse>(CanInterface::GetCurrentMasterAddress(), src);
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
// This is the old version, retained for now for compatibility with expansion boards running older firmware. Remove it in due course.
static void HandleInputStateChangedV1(const CanMessageInputChangedV1& msg, CanAddress src, uint16_t timeStamp) noexcept
{
	bool endstopStatesChanged = false;
	Platform& p = reprap.GetPlatform();
	for (unsigned int i = 0; i < msg.numHandles; ++i)
	{
		const RemoteInputHandle handle(msg.GetEntryHandle(i));
		const bool state = (msg.states & (1u << i)) != 0;
		switch (handle.parts.type)
		{
		case RemoteInputHandle::typeEndstop:
			p.GetEndstops().HandleRemoteEndstopChange(src, handle.parts.major, handle.parts.minor, timeStamp, state);
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeZprobe:
			p.GetEndstops().HandleRemoteZProbeChange(src, handle.parts.major, handle.parts.minor, timeStamp, state, msg.GetEntryReading(i));
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeGpIn:
			p.HandleRemoteGpInChange(src, handle.parts.major, handle.parts.minor, state);
			break;

		case RemoteInputHandle::typeStallEndstop:
			// In this case there should be exactly one handle and the 'reading' is a bitmap of stalled drivers
			p.GetEndstops().HandleStalledRemoteDrivers(src, LocalDriversBitmap((LocalDriversBitmap::BaseType)msg.GetEntryReading(i)), timeStamp);
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

// Handle an input state change message
static void HandleInputStateChangedV2(const CanMessageInputChangedV2& msg, CanAddress src) noexcept
{
	bool endstopStatesChanged = false;
	Platform& p = reprap.GetPlatform();
	for (unsigned int i = 0; i < msg.numHandles; ++i)
	{
		const RemoteInputHandle handle(msg.GetEntryHandle(i));
		const bool state = (msg.states & (1u << i)) != 0;
		switch (handle.parts.type)
		{
		case RemoteInputHandle::typeEndstop:
			p.GetEndstops().HandleRemoteEndstopChange(src, handle.parts.major, handle.parts.minor, msg.GetWhen(i), state);
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeZprobe:
			p.GetEndstops().HandleRemoteZProbeChange(src, handle.parts.major, handle.parts.minor, msg.GetWhen(i), msg.GetEntryReading(i), state);
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeGpIn:
			p.HandleRemoteGpInChange(src, handle.parts.major, handle.parts.minor, state);
			break;

		case RemoteInputHandle::typeStallEndstop:
			// In this case there should be exactly one handle and the 'reading' is a bitmap of stalled drivers
			p.GetEndstops().HandleStalledRemoteDrivers(src, LocalDriversBitmap((LocalDriversBitmap::BaseType)msg.GetEntryReading(i)), msg.GetWhen(i));
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

static GCodeResult EutGetInfo(const CanMessageReturnInfo& msg, const StringRef& reply, uint8_t& extra) noexcept
{
	switch (msg.type)
	{
	case CanMessageReturnInfo::typeFirmwareVersion:
		reply.printf("%s firmware version " VERSION " (%s%s)", reprap.GetPlatform().GetElectronicsString(), DateText, TimeSuffix);
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

	case CanMessageReturnInfo::typeBoardUniqueId:
		reprap.GetPlatform().GetUniqueId().AppendCharsToString(reply);
		break;

	default:
		if (msg.type >= CanMessageReturnInfo::typeDiagnosticsPart0 && msg.type < CanMessageReturnInfo::typeDiagnosticsPart0 + reprap.GetNumberOfDiagnosticParts())
		{
			reply.Clear();
			reprap.GetDiagnosticsPart(msg.type - CanMessageReturnInfo::typeDiagnosticsPart0, reply);
			extra = reprap.GetNumberOfDiagnosticParts() - 1;
		}
		else
		{
			reply.copy("unknown GetInfo subfunction");
			return GCodeResult::error;
		}
		break;
	}
	return GCodeResult::ok;
}

static GCodeResult InitiateFirmwareUpdate(const CanMessageUpdateYourFirmware& msg, const StringRef& reply) noexcept
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
	if (buf->id.Src() != CanInterface::GetCanAddress())				// I don't think we should receive our own messages, but in case we do...
	{
		const CanMessageType id = buf->id.MsgType();
		if (   buf->id.Dst() != CanId::BroadcastAddress				// ignore broadcast messages e.g. temperature reports
			&& id != CanMessageType::fansReport						// don't flash whenever we receive a regular status message
			&& id != CanMessageType::heatersStatusReport
			&& id != CanMessageType::boardStatusReportV0
			&& id != CanMessageType::boardStatusReportV1
			&& id != CanMessageType::driversStatusReport
			&& id != CanMessageType::filamentMonitorsStatusReportV2
		   )
		{
			reprap.GetPlatform().OnProcessingCanMessage();
		}

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
				// Check for duplicate and out-of-sequence message
				// We can get out-of-sequence messages because of a bug in the CAN hardware; so use only the sequence number to detect duplicates
				{
					const uint8_t seq = buf->msg.moveLinearShaped.seq;
					if (((seq + 1) & CanMessageMovementLinearShaped::SeqMask) == expectedSeq)
					{
						++duplicateMotionMessages;
						return;
					}

					if (seq != expectedSeq && expectedSeq != 0xFF)
					{
						switch ((seq - expectedSeq) & CanMessageMovementLinearShaped::SeqMask)
						{
						case 1:
							++oosMessages1Ahead;
							break;

						case 2:
							++oosMessages2Ahead;
							break;

						case 0x7E:
							++oosMessages2Behind;
							break;

						default:
							++oosMessagesOther;
							break;
						}
					}

					expectedSeq = (seq + 1) & CanMessageMovementLinearShaped::SeqMask;
				}

				if (StepTimer::IsSynced())
				{
					reprap.GetMove().AddMoveFromRemote(buf->msg.moveLinearShaped);
				}
				else
				{
					CanInterface::LogIgnoredMovementMessage();
				}
				return;							// no reply needed

			case CanMessageType::stopMovement:
				reprap.GetMove().StopDriversFromRemote(buf->msg.stopMovement.whichDrives);
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

			case CanMessageType::heaterFeedForwardV1:
				requestId = CanRequestIdNoReplyNeeded;
				rslt = reprap.GetHeat().ApplyFeedForward(buf->msg.heaterFeedForwardV1, replyRef);
				break;

			case CanMessageType::heaterModelV3:
				requestId = buf->msg.heaterModelV3.requestId;
				rslt = reprap.GetHeat().ProcessM307(buf->msg.heaterModelV3, replyRef);
				break;

			case CanMessageType::setHeaterTemperatureV1:
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

			case CanMessageType::setDefaultHeaterModel:
				reprap.GetHeat().SetDefaultHeaterModel(*buf);
				CanInterface::SendResponseNoFree(buf);
				return;

			case CanMessageType::m308V1:
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
				rslt = reprap.GetMove().EutSetMotorCurrents(buf->msg.multipleDrivesRequestFloat, buf->dataLength, replyRef);
				break;

			case CanMessageType::setStepsPerMmAndMicrostepping:
				requestId = buf->msg.multipleDrivesStepsPerUnitAndMicrostepping.requestId;
				rslt = reprap.GetMove().EutSetStepsPerMmAndMicrostepping(buf->msg.multipleDrivesStepsPerUnitAndMicrostepping, buf->dataLength, replyRef);
				break;

			case CanMessageType::setDriverStates:
				requestId = buf->msg.multipleDrivesRequestUint16.requestId;
				rslt = reprap.GetMove().EutHandleSetDriverStates(buf->msg.multipleDrivesRequestDriverState, replyRef);
				break;

			case CanMessageType::m915:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetMove().EutProcessM915(buf->msg.generic, replyRef);
				break;

			case CanMessageType::setPressureAdvanceV1:
				requestId = buf->msg.multipleDrivesRequestFloat.requestId;
				rslt = reprap.GetMove().EutSetRemotePressureAdvanceV1(buf->msg.multipleDrivesRequestFloat, buf->dataLength, replyRef);
				break;

			case CanMessageType::setPressureAdvanceV2:
				requestId = buf->msg.multipleDrivesRequestFloat.requestId;
				rslt = reprap.GetMove().EutSetRemotePressureAdvanceV2(buf->msg.multipleDrivesRequestPressureAdvance, buf->dataLength, replyRef);
				break;

			case CanMessageType::setInputShapingV1:
				requestId = buf->msg.setInputShapingV1.requestId;
				rslt = reprap.GetMove().EutSetInputShaping(buf->msg.setInputShapingV1, buf->dataLength, replyRef);
				break;

			case CanMessageType::m569:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetMove().EutProcessM569(buf->msg.generic, replyRef);
				break;

			case CanMessageType::m569p2:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetMove().EutProcessM569Point2(buf->msg.generic, replyRef);
				break;

			case CanMessageType::m569p7:
				requestId = buf->msg.generic.requestId;
				rslt = reprap.GetMove().EutProcessM569Point7(buf->msg.generic, replyRef);
				break;

			case CanMessageType::createInputMonitorV1:
				requestId = buf->msg.createInputMonitorV1.requestId;
				rslt = InputMonitor::Create(buf->msg.createInputMonitorV1, buf->dataLength, replyRef, extra);
				break;

			case CanMessageType::changeInputMonitorV1:
				requestId = buf->msg.changeInputMonitorV1.requestId;
				rslt = InputMonitor::Change(buf->msg.changeInputMonitorV1, replyRef, extra);
				break;

			case CanMessageType::enableStallEndstop:
				requestId = buf->msg.enableStallEndstop.requestId;
				rslt = reprap.GetMove().SetStallEndstopReporting(buf->msg.enableStallEndstop, replyRef);
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

			case CanMessageType::m655:
				requestId = buf->msg.generic.requestId;
				reply.copy("not supported by this board");
				rslt = GCodeResult::error;
				break;

			case CanMessageType::diagnosticTest:
				requestId = buf->msg.diagnosticTest.requestId;
				//TODO support this function
				rslt = GCodeResult::errorNotSupported;
				break;

			case CanMessageType::m111:
				requestId = buf->msg.diagnosticTest.requestId;
				rslt = reprap.ProcessRemoteM111(buf->msg.generic, replyRef);
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
				break;
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
			case CanMessageType::inputStateChangedV1:
				//TODO we should preferably handle this one using a separate high-priority queue or buffer
				HandleInputStateChangedV1(buf->msg.inputChangedV1, buf->id.Src(), buf->timeStamp);
				break;

			case CanMessageType::inputStateChangedV2:
				//TODO we should preferably handle this one using a separate high-priority queue or buffer
				HandleInputStateChangedV2(buf->msg.inputChangedV2, buf->id.Src());
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

			case CanMessageType::boardStatusReportV0:
			case CanMessageType::boardStatusReportV1:
				reprap.GetExpansion().ProcessBoardStatusReport(buf);
				break;

			case CanMessageType::heaterTuningReport:
				reprap.GetHeat().ProcessRemoteHeaterTuningReport(buf->id.Src(), buf->msg.heaterTuningReport);
				break;

			case CanMessageType::fansReport:
				reprap.GetFansManager().ProcessRemoteFanRpms(buf->id.Src(), buf->msg.fansReport);
				break;

			case CanMessageType::announceV0:
				reprap.GetExpansion().ProcessAnnouncement(buf, false);
				break;

			case CanMessageType::announceV1:
				reprap.GetExpansion().ProcessAnnouncement(buf, true);
				break;

			case CanMessageType::filamentMonitorsStatusReportV2:
				FilamentMonitor::UpdateRemoteFilamentStatus(buf->id.Src(), buf->msg.filamentMonitorsStatusV2);
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
