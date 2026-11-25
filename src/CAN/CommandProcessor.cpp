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
static bool isProgrammed = false;					// this is set to true after we have received a configuration command
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
// 'buf' holds the request
// 'buf->useBrs' is true if the request used bit rate switching, in which case we use it for the response too
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

		// Some bootloaders inadvertently set the BRS bit in the request even though bootloaders shouldn't use BRS.
		// Clear the BRS bit to ensure that the response is sent without using BRS.
		buf->useBrs = 0;

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
static void HandleInputStateChangedV1(const CanMessageInputChangedV1& msg, CanAddress src) noexcept
{
	bool endstopStatesChanged = false;
	Platform& p = reprap.GetPlatform();
	for (unsigned int i = 0; i < msg.numHandles; i++)
	{
		const RemoteInputHandle handle(msg.GetEntryHandle(i));
		const bool state = (msg.states & (1u << i)) != 0;
		switch (handle.parts.type)
		{
		case RemoteInputHandle::typeEndstop:
			p.GetEndstops().HandleRemoteEndstopChange(src, handle.parts.major, handle.parts.minor, StepTimer::GetTimerTicks(), state);
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeZprobe:
			p.GetEndstops().HandleRemoteZProbeChange(src, handle.parts.major, handle.parts.minor, StepTimer::GetTimerTicks(), state, msg.GetEntryReading(i));
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeGpIn:
			p.HandleRemoteGpInChange(src, handle.parts.major, handle.parts.minor, state);
			endstopStatesChanged = true;					// the port may be bound to an extruder endstop, see the E parameter of M574
			break;

		case RemoteInputHandle::typeStallEndstop:
			// In this case there should be exactly one handle and the 'reading' is a bitmap of stalled drivers
			p.GetEndstops().HandleStalledRemoteDrivers(src, LocalDriversBitmap((LocalDriversBitmap::BaseType)msg.GetEntryReading(i)), StepTimer::GetTimerTicks());
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
	for (unsigned int i = 0; i < msg.numHandles; i++)
	{
		const RemoteInputHandle handle(msg.GetEntryHandle(i));
		const bool state = (msg.states & (1u << i)) != 0;
		switch (handle.parts.type)
		{
		case RemoteInputHandle::typeEndstop:
			p.GetEndstops().HandleRemoteEndstopChange(src, handle.parts.major, handle.parts.minor, CanInterface::Convert16bitReceivedTimeStampTo32bits(msg.GetWhen(i)), state);
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeZprobe:
			p.GetEndstops().HandleRemoteZProbeChange(src, handle.parts.major, handle.parts.minor, CanInterface::Convert16bitReceivedTimeStampTo32bits(msg.GetWhen(i)), state, msg.GetEntryReading(i));
			endstopStatesChanged = true;
			break;

		case RemoteInputHandle::typeGpIn:
			p.HandleRemoteGpInChange(src, handle.parts.major, handle.parts.minor, state);
			endstopStatesChanged = true;					// the port may be bound to an extruder endstop, see the E parameter of M574
			break;

		case RemoteInputHandle::typeStallEndstop:
			// In this case there should be exactly one handle and the 'reading' is a bitmap of stalled drivers
			p.GetEndstops().HandleStalledRemoteDrivers(src, LocalDriversBitmap((LocalDriversBitmap::BaseType)msg.GetEntryReading(i)), CanInterface::Convert16bitReceivedTimeStampTo32bits(msg.GetWhen(i)));
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
		reply.printf("%s firmware version " VERSION " (%s)", reprap.GetPlatform().GetElectronicsString(), DateTimeText);
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

// Declare 'msg' as the given variant of the received message, and reject the message if it is too short to be a valid one
#define GET_VALID_MESSAGE(_variant) \
	const auto& msg = buf->msg._variant; \
	if (!msg.ValidMessageLength(buf->dataLength)) \
	{ \
		badMessage = true; \
		break; \
	}

// Process a received broadcast or request message. Don't free the message buffer
void CommandProcessor::ProcessReceivedMessage(CanMessageBuffer *buf) noexcept
{
	if (buf->id.Src() != CanInterface::GetCanAddress())				// I don't think we should receive our own messages, but in case we do...
	{
		const CanMessageType id = buf->id.MsgType();
		if (   buf->id.Dst() != CanId::BroadcastAddress				// don't flash the LED on broadcast messages e.g. temperature reports and time sync
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
			const bool requestUsedBrs = buf->useBrs;
			bool badMessage = false;

			switch (id)
			{
			case CanMessageType::timeSync:
				{
					GET_VALID_MESSAGE(sync);
					StepTimer::ProcessTimeSyncMessage(msg, buf->dataLength, buf->timeStamp);
					CanInterface::CheckBrs(msg);
				}
				return;							// no reply needed

			case CanMessageType::emergencyStop:
				reprap.EmergencyStop();
				reprap.ScheduleReset();
				return;							// no reply needed

			case CanMessageType::movementLinearShaped:
				// Check for duplicate and out-of-sequence message
				// We can get out-of-sequence messages because of a bug in the CAN hardware; so use only the sequence number to detect duplicates
				{
					GET_VALID_MESSAGE(moveLinearShaped);
					const uint8_t seq = msg.seq;
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
				{
					GET_VALID_MESSAGE(stopMovement);
					reprap.GetMove().StopDriversFromRemote(msg.whichDrives);
				}
				return;							// no reply needed

			case CanMessageType::revertPosition:
				{
					GET_VALID_MESSAGE(revertPosition);
					reprap.GetMove().RevertPosition(msg);
				}
				return;							// no reply needed

			case CanMessageType::acknowledgeAnnounce:
				CanInterface::MainBoardAcknowledgedAnnounce();
				return;

			case CanMessageType::startup:
				if (millis64() > 5000 || isProgrammed)				// if we've only just powered up and the main board hasn't programmed us yet, no need to start up again
				{
					reprap.EmergencyStop();
					reprap.ScheduleReset();
				}
				else
				{
					CanInterface::MainBoardRequestedAnnounce();		// we've recently started up, so no need to reset; but assume that we need to announce ourselves
				}
				return;							// no reply needed

			case CanMessageType::updateFirmware:
				{
					GET_VALID_MESSAGE(updateYourFirmware);
					requestId = msg.requestId;
					rslt = InitiateFirmwareUpdate(msg, replyRef);
				}
				break;

			case CanMessageType::reset:
				{
					GET_VALID_MESSAGE(reset);
					requestId = msg.requestId;
					reply.printf("Board %u resetting", CanInterface::GetCanAddress());
					reprap.EmergencyStop();
					reprap.ScheduleReset();
					rslt = GCodeResult::ok;
				}
				break;

			case CanMessageType::returnInfo:
				{
					GET_VALID_MESSAGE(getInfo);
					requestId = msg.requestId;
					rslt = EutGetInfo(msg, replyRef, extra);
				}
				break;

			// Heater commands
			case CanMessageType::m950Heater:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetHeat().ConfigureHeater(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::heaterFeedForwardV1:
				{
					GET_VALID_MESSAGE(heaterFeedForwardV1);
					requestId = CanRequestIdNoReplyNeeded;
					rslt = reprap.GetHeat().ApplyFeedForward(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::heaterModelV3:
				{
					GET_VALID_MESSAGE(heaterModelV3);
					requestId = msg.requestId;
					rslt = reprap.GetHeat().ProcessM307(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::setHeaterTemperatureV1:
				{
					GET_VALID_MESSAGE(setTemp);
					requestId = msg.requestId;
					rslt = reprap.GetHeat().SetTemperature(msg, replyRef);
				}
				break;

			case CanMessageType::heaterTuningCommand:
				{
					GET_VALID_MESSAGE(heaterTuningCommand);
					requestId = msg.requestId;
					rslt = reprap.GetHeat().TuningCommand(msg, replyRef);
				}
				break;

			case CanMessageType::setHeaterFaultDetection:
				requestId = buf->msg.setHeaterFaultDetection.requestId;
				rslt = reprap.GetHeat().SetFaultDetection(buf->msg.setHeaterFaultDetection, replyRef);
				break;

			case CanMessageType::setHeaterMonitors:
				{
					GET_VALID_MESSAGE(setHeaterMonitors);
					requestId = msg.requestId;
					rslt = reprap.GetHeat().SetHeaterMonitors(msg, replyRef);
				}
				break;

			case CanMessageType::setDefaultHeaterModel:
				{
					GET_VALID_MESSAGE(setDefaultHeaterModel);
					reprap.GetHeat().SetDefaultHeaterModel(*buf);
					CanInterface::SendResponseNoFree(buf);
				}
				return;

			case CanMessageType::m308V1:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetHeat().ProcessM308(msg, replyRef);
				}
				break;

			// Fan commands
			case CanMessageType::m950Fan:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetFansManager().ConfigureFanPort(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::fanParameters:
				{
					GET_VALID_MESSAGE(fanParameters);
					requestId = msg.requestId;
					rslt = reprap.GetFansManager().ConfigureFan(msg, replyRef);
				}
				break;

			case CanMessageType::setFanSpeed:
				{
					GET_VALID_MESSAGE(setFanSpeed);
					requestId = msg.requestId;
					rslt = reprap.GetFansManager().SetFanSpeed(msg, replyRef);
				}
				break;

			// GPIO commands
			case CanMessageType::m950Gpio:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetPlatform().EutHandleM950Gpio(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::writeGpio:
				{
					GET_VALID_MESSAGE(writeGpio);
					requestId = msg.requestId;
					rslt = reprap.GetPlatform().EutHandleGpioWrite(msg, replyRef);
				}
				break;

			// LED strip commands
			case CanMessageType::m950Led:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetPlatform().GetLedStripManager().HandleM950Led(msg, replyRef, extra);
					isProgrammed = true;
				}
				break;

			case CanMessageType::writeLedStrip:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetPlatform().GetLedStripManager().HandleLedSetColours(msg, replyRef);
				}
				break;

			// Driver commands
			case CanMessageType::setMotorCurrents:
				{
					GET_VALID_MESSAGE(multipleDrivesRequestFloat);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutSetMotorCurrents(msg, buf->dataLength, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::setStepsPerMmAndMicrostepping:
				{
					GET_VALID_MESSAGE(multipleDrivesStepsPerUnitAndMicrostepping);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutSetStepsPerMmAndMicrostepping(msg, buf->dataLength, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::setDriverStates:
				{
					GET_VALID_MESSAGE(multipleDrivesRequestDriverState);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutHandleSetDriverStates(msg, buf->dataLength, replyRef);
				}
				break;

			case CanMessageType::m915:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutProcessM915(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::setPressureAdvanceV2:
				{
					GET_VALID_MESSAGE(multipleDrivesRequestPressureAdvance);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutSetRemotePressureAdvanceV2(msg, buf->dataLength, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::setInputShapingV1:
				{
					GET_VALID_MESSAGE(setInputShapingV1);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutSetInputShaping(msg, buf->dataLength, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::m569:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutProcessM569(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::m569p2:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutProcessM569Point2(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::m569p7:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutProcessM569Point7(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::setStandstillCurrentFactor:
				{
					GET_VALID_MESSAGE(multipleDrivesRequestFloat);
					requestId = msg.requestId;
					rslt = reprap.GetMove().EutSetStandstillCurrentFactor(msg, buf->dataLength, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::m970:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
# if SUPPORT_PHASE_STEPPING
					rslt = reprap.GetMove().EutProcessM970(msg, replyRef);
					isProgrammed = true;
# else
					rslt = GCodeResult::errorNotSupported;
# endif
				}
				break;

			case CanMessageType::m970p3:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
# if SUPPORT_PHASE_STEPPING
					rslt = reprap.GetMove().EutProcessM970Point3(msg, replyRef);
					isProgrammed = true;
# else
					rslt = GCodeResult::errorNotSupported;
# endif
				}
				break;

			case CanMessageType::createInputMonitorV1:
				{
					GET_VALID_MESSAGE(createInputMonitorV1);
					requestId = msg.requestId;
					rslt = InputMonitor::Create(msg, buf->dataLength, replyRef, extra);
					isProgrammed = true;
				}
				break;

			case CanMessageType::changeInputMonitorV1:
				{
					GET_VALID_MESSAGE(changeInputMonitorV1);
					requestId = msg.requestId;
					rslt = InputMonitor::Change(msg, replyRef, extra);
					isProgrammed = true;
				}
				break;

			case CanMessageType::enableStallEndstop:
				{
					GET_VALID_MESSAGE(enableStallEndstop);
					requestId = msg.requestId;
					rslt = reprap.GetMove().SetStallEndstopReporting(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::readInputsRequest:
				// This one has its own reply message type
				{
					GET_VALID_MESSAGE(readInputsRequest);
					InputMonitor::ReadInputs(buf);
					CanInterface::SendResponseNoFree(buf);
				}
				return;

			// Filament monitor commands
			case CanMessageType::createFilamentMonitor:
				{
					GET_VALID_MESSAGE(createFilamentMonitor);
					requestId = msg.requestId;
					rslt = FilamentMonitor::Create(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::deleteFilamentMonitor:
				{
					GET_VALID_MESSAGE(deleteFilamentMonitor);
					requestId = msg.requestId;
					rslt = FilamentMonitor::Delete(msg, replyRef);
				}
				break;

			case CanMessageType::configureFilamentMonitor:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = FilamentMonitor::Configure(msg, replyRef);
				}
				break;

			case CanMessageType::m655:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					reply.copy("not supported by this board");
					rslt = GCodeResult::error;
				}
				break;

			case CanMessageType::diagnosticTest:
				requestId = buf->msg.diagnosticTest.requestId;
				//TODO support this function
				rslt = GCodeResult::errorNotSupported;
				break;

			case CanMessageType::m111:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = reprap.ProcessRemoteM111(msg, replyRef);
					isProgrammed = true;
				}
				break;

			case CanMessageType::setConnectionTimeout:
				{
					GET_VALID_MESSAGE(generic);
					requestId = msg.requestId;
					rslt = CanInterface::ProcessM959(msg, replyRef);
					isProgrammed = true;
				}
				break;

			default:
				// We received a message type that we don't recognise. If it's a broadcast, ignore it. If it's addressed to us, send a reply.
				if (buf->id.Dst() != CanInterface::GetCanAddress())
				{
					return;
				}
				requestId = CanRequestIdAcceptAlways;
				reply.printf("Board %u received unknown msg type %u", CanInterface::GetCanAddress(), (unsigned int)buf->id.MsgType());
				rslt = GCodeResult::error;
				break;
			}

			if (badMessage)
			{
				if (buf->id.Dst() != CanInterface::GetCanAddress())
				{
					return;
				}
				requestId = CanRequestIdAcceptAlways;
				reply.printf("Board %u received msg type %u with bad length %u", CanInterface::GetCanAddress(), (unsigned int)buf->id.MsgType(), (unsigned int)buf->dataLength);
				rslt = GCodeResult::error;
			}

			if (requestId != CanRequestIdNoReplyNeeded)				// if a reply is needed
			{
				// Re-use the message buffer to send a standard reply
				const CanAddress srcAddress = buf->id.Src();
				CanMessageStandardReply *msg = buf->SetupResponseMessage<CanMessageStandardReply>(requestId, CanInterface::GetCanAddress(), srcAddress);
				buf->useBrs = requestUsedBrs;
				msg->resultCode = (uint16_t)rslt;
				msg->extra = extra;
				const size_t totalLength = reply.strlen();
				size_t lengthDone = 0;
				uint8_t fragmentNumber = 0;
				for (;;)
				{
					const size_t fragmentLength = min<size_t>(totalLength - lengthDone, msg->GetMaxTextLength());
					memcpy(msg->GetText(), reply.c_str() + lengthDone, fragmentLength);
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
					msg->numWords = 0;						// data words go in fragment 0 only
				}
			}
		}
		else
#endif
		{
			// Handle messages received in normal operation mode
			bool badMessage = false;
			switch (id)
			{
			case CanMessageType::inputStateChangedV1:
				//TODO we should preferably handle this one using a separate high-priority queue or buffer
				{
					GET_VALID_MESSAGE(inputChangedV1);
					HandleInputStateChangedV1(msg, buf->id.Src());
				}
				break;

			case CanMessageType::inputStateChangedV2:
				//TODO we should preferably handle this one using a separate high-priority queue or buffer
				{
					GET_VALID_MESSAGE(inputChangedV2);
					HandleInputStateChangedV2(msg, buf->id.Src());
				}
				break;

			case CanMessageType::firmwareBlockRequest:
				{
					GET_VALID_MESSAGE(firmwareUpdateRequest);
					HandleFirmwareBlockRequest(buf);
				}
				break;

			case CanMessageType::sensorTemperaturesReport:
				{
					GET_VALID_MESSAGE(sensorTemperaturesBroadcast);
					reprap.GetHeat().ProcessRemoteSensorsReport(buf->id.Src(), msg);
				}
				break;

			case CanMessageType::heatersStatusReport:
				{
					GET_VALID_MESSAGE(heatersStatusBroadcast);
					reprap.GetHeat().ProcessRemoteHeatersReport(buf->id.Src(), msg);
				}
				break;

			case CanMessageType::driversStatusReport:
				{
					GET_VALID_MESSAGE(driversStatus);
					reprap.GetExpansion().ProcessDriveStatusReport(buf);
				}
				break;

			case CanMessageType::boardStatusReportV0:
				reprap.GetExpansion().ProcessBoardStatusReport(buf);
				break;

			case CanMessageType::boardStatusReportV1:
				{
					GET_VALID_MESSAGE(boardStatusV1);
					reprap.GetExpansion().ProcessBoardStatusReport(buf);
				}
				break;

			case CanMessageType::heaterTuningReport:
				{
					GET_VALID_MESSAGE(heaterTuningReport);
					reprap.GetHeat().ProcessRemoteHeaterTuningReport(buf->id.Src(), msg);
				}
				break;

			case CanMessageType::fansReport:
				{
					GET_VALID_MESSAGE(fansReport);
					reprap.GetFansManager().ProcessRemoteFanRpms(buf->id.Src(), msg);
				}
				break;

			case CanMessageType::announceV0:
				{
					GET_VALID_MESSAGE(announceV0);
					reprap.GetExpansion().ProcessAnnouncement(buf, false);
				}
				break;

			case CanMessageType::announceV1:
				{
					GET_VALID_MESSAGE(announceV1);
					reprap.GetExpansion().ProcessAnnouncement(buf, true);
				}
				break;

			case CanMessageType::filamentMonitorsStatusReportV2:
				{
					GET_VALID_MESSAGE(filamentMonitorsStatusV2);
					FilamentMonitor::UpdateRemoteFilamentStatus(buf->id.Src(), msg);
				}
				break;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
			case CanMessageType::closedLoopData:
				{
					GET_VALID_MESSAGE(closedLoopData);
					ClosedLoop::ProcessReceivedData(buf->id.Src(), msg, buf->dataLength);
				}
				break;
#endif

			case CanMessageType::event:
				{
					GET_VALID_MESSAGE(event);
					Event::Add(msg, buf->id.Src(), buf->dataLength);
				}
				break;

			case CanMessageType::debugText:
				{
					GET_VALID_MESSAGE(debugText);
					reprap.GetPlatform().MessageF(GenericMessage, "Debug from %u: %.*s\n", buf->id.Src(), msg.GetMaxTextLength(buf->dataLength), msg.text);
				}
				break;

#if SUPPORT_ACCELEROMETERS
			case CanMessageType::accelerometerData:
				{
					GET_VALID_MESSAGE(accelerometerData);
					Accelerometers::ProcessReceivedData(buf->id.Src(), msg, buf->dataLength);
				}
				break;
#endif

#if SUPPORT_REMOTE_COMMANDS
			case CanMessageType::enterTestMode:
				{
					GET_VALID_MESSAGE(enterTestMode);
					if (msg.passwd == CanMessageEnterTestMode::Passwd)
					{
						const CanAddress newAddress = msg.address;

						// Send a standard response before we switch
						const CanAddress srcAddress = buf->id.Src();
						const CanRequestId requestId = msg.requestId;
						const bool requestUsedBrs = buf->useBrs;

						CanMessageStandardReply * const replyMsg = buf->SetupResponseMessage<CanMessageStandardReply>(requestId, CanInterface::GetCanAddress(), srcAddress);
						buf->useBrs = requestUsedBrs;
						replyMsg->resultCode = (uint16_t)GCodeResult::ok;
						replyMsg->text[0] = 0;
						buf->dataLength = replyMsg->GetActualDataLength(0);
						replyMsg->fragmentNumber = 0;
						replyMsg->moreFollows = false;
						CanInterface::SendResponseNoFree(buf);

						delay(25);							// allow time for the response to be sent before we re-initialise CAN
						CanInterface::SwitchToExpansionMode(newAddress, true);
					}
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

			if (badMessage && reprap.Debug(Module::CAN))
			{
				buf->DebugPrint("Bad length: ");
			}
		}
	}
}

#endif

// End
