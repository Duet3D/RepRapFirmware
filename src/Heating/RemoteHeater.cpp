/*
 * RemoteHeater.cpp
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#include "RemoteHeater.h"

#if SUPPORT_CAN_EXPANSION

#include <Platform/RepRap.h>
#include "Heat.h"
#include <Platform/Platform.h>
#include <CAN/CanMessageGenericConstructor.h>
#include <CAN/CanInterface.h>
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>
#include <CanMessageGenericTables.h>

// Static variables used only during tuning
uint32_t RemoteHeater::timeSetHeating;
float RemoteHeater::currentCoolingRate;
unsigned int RemoteHeater::tuningCyclesDone;
bool RemoteHeater::newTuningResult = false;

RemoteHeater::RemoteHeater(unsigned int num, CanAddress board) noexcept
	: Heater(num), boardAddress(board), lastMode(HeaterMode::offline), averagePwm(0), tuningState(TuningState::notTuning), lastTemperature(0.0), whenLastStatusReceived(0)
{
}

RemoteHeater::~RemoteHeater() noexcept
{
	CanMessageGenericConstructor cons(M950HeaterParams);
	cons.AddUParam('H', GetHeaterNumber());
	cons.AddStringParam('C', "nil");
	String<1> dummy;
	(void)cons.SendAndGetResponse(CanMessageType::m950Heater, boardAddress, dummy.GetRef());
}

void RemoteHeater::Spin() noexcept
{
	const uint32_t now = millis();
	switch (tuningState)
	{
	case TuningState::notTuning:
		break;

	case TuningState::stabilising:
		if (tuningStartTemp.GetNumSamples() < 5000/HeatSampleIntervalMillis)
		{
			tuningStartTemp.Add(lastTemperature);						// take another reading until we have samples temperatures for 5 seconds
		}
		else if (tuningStartTemp.GetDeviation() <= 2.0)
		{
			timeSetHeating = now;
			ClearCounters();
			timeSetHeating = millis();
			String<StringLength100> reply;
			if (SendTuningCommand(reply.GetRef(), true) == GCodeResult::ok)
			{
				tuningState = TuningState::heatingUp;
				tuningPhase = 1;
				ReportTuningUpdate();
			}
			else
			{
				reprap.GetPlatform().Message(ErrorMessage, "Failed to start heater tuning\n");
				tuningState = TuningState::notTuning;
			}
		}
		else if (now - tuningBeginTime >= 20000)						// allow up to 20 seconds for starting temperature to settle
		{
			reprap.GetPlatform().Message(GenericMessage, "Auto tune cancelled because starting temperature is not stable\n");
			StopTuning();
		}
		break;

	case TuningState::heatingUp:
		{
			const bool isBedOrChamberHeater = reprap.GetHeat().IsBedOrChamberHeater(GetHeaterNumber());
			const uint32_t heatingTime = now - timeSetHeating;
			const float extraTimeAllowed = (isBedOrChamberHeater) ? 120.0 : 30.0;
			if (heatingTime > (uint32_t)((GetModel().GetDeadTime() + extraTimeAllowed) * SecondsToMillis) && (lastTemperature - tuningStartTemp.GetMean()) < 3.0)
			{
				reprap.GetPlatform().Message(GenericMessage, "Auto tune cancelled because temperature is not increasing\n");
				StopTuning();
				break;
			}

			const uint32_t timeoutMinutes = (isBedOrChamberHeater) ? 30 : 7;
			if (heatingTime >= timeoutMinutes * 60 * (uint32_t)SecondsToMillis)
			{
				reprap.GetPlatform().Message(GenericMessage, "Auto tune cancelled because target temperature was not reached\n");
				StopTuning();
				break;
			}

			if (lastTemperature >= tuningTargetTemp)							// if reached target
			{
				// Move on to next phase
				peakTemp = afterPeakTemp = lastTemperature;
				lastOffTime = peakTime = afterPeakTime = now;
				tuningVoltage.Clear();
				idleCyclesDone = 0;
				newTuningResult = false;
				tuningState = TuningState::idleCycles;
				tuningPhase = 2;
				ReportTuningUpdate();
			}
		}
		break;

	case TuningState::idleCycles:
		if (newTuningResult)
		{
			// To allow for heat reservoirs, we do idle cycles until the cooling rate decreases by no more than a certain amount in a single cycle
			if (idleCyclesDone == TuningHeaterMaxIdleCycles || (idleCyclesDone >= TuningHeaterMinIdleCycles && currentCoolingRate >= lastCoolingRate * HeaterSettledCoolingTimeRatio))
			{
				tuningPhase = 3;
				tuningState = TuningState::cycling;
				ReportTuningUpdate();
			}
			else
			{
				lastCoolingRate = currentCoolingRate;
				ClearCounters();
				++idleCyclesDone;
			}
			newTuningResult = false;
		}
		break;

	case TuningState::cycling:
		if (newTuningResult)
		{
			if (coolingRate.GetNumSamples() >= MinTuningHeaterCycles)
			{
				const bool isConsistent = dLow.DeviationFractionWithin(0.2)
										&& dHigh.DeviationFractionWithin(0.2)
										&& heatingRate.DeviationFractionWithin(0.1)
										&& coolingRate.DeviationFractionWithin(0.1);
				if (isConsistent || coolingRate.GetNumSamples() == MaxTuningHeaterCycles)
				{
					if (!isConsistent)
					{
						reprap.GetPlatform().Message(WarningMessage, "heater behaviour was not consistent during tuning\n");
					}

					if (tuningPhase == 3)
					{
						CalculateModel(fanOffParams);
						if (tuningFans.IsEmpty())
						{
							SetAndReportModel(false);
							StopTuning();
							break;
						}
						else
						{
							tuningPhase = 4;
							ClearCounters();
#if TUNE_WITH_HALF_FAN
							reprap.GetFansManager().SetFansValue(tuningFans,tuningFanPwm *  0.5);	// turn fans on at half PWM
#else
							reprap.GetFansManager().SetFansValue(tuningFans, tuningFanPwm);		// turn fans on at full PWM
#endif
							ReportTuningUpdate();
						}
					}
#if TUNE_WITH_HALF_FAN
					else if (tuningPhase == 4)
					{
						CalculateModel(fanOnParams);
						tuningPhase = 5;
						ClearCounters();
						reprap.GetFansManager().SetFansValue(tuningFans, tuningFanPwm);			// turn fans fully on
						ReportTuningUpdate();
					}
#endif
					else
					{
						reprap.GetFansManager().SetFansValue(tuningFans, 0.0);					// turn fans off
						CalculateModel(fanOnParams);
						SetAndReportModel(true);
						StopTuning();
						break;
					}
				}
			}
			newTuningResult = false;
		}
		break;
	}
}

void RemoteHeater::ResetHeater() noexcept
{
	// This is only called by UpdateModel. Nothing needed here.
}

GCodeResult RemoteHeater::ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply)
{
	SetSensorNumber(sn);
	CanMessageGenericConstructor cons(M950HeaterParams);
	cons.AddUParam('H', GetHeaterNumber());
	cons.AddUParam('Q', freq);
	cons.AddUParam('T', sn);
	cons.AddStringParam('C', portName);
	return cons.SendAndGetResponse(CanMessageType::m950Heater, boardAddress, reply);
}

GCodeResult RemoteHeater::SetPwmFrequency(PwmFrequency freq, const StringRef& reply)
{
	CanMessageGenericConstructor cons(M950HeaterParams);
	cons.AddUParam('H', GetHeaterNumber());
	cons.AddUParam('Q', freq);
	return cons.SendAndGetResponse(CanMessageType::m950Heater, boardAddress, reply);
}

GCodeResult RemoteHeater::ReportDetails(const StringRef& reply) const noexcept
{
	CanMessageGenericConstructor cons(M950HeaterParams);
	cons.AddUParam('H', GetHeaterNumber());
	return cons.SendAndGetResponse(CanMessageType::m950Heater, boardAddress, reply);
}

void RemoteHeater::SwitchOff() noexcept
{
	constexpr const char *errMsg = "Failed to switch off remote heater %u: %s\n";
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, errMsg, GetHeaterNumber(), "no CAN buffer available");
	}
	else
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
		auto msg = buf->SetupRequestMessage<CanMessageSetHeaterTemperature>(rid, CanInterface::GetCanAddress(), boardAddress);
		msg->heaterNumber = GetHeaterNumber();
		msg->setPoint = GetTargetTemperature();
		msg->command = CanMessageSetHeaterTemperature::commandOff;
		String<StringLength100> reply;
		if (CanInterface::SendRequestAndGetStandardReply(buf, rid, reply.GetRef()) != GCodeResult::ok)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, errMsg, GetHeaterNumber(), reply.c_str());
		}
	}
}

GCodeResult RemoteHeater::ResetFault(const StringRef& reply) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageSetHeaterTemperature>(rid, CanInterface::GetCanAddress(), boardAddress);
	msg->heaterNumber = GetHeaterNumber();
	msg->setPoint = GetTargetTemperature();
	msg->command = CanMessageSetHeaterTemperature::commandResetFault;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

float RemoteHeater::GetTemperature() const noexcept
{
	if (millis() - whenLastStatusReceived < RemoteStatusTimeout)
	{
		return lastTemperature;
	}

	TemperatureError err;
	return reprap.GetHeat().GetSensorTemperature(GetSensorNumber(), err);
}

float RemoteHeater::GetAveragePWM() const noexcept
{
	return (millis() - whenLastStatusReceived < RemoteStatusTimeout) ? (float)averagePwm / 255.0 : 0;
}

// Return the integral accumulator
float RemoteHeater::GetAccumulator() const noexcept
{
	return 0.0;		// not supported
}

// Auto tune this heater. The caller has already checked that no other heater is being tuned and has set up tuningTargetTemp, tuningPwm, tuningFans, tuningHysteresis and tuningFanPwm.
GCodeResult RemoteHeater::StartAutoTune(const StringRef& reply, bool seenA, float ambientTemp) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	reprap.GetFansManager().SetFansValue(tuningFans, 0.0);

	tuningStartTemp.Clear();
	tuningBeginTime = millis();
	tuned = false;

	if (seenA)
	{
		tuningStartTemp.Add(ambientTemp);
		ClearCounters();
		timeSetHeating = millis();
		GCodeResult rslt = SendTuningCommand(reply, true);
		if (rslt != GCodeResult::ok)
		{
			return rslt;
		}
		tuningState = TuningState::heatingUp;
		tuningPhase = 1;
		ReportTuningUpdate();
	}
	else
	{
		tuningState = TuningState::stabilising;
		tuningPhase = 0;
	}

	return GCodeResult::ok;
}

void RemoteHeater::FeedForwardAdjustment(float fanPwmChange, float extrusionChange) noexcept
{
	constexpr const char* warnMsg = "Failed to make heater feedforward adjustment: %s\n";
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reprap.GetPlatform().MessageF(WarningMessage, warnMsg, "no CAN buffer");
	}
	else
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
		auto msg = buf->SetupRequestMessage<CanMessageHeaterFeedForward>(rid, CanInterface::GetCanAddress(), boardAddress);
		msg->heaterNumber = GetHeaterNumber();
		msg->fanPwmAdjustment = fanPwmChange;
		msg->extrusionAdjustment = extrusionChange;
		String<StringLength100> reply;
		if (CanInterface::SendRequestAndGetStandardReply(buf, rid, reply.GetRef()) != GCodeResult::ok)
		{
			reprap.GetPlatform().MessageF(WarningMessage, reply.c_str());
		}
	}
}

void RemoteHeater::Suspend(bool sus) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
		auto msg = buf->SetupRequestMessage<CanMessageSetHeaterTemperature>(rid, CanInterface::GetCanAddress(), boardAddress);
		msg->heaterNumber = GetHeaterNumber();
		msg->setPoint = GetTargetTemperature();
		msg->command = (sus) ? CanMessageSetHeaterTemperature::commandSuspend : CanMessageSetHeaterTemperature::commandUnsuspend;
		String<1> dummy;
		(void) CanInterface::SendRequestAndGetStandardReply(buf, rid, dummy.GetRef());
	}
}

HeaterMode RemoteHeater::GetMode() const noexcept
{
	return (tuningState != TuningState::notTuning) ? HeaterMode::tuning0
		: (millis() - whenLastStatusReceived < RemoteStatusTimeout) ? lastMode
			: HeaterMode::offline;
}

// This isn't just called to turn the heater on, it is called when the temperature needs to be updated
GCodeResult RemoteHeater::SwitchOn(const StringRef& reply) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageSetHeaterTemperature>(rid, CanInterface::GetCanAddress(), boardAddress);
	msg->heaterNumber = GetHeaterNumber();
	msg->setPoint = GetTargetTemperature();
	msg->command = CanMessageSetHeaterTemperature::commandOn;
	const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
	if (lastMode == HeaterMode::off && rslt <= GCodeResult::warning)
	{
		// If the heater was previously off then we need to change lastMode, otherwise if M116 is executed before we get an update from the expansion board
		// then it will treat the heater as inactive and not wait for it to reach temperature
		lastMode = HeaterMode::heating;
	}
	return rslt;
}

// This is called when the heater model has been updated
GCodeResult RemoteHeater::UpdateModel(const StringRef& reply) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
		CanMessageUpdateHeaterModelNew * const msg = buf->SetupRequestMessage<CanMessageUpdateHeaterModelNew>(rid, CanInterface::GetCanAddress(), boardAddress);
		GetModel().SetupCanMessage(GetHeaterNumber(), *msg);
		return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
	}

	reply.copy("No CAN buffer");
	return GCodeResult::error;
}

GCodeResult RemoteHeater::UpdateFaultDetectionParameters(const StringRef& reply) noexcept
{
	CanMessageBuffer *const buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
		CanMessageSetHeaterFaultDetectionParameters * const msg = buf->SetupRequestMessage<CanMessageSetHeaterFaultDetectionParameters>(rid, CanInterface::GetCanAddress(), boardAddress);
		msg->heater = GetHeaterNumber();
		msg->maxFaultTime = GetMaxHeatingFaultTime();
		msg->maxTempExcursion = GetMaxTemperatureExcursion();
		return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
	}

	reply.copy("No CAN buffer");
	return GCodeResult::error;
}

GCodeResult RemoteHeater::UpdateHeaterMonitors(const StringRef& reply) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
		CanMessageSetHeaterMonitors * const msg = buf->SetupRequestMessage<CanMessageSetHeaterMonitors>(rid, CanInterface::GetCanAddress(), boardAddress);
		msg->heater = GetHeaterNumber();
		msg->numMonitors = MaxMonitorsPerHeater;
		for (size_t i = 0; i < MaxMonitorsPerHeater; ++i)
		{
			msg->monitors[i].limit = monitors[i].GetTemperatureLimit();
			msg->monitors[i].sensor = monitors[i].GetSensorNumber();
			msg->monitors[i].action = (uint8_t)monitors[i].GetAction();
			msg->monitors[i].trigger = (int8_t)monitors[i].GetTrigger();
		}
		return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
	}

	reply.copy("No CAN buffer");
	return GCodeResult::error;
}

// This function processes an incoming heater report from an expansion board
void RemoteHeater::UpdateRemoteStatus(CanAddress src, const CanHeaterReport& report) noexcept
{
	if (src == boardAddress)
	{
		lastMode = (HeaterMode)report.mode;
		averagePwm = report.averagePwm;
		lastTemperature = report.temperature;
		whenLastStatusReceived = millis();
	}
}

// This function processes an incoming heater tuning report from an expansion board
void RemoteHeater::UpdateHeaterTuning(CanAddress src, const CanMessageHeaterTuningReport& msg) noexcept
{
	if (src == boardAddress && tuningState >= TuningState::idleCycles && !newTuningResult)
	{
		tOn.Add((float)msg.ton);
		tOff.Add((float)msg.toff);
		dHigh.Add((float)msg.dhigh);
		dLow.Add((float)msg.dlow);
		heatingRate.Add(msg.heatingRate);
		coolingRate.Add(msg.coolingRate);
		tuningVoltage.Add(msg.voltage);
		currentCoolingRate = msg.coolingRate;
		tuningCyclesDone = msg.cyclesDone;
		newTuningResult = true;
	}
}

GCodeResult RemoteHeater::SendTuningCommand(const StringRef& reply, bool on) noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	auto msg = buf->SetupRequestMessage<CanMessageHeaterTuningCommand>(rid, CanInterface::GetCanAddress(), boardAddress);
	msg->heaterNumber = GetHeaterNumber();
	msg->on = on;
	msg->highTemp = tuningTargetTemp;
	msg->lowTemp = tuningTargetTemp - tuningHysteresis;
	msg->pwm = tuningPwm;
	msg->peakTempDrop = TuningPeakTempDrop;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

void RemoteHeater::StopTuning() noexcept
{
	tuningState = TuningState::notTuning;
	String<StringLength100> reply;
	if (SendTuningCommand(reply.GetRef(), false) != GCodeResult::ok)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "%s\n", reply.c_str());
		reprap.GetPlatform().MessageF(ErrorMessage, "DANGER! Failed to stop tuning heater %u on CAN board %u, suggest turn power off\n", GetHeaterNumber(), boardAddress);
	}
}

#if SUPPORT_REMOTE_COMMANDS

// This should never be called
GCodeResult RemoteHeater::TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept
{
	reply.copy("not supported on remote heaters");
	return GCodeResult::error;
}

#endif

#endif

// End
