/*
 * Heater.cpp
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#include "Heater.h"
#include "RepRap.h"
#include "Platform.h"
#include "Heat.h"
#include "HeaterMonitor.h"
#include "Sensors/TemperatureSensor.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <GCodes/GCodeException.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Heater, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(Heater, __VA_ARGS__)

constexpr ObjectModelArrayDescriptor Heater::monitorsArrayDescriptor =
{
	nullptr,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return MaxMonitorsPerHeater; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(self, 1); }

};

constexpr ObjectModelTableEntry Heater::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Heater members
	{ "active",		OBJECT_MODEL_FUNC(self->GetActiveTemperature(), 1), 									ObjectModelEntryFlags::live },
	{ "current",	OBJECT_MODEL_FUNC(self->GetTemperature(), 1), 											ObjectModelEntryFlags::live },
	{ "max",		OBJECT_MODEL_FUNC(self->GetHighestTemperatureLimit(), 1), 								ObjectModelEntryFlags::none },
	{ "min",		OBJECT_MODEL_FUNC(self->GetLowestTemperatureLimit(), 1), 								ObjectModelEntryFlags::none },
	{ "model",		OBJECT_MODEL_FUNC((const FopDt *)&self->GetModel()),									ObjectModelEntryFlags::verbose },
	{ "monitors",	OBJECT_MODEL_FUNC_NOSELF(&monitorsArrayDescriptor), 									ObjectModelEntryFlags::none },
	{ "sensor",		OBJECT_MODEL_FUNC((int32_t)self->GetSensorNumber()), 									ObjectModelEntryFlags::none },
	{ "standby",	OBJECT_MODEL_FUNC(self->GetStandbyTemperature(), 1), 									ObjectModelEntryFlags::live },
	{ "state",		OBJECT_MODEL_FUNC(self->GetStatus().ToString()), 										ObjectModelEntryFlags::live },

	// 1. Heater.monitors[] members
	{ "action",		OBJECT_MODEL_FUNC_IF(self->monitors[context.GetLastIndex()].GetTrigger() != HeaterMonitorTrigger::Disabled,
										(int32_t)self->monitors[context.GetLastIndex()].GetAction()), 		ObjectModelEntryFlags::none },
	{ "condition",	OBJECT_MODEL_FUNC(self->monitors[context.GetLastIndex()].GetTriggerName()), 			ObjectModelEntryFlags::none },
	{ "limit",		OBJECT_MODEL_FUNC_IF(self->monitors[context.GetLastIndex()].GetTrigger() != HeaterMonitorTrigger::Disabled,
										self->monitors[context.GetLastIndex()].GetTemperatureLimit(), 1),	ObjectModelEntryFlags::none },
};

constexpr uint8_t Heater::objectModelTableDescriptor[] = { 2, 9, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE(Heater)

#endif

Heater::Heater(unsigned int num) noexcept
	: heaterNumber(num), sensorNumber(-1), activeTemperature(0.0), standbyTemperature(0.0),
	  maxTempExcursion(DefaultMaxTempExcursion), maxHeatingFaultTime(DefaultMaxHeatingFaultTime),
	  active(false)
{
}

Heater::~Heater() noexcept
{
	for (HeaterMonitor& h : monitors)
	{
		h.Disable();
	}
}

void Heater::SetSensorNumber(int sn) noexcept
{
	if (sn != sensorNumber)
	{
		sensorNumber = sn;
		SetDefaultMonitors();
	}
}

void Heater::SetDefaultMonitors() noexcept
{
	for (HeaterMonitor& h : monitors)
	{
		h.Disable();
	}

	if (sensorNumber >= 0 && sensorNumber < (int)MaxSensors)
	{
		const float limit = (reprap.GetHeat().IsBedOrChamberHeater(heaterNumber)) ? DefaultBedTemperatureLimit : DefaultHotEndTemperatureLimit;
		monitors[0].Set(sensorNumber, limit, HeaterMonitorAction::GenerateFault, HeaterMonitorTrigger::TemperatureExceeded);
	}
}

GCodeResult Heater::SetOrReportModel(unsigned int heater, GCodeBuffer& gb, const StringRef& reply) noexcept
{
	bool seen = false;
	float gain = model.GetGain(),
		tc = model.GetTimeConstant(),
		td = model.GetDeadTime(),
		maxPwm = model.GetMaxPwm(),
		voltage = model.GetVoltage();
	int32_t dontUsePid = model.UsePid() ? 0 : 1;
	int32_t inversionParameter = 0;

	gb.TryGetFValue('A', gain, seen);
	gb.TryGetFValue('C', tc, seen);
	gb.TryGetFValue('D', td, seen);
	gb.TryGetIValue('B', dontUsePid, seen);
	gb.TryGetFValue('S', maxPwm, seen);
	gb.TryGetFValue('V', voltage, seen);
	gb.TryGetIValue('I', inversionParameter, seen);

	if (seen)
	{
		const bool inverseTemperatureControl = (inversionParameter == 1 || inversionParameter == 3);
		const GCodeResult rslt = SetModel(gain, tc, td, maxPwm, voltage, dontUsePid == 0, inverseTemperatureControl, reply);
		if (rslt != GCodeResult::ok)
		{
			return rslt;
		}
	}
	else if (!model.IsEnabled())
	{
		reply.printf("Heater %u is disabled", heater);
	}
	else
	{
		const char* const mode = (!model.UsePid()) ? "bang-bang"
									: (model.ArePidParametersOverridden()) ? "custom PID"
										: "PID";
		reply.printf("Heater %u model: gain %.1f, time constant %.1f, dead time %.1f, max PWM %.2f, calibration voltage %.1f, mode %s", heater,
					 (double)model.GetGain(), (double)model.GetTimeConstant(), (double)model.GetDeadTime(), (double)model.GetMaxPwm(), (double)model.GetVoltage(), mode);
		if (model.IsInverted())
		{
			reply.cat(", inverted temperature control");
		}
		if (model.UsePid())
		{
			// When reporting the PID parameters, we scale them by 255 for compatibility with older firmware and other firmware
			M301PidParameters params = model.GetM301PidParameters(false);
			reply.catf("\nComputed PID parameters for setpoint change: P%.1f, I%.3f, D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
			params = model.GetM301PidParameters(true);
			reply.catf("\nComputed PID parameters for load change: P%.1f, I%.3f, D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
		}
	}
	return GCodeResult::ok;
}

// Set the process model returning true if successful
GCodeResult Heater::SetModel(float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, const StringRef& reply) noexcept
{
	GCodeResult rslt;
	if (model.SetParameters(gain, tc, td, maxPwm, GetHighestTemperatureLimit(), voltage, usePid, inverted))
	{
		if (model.IsEnabled())
		{
			rslt = UpdateModel(reply);
			if (rslt == GCodeResult::ok)
			{
				const float predictedMaxTemp = gain + NormalAmbientTemperature;
				const float noWarnTemp = (GetHighestTemperatureLimit() - NormalAmbientTemperature) * 1.5 + 50.0;		// allow 50% extra power plus enough for an extra 50C
				if (predictedMaxTemp > noWarnTemp)
				{
					reply.printf("Heater %u appears to be over-powered. If left on at full power, its temperature is predicted to reach %dC", GetHeaterNumber(), (int)predictedMaxTemp);
					rslt = GCodeResult::warning;
				}
			}
		}
		else
		{
			ResetHeater();
			rslt = GCodeResult::ok;
		}
	}
	else
	{
		reply.copy("bad model parameters");
		rslt = GCodeResult::error;
	}

	reprap.HeatUpdated();
	return rslt;
}

GCodeResult Heater::SetFaultDetectionParameters(float pMaxTempExcursion, float pMaxFaultTime, const StringRef& reply) noexcept
{
	maxTempExcursion = pMaxTempExcursion;
	maxHeatingFaultTime = pMaxFaultTime;
	const GCodeResult rslt = UpdateFaultDetectionParameters(reply);
	reprap.HeatUpdated();
	return rslt;
}

GCodeResult Heater::ConfigureMonitor(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	// Get any parameters that have been provided
	const bool seenP = gb.Seen('P');
	const size_t index = (seenP) ? gb.GetLimitedUIValue('P', MaxMonitorsPerHeater) : 0;

	const bool seenSensor = gb.Seen('T');
	const int monitoringSensor = (seenSensor) ? gb.GetLimitedUIValue('T', MaxSensors) : GetSensorNumber();

	const bool seenAction = gb.Seen('A');
	const HeaterMonitorAction action = (seenAction)
										? static_cast<HeaterMonitorAction>(gb.GetLimitedUIValue('A', (unsigned int)MaxHeaterMonitorAction + 1))
											: HeaterMonitorAction::GenerateFault;

	const bool seenCondition = gb.Seen('C');
	const HeaterMonitorTrigger trigger = (seenCondition)
											? static_cast<HeaterMonitorTrigger>(gb.GetLimitedIValue('C', -1, (int)MaxHeaterMonitorTrigger))
												: HeaterMonitorTrigger::TemperatureExceeded;

	const bool seenLimit = gb.Seen('S');
	const float limit = (seenLimit) ? gb.GetFValue() : monitors[index].GetTemperatureLimit();
	if (limit <= BadLowTemperature || limit >= BadErrorTemperature)
	{
		reply.copy("Invalid temperature limit");
		return GCodeResult::error;
	}

	if (seenSensor || seenLimit || seenAction || seenCondition)
	{
		monitors[index].Set(monitoringSensor, limit, action, trigger);
		const GCodeResult rslt = UpdateHeaterMonitors(reply);
		reprap.HeatUpdated();
		return rslt;
	}

	// Else we are reporting on one or all of the monitors
	if (seenP)
	{
		monitors[index].Report(heaterNumber, index, reply);
	}
	else
	{
		for (size_t i = 0; i < MaxMonitorsPerHeater; ++i)
		{
			monitors[i].Report(heaterNumber, i, reply);
		}
	}
	return GCodeResult::ok;
}

float Heater::GetHighestTemperatureLimit() const noexcept
{
	float limit = BadErrorTemperature;
	for (const HeaterMonitor& prot : monitors)
	{
		if (prot.GetTrigger() == HeaterMonitorTrigger::TemperatureExceeded)
		{
			const float t = prot.GetTemperatureLimit();
			if (limit == BadErrorTemperature || t > limit)
			{
				limit = t;
			}
		}
	}
	return limit;
}

float Heater::GetLowestTemperatureLimit() const noexcept
{
	float limit = ABS_ZERO;
	for (const HeaterMonitor& prot : monitors)
	{
		if (prot.GetTrigger() == HeaterMonitorTrigger::TemperatureTooLow)
		{
			const float t = prot.GetTemperatureLimit();
			if (limit == ABS_ZERO || t < limit)
			{
				limit = t;
			}
		}
	}
	return limit;
}

HeaterStatus Heater::GetStatus() const noexcept
{
	const HeaterMode mode = GetMode();
	return (mode == HeaterMode::fault) ? HeaterStatus::fault
			: (mode == HeaterMode::offline) ? HeaterStatus::offline
				: (mode == HeaterMode::off) ? HeaterStatus::off
					: (mode >= HeaterMode::tuning0) ? HeaterStatus::tuning
						: (active) ? HeaterStatus::active
							: HeaterStatus::standby;
}

const char* Heater::GetSensorName() const noexcept
{
	const auto sensor = reprap.GetHeat().FindSensor(sensorNumber);
	return (sensor.IsNotNull()) ? sensor->GetSensorName() : nullptr;
}

GCodeResult Heater::Activate(const StringRef& reply) noexcept
{
	if (GetMode() != HeaterMode::fault)
	{
		active = true;
		return SwitchOn(reply);
	}
	reply.printf("Can't activate heater %u while in fault state", heaterNumber);
	return GCodeResult::error;
}

void Heater::Standby() noexcept
{
	if (GetMode() != HeaterMode::fault)
	{
		active = false;
		String<1> dummy;
		(void)SwitchOn(dummy.GetRef());
	}
}

void Heater::SetTemperature(float t, bool activeNotStandby) THROWS(GCodeException)
{
	if (t > GetHighestTemperatureLimit())
	{
		throw GCodeException(-1, -1, "Temperature too high for heater %" PRIu32, (uint32_t)GetHeaterNumber());
	}
	else if (t < GetLowestTemperatureLimit())
	{
		throw GCodeException(-1, -1, "Temperature too low for heater %" PRIu32, (uint32_t)GetHeaterNumber());
	}
	else
	{
		((activeNotStandby) ? activeTemperature : standbyTemperature) = t;
		if (GetMode() > HeaterMode::suspended && active == activeNotStandby)
		{
			String<1> dummy;
			(void)SwitchOn(dummy.GetRef());
		}
	}
}

void Heater::SetModelDefaults() noexcept
{
	if (reprap.GetHeat().IsBedOrChamberHeater(heaterNumber))
	{
		model.SetParameters(DefaultBedHeaterGain, DefaultBedHeaterTimeConstant, DefaultBedHeaterDeadTime, 1.0, DefaultBedTemperatureLimit, 0.0, false, false);
	}
	else
	{
		model.SetParameters(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, 1.0, DefaultHotEndTemperatureLimit, 0.0, true, false);
	}

	String<1> dummy;
	(void)UpdateModel(dummy.GetRef());
}

// End
