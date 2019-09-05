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
#include "HeaterProtection.h"
#include "Sensors/TemperatureSensor.h"

Heater::Heater(unsigned int num)
	: heaterNumber(num), sensorNumber(-1), activeTemperature(0.0), standbyTemperature(0.0),
	  maxTempExcursion(DefaultMaxTempExcursion), maxHeatingFaultTime(DefaultMaxHeatingFaultTime),
	  heaterProtection(nullptr), active(false)
{
}

Heater::~Heater()
{
}

// Set the process model returning true if successful
GCodeResult Heater::SetModel(float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, const StringRef& reply)
{
	const float temperatureLimit = GetHighestTemperatureLimit();
	const bool rslt = model.SetParameters(gain, tc, td, maxPwm, temperatureLimit, voltage, usePid, inverted);
	if (rslt)
	{
		if (model.IsEnabled())
		{
			const GCodeResult rslt = UpdateModel(reply);
			if (rslt != GCodeResult::ok)
			{
				return rslt;
			}
			const float predictedMaxTemp = gain + NormalAmbientTemperature;
			const float noWarnTemp = (temperatureLimit - NormalAmbientTemperature) * 1.5 + 50.0;		// allow 50% extra power plus enough for an extra 50C
			if (predictedMaxTemp > noWarnTemp)
			{
				reply.printf("heater %u appears to be over-powered. If left on at full power, its temperature is predicted to reach %dC.\n",
						GetHeaterNumber(), (int)predictedMaxTemp);
				return GCodeResult::warning;
			}
		}
		else
		{
			ResetHeater();
		}
		return GCodeResult::ok;
	}

	reply.copy("bad model parameters");
	return GCodeResult::error;
}

HeaterStatus Heater::GetStatus() const
{
	const HeaterMode mode = GetMode();
	return (mode == HeaterMode::fault) ? HeaterStatus::fault
			: (mode == HeaterMode::offline) ? HeaterStatus::offline
				: (mode == HeaterMode::off) ? HeaterStatus::off
					: (mode >= HeaterMode::tuning0) ? HeaterStatus::tuning
						: (active) ? HeaterStatus::active
							: HeaterStatus::standby;
}

const char* Heater::GetSensorName() const
{
	const auto sensor = reprap.GetHeat().FindSensor(sensorNumber);
	return (sensor.IsNotNull()) ? sensor->GetSensorName() : nullptr;
}

GCodeResult Heater::Activate(const StringRef& reply)
{
	if (GetMode() != HeaterMode::fault)
	{
		active = true;
		return SwitchOn(reply);
	}
	reply.printf("Can't activate heater %u while in fault state", heaterNumber);
	return GCodeResult::error;
}

void Heater::Standby()
{
	if (GetMode() != HeaterMode::fault)
	{
		active = false;
		String<1> dummy;
		(void)SwitchOn(dummy.GetRef());
	}
}

void Heater::SetActiveTemperature(float t)
{
	if (t > GetHighestTemperatureLimit())
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Temperature %.1f" DEGREE_SYMBOL "C too high for heater %u\n", (double)t, GetHeaterNumber());
	}
	else if (t < GetLowestTemperatureLimit())
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Temperature %.1f" DEGREE_SYMBOL "C too low for heater %u\n", (double)t, GetHeaterNumber());
	}
	else
	{
		activeTemperature = t;
		if (GetMode() > HeaterMode::suspended && active)
		{
			String<1> dummy;
			(void)SwitchOn(dummy.GetRef());
		}
	}
}

void Heater::SetStandbyTemperature(float t)
{
	if (t > GetHighestTemperatureLimit())
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Temperature %.1f" DEGREE_SYMBOL "C too high for heater %u\n", (double)t, GetHeaterNumber());
	}
	else if (t < GetLowestTemperatureLimit())
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Temperature %.1f" DEGREE_SYMBOL "C too low for heater %u\n", (double)t, GetHeaterNumber());
	}
	else
	{
		standbyTemperature = t;
		if (GetMode() > HeaterMode::suspended && !active)
		{
			String<1> dummy;
			(void)SwitchOn(dummy.GetRef());
		}
	}
}

// Get the highest temperature limit
float Heater::GetHighestTemperatureLimit() const
{
	return reprap.GetHeat().GetHighestTemperatureLimit(GetHeaterNumber());
}

// Get the lowest temperature limit
float Heater::GetLowestTemperatureLimit() const
{
	return reprap.GetHeat().GetLowestTemperatureLimit(GetHeaterNumber());
}

void Heater::SetHeaterProtection(HeaterProtection *h)
{
	heaterProtection = h;
}

// Check heater protection elements and return true if everything is good
bool Heater::CheckProtection() const
{
	for (HeaterProtection *prot = heaterProtection; prot != nullptr; prot = prot->Next())
	{
		if (!prot->Check())
		{
			// Something is not right
			return false;
		}
	}
	return true;
}

bool Heater::CheckGood() const
{
	return GetMode() != HeaterMode::fault && CheckProtection();
}

void Heater::SetModelDefaults()
{
	if (reprap.GetHeat().IsBedOrChamberHeater(GetHeaterNumber()))
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
