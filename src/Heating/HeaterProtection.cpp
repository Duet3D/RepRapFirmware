/*
 * HeaterProtection.cpp
 *
 *  Created on: 16 Nov 2017
 *      Author: Christian
 */

#include "HeaterProtection.h"

#include "Platform.h"
#include "RepRap.h"
#include "Heat.h"


HeaterProtection::HeaterProtection(size_t index) : next(nullptr)
{
	// By default each heater protection element is mapped to its corresponding heater.
	// All other heater protection elements are unused and can be optionally assigned.
	heater = supervisedHeater = (index >= NumHeaters) ? -1 : (int8_t)index;
}

void HeaterProtection::Init(float tempLimit)
{
	next = nullptr;
	limit = tempLimit;
	action = HeaterProtectionAction::GenerateFault;
	trigger = HeaterProtectionTrigger::TemperatureExceeded;

	badTemperatureCount = 0;
}

// Check if any action needs to be taken. Returns true if everything is OK
bool HeaterProtection::Check()
{
	if (supervisedHeater >= 0)
	{
		TemperatureError err;
		const float temperature = reprap.GetHeat().GetTemperature(supervisedHeater, err);

		if (err != TemperatureError::success)
		{
			badTemperatureCount++;
			if (badTemperatureCount > MaxBadTemperatureCount)
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "Temperature reading error on heater %d\n", supervisedHeater);
				return false;
			}
		}
		else
		{
			badTemperatureCount = 0;
			switch (trigger)
			{
			case HeaterProtectionTrigger::TemperatureExceeded:
				return (temperature <= limit);

			case HeaterProtectionTrigger::TemperatureTooLow:
				return (temperature >= limit);
			}
		}
	}
	return true;
}

void HeaterProtection::SetHeater(int8_t newHeater)
{
	heater = newHeater;
	reprap.GetHeat().UpdateHeaterProtection();
}

// End
