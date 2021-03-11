/*
 * HeaterProtection.cpp
 *
 *  Created on: 16 Nov 2017
 *      Author: Christian
 */

#include "HeaterMonitor.h"

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include "Heat.h"

HeaterMonitor::HeaterMonitor() noexcept
	: sensorNumber(-1), trigger(HeaterMonitorTrigger::Disabled), badTemperatureCount(0)
{
}

// Check if any action needs to be taken. Returns true if everything is OK
bool HeaterMonitor::Check() noexcept
{
	if (sensorNumber >= 0 && trigger != HeaterMonitorTrigger::Disabled)
	{
		TemperatureError err;
		const float temperature = reprap.GetHeat().GetSensorTemperature(sensorNumber, err);

		if (err != TemperatureError::success)
		{
			badTemperatureCount++;
			if (badTemperatureCount > MaxBadTemperatureCount)
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "Temperature reading error on sensor %d\n", sensorNumber);
				return false;
			}
		}
		else
		{
			badTemperatureCount = 0;
			switch (trigger)
			{
			case HeaterMonitorTrigger::TemperatureExceeded:
				return (temperature <= limit);

			case HeaterMonitorTrigger::TemperatureTooLow:
				return (temperature >= limit);

			default:
				break;
			}
		}
	}
	return true;
}

// Append a report of this monitor to the string
void HeaterMonitor::Report(unsigned int heater, unsigned int index, const StringRef& reply) const noexcept
{
	reply.lcatf("Heater %d monitor %d ", heater, index);
	if (trigger == HeaterMonitorTrigger::Disabled)
	{
		reply.cat("is disabled");
	}
	else
	{
		const char *actionString, *triggerString;
		switch (action)
		{
		case HeaterMonitorAction::GenerateFault:		actionString = "generate a heater fault"; break;
		case HeaterMonitorAction::PermanentSwitchOff:	actionString = "permanently switch off"; break;
		case HeaterMonitorAction::TemporarySwitchOff:	actionString = "temporarily switch off"; break;
		case HeaterMonitorAction::ShutDown:				actionString = "shut down the printer"; break;
		default:										actionString = "(undefined)"; break;
		}

		switch (trigger)
		{
		case HeaterMonitorTrigger::TemperatureExceeded:	triggerString = "exceeds"; break;
		case HeaterMonitorTrigger::TemperatureTooLow:	triggerString = "falls below"; break;
		default:										triggerString = "(undefined)"; break;
		}

		reply.catf("uses sensor %d to %s if the reading %s %.1f" DEGREE_SYMBOL "C", sensorNumber, actionString, triggerString, (double)limit);
	}
}

// Get the condition for a temperature event as a string
const char *HeaterMonitor::GetTriggerName() const noexcept
{
	switch (trigger)
	{
	case HeaterMonitorTrigger::Disabled:				return "disabled";
	case HeaterMonitorTrigger::TemperatureExceeded:		return "tooHigh";
	case HeaterMonitorTrigger::TemperatureTooLow:		return "tooLow";
	default:											return "undefined";
	}
}

// End
