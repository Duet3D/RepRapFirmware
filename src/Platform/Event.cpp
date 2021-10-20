/*
 * Event.cpp
 *
 *  Created on: 18 Oct 2021
 *      Author: David
 */

#include <Platform/Event.h>
#include <RepRapFirmware.h>

Event::Event(Event *p_next, EventType et, EventParameter p_param, CanAddress p_ba, uint8_t devNum) noexcept
	: next(p_next), param(p_param), type(et), boardAddress(p_ba), deviceNumber(devNum)
{
}

void Event::AppendText(const StringRef &str) const noexcept
{
	// First append the event type with underscores changed to spaces
	const char *p = type.ToString();
	while (*p != 0)
	{
		str.cat((*p == '_') ? ' ' : *p);
		++p;
	}

	// Now append further details of the event
	switch (type.ToBaseType())
	{
	case EventType::Heater_fault:
		str.catf("on heater %u: %s", deviceNumber, HeaterFaultType(param.heaterFaultStatus).ToString());
		break;

	case EventType::Driver_error:
	case EventType::Driver_warning:
#if SUPPORT_CAN_EXPANSION
		str.catf(" on %u.%u", boardAddress, deviceNumber);
#else
		str.catf(" on %u", deviceNumber);
#endif
		param.driverStatus.AppendText(str, (type == EventType::Driver_error) ? 2 : 1);
		break;

	case EventType::Filament_error:
		str.catf(" on extruder %u: %s", deviceNumber, FilamentSensorStatus(param.filamentStatus).ToString());
		break;

	case EventType::Main_board_power_failure:
		break;

	case EventType::Trigger:
		str.catf(" %u activated", deviceNumber);
		break;

	case EventType::Mcu_temperature_warning:
#if SUPPORT_CAN_EXPANSION
		str.catf("on board %u: temperature %.1fC", boardAddress, (double)param.fVal);
#else
		str.catf(": temperature %.1fC", (double)param.fVal);
#endif
		break;
	}
}

// Append the name of the macro that we run when this event occurs
void Event::GetMacroFileName(const StringRef& fname) const noexcept
{
	const char *p = type.ToString();
	fname.cat((char)tolower(*p++));
	while (*p != 0)
	{
		if (*p == '_' && p[1] != 0)
		{
			fname.cat(toupper(p[1]));
			p += 2;
		}
		else
		{
			fname.cat(*p++);
		}
	}
	fname.cat(".g");
}

// End
