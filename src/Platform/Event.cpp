/*
 * Event.cpp
 *
 *  Created on: 18 Oct 2021
 *      Author: David
 */

#include <Platform/Event.h>
#include <RepRapFirmware.h>

Event::Event(Event *p_next, EventType et, EventParameter p_param, CanAddress p_ba, uint8_t devNum) noexcept : next(p_next), param(p_param), type(et), boardAddress(p_ba), deviceNumber(devNum)
{
	//TODO fill in other fields
}

void Event::AppendText(const StringRef &str) const noexcept
{
	str.cat(type.ToString());
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

	case EventType::Mcu_temperature_warning:
#if SUPPORT_CAN_EXPANSION
		str.catf("on board %u: temperature %.1fC", boardAddress, (double)param.fVal);
#else
		str.catf(": temperature %.1fC", (double)param.fVal);
#endif
		break;
	}
}

// End
