/*
 * SimpleFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "SimpleFilamentMonitor.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

SimpleFilamentMonitor::SimpleFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: FilamentMonitor(drv, monitorType, did), highWhenNoFilament(monitorType == 2), filamentPresent(false)
{
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult SimpleFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException)
{
	const GCodeResult rslt = CommonConfigure(gb, reply, InterruptMode::none, seen);
	if (Succeeded(rslt))
	{
		if (seen)
		{
			Check(false, false, 0, 0.0);
			reprap.SensorsUpdated();
		}
		else
		{
			reply.copy("Simple filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(GetEnableMode() == 2) ? "enabled always" : (GetEnableMode() == 1) ? "enabled when printing from SD card" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}

// ISR for when the pin state changes
bool SimpleFilamentMonitor::Interrupt() noexcept
{
	// Nothing needed here
	GetPort().DetachInterrupt();
	return false;
}

// Call the following regularly to keep the status up to date
void SimpleFilamentMonitor::Poll() noexcept
{
	const bool b = GetPort().ReadDigital();
	filamentPresent = (highWhenNoFilament) ? !b : b;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus SimpleFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
{
	Poll();
	return (GetEnableMode() == 0 || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus SimpleFilamentMonitor::Clear() noexcept
{
	Poll();
	return (GetEnableMode() == 0 || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Print diagnostic info for this sensor
void SimpleFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder) noexcept
{
	Poll();
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: %s\n", extruder, (filamentPresent) ? "ok" : "no filament");
}

#if SUPPORT_REMOTE_COMMANDS

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult SimpleFilamentMonitor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	const GCodeResult rslt = CommonConfigure(parser, reply, InterruptMode::none, seen);
	if (rslt <= GCodeResult::warning)
	{
		if (seen)
		{
			Check(false, false, 0, 0.0);
		}
		else
		{
			reply.copy("Simple filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(GetEnableMode() != 0) ? "enabled" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}

// Store collected data in a CAN message slot
void SimpleFilamentMonitor::GetLiveData(FilamentMonitorDataNew& data) const noexcept
{
	data.hasLiveData = false;
}

// Print diagnostic info for this sensor
void SimpleFilamentMonitor::Diagnostics(const StringRef& reply) noexcept
{
	Poll();
	reply.lcatf("Driver %u: %s", GetDriver(), (filamentPresent) ? "ok" : "no filament");
}

#endif

#if SUPPORT_CAN_EXPANSION

void SimpleFilamentMonitor::UpdateLiveData(const FilamentMonitorDataNew& data) noexcept
{
	// nothing needed here
}

#endif

// End
