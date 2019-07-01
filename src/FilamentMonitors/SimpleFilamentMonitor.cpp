/*
 * SimpleFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "SimpleFilamentMonitor.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

SimpleFilamentMonitor::SimpleFilamentMonitor(unsigned int extruder, unsigned int type)
	: FilamentMonitor(extruder, type), highWhenNoFilament(type == 2), filamentPresent(false), enabled(false)
{
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool SimpleFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, INTERRUPT_MODE_NONE, seen))
	{
		return true;
	}

	if (gb.Seen('S'))
	{
		seen = true;
		enabled = (gb.GetIValue() > 0);
	}

	if (seen)
	{
		Check(false, false, 0, 0.0);
	}
	else
	{
		reply.copy("Simple filament sensor on pin ");
		GetPort().AppendPinName(reply);
		reply.catf(", %s, output %s when no filament, filament present: %s",
					(enabled) ? "enabled" : "disabled",
					(highWhenNoFilament) ? "high" : "low",
					(filamentPresent) ? "yes" : "no");
	}

	return false;
}

// ISR for when the pin state changes
bool SimpleFilamentMonitor::Interrupt()
{
	// Nothing needed here
	GetPort().DetachInterrupt();
	return false;
}

// Call the following regularly to keep the status up to date
void SimpleFilamentMonitor::Poll()
{
	const bool b = GetPort().Read();
	filamentPresent = (highWhenNoFilament) ? !b : b;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus SimpleFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed)
{
	Poll();
	return (!enabled || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus SimpleFilamentMonitor::Clear()
{
	Poll();
	return (!enabled || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Print diagnostic info for this sensor
void SimpleFilamentMonitor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	Poll();
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: %s\n", extruder, (filamentPresent) ? "ok" : "no filament");
}

// End
