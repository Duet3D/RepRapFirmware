/*
 * SimpleFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "SimpleFilamentSensor.h"
#include "RepRap.h"
#include "Platform.h"

SimpleFilamentSensor::SimpleFilamentSensor(int type) : FilamentSensor(type), highWhenNoFilament(type == 2), filamentPresent(false)
{
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool SimpleFilamentSensor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, seen))
	{
		return true;
	}

	if (seen)
	{
		Check(true, 0.0);
	}
	else
	{
		reply.printf("Simple filament sensor on endstop %d, output %s when no filament", GetEndstopNumber(), (highWhenNoFilament) ? "high" : "low");
	}

	return false;
}

// ISR for when the pin state changes
void SimpleFilamentSensor::Interrupt()
{
	// Nothing needed here
	detachInterrupt(GetPin());
}

// Call the following regularly to keep the status up to date
void SimpleFilamentSensor::Poll()
{
	const bool b = IoPort::ReadPin(GetPin());
	filamentPresent = (highWhenNoFilament) ? !b : b;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus SimpleFilamentSensor::Check(bool full, float filamentConsumed)
{
	Poll();
	return (filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus SimpleFilamentSensor::Clear(bool full)
{
	Poll();
	return (filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Print diagnostic info for this sensor
void SimpleFilamentSensor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: %s\n", extruder, (filamentPresent) ? "ok" : "no filament");
}

// End
