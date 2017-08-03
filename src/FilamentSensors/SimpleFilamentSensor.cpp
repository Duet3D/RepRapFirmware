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

// Configure this sensor returning true if error
bool SimpleFilamentSensor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, seen))
	{
		return true;
	}

	if (seen)
	{
		Poll();
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
	const bool b = Platform::ReadPin(GetPin());
	filamentPresent = (highWhenNoFilament) ? !b : b;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
// Return nullptr if everything is OK, else an error reason to include in a message.
const char *SimpleFilamentSensor::Check(float filamentConsumed)
{
	return (filamentPresent) ? nullptr : "no filament";
}

// Print diagnostic info for this sensor
void SimpleFilamentSensor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: %s\n", extruder, (filamentPresent) ? "ok" : "no filament");
}

// End
