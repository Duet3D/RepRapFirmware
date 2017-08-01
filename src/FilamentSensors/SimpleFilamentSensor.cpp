/*
 * SimpleFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include <FilamentSensors/SimpleFilamentSensor.h>

SimpleFilamentSensor::SimpleFilamentSensor(int type) : FilamentSensor(type), highWhenNoFilament(type == 2)
{
}

// Configure this sensor returning true if error
bool SimpleFilamentSensor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, seen))
	{
		return true;
	}

	if (!seen)
	{
		reply.printf("Simple filament sensor on endstop %d, output %s when no filament", GetEndstopNumber(), (highWhenNoFilament) ? "high" : "low");
	}

	return false;
}

// ISR for when the pin state changes
void SimpleFilamentSensor::Interrupt()
{
	//TODO
}

// End
