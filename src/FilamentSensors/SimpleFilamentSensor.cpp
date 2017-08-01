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

// End
