/*
 * Duet3DFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "Duet3DFilamentSensor.h"
#include "GCodes/GCodeBuffer.h"

// Constructors
Duet3DFilamentSensor::Duet3DFilamentSensor(int type) : FilamentSensor(type), mmPerRev(DefaultMmPerRev), tolerance(DefaultTolerance), withSwitch(type == 4)
{
}

// Configure this sensor returning true if error
bool Duet3DFilamentSensor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, seen))
	{
		return true;
	}

	gb.TryGetFValue('S', mmPerRev, seen);

	if (gb.Seen('R'))
	{
		seen = true;
		const float tol = gb.GetFValue();
		if (tolerance < 0.0 || tolerance >= 1.0)
		{
			reply.copy("Tolerance must be between 0 and 1");
			return true;
		}
		tolerance = tol;
	}

	if (!seen)
	{
		reply.printf("Duet3D filament sensor (%s microswitch), %.1fmm per rev, tolerance %.2f", (withSwitch) ? "with" : "no", mmPerRev, tolerance);
	}

	return false;
}

// ISR for when the pin state changes
void Duet3DFilamentSensor::Interrupt()
{
	//TODO
}

// End
