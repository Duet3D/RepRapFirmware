/*
 * FilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "FilamentSensor.h"
#include "SimpleFilamentSensor.h"
#include "Duet3DFilamentSensor.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"

FilamentSensor::~FilamentSensor()
{
	if (pin != NoPin)
	{
		detachInterrupt(pin);
	}
}

// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided and returning true if error
bool FilamentSensor::ConfigurePin(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (gb.Seen('C'))
	{
		seen = true;
		// The C parameter is an endstop number in RRF
		const int endstop = gb.GetIValue();
		const Pin p = reprap.GetPlatform().GetEndstopPin(endstop);
		if (p == NoPin)
		{
			reply.copy("Bad endstop number");
			return true;
		}
		endstopNumber = endstop;
		pin = p;
	}
	return false;
}

// Factory function
FilamentSensor *FilamentSensor::Create(int type)
{
	switch (type)
	{
	case 1:		// active high switch
		return new SimpleFilamentSensor(type);
		break;

	case 2:		// active low switch
		return new SimpleFilamentSensor(type);
		break;

	case 3:		// duet3d, no switch
		return new Duet3DFilamentSensor(type);
		break;

	case 4:		// duet3d + switch
		return new Duet3DFilamentSensor(type);
		break;

	default:	// no sensor, or unknown sensor
		return nullptr;
	}
}

// End
