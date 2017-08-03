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

// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided and returning true if error.
// Also attaches the ISR.
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
			reply.copy("bad endstop number");
			return true;
		}
		endstopNumber = endstop;
		pin = p;
		attachInterrupt(pin, InterruptEntry, CHANGE, this);
	}
	else if (seen)
	{
		// We already had a P parameter, therefore it is an error not to have a C parameter too
		reply.copy("no endstop number given");
		return true;
	}
	return false;
}

// Factory function
/*static*/ FilamentSensor *FilamentSensor::Create(int type)
{
	switch (type)
	{
	case 1:		// active high switch
	case 2:		// active low switch
		return new SimpleFilamentSensor(type);
		break;

	case 3:		// duet3d, no switch
	case 4:		// duet3d + switch
		return new Duet3DFilamentSensor(type);
		break;

	default:	// no sensor, or unknown sensor
		return nullptr;
	}
}

// ISR
/*static*/ void FilamentSensor::InterruptEntry(void *param)
{
	static_cast<FilamentSensor*>(param)->Interrupt();
}

// End
