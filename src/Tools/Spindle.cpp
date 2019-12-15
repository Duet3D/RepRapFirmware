/*
 * Spindle.cpp
 *
 *  Created on: Mar 21, 2018
 *      Author: Christian
 */

#include "Spindle.h"

// Allocate the pins returning true if successful
bool Spindle::AllocatePins(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	IoPort * const ports[] = { &spindleForwardPort, &spindleReversePort };
	const PinAccess access[] = { PinAccess::pwm, PinAccess::pwm };
	return IoPort::AssignPorts(gb, reply, PinUsedBy::spindle, 2, ports, access);
}

void Spindle::SetFrequency(PwmFrequency freq) noexcept
{
	spindleForwardPort.SetFrequency(freq);
	spindleReversePort.SetFrequency(freq);
}

void Spindle::SetRpm(float rpm) noexcept
{
	const float pwm = abs(rpm / maxRpm);
	if (rpm >= 0.0)
	{
		spindleReversePort.WriteAnalog(0.0);
		spindleForwardPort.WriteAnalog(pwm);
	}
	else
	{
		spindleReversePort.WriteAnalog(pwm);
		spindleForwardPort.WriteAnalog(0.0);
	}
	currentRpm = configuredRpm = rpm;
}

void Spindle::TurnOff() noexcept
{
	spindleReversePort.WriteAnalog(0.0);
	spindleForwardPort.WriteAnalog(0.0);
	currentRpm = 0.0;
}

// End
