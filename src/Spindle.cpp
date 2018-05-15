/*
 * Spindle.cpp
 *
 *  Created on: Mar 21, 2018
 *      Author: Christian
 */

#include "Spindle.h"

bool Spindle::SetPins(LogicalPin lpf, LogicalPin lpr, bool invert)
{
	const bool ok1 = spindleForwardPort.Set(lpf, PinAccess::pwm, invert);
	if (lpr == NoLogicalPin)
	{
		spindleReversePort.Clear();
		return ok1;
	}
	const bool ok2 = spindleReversePort.Set(lpr, PinAccess::pwm, invert);
	return ok1 && ok2;
}

void Spindle::GetPins(LogicalPin& lpf, LogicalPin& lpr, bool& invert) const
{
	lpf = spindleForwardPort.GetLogicalPin(invert);
	lpr = spindleReversePort.GetLogicalPin();
}

void Spindle::SetPwmFrequency(float freq)
{
	spindleReversePort.SetFrequency(freq);
	spindleForwardPort.SetFrequency(freq);
}

void Spindle::SetRpm(float rpm)
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

void Spindle::TurnOff()
{
	spindleReversePort.WriteAnalog(0.0);
	spindleForwardPort.WriteAnalog(0.0);
	currentRpm = 0.0;
}

// End
