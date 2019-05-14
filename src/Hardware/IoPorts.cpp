/*
 * IoPort.cpp
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#include "IoPorts.h"
#include "RepRap.h"
#include "Platform.h"
#include "Configuration.h"

#ifdef DUET_NG
# include "DuetNG/DueXn.h"
#endif

// members of class IoPort
IoPort::IoPort()
{
	Clear();
}

void IoPort::Clear()
{
	logicalPort = NoLogicalPin;
	pin = NoPin;
	invert = false;
}

bool IoPort::Set(LogicalPin lp, PinAccess access, bool pInvert)
{
	const bool ret = reprap.GetPlatform().GetFirmwarePin(lp, access, pin, invert);
	if (ret)
	{
		if (pInvert)
		{
			invert = !invert;
		}
	}
	else
	{
		Clear();
	}
	return ret;
}

/*static*/ void IoPort::SetPinMode(Pin pin, PinMode mode)
{
#ifdef DUET_NG
	if (pin >= DueXnExpansionStart)
	{
		DuetExpansion::SetPinMode(pin, mode);
	}
	else
	{
		pinMode(pin, mode);
	}
#else
	pinMode(pin, mode);
#endif
}

/*static*/ bool IoPort::ReadPin(Pin pin)
{
#ifdef DUET_NG
	if (pin >= DueXnExpansionStart)
	{
		return DuetExpansion::DigitalRead(pin);
	}
	else
	{
		return digitalRead(pin);
	}
#else
	return digitalRead(pin);
#endif
}

/*static*/ void IoPort::WriteDigital(Pin pin, bool high)
{
#ifdef DUET_NG
	if (pin >= DueXnExpansionStart)
	{
		DuetExpansion::DigitalWrite(pin, high);
	}
	else
	{
		digitalWrite(pin, high);
	}
#else
	digitalWrite(pin, high);
#endif
}

/*static*/ void IoPort::WriteAnalog(Pin pin, float pwm, uint16_t freq)
{
#ifdef DUET_NG
	if (pin >= DueXnExpansionStart)
	{
		DuetExpansion::AnalogOut(pin, pwm);
	}
	else
	{
		AnalogOut(pin, pwm, freq);
	}
#else
	AnalogOut(pin, pwm, freq);
#endif
}

// Members of class PwmPort
PwmPort::PwmPort()
{
	frequency = DefaultPinWritePwmFreq;
}

void PwmPort::SetFrequency(float freq)
{
	frequency = (uint16_t)constrain<float>(freq, 1.0, 65535);
}

void PwmPort::WriteAnalog(float pwm) const
{
	if (pin != NoPin)
	{
		IoPort::WriteAnalog(pin, ((invert) ? 1.0 - pwm : pwm), frequency);
	}
}

// End
