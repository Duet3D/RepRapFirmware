/*
 * Fan.cpp
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#include "Fan.h"
#include "RepRapFirmware.h"

void Fan::Init(Pin p_pin, bool hwInverted)
{
	val = 0.0;
	freq = DefaultFanPwmFreq;
	pin = p_pin;
	hardwareInverted = hwInverted;
	inverted = false;
	heatersMonitored = 0;
	triggerTemperature = HOT_END_FAN_TEMPERATURE;
	Refresh();
}

void Fan::SetValue(float speed)
{
	if (speed > 1.0)
	{
		speed /= 255.0;
	}
	val = constrain<float>(speed, 0.0, 1.0);
	Refresh();
}

void Fan::SetInverted(bool inv)
{
	inverted = inv;
	Refresh();
}

void Fan::SetHardwarePwm(float pwmVal)
{
	if (pin >= 0)
	{
		uint32_t p = (uint32_t)(255.0 * pwmVal);
		bool invert = hardwareInverted;
		if (inverted)
		{
			invert = !invert;
		}
		AnalogWrite(pin, (invert) ? (255 - p) : p, freq);
	}
}

void Fan::SetPwmFrequency(float p_freq)
{
	freq = (uint16_t)constrain<float>(p_freq, 1.0, 65535.0);
	Refresh();
}

void Fan::SetHeatersMonitored(uint16_t h)
{
	heatersMonitored = h;
	Refresh();
}

void Fan::Refresh()
{
	if (heatersMonitored != 0)
	{
		const float pwmVal = (reprap.GetPlatform()->AnyHeaterHot(heatersMonitored, triggerTemperature))
				? max<float>(0.5, val)			// make sure that thermostatic fans always run at 50% speed or more
				: 0.0;
		SetHardwarePwm(pwmVal);
	}
	else
	{
		SetHardwarePwm(val);
	}
}

void Fan::Check()
{
	if (heatersMonitored != 0)
	{
		Refresh();
	}
}

// End
