/*
 * Fan.cpp
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#include "RepRapFirmware.h"

void Fan::Init(Pin p_pin, bool hwInverted)
{
	val = 0.0;
	minVal = 0.1;				// 10% minimum fan speed
	blipTime = 100;				// 100ms fan blip
	blipStartTime = 0;
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
	const float newVal = (speed > 0.0) ? constrain<float>(speed, minVal, 1.0) : 0.0;
	if (val == 0.0 && newVal < 1.0 && blipTime != 0)
	{
		// Starting the fan from standstill, so blip the fan
		blipStartTime = millis();
	}
	val = newVal;
	Refresh();
}

void Fan::SetMinValue(float speed)
{
	if (speed > 1.0)
	{
		speed /= 255.0;
	}
	minVal = constrain<float>(speed, 0.0, 1.0);
	Refresh();
}

void Fan::SetBlipTime(float t)
{
	blipTime = (uint32_t)(max<float>(t, 0.0) * SecondsToMillis);
}

void Fan::SetInverted(bool inv)
{
	inverted = inv;
	Refresh();
}

void Fan::SetHardwarePwm(float pwmVal)
{
	if (pin != NoPin)
	{
		bool invert = hardwareInverted;
		if (inverted)
		{
			invert = !invert;
		}
		Platform::WriteAnalog(pin, (invert) ? (1.0 - pwmVal) : pwmVal, freq);
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
	float reqVal = (heatersMonitored == 0)
					? val
					: (reprap.GetPlatform()->AnyHeaterHot(heatersMonitored, triggerTemperature))
						? max<float>(0.5, val)			// make sure that thermostatic fans always run at 50% speed or more
						: 0.0;
	if (reqVal > 0.0 && millis() - blipStartTime < blipTime)
	{
		SetHardwarePwm(1.0);
	}
	else if (reqVal > 0.0 && reqVal < minVal)
	{
		SetHardwarePwm(minVal);
	}
	else
	{
		SetHardwarePwm(reqVal);
	}
}

void Fan::Check()
{
	if (heatersMonitored != 0 || blipTime != 0)
	{
		Refresh();
	}
}

// End
