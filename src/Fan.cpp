/*
 * Fan.cpp
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#include "Fan.h"
#include "Platform.h"
#include "RepRap.h"

void Fan::Init(Pin p_pin, bool hwInverted)
{
	val = 0.0;
	minVal = 0.1;				// 10% minimum fan speed
	blipTime = 100;				// 100ms fan blip
	freq = DefaultFanPwmFreq;
	pin = p_pin;
	hardwareInverted = hwInverted;
	inverted = blipping = false;
	heatersMonitored = 0;
	triggerTemperature = HOT_END_FAN_TEMPERATURE;
	lastPwm = -1.0;
	Refresh();
}

void Fan::SetValue(float speed)
{
	if (speed > 1.0)
	{
		speed /= 255.0;
	}
	const float newVal = constrain<float>(speed, 0.0, 1.0);
	if (val == 0.0 && newVal > 0.0 && newVal < 1.0 && blipTime != 0)
	{
		// Starting the fan from standstill, so blip the fan
		blipping = true;
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
		if (invert)
		{
			pwmVal = 1.0 - pwmVal;
		}

		// Only set the PWM if it has changed, to avoid a lot of I2C traffic when we have a DueX5 connected
		if (pwmVal != lastPwm)
		{
			lastPwm = pwmVal;
			Platform::WriteAnalog(pin, pwmVal, freq);
		}
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
	if (reqVal > 0.0)
	{
		if (reqVal < minVal)
		{
			reqVal = minVal;
		}

		if (blipping)
		{
			if (millis() - blipStartTime < blipTime)
			{
				reqVal = 1.0;
			}
			else
			{
				blipping = false;
			}
		}
	}
	SetHardwarePwm(reqVal);
}

void Fan::Check()
{
	if (heatersMonitored != 0 || blipping)
	{
		Refresh();
	}
}

void Fan::Disable()
{
	if (pin != NoPin)
	{
		inverted = false;
		SetHardwarePwm(0.0);
	}
	pin = NoPin;
}

// End
