/*
 * Fan.cpp
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#include "Fan.h"
#include "Platform.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"
#include "Heating/Heat.h"

void Fan::Init(Pin p_pin, bool hwInverted)
{
	isConfigured = false;
	val = lastVal = 0.0;
	minVal = 0.1;				// 10% minimum fan speed
	blipTime = 100;				// 100ms fan blip
	freq = DefaultFanPwmFreq;
	pin = p_pin;
	hardwareInverted = hwInverted;
	inverted = blipping = false;
	heatersMonitored = 0;
	triggerTemperatures[0] = triggerTemperatures[1] = HOT_END_FAN_TEMPERATURE;
	lastPwm = -1.0;				// force a refresh
	Refresh();
}

// Set or report the parameters for this fan
// If 'mcode' is an M-code used to set parameters for the current kinematics (which should only ever be 106)
// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
// Exceptions:
// 1. Only process the S parameter unless other values were processed.
// 2. Don't process the R parameter, but if it is present don't print the existing configuration.
bool Fan::Configure(unsigned int mcode, int fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error)
{
	if (!IsEnabled())
	{
		reply.printf("Fan %d is disabled", fanNum);
		error = true;
		return true;											// say we have processed it
	}

	bool seen = false;
	if (mcode == 106)
	{
		if (gb.Seen('I'))		// Invert cooling
		{
			seen = true;
			const int invert = gb.GetIValue();
			if (invert < 0)
			{
				Disable();
			}
			else
			{
				inverted = (invert > 0);
			}
		}

		if (gb.Seen('F'))										// Set PWM frequency
		{
			seen = true;
			freq = (PwmFrequency)constrain<int>(gb.GetIValue(), 1, 65535);
			lastPwm = -1.0;										// force the PWM to be updated
			Refresh();
		}

		if (gb.Seen('T'))
		{
			seen = true;
			size_t numTemps = 2;
			gb.GetFloatArray(triggerTemperatures, numTemps, true);
		}

		if (gb.Seen('B'))										// Set blip time
		{
			seen = true;
			blipTime = (uint32_t)(max<float>(gb.GetFValue(), 0.0) * SecondsToMillis);
		}

		if (gb.Seen('L'))		// Set minimum speed
		{
			seen = true;
			float speed = gb.GetFValue();
			if (speed > 1.0)
			{
				speed /= 255.0;
			}
			minVal = constrain<float>(speed, 0.0, 1.0);
		}

		if (gb.Seen('H'))		// Set thermostatically-controlled heaters
		{
			seen = true;
			int32_t heaters[Heaters + MaxVirtualHeaters];		// signed because we use H-1 to disable thermostatic mode
			size_t numH = ARRAY_SIZE(heaters);
			gb.GetIntArray(heaters, numH, false);

			// Note that M106 H-1 disables thermostatic mode. The following code implements that automatically.
			heatersMonitored = 0;
			for (size_t h = 0; h < numH; ++h)
			{
				const int hnum = heaters[h];
				if (hnum >= 0 && hnum < (int)Heaters)
				{
					SetBit(heatersMonitored, (unsigned int)hnum);
				}
				else if (hnum >= (int)FirstVirtualHeater && hnum < (int)(FirstVirtualHeater + MaxVirtualHeaters))
				{
					// Heaters 100, 101... are virtual heaters i.e. CPU and driver temperatures
					SetBit(heatersMonitored, Heaters + (unsigned int)hnum - FirstVirtualHeater);
				}
			}
			if (heatersMonitored != 0)
			{
				SetPwm(1.0);			// default the fan speed to full for safety
			}
		}

		// We only act on the 'S' parameter here if we have processed other parameters
		if (seen && gb.Seen('S'))		// Set new fan value - process this after processing 'H' or it may not be acted on
		{
			const float f = constrain<float>(gb.GetFValue(), 0.0, 255.0);
			SetPwm(f);
		}

		if (seen)
		{
			isConfigured = true;
			Refresh();
		}
		else if (!gb.Seen('R') && !gb.Seen('S'))
		{
			// Report the configuration of the specified fan
			reply.printf("Fan%i frequency: %uHz, speed: %d%%, min: %d%%, blip: %.2f, inverted: %s",
							fanNum,
							(unsigned int)freq,
							(int)(val * 100.0),
							(int)(minVal * 100.0),
							(double)(blipTime * MillisToSeconds),
							(inverted) ? "yes" : "no");
			if (heatersMonitored != 0)
			{
				reply.catf(", temperature: %.1f:%.1fC, heaters:", (double)triggerTemperatures[0], (double)triggerTemperatures[1]);
				for (unsigned int i = 0; i < Heaters + MaxVirtualHeaters; ++i)
				{
					if (IsBitSet(heatersMonitored, i))
					{
						reply.catf(" %u", (i < Heaters) ? i : FirstVirtualHeater + i - Heaters);
					}
				}
				reply.catf(", current speed: %d%%:", (int)(lastVal * 100.0));
			}
		}
	}

	return seen;
}

void Fan::SetPwm(float speed)
{
	if (speed > 1.0)
	{
		speed /= 255.0;
	}
	val = constrain<float>(speed, 0.0, 1.0);
	Refresh();
}

// Set the hardware PWM
// If you want make sure that the PWM is definitely updated, set lastPWM negative before calling this
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
			IoPort::WriteAnalog(pin, pwmVal, freq);
		}
	}
}

void Fan::SetHeatersMonitored(HeatersMonitoredBitmap h)
{
	heatersMonitored = h;
	Refresh();
}

// Refresh the fan PWM
// If you want make sure that the PWM is definitely updated, set lastPWM negative before calling this
void Fan::Refresh()
{
	float reqVal;
#if HAS_SMART_DRIVERS
	uint32_t driverChannelsMonitored = 0;
#endif

	if (heatersMonitored == 0)
	{
		reqVal = val;
	}
	else
	{
		reqVal = 0.0;
		const bool bangBangMode = (triggerTemperatures[1] <= triggerTemperatures[0]);
		for (size_t h = 0; h < Heaters + MaxVirtualHeaters; ++h)
		{
			// Check if this heater is both monitored by this fan and in use
			if (   IsBitSet(heatersMonitored, h)
				&& (h < reprap.GetToolHeatersInUse() || (h >= Heaters && h < Heaters + MaxVirtualHeaters) || reprap.GetHeat().IsBedOrChamberHeater(h))
			   )
			{
				// This heater is both monitored and potentially active
				if (h < Heaters && reprap.GetHeat().IsTuning(h))
				{
					reqVal = 1.0;			// when turning the PID for a monitored heater, turn the fan on
				}
				else
				{
					const size_t heaterHumber = (h >= Heaters) ? (h - Heaters) + FirstVirtualHeater : h;
					TemperatureError err;
					const float ht = reprap.GetHeat().GetTemperature(heaterHumber, err);
					if (err != TemperatureError::success || ht < BAD_LOW_TEMPERATURE || ht >= triggerTemperatures[1])
					{
						reqVal = max<float>(reqVal, (bangBangMode) ? max<float>(0.5, val) : 1.0);
					}
					else if (!bangBangMode && ht > triggerTemperatures[0])
					{
						// We already know that ht < triggerTemperatures[1], therefore unless we have NaNs it is safe to divide by (triggerTemperatures[1] - triggerTemperatures[0])
						reqVal = max<float>(reqVal, (ht - triggerTemperatures[0])/(triggerTemperatures[1] - triggerTemperatures[0]));
					}
					else if (lastVal != 0.0 && ht + ThermostatHysteresis > triggerTemperatures[0])
					{
						// If the fan is on, add a hysteresis before turning it off
						reqVal = max<float>(reqVal, (bangBangMode) ? max<float>(0.5, val) : minVal);
					}
#if HAS_SMART_DRIVERS
					const unsigned int channel = reprap.GetHeat().GetHeaterChannel(heaterHumber);
					if (channel >= FirstTmcDriversSenseChannel && channel < FirstTmcDriversSenseChannel + NumTmcDriversSenseChannels)
					{
						driverChannelsMonitored |= 1 << (channel - FirstTmcDriversSenseChannel);
					}
#endif
				}
			}
		}
	}

	if (reqVal > 0.0)
	{
		if (reqVal < minVal)
		{
			reqVal = minVal;
		}

		if (lastVal == 0.0)
		{
			// We are turning this fan on
#if HAS_SMART_DRIVERS
			if (driverChannelsMonitored != 0)
			{
				reprap.GetPlatform().DriverCoolingFansOn(driverChannelsMonitored);		// tell Platform that we have started a fan that cools drivers
			}
#endif
			if (reqVal < 1.0 && blipTime != 0)
			{
				// Starting the fan from standstill, so blip the fan
				blipping = true;
				blipStartTime = millis();
			}
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
	lastVal = reqVal;
}

bool Fan::Check()
{
	if (heatersMonitored != 0 || blipping)
	{
		Refresh();
	}
	return heatersMonitored != 0 && lastVal != 0.0;
}

void Fan::Disable()
{
	if (pin != NoPin)
	{
		inverted = false;
		lastPwm = -1;
		SetHardwarePwm(0.0);
	}
	pin = NoPin;
}

// Save the settings of this fan if it isn't thermostatic
bool Fan::WriteSettings(FileStore *f, size_t fanNum) const
{
	if (heatersMonitored == 0)
	{
		char bufSpace[50];
		StringRef buf(bufSpace, ARRAY_SIZE(bufSpace));
		buf.printf("M106 P%u S%.2f\n", fanNum, (double)val);
		return f->Write(buf.c_str());
	}

	return true;
}

// End
