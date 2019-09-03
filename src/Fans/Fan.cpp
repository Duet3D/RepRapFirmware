/*
 * Fan.cpp
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#include "Fan.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

Fan::Fan(unsigned int fanNum)
	: number(fanNum),
	  val(0.0), lastVal(0.0),
	  minVal(DefaultMinFanPwm),
	  maxVal(1.0),										// 100% maximum fan speed
	  blipTime(DefaultFanBlipTime),
	  sensorsMonitored(0),
	  isConfigured(false)
{
	triggerTemperatures[0] = triggerTemperatures[1] = DefaultHotEndFanTemperature;
}

// Set or report the parameters for this fan
// If 'mcode' is an M-code used to set parameters for the f (which should only ever be 106)
// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
// Exceptions:
// 1. Only process the S parameter if other values were processed.
// 2. Don't process the R parameter, but if it is present don't print the existing configuration.
bool Fan::Configure(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error)
{
	bool seen = false;
	if (mcode == 106)
	{
		// Check that the fan is enabled
		if (!IsEnabled())
		{
			reply.printf("Fan %u is disabled", fanNum);
			error = true;
			return true;											// say we have processed it
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
			minVal = constrain<float>(speed, 0.0, maxVal);
		}

		if (gb.Seen('X'))		// Set maximum speed
		{
			seen = true;
			float speed = gb.GetFValue();
			if (speed > 1.0)
			{
				speed /= 255.0;
			}
			maxVal = constrain<float>(speed, minVal, 1.0);
		}

		if (gb.Seen('H'))		// Set thermostatically-controlled sensors
		{
			seen = true;
			int32_t sensors[MaxSensorsInSystem];		// signed because we use H-1 to disable thermostatic mode
			size_t numH = ARRAY_SIZE(sensors);
			gb.GetIntArray(sensors, numH, false);

			// Note that M106 H-1 disables thermostatic mode. The following code implements that automatically.
			sensorsMonitored = 0;
			for (size_t h = 0; h < numH; ++h)
			{
				const int hnum = sensors[h];
				if (hnum >= 0)
				{
					if (hnum < (int)MaxSensorsInSystem)
					{
						SetBit(sensorsMonitored, (unsigned int)hnum);
					}
					else
					{
						reply.copy("Sensor number out of range");
						error = true;
					}
				}
			}
			if (sensorsMonitored != 0)
			{
				SetPwm(1.0);			// default the fan speed to full for safety
			}
		}

		if (gb.Seen('C') && gb.GetQuotedString(name.GetRef()))
		{
			seen = true;
		}

		// We only act on the 'S' parameter here if we have processed other parameters
		if (seen && gb.Seen('S'))		// Set new fan value - process this after processing 'H' or it may not be acted on
		{
			SetPwm(gb.GetPwmValue());
		}

		if (seen)
		{
			isConfigured = true;
			UpdateFanConfiguration();
			Refresh();
		}
		else if (!gb.Seen('R') && !gb.Seen('S'))
		{
			// Report the configuration of the specified fan
			reply.printf("Fan %u", fanNum);
			if (name.strlen() != 0)
			{
				reply.catf(" (%s)", name.c_str());
			}
			reply.catf(", speed: %d%%, min: %d%%, max: %d%%, blip: %.2f",
						(int)(val * 100.0),
						(int)(minVal * 100.0),
						(int)(maxVal * 100.0),
						(double)(blipTime * MillisToSeconds)
					  );
			if (sensorsMonitored != 0)
			{
				reply.catf(", temperature: %.1f:%.1fC, sensors:", (double)triggerTemperatures[0], (double)triggerTemperatures[1]);
				for (unsigned int i = 0; i < MaxSensorsInSystem; ++i)
				{
					if (IsBitSet(sensorsMonitored, i))
					{
						reply.catf(" %u", i);
					}
				}
				reply.catf(", current speed: %d%%:", (int)(lastVal * 100.0));
			}
		}
	}

	return seen;
}

// Set the PWM. 'speed' is in the interval 0.0..1.0.
void Fan::SetPwm(float speed)
{
	val = speed;
	Refresh();
}

void Fan::SetSensorsMonitored(SensorsBitmap h)
{
	sensorsMonitored = h;
	Refresh();
}

#if HAS_MASS_STORAGE

// Save the settings of this fan if it isn't thermostatic
bool Fan::WriteSettings(FileStore *f, size_t fanNum) const
{
	if (sensorsMonitored == 0)
	{
		String<StringLength20> fanCommand;
		fanCommand.printf("M106 P%u S%.2f\n", fanNum, (double)val);
		return f->Write(fanCommand.c_str());
	}
	return true;
}

#endif

// End
