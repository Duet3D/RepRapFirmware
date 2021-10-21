/*
 * Fan.cpp
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#include "Fan.h"
#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Fan, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(Fan, _condition,__VA_ARGS__)

constexpr ObjectModelTableEntry Fan::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Fan members
	{ "actualValue",		OBJECT_MODEL_FUNC(self->GetPwm(), 2), 															ObjectModelEntryFlags::live },
	{ "blip",				OBJECT_MODEL_FUNC(0.001f * (float)self->blipTime, 2), 											ObjectModelEntryFlags::none },
	{ "frequency",			OBJECT_MODEL_FUNC((int32_t)self->GetPwmFrequency()), 											ObjectModelEntryFlags::none },
	{ "max",				OBJECT_MODEL_FUNC(self->maxVal, 2), 															ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->minVal, 2), 															ObjectModelEntryFlags::none },
	{ "name",				OBJECT_MODEL_FUNC(self->name.c_str()), 															ObjectModelEntryFlags::none },
	{ "requestedValue",		OBJECT_MODEL_FUNC(self->val, 2), 																ObjectModelEntryFlags::live },
	{ "rpm",				OBJECT_MODEL_FUNC(self->GetRPM()), 																ObjectModelEntryFlags::live },
	{ "thermostatic",		OBJECT_MODEL_FUNC(self, 1), 																	ObjectModelEntryFlags::none },

	// 1. Fan.thermostatic members
	{ "heaters",			OBJECT_MODEL_FUNC(self->sensorsMonitored),														ObjectModelEntryFlags::none },	// empty if not thermostatic
	{ "highTemperature",	OBJECT_MODEL_FUNC_IF(self->sensorsMonitored.IsNonEmpty(), self->triggerTemperatures[1], 1), 	ObjectModelEntryFlags::none },
	{ "lowTemperature",		OBJECT_MODEL_FUNC_IF(self->sensorsMonitored.IsNonEmpty(), self->triggerTemperatures[0], 1), 	ObjectModelEntryFlags::none },
};

constexpr uint8_t Fan::objectModelTableDescriptor[] = { 2, 9, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE(Fan)

#endif

Fan::Fan(unsigned int fanNum) noexcept
	: fanNumber(fanNum),
	  val(0.0),
	  minVal(DefaultMinFanPwm),
	  maxVal(1.0),										// 100% maximum fan speed
	  blipTime(DefaultFanBlipTime)
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
bool Fan::Configure(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
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
			minVal = min<float>(gb.GetPwmValue(), maxVal);
		}

		if (gb.Seen('X'))		// Set maximum speed
		{
			seen = true;
			maxVal = max<float>(gb.GetPwmValue(), minVal);
		}

		if (gb.Seen('H'))		// Set thermostatically-controlled sensors
		{
			seen = true;
			int32_t sensors[MaxSensors];			// signed because we use H-1 to disable thermostatic mode
			size_t numH = ARRAY_SIZE(sensors);
			gb.GetIntArray(sensors, numH, false);

			// Note that M106 H-1 disables thermostatic mode. The following code implements that automatically.
			sensorsMonitored.Clear();
			for (size_t h = 0; h < numH; ++h)
			{
				const int hnum = sensors[h];
				if (hnum >= 0)
				{
					if (hnum < (int)MaxSensors)
					{
						sensorsMonitored.SetBit((unsigned int)hnum);
					}
					else
					{
						reply.copy("Sensor number out of range");
						error = true;
					}
				}
			}
			if (sensorsMonitored.IsNonEmpty())
			{
				val = 1.0;					// default the fan speed to full for safety
			}
		}

		if (gb.Seen('C'))
		{
			seen = true;
			gb.GetQuotedString(name.GetRef());
		}

		if (seen)
		{
			// We only act on the 'S' parameter here if we have processed other parameters
			if (seen && gb.Seen('S'))		// Set new fan value - process this after processing 'H' or it may not be acted on
			{
				val = gb.GetPwmValue();
			}

			if (!UpdateFanConfiguration(reply))
			{
				error = true;
			}
			reprap.FansUpdated();
		}
		else if (!gb.Seen('R') && !gb.Seen('S'))
		{
			// Report the configuration of the specified fan
			reply.printf("Fan %u", fanNum);
			if (name.strlen() != 0)
			{
				reply.catf(" (%s)", name.c_str());
			}
			if (sensorsMonitored.IsEmpty())
			{
				reply.catf(", speed %d%%", (int)(val * 100.0));
			}
			reply.catf(", min: %d%%, max: %d%%, blip: %.2f",
						(int)(minVal * 100.0),
						(int)(maxVal * 100.0),
						(double)(blipTime * MillisToSeconds)
					  );
			if (sensorsMonitored.IsNonEmpty())
			{
				reply.catf(", temperature: %.1f:%.1fC, sensors:", (double)triggerTemperatures[0], (double)triggerTemperatures[1]);
				sensorsMonitored.Iterate([&reply](unsigned int sensorNum, unsigned int) noexcept { reply.catf(" %u", sensorNum); });
				reply.cat(", current speed: ");
				const float lastVal = GetPwm();
				if (lastVal >= 0.0)
				{
					reply.catf("%d%%:", (int)(lastVal * 100.0));
				}
				else
				{
					reply.cat("unknown");
				}
			}
		}
	}

	return seen;
}

// Set the PWM. 'speed' is in the interval 0.0..1.0.
GCodeResult Fan::SetPwm(float speed, const StringRef& reply) noexcept
{
	val = speed;
	return Refresh(reply);
}

#if SUPPORT_REMOTE_COMMANDS

// Set the parameters for this fan
GCodeResult Fan::Configure(const CanMessageFanParameters& msg, const StringRef& reply) noexcept
{
	triggerTemperatures[0] = msg.triggerTemperatures[0];
	triggerTemperatures[1] = msg.triggerTemperatures[1];
	blipTime = msg.blipTime;
	val = msg.val;
	minVal = msg.minVal;
	maxVal = msg.maxVal;
	sensorsMonitored.SetFromRaw(msg.sensorsMonitored);
	(void)UpdateFanConfiguration(reply);
	return GCodeResult::ok;
}

#endif

#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE

// Save the settings of this fan if it isn't thermostatic
bool Fan::WriteSettings(FileStore *f, size_t fanNum) const noexcept
{
	if (sensorsMonitored.IsEmpty())
	{
		String<StringLength20> fanCommand;
		fanCommand.printf("M106 P%u S%.2f\n", fanNum, (double)val);
		return f->Write(fanCommand.c_str());
	}
	return true;
}

#endif

// End
