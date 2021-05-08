/*
 * InputShaper.cpp
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#include "InputShaper.h"

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include "StepTimer.h"

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(InputShaper, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(InputShaper, __VA_ARGS__)

constexpr ObjectModelTableEntry InputShaper::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. InputShaper members
	{ "damping",				OBJECT_MODEL_FUNC(self->GetFloatDamping(), 2), 							ObjectModelEntryFlags::none },
	{ "frequency",				OBJECT_MODEL_FUNC(self->GetFrequency(), 2), 							ObjectModelEntryFlags::none },
	{ "minimumAcceleration",	OBJECT_MODEL_FUNC(self->minimumAcceleration, 1),						ObjectModelEntryFlags::none },
	{ "type", 					OBJECT_MODEL_FUNC(self->type.ToString()), 								ObjectModelEntryFlags::none },
};

constexpr uint8_t InputShaper::objectModelTableDescriptor[] = { 1, 4 };

DEFINE_GET_OBJECT_MODEL_TABLE(InputShaper)

InputShaper::InputShaper() noexcept
	: halfPeriod((uint16_t)lrintf(StepTimer::StepClockRate/(2 * DefaultFrequency))),
	  damping(lrintf(DefaultDamping * 65536)),
	  minimumAcceleration(DefaultMinimumAcceleration),
	  type(InputShaperType::none)
{
}

// Process M593
GCodeResult InputShaper::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const float MinimumInputShapingFrequency = (float)StepTimer::StepClockRate/(2 * 65535);			// we use a 16-bit number of step clocks to represent half the input shaping period
	const float MaximumInputShapingFrequency = 1000.0;
	bool seen = false;
	if (gb.Seen('F'))
	{
		seen = true;
		halfPeriod = (float)StepTimer::StepClockRate/(2 * gb.GetLimitedFValue('F', MinimumInputShapingFrequency, MaximumInputShapingFrequency));
	}
	if (gb.Seen('L'))
	{
		seen = true;
		minimumAcceleration = max<float>(gb.GetFValue(), 1.0);		// very low accelerations cause problems with the maths
	}
	if (gb.Seen('S'))
	{
		seen = true;
		damping = (uint16_t)lrintf(63336 * gb.GetLimitedFValue('S', 0.0, 0.99));
	}

	if (gb.Seen('P'))
	{
		String<StringLength20> shaperName;
		gb.GetReducedString(shaperName.GetRef());
		const InputShaperType newType(shaperName.c_str());
		if (!newType.IsValid())
		{
			reply.printf("Unsupported input shaper type '%s'", shaperName.c_str());
			return GCodeResult::error;
		}
		seen = true;
		type = newType;
	}
	else if (seen && type == InputShaperType::none)
	{
		// For backwards compatibility, if we have set input shaping parameters but not defined shaping type, default to DAA for now. Change this when we support better types of input shaping.
		type = InputShaperType::daa;
	}

	if (seen)
	{
		reprap.MoveUpdated();
	}
	else if (type != InputShaperType::none)
	{
		reply.printf("Input shaping '%s' at %.1fHz damping factor %.2f, min. acceleration %.1f",
						type.ToString(), (double)GetFrequency(), (double)GetFloatDamping(), (double)minimumAcceleration);
	}
	else
	{
		reply.copy("Input shaping is disabled");
	}
	return GCodeResult::ok;
}

// Return the full period in seconds
float InputShaper::GetFullPeriod() const noexcept
{
	return (float)halfPeriod/(float)(StepTimer::StepClockRate/2);
}

float InputShaper::GetFrequency() const noexcept
{
	return (float)StepTimer::StepClockRate/(2.0 * (float)halfPeriod);
}

float InputShaper::GetFloatDamping() const noexcept
{
	return ((float)damping)/65536;
}

// End
