/*
 * Spindle.cpp
 *
 *  Created on: Mar 21, 2018
 *      Author: Christian
 */

#include "Spindle.h"
#include <RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Spindle, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(Spindle, __VA_ARGS__)

constexpr ObjectModelTableEntry Spindle::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Spindle members
	{ "active",			OBJECT_MODEL_FUNC(self->configuredRpm),			ObjectModelEntryFlags::none },
	{ "current",		OBJECT_MODEL_FUNC(self->currentRpm),			ObjectModelEntryFlags::live },
	{ "frequency",		OBJECT_MODEL_FUNC((int32_t)self->frequency),	ObjectModelEntryFlags::verbose },
	{ "max",			OBJECT_MODEL_FUNC(self->maxRpm),				ObjectModelEntryFlags::verbose },
	{ "min",			OBJECT_MODEL_FUNC(self->minRpm),				ObjectModelEntryFlags::verbose },
	{ "tool",			OBJECT_MODEL_FUNC((int32_t)self->toolNumber),	ObjectModelEntryFlags::verbose },
};

constexpr uint8_t Spindle::objectModelTableDescriptor[] = { 1, 6 };

DEFINE_GET_OBJECT_MODEL_TABLE(Spindle)

#endif

Spindle::Spindle() noexcept : currentRpm(0), configuredRpm(0), minRpm(DefaultMinSpindleRpm), maxRpm(DefaultMaxSpindleRpm), frequency(0), toolNumber(-1)
{
}

GCodeResult Spindle::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	if (gb.Seen('C'))
	{
		seen = true;
		IoPort * const ports[] = { &pwmPort, &onOffPort, &reverseNotForwardPort };
		const PinAccess access[] = { PinAccess::pwm, PinAccess::write0, PinAccess::write0 };
		if (IoPort::AssignPorts(gb, reply, PinUsedBy::spindle, 3, ports, access) == 0)
		{
			return GCodeResult::error;
		}
	}

	if (gb.Seen('Q'))
	{
		seen = true;
		frequency = gb.GetPwmFrequency();
		pwmPort.SetFrequency(frequency);
	}

	if (gb.Seen('R'))
	{
		seen = true;
		uint32_t rpm[2];
		size_t numValues = 2;
		gb.GetUnsignedArray(rpm, numValues, false);
		if (numValues == 2)
		{
			minRpm = (int32_t)rpm[0];
			maxRpm = (int32_t)rpm[1];
		}
		else
		{
			minRpm = DefaultMinSpindleRpm;
			maxRpm = (int32_t)rpm[0];
		}
	}

	if (gb.Seen('T'))
	{
		seen = true;
		toolNumber = gb.GetIValue();
	}

	if (seen)
	{
		reprap.SpindlesUpdated();
	}
	return GCodeResult::ok;
}

void Spindle::SetRpm(int32_t rpm) noexcept
{
	if (rpm > 0)
	{
		rpm = constrain<int>(rpm, minRpm, maxRpm);
		reverseNotForwardPort.WriteDigital(false);
		pwmPort.WriteAnalog((float)(rpm - minRpm) / (float)(maxRpm - minRpm));
		onOffPort.WriteDigital(true);
		currentRpm = rpm;					// current rpm is flagged live, so no need to change seqs.spindles
	}
	else if (rpm < 0)
	{
		rpm = constrain<int>(rpm, -maxRpm, -minRpm);
		reverseNotForwardPort.WriteDigital(true);
		pwmPort.WriteAnalog((float)(-rpm - minRpm) / (float)(maxRpm - minRpm));
		onOffPort.WriteDigital(true);
		currentRpm = rpm;					// current rpm is flagged live, so no need to change seqs.spindles
	}
	else
	{
		TurnOff();
	}

	if (configuredRpm != currentRpm)
	{
		configuredRpm = currentRpm;
		reprap.SpindlesUpdated();			// configuredRpm is not flagged live
	}
}

void Spindle::TurnOff() noexcept
{
	onOffPort.WriteDigital(false);
	pwmPort.WriteAnalog(0.0);
	currentRpm = 0;
}

// End
