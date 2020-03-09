/*
 * Spindle.cpp
 *
 *  Created on: Mar 21, 2018
 *      Author: Christian
 */

#include "Spindle.h"

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
	{ "tool",			OBJECT_MODEL_FUNC((int32_t)self->toolNumber),	ObjectModelEntryFlags::verbose },
};

constexpr uint8_t Spindle::objectModelTableDescriptor[] = { 1, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(Spindle)

#endif

// Allocate the pins returning true if successful
bool Spindle::AllocatePins(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	IoPort * const ports[] = { &spindleForwardPort, &spindleReversePort };
	const PinAccess access[] = { PinAccess::pwm, PinAccess::pwm };
	return IoPort::AssignPorts(gb, reply, PinUsedBy::spindle, 2, ports, access);
}

void Spindle::SetFrequency(PwmFrequency freq) noexcept
{
	frequency = freq;
	spindleForwardPort.SetFrequency(freq);
	spindleReversePort.SetFrequency(freq);
}

void Spindle::SetRpm(float rpm) noexcept
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

void Spindle::TurnOff() noexcept
{
	spindleReversePort.WriteAnalog(0.0);
	spindleForwardPort.WriteAnalog(0.0);
	currentRpm = 0.0;
}

// End
