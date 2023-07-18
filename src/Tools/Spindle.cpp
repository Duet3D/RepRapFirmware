/*
 * Spindle.cpp
 *
 *  Created on: Mar 21, 2018
 *      Author: Christian
 */

#include "Spindle.h"
#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(Spindle, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(Spindle, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry Spindle::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Spindle members
	{ "active",			OBJECT_MODEL_FUNC((int32_t)self->configuredRpm),			ObjectModelEntryFlags::none },
	{ "canReverse",		OBJECT_MODEL_FUNC(self->reverseNotForwardPort.IsValid()),	ObjectModelEntryFlags::none },
	{ "current",		OBJECT_MODEL_FUNC((int32_t)self->currentRpm),				ObjectModelEntryFlags::live },
	{ "frequency",		OBJECT_MODEL_FUNC((int32_t)self->frequency),				ObjectModelEntryFlags::verbose },
	{ "idlePwm",		OBJECT_MODEL_FUNC(self->idlePwm, 2),						ObjectModelEntryFlags::verbose },
	{ "max",			OBJECT_MODEL_FUNC((int32_t)self->maxRpm),					ObjectModelEntryFlags::verbose },
	{ "maxPwm",			OBJECT_MODEL_FUNC(self->maxPwm, 2),							ObjectModelEntryFlags::verbose },
	{ "min",			OBJECT_MODEL_FUNC((int32_t)self->minRpm),					ObjectModelEntryFlags::verbose },
	{ "minPwm",			OBJECT_MODEL_FUNC(self->minPwm, 2),							ObjectModelEntryFlags::verbose },
	{ "state",			OBJECT_MODEL_FUNC(self->state.ToString()),					ObjectModelEntryFlags::live },
};

constexpr uint8_t Spindle::objectModelTableDescriptor[] = { 1, 10 };

DEFINE_GET_OBJECT_MODEL_TABLE(Spindle)

#endif

Spindle::Spindle() noexcept
	: minPwm(DefaultMinSpindlePwm), maxPwm(DefaultMaxSpindlePwm), idlePwm(DefaultIdleSpindlePwm),
	  currentRpm(0), configuredRpm(0), minRpm(DefaultMinSpindleRpm), maxRpm(DefaultMaxSpindleRpm),
	  frequency(0), state(SpindleState::unconfigured)
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

	if (gb.Seen('K'))
	{
		seen = true;
		float pwm[3];
		size_t numValues = 3;
		gb.GetFloatArray(pwm, numValues, false);
		if (numValues >= 2)
		{
			minPwm = constrain(pwm[0], 0.0F, 1.0F);
			maxPwm = constrain(pwm[1], max<float>(pwm[0], minPwm), 1.0F);
		}
		else
		{
			minPwm = DefaultMinSpindlePwm;
			maxPwm = constrain(pwm[0], 0.0F, 1.0F);
		}
		idlePwm = (numValues == 3) ? constrain(pwm[2], 0.0F, 1.0F) : DefaultIdleSpindlePwm;
	}

	if (gb.Seen('L'))
	{
		seen = true;
		uint32_t rpm[2];
		size_t numValues = 2;
		gb.GetUnsignedArray(rpm, numValues, false);
		if (numValues == 2)
		{
			minRpm = rpm[0];
			maxRpm = rpm[1];
		}
		else
		{
			minRpm = DefaultMinSpindleRpm;
			maxRpm = rpm[0];
		}
	}

	if (seen)
	{
		state = SpindleState::stopped;
		reprap.SpindlesUpdated();
	}
	return GCodeResult::ok;
}

void Spindle::SetConfiguredRpm(const uint32_t rpm, bool updateCurrentRpm) noexcept
{
	configuredRpm = rpm;
	if (updateCurrentRpm)
	{
		SetRpm(configuredRpm);
	}
	reprap.SpindlesUpdated();			// configuredRpm is not flagged live
}

void Spindle::SetRpm(uint32_t rpm) noexcept
{
	if (state == SpindleState::stopped || rpm == 0)
	{
		onOffPort.WriteDigital(false);
		pwmPort.WriteAnalog(idlePwm);
		currentRpm = 0;						// current rpm is flagged live, so no need to change seqs.spindles
	}
	else if (state == SpindleState::forward)
	{
		rpm = constrain<int>(rpm, minRpm, maxRpm);
		reverseNotForwardPort.WriteDigital(false);
		pwmPort.WriteAnalog(((float)(rpm - minRpm) / (float)(maxRpm - minRpm)) * (maxPwm - minPwm) + minPwm);
		onOffPort.WriteDigital(true);
		currentRpm = rpm;					// current rpm is flagged live, so no need to change seqs.spindles
	}
	else if (state == SpindleState::reverse)
	{
		rpm = constrain<int>(-rpm, -maxRpm, -minRpm);
		reverseNotForwardPort.WriteDigital(true);
		pwmPort.WriteAnalog(((float)(-rpm - minRpm) / (float)(maxRpm - minRpm)) * (maxPwm - minPwm) + minPwm);
		onOffPort.WriteDigital(true);
		currentRpm = -rpm;					// current rpm is flagged live, so no need to change seqs.spindles
	}
}

void Spindle::SetState(const SpindleState newState) noexcept
{
	state = newState;
	SetRpm(configuredRpm);					// depending on the configured SpindleState this might actually stop the spindle
}

// End
