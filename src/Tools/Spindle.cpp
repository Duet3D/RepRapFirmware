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
	{ "type", 			OBJECT_MODEL_FUNC(self->type.ToString()),					ObjectModelEntryFlags::verbose },
};

constexpr uint8_t Spindle::objectModelTableDescriptor[] = { 1, 11 };

DEFINE_GET_OBJECT_MODEL_TABLE(Spindle)

#endif

Spindle::Spindle() noexcept
	: minPwm(DefaultMinSpindlePwm), maxPwm(DefaultMaxSpindlePwm), idlePwm(DefaultIdleSpindlePwm),
	  currentRpm(0), configuredRpm(0), minRpm(DefaultMinSpindleRpm), maxRpm(DefaultMaxSpindleRpm),
	  frequency(0), type(DefaultSpindleType), state(SpindleState::unconfigured)
{
}

GCodeResult Spindle::Configure(uint32_t spindleNumber, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	if (gb.Seen('C'))
	{
		seen = true;
		IoPort *_ecv_from const ports[] = { &pwmPort, &onOffPort, &reverseNotForwardPort };
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

	if (gb.Seen('T'))
	{
		seen = true;
		type = (SpindleType)gb.GetLimitedUIValue('T', 2);
	}

	if (seen)
	{
		state = SpindleState::stopped;
		reprap.SpindlesUpdated();
		return GCodeResult::ok;
	}

	// If we get here then we are reporting on a spindle
	reply.printf("Spindle %" PRIu32, spindleNumber);
	if (state == SpindleState::unconfigured)
	{
		reply.cat(" is not configured");
	}
	else
	{
		reply.cat(": ");

		if (state == SpindleState::forward || state == SpindleState::reverse)
		{
			reply.catf("running %s at %" PRIu32 " rpm, ", state.ToString(), GetCurrentRpm());
		}

		reply.catf("type %s", type.ToString());

		const bool isEnaDir = (type == SpindleType::enaDir);

		if (onOffPort.IsValid())
		{
			reply.cat(", ");
			reply.catf(isEnaDir ? "enable" : "forward");
			onOffPort.AppendBasicDetails(reply);
		}

		if (reverseNotForwardPort.IsValid())
		{
			reply.cat(", ");
			reply.catf(isEnaDir? "direction" : "reverse");
			reverseNotForwardPort.AppendBasicDetails(reply);
		}

		if (pwmPort.IsValid())
		{
			reply.cat(", rpm");
			pwmPort.AppendFullDetails(reply);
		}

		reply.catf(", rpm min %" PRIu32 ", max %" PRIu32, minRpm, maxRpm);
	}
	return GCodeResult::ok;
}

void Spindle::SetConfiguredRpm(uint32_t rpm, bool updateCurrentRpm) noexcept
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
	// Normal mode:
	//   Forward: onOffPort=1, reverseNotForwardPort=0
	//   Reverse: onOffPort=1, reverseNotForwardPort=1
	//   Stopped: onOffPort=0, reverseNotForwardPort=0
	// Alternate mode:
	//   Forward: onOffPort=1, reverseNotForwardPort=0
	//   Reverse: onOffPort=0, reverseNotForwardPort=1
	//   Stopped: onOffPort=0, reverseNotForwardPort=0

	if (state == SpindleState::stopped || rpm == 0)
	{
		onOffPort.WriteDigital(false);
		// Make sure reverse port is set correctly in stopped state.
		reverseNotForwardPort.WriteDigital(false);
		pwmPort.WriteAnalog(idlePwm);
		currentRpm = 0;						// current rpm is flagged live, so no need to change seqs.spindles
	}
	else if (state == SpindleState::forward)
	{
		rpm = constrain<uint32_t>(rpm, minRpm, maxRpm);
		reverseNotForwardPort.WriteDigital(false);
		pwmPort.WriteAnalog(((float)(rpm - minRpm) / (float)(maxRpm - minRpm)) * (maxPwm - minPwm) + minPwm);
		onOffPort.WriteDigital(true);
		currentRpm = rpm;					// current rpm is flagged live, so no need to change seqs.spindles
	}
	else if (state == SpindleState::reverse)
	{
		rpm = constrain<uint32_t>(rpm, minRpm, maxRpm);
		reverseNotForwardPort.WriteDigital(true);
		pwmPort.WriteAnalog(((float)(rpm - minRpm) / (float)(maxRpm - minRpm)) * (maxPwm - minPwm) + minPwm);
		onOffPort.WriteDigital(type != SpindleType::fwdRev);
		currentRpm = rpm;					// current rpm is flagged live, so no need to change seqs.spindles
	}
}

void Spindle::SetState(SpindleState newState) noexcept
{
	state = newState;
	SetRpm(configuredRpm);					// depending on the configured SpindleState this might actually stop the spindle
}

// End
