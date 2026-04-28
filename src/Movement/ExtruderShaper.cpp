/*
 * PressureAdvanceShaper.cpp
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#include "ExtruderShaper.h"

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(ExtruderShaper, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(ExtruderShaper, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry ExtruderShaper::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. InputShaper members
	{ "d",				OBJECT_MODEL_FUNC((float)self->dk, 2), 							ObjectModelEntryFlags::none },
	{ "k0",				OBJECT_MODEL_FUNC((float)self->k0 * StepClocksToSeconds, 3), 	ObjectModelEntryFlags::none },
	{ "k1",				OBJECT_MODEL_FUNC((float)self->k1 * StepClocksToSeconds, 3), 	ObjectModelEntryFlags::none },
};

constexpr uint8_t ExtruderShaper::objectModelTableDescriptor[] = { 1, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE(ExtruderShaper)

// Set the pressure advance parameters
void ExtruderShaper::SetParameters(const PressureAdvanceParameters& params) noexcept
{
	k0 = (motioncalc_t)(params.k[0] * StepClockRate);
	k1 = (motioncalc_t)(params.k[1] * StepClockRate);
	dk = (motioncalc_t)params.dk;
	if (k0 == 0)
	{
		vk = dk = std::numeric_limits<motioncalc_t>::infinity();
	}
	else
	{
		vk = dk/k0;
		d0 = dk * ((motioncalc_t)1.0 - (k1/k0));
	}
}

// Set single-slope pressure advance
void ExtruderShaper::SetParametersSimple(float f) noexcept
{
	k0 = k1 = (motioncalc_t)f;
	d0 = (motioncalc_t)0.0;
	dk = vk = std::numeric_limits<motioncalc_t>::infinity();
}

#if SUPPORT_REMOTE_COMMANDS

// Set the pressure advance parameters
void ExtruderShaper::SetParameters(const ShortPressureAdvanceParameters& params) noexcept
{
	k0 = (motioncalc_t)((float)params.k[0] * StepClockRate);
	k1 = (motioncalc_t)((float)params.k[1] * StepClockRate);
	dk = (motioncalc_t)params.dk;
	if (k0 == (motioncalc_t)0.0)
	{
		vk = dk = std::numeric_limits<motioncalc_t>::infinity();
		d0 = (motioncalc_t)0.0;
	}
	else
	{
		vk = dk/k0;
		d0 = dk * ((motioncalc_t)1.0 - (k1/k0));
	}
}

#endif

// Get the pressure advance distance for a given extrusion speed. This is not currently used.
motioncalc_t ExtruderShaper::GetPressureAdvanceDistance(motioncalc_t speed) const noexcept
{
	return (speed <= vk) ? k0 * speed : d0 + k1 * speed;
}

// Get the average number of pressure advance clocks for a move segment that changes speed. Must have lowSpeed <= highSpeed.
motioncalc_t ExtruderShaper::GetAverageAdvanceClocks(motioncalc_t lowSpeed, motioncalc_t highSpeed, motioncalc_t steps) const noexcept
{
	const motioncalc_t actualLowSpeed = lowSpeed * steps;
	const motioncalc_t actualHighSpeed = highSpeed * steps;

	// Optimisation for when the speed change doesn't cross the knee
	if (actualHighSpeed <= vk)
	{
		return k0;
	}
	if (actualLowSpeed >= vk)
	{
		return k1;
	}

	// actualLowSpeed is below the knee and actualHighSpeed is above it
	const motioncalc_t lowDistance = k0 * actualLowSpeed;
	const motioncalc_t highDistance = d0 + k1 * actualHighSpeed;
	return (highDistance - lowDistance)/(actualHighSpeed - actualLowSpeed);
}

// Append the pressure advance parameters to a string
void ExtruderShaper::AppendParameters(const StringRef& reply) const noexcept
{
	if (std::isinf(dk))
	{
		reply.catf("%.3f", (double)(k0 * StepClocksToSeconds));
	}
	else
	{
		reply.catf("(%.3f %.3f %.2f)", (double)(k0 * StepClocksToSeconds), (double)(k1 * StepClocksToSeconds), (double)dk);
	}
}

// End
