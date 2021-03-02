/*
 * InputShaper.cpp
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#include "InputShaper.h"

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <RepRap.h>
#include "StepTimer.h"
#include "DDA.h"
#include "MoveSegment.h"

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
		seen = true;
		type = (InputShaperType)gb.GetLimitedUIValue('P', InputShaperType::NumValues);
	}
	else if (seen && type == InputShaperType::none)
	{
		// For backwards compatibility, if we have set input shaping parameters but not defined shaping type, default to DAA for now. Change this when we support better types of input shaping.
		type = InputShaperType::DAA;
	}

	if (seen)
	{
		reprap.MoveUpdated();
	}
	else if (type != InputShaperType::none)
	{
		reply.printf("%s input shaping at %.1fHz damping factor %.2f, min. acceleration %.1f",
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

InputShaperPlan InputShaper::PlanShaping(DDA& dda) const noexcept
{
	switch (type.RawValue())
	{
	case InputShaperType::DAA:
		{
			// Try to reduce the acceleration/deceleration of the move to cancel ringing
			const float idealPeriod = GetFullPeriod();

			float proposedAcceleration = dda.acceleration, proposedAccelDistance = dda.beforePrepare.accelDistance;
			bool adjustAcceleration = false;
			if ((dda.prev->state != DDA::DDAState::frozen && dda.prev->state != DDA::DDAState::executing) || !dda.prev->IsAccelerationMove())
			{
				const float accelTime = (dda.topSpeed - dda.startSpeed)/dda.acceleration;
				if (accelTime < idealPeriod)
				{
					proposedAcceleration = (dda.topSpeed - dda.startSpeed)/idealPeriod;
					adjustAcceleration = true;
				}
				else if (accelTime < idealPeriod * 2)
				{
					proposedAcceleration = (dda.topSpeed - dda.startSpeed)/(idealPeriod * 2);
					adjustAcceleration = true;
				}
				if (adjustAcceleration)
				{
					proposedAccelDistance = (fsquare(dda.topSpeed) - fsquare(dda.startSpeed))/(2 * proposedAcceleration);
				}
			}

			float proposedDeceleration = dda.deceleration, proposedDecelDistance = dda.beforePrepare.decelDistance;
			bool adjustDeceleration = false;
			if (dda.next->state != DDA::DDAState::provisional || !dda.next->IsDecelerationMove())
			{
				const float decelTime = (dda.topSpeed - dda.endSpeed)/dda.deceleration;
				if (decelTime < idealPeriod)
				{
					proposedDeceleration = (dda.topSpeed - dda.endSpeed)/idealPeriod;
					adjustDeceleration = true;
				}
				else if (decelTime < idealPeriod * 2)
				{
					proposedDeceleration = (dda.topSpeed - dda.endSpeed)/(idealPeriod * 2);
					adjustDeceleration = true;
				}
				if (adjustDeceleration)
				{
					proposedDecelDistance = (fsquare(dda.topSpeed) - fsquare(dda.endSpeed))/(2 * proposedDeceleration);
				}
			}

			if (adjustAcceleration || adjustDeceleration)
			{
				const float drcMinimumAcceleration = GetMinimumAcceleration();
				if (proposedAccelDistance + proposedDecelDistance <= dda.totalDistance)
				{
					if (proposedAcceleration < drcMinimumAcceleration || proposedDeceleration < drcMinimumAcceleration)
					{
						break;
					}
					dda.acceleration = proposedAcceleration;
					dda.deceleration = proposedDeceleration;
					dda.beforePrepare.accelDistance = proposedAccelDistance;
					dda.beforePrepare.decelDistance = proposedDecelDistance;
				}
				else
				{
					// We can't keep this as a trapezoidal move with the original top speed.
					// Try an accelerate-decelerate move with acceleration and deceleration times equal to the ideal period.
					const float twiceTotalDistance = 2 * dda.totalDistance;
					float proposedTopSpeed = dda.totalDistance/idealPeriod - (dda.startSpeed + dda.endSpeed)/2;
					if (proposedTopSpeed > dda.startSpeed && proposedTopSpeed > dda.endSpeed)
					{
						proposedAcceleration = (twiceTotalDistance - ((3 * dda.startSpeed + dda.endSpeed) * idealPeriod))/(2 * fsquare(idealPeriod));
						proposedDeceleration = (twiceTotalDistance - ((dda.startSpeed + 3 * dda.endSpeed) * idealPeriod))/(2 * fsquare(idealPeriod));
						if (   proposedAcceleration < drcMinimumAcceleration || proposedDeceleration < drcMinimumAcceleration
							|| proposedAcceleration > dda.acceleration || proposedDeceleration > dda.deceleration
						   )
						{
							break;
						}
						dda.topSpeed = proposedTopSpeed;
						dda.acceleration = proposedAcceleration;
						dda.deceleration = proposedDeceleration;
						dda.beforePrepare.accelDistance = dda.startSpeed * idealPeriod + (dda.acceleration * fsquare(idealPeriod))/2;
						dda.beforePrepare.decelDistance = dda.endSpeed * idealPeriod + (dda.deceleration * fsquare(idealPeriod))/2;
					}
					else if (dda.startSpeed < dda.endSpeed)
					{
						// Change it into an accelerate-only move, accelerating as slowly as we can
						proposedAcceleration = (fsquare(dda.endSpeed) - fsquare(dda.startSpeed))/twiceTotalDistance;
						if (proposedAcceleration < drcMinimumAcceleration)
						{
							break;		// avoid very small accelerations because they can be problematic
						}
						dda.acceleration = proposedAcceleration;
						dda.topSpeed = dda.endSpeed;
						dda.beforePrepare.accelDistance = dda.totalDistance;
						dda.beforePrepare.decelDistance = 0.0;
					}
					else if (dda.startSpeed > dda.endSpeed)
					{
						// Change it into a decelerate-only move, decelerating as slowly as we can
						proposedDeceleration = (fsquare(dda.startSpeed) - fsquare(dda.endSpeed))/twiceTotalDistance;
						if (proposedDeceleration < drcMinimumAcceleration)
						{
							break;		// avoid very small accelerations because they can be problematic
						}
						dda.deceleration = proposedDeceleration;
						dda.topSpeed = dda.startSpeed;
						dda.beforePrepare.accelDistance = 0.0;
						dda.beforePrepare.decelDistance = dda.totalDistance;
					}
					else
					{
						// Start and end speeds are exactly the same, possibly zero, so give up trying to adjust this move
						break;
					}
				}

				if (reprap.Debug(moduleMove))
				{
					debugPrintf("New a=%.1f d=%.1f\n", (double)dda.acceleration, (double)dda.deceleration);
				}
			}
		}
		break;

	case InputShaperType::ZVD:
		//TODO
		break;

	case InputShaperType::ZVDD:
		//TODO
		break;

	case InputShaperType::EI2:
		//TODO
		break;

	case InputShaperType::none:
	default:
		break;
	}

	// If we get here then either we don't shape this move or we are using DAA, so just one acceleration and one deceleration segment
	return InputShaperPlan();
}

MoveSegment *InputShaper::GetAccelerationSegments(InputShaperPlan plan, const DDA& dda, float distanceLimit, MoveSegment *nextSegment) const noexcept
{
	if (plan.accelSegments == 1)
	{
		MoveSegment * const seg = MoveSegment::Allocate(nextSegment);
		const float uDivA = dda.startSpeed/(dda.acceleration * StepTimer::StepClockRate);
		const float twoDistDivA = (2.0 * dda.totalDistance)/(dda.acceleration * StepTimer::StepClockRateSquared);
		seg->SetNonLinear(distanceLimit, fsquare(uDivA), twoDistDivA, -uDivA);
		if (nextSegment == nullptr)
		{
			seg->SetLast();
		}
		return seg;
	}

	RRF_ASSERT(false);
	return nullptr;
}

MoveSegment *InputShaper::GetDecelerationSegments(InputShaperPlan plan, const DDA& dda, float distanceLimit, float decelStartDistance, float decelStartClocks) const noexcept
{
	if (plan.decelSegments == 1)
	{
		MoveSegment * const seg = MoveSegment::Allocate(nullptr);
		const float uDivD = dda.topSpeed/(dda.deceleration * StepTimer::StepClockRate);
		const float twoDistDivD = (2.0 * dda.totalDistance)/(dda.deceleration * StepTimer::StepClockRateSquared);
		seg->SetNonLinear(distanceLimit, fsquare(uDivD) - (2.0 * decelStartDistance)/(dda.deceleration * StepTimer::StepClockRate), twoDistDivD, uDivD + decelStartClocks);
		seg->SetLast();
		return seg;
	}

	RRF_ASSERT(false);
	return nullptr;
}

// End
