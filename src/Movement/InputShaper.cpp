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
	{ "damping",				OBJECT_MODEL_FUNC(self->zeta, 2), 							ObjectModelEntryFlags::none },
	{ "frequency",				OBJECT_MODEL_FUNC(self->frequency, 2), 						ObjectModelEntryFlags::none },
#if SUPPORT_DAA
	{ "minimumAcceleration",	OBJECT_MODEL_FUNC(self->daaMinimumAcceleration, 1),			ObjectModelEntryFlags::none },
#endif
	{ "type", 					OBJECT_MODEL_FUNC(self->type.ToString()), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t InputShaper::objectModelTableDescriptor[] = { 1, 3 + SUPPORT_DAA };

DEFINE_GET_OBJECT_MODEL_TABLE(InputShaper)

InputShaper::InputShaper() noexcept
	: frequency(DefaultFrequency),
	  zeta(DefaultDamping),
#if SUPPORT_DAA
	  daaMinimumAcceleration(DefaultDAAMinimumAcceleration),
#endif
	  type(InputShaperType::none),
	  numImpulses(1)
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
		frequency = gb.GetLimitedFValue('F', MinimumInputShapingFrequency, MaximumInputShapingFrequency);
	}
#if SUPPORT_DAA
	if (gb.Seen('L'))
	{
		seen = true;
		daaMinimumAcceleration = max<float>(gb.GetFValue(), 1.0);		// very low accelerations cause problems with the maths
	}
#endif
	if (gb.Seen('S'))
	{
		seen = true;
		zeta = gb.GetLimitedFValue('S', 0.0, 0.99);
	}

	if (gb.Seen('P'))
	{
		seen = true;
		type = (InputShaperType)gb.GetLimitedUIValue('P', InputShaperType::NumValues);
	}
	else if (seen && type == InputShaperType::none)
	{
#if SUPPORT_DAA
		// For backwards compatibility, if we have set input shaping parameters but not defined shaping type, default to DAA for now. Change this when we support better types of input shaping.
		type = InputShaperType::DAA;
#else
		type = InputShaperType::ZVD;
#endif
	}

	if (seen)
	{
		const float sqrtOneMinusZetaSquared = fastSqrtf(1.0 - fsquare(zeta));
		const float dampedFrequency = frequency * sqrtOneMinusZetaSquared;
		const float k = expf(-zeta * Pi/sqrtOneMinusZetaSquared);
		switch (type.RawValue())
		{
		case InputShaperType::none:
			numImpulses = 1;
			shapingTime = 0.0;
			break;

#if SUPPORT_DAA
		case InputShaperType::DAA:
			numImpulses	= 1;
			times[1] = 1.0/dampedFrequency;
			times[0] = 0.5 * times[1];
			shapingTime = 0.0;
			break;
#endif

		case InputShaperType::ZVD:		// see https://www.researchgate.net/publication/316556412_INPUT_SHAPING_CONTROL_TO_REDUCE_RESIDUAL_VIBRATION_OF_A_FLEXIBLE_BEAM
			{
				const float j = 1.0 + 2.0 * k + fsquare(k);
				coefficients[0] = 1.0/j;
				coefficients[1] = 2.0 * k/j;
				coefficients[2] = fsquare(k)/j;
			}
			times[1] = 1.0/dampedFrequency;
			times[0] = 0.5 * times[1];
			numImpulses = 3;
			shapingTime = times[1] * StepTimer::StepClockRate;
			break;

		case InputShaperType::ZVDD:		// see https://www.researchgate.net/publication/316556412_INPUT_SHAPING_CONTROL_TO_REDUCE_RESIDUAL_VIBRATION_OF_A_FLEXIBLE_BEAM
			{
				const float j = 1.0 + 3.0 * (k + fsquare(k)) + k * fsquare(k);
				coefficients[0] = 1.0/j;
				coefficients[1] = 3.0 * k/j;
				coefficients[2] = 3.0 * fsquare(k)/j;
				coefficients[3] = k * fsquare(k)/j;
			}
			times[1] = 1.0/dampedFrequency;
			times[0] = 0.5 *times[1];
			times[2] = times[0] + times[1];
			numImpulses = 4;
			shapingTime = times[2] * StepTimer::StepClockRate;
			break;

		case InputShaperType::EI2:		// see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.465.1337&rep=rep1&type=pdf. United States patent #4,916,635.
			{
				const float zetaSquared = fsquare(zeta);
				const float zetaCubed = zetaSquared * zeta;
				coefficients[0] = 0.16054 +  0.76699 * zeta +  2.26560 * zetaSquared + -1.22750 * zetaCubed;
				coefficients[1] = 0.33911 +  0.45081 * zeta + -2.58080 * zetaSquared +  1.73650 * zetaCubed;
				coefficients[2] = 0.34089 + -0.61533 * zeta + -0.68765 * zetaSquared +  0.42261 * zetaCubed;
				coefficients[3] = 0.15997 + -0.60246 * zeta +  1.00280 * zetaSquared + -0.93145 * zetaCubed;
				times[0] = (0.49890 +  0.16270 * zeta + -0.54262 * zetaSquared + 6.16180 * zetaCubed)/dampedFrequency;
				times[1] = (0.99748 +  0.18382 * zeta + -1.58270 * zetaSquared + 8.17120 * zetaCubed)/dampedFrequency;
				times[2] = (1.49920 + -0.09297 * zeta + -0.28338 * zetaSquared + 1.85710 * zetaCubed)/dampedFrequency;
			}
			numImpulses = 4;
			shapingTime = times[2] * StepTimer::StepClockRate;
			break;

		case InputShaperType::EI3:		// see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.465.1337&rep=rep1&type=pdf. United States patent #4,916,635
			{
				const float zetaSquared = fsquare(zeta);
				const float zetaCubed = zetaSquared * zeta;
				coefficients[0] = 0.11275 +  0.76632 * zeta +  3.29160 * zetaSquared + -1.44380 * zetaCubed;
				coefficients[1] = 0.23698 +  0.61164 * zeta + -2.57850 * zetaSquared +  4.85220 * zetaCubed;
				coefficients[2] = 0.30008 + -0.19062 * zeta + -2.14560 * zetaSquared +  0.13744 * zetaCubed;
				coefficients[3] = 0.23775 + -0.73297 * zeta +  0.46885 * zetaSquared + -2.08650 * zetaCubed;
				coefficients[4] = 0.11244 + -0.45439 * zeta +  0.96382 * zetaSquared + -1.46000 * zetaCubed;
				times[0] = (0.49974 +  0.23834 * zeta +  0.44559 * zetaSquared + 12.4720 * zetaCubed)/dampedFrequency;
				times[1] = (0.99849 +  0.29808 * zeta + -2.36460 * zetaSquared + 23.3990 * zetaCubed)/dampedFrequency;
				times[2] = (1.49870 +  0.10306 * zeta + -2.01390 * zetaSquared + 17.0320 * zetaCubed)/dampedFrequency;
				times[3] = (1.99960 + -0.28231 * zeta +  0.61536 * zetaSquared + 5.40450 * zetaCubed)/dampedFrequency;
			}
			numImpulses = 5;
			shapingTime = times[3] * StepTimer::StepClockRate;
			break;
		}

		float tLostAtStart = 0.0;
		float tLostAtEnd = 0.0;
		for (uint8_t i = 0; i < numImpulses - 1; ++i)
		{
			tLostAtStart += (1.0 - coefficients[i]) * times[i];
			tLostAtEnd += coefficients[i] * times[i];
		}
		clocksLostAtStart = tLostAtStart * StepTimer::StepClockRate;
		clocksLostAtEnd = tLostAtEnd * StepTimer::StepClockRate;

		reprap.MoveUpdated();
	}
	else if (type == InputShaperType::none)
	{
		reply.copy("Input shaping is disabled");
	}
	else
	{
		reply.printf("%s input shaping at %.1fHz damping factor %.2f", type.ToString(), (double)frequency, (double)zeta);
#if SUPPORT_DAA
		if (type == InputShaperType::DAA)
		{
			reply.catf(", min. acceleration %.1f", (double)daaMinimumAcceleration);
		}
#endif
	}
	return GCodeResult::ok;
}

InputShaperPlan InputShaper::PlanShaping(DDA& dda, BasicPrepParams& params, bool shapingEnabled) const noexcept
{
	InputShaperPlan plan;			// this clears out all the fields

	switch ((shapingEnabled) ? type.RawValue() : InputShaperType::none)
	{
#if SUPPORT_DAA
	case InputShaperType::DAA:
		{
			// Try to reduce the acceleration/deceleration of the move to cancel ringing
			const float idealPeriod = times[1];					// this the full period

			float proposedAcceleration = dda.acceleration, proposedAccelDistance = dda.beforePrepare.accelDistance;
			bool adjustAcceleration = false;
			if (dda.topSpeed > dda.startSpeed && ((dda.GetPrevious()->state != DDA::DDAState::frozen && dda.GetPrevious()->state != DDA::DDAState::executing) || !dda.GetPrevious()->flags.wasAccelOnlyMove))
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
			if (dda.GetNext()->state != DDA::DDAState::provisional || !dda.GetNext()->IsDecelerationMove())
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
				if (proposedAccelDistance + proposedDecelDistance <= dda.totalDistance)
				{
					if (proposedAcceleration < daaMinimumAcceleration || proposedDeceleration < daaMinimumAcceleration)
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
						if (   proposedAcceleration < daaMinimumAcceleration || proposedDeceleration < daaMinimumAcceleration
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
						if (proposedAcceleration < daaMinimumAcceleration)
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
						if (proposedDeceleration < daaMinimumAcceleration)
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
					debugPrintf("DAA: new a=%.1f d=%.1f\n", (double)dda.acceleration, (double)dda.deceleration);
				}
			}
		}
#endif
		// no break
	case InputShaperType::none:
	default:
		params.SetFromDDA(dda);
		break;

	// The other input shapers all have multiple impulses with varying coefficients
	case InputShaperType::ZVD:
	case InputShaperType::ZVDD:
	case InputShaperType::EI2:
	case InputShaperType::EI3:
		// Set up the provisional parameters
		params.SetFromDDA(dda);

		// Set the plan to what we would like to do, if possible
		plan.shapeAccelStart = params.accelClocks + clocksLostAtStart >= shapingTime
									&& ((dda.GetPrevious()->state != DDA::DDAState::frozen && dda.GetPrevious()->state != DDA::DDAState::executing) || !dda.GetPrevious()->flags.wasAccelOnlyMove);
		plan.shapeAccelEnd =   params.accelClocks + clocksLostAtEnd >= shapingTime
									&& params.decelStartDistance > params.accelDistance;
		plan.shapeDecelStart = params.decelClocks + clocksLostAtStart >= shapingTime
									&& params.decelStartDistance > params.accelDistance;
		plan.shapeDecelEnd =   params.decelClocks + clocksLostAtEnd >= shapingTime
									&& (dda.GetNext()->GetState() != DDA::DDAState::provisional || !dda.GetNext()->IsDecelerationMove());

//		debugPrintf("Original plan %03x ", (unsigned int)plan.all);
		{
			// See if we can shape the acceleration
			if (plan.shapeAccelStart || plan.shapeAccelEnd)
			{
				if (plan.shapeAccelStart && plan.shapeAccelEnd && params.accelClocks < 2 * shapingTime)
				{
					// Acceleration segment is too short to shape both the start and the end
					plan.shapeAccelStart = plan.shapeAccelEnd = false;
				}
				else
				{
					float extraAccelDistance = (plan.shapeAccelStart) ? GetExtraAccelStartDistance(dda) : 0.0;
					if (plan.shapeAccelEnd)
					{
						extraAccelDistance += GetExtraAccelEndDistance(dda);
					}
//					debugPrintf("Extra accel dist: %g, %g\n", (double)GetExtraAccelStartDistance(dda), (double)GetExtraAccelEndDistance(dda));
					if (params.accelDistance + extraAccelDistance <= params.decelStartDistance)
					{
						params.accelDistance += extraAccelDistance;
						if (plan.shapeAccelStart)
						{
							params.accelClocks += clocksLostAtStart;
						}
						if (plan.shapeAccelEnd)
						{
							params.accelClocks += clocksLostAtEnd;
						}
					}
					else
					{
						// Not enough constant speed time to the acceleration shaping
						// TODO look at overlapping accel start/accel end
						plan.shapeAccelStart = plan.shapeAccelEnd = false;
						if (reprap.Debug(Module::moduleDda))
						{
							debugPrintf("Can't shape acceleration\n");
						}
					}
				}
			}

			// See if we can shape the deceleration
			if (plan.shapeDecelStart || plan.shapeDecelEnd)
			{
				if (plan.shapeDecelStart && plan.shapeDecelEnd && params.decelClocks < 2 * shapingTime)
				{
					// Deceleration segment is too short to shape both the start and the end
					plan.shapeDecelStart = plan.shapeDecelEnd = false;
				}
				else
				{
					float extraDecelDistance = (plan.shapeDecelStart) ? GetExtraDecelStartDistance(dda) : 0.0;
					if (plan.shapeDecelEnd)
					{
						extraDecelDistance += GetExtraDecelEndDistance(dda);
					}
					if (params.accelDistance + extraDecelDistance <= params.decelStartDistance)
					{
						params.decelStartDistance -= extraDecelDistance;
						if (plan.shapeDecelStart)
						{
							params.decelClocks += clocksLostAtStart;
						}
						if (plan.shapeDecelEnd)
						{
							params.decelClocks += clocksLostAtEnd;
						}
					}
					else
					{
						// Not enough constant speed time to the acceleration shaping
						// TODO look at overlapping decel start/decel end
						plan.shapeDecelStart = plan.shapeDecelEnd = false;
						if (reprap.Debug(Module::moduleDda))
						{
							debugPrintf("Can't shape deceleration\n");
						}
					}
				}
			}
		}
		break;
	}

	MoveSegment * const accelSegs = GetAccelerationSegments(dda, params, plan);
	MoveSegment * const decelSegs = GetDecelerationSegments(dda, params, plan);

	params.Finalise(dda);									// this sets up params.steadyClocks, which is needed by FinishSegments
	FinishSegments(dda, params, accelSegs, decelSegs);
//	debugPrintf(" final plan %03x\n", (unsigned int)plan.all);
	return plan;
}

// If there is an acceleration phase, generate the acceleration segments according to the plan, and set the number of acceleration segments in the plan
MoveSegment *InputShaper::GetAccelerationSegments(DDA& dda, BasicPrepParams& params, InputShaperPlan& plan) const noexcept
{
	if (dda.beforePrepare.accelDistance > 0.0)
	{
		unsigned int numAccelSegs = 0;
		float accumulatedSegTime = 0.0;
		float endDistance = params.accelDistance;
		MoveSegment *endAccelSegs = nullptr;
		if (plan.shapeAccelEnd)
		{
			// Shape the end of the acceleration
			float segStartSpeed = dda.topSpeed;
			for (int i = numImpulses - 2; i >= 0; --i)
			{
				++numAccelSegs;
				endAccelSegs = MoveSegment::Allocate(endAccelSegs);
				const float acceleration = dda.acceleration * (1.0 - coefficients[i]);
				const float segTime = ((i == 0) ? times[0] : times[i] - times[i - 1]);
				segStartSpeed -= acceleration * segTime;
				const float uDivA = (segStartSpeed * StepTimer::StepClockRate)/acceleration;
				const float twoDistDivA = (2 * StepTimer::StepClockRateSquared * dda.totalDistance)/acceleration;
				endAccelSegs->SetNonLinear(endDistance/dda.totalDistance, segTime * StepTimer::StepClockRate, uDivA, twoDistDivA, acceleration/StepTimer::StepClockRateSquared);
				endDistance -= (segStartSpeed + (0.5 * acceleration * segTime)) * segTime;
			}
			accumulatedSegTime += times[numImpulses - 2];
		}

		float startDistance = 0.0;
		float startSpeed = dda.startSpeed;
		MoveSegment *startAccelSegs = nullptr;
		if (plan.shapeAccelStart)
		{
			// Shape the start of the acceleration
			for (int i = 0; i < numImpulses - 1; ++i)
			{
				++numAccelSegs;
				MoveSegment *seg = MoveSegment::Allocate(nullptr);
				const float acceleration = dda.acceleration * coefficients[i];
				const float segTime = ((i == 0) ? times[0] : times[i] - times[i - 1]);
				const float uDivA = (startSpeed * StepTimer::StepClockRate)/acceleration;
				const float twoDistDivA = (2 * StepTimer::StepClockRateSquared * dda.totalDistance)/acceleration;
				startDistance += (startSpeed + (0.5 * acceleration * segTime)) * segTime;
				seg->SetNonLinear(startDistance/dda.totalDistance, segTime * StepTimer::StepClockRate, uDivA, twoDistDivA, acceleration/StepTimer::StepClockRateSquared);
				if (i == 0)
				{
					startAccelSegs = seg;
				}
				else
				{
					startAccelSegs->AddToTail(seg);
				}
				startSpeed += acceleration * segTime;
			}
			accumulatedSegTime += times[numImpulses - 2];
		}

		// Do the constant acceleration part
		if (endDistance > startDistance)
		{
			++numAccelSegs;
			endAccelSegs = MoveSegment::Allocate(endAccelSegs);
			const float uDivA = (startSpeed * StepTimer::StepClockRate)/dda.acceleration;
			const float twoDistDivA = (2 * StepTimer::StepClockRateSquared * dda.totalDistance)/dda.acceleration;
			endAccelSegs->SetNonLinear(endDistance/dda.totalDistance, params.accelClocks - (accumulatedSegTime * StepTimer::StepClockRate), uDivA, twoDistDivA, dda.acceleration/StepTimer::StepClockRateSquared);
		}

		plan.accelSegments = numAccelSegs;
		if (startAccelSegs == nullptr)
		{
			return endAccelSegs;
		}

		if (endAccelSegs != nullptr)
		{
			startAccelSegs->AddToTail(endAccelSegs);
		}
		return startAccelSegs;
	}

	plan.accelSegments = 0;
	return nullptr;
}

// If there is a deceleration phase, generate the deceleration segments according to the plan, and set the number of deceleration segments in the plan
MoveSegment *InputShaper::GetDecelerationSegments(DDA& dda, BasicPrepParams& params, InputShaperPlan& plan) const noexcept
{
	if (dda.beforePrepare.decelDistance > 0.0)
	{
		unsigned int numDecelSegs = 0;
		float accumulatedSegTime = 0.0;
		float endDistance = dda.totalDistance;
		MoveSegment *endDecelSegs = nullptr;
		if (plan.shapeDecelEnd)
		{
			// Shape the end of the deceleration
			float segStartSpeed = dda.endSpeed;
			for (int i = numImpulses - 2; i >= 0; --i)
			{
				++numDecelSegs;
				endDecelSegs = MoveSegment::Allocate(endDecelSegs);
				const float acceleration = -dda.deceleration * (1.0 - coefficients[i]);
				const float segTime = ((i == 0) ? times[0] : times[i] - times[i - 1]);
				segStartSpeed -= acceleration * segTime;
				const float uDivA = (segStartSpeed * StepTimer::StepClockRate)/acceleration;
				const float twoDistDivA = (2 * StepTimer::StepClockRateSquared * dda.totalDistance)/acceleration;
				endDecelSegs->SetNonLinear(endDistance/dda.totalDistance, segTime * StepTimer::StepClockRate, uDivA, twoDistDivA, acceleration/StepTimer::StepClockRateSquared);
				endDistance -= (segStartSpeed + (0.5 * acceleration * segTime)) * segTime;
			}
			accumulatedSegTime += times[numImpulses - 2];
		}

		float startDistance = params.decelStartDistance;
		float startSpeed = dda.topSpeed;
		MoveSegment *startDecelSegs = nullptr;
		if (plan.shapeDecelStart)
		{
			// Shape the start of the deceleration
			for (int i = 0; i < numImpulses - 1; ++i)
			{
				++numDecelSegs;
				MoveSegment *seg = MoveSegment::Allocate(nullptr);
				const float acceleration = -dda.deceleration * coefficients[i];
				const float segTime = ((i == 0) ? times[0] : times[i] - times[i - 1]);
				const float uDivA = (startSpeed * StepTimer::StepClockRate)/acceleration;
				const float twoDistDivA = (2 * StepTimer::StepClockRateSquared * dda.totalDistance)/acceleration;
				startDistance += (startSpeed + (0.5 * acceleration * segTime)) * segTime;
				seg->SetNonLinear(startDistance/dda.totalDistance, segTime * StepTimer::StepClockRate, uDivA, twoDistDivA, acceleration/StepTimer::StepClockRateSquared);
				if (i == 0)
				{
					startDecelSegs = seg;
				}
				else
				{
					startDecelSegs->AddToTail(seg);
				}
				startSpeed += acceleration * segTime;
			}
			accumulatedSegTime += times[numImpulses - 2];
		}

		// Do the constant deceleration part
		if (endDistance > startDistance)
		{
			++numDecelSegs;
			endDecelSegs = MoveSegment::Allocate(endDecelSegs);
			const float uDivD = (startSpeed * StepTimer::StepClockRate)/dda.deceleration;
			const float twoDistDivD = (2 * StepTimer::StepClockRateSquared * dda.totalDistance)/dda.deceleration;
			endDecelSegs->SetNonLinear(endDistance/dda.totalDistance, params.decelClocks - (accumulatedSegTime * StepTimer::StepClockRate), -uDivD, -twoDistDivD, -(dda.deceleration/StepTimer::StepClockRateSquared));
		}

		plan.decelSegments = numDecelSegs;
		if (startDecelSegs == nullptr)
		{
			return endDecelSegs;
		}

		if (endDecelSegs != nullptr)
		{
			startDecelSegs->AddToTail(endDecelSegs);
		}
		return startDecelSegs;
	}

	plan.decelSegments = 0;
	return nullptr;
}

// Generate the steady speed segment (if any), tack the segments together, and attach them to the DDA
void InputShaper::FinishSegments(DDA& dda, BasicPrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept
{
	if (params.steadyClocks > 0.0)
	{
		// Insert a steady speed segment before the deceleration segments
		decelSegs = MoveSegment::Allocate(decelSegs);
		decelSegs->SetLinear(params.decelStartDistance/dda.totalDistance, params.steadyClocks, (dda.totalDistance * StepTimer::StepClockRate)/dda.topSpeed);
	}

	if (accelSegs != nullptr)
	{
		if (decelSegs != nullptr)
		{
			accelSegs->AddToTail(decelSegs);
		}
		dda.segments = accelSegs;
	}
	else
	{
		dda.segments = decelSegs;
	}
}

// Calculate the additional acceleration distance needed if we shape the start of acceleration
float InputShaper::GetExtraAccelStartDistance(const DDA& dda) const noexcept
{
	float extraDistance = 0.0;
	float u = dda.startSpeed;
	for (int seg = 0; seg < numImpulses - 1; ++seg)
	{
		const float segTime = (seg == 0) ? times[0] : times[seg] - times[seg - 1];
		const float speedChange = coefficients[seg] * dda.acceleration * segTime;
		extraDistance += (1.0 - coefficients[seg]) * (u + 0.5 * speedChange) * segTime;
		u += speedChange;
	}
	return extraDistance;
}

// Calculate the additional acceleration distance needed if we shape the end of acceleration
float InputShaper::GetExtraAccelEndDistance(const DDA& dda) const noexcept
{
	float extraDistance = 0.0;
	float v = dda.topSpeed;
	for (int seg = numImpulses - 2; seg >= 0; --seg)
	{
		const float segTime = (seg == 0) ? times[0] : times[seg] - times[seg - 1];
		const float speedChange = (1.0 - coefficients[seg]) * dda.acceleration * segTime;
		extraDistance += coefficients[seg] * (v - 0.5 * speedChange) * segTime;
		v -= speedChange;
	}
	return extraDistance;
}

// Calculate the additional deceleration distance needed if we shape the start of deceleration
float InputShaper::GetExtraDecelStartDistance(const DDA& dda) const noexcept
{
	float extraDistance = 0.0;
	float u = dda.topSpeed;
	for (int seg = 0; seg + 1 < numImpulses; ++seg)
	{
		const float segTime = (seg == 0) ? times[0] : times[seg] - times[seg - 1];
		const float speedChange = coefficients[seg] * dda.deceleration * segTime;
		extraDistance += (1.0 - coefficients[seg]) * (u - 0.5 * speedChange) * segTime;
		u -= speedChange;
	}
	return extraDistance;
}

// Calculate the additional deceleration distance needed if we shape the end of deceleration
float InputShaper::GetExtraDecelEndDistance(const DDA& dda) const noexcept
{
	float extraDistance = 0.0;
	float v = dda.endSpeed;
	for (int seg = numImpulses - 2; seg >= 0; --seg)
	{
		const float segTime = (seg == 0) ? times[0] : times[seg] - times[seg - 1];
		const float speedChange = coefficients[seg] * dda.deceleration * segTime;
		extraDistance += coefficients[seg] * (v + 0.5 * speedChange) * segTime;
		v += speedChange;
	}
	return extraDistance;
}

#if SUPPORT_REMOTE_COMMANDS

void InputShaper::GetSegments(DDA& dda, const BasicPrepParams& params) const noexcept
{
	// Deceleration phase
	MoveSegment * tempSegments;
	if (params.decelClocks > 0.0)
	{
		//TODO for now we assume just one deceleration segment
		tempSegments = MoveSegment::Allocate(nullptr);
		const float uDivD = dda.topSpeed/dda.deceleration;
		const float twoDistDivD = 2.0/dda.deceleration;
		tempSegments->SetNonLinear(params.decelDistance, params.decelClocks, -uDivD, -twoDistDivD, -dda.deceleration);
	}
	else
	{
		tempSegments = nullptr;
	}

	// Steady speed phase
	if (params.steadyClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(tempSegments);
		tempSegments->SetLinear(params.decelStartDistance, params.accelClocks + dda.beforePrepare.accelDistance/dda.topSpeed, 1.0/dda.topSpeed);
	}

	// Acceleration phase
	if (params.accelClocks > 0.0)
	{
		//TODO for now we assume just one acceleration segment
		tempSegments = MoveSegment::Allocate(tempSegments);
		const float uDivA = dda.startSpeed/dda.acceleration;
		const float twoDistDivA = 2.0/dda.acceleration;
		tempSegments->SetNonLinear(params.accelDistance, params.accelClocks, uDivA, twoDistDivA, dda.acceleration);
	}

	dda.segments = tempSegments;
}

#endif

// End
