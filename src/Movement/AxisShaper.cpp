/*
 * InputShaper.cpp
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#include "AxisShaper.h"

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include "StepTimer.h"
#include "DDA.h"
#include "MoveSegment.h"

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(AxisShaper, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(AxisShaper, __VA_ARGS__)

constexpr ObjectModelTableEntry AxisShaper::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. InputShaper members
	{ "damping",				OBJECT_MODEL_FUNC(self->zeta, 2), 							ObjectModelEntryFlags::none },
	{ "frequency",				OBJECT_MODEL_FUNC(self->frequency, 2), 						ObjectModelEntryFlags::none },
	{ "minAcceleration",		OBJECT_MODEL_FUNC(self->minimumAcceleration, 1),			ObjectModelEntryFlags::none },
	{ "type", 					OBJECT_MODEL_FUNC(self->type.ToString()), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t AxisShaper::objectModelTableDescriptor[] = { 1, 4 };

DEFINE_GET_OBJECT_MODEL_TABLE(AxisShaper)

AxisShaper::AxisShaper() noexcept
	: numExtraImpulses(0),
	  frequency(DefaultFrequency),
	  zeta(DefaultDamping),
	  minimumAcceleration(ConvertAcceleration(DefaultMinimumAcceleration)),
	  type(InputShaperType::none)
{
}

// Process M593
GCodeResult AxisShaper::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	constexpr float MinimumInputShapingFrequency = (float)StepClockRate/(2 * 65535);		// we use a 16-bit number of step clocks to represent half the input shaping period
	constexpr float MaximumInputShapingFrequency = 1000.0;
	bool seen = false;
	if (gb.Seen('F'))
	{
		seen = true;
		frequency = gb.GetLimitedFValue('F', MinimumInputShapingFrequency, MaximumInputShapingFrequency);
	}
	if (gb.Seen('L'))
	{
		seen = true;
		minimumAcceleration = ConvertAcceleration(max<float>(gb.GetFValue(), 1.0));			// very low accelerations cause problems with the maths
	}
	if (gb.Seen('S'))
	{
		seen = true;
		zeta = gb.GetLimitedFValue('S', 0.0, 0.99);
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
#if SUPPORT_DAA
		// For backwards compatibility, if we have set input shaping parameters but not defined shaping type, default to DAA for now. Change this when we support better types of input shaping.
		type = InputShaperType::daa;
#else
		type = InputShaperType::zvd;
#endif
	}

	if (seen)
	{
		const float sqrtOneMinusZetaSquared = fastSqrtf(1.0 - fsquare(zeta));
		const float dampedFrequency = frequency * sqrtOneMinusZetaSquared;
		const float dampedPeriod = StepClockRate/dampedFrequency;
		const float k = expf(-zeta * Pi/sqrtOneMinusZetaSquared);
		switch (type.RawValue())
		{
		case InputShaperType::none:
			numExtraImpulses = 0;
			break;

		case InputShaperType::custom:
			{
				// Get the coefficients
				size_t numAmplitudes = MaxExtraImpulses;
				gb.MustSee('H');
				gb.GetFloatArray(coefficients, numAmplitudes, false);

				// Get the impulse durations, if provided
				if (gb.Seen('T'))
				{
					size_t numDurations = numAmplitudes;
					gb.GetFloatArray(durations, numDurations, true);

					// Check we have the same number of both
					if (numDurations != numAmplitudes)
					{
						reply.copy("Too few durations given");
						type = InputShaperType::none;
						return GCodeResult::error;
					}
					for (unsigned int i = 0; i < numAmplitudes; ++i)
					{
						durations[i] *= StepClockRate;			// convert from seconds to step clocks
					}
				}
				else
				{
					for (unsigned int i = 0; i < numAmplitudes; ++i)
					{
						durations[i] = 0.5 * dampedPeriod;
					}
				}
				numExtraImpulses = numAmplitudes;
			}
			break;

#if SUPPORT_DAA
		case InputShaperType::daa:
			durations[0] = dampedPeriod;
			numExtraImpulses = 0;
			break;
#endif

		case InputShaperType::zvd:		// see https://www.researchgate.net/publication/316556412_INPUT_SHAPING_CONTROL_TO_REDUCE_RESIDUAL_VIBRATION_OF_A_FLEXIBLE_BEAM
			{
				const float j = 1.0 + 2.0 * k + fsquare(k);
				coefficients[0] = 1.0/j;
				coefficients[1] = coefficients[0] + 2.0 * k/j;
			}
			durations[0] = durations[1] = 0.5 * dampedPeriod;
			numExtraImpulses = 2;
			break;

		case InputShaperType::zvdd:		// see https://www.researchgate.net/publication/316556412_INPUT_SHAPING_CONTROL_TO_REDUCE_RESIDUAL_VIBRATION_OF_A_FLEXIBLE_BEAM
			{
				const float j = 1.0 + 3.0 * (k + fsquare(k)) + k * fsquare(k);
				coefficients[0] = 1.0/j;
				coefficients[1] = coefficients[0] + 3.0 * k/j;
				coefficients[2] = coefficients[1] + 3.0 * fsquare(k)/j;
			}
			durations[0] = durations[1] = durations[2] = 0.5 * dampedPeriod;
			numExtraImpulses = 3;
			break;

		case InputShaperType::ei2:		// see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.465.1337&rep=rep1&type=pdf. United States patent #4,916,635.
			{
				const float zetaSquared = fsquare(zeta);
				const float zetaCubed = zetaSquared * zeta;
				coefficients[0] = (0.16054)                     + (0.76699)                     * zeta + (2.26560)                     * zetaSquared + (-1.22750)                     * zetaCubed;
				coefficients[1] = (0.16054 + 0.33911)           + (0.76699 + 0.45081)           * zeta + (2.26560 - 2.58080)           * zetaSquared + (-1.22750 + 1.73650)           * zetaCubed;
				coefficients[2] = (0.16054 + 0.33911 + 0.34089) + (0.76699 + 0.45081 - 0.61533) * zeta + (2.26560 - 2.58080 - 0.68765) * zetaSquared + (-1.22750 + 1.73650 + 0.42261) * zetaCubed;

				durations[0] = ((0.49890)           + ( 0.16270          ) * zeta + (          -0.54262) * zetaSquared + (          6.16180) * zetaCubed) * dampedPeriod;
				durations[1] = ((0.99748 - 0.49890) + ( 0.18382 - 0.16270) * zeta + (-1.58270 + 0.54262) * zetaSquared + (8.17120 - 6.16180) * zetaCubed) * dampedPeriod;
				durations[2] = ((1.49920 - 0.99748) + (-0.09297 - 0.18382) * zeta + (-0.28338 + 1.58270) * zetaSquared + (1.85710 - 8.17120) * zetaCubed) * dampedPeriod;
			}
			numExtraImpulses = 3;
			break;

		case InputShaperType::ei3:		// see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.465.1337&rep=rep1&type=pdf. United States patent #4,916,635
			{
				const float zetaSquared = fsquare(zeta);
				const float zetaCubed = zetaSquared * zeta;
				coefficients[0] = (0.11275)                               + 0.76632                                 * zeta + (3.29160)                               * zetaSquared + (-1.44380)                               * zetaCubed;
				coefficients[1] = (0.11275 + 0.23698)                     + (0.76632 + 0.61164)                     * zeta + (3.29160 - 2.57850)                     * zetaSquared + (-1.44380 + 4.85220)                     * zetaCubed;
				coefficients[2] = (0.11275 + 0.23698 + 0.30008)           + (0.76632 + 0.61164 - 0.19062)           * zeta + (3.29160 - 2.57850 - 2.14560)           * zetaSquared + (-1.44380 + 4.85220 + 0.13744)           * zetaCubed;
				coefficients[3] = (0.11275 + 0.23698 + 0.30008 + 0.23775) + (0.76632 + 0.61164 - 0.19062 - 0.73297) * zeta + (3.29160 - 2.57850 - 2.14560 + 0.46885) * zetaSquared + (-1.44380 + 4.85220 + 0.13744 - 2.08650) * zetaCubed;

				durations[0] = ((0.49974)           + (0.23834)            * zeta + (0.44559)            * zetaSquared + (12.4720)           * zetaCubed) * dampedPeriod;
				durations[1] = ((0.99849 - 0.49974) + (0.29808 - 0.23834)  * zeta + (-2.36460 - 0.44559) * zetaSquared + (23.3990 - 12.4720) * zetaCubed) * dampedPeriod;
				durations[2] = ((1.49870 - 0.99849) + (0.10306 - 0.29808)  * zeta + (-2.01390 + 2.36460) * zetaSquared + (17.0320 - 23.3990) * zetaCubed) * dampedPeriod;
				durations[3] = ((1.99960 - 1.49870) + (-0.28231 - 0.10306) * zeta + (0.61536 + 2.01390)  * zetaSquared + (5.40450 - 17.0320) * zetaCubed) * dampedPeriod;
			}
			numExtraImpulses = 4;
			break;
		}

		// Calculate the total extra duration of input shaping
		totalShapingClocks = 0.0;
		extraClocksAtStart = 0.0;
		extraClocksAtEnd = 0.0;
		extraDistanceAtStart = 0.0;
		extraDistanceAtEnd = 0.0;

		{
			float u = 0.0;
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				const float segTime = durations[i];
				totalShapingClocks += segTime;
				extraClocksAtStart += (1.0 - coefficients[i]) * segTime;
				extraClocksAtEnd += coefficients[i] * segTime;
				const float speedChange = coefficients[i] * segTime;
				extraDistanceAtStart += (1.0 - coefficients[i]) * (u + 0.5 * speedChange) * segTime;
				u += speedChange;
			}
		}

		minimumShapingStartOriginalClocks = totalShapingClocks - extraClocksAtStart + (MinimumMiddleSegmentTime * StepClockRate);
		minimumShapingEndOriginalClocks = totalShapingClocks - extraClocksAtEnd + (MinimumMiddleSegmentTime * StepClockRate);
		minimumNonOverlappedOriginalClocks = (totalShapingClocks * 2) - extraClocksAtStart - extraClocksAtEnd + (MinimumMiddleSegmentTime * StepClockRate);

		{
			float v = 0.0;
			for (int i = numExtraImpulses - 1; i >= 0; --i)
			{
				const float segTime = durations[i];
				const float speedChange = (1.0 - coefficients[i]) * segTime;
				extraDistanceAtEnd += coefficients[i] * (v - 0.5 * speedChange) * segTime;
				v -= speedChange;
			}
		}

		if (numExtraImpulses != 0)
		{
			overlappedShapingClocks = 2 * totalShapingClocks;
			// Calculate the clocks and coefficients needed when we shape the start of acceleration/deceleration and then immediately shape the end
			float maxVal = 0.0;
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				overlappedDurations[i] = overlappedDurations[i + numExtraImpulses] = durations[i];
				float val = coefficients[i];
				overlappedCoefficients[i] = val;
				if (val > maxVal)
				{
					maxVal = val;
				}
				val = 1.0 - val;
				overlappedCoefficients[i + numExtraImpulses] = val;
				if (val > maxVal)
				{
					maxVal = val;
				}
			}

			// Now scale the values by maxVal so that the highest coefficient is 1.0, and calculate the total distance per unit acceleration
			overlappedDeltaVPerA = totalShapingClocks/maxVal;
			overlappedDistancePerA = 0.0;
			float u = 0.0;
			for (unsigned int i = 0; i < 2 * numExtraImpulses; ++i)
			{
				overlappedCoefficients[i] /= maxVal;
				const float speedChange = overlappedCoefficients[i] * overlappedDurations[i];
				overlappedDistancePerA += (u + 0.5 * speedChange) * overlappedDurations[i];
				u += speedChange;
			}
		}

		reprap.MoveUpdated();
	}
	else if (type == InputShaperType::none)
	{
		reply.copy("Input shaping is disabled");
	}
	else
	{
		reply.printf("Input shaping '%s' at %.1fHz damping factor %.2f, min. acceleration %.1f",
						type.ToString(), (double)frequency, (double)zeta, (double)InverseConvertAcceleration(minimumAcceleration));
		if (numExtraImpulses != 0)
		{
			reply.cat(", impulses");
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				reply.catf(" %.3f", (double)coefficients[i]);
			}
			reply.cat(" with durations (ms)");
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				reply.catf(" %.2f", (double)(durations[i] * StepClocksToMillis));
			}
		}
	}
	return GCodeResult::ok;
}

// Plan input shaping, generate the MoveSegment, and set up the basic move parameters.
// On entry, params.shapingPlan is set to 'no shaping'.
// Currently we use a single input shaper for all axes, so the move segments are attached to the DDA not the DM
void AxisShaper::PlanShaping(DDA& dda, PrepParams& params, bool shapingEnabled) const noexcept
{
	switch ((shapingEnabled) ? type.RawValue() : InputShaperType::none)
	{
#if SUPPORT_DAA
	case InputShaperType::daa:
		do
		{
			// Try to reduce the acceleration/deceleration of the move to cancel ringing
			const float idealPeriod = 1.0/frequency;					// for DAA this the full period, 1.0

			float proposedAcceleration = dda.acceleration, proposedAccelDistance = dda.beforePrepare.accelDistance;
			bool adjustAcceleration = false;
			if (dda.topSpeed > dda.startSpeed && ((dda.GetPrevious()->state != DDA::DDAState::frozen && dda.GetPrevious()->state != DDA::DDAState::executing) || !dda.GetPrevious()->flags.wasAccelOnlyMove))
			{
				const float accelTime = (dda.topSpeed - dda.startSpeed)/dda.acceleration;
				if (accelTime < idealPeriod)
				{
					proposedAcceleration = (dda.topSpeed - dda.startSpeed) * frequency;
					adjustAcceleration = true;
				}
				else if (accelTime < idealPeriod * 2)
				{
					proposedAcceleration = (dda.topSpeed - dda.startSpeed) * frequency * 0.5;
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
					proposedDeceleration = (dda.topSpeed - dda.endSpeed) * frequency;
					adjustDeceleration = true;
				}
				else if (decelTime < idealPeriod * 2)
				{
					proposedDeceleration = (dda.topSpeed - dda.endSpeed) * frequency * 0.5;
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
					if (proposedAcceleration < minimumAcceleration || proposedDeceleration < minimumAcceleration)
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
					float proposedTopSpeed = dda.totalDistance * frequency - (dda.startSpeed + dda.endSpeed)/2;
					if (proposedTopSpeed > dda.startSpeed && proposedTopSpeed > dda.endSpeed)
					{
						proposedAcceleration = (twiceTotalDistance - ((3 * dda.startSpeed + dda.endSpeed) * idealPeriod)) * fsquare(frequency) * 0.5;
						proposedDeceleration = (twiceTotalDistance - ((dda.startSpeed + 3 * dda.endSpeed) * idealPeriod)) * fsquare(frequency) * 0.5;
						if (   proposedAcceleration < minimumAcceleration || proposedDeceleration < minimumAcceleration
							|| proposedAcceleration > dda.acceleration || proposedDeceleration > dda.deceleration
						   )
						{
							break;
						}
						dda.topSpeed = proposedTopSpeed;
						dda.acceleration = proposedAcceleration;
						dda.deceleration = proposedDeceleration;
						dda.beforePrepare.accelDistance = dda.startSpeed * idealPeriod + (dda.acceleration * fsquare(idealPeriod)) * 0.5;
						dda.beforePrepare.decelDistance = dda.endSpeed * idealPeriod + (dda.deceleration * fsquare(idealPeriod)) * 0.5;
					}
					else if (dda.startSpeed < dda.endSpeed)
					{
						// Change it into an accelerate-only move, accelerating as slowly as we can
						proposedAcceleration = (fsquare(dda.endSpeed) - fsquare(dda.startSpeed))/twiceTotalDistance;
						if (proposedAcceleration < minimumAcceleration)
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
						if (proposedDeceleration < minimumAcceleration)
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
		} while (false);			// this loop is solely for the purpose of catching 'break' statements
		params.SetFromDDA(dda);
		break;
#endif

	case InputShaperType::none:
	default:
		params.SetFromDDA(dda);
		break;

	// The other input shapers all have multiple impulses with varying coefficients
	case InputShaperType::zvd:
	case InputShaperType::zvdd:
	case InputShaperType::ei2:
	case InputShaperType::ei3:
		// Set up the provisional parameters
		params.SetFromDDA(dda);

		if (params.accelDistance < params.decelStartDistance)			// we can't do any shaping unless there is a steady speed segment that can be shortened
		{
			//TODO if we want to shape both acceleration and deceleration but the steady distance is zero or too short, we could reduce the top speed
			if (params.accelDistance > 0.0)
			{
				if ((dda.GetPrevious()->state != DDA::DDAState::frozen && dda.GetPrevious()->state != DDA::DDAState::executing) || !dda.GetPrevious()->flags.wasAccelOnlyMove)
				{
					TryShapeAccelBoth(dda, params);
				}
				else if (params.accelClocks >= minimumShapingEndOriginalClocks)
				{
					TryShapeAccelEnd(dda, params);
				}
			}
			if (params.decelDistance > 0.0)
			{
				if (dda.GetNext()->GetState() != DDA::DDAState::provisional || !dda.GetNext()->IsDecelerationMove())
				{
					TryShapeDecelBoth(dda, params);
				}
				else if (params.decelClocks >= minimumShapingStartOriginalClocks)
				{
					TryShapeDecelStart(dda, params);
				}
			}
		}
		break;
	}

	// If we are doing any input shaping then set up dda.shapedSegments, else leave it as null
	if (params.shapingPlan.IsShaped())
	{
		MoveSegment * const accelSegs = GetAccelerationSegments(dda, params);
		MoveSegment * const decelSegs = GetDecelerationSegments(dda, params);
		params.Finalise(dda);									// this sets up params.steadyClocks, which is needed by FinishSegments
		dda.shapedSegments = FinishSegments(dda, params, accelSegs, decelSegs);

		// Update the acceleration and deceleration in the DDA and the acceleration/deceleration distances and times in the PrepParams, so that if we generate unshaped segments or CAN motion too, they will be in sync
		// Replace the shaped acceleration by a linear acceleration followed by constant speed time
		if (params.shapingPlan.shapeAccelStart || params.shapingPlan.shapeAccelEnd || params.shapingPlan.shapeAccelOverlapped)
		{
			const float speedIncrease = dda.topSpeed - dda.startSpeed;
			params.accelClocks = 2 * (dda.topSpeed * params.accelClocks - params.accelDistance)/speedIncrease;
			params.accelDistance = (dda.startSpeed + dda.topSpeed) * params.accelClocks * 0.5;
			dda.acceleration = speedIncrease/params.accelClocks;
		}
		if (params.shapingPlan.shapeDecelStart || params.shapingPlan.shapeDecelEnd || params.shapingPlan.shapeDecelOverlapped)
		{
			const float speedDecrease = dda.topSpeed - dda.endSpeed;
			params.decelClocks = 2 * (dda.topSpeed * params.decelClocks - params.decelDistance)/speedDecrease;
			params.decelDistance = (dda.topSpeed + dda.endSpeed) * params.decelClocks * 0.5;
			params.decelStartDistance = dda.totalDistance - params.decelDistance;
			dda.deceleration = speedDecrease/params.decelClocks;
		}
	}
	else
	{
		params.Finalise(dda);									// this sets up params.steadyClocks
	}

//	debugPrintf(" final plan %03x\n", (unsigned int)params.shapingPlan.all);
}

// Try to shape the start of the acceleration. We already know that there is sufficient acceleration time to do this, but we still need to check that there is enough distance.
void AxisShaper::TryShapeAccelStart(const DDA& dda, PrepParams& params) const noexcept
{
	const float extraAccelDistance = GetExtraAccelStartDistance(dda);
	if (params.accelDistance + extraAccelDistance <= params.decelStartDistance)
	{
		params.shapingPlan.shapeAccelStart = true;
		params.accelDistance += extraAccelDistance;
		params.accelClocks += extraClocksAtStart;
	}
	else
	{
		// Not enough constant speed time to the acceleration shaping
		if (reprap.Debug(Module::moduleDda))
		{
			debugPrintf("Can't shape accel start\n");
		}
	}
}

// Try to shape the end of the acceleration. We already know that there is sufficient acceleration time to do this, but we still need to check that there is enough distance.
void AxisShaper::TryShapeAccelEnd(const DDA& dda, PrepParams& params) const noexcept
{
	const float extraAccelDistance = GetExtraAccelEndDistance(dda);
	if (params.accelDistance + extraAccelDistance <= params.decelStartDistance)
	{
		params.shapingPlan.shapeAccelEnd = true;
		params.accelDistance += extraAccelDistance;
		params.accelClocks += extraClocksAtEnd;
	}
	else
	{
		// Not enough constant speed time to the acceleration shaping
		if (reprap.Debug(Module::moduleDda))
		{
			debugPrintf("Can't shape accel end\n");
		}
	}
}

void AxisShaper::TryShapeAccelBoth(DDA& dda, PrepParams& params) const noexcept
{
	if (dda.topSpeed - dda.startSpeed <= overlappedDeltaVPerA * dda.acceleration)
	{
		// We can use overlapped shaping
		const float newAcceleration = (dda.topSpeed - dda.startSpeed)/overlappedDeltaVPerA;
		if (newAcceleration < minimumAcceleration)
		{
			return;
		}
		const float newAccelDistance = (dda.startSpeed * overlappedShapingClocks) + (newAcceleration * overlappedDistancePerA);
		if (newAccelDistance >= params.decelStartDistance)
		{
			return;
		}
		dda.acceleration = newAcceleration;
		params.accelDistance = newAccelDistance;
		params.accelClocks = overlappedShapingClocks;
		params.shapingPlan.shapeAccelOverlapped = true;
	}
	else
	{
		if (params.accelClocks < minimumNonOverlappedOriginalClocks)
		{
			// The speed change is too high to allow overlapping, but non-overlapped shaping will give a very short steady acceleration segment.
			// If we have enough spare distance, reduce the acceleration slightly to lengthen that segment.
			const float newAcceleration = (dda.acceleration * params.accelClocks)/minimumNonOverlappedOriginalClocks;
			const float newAccelDistance = (dda.startSpeed + (0.5 * newAcceleration * minimumNonOverlappedOriginalClocks)) * minimumNonOverlappedOriginalClocks;
			if (newAccelDistance >= params.decelStartDistance)
			{
				return;
			}
			dda.acceleration = newAcceleration;
			params.accelDistance = newAccelDistance;
			params.accelClocks = minimumNonOverlappedOriginalClocks;
		}
		TryShapeAccelStart(dda, params);
		TryShapeAccelEnd(dda, params);
	}
}

// Try to shape the start of the deceleration. We already know that there is sufficient deceleration time to do this, but we still need to check that there is enough distance.
void AxisShaper::TryShapeDecelStart(const DDA& dda, PrepParams& params) const noexcept
{
	float extraDecelDistance = GetExtraDecelStartDistance(dda);
	if (params.accelDistance + extraDecelDistance <= params.decelStartDistance)
	{
		params.shapingPlan.shapeDecelStart = true;
		params.decelStartDistance -= extraDecelDistance;
		params.decelDistance += extraDecelDistance;
		params.decelClocks += extraClocksAtStart;
	}
	else
	{
		// Not enough constant speed time to do deceleration shaping
		if (reprap.Debug(Module::moduleDda))
		{
			debugPrintf("Can't shape decel start\n");
		}
	}
}

// Try to shape the end of the deceleration. We already know that there is sufficient deceleration time to do this, but we still need to check that there is enough distance.
void AxisShaper::TryShapeDecelEnd(const DDA& dda, PrepParams& params) const noexcept
{
	float extraDecelDistance = GetExtraDecelEndDistance(dda);
	if (params.accelDistance + extraDecelDistance <= params.decelStartDistance)
	{
		params.shapingPlan.shapeDecelEnd = true;
		params.decelStartDistance -= extraDecelDistance;
		params.decelDistance += extraDecelDistance;
		params.decelClocks += extraClocksAtEnd;
	}
	else
	{
		// Not enough constant speed time to do deceleration shaping
		if (reprap.Debug(Module::moduleDda))
		{
			debugPrintf("Can't shape decel and\n");
		}
	}
}

void AxisShaper::TryShapeDecelBoth(DDA& dda, PrepParams& params) const noexcept
{
	if (dda.topSpeed - dda.endSpeed <= overlappedDeltaVPerA * dda.deceleration)
	{
		// We can use overlapped shaping
		const float newDeceleration = (dda.topSpeed - dda.endSpeed)/overlappedDeltaVPerA;
		if (newDeceleration < minimumAcceleration)
		{
			return;
		}
		const float newDecelDistance = (dda.topSpeed * overlappedShapingClocks) - (newDeceleration * overlappedDistancePerA);
		const float newDecelStartDistance = dda.totalDistance - newDecelDistance;
		if (newDecelStartDistance < params.accelDistance)
		{
			return;
		}
		dda.deceleration = newDeceleration;
		params.decelDistance = newDecelDistance;
		params.decelStartDistance = newDecelStartDistance;
		params.decelClocks = overlappedShapingClocks;
		params.shapingPlan.shapeDecelOverlapped = true;
	}
	else
	{
		if (params.decelClocks < minimumNonOverlappedOriginalClocks)
		{
			// The speed change is too high to allow overlapping, but non-overlapped shaping will give a very short steady acceleration segment.
			// If we have enough spare distance, reduce the acceleration slightly to lengthen that segment.
			const float newDeceleration = (dda.deceleration * params.decelClocks)/minimumNonOverlappedOriginalClocks;
			const float newDecelDistance = (dda.endSpeed + (0.5 * newDeceleration * minimumNonOverlappedOriginalClocks)) * minimumNonOverlappedOriginalClocks;
			const float newDecelStartDistance = dda.totalDistance - newDecelDistance;
			if (newDecelStartDistance <= params.accelDistance)
			{
				return;
			}
			dda.deceleration = newDeceleration;
			params.decelStartDistance = newDecelStartDistance;
			params.decelClocks = overlappedShapingClocks;
		}
		TryShapeDecelStart(dda, params);
		TryShapeDecelEnd(dda, params);
	}
}

// If there is an acceleration phase, generate the acceleration segments according to the plan, and set the number of acceleration segments in the plan
MoveSegment *AxisShaper::GetAccelerationSegments(const DDA& dda, const PrepParams& params) const noexcept
{
	if (dda.beforePrepare.accelDistance > 0.0)
	{
		if (params.shapingPlan.shapeAccelOverlapped)
		{
			MoveSegment *accelSegs = nullptr;
			float segStartSpeed = dda.topSpeed;
			for (int i = (2 * numExtraImpulses) - 1; i >= 0; --i)
			{
				accelSegs = MoveSegment::Allocate(accelSegs);
				const float acceleration = dda.acceleration * overlappedCoefficients[i];
				const float segTime = overlappedDurations[i];
				segStartSpeed -= acceleration * segTime;
				const float b = segStartSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float segLen = (segStartSpeed + (0.5 * acceleration * segTime)) * segTime;
				accelSegs->SetNonLinear(segLen, segTime, b, c);
			}
			return accelSegs;
		}

		float accumulatedSegTime = 0.0;
		float endDistance = params.accelDistance;
		MoveSegment *endAccelSegs = nullptr;
		if (params.shapingPlan.shapeAccelEnd)
		{
			// Shape the end of the acceleration
			float segStartSpeed = dda.topSpeed;
			for (int i = numExtraImpulses - 1; i >= 0; --i)
			{
				endAccelSegs = MoveSegment::Allocate(endAccelSegs);
				const float acceleration = dda.acceleration * (1.0 - coefficients[i]);
				const float segTime = durations[i];
				segStartSpeed -= acceleration * segTime;
				const float b = segStartSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float segLen = (segStartSpeed + (0.5 * acceleration * segTime)) * segTime;
				endDistance -= segLen;
				endAccelSegs->SetNonLinear(segLen, segTime, b, c);
			}
			accumulatedSegTime += totalShapingClocks;
		}

		float startDistance = 0.0;
		float startSpeed = dda.startSpeed;
		MoveSegment *startAccelSegs = nullptr;
		if (params.shapingPlan.shapeAccelStart)
		{
			// Shape the start of the acceleration
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				MoveSegment *seg = MoveSegment::Allocate(nullptr);
				const float acceleration = dda.acceleration * coefficients[i];
				const float segTime = durations[i];
				const float b = startSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float segLen = (startSpeed + (0.5 * acceleration * segTime)) * segTime;
				startDistance += segLen;
				seg->SetNonLinear(segLen, segTime, b, c);
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
			accumulatedSegTime += totalShapingClocks;
		}

		// Do the constant acceleration part
		if (endDistance > startDistance)
		{
			endAccelSegs = MoveSegment::Allocate(endAccelSegs);
			const float b = startSpeed/(-dda.acceleration);
			const float c = 2.0/dda.acceleration;
			endAccelSegs->SetNonLinear(endDistance - startDistance, params.accelClocks - accumulatedSegTime, b, c);
		}

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

	return nullptr;
}

// If there is a deceleration phase, generate the deceleration segments according to the plan, and set the number of deceleration segments in the plan
MoveSegment *AxisShaper::GetDecelerationSegments(const DDA& dda, const PrepParams& params) const noexcept
{
	if (dda.beforePrepare.decelDistance > 0.0)
	{
		if (params.shapingPlan.shapeDecelOverlapped)
		{
			MoveSegment *decelSegs = nullptr;
			float segStartSpeed = dda.endSpeed;
			for (int i = (2 * numExtraImpulses) - 1; i >= 0; --i)
			{
				decelSegs = MoveSegment::Allocate(decelSegs);
				const float deceleration = dda.deceleration * overlappedCoefficients[i];
				const float segTime = overlappedDurations[i];
				segStartSpeed += deceleration * segTime;
				const float b = segStartSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float segLen = (segStartSpeed + (-0.5 * deceleration * segTime)) * segTime;
				decelSegs->SetNonLinear(segLen, segTime, b, c);
			}
			return decelSegs;
		}

		float accumulatedSegTime = 0.0;
		float endDistance = dda.totalDistance;
		MoveSegment *endDecelSegs = nullptr;
		if (params.shapingPlan.shapeDecelEnd)
		{
			// Shape the end of the deceleration
			float segStartSpeed = dda.endSpeed;
			for (int i = numExtraImpulses - 1; i >= 0; --i)
			{
				endDecelSegs = MoveSegment::Allocate(endDecelSegs);
				const float deceleration = dda.deceleration * (1.0 - coefficients[i]);
				const float segTime = durations[i];
				segStartSpeed += deceleration * segTime;
				const float b = segStartSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float segLen = (segStartSpeed + (-0.5 * deceleration * segTime)) * segTime;
				endDecelSegs->SetNonLinear(segLen, segTime, b, c);
				endDistance -= segLen;
			}
			accumulatedSegTime += totalShapingClocks;
		}

		float startDistance = params.decelStartDistance;
		float startSpeed = dda.topSpeed;
		MoveSegment *startDecelSegs = nullptr;
		if (params.shapingPlan.shapeDecelStart)
		{
			// Shape the start of the deceleration
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				MoveSegment *seg = MoveSegment::Allocate(nullptr);
				const float deceleration = dda.deceleration * coefficients[i];
				const float segTime = durations[i];
				const float b = startSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float segLen = (startSpeed + (-0.5 * deceleration * segTime)) * segTime;
				startDistance += segLen;
				seg->SetNonLinear(segLen, segTime, b, c);
				if (i == 0)
				{
					startDecelSegs = seg;
				}
				else
				{
					startDecelSegs->AddToTail(seg);
				}
				startSpeed -= deceleration * segTime;
			}
			accumulatedSegTime += totalShapingClocks;
		}

		// Do the constant deceleration part
		if (endDistance > startDistance)
		{
			endDecelSegs = MoveSegment::Allocate(endDecelSegs);
			const float b = startSpeed/dda.deceleration;
			const float c = -2.0/dda.deceleration;
			endDecelSegs->SetNonLinear(endDistance - startDistance, params.decelClocks - accumulatedSegTime, b, c);
		}

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

	return nullptr;
}

// Generate the steady speed segment (if any), tack the segments together, and attach them to the DDA
// Must set up params.steadyClocks before calling this
MoveSegment *AxisShaper::FinishSegments(const DDA& dda, const PrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept
{
	if (params.steadyClocks > 0.0)
	{
		// Insert a steady speed segment before the deceleration segments
		decelSegs = MoveSegment::Allocate(decelSegs);
		const float c = 1.0/dda.topSpeed;
		decelSegs->SetLinear(params.decelStartDistance - params.accelDistance, params.steadyClocks, c);
	}

	if (accelSegs != nullptr)
	{
		if (decelSegs != nullptr)
		{
			accelSegs->AddToTail(decelSegs);
		}
		return accelSegs;
	}

	return decelSegs;
}

// Calculate the additional acceleration distance needed if we shape the start of acceleration
inline float AxisShaper::GetExtraAccelStartDistance(const DDA& dda) const noexcept
{
	return (extraClocksAtStart * dda.startSpeed) + (extraDistanceAtStart * dda.acceleration);
}

// Calculate the additional acceleration distance needed if we shape the end of acceleration
inline float AxisShaper::GetExtraAccelEndDistance(const DDA& dda) const noexcept
{
	return (extraClocksAtEnd * dda.topSpeed) + (extraDistanceAtEnd * dda.acceleration);
}

inline float AxisShaper::GetExtraDecelStartDistance(const DDA& dda) const noexcept
{
	return (extraClocksAtStart * dda.topSpeed) - (extraDistanceAtStart * dda.deceleration);
}

// Calculate the additional deceleration distance needed if we shape the end of deceleration
inline float AxisShaper::GetExtraDecelEndDistance(const DDA& dda) const noexcept
{
	return (extraClocksAtEnd * dda.endSpeed) - (extraDistanceAtEnd * dda.deceleration);
}

/*static*/ MoveSegment *AxisShaper::GetUnshapedSegments(DDA& dda, const PrepParams& params) noexcept
{
	// Deceleration phase
	MoveSegment * tempSegments;
	if (params.decelClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(nullptr);
		const float b = dda.topSpeed/dda.deceleration;
		const float c = -2.0/dda.deceleration;
		tempSegments->SetNonLinear(params.decelDistance, params.decelClocks, b, c);
	}
	else
	{
		tempSegments = nullptr;
	}

	// Steady speed phase
	if (params.steadyClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(tempSegments);
		const float c = 1.0/dda.topSpeed;
		tempSegments->SetLinear(params.decelStartDistance - params.accelDistance, params.steadyClocks, c);
	}

	// Acceleration phase
	if (params.accelClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(tempSegments);
		const float b = dda.startSpeed/(-dda.acceleration);
		const float c = 2.0/dda.acceleration;
		tempSegments->SetNonLinear(params.accelDistance, params.accelClocks, b, c);
	}

	return tempSegments;
}

// End
