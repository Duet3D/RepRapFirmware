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

constexpr ObjectModelArrayDescriptor AxisShaper::amplitudesArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return ((const AxisShaper*)self)->numExtraImpulses; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept
										-> ExpressionValue { return ExpressionValue(((const AxisShaper*)self)->coefficients[context.GetIndex(0)], 3); }
};

constexpr ObjectModelArrayDescriptor AxisShaper::durationsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return ((const AxisShaper*)self)->numExtraImpulses; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept
										-> ExpressionValue { return ExpressionValue(((const AxisShaper*)self)->durations[context.GetIndex(0)] * (1.0/StepClockRate), 5); }
};

constexpr ObjectModelTableEntry AxisShaper::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. InputShaper members
	{ "amplitudes",				OBJECT_MODEL_FUNC_NOSELF(&amplitudesArrayDescriptor), 		ObjectModelEntryFlags::none },
	{ "damping",				OBJECT_MODEL_FUNC(self->zeta, 2), 							ObjectModelEntryFlags::none },
	{ "durations",				OBJECT_MODEL_FUNC_NOSELF(&durationsArrayDescriptor), 		ObjectModelEntryFlags::none },
	{ "frequency",				OBJECT_MODEL_FUNC(self->frequency, 2), 						ObjectModelEntryFlags::none },
	{ "minAcceleration",		OBJECT_MODEL_FUNC(self->minimumAcceleration, 1),			ObjectModelEntryFlags::none },
	{ "type", 					OBJECT_MODEL_FUNC(self->type.ToString()), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t AxisShaper::objectModelTableDescriptor[] = { 1, 6 };

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

	// If we are changing the type, frequency, damping or custom parameters, we will change multiple stored values used by the motion planner, so wait until movement has stopped.
	// Changing just the minimum acceleration is OK because no other variables depend on it.
	if (gb.SeenAny("FSPHT"))
	{
		if (!reprap.GetGCodes().LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
	}

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

		case InputShaperType::mzv:		// I can't find any references in the literature to this input shaper type, so the values are taken from Klipper source code
			{
				// Klipper gives amplitude steps of [a3 = k^2 * (1 - 1/sqrt(2)), a2 = k * (sqrt(2) - 1), a1 = 1 - 1/sqrt(2)] all divided by (a1 + a2 + a3)
				// Rearrange to: a3 = k^2 * (1 - sqrt(2)/2), a2 = k * (sqrt(2) - 1), a1 = (1 - sqrt(2)/2)
				const float kMzv = expf(-zeta * 0.75 * Pi/sqrtOneMinusZetaSquared);
				const float a1 = 1.0 - 0.5 * sqrtf(2.0);
				const float a2 = (sqrtf(2.0) - 1.0) * kMzv;
				const float a3 = a1 * fsquare(kMzv);
			    const float sum = (a1 + a2 + a3);
			    coefficients[0] = a3/sum;
			    coefficients[1] = (a2 + a3)/sum;
			}
			durations[0] = durations[1] = 0.375 * dampedPeriod;
			numExtraImpulses = 2;
			break;

		case InputShaperType::zvd:		// see https://www.researchgate.net/publication/316556412_INPUT_SHAPING_CONTROL_TO_REDUCE_RESIDUAL_VIBRATION_OF_A_FLEXIBLE_BEAM
			{
				const float j = fsquare(1.0 + k);
				coefficients[0] = 1.0/j;
				coefficients[1] = coefficients[0] + 2.0 * k/j;
			}
			durations[0] = durations[1] = 0.5 * dampedPeriod;
			numExtraImpulses = 2;
			break;

		case InputShaperType::zvdd:		// see https://www.researchgate.net/publication/316556412_INPUT_SHAPING_CONTROL_TO_REDUCE_RESIDUAL_VIBRATION_OF_A_FLEXIBLE_BEAM
			{
				const float j = fcube(1.0 + k);
				coefficients[0] = 1.0/j;
				coefficients[1] = coefficients[0] + 3.0 * k/j;
				coefficients[2] = coefficients[1] + 3.0 * fsquare(k)/j;
			}
			durations[0] = durations[1] = durations[2] = 0.5 * dampedPeriod;
			numExtraImpulses = 3;
			break;

		case InputShaperType::zvddd:
			{
				const float j = fsquare(fsquare(1.0 + k));
				coefficients[0] = 1.0/j;
				coefficients[1] = coefficients[0] + 4.0 * k/j;
				coefficients[2] = coefficients[1] + 6.0 * fsquare(k)/j;
				coefficients[3] = coefficients[2] + 4.0 * fcube(k)/j;
			}
			durations[0] = durations[1] = durations[2] = durations[3] = 0.5 * dampedPeriod;
			numExtraImpulses = 4;
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
			overlappedDistancePerA = 0.0;
			float u = 0.0;
			for (unsigned int i = 0; i < 2 * numExtraImpulses; ++i)
			{
				overlappedCoefficients[i] /= maxVal;
				const float speedChange = overlappedCoefficients[i] * overlappedDurations[i];
				overlappedDistancePerA += (u + 0.5 * speedChange) * overlappedDurations[i];
				u += speedChange;
			}
			overlappedDeltaVPerA = u;
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
			if (reprap.Debug(moduleMove))
			{
				reply.catf(" odpa=%.4e odvpa=%.4e ovc=", (double)overlappedDistancePerA, (double)overlappedDeltaVPerA);
				for (unsigned int i = 0; i < 2 * numExtraImpulses; ++i)
				{
					reply.catf(" %.3f", (double)overlappedCoefficients[i]);
				}
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
	case InputShaperType::mzv:
	case InputShaperType::zvdd:
	case InputShaperType::zvddd:
	case InputShaperType::ei2:
	case InputShaperType::ei3:
		params.SetFromDDA(dda);															// set up the provisional parameters

		if (params.unshaped.accelDistance < params.unshaped.decelStartDistance)			// we can't do any shaping unless there is a steady speed segment that can be shortened
		{
			params.shaped = params.unshaped;
			//TODO if we want to shape both acceleration and deceleration but the steady distance is zero or too short, we could reduce the top speed
			if (params.unshaped.accelDistance > 0.0)
			{
				if ((dda.GetPrevious()->state != DDA::DDAState::frozen && dda.GetPrevious()->state != DDA::DDAState::executing) || !dda.GetPrevious()->flags.wasAccelOnlyMove)
				{
					TryShapeAccelBoth(dda, params);
				}
				else if (params.unshaped.accelClocks >= minimumShapingEndOriginalClocks)
				{
					TryShapeAccelEnd(dda, params);
				}
			}
			if (params.unshaped.decelStartDistance < dda.totalDistance)
			{
				if (dda.GetNext()->GetState() != DDA::DDAState::provisional || !dda.GetNext()->IsDecelerationMove())
				{
					TryShapeDecelBoth(dda, params);
				}
				else if (params.unshaped.decelClocks >= minimumShapingStartOriginalClocks)
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
		params.shaped.Finalise(dda.topSpeed);									// this sets up params.shaped.steadyClocks, which is needed by FinishShapedSegments
		dda.clocksNeeded = params.shaped.TotalClocks();
		dda.shapedSegments = FinishShapedSegments(dda, params, accelSegs, decelSegs);
		params.unshaped.steadyClocks = max<float>(dda.clocksNeeded - params.unshaped.accelClocks - params.unshaped.decelClocks, 0.0);
	}
	else
	{
		params.unshaped.Finalise(dda.topSpeed);									// this sets up params.steadyClocks
		dda.clocksNeeded = params.unshaped.TotalClocks();
	}

//	debugPrintf(" final plan %03x\n", (unsigned int)params.shapingPlan.all);
}

// Try to shape the end of the acceleration. We already know that there is sufficient acceleration time to do this, but we still need to check that there is enough distance.
void AxisShaper::TryShapeAccelEnd(const DDA& dda, PrepParams& params) const noexcept
{
	const float extraAccelDistance = GetExtraAccelEndDistance(dda.topSpeed, params.unshaped.acceleration);
	if (ImplementAccelShaping(dda, params, params.unshaped.accelDistance + extraAccelDistance, params.unshaped.accelClocks + extraClocksAtEnd))
	{
		params.shapingPlan.shapeAccelEnd = true;
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
	const float speedIncrease = dda.topSpeed - dda.startSpeed;
	if (speedIncrease <= overlappedDeltaVPerA * params.unshaped.acceleration)
	{
		// We can use overlapped shaping
		const float newAcceleration = speedIncrease/overlappedDeltaVPerA;
		if (newAcceleration >= minimumAcceleration)
		{
			const float newAccelDistance = (dda.startSpeed * overlappedShapingClocks) + (newAcceleration * overlappedDistancePerA);
			if (ImplementAccelShaping(dda, params, newAccelDistance, overlappedShapingClocks))
			{
				params.shapingPlan.shapeAccelOverlapped = true;
				params.shaped.acceleration = newAcceleration;
			}
		}
	}
	else if (params.unshaped.accelClocks < minimumNonOverlappedOriginalClocks)
	{
		// The speed change is too high to allow overlapping, but non-overlapped shaping will give a very short steady acceleration segment.
		// If we have enough spare distance, reduce the acceleration slightly to lengthen that segment.
		const float newAcceleration = speedIncrease/minimumNonOverlappedOriginalClocks;
		const float newUnshapedAccelDistance = (dda.startSpeed + 0.5 * newAcceleration * minimumNonOverlappedOriginalClocks) * minimumNonOverlappedOriginalClocks;
		const float extraAccelDistance = GetExtraAccelStartDistance(dda.startSpeed, newAcceleration) + GetExtraAccelEndDistance(dda.topSpeed, newAcceleration);
		if (ImplementAccelShaping(dda, params, newUnshapedAccelDistance + extraAccelDistance, minimumNonOverlappedOriginalClocks + extraClocksAtStart + extraClocksAtEnd))
		{
			params.shapingPlan.shapeAccelStart = params.shapingPlan.shapeAccelEnd = true;
			params.shaped.acceleration = newAcceleration;
			//params.shapingPlan.debugPrint = true;
		}
	}
	else
	{
		// We only attempt shaping if we can shape both the start and end of acceleration
		const float extraAccelDistance = GetExtraAccelStartDistance(dda.startSpeed, params.unshaped.acceleration) + GetExtraAccelEndDistance(dda.topSpeed, params.unshaped.acceleration);
		if (ImplementAccelShaping(dda, params, params.unshaped.accelDistance + extraAccelDistance, params.unshaped.accelClocks + extraClocksAtStart + extraClocksAtEnd))
		{
			params.shapingPlan.shapeAccelStart = params.shapingPlan.shapeAccelEnd = true;
		}
	}
}

// Check whether we can implement acceleration shaping using the proposed parameters; if so then implement it and return true; else return false with nothing changed
bool AxisShaper::ImplementAccelShaping(const DDA& dda, PrepParams& params, float newAccelDistance, float newAccelClocks) const noexcept
{
	if (newAccelDistance <= params.shaped.decelStartDistance)
	{
		const float speedIncrease = dda.topSpeed - dda.startSpeed;
		const float unshapedAccelClocks = 2 * (dda.topSpeed * newAccelClocks - newAccelDistance)/speedIncrease;
		const float unshapedAccelDistance = (dda.startSpeed + dda.topSpeed) * unshapedAccelClocks * 0.5;
		if (unshapedAccelDistance <= params.unshaped.decelStartDistance)
		{
			params.shaped.accelDistance = newAccelDistance;
			params.shaped.accelClocks = newAccelClocks;
			params.unshaped.accelClocks = unshapedAccelClocks;
			params.unshaped.accelDistance = unshapedAccelDistance;
			params.unshaped.acceleration = speedIncrease/unshapedAccelClocks;
			return true;
		}
	}

	return false;
}

// Try to shape the start of the deceleration. We already know that there is sufficient deceleration time to do this, but we still need to check that there is enough distance.
void AxisShaper::TryShapeDecelStart(const DDA& dda, PrepParams& params) const noexcept
{
	const float extraDecelDistance = GetExtraDecelStartDistance(dda.topSpeed, params.unshaped.deceleration);
	if (ImplementDecelShaping(dda, params, params.unshaped.decelStartDistance - extraDecelDistance, params.unshaped.decelClocks + extraClocksAtStart))
	{
		params.shapingPlan.shapeDecelStart = true;
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

void AxisShaper::TryShapeDecelBoth(DDA& dda, PrepParams& params) const noexcept
{
	const float speedDecrease = dda.topSpeed - dda.endSpeed;
	if (speedDecrease <= overlappedDeltaVPerA * params.unshaped.deceleration)
	{
		// We can use overlapped shaping
		const float newDeceleration = speedDecrease/overlappedDeltaVPerA;
		if (newDeceleration >= minimumAcceleration)
		{
			const float newDecelDistance = (dda.topSpeed * overlappedShapingClocks) - (newDeceleration * overlappedDistancePerA);
			if (ImplementDecelShaping(dda, params, dda.totalDistance - newDecelDistance, overlappedShapingClocks))
			{
				params.shapingPlan.shapeDecelOverlapped = true;
				params.shaped.deceleration = newDeceleration;
			}
		}
	}
	else if (params.unshaped.decelClocks < minimumNonOverlappedOriginalClocks)
	{
		// The speed change is too high to allow overlapping, but non-overlapped shaping will give a very short steady acceleration segment.
		// If we have enough spare distance, reduce the acceleration slightly to lengthen that segment.
		const float newDeceleration = speedDecrease/minimumNonOverlappedOriginalClocks;
		const float newUnshapedDecelDistance = (dda.endSpeed + (0.5 * newDeceleration * minimumNonOverlappedOriginalClocks)) * minimumNonOverlappedOriginalClocks;
		const float extraDecelDistance = GetExtraDecelStartDistance(dda.topSpeed, newDeceleration) + GetExtraDecelEndDistance(dda.endSpeed, newDeceleration);
		if (ImplementDecelShaping(dda, params, dda.totalDistance - (newUnshapedDecelDistance + extraDecelDistance), minimumNonOverlappedOriginalClocks + extraClocksAtStart + extraClocksAtEnd))
		{
			params.shapingPlan.shapeDecelStart = params.shapingPlan.shapeDecelEnd = true;
			params.shaped.deceleration = newDeceleration;
			//params.shapingPlan.debugPrint = true;
		}
	}
	else
	{
		// Only perform shaping if we can shape both the start and end of deceleration, otherwise we may not be able to generate a corresponding unshaped move because it might require negative steady distance
		const float extraDecelDistance = GetExtraDecelStartDistance(dda.topSpeed, params.unshaped.deceleration) + GetExtraDecelEndDistance(dda.endSpeed, params.unshaped.deceleration);
		if (ImplementDecelShaping(dda, params, params.unshaped.decelStartDistance - extraDecelDistance, params.unshaped.decelClocks + extraClocksAtStart + extraClocksAtEnd))
		{
			params.shapingPlan.shapeDecelStart = params.shapingPlan.shapeDecelEnd = true;
		}
	}
}

// Check whether we can implement acceleration shaping using the proposed parameters; if so then implement it and return true; else return false with nothing changed
bool AxisShaper::ImplementDecelShaping(const DDA& dda, PrepParams& params, float newDecelStartDistance, float newDecelClocks) const noexcept
{
	if (params.shaped.accelDistance <= newDecelStartDistance)
	{
		const float speedDecrease = dda.topSpeed - dda.endSpeed;
		const float unshapedDecelClocks = 2 * (dda.topSpeed * newDecelClocks - (dda.totalDistance - newDecelStartDistance))/speedDecrease;
		const float unshapedDecelDistance = (dda.topSpeed + dda.endSpeed) * unshapedDecelClocks * 0.5;
		if (params.unshaped.accelDistance + unshapedDecelDistance <= dda.totalDistance)
		{
			params.shaped.decelStartDistance = newDecelStartDistance;
			params.shaped.decelClocks = newDecelClocks;
			params.unshaped.decelClocks = unshapedDecelClocks;
			params.unshaped.decelStartDistance = dda.totalDistance - unshapedDecelDistance;
			params.unshaped.deceleration = speedDecrease/unshapedDecelClocks;
			return true;
		}
	}

	return false;
}

// If there is an acceleration phase, generate the acceleration segments according to the plan, and set the number of acceleration segments in the plan
MoveSegment *AxisShaper::GetAccelerationSegments(const DDA& dda, PrepParams& params) const noexcept
{
	if (params.shaped.accelDistance > 0.0)
	{
		if (params.shapingPlan.shapeAccelOverlapped)
		{
			MoveSegment *accelSegs = nullptr;
			float segStartSpeed = dda.topSpeed;
			for (unsigned int i = 2 * numExtraImpulses; i != 0; )
			{
				--i;
				accelSegs = MoveSegment::Allocate(accelSegs);
				const float acceleration = params.shaped.acceleration * overlappedCoefficients[i];
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
		float endDistance = params.shaped.accelDistance;
		MoveSegment *endAccelSegs = nullptr;
		if (params.shapingPlan.shapeAccelEnd)
		{
			// Shape the end of the acceleration
			float segStartSpeed = dda.topSpeed;
			for (unsigned int i = numExtraImpulses; i != 0; )
			{
				--i;
				endAccelSegs = MoveSegment::Allocate(endAccelSegs);
				const float acceleration = params.shaped.acceleration * (1.0 - coefficients[i]);
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
				const float acceleration = params.shaped.acceleration * coefficients[i];
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
			const float b = startSpeed/(-params.shaped.acceleration);
			const float c = 2.0/params.shaped.acceleration;
			endAccelSegs->SetNonLinear(endDistance - startDistance, params.shaped.accelClocks - accumulatedSegTime, b, c);
		}
		else if (reprap.Debug(moduleMove))
		{
			debugPrintf("Missing steady accel segment\n");
			params.shapingPlan.debugPrint = true;
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
MoveSegment *AxisShaper::GetDecelerationSegments(const DDA& dda, PrepParams& params) const noexcept
{
	if (params.shaped.decelStartDistance < dda.totalDistance)
	{
		if (params.shapingPlan.shapeDecelOverlapped)
		{
			MoveSegment *decelSegs = nullptr;
			float segStartSpeed = dda.endSpeed;
			for (unsigned int i = 2 * numExtraImpulses; i != 0; )
			{
				--i;
				decelSegs = MoveSegment::Allocate(decelSegs);
				const float deceleration = params.shaped.deceleration * overlappedCoefficients[i];
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
			for (unsigned int i = numExtraImpulses; i != 0; )
			{
				--i;
				endDecelSegs = MoveSegment::Allocate(endDecelSegs);
				const float deceleration = params.shaped.deceleration * (1.0 - coefficients[i]);
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

		float startDistance = params.shaped.decelStartDistance;
		float startSpeed = dda.topSpeed;
		MoveSegment *startDecelSegs = nullptr;
		if (params.shapingPlan.shapeDecelStart)
		{
			// Shape the start of the deceleration
			for (unsigned int i = 0; i < numExtraImpulses; ++i)
			{
				MoveSegment *seg = MoveSegment::Allocate(nullptr);
				const float deceleration = params.shaped.deceleration * coefficients[i];
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
			const float b = startSpeed/params.shaped.deceleration;
			const float c = -2.0/params.shaped.deceleration;
			endDecelSegs->SetNonLinear(endDistance - startDistance, params.shaped.decelClocks - accumulatedSegTime, b, c);
		}
		else if (reprap.Debug(moduleMove))
		{
			debugPrintf("Missing steady decel segment\n");
			params.shapingPlan.debugPrint = true;
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
MoveSegment *AxisShaper::FinishShapedSegments(const DDA& dda, const PrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept
{
	if (params.shaped.steadyClocks > 0.0)
	{
		// Insert a steady speed segment before the deceleration segments
		decelSegs = MoveSegment::Allocate(decelSegs);
		const float c = 1.0/dda.topSpeed;
		decelSegs->SetLinear(params.shaped.decelStartDistance - params.shaped.accelDistance, params.shaped.steadyClocks, c);
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
inline float AxisShaper::GetExtraAccelStartDistance(float startSpeed, float acceleration) const noexcept
{
	return (extraClocksAtStart * startSpeed) + (extraDistanceAtStart * acceleration);
}

// Calculate the additional acceleration distance needed if we shape the end of acceleration
inline float AxisShaper::GetExtraAccelEndDistance(float topSpeed, float acceleration) const noexcept
{
	return (extraClocksAtEnd * topSpeed) + (extraDistanceAtEnd * acceleration);
}

inline float AxisShaper::GetExtraDecelStartDistance(float topSpeed, float deceleration) const noexcept
{
	return (extraClocksAtStart * topSpeed) - (extraDistanceAtStart * deceleration);
}

// Calculate the additional deceleration distance needed if we shape the end of deceleration
inline float AxisShaper::GetExtraDecelEndDistance(float endSpeed, float deceleration) const noexcept
{
	return (extraClocksAtEnd * endSpeed) - (extraDistanceAtEnd * deceleration);
}

/*static*/ MoveSegment *AxisShaper::GetUnshapedSegments(DDA& dda, const PrepParams& params) noexcept
{
	// Deceleration phase
	MoveSegment * tempSegments;
	if (params.unshaped.decelClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(nullptr);
		const float b = dda.topSpeed/params.unshaped.deceleration;
		const float c = -2.0/params.unshaped.deceleration;
		tempSegments->SetNonLinear(dda.totalDistance - params.unshaped.decelStartDistance, params.unshaped.decelClocks, b, c);
	}
	else
	{
		tempSegments = nullptr;
	}

	// Steady speed phase
	if (params.unshaped.steadyClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(tempSegments);
		const float c = 1.0/dda.topSpeed;
		tempSegments->SetLinear(params.unshaped.decelStartDistance - params.unshaped.accelDistance, params.unshaped.steadyClocks, c);
	}

	// Acceleration phase
	if (params.unshaped.accelClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(tempSegments);
		const float b = dda.startSpeed/(-params.unshaped.acceleration);
		const float c = 2.0/params.unshaped.acceleration;
		tempSegments->SetNonLinear(params.unshaped.accelDistance, params.unshaped.accelClocks, b, c);
	}

	return tempSegments;
}

// End
