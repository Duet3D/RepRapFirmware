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

constexpr ObjectModelArrayTableEntry AxisShaper::objectModelArrayTable[] =
{
	// 0. Amplitudes
	{
		nullptr,					// no lock needed
		[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return ((const AxisShaper*)self)->numExtraImpulses; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept
											-> ExpressionValue { return ExpressionValue(((const AxisShaper*)self)->coefficients[context.GetLastIndex()], 3); }
	},
	// 1. Durations
	{
		nullptr,					// no lock needed
		[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t { return ((const AxisShaper*)self)->numExtraImpulses; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept
											-> ExpressionValue { return ExpressionValue(((const AxisShaper*)self)->durations[context.GetLastIndex()] * (1.0/StepClockRate), 5); }
	}
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(AxisShaper)

constexpr ObjectModelTableEntry AxisShaper::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. InputShaper members
	{ "amplitudes",				OBJECT_MODEL_FUNC_ARRAY(0), 								ObjectModelEntryFlags::none },
	{ "damping",				OBJECT_MODEL_FUNC(self->zeta, 2), 							ObjectModelEntryFlags::none },
	{ "durations",				OBJECT_MODEL_FUNC_ARRAY(1), 								ObjectModelEntryFlags::none },
	{ "frequency",				OBJECT_MODEL_FUNC(self->frequency, 2), 						ObjectModelEntryFlags::none },
	{ "minAcceleration",		OBJECT_MODEL_FUNC(self->minimumAcceleration, 1),			ObjectModelEntryFlags::none },
	{ "type", 					OBJECT_MODEL_FUNC(self->type.ToString()), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t AxisShaper::objectModelTableDescriptor[] = { 1, 6 };

DEFINE_GET_OBJECT_MODEL_TABLE(AxisShaper)

AxisShaper::AxisShaper() noexcept
	: type(InputShaperType::none),
	  frequency(DefaultFrequency),
	  zeta(DefaultDamping),
	  minimumAcceleration(ConvertAcceleration(DefaultMinimumAcceleration)),
	  numExtraImpulses(0)
{
}

// Process M593 (configure input shaping)
GCodeResult AxisShaper::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	constexpr float MinimumInputShapingFrequency = (float)StepClockRate/(2 * 65535);		// we use a 16-bit number of step clocks to represent half the input shaping period
	constexpr float MaximumInputShapingFrequency = 1000.0;
	bool seen = false;

	// If we are changing the type, frequency, damping or custom parameters, we will change multiple stored values used by the motion planner, so wait until movement has stopped.
	// Changing just the minimum acceleration is OK because no other variables depend on it.
	if (gb.SeenAny("FSPHT"))
	{
		if (!reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(gb))
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
		minimumAcceleration = ConvertAcceleration(max<float>(gb.GetNonNegativeFValue(), 1.0));			// very low accelerations cause problems with the maths
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
		type = InputShaperType::zvd;
	}

	if (seen)
	{
		// Calculate the parameters that define the input shaping, which are the number of extra acceleration segments, and their amplitudes and durations
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

		CalculateDerivedParameters();
		reprap.MoveUpdated();

#if SUPPORT_CAN_EXPANSION
		return reprap.GetPlatform().UpdateRemoteInputShaping(numExtraImpulses, coefficients, durations, reply);
#else
		// Fall through to return GCodeResult::ok
#endif
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
			if (reprap.Debug(Module::Move))
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

#if SUPPORT_REMOTE_COMMANDS

// Handle a request from the master board to set input shaping parameters
GCodeResult AxisShaper::EutSetInputShaping(const CanMessageSetInputShaping& msg, size_t dataLength, const StringRef& reply) noexcept
{
	if (msg.numExtraImpulses <= MaxExtraImpulses && dataLength >= msg.GetActualDataLength())
	{
		numExtraImpulses = msg.numExtraImpulses;
		for (size_t i = 0; i < numExtraImpulses; ++i)
		{
			coefficients[i] = msg.impulses[i].coefficient;
			durations[i] = msg.impulses[i].duration;
		}
		CalculateDerivedParameters();
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}

#endif

// Calculate the input shaping parameters that we can derive from the primary ones
void AxisShaper::CalculateDerivedParameters() noexcept
{
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
}

// Plan input shaping, generate the MoveSegment, and set up the basic move parameters.
// On entry, params.shapingPlan is set to 'no shaping'.
// Currently we use a single input shaper for all axes, so the move segments are attached to the DDA not the DM
void AxisShaper::PlanShaping(DDA& dda, PrepParams& params, bool shapingEnabled) const noexcept
{
	params.SetFromDDA(dda);												// set up the provisional parameters
	if (numExtraImpulses != 0)
	{
		// Work out what the ideal plan is, then see how far we can achieve it
		InputShaperPlan idealPlan;
		const DDA *const nextDda = dda.GetNext();
		const DDA *const prevDda = dda.GetPrevious();

		if (params.accelDistance >= 0.0 && params.accelDistance < dda.totalDistance)
		{
			const DDA::DDAState prevState = prevDda->state;
			if ((prevState != DDA::DDAState::frozen && prevState != DDA::DDAState::executing) || !prevDda->flags.wasAccelOnlyMove)
			{
				idealPlan.shapeAccelStart = true;
			}
			if (params.accelDistance < dda.totalDistance || (nextDda->state == DDA::DDAState::provisional && nextDda->startSpeed >= nextDda->topSpeed))
			{
				idealPlan.shapeAccelEnd = true;
			}
		}

		if (params.decelStartDistance > 0.0 && params.decelStartDistance < dda.totalDistance)
		{
			const DDA::DDAState nextState = nextDda->state;
			if (nextState != DDA::DDAState::provisional || !nextDda->IsDecelerationMove())
			{
				idealPlan.shapeDecelEnd = true;
			}
			if (params.decelStartDistance > 0.0 || prevDda->endSpeed >= prevDda->topSpeed)
			{
				idealPlan.shapeDecelStart = true;
			}
		}

		if (idealPlan.IsShaped())
		{
			if (params.accelDistance > 0.0 && params.decelStartDistance < dda.totalDistance)
			{
				// This move has both acceleration and deceleration. Therefore we can reduce the top speed in order to increase the steady speed phase,
				// which may allow us to apply input shaping.
				//TODO
			}

			if (params.accelDistance < params.decelStartDistance)			// we can't do any shaping unless there is a steady speed segment that can be shortened
			{
				if (params.accelDistance > 0.0 && idealPlan.shapeAccelEnd)
				{
					if (idealPlan.shapeAccelStart)
					{
						TryShapeAccelBoth(dda, params);
					}
					else if (params.accelClocks >= minimumShapingEndOriginalClocks)
					{
						TryShapeAccelEnd(dda, params);
					}
				}
				if (params.decelStartDistance < dda.totalDistance && idealPlan.shapeDecelStart)
				{
					if (idealPlan.shapeDecelEnd)
					{
						TryShapeDecelBoth(dda, params);
					}
					else if (params.decelClocks >= minimumShapingStartOriginalClocks)
					{
						TryShapeDecelStart(dda, params);
					}
				}
			}
		}
	}

	if (reprap.Debug(Module::Move) && reprap.Debug(Module::Dda))
	{
		debugPrintf("plan=%u ad=%.4e dd=%.4e\n", (unsigned int)params.shapingPlan.condensedPlan, (double)params.accelDistance, (double)(params.totalDistance - params.decelStartDistance));
	}

	// If we are doing any input shaping then set up dda.shapedSegments, else leave it as null
	if (params.shapingPlan.IsShaped())
	{
		MoveSegment * const accelSegs = GetAccelerationSegments(dda, params);
		MoveSegment * const decelSegs = GetDecelerationSegments(dda, params);
		params.Finalise(dda.topSpeed);									// this sets up params.shaped.steadyClocks, which is needed by FinishShapedSegments
		dda.clocksNeeded = params.TotalClocks();
		dda.segments = FinishShapedSegments(dda, params, accelSegs, decelSegs);
	}
	else
	{
		params.Finalise(dda.topSpeed);									// this sets up params.steadyClocks
		dda.clocksNeeded = params.TotalClocks();
	}

//	debugPrintf(" final plan %03x\n", (unsigned int)params.shapingPlan.all);
}

#if SUPPORT_REMOTE_COMMANDS

// Calculate up the shaped segments for a move. The only field in PrepParams that this ever modifies is the debugPrint flag.
void AxisShaper::GetRemoteSegments(DDA& dda, PrepParams& params) const noexcept
{
	// Do the acceleration phase
	float accelDistanceExTopSpeedPerA;									// the distance needed for acceleration minus the contribution from the top speed, per unit acceleration, in stepClocks^2
	float effectiveAccelTime;
	if (params.shapingPlan.shapeAccelOverlapped)
	{
		effectiveAccelTime = overlappedDeltaVPerA;
		accelDistanceExTopSpeedPerA = overlappedDistancePerA - effectiveAccelTime * params.accelClocks;
	}
	else
	{
		effectiveAccelTime = params.accelClocks;
		accelDistanceExTopSpeedPerA = 0.0;
		if (params.shapingPlan.shapeAccelEnd)
		{
			effectiveAccelTime -= extraClocksAtEnd;
			accelDistanceExTopSpeedPerA += extraDistanceAtEnd;
		}
		if (params.shapingPlan.shapeAccelStart)
		{
			effectiveAccelTime -= extraClocksAtStart;
			accelDistanceExTopSpeedPerA += extraDistanceAtStart - effectiveAccelTime * extraClocksAtStart;
		}
		accelDistanceExTopSpeedPerA -= 0.5 * fsquare(effectiveAccelTime);
	}
	const float accelDistanceExTopSpeed = accelDistanceExTopSpeedPerA * params.acceleration;

	// Do the deceleration phase
	float decelDistanceExTopSpeedPerA;									// the distance needed for deceleration minus the contribution from the top speed
	float effectiveDecelTime;
	if (params.shapingPlan.shapeDecelOverlapped)
	{
		effectiveDecelTime = overlappedDeltaVPerA;
		decelDistanceExTopSpeedPerA = -overlappedDistancePerA;
	}
	else
	{
		effectiveDecelTime = params.decelClocks;
		decelDistanceExTopSpeedPerA = 0.0;
		if (params.shapingPlan.shapeDecelStart)
		{
			effectiveDecelTime -= extraClocksAtStart;
			decelDistanceExTopSpeedPerA -= extraDistanceAtStart;
		}
		if (params.shapingPlan.shapeDecelEnd)
		{
			effectiveDecelTime -= extraClocksAtEnd;
			decelDistanceExTopSpeedPerA -= extraDistanceAtEnd + effectiveDecelTime * extraClocksAtEnd;
		}
		decelDistanceExTopSpeedPerA -= 0.5 * fsquare(effectiveDecelTime);
	}
	const float decelDistanceExTopSpeed = decelDistanceExTopSpeedPerA * params.deceleration;

	dda.topSpeed = (1.0 - accelDistanceExTopSpeed - decelDistanceExTopSpeed)/dda.clocksNeeded;
	dda.startSpeed = dda.topSpeed - params.acceleration * effectiveAccelTime;
	dda.endSpeed = dda.topSpeed - params.deceleration * effectiveDecelTime;
	params.accelDistance =      accelDistanceExTopSpeed + dda.topSpeed * params.accelClocks;
	const float decelDistance = decelDistanceExTopSpeed + dda.topSpeed * params.decelClocks;
	params.decelStartDistance =  1.0 - decelDistance;

	if (reprap.Debug(Module::Move) && reprap.Debug(Module::Dda))
	{
		debugPrintf("plan=%u ad=%.4e dd=%.4e\n", (unsigned int)params.shapingPlan.condensedPlan, (double)params.accelDistance, (double)decelDistance);
	}

	MoveSegment * const accelSegs = GetAccelerationSegments(dda, params);
	MoveSegment * const decelSegs = GetDecelerationSegments(dda, params);
	dda.segments = FinishShapedSegments(dda, params, accelSegs, decelSegs);
}

#endif

// Try to shape the end of the acceleration. We already know that there is sufficient acceleration time to do this, but we still need to check that there is enough distance.
void AxisShaper::TryShapeAccelEnd(const DDA& dda, PrepParams& params) const noexcept
{
	const float extraAccelDistance = GetExtraAccelEndDistance(dda.topSpeed, params.acceleration);
	if (ImplementAccelShaping(dda, params, params.accelDistance + extraAccelDistance, params.accelClocks + extraClocksAtEnd))
	{
		params.shapingPlan.shapeAccelEnd = true;
	}
	else
	{
		// Not enough constant speed time to the acceleration shaping
		if (reprap.Debug(Module::Module::Dda))
		{
			debugPrintf("Can't shape accel end\n");
		}
	}
}

void AxisShaper::TryShapeAccelBoth(DDA& dda, PrepParams& params) const noexcept
{
	const float speedIncrease = dda.topSpeed - dda.startSpeed;
	if (speedIncrease <= overlappedDeltaVPerA * params.acceleration)
	{
		// We can use overlapped shaping
		const float newAcceleration = speedIncrease/overlappedDeltaVPerA;
		if (newAcceleration >= minimumAcceleration)
		{
			const float newAccelDistance = (dda.startSpeed * overlappedShapingClocks) + (newAcceleration * overlappedDistancePerA);
			if (ImplementAccelShaping(dda, params, newAccelDistance, overlappedShapingClocks))
			{
				params.shapingPlan.shapeAccelOverlapped = true;
				params.acceleration = newAcceleration;
			}
		}
	}
	else if (params.accelClocks < minimumNonOverlappedOriginalClocks)
	{
		// The speed change is too high to allow overlapping, but non-overlapped shaping will give a very short steady acceleration segment.
		// If we have enough spare distance, reduce the acceleration slightly to lengthen that segment.
		const float newAcceleration = speedIncrease/minimumNonOverlappedOriginalClocks;
		const float newUnshapedAccelDistance = (dda.startSpeed + 0.5 * newAcceleration * minimumNonOverlappedOriginalClocks) * minimumNonOverlappedOriginalClocks;
		const float extraAccelDistance = GetExtraAccelStartDistance(dda.startSpeed, newAcceleration) + GetExtraAccelEndDistance(dda.topSpeed, newAcceleration);
		if (ImplementAccelShaping(dda, params, newUnshapedAccelDistance + extraAccelDistance, minimumNonOverlappedOriginalClocks + extraClocksAtStart + extraClocksAtEnd))
		{
			params.shapingPlan.shapeAccelStart = params.shapingPlan.shapeAccelEnd = true;
			params.acceleration = newAcceleration;
			//params.shapingPlan.debugPrint = true;
		}
	}
	else
	{
		// We only attempt shaping if we can shape both the start and end of acceleration
		const float extraAccelDistance = GetExtraAccelStartDistance(dda.startSpeed, params.acceleration) + GetExtraAccelEndDistance(dda.topSpeed, params.acceleration);
		if (ImplementAccelShaping(dda, params, params.accelDistance + extraAccelDistance, params.accelClocks + extraClocksAtStart + extraClocksAtEnd))
		{
			params.shapingPlan.shapeAccelStart = params.shapingPlan.shapeAccelEnd = true;
		}
	}
}

// Check whether we can implement acceleration shaping using the proposed parameters; if so then implement it and return true; else return false with nothing changed
bool AxisShaper::ImplementAccelShaping(const DDA& dda, PrepParams& params, float newAccelDistance, float newAccelClocks) const noexcept
{
	if (newAccelDistance <= params.decelStartDistance)
	{
		const float speedIncrease = dda.topSpeed - dda.startSpeed;
		const float unshapedAccelClocks = 2 * (dda.topSpeed * newAccelClocks - newAccelDistance)/speedIncrease;
		const float unshapedAccelDistance = (dda.startSpeed + dda.topSpeed) * unshapedAccelClocks * 0.5;
		if (unshapedAccelDistance <= params.decelStartDistance)
		{
			params.accelDistance = newAccelDistance;
			params.accelClocks = newAccelClocks;
			return true;
		}
	}

	return false;
}

// Try to shape the start of the deceleration. We already know that there is sufficient deceleration time to do this, but we still need to check that there is enough distance.
void AxisShaper::TryShapeDecelStart(const DDA& dda, PrepParams& params) const noexcept
{
	const float extraDecelDistance = GetExtraDecelStartDistance(dda.topSpeed, params.deceleration);
	if (ImplementDecelShaping(dda, params, params.decelStartDistance - extraDecelDistance, params.decelClocks + extraClocksAtStart))
	{
		params.shapingPlan.shapeDecelStart = true;
	}
	else
	{
		// Not enough constant speed time to do deceleration shaping
		if (reprap.Debug(Module::Module::Dda))
		{
			debugPrintf("Can't shape decel start\n");
		}
	}
}

void AxisShaper::TryShapeDecelBoth(DDA& dda, PrepParams& params) const noexcept
{
	const float speedDecrease = dda.topSpeed - dda.endSpeed;
	if (speedDecrease <= overlappedDeltaVPerA * params.deceleration)
	{
		// We can use overlapped shaping
		const float newDeceleration = speedDecrease/overlappedDeltaVPerA;
		if (newDeceleration >= minimumAcceleration)
		{
			const float newDecelDistance = (dda.topSpeed * overlappedShapingClocks) - (newDeceleration * overlappedDistancePerA);
			if (ImplementDecelShaping(dda, params, dda.totalDistance - newDecelDistance, overlappedShapingClocks))
			{
				params.shapingPlan.shapeDecelOverlapped = true;
				params.deceleration = newDeceleration;
			}
		}
	}
	else if (params.decelClocks < minimumNonOverlappedOriginalClocks)
	{
		// The speed change is too high to allow overlapping, but non-overlapped shaping will give a very short steady acceleration segment.
		// If we have enough spare distance, reduce the acceleration slightly to lengthen that segment.
		const float newDeceleration = speedDecrease/minimumNonOverlappedOriginalClocks;
		const float newUnshapedDecelDistance = (dda.endSpeed + (0.5 * newDeceleration * minimumNonOverlappedOriginalClocks)) * minimumNonOverlappedOriginalClocks;
		const float extraDecelDistance = GetExtraDecelStartDistance(dda.topSpeed, newDeceleration) + GetExtraDecelEndDistance(dda.endSpeed, newDeceleration);
		if (ImplementDecelShaping(dda, params, dda.totalDistance - (newUnshapedDecelDistance + extraDecelDistance), minimumNonOverlappedOriginalClocks + extraClocksAtStart + extraClocksAtEnd))
		{
			params.shapingPlan.shapeDecelStart = params.shapingPlan.shapeDecelEnd = true;
			params.deceleration = newDeceleration;
			//params.shapingPlan.debugPrint = true;
		}
	}
	else
	{
		// Only perform shaping if we can shape both the start and end of deceleration, otherwise we may not be able to generate a corresponding unshaped move because it might require negative steady distance
		const float extraDecelDistance = GetExtraDecelStartDistance(dda.topSpeed, params.deceleration) + GetExtraDecelEndDistance(dda.endSpeed, params.deceleration);
		if (ImplementDecelShaping(dda, params, params.decelStartDistance - extraDecelDistance, params.decelClocks + extraClocksAtStart + extraClocksAtEnd))
		{
			params.shapingPlan.shapeDecelStart = params.shapingPlan.shapeDecelEnd = true;
		}
	}
}

// Check whether we can implement deceleration shaping using the proposed parameters; if so then implement it and return true; else return false with nothing changed
bool AxisShaper::ImplementDecelShaping(const DDA& dda, PrepParams& params, float newDecelStartDistance, float newDecelClocks) const noexcept
{
	if (params.accelDistance <= newDecelStartDistance)
	{
		const float speedDecrease = dda.topSpeed - dda.endSpeed;
		const float unshapedDecelClocks = 2 * (dda.topSpeed * newDecelClocks - (dda.totalDistance - newDecelStartDistance))/speedDecrease;
		const float unshapedDecelDistance = (dda.topSpeed + dda.endSpeed) * unshapedDecelClocks * 0.5;
		if (params.accelDistance + unshapedDecelDistance <= dda.totalDistance)
		{
			params.decelStartDistance = newDecelStartDistance;
			params.decelClocks = newDecelClocks;
			return true;
		}
	}

	return false;
}

// If there is an acceleration phase, generate the acceleration segments according to the plan, and set the number of acceleration segments in the plan
MoveSegment *AxisShaper::GetAccelerationSegments(const DDA& dda, PrepParams& params) const noexcept
{
	if (params.accelDistance > 0.0)
	{
		if (params.shapingPlan.shapeAccelOverlapped)
		{
			MoveSegment *accelSegs = nullptr;
			float segStartSpeed = dda.topSpeed;
			for (unsigned int i = 2 * numExtraImpulses; i != 0; )
			{
				--i;
				accelSegs = MoveSegment::Allocate(accelSegs);
				const float acceleration = params.acceleration * overlappedCoefficients[i];
				const float segTime = overlappedDurations[i];
				const float speedIncrease = acceleration * segTime;
				segStartSpeed -= speedIncrease;
				const float b = segStartSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float segLen = (segStartSpeed + (0.5 * speedIncrease)) * segTime;
				accelSegs->SetNonLinear(segLen, segTime, b, c, acceleration);
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
			for (unsigned int i = numExtraImpulses; i != 0; )
			{
				--i;
				endAccelSegs = MoveSegment::Allocate(endAccelSegs);
				const float acceleration = params.acceleration * (1.0 - coefficients[i]);
				const float segTime = durations[i];
				const float speedIncrease = acceleration * segTime;
				segStartSpeed -= speedIncrease;
				const float b = segStartSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float segLen = (segStartSpeed + (0.5 * speedIncrease)) * segTime;
				endDistance -= segLen;
				endAccelSegs->SetNonLinear(segLen, segTime, b, c, acceleration);
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
				const float acceleration = params.acceleration * coefficients[i];
				const float segTime = durations[i];
				const float b = startSpeed/(-acceleration);
				const float c = 2.0/acceleration;
				const float speedIncrease = acceleration * segTime;
				const float segLen = (startSpeed + (0.5 * speedIncrease)) * segTime;
				startDistance += segLen;
				seg->SetNonLinear(segLen, segTime, b, c, acceleration);
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
			const float b = startSpeed/(-params.acceleration);
			const float c = 2.0/params.acceleration;
			const float segTime = params.accelClocks - accumulatedSegTime;
			endAccelSegs->SetNonLinear(endDistance - startDistance, segTime, b, c, params.acceleration);
		}
		else if (reprap.Debug(Module::Move))
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
	if (params.decelStartDistance < dda.totalDistance)
	{
		if (params.shapingPlan.shapeDecelOverlapped)
		{
			MoveSegment *decelSegs = nullptr;
			float segStartSpeed = dda.endSpeed;
			for (unsigned int i = 2 * numExtraImpulses; i != 0; )
			{
				--i;
				decelSegs = MoveSegment::Allocate(decelSegs);
				const float deceleration = params.deceleration * overlappedCoefficients[i];
				const float segTime = overlappedDurations[i];
				const float speedDecrease = deceleration * segTime;
				segStartSpeed += speedDecrease;
				const float b = segStartSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float segLen = (segStartSpeed + (-0.5 * speedDecrease)) * segTime;
				decelSegs->SetNonLinear(segLen, segTime, b, c, -deceleration);
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
				const float deceleration = params.deceleration * (1.0 - coefficients[i]);
				const float segTime = durations[i];
				const float speedDecreasee = deceleration * segTime;
				segStartSpeed += speedDecreasee;
				const float b = segStartSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float segLen = (segStartSpeed + (-0.5 * speedDecreasee)) * segTime;
				endDecelSegs->SetNonLinear(segLen, segTime, b, c, -deceleration);
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
				const float deceleration = params.deceleration * coefficients[i];
				const float segTime = durations[i];
				const float b = startSpeed/deceleration;
				const float c = -2.0/deceleration;
				const float speedDecrease = deceleration * segTime;
				const float segLen = (startSpeed + (-0.5 * speedDecrease)) * segTime;
				startDistance += segLen;
				seg->SetNonLinear(segLen, segTime, b, c, -deceleration);
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
			const float b = startSpeed/params.deceleration;
			const float c = -2.0/params.deceleration;
			const float segTime = params.decelClocks - accumulatedSegTime;
			endDecelSegs->SetNonLinear(endDistance - startDistance, segTime, b, c, -params.deceleration);
		}
		else if (reprap.Debug(Module::Move))
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

// Generate the steady speed segment (if any), tack all the segments together, and return them
// Must set up params.steadyClocks before calling this
MoveSegment *AxisShaper::FinishShapedSegments(const DDA& dda, const PrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept
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

// Calculate the move segments when input shaping is not in use
/*static*/ MoveSegment *AxisShaper::GetUnshapedSegments(DDA& dda, const PrepParams& params) noexcept
{
	// Deceleration phase
	MoveSegment * tempSegments;
	if (params.decelClocks > 0.0)
	{
		tempSegments = MoveSegment::Allocate(nullptr);
		const float b = dda.topSpeed/params.deceleration;
		const float c = -2.0/params.deceleration;
		tempSegments->SetNonLinear(dda.totalDistance - params.decelStartDistance, params.decelClocks, b, c, -params.deceleration);
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
		const float b = dda.startSpeed/(-params.acceleration);
		const float c = 2.0/params.acceleration;
		tempSegments->SetNonLinear(params.accelDistance, params.accelClocks, b, c, params.acceleration);
	}

	return tempSegments;
}

// End
