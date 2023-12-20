/*
 * InputShaper.cpp
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#include "AxisShaper.h"

#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include "StepTimer.h"
#include "DDA.h"
#include "MoveSegment.h"
#include "MoveDebugFlags.h"

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(AxisShaper, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(AxisShaper, _condition, __VA_ARGS__)

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
	{ "type", 					OBJECT_MODEL_FUNC(self->type.ToString()), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t AxisShaper::objectModelTableDescriptor[] = { 1, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(AxisShaper)

AxisShaper::AxisShaper() noexcept
	: type(InputShaperType::none),
	  frequency(DefaultFrequency),
	  zeta(DefaultDamping),
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
		reply.printf("Input shaping '%s' at %.1fHz damping factor %.2f", type.ToString(), (double)frequency, (double)zeta);
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
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}

#endif

void AxisShaper::Diagnostics(MessageType mtype) noexcept
{
	// We no longer report anything here
}

// End
