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
#include "Move.h"
#include "MoveDebugFlags.h"

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
#endif

constexpr float MaxZeta = 0.9;
constexpr float MaxZetaEI2 = 0.3;
constexpr float MaxZetaEI3 = 0.2;

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(AxisShaper, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(AxisShaper, _condition, __VA_ARGS__)
#define OBJECT_MODEL_ARRAY_COUNT(_value)		OBJECT_MODEL_ARRAY_COUNT_BODY(AxisShaper, _value)
#define OBJECT_MODEL_ARRAY_VALUE(...)			OBJECT_MODEL_ARRAY_VALUE_BODY(AxisShaper, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry AxisShaper::objectModelArrayTable[] =
{
	// 0. Amplitudes
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT(self->numImpulses),
		OBJECT_MODEL_ARRAY_VALUE((float)self->coefficients[context.GetLastIndex()], 3)
	},
	// 1. Delays
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT(self->numImpulses),
		OBJECT_MODEL_ARRAY_VALUE(self->delays[context.GetLastIndex()] * (1.0/StepClockRate), 5)
	}
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(AxisShaper)

constexpr ObjectModelTableEntry AxisShaper::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. InputShaper members
	{ "amplitudes",				OBJECT_MODEL_FUNC_ARRAY(0), 								ObjectModelEntryFlags::none },
	{ "damping",				OBJECT_MODEL_FUNC(self->zeta, 2), 							ObjectModelEntryFlags::none },
	{ "delays",					OBJECT_MODEL_FUNC_ARRAY(1), 								ObjectModelEntryFlags::none },
	{ "frequency",				OBJECT_MODEL_FUNC(self->frequency, 2), 						ObjectModelEntryFlags::none },
	{ "type", 					OBJECT_MODEL_FUNC(self->type.ToString()), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t AxisShaper::objectModelTableDescriptor[] = { 1, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(AxisShaper)

AxisShaper::AxisShaper() noexcept
	: type(InputShaperType::none),
	  frequency(DefaultFrequency),
	  zeta(DefaultDamping),
	  numImpulses(1), shapingTime(0), prepareAdvanceTime(MoveTiming::UsualMinimumPreparedTime)
{
	coefficients[0] = 1.0;
	delays[0] = 0;
}

// Process M593 (configure input shaping)
GCodeResult AxisShaper::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;

	// If we are changing the type, frequency, damping or custom parameters, we will change multiple stored values used by the motion planner, so wait until movement has stopped.
	if (gb.SeenAny("FSPHT"))
	{
		if (!reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
		seen = true;
	}

	InputShaperType newType(type);
	if (gb.Seen('P'))
	{
		String<StringLength20> shaperName;
		gb.GetReducedString(shaperName.GetRef());
		const InputShaperType providedType(shaperName.c_str());
		if (!providedType.IsValid())
		{
			reply.printf("Unknown input shaper type '%s'", shaperName.c_str());
			return GCodeResult::error;
		}
		newType = providedType;
	}

	float newFrequency = frequency;
	gb.TryGetLimitedFValue('F', newFrequency, seen, MinimumInputShapingFrequency, MaximumInputShapingFrequency);
	const float maxZeta = (newType == InputShaperType::ei2) ? MaxZetaEI2
							: (newType == InputShaperType::ei3) ? MaxZetaEI3
								: MaxZeta;
	float newZeta = zeta;
	gb.TryGetLimitedFValue('S', newZeta, seen, 0.0, maxZeta);

	if (seen)
	{
		// Calculate the parameters that define the input shaping, which are the number of segments, their amplitudes and their delays
		const float sqrtOneMinusZetaSquared = fastSqrtf(1.0 - fsquare(newZeta));
		const float dampedFrequency = newFrequency * sqrtOneMinusZetaSquared;
		const uint32_t dampedPeriod = lrintf(StepClockRate/dampedFrequency);
		const float k = expf(-newZeta * Pi/sqrtOneMinusZetaSquared);
		delays[0] = 0;								// this never changes

		switch (newType.RawValue())
		{
		case InputShaperType::none:
			numImpulses = 1;
			break;

		case InputShaperType::custom:
			{
				// Get the coefficients
				size_t numAmplitudes = MaxImpulses - 1;
				gb.MustSee('H');
				float fCoefficients[MaxImpulses - 1];
				gb.GetFloatArray(fCoefficients, numAmplitudes, false);

				// Get the impulse delays, if provided
				if (gb.Seen('T'))
				{
					float rawDelays[MaxImpulses - 1];
					size_t numDelays = MaxImpulses - 1;
					gb.GetFloatArray(rawDelays, numDelays, true);						//TODO delays must be positive and in increasing order

					// Check we have the same number of both
					if (numDelays != numAmplitudes)
					{
						reply.copy("Number of delays must be same as number of amplitudes");
						return GCodeResult::error;
					}

					// Check that the delays are all positive, distinct and in ascending order
					for (unsigned int i = 0; i < numAmplitudes; ++i)
					{
						if (rawDelays[i] <= 0 || (i != 0 && rawDelays[i] <= rawDelays[i - 1]))
						{
							reply.copy("Delays must be positive and in strictly increasing order");
							return GCodeResult::error;
						}
					}

					for (unsigned int i = 0; i < numAmplitudes; ++i)
					{
						delays[i + 1] = lrintf(rawDelays[i] * StepClockRate);			// convert from seconds to step clocks
					}
				}
				else
				{
					for (unsigned int i = 1; i <= numAmplitudes; ++i)
					{
						delays[i] = (dampedPeriod * i)/2;
					}
				}
				for (unsigned int i = 0; i < numAmplitudes; ++i)
				{
					coefficients[i] = (motioncalc_t)fCoefficients[i];
				}
				numImpulses = numAmplitudes + 1;
			}
			break;

		case InputShaperType::mzv:		// I can't find any references in the literature to this input shaper type, so the values are taken from Klipper source code
			{
				// Klipper gives amplitude steps of [a3 = k^2 * (1 - 1/sqrt(2)), a2 = k * (sqrt(2) - 1), a1 = 1 - 1/sqrt(2)] all divided by (a1 + a2 + a3)
				// Rearrange to: a3 = k^2 * (1 - sqrt(2)/2), a2 = k * (sqrt(2) - 1), a1 = (1 - sqrt(2)/2)
				const float kMzv = expf(-newZeta * 0.75 * Pi/sqrtOneMinusZetaSquared);
				const float a1 = 1.0 - 0.5 * sqrtf(2.0);
				const float a2 = (sqrtf(2.0) - 1.0) * kMzv;
				const float a3 = a1 * fsquare(kMzv);
			    const float sum = (a1 + a2 + a3);
			    coefficients[0] = a1/sum;
			    coefficients[1] = a2/sum;
			}
			delays[1] = (3 * dampedPeriod)/8;
			delays[2] = 2 * delays[1];
			numImpulses = 3;
			break;

		case InputShaperType::zvd:		// see https://www.researchgate.net/publication/316556412_INPUT_SHAPING_CONTROL_TO_REDUCE_RESIDUAL_VIBRATION_OF_A_FLEXIBLE_BEAM
			{
				const float j = fsquare(1.0 + k);
				coefficients[0] = 1.0/j;
				coefficients[1] = 2.0 * k/j;
			}
			delays[1] = dampedPeriod/2;
			delays[2] = dampedPeriod;
			numImpulses = 3;
			break;

		case InputShaperType::zvdd:		// see https://www.researchgate.net/publication/316556412_INPUT_SHAPING_CONTROL_TO_REDUCE_RESIDUAL_VIBRATION_OF_A_FLEXIBLE_BEAM
			{
				const float j = fcube(1.0 + k);
				coefficients[0] = 1.0/j;
				coefficients[1] = 3.0 * k/j;
				coefficients[2] = 3.0 * fsquare(k)/j;
			}
			delays[1] = dampedPeriod/2;
			delays[2] = dampedPeriod;
			delays[3] = (3 * dampedPeriod)/2;
			numImpulses = 4;
			break;

		case InputShaperType::zvddd:
			{
				const float j = fsquare(fsquare(1.0 + k));
				coefficients[0] = 1.0/j;
				coefficients[1] = 4.0 * k/j;
				coefficients[2] = 6.0 * fsquare(k)/j;
				coefficients[3] = 4.0 * fcube(k)/j;
			}
			delays[1] = dampedPeriod/2;
			delays[2] = dampedPeriod;
			delays[3] = (3 * dampedPeriod)/2;
			delays[4] = 2 * dampedPeriod;
			numImpulses = 5;
			break;

		case InputShaperType::ei2:		// see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.465.1337&rep=rep1&type=pdf. United States patent #4,916,635.
			{
				if (newZeta > MaxZetaEI2) { newZeta = MaxZetaEI2; }
				const float zetaSquared = fsquare(newZeta);
				const float zetaCubed = zetaSquared * newZeta;
				coefficients[0] = (0.16054) + ( 0.76699) * newZeta + ( 2.26560)	* zetaSquared + (-1.22750)	* zetaCubed;
				coefficients[1] = (0.33911) + ( 0.45081) * newZeta + (-2.58080)	* zetaSquared + ( 1.73650)	* zetaCubed;
				coefficients[2] = (0.34089)	+ (-0.61533) * newZeta + (-0.68765)	* zetaSquared + ( 0.42261)	* zetaCubed;
				delays[1] = lrintf((0.49890 + ( 0.16270) * newZeta + (-0.54262) * zetaSquared + (6.16180) * zetaCubed) * (float)dampedPeriod);
				delays[2] = lrintf((0.99748 + ( 0.18382) * newZeta + (-1.58270) * zetaSquared + (8.17120) * zetaCubed) * (float)dampedPeriod);
				delays[3] = lrintf((1.49920 + (-0.09297) * newZeta + (-0.28338) * zetaSquared + (1.85710) * zetaCubed) * (float)dampedPeriod);
			}
			numImpulses = 4;
			break;

		case InputShaperType::ei3:		// see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.465.1337&rep=rep1&type=pdf. United States patent #4,916,635
			{
				if (newZeta > MaxZetaEI3) { newZeta = MaxZetaEI3; }
				const float zetaSquared = fsquare(newZeta);
				const float zetaCubed = zetaSquared * newZeta;
				coefficients[0] = (0.11275)	+ ( 0.76632) * newZeta + ( 3.29160)	* zetaSquared + (-1.44380)	* zetaCubed;
				coefficients[1] = (0.23698)	+ ( 0.61164) * newZeta + (-2.57850)	* zetaSquared + ( 4.85220)	* zetaCubed;
				coefficients[2] = (0.30008)	+ (-0.19062) * newZeta + (-2.14560)	* zetaSquared + ( 0.13744)	* zetaCubed;
				coefficients[3] = (0.23775)	+ (-0.73297) * newZeta + ( 0.46885) * zetaSquared + (-2.08650)	* zetaCubed;
				delays[1] = lrintf((0.49974 + (0.23834)  * newZeta + (0.44559)  * zetaSquared + (12.4720) * zetaCubed) * (float)dampedPeriod);
				delays[2] = lrintf((0.99849 + (0.29808)  * newZeta + (-2.36460) * zetaSquared + (23.3990) * zetaCubed) * (float)dampedPeriod);
				delays[3] = lrintf((1.49870 + (0.10306)  * newZeta + (-2.01390) * zetaSquared + (17.0320) * zetaCubed) * (float)dampedPeriod);
				delays[4] = lrintf((1.99960 + (-0.28231) * newZeta + (0.61536)  * zetaSquared + (5.40450) * zetaCubed) * (float)dampedPeriod);
			}
			numImpulses = 5;
			break;
		}

		// If we get here then the new input shaping parameters have been accepted
		type = newType;
		frequency = newFrequency;
		zeta = newZeta;

		// The sum of the coefficients must total 1, use this to fill in the last coefficient
		// Also calculate the longest interval between adjacent impulses
		motioncalc_t sum = 0.0;
		uint32_t longestSegment = 0;
		for (size_t i = 0; i + 1 < numImpulses; ++i)
		{
			sum += coefficients[i];
			const uint32_t thisInterval = delays[i + 1] - delays[i];
			if (thisInterval > longestSegment)
			{
				longestSegment = thisInterval;
			}
		}
		coefficients[numImpulses - 1] = (motioncalc_t)1.0 - sum;
		shapingTime = delays[numImpulses - 1];
		prepareAdvanceTime = max<uint32_t>(longestSegment + MoveTiming::AbsoluteMinimumPreparedTime, MoveTiming::UsualMinimumPreparedTime);

		reprap.MoveUpdated();

#if SUPPORT_CAN_EXPANSION
		return UpdateRemoteInputShaping(reply);
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
		if (numImpulses > 1)
		{
			reply.cat(", impulses");
			for (unsigned int i = 0; i < numImpulses; ++i)
			{
				reply.catf(" %.3f", (double)coefficients[i]);
			}
			reply.cat(" with delays (ms)");
			for (unsigned int i = 0; i < numImpulses; ++i)
			{
				reply.catf(" %.2f", (double)(delays[i] * StepClocksToMillis));
			}
		}
	}
	return GCodeResult::ok;
}

#if SUPPORT_CAN_EXPANSION

GCodeResult AxisShaper::UpdateRemoteInputShaping(const StringRef& reply) const noexcept
{
	const ExpansionManager& expansion = reprap.GetExpansion();
	GCodeResult res = GCodeResult::ok;
	if (expansion.GetNumExpansionBoards() != 0)
	{
		// Build a CAN message to update the remote drivers
		for (uint8_t addr = 0; addr < CanId::MaxCanAddress; ++addr)
		{
			if (addr != CanInterface::GetCanAddress())
			{
				const ExpansionBoardData *boardData = expansion.GetBoardDetails(addr);
				if (boardData != nullptr && boardData->numDrivers != 0)
				{
					CanMessageBuffer *const buf = CanMessageBuffer::BlockingAllocate();
					const CanRequestId rid = CanInterface::AllocateRequestId(addr, buf);
					auto msg = buf->SetupRequestMessage<CanMessageSetInputShapingV1>(rid, CanInterface::GetCanAddress(), addr);
					msg->numImpulses = numImpulses;
					for (unsigned int i = 0; i < numImpulses; ++i)
					{
						msg->impulses[i].coefficient = (float)coefficients[i];
						msg->impulses[i].impulseDelay = delays[i];
					}
					buf->dataLength = msg->GetActualDataLength();
					msg->SetRequestId(rid);
					const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf, rid, reply, 0);
					if (rslt > res)
					{
						res = rslt;
					}
				}
			}
		}
	}
	return res;
}

#endif

#if SUPPORT_REMOTE_COMMANDS

// Handle a request from the master board to set input shaping parameters
GCodeResult AxisShaper::EutSetInputShaping(const CanMessageSetInputShapingV1& msg, size_t dataLength, const StringRef& reply) noexcept
{
	if (msg.numImpulses <= MaxImpulses && dataLength >= msg.GetActualDataLength())
	{
		numImpulses = msg.numImpulses;
		for (size_t i = 0; i < numImpulses; ++i)
		{
			coefficients[i] = msg.impulses[i].coefficient;
			delays[i] = msg.impulses[i].impulseDelay;
		}
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}

#endif

// End
