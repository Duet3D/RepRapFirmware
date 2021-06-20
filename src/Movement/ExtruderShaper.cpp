/*
 * PressureAdvanceShaper.cpp
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#include "ExtruderShaper.h"

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include "StepTimer.h"
#include "DDA.h"
#include "MoveSegment.h"

MoveSegment *ExtruderShaper::GetSegments(const DDA &dda, const BasicPrepParams &params, float extrusionProportion) const noexcept
{
	const float compensationClocks = k * StepTimer::StepClockRate;
	const float accelExtraDistance = k * dda.acceleration * params.accelClocks;
	const float totalAccelDistance = params.accelDistance + accelExtraDistance;
	float actualTotalDistance = params.decelStartDistance + accelExtraDistance;			// this starts off being the decel start distance and ends up being the total distance

	// Deceleration phase
	float reverseStartDistance;
	float forwardExtrusion, reverseExtrusion;			// these need to be multiplied by extrusionProportion to get actual distances, then by steps/mm to get actual steps
	MoveSegment *segs = nullptr;
	if (params.decelDistance > 0.0)
	{
		// If we are using pressure advance then there may be a reverse phase. The motion constants are the same for both forward and reverse phases.
		// If there is a reverse phase then the stored distance limit is the point at which we reverse. The reverse phase will the be executed until the step count is correct.
		const float decelCompensationSpeedReduction = dda.deceleration * k;
		const bool hasReversePhase = dda.endSpeed < decelCompensationSpeedReduction;
		const bool hasForwardPhase = dda.topSpeed > decelCompensationSpeedReduction;
		const float initialDecelSpeed = dda.topSpeed - decelCompensationSpeedReduction;
		reverseStartDistance = fsquare(initialDecelSpeed)/(2.0 * dda.deceleration) + actualTotalDistance;
		const float uDivDminusP = (dda.topSpeed * StepTimer::StepClockRate)/dda.deceleration - compensationClocks;
		const float twoDistDivA = (2 * StepTimer::StepClockRateSquared * dda.totalDistance)/dda.deceleration;
		if (hasForwardPhase && hasReversePhase)
		{
			// Generate both forward and reverse deceleration segments
			forwardExtrusion = actualTotalDistance + fsquare(dda.topSpeed - decelCompensationSpeedReduction)/(2.0 * dda.deceleration);
			reverseExtrusion = fsquare(decelCompensationSpeedReduction - dda.endSpeed)/(2.0 * dda.deceleration);
			actualTotalDistance += forwardExtrusion + reverseExtrusion;
			const float forwardClocks = initialDecelSpeed/(dda.deceleration * StepTimer::StepClockRate);
			segs = MoveSegment::Allocate(nullptr);
			segs->SetNonLinear((params.decelStartDistance + fsquare(initialDecelSpeed)/(2.0 * dda.deceleration))/dda.totalDistance,
								params.decelClocks - forwardClocks,
								0.0,						// initial speed is zero
								twoDistDivA);
			segs->SetReverse();
			segs = MoveSegment::Allocate(segs);
			segs->SetNonLinear(1.0,
								forwardClocks,
								forwardClocks,				// clocks to zero speed is the same as the segment time
								twoDistDivA);
		}
		else if (hasForwardPhase)
		{
			// Generate a forward deceleration segment only
			forwardExtrusion = (fsquare(decelCompensationSpeedReduction - dda.topSpeed) - fsquare(decelCompensationSpeedReduction - dda.endSpeed))/(2.0 * dda.deceleration);
			reverseExtrusion = 0.0;
			actualTotalDistance += forwardExtrusion;
			segs = MoveSegment::Allocate(nullptr);
			segs->SetNonLinear(1.0,
								params.decelClocks,
								(dda.topSpeed - decelCompensationSpeedReduction)/(dda.deceleration * StepTimer::StepClockRate),
								twoDistDivA);
		}
		else if (hasReversePhase)
		{
			// Generate a reverse deceleration segment only
			forwardExtrusion = 0.0;
			reverseExtrusion = (fsquare(decelCompensationSpeedReduction - dda.endSpeed) - fsquare(decelCompensationSpeedReduction - dda.topSpeed))/(2.0 * dda.deceleration);
			actualTotalDistance -= reverseExtrusion;
			segs = MoveSegment::Allocate(nullptr);
			segs->SetNonLinear(1.0,
								params.decelClocks,
								(decelCompensationSpeedReduction - dda.topSpeed)/(dda.deceleration * StepTimer::StepClockRate),
								twoDistDivA);
			segs->SetReverse();
		}

		segs = MoveSegment::Allocate(nullptr);
		segs->SetNonLinear(actualTotalDistance/dda.totalDistance,
							reverseStartDistance,
								(2.0 * actualTotalDistance)/(dda.deceleration * StepTimer::StepClockRate),
								params.accelClocks + params.steadyClocks + uDivDminusP);
	}
	else
	{
		forwardExtrusion = reverseExtrusion = 0.0;
	}

	// Steady phase
	if (params.steadyClocks > 0.0)
	{
		segs = MoveSegment::Allocate(segs);
		const float ts = dda.topSpeed * StepTimer::StepClockRate;
		const float steadyClocks = dda.totalDistance/ts;
		const float dDivU = (dda.totalDistance * StepTimer::StepClockRate)/dda.topSpeed;
		segs->SetLinear(totalAccelDistance/dda.totalDistance, steadyClocks, dDivU);
	}

	// Acceleration phase
	if (params.accelDistance > 0)
	{
		const float uDivAplusP = (dda.startSpeed/dda.acceleration + k) * StepTimer::StepClockRate;
		segs = MoveSegment::Allocate(segs);
		segs->SetNonLinear(totalAccelDistance/dda.totalDistance,
							params.accelClocks,
							(2.0 * dda.totalDistance)/(dda.acceleration * StepTimer::StepClockRateSquared),
							-uDivAplusP);
	}

	return segs;
}

// End
