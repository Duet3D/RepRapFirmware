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

MoveSegment *ExtruderShaper::GetSegments(const DDA &dda, const BasicPrepParams &params, float extrusionAmount) const noexcept
{

	//TODO compute everything in step clocks?
	const float compensationClocks = k * StepTimer::StepClockRate;
	const float accelExtraDistance = k * dda.acceleration * params.accelClocks;
	const float totalAccelDistance = params.accelDistance + accelExtraDistance;
	float actualTotalDistance = params.decelStartDistance + accelExtraDistance;			// this starts off being the decel start distance and ends up being the total distance

	// Deceleration phase
	bool hasReversePhase;
	MoveSegment *segs = nullptr;
	if (params.decelDistance > 0.0)
	{
		// If we are using pressure advance then there may be a reverse phase. The motion constants are the same for both forward and reverse phases.
		// If there is a reverse phase then the stored distance limit is the point at which we reverse. The reverse phase will the be executed until the step count is correct.
		const float decelCompensationSpeedReduction = dda.deceleration * k;
		float distanceLimit;
		const float reverseStartDistance = fsquare(dda.topSpeed - decelCompensationSpeedReduction)/(2.0 * dda.deceleration) + actualTotalDistance;
		hasReversePhase = dda.endSpeed < decelCompensationSpeedReduction;
		const bool hasForwardPhase = dda.topSpeed > decelCompensationSpeedReduction;
		if (hasForwardPhase && hasReversePhase)
		{
			const float forwardDistance = fsquare(dda.topSpeed - decelCompensationSpeedReduction)/(2.0 * dda.deceleration);
			const float reverseDistance = fsquare(decelCompensationSpeedReduction - dda.endSpeed)/(2.0 * dda.deceleration);
			distanceLimit = actualTotalDistance + forwardDistance;
			actualTotalDistance = distanceLimit + reverseDistance;
		}
		else if (hasForwardPhase)
		{
			const float forwardDistance = (fsquare(decelCompensationSpeedReduction - dda.topSpeed) - fsquare(decelCompensationSpeedReduction - dda.endSpeed))/(2.0 * dda.deceleration);
			distanceLimit = actualTotalDistance + forwardDistance;
			actualTotalDistance = distanceLimit;
		}
		else if (hasReversePhase)
		{
			const float reverseDistance = (fsquare(decelCompensationSpeedReduction - dda.endSpeed) - fsquare(decelCompensationSpeedReduction - dda.topSpeed))/(2.0 * dda.deceleration);
			distanceLimit = actualTotalDistance;
			actualTotalDistance += reverseDistance;
		}

		const float uDivDminusP = (dda.topSpeed * StepTimer::StepClockRate)/dda.deceleration - compensationClocks;
		segs = MoveSegment::Allocate(nullptr);
		segs->SetNonLinear(actualTotalDistance/dda.totalDistance,
							reverseStartDistance,
								(2.0 * actualTotalDistance)/(dda.deceleration * StepTimer::StepClockRate),
								params.accelClocks + params.steadyClocks + uDivDminusP, dda.deceleration * StepTimer::StepClockRate);
	}

	// Steady phase
	if (params.steadyClocks > 0.0)
	{
		const float ts = dda.topSpeed * StepTimer::StepClockRate;
		segs = MoveSegment::Allocate(segs);
		segs->SetLinear((params.decelStartDistance + accelExtraDistance)/dda.totalDistance, dda.totalDistance/ts, dDivU);
	}

	// Acceleration phase
	if (params.accelDistance > 0)
	{
		const float uDivAplusP = (dda.startSpeed/dda.acceleration + k) * StepTimer::StepClockRate;
		segs = MoveSegment::Allocate(segs);
		segs->SetNonLinear(totalAccelDistance/dda.totalDistance,
							fsquare(uDivAplusP),
							(2.0 * dda.totalDistance)/(dda.acceleration * StepTimer::StepClockRateSquared),
							-uDivAplusP,
							dda.acceleration * StepTimer::StepClockRateSquared);
	}

	// Calculate the total movement, adjusted for pressure advance
	if (hasReversePhase)
	{
		const uint32_t forwardSteps = (uint32_t)(reverseStartDistance * effectiveStepsPerMm);
		reverseStartStep = forwardSteps + 1;
		totalSteps = 2 * forwardSteps - netSteps;
	}
	else
	{
		totalSteps = netSteps;
		reverseStartStep = totalSteps + 1;
	}




	MoveSegment * const accelSegs = GetAccelerationSegments(dda, params);
	MoveSegment * const decelSegs = GetDecelerationSegments(dda, params);
	return FinishSegments(dda, params, accelSegs, decelSegs);
}

// End
