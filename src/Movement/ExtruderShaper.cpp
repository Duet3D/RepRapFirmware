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
//	const float compensationClocks = k * StepTimer::StepClockRate;
	const float accelExtraDistance = k * dda.acceleration * params.accelClocks;
	const float totalAccelDistance = params.accelDistance + accelExtraDistance;
	const float actualDecelStartDistance = params.decelStartDistance + accelExtraDistance;

	// Deceleration phase
	float forwardExtrusion, reverseExtrusion;			// these need to be multiplied by extrusionProportion to get actual distances, then by steps/mm to get actual steps
	MoveSegment *segs = nullptr;

	if (params.decelDistance > 0.0)
	{
		// If we are using pressure advance then there may be a reverse phase. The motion constants are the same for both forward and reverse phases.
		// We split the deceleration phase into two in this case.
		const float decelCompensationSpeedReduction = dda.deceleration * k;
		const bool hasReversePhase = dda.endSpeed < decelCompensationSpeedReduction;
		const bool hasForwardPhase = dda.topSpeed > decelCompensationSpeedReduction;
		const float twoDistDivA = (2 * StepTimer::StepClockRateSquared * dda.totalDistance)/dda.deceleration;
		const float forwardClocks = (!hasForwardPhase) ? 0.0
										: (hasReversePhase) ? (dda.topSpeed - decelCompensationSpeedReduction)/(dda.deceleration * StepTimer::StepClockRate)
											: params.decelClocks;
		if (hasReversePhase)
		{
			reverseExtrusion = fsquare(decelCompensationSpeedReduction - dda.endSpeed)/(2.0 * dda.deceleration);
			const float initialDecelSpeed = dda.topSpeed - decelCompensationSpeedReduction;
			const float reverseStartDistance = (hasForwardPhase) ? actualDecelStartDistance + fsquare(initialDecelSpeed)/(2.0 * dda.deceleration) : actualDecelStartDistance;
			forwardExtrusion = reverseStartDistance;
			reverseExtrusion = fsquare(decelCompensationSpeedReduction - dda.endSpeed)/(2.0 * dda.deceleration);
			segs = MoveSegment::Allocate(nullptr);
			segs->SetNonLinear(forwardExtrusion/dda.totalDistance,		// movement limit
								params.decelClocks - forwardClocks,
								0.0,									// initial speed is zero
								twoDistDivA);
			segs->SetReverse();
		}

		if (hasForwardPhase)
		{
			segs = MoveSegment::Allocate(segs);
			segs->SetNonLinear(1.0,
								forwardClocks,
								(hasReversePhase) ? forwardClocks : (dda.topSpeed - decelCompensationSpeedReduction)/(dda.deceleration * StepTimer::StepClockRate),
								twoDistDivA);
		}
	}
	else
	{
		forwardExtrusion = actualDecelStartDistance;
		reverseExtrusion = 0.0;
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

	(void)reverseExtrusion;		//TODO use this
	return segs;
}

// End
