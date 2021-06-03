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

MoveSegment *ExtruderShaper::GetSegments(const DDA &dda, const BasicPrepParams &params) const noexcept
{
	MoveSegment * const accelSegs = GetAccelerationSegments(dda, params);
	MoveSegment * const decelSegs = GetDecelerationSegments(dda, params);
	return FinishSegments(dda, params, accelSegs, decelSegs);
}

MoveSegment* ExtruderShaper::GetAccelerationSegments(const DDA &dda, const BasicPrepParams &params) const noexcept
{
	qq;
}

MoveSegment* ExtruderShaper::GetDecelerationSegments(const DDA &dda, const BasicPrepParams &params) const noexcept
{
	qq;
}

MoveSegment* ExtruderShaper::FinishSegments(const DDA &dda, const BasicPrepParams &params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept
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
		return accelSegs;
	}

	return decelSegs;
}

// End
