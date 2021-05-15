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
	qq;
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
	qq;
}

// End
