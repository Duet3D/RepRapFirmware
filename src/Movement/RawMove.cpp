/*
 * RawMove.cpp
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#include "RawMove.h"
#include <GCodes/GCodeQueue.h>

// Set up some default values in the move buffer for special moves, e.g. for Z probing and firmware retraction
void RawMove::SetDefaults(size_t firstDriveToZero) noexcept
{
	moveType = 0;
	isCoordinated = false;
	applyM220M221 = false;
	usingStandardFeedrate = false;
	usePressureAdvance = false;
	checkEndstops = false;
	reduceAcceleration = false;
	hasPositiveExtrusion = false;
	filePos = noFilePosition;
	tool = nullptr;
	cosXyAngle = 1.0;
	for (size_t drive = firstDriveToZero; drive < MaxAxesPlusExtruders; ++drive)
	{
		coords[drive] = 0.0;			// clear extrusion
	}
}

float MovementState::GetProportionDone() const noexcept
{
	return (float)(totalSegments - segmentsLeft)/(float)totalSegments;
}

void MovementState::Reset() noexcept
{
	ClearMove();
	filePos = noFilePosition;
	codeQueue->Clear();
	currentCoordinateSystem = 0;
	pausedInMacro = false;

	for (float& f : coords)
	{
		f = 0.0;									// clear out all axis and extruder coordinates
	}

	currentZHop = 0.0;							// clear this before calling ToolOffsetInverseTransform
	tool = nullptr;
	latestVirtualExtruderPosition = moveStartVirtualExtruderPosition = 0.0;
#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif
	ClearMove();
	updateUserPositionGb = nullptr;
	restartMoveFractionDone = 0.0;
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	fileOffsetToPrint = 0;
#endif
	for (RestorePoint& rp : numberedRestorePoints)
	{
		rp.Init();
	}
}

void MovementState::ChangeExtrusionFactor(unsigned int extruder, float multiplier) noexcept
{
	if (segmentsLeft != 0 && applyM220M221)
	{
		coords[ExtruderToLogicalDrive(extruder)] *= multiplier;		// last move not gone, so update it
	}
}

void MovementState::ClearMove() noexcept
{
	TaskCriticalSectionLocker lock;				// make sure that other tasks sees a consistent memory state

	segmentsLeft = 0;
	segMoveState = SegmentedMoveState::inactive;
	doingArcMove = false;
	checkEndstops = false;
	reduceAcceleration = false;
	moveType = 0;
	applyM220M221 = false;
	moveFractionToSkip = 0.0;
}

#if SUPPORT_ASYNC_MOVES

void AsyncMove::SetDefaults() noexcept
{
	for (float& f : movements)
	{
		f = 0.0;			// clear extrusion
	}
	startSpeed = endSpeed = 0.0;
}

#endif

// End
