/*
 * RawMove.cpp
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#include "RawMove.h"
#include <GCodes/GCodes.h>
#include <GCodes/GCodeQueue.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Tools/Tool.h>

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
	currentTool = nullptr;
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

	maxPrintingAcceleration = ConvertAcceleration(DefaultPrintingAcceleration);
	maxTravelAcceleration = ConvertAcceleration(DefaultTravelAcceleration);

	currentZHop = 0.0;								// clear this before calling ToolOffsetInverseTransform
	currentTool = nullptr;
	latestVirtualExtruderPosition = moveStartVirtualExtruderPosition = 0.0;
	virtualFanSpeed = 0.0;
	speedFactor = 1.0;
	newToolNumber = -1;
	previousToolNumber = -1;

#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif
	ClearMove();
	updateUserPositionGb = nullptr;
	restartMoveFractionDone = 0.0;
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	fileOffsetToPrint = 0;
#endif
	for (RestorePoint& rp : restorePoints)
	{
		rp.Init();
	}
	InitObjectCancellation();
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

void MovementState::Diagnostics(MessageType mtype, unsigned int moveSystemNumber) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "Segments left Q%u: %u\n", moveSystemNumber, segmentsLeft);
	codeQueue->Diagnostics(mtype, moveSystemNumber);
}

void MovementState::SavePosition(unsigned int restorePointNumber, size_t numAxes, float p_feedRate, FilePosition p_filePos) noexcept
{
	RestorePoint& rp = restorePoints[restorePointNumber];
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		rp.moveCoords[axis] = currentUserPosition[axis];
	}

	rp.feedRate = p_feedRate;
	rp.virtualExtruderPosition = latestVirtualExtruderPosition;
	rp.filePos = p_filePos;
	rp.toolNumber = GetCurrentToolNumber();
	rp.fanSpeed = virtualFanSpeed;

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = laserPwmOrIoBits;
#endif
}

// Select the specified tool, putting the existing current tool into standby
void MovementState::SelectTool(int toolNumber, bool simulating) noexcept
{
	ReadLockedPointer<Tool> const newTool = Tool::GetLockedTool(toolNumber);
	if (!simulating && currentTool != nullptr && currentTool != newTool.Ptr())
	{
		currentTool->Standby();
	}
	currentTool = newTool.Ptr();					// must do this first so that Activate() will always work
	if (!simulating && newTool.IsNotNull())
	{
		newTool->Activate();
	}
}

// Get a locked pointer to the current tool, or null if there is no current tool
ReadLockedPointer<Tool> MovementState::GetLockedCurrentTool() const noexcept
{
	ReadLocker lock(Tool::toolListLock);
	return ReadLockedPointer<Tool>(lock, currentTool);
}

// Get the current tool, or failing that the default tool. May return nullptr if there are no tools.
// Called when a M104 or M109 command doesn't specify a tool number.
ReadLockedPointer<Tool> MovementState::GetLockedCurrentOrDefaultTool() const noexcept
{
	ReadLocker lock(Tool::toolListLock);
	// If a tool is already selected, use that one, else use the lowest-numbered tool which is the one at the start of the tool list
	return ReadLockedPointer<Tool>(lock, (currentTool != nullptr) ? currentTool : Tool::GetToolList());
}

// Return the current tool number, or -1 if no tool selected
int MovementState::GetCurrentToolNumber() const noexcept
{
	return (currentTool == nullptr) ? -1 : currentTool->Number();
}

// Set the previous tool number. Inline because it is only called from one place.
void MovementState::SetPreviousToolNumber() noexcept
{
	previousToolNumber = (currentTool != nullptr) ? currentTool->Number() : -1;
}

// Get the current axes used as the specified axis
AxesBitmap MovementState::GetCurrentAxisMapping(unsigned int axis) const noexcept
{
	return Tool::GetAxisMapping(currentTool, axis);
}

// Get the current axes used as X axis
AxesBitmap MovementState::GetCurrentXAxes() const noexcept
{
	return Tool::GetXAxes(currentTool);
}

// Get the current axes used as Y axis
AxesBitmap MovementState::GetCurrentYAxes() const noexcept
{
	return Tool::GetYAxes(currentTool);
}

// Get an axis offset of the current tool
float MovementState::GetCurrentToolOffset(size_t axis) const noexcept
{
	return (currentTool == nullptr) ? 0.0 : currentTool->GetOffset(axis);
}

// We are currently printing, but we must now stop because the current object is cancelled
void MovementState::StopPrinting(GCodeBuffer& gb) noexcept
{
	currentObjectCancelled = true;
	virtualToolNumber = GetCurrentToolNumber();
}

// We are currently not printing because the current object was cancelled, but now we need to print again
void MovementState::ResumePrinting(GCodeBuffer& gb) noexcept
{
	currentObjectCancelled = false;
	printingJustResumed = true;
	reprap.GetGCodes().SavePosition(gb, ResumeObjectRestorePointNumber);	// save the position we should be at for the start of the next move
	if (GetCurrentToolNumber() != virtualToolNumber)						// if the wrong tool is loaded
	{
		reprap.GetGCodes().StartToolChange(gb, virtualToolNumber, DefaultToolChangeParam);
	}
}

void MovementState::InitObjectCancellation() noexcept
{
	currentObjectNumber = -1;
	currentObjectCancelled = printingJustResumed = false;
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
