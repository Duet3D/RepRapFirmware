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
#include <Movement/Move.h>
#include <Movement/Kinematics/Kinematics.h>

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
	inverseTimeMode = false;
	linearAxesMentioned = false;
	rotationalAxesMentioned = false;
	filePos = noFilePosition;
	movementTool = nullptr;
	cosXyAngle = 1.0;
	for (size_t drive = firstDriveToZero; drive < MaxAxesPlusExtruders; ++drive)
	{
		coords[drive] = 0.0;			// clear extrusion
	}
}

#if SUPPORT_ASYNC_MOVES

AxesBitmap MovementState::axesAndExtrudersMoved;						// axes and extruders that are owned by any movement system
float MovementState::lastKnownMachinePositions[MaxAxesPlusExtruders];	// the last stored machine position of the axes

/*static*/ void MovementState::GlobalInit(size_t numVisibleAxes) noexcept
{
	axesAndExtrudersMoved.Clear();
	reprap.GetMove().GetKinematics().GetAssumedInitialPosition(numVisibleAxes, lastKnownMachinePositions);
	for (size_t i = numVisibleAxes; i < MaxAxesPlusExtruders; ++i)
	{
		lastKnownMachinePositions[i] = 0.0;
	}
}

#endif

float MovementState::GetProportionDone() const noexcept
{
	return (float)(totalSegments - segmentsLeft)/(float)totalSegments;
}

// Initialise this MovementState. If SUPPORT_ASYNC_MOVES is set then must call MovementState::GlobalInit before calling this to initialise lastKnownMachinePositions.
void MovementState::Init(MovementSystemNumber p_msNumber) noexcept
{
	msNumber = p_msNumber;
	ClearMove();
	filePos = noFilePosition;
	codeQueue->Clear();
	currentCoordinateSystem = 0;
	pausedInMacro = false;

#if SUPPORT_ASYNC_MOVES
	memcpyf(coords, lastKnownMachinePositions, MaxAxesPlusExtruders);
	axesAndExtrudersOwned.Clear();
	ownedAxisLetters.Clear();
#else
	for (float& f : coords)
	{
		f = 0.0;									// clear out all axis and extruder coordinates
	}
#endif

	maxPrintingAcceleration = ConvertAcceleration(DefaultPrintingAcceleration);
	maxTravelAcceleration = ConvertAcceleration(DefaultTravelAcceleration);

	currentZHop = 0.0;								// clear this before calling ToolOffsetInverseTransform
	movementTool = currentTool = nullptr;
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

void MovementState::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "Q%u segments left %u"
#if SUPPORT_ASYNC_MOVES
											", axes/extruders owned 0x%07x"
#endif
											"\n",
													GetMsNumber(),
													segmentsLeft
#if SUPPORT_ASYNC_MOVES
													, (unsigned int)axesAndExtrudersOwned.GetRaw()
#endif
									);
	codeQueue->Diagnostics(mtype, GetMsNumber());
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
	return ReadLockedPointer<Tool>(Tool::toolListLock, currentTool);
}

// Get the current tool, or failing that the default tool. May return nullptr if there are no tools.
// Called when a M104 or M109 command doesn't specify a tool number.
ReadLockedPointer<Tool> MovementState::GetLockedCurrentOrDefaultTool() const noexcept
{
	// If a tool is already selected, use that one, else use the lowest-numbered tool which is the one at the start of the tool list
	return ReadLockedPointer<Tool>(Tool::toolListLock, (currentTool != nullptr) ? currentTool : Tool::GetToolList());
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
}

// We are currently not printing because the current object was cancelled, but now we need to print again
void MovementState::ResumePrinting(GCodeBuffer& gb) noexcept
{
	currentObjectCancelled = false;
	printingJustResumed = true;
	reprap.GetGCodes().SavePosition(gb, ResumeObjectRestorePointNumber);	// save the position we should be at for the start of the next move
	if (GetCurrentToolNumber() != newToolNumber)							// if the wrong tool is loaded
	{
		reprap.GetGCodes().StartToolChange(gb, *this, DefaultToolChangeParam);
	}
}

void MovementState::InitObjectCancellation() noexcept
{
	currentObjectNumber = -1;
	currentObjectCancelled = printingJustResumed = false;
}

#if SUPPORT_ASYNC_MOVES

// When releasing axes we must also release the corresponding axis letters, because they serve as a cache
void MovementState::ReleaseOwnedAxesAndExtruders() noexcept
{
	axesAndExtrudersMoved.ClearBits(axesAndExtrudersOwned);
	axesAndExtrudersOwned.Clear();
	ownedAxisLetters.Clear();
}

void MovementState::ReleaseAxesAndExtruders(AxesBitmap axesToRelease) noexcept
{
	axesAndExtrudersOwned &= ~axesToRelease;						// clear the axes/extruders we have been asked to release
	axesAndExtrudersMoved.ClearBits(axesToRelease);					// remove them from the own axes/extruders
	ownedAxisLetters.Clear();										// clear the cache of owned axis letters
}

AxesBitmap MovementState::AllocateAxes(AxesBitmap axes, ParameterLettersBitmap axisLetters) noexcept
{
	SaveOwnAxisCoordinates();										// we must do this before we allocate new axis to ourselves
	const AxesBitmap unAvailable = axes & ~axesAndExtrudersOwned & axesAndExtrudersMoved;
	if (unAvailable.IsEmpty())
	{
		axesAndExtrudersMoved |= axes;
		axesAndExtrudersOwned |= axes;
		ownedAxisLetters |= axisLetters;
	}
	return unAvailable;
}

// Fetch and save the coordinates of axes we own to lastKnownMachinePositions, also copy them to our own coordinates in case we just did a homing move
void MovementState::SaveOwnAxisCoordinates() noexcept
{
	Move& move = reprap.GetMove();
	move.GetPartialMachinePosition(lastKnownMachinePositions, msNumber, axesAndExtrudersOwned);

	// Only update our own position if something has changed, to avoid frequent inverse and forward transforms
	if (!memeqf(coords, lastKnownMachinePositions, MaxAxesPlusExtruders))
	{
		memcpyf(coords, lastKnownMachinePositions, MaxAxesPlusExtruders);
		move.SetRawPosition(lastKnownMachinePositions, msNumber);
		move.InverseAxisAndBedTransform(coords, currentTool);
	}
}

// Update changed coordinates of some owned axes - called after G92
void MovementState::OwnedAxisCoordinatesUpdated(AxesBitmap axesIncluded) noexcept
{
	axesIncluded.Iterate([this](unsigned int bitNumber, unsigned int count)->void
							{ lastKnownMachinePositions[bitNumber] = coords[bitNumber]; }
						);
}

// Update the machine coordinate of an axis we own - called after Z probing
void MovementState::OwnedAxisCoordinateUpdated(size_t axis) noexcept
{
	lastKnownMachinePositions[axis] = coords[axis];
}

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
