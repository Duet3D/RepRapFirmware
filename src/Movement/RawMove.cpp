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
void MovementState::SetDefaults(size_t firstDriveToZero) noexcept
{
	moveType = 0;
	isCoordinated = false;
	applyM220M221 = false;
	usingStandardFeedrate = false;
	usePressureAdvance = false;
	doingArcMove = false;
	checkEndstops = false;
	reduceAcceleration = false;
	hasPositiveExtrusion = false;
	inverseTimeMode = false;
	linearAxesMentioned = false;
	rotationalAxesMentioned = false;
#if SUPPORT_SCANNING_PROBES
	scanningProbeMove = false;
#endif
#if SUPPORT_LASER
	laserPixelData.Clear();
#endif
	filePos = noFilePosition;
	movementTool = nullptr;
	moveFractionToSkip = 0.0;
	cosXyAngle = 1.0;
	for (size_t drive = firstDriveToZero; drive < MaxAxesPlusExtruders; ++drive)
	{
		coords[drive] = 0.0;					// clear extrusion
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

	movementTool = currentTool = nullptr;
	latestVirtualExtruderPosition = moveStartVirtualExtruderPosition = 0.0;
	virtualFanSpeed = 0.0;
	speedFactor = 1.0;
	newToolNumber = -1;
	previousToolNumber = -1;

	ResetLaser();

	updateUserPositionGb = nullptr;
	restartMoveFractionDone = 0.0;
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	fileOffsetToPrint = 0;
# if SUPPORT_ASYNC_MOVES
	fileOffsetToSkipTo = 0;
# endif
#endif
	for (RestorePoint& rp : restorePoints)
	{
		rp.Init();
	}
	InitObjectCancellation();
}

// Reset the laser parameters (also resets iobits because that is shared with laser)
void MovementState::ResetLaser() noexcept
{
#if SUPPORT_LASER
	laserPixelData.Clear();
#endif
#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif
}

void MovementState::ChangeExtrusionFactor(unsigned int extruder, float multiplier) noexcept
{
	if (segmentsLeft != 0 && applyM220M221)
	{
		coords[ExtruderToLogicalDrive(extruder)] *= multiplier;		// last move not gone, so update it
	}
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
#if SUPPORT_LASER
	rp.laserPixelData = laserPixelData;
#endif
}

// Restore current values from the pause restore point
void MovementState::ResumeAfterPause() noexcept
{
	moveStartVirtualExtruderPosition = latestVirtualExtruderPosition = GetPauseRestorePoint().virtualExtruderPosition;	// reset the extruder position in case we are receiving absolute extruder moves
	moveFractionToSkip = GetPauseRestorePoint().proportionDone;
	restartInitialUserC0 = GetPauseRestorePoint().initialUserC0;
	restartInitialUserC1 = GetPauseRestorePoint().initialUserC1;
#if SUPPORT_ASYNC_MOVES
	fileOffsetToSkipTo = GetPauseRestorePoint().filePos;
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

// Get the current axes used as Y axis
AxesBitmap MovementState::GetCurrentZAxes() const noexcept
{
	return Tool::GetZAxes(currentTool);
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

// Return the current machine axis and extruder coordinates. They are needed only to service status requests from DWC, PanelDue, M114.
// Transforming the machine motor coordinates to Cartesian coordinates is quite expensive, and a status request or object model request will call this for each axis.
// So we cache the latest coordinates and only update them if it is some time since we last did, or if we have just waited for movement to stop.
// Interrupts are assumed enabled on entry
float MovementState::LiveCoordinate(unsigned int axisOrExtruder) const noexcept
{
	if (forceLiveCoordinatesUpdate || millis() - latestLiveCoordinatesFetchedAt > 200)
	{
		reprap.GetMove().GetLiveCoordinates(msNumber, currentTool, latestLiveCoordinates);
		latestLiveCoordinatesFetchedAt = millis();
	}
	return latestLiveCoordinates[axisOrExtruder];
}

#if SUPPORT_ASYNC_MOVES

// Release all owned axes and extruders
void MovementState::ReleaseAllOwnedAxesAndExtruders() noexcept
{
	ReleaseAxesAndExtruders(axesAndExtrudersOwned);
}

// Release some of the axes that we own. We must also clear the cache of owned axis letters.
void MovementState::ReleaseAxesAndExtruders(AxesBitmap axesToRelease) noexcept
{
	SaveOwnAxisCoordinates();										// save the positions of the axes we own before we release them, otherwise we will get the wrong positions when we allocate them again
	axesAndExtrudersOwned &= ~axesToRelease;						// clear the axes/extruders we have been asked to release
	axesAndExtrudersMoved.ClearBits(axesToRelease);					// remove them from the own axes/extruders
	ownedAxisLetters.Clear();										// clear the cache of owned axis letters
}

// Release all axes and extruders we own except those used by our current tool
void MovementState::ReleaseNonToolAxesAndExtruders() noexcept
{
	AxesBitmap axesToRelease = GetAxesAndExtrudersOwned();
	if (currentTool != nullptr)
	{
		axesToRelease &= ~currentTool->GetXYAxesAndExtruders();
	}
	ReleaseAxesAndExtruders(axesToRelease);
}

// Allocate additional axes
AxesBitmap MovementState::AllocateAxes(AxesBitmap axes, ParameterLettersBitmap axisLetters) noexcept
{
	// Sometimes we ask to allocate aces that we already own, e.g. when doing firmware retraction. Optimise this case.
	const AxesBitmap axesNeeded = axes & ~axesAndExtrudersOwned;
	if (axesNeeded.IsEmpty())
	{
		ownedAxisLetters |= axisLetters;
		return axesNeeded;											// return empty bitmap
	}

	SaveOwnAxisCoordinates();										// we must do this before we allocate new axes to ourselves
	const AxesBitmap unAvailable = axesNeeded & axesAndExtrudersMoved;
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
	const size_t totalAxes = reprap.GetGCodes().GetTotalAxes();
	if (!memeqf(coords, lastKnownMachinePositions, totalAxes))
	{
#if 0	//DEBUG
		for (size_t i = 0; i < totalAxes; ++i)
		{
			if (coords[i] != lastKnownMachinePositions[i])
			{
				debugPrintf("Coord %u changed from %.4f to %.4f in ms %u\n", i, (double)coords[i], (double)lastKnownMachinePositions[i], GetMsNumber());
			}
		}
#endif	//END DEBUGB
		memcpyf(coords, lastKnownMachinePositions, totalAxes);
		move.SetRawPosition(coords, msNumber);
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
