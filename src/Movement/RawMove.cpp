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

// Set up some default values in the move buffer for special moves, e.g. for Z probing and firmware retraction. The movement tool is set to null.
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
#if 0	// we don't use this yet
	cosXyAngle = 1.0;
#endif
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

int32_t MovementState::lastKnownEndpoints[MaxAxesPlusExtruders];			// the last stored  position of the logical drives
int32_t MovementState::endpointsAtSimulationStart[MaxAxesPlusExtruders];	// what the endpoints were before we started simulating

#if SUPPORT_ASYNC_MOVES
LogicalDrivesBitmap MovementState::allLogicalDrivesOwned;					// logical drives that are owned by any movement system
#endif

/*static*/ void MovementState::SetInitialMotorPositions(const float initialPosition[MaxAxesPlusExtruders]) noexcept
{
#if SUPPORT_ASYNC_MOVES
	allLogicalDrivesOwned.Clear();
#endif
	memseti32(lastKnownEndpoints, 0, ARRAY_SIZE(lastKnownEndpoints));
	Move& move = reprap.GetMove();
	move.CartesianToMotorSteps(initialPosition, lastKnownEndpoints, false);
	move.SetMotorPositions(allLogicalDrives, lastKnownEndpoints);
}

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
	axesAndExtrudersOwned.Clear();
	logicalDrivesOwned.Clear();
	ownedAxisLetters.Clear();
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

void MovementState::SetInitialMachineCoordinates(const float initialPosition[MaxAxesPlusExtruders]) noexcept
{
	memcpyf(coords, initialPosition, MaxAxesPlusExtruders);
	reprap.GetMove().SetLastEndpoints(msNumber, allLogicalDrives, lastKnownEndpoints);
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

// Get a single coordinate for reporting e.g.in the OM
// Return the current machine axis and extruder coordinates. They are needed only to service status requests from DWC, PanelDue, M114.
// Transforming the machine motor coordinates to Cartesian coordinates is quite expensive, and a status request or object model request will call this for each axis.
// So we cache the latest coordinates and only update them if it is some time since we last did, or if we have just waited for movement to stop.
// Interrupts are assumed enabled on entry
// Note, this no longer applies inverse mesh bed compensation or axis skew compensation to the returned machine coordinates, so they are the compensated coordinates!
float MovementState::LiveMachineCoordinate(unsigned int axisOrExtruder) const noexcept
{
	if (forceLiveCoordinatesUpdate || millis() - latestLiveCoordinatesFetchedAt > MoveTiming::MachineCoordinateUpdateInterval)
	{
		reprap.GetMove().UpdateLiveMachineCoordinates(latestLiveCoordinates, currentTool);
		forceLiveCoordinatesUpdate = false;
		latestLiveCoordinatesFetchedAt = millis();
	}
	return latestLiveCoordinates[axisOrExtruder];
}

void MovementState::Diagnostics(MessageType mtype) const noexcept
{
	reprap.GetPlatform().MessageF(mtype, "Segments left %u"
#if SUPPORT_ASYNC_MOVES
											", axes/extruders owned 0x%08" PRIx32 ", drives owned 0x%08" PRIx32
#endif
											"\n",
											segmentsLeft
#if SUPPORT_ASYNC_MOVES
											, axesAndExtrudersOwned.GetRaw(), logicalDrivesOwned.GetRaw()
#endif
									);
	codeQueue->Diagnostics(mtype);
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

void MovementState::SetNewPositionOfOwnedAxes(float ncoords[MaxAxes]) noexcept
{
	int32_t endpoints[MaxAxesPlusExtruders];
	memcpyi32(endpoints, lastKnownEndpoints, ARRAY_SIZE(endpoints));
	Move& move = reprap.GetMove();
	move.CartesianToMotorSteps(ncoords, endpoints, false);
	move.SetLastEndpoints(msNumber, logicalDrivesOwned, endpoints);
	move.SetMotorPositions(logicalDrivesOwned, endpoints);
	move.UpdateStartCoordinates(msNumber, ncoords);
}

// Fetch lastKnownEndpoints from the motors for our owned drives and update the endpoints in our DDA ring
void MovementState::UpdateOwnedDriveEndpointsFromMotors() noexcept
{
	Move& move = reprap.GetMove();
	logicalDrivesOwned.Iterate([&move](unsigned int drive, unsigned int count) noexcept
								{
									lastKnownEndpoints[drive] = move.GetLiveMotorPosition(drive);
								}
							  );
	move.SetLastEndpoints(msNumber, logicalDrivesOwned, lastKnownEndpoints);
}

// Update lastKnownEndpoints for our owned drives. Called when pausing.
void MovementState::UpdateOwnedDriveLastEndpoints(const int32_t endpoints[MaxAxes]) noexcept
{
	logicalDrivesOwned.Iterate([this, endpoints](unsigned int drive, unsigned int count) noexcept
								{
									lastKnownEndpoints[drive] = endpoints[drive];
								}
							  );
}

// Fetch the positions of currently owned drives and save them to lastKnownEndpoints
void MovementState::SaveOwnDriveCoordinates() const noexcept
{
	Move& move = reprap.GetMove();
	move.GetLastEndpoints(msNumber, logicalDrivesOwned, lastKnownEndpoints);
}

void MovementState::ChangeEndpointsAfterHoming(LogicalDrivesBitmap drives, const int32_t endpoints[MaxAxes]) noexcept
{
	reprap.GetMove().ChangeEndpointsAfterHoming(GetNumber(), drives, endpoints);
	drives.Iterate([endpoints](unsigned int drive, unsigned int count) noexcept { lastKnownEndpoints[drive] = endpoints[drive]; });
}

void MovementState::ChangeSingleEndpointAfterHoming(size_t drive, int32_t ep) noexcept
{
	reprap.GetMove().ChangeSingleEndpointAfterHoming(GetNumber(), drive, ep);
	lastKnownEndpoints[drive] = ep;
}

#if SUPPORT_ASYNC_MOVES

// This is how we handle axis and extruder allocation and release:
//
// 0. Axes are subject to the following mapping: user axis (i.e. as identified by an axis letter in the GCode) -> machine axis numbers (by applying tool-specific axis mapping) -> logical drive numbers (by applying kinematics).
//    Extruder numbers are mapped like this: extruder number -> logical drive number.
//
// 1. We keep track of which machine axes and extruders a MovementSystem owns ('machine' means after tool-specific axis mapping).
// 2. We keep track of which logical drives a Movement System owns (the logical drives that the owned machine axes and extruders use).
// 3. We keep a cache of user axis letters for which we definitely own the corresponding machine axes.
//    This is to make it faster to check whether we own a machine axis and its drivers at the start of processing a G0/1/2/3 command.
//
// 4. We must clear the cache of user axis letters any time we release axes/extruders, or change the current tool (because the tool axis mapping may change).
//
// 5. To allocate physical axes that we don't (or may not) already own, we check that they are not already owned other than by this MovementSystem.
//    Then we ask the kinematics which logical drives control those axes.
//    Then we check that none of those logical drives is already owned.
//	  Then we can allocate those machine axes and logical drives.
//    Note, we could check for other axes that use any of those logical drives too, and then recurse until we have the closure of all affected logical drivers.
//    However, any such additional axes can't be already owned because the corresponding drivers are not already owned; so we don't need to do that.
//
// 6. When allocating a machine axis, we must update our user position to reflect the position of that axis as it was left by whatever MovementSystem previously used it.
//    We fetch the machine axis positions from lastKnownAxisPositions and transform it to user coordinates.
//
// 7. When allocating a logical driver that is used for axis movement we must update the initial endpoints that we use when calculating the amount of movement in the next move.
//    We fetch these from lastKnownEndpoints.
//
// 8. When releasing a machine axis we must store its position in lastKnownAxisPositions.
//
// 9. When releasing a logical drive we must store its final endpoint in lastKnownEndpoints.

// Release all owned axes and extruders
void MovementState::ReleaseAllOwnedAxesAndExtruders() noexcept
{
	ReleaseAxesAndExtruders(axesAndExtrudersOwned);
}

// Release some of the axes that we own. We must also clear the cache of owned axis letters.
// Called when we release a tool and when we release all axes and extruders.
void MovementState::ReleaseAxesAndExtruders(AxesBitmap axesToRelease) noexcept
{
	SaveOwnDriveCoordinates();										// save the positions of the drives we own before we release them, otherwise we will get the wrong positions when we allocate them again
	Move& move = reprap.GetMove();
	const LogicalDrivesBitmap drivesStillOwned = move.GetKinematics().GetAllDrivesUsed(axesAndExtrudersOwned);
	const LogicalDrivesBitmap drivesToRelease = logicalDrivesOwned & ~drivesStillOwned;
	logicalDrivesOwned = drivesStillOwned;

	// We must not release any axes that are affected by the logical drives that we still own
	const AxesBitmap additionalAxesOwned = move.GetKinematics().GetAffectedAxes(drivesStillOwned, reprap.GetGCodes().GetVisibleAxes());
	axesAndExtrudersOwned = (axesAndExtrudersOwned & ~axesToRelease) | additionalAxesOwned;	// clear the axes/extruders we have been released
	allLogicalDrivesOwned.ClearBits(drivesToRelease);
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

// Allocate additional axes, returning the bitmap of any logical drives we can't allocate
LogicalDrivesBitmap MovementState::AllocateAxes(AxesBitmap axes, ParameterLettersBitmap axisLetters) noexcept
{
	// Sometimes we ask to allocate axes that we already own, e.g. when doing firmware retraction. Optimise this case.
	const AxesBitmap axesNeeded = axes & ~axesAndExtrudersOwned;
	if (axesNeeded.IsEmpty())
	{
		ownedAxisLetters |= axisLetters;
		return axesNeeded;											// return empty bitmap
	}

	// We don't need to check whether the axes needed are free because if any are already owned, the corresponding logical drives will be owned too
	Move& move = reprap.GetMove();
	const LogicalDrivesBitmap drivesNeeded = move.GetKinematics().GetAllDrivesUsed(axesNeeded) & ~logicalDrivesOwned;
	const LogicalDrivesBitmap unavailableDrives = drivesNeeded & allLogicalDrivesOwned;
	if (unavailableDrives.IsEmpty())
	{
		// Update the cache of axis letters that we own
		ownedAxisLetters |= axisLetters;

		// Update the set of logical drives that we own
		move.GetLastEndpoints(msNumber, logicalDrivesOwned, lastKnownEndpoints);
		allLogicalDrivesOwned |= drivesNeeded;
		logicalDrivesOwned |= drivesNeeded;

		// Update the set of axes and extruders that we own
		const AxesBitmap axesMask = AxesBitmap::MakeLowestNBits(MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders());
		const AxesBitmap extrudersMask = ~axesMask;
		const AxesBitmap axesAffected = move.GetKinematics().GetAffectedAxes(drivesNeeded, reprap.GetGCodes().GetVisibleAxes());
		axesAndExtrudersOwned |= axesAffected | (axesNeeded & extrudersMask);

		// If we allocated any logical drives, get the last endpoints for those drives and update our Cartesian coordinates
		if (!drivesNeeded.IsEmpty())
		{
			move.SetLastEndpoints(msNumber, drivesNeeded, lastKnownEndpoints);
			move.MotorStepsToCartesian(lastKnownEndpoints, reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), coords);
			move.InverseAxisAndBedTransform(coords, currentTool);
		}
	}
	return unavailableDrives;
}

// Try to allocate logical drives directly, returning the bitmap of any logical drives we can't allocate
LogicalDrivesBitmap MovementState::AllocateDrives(LogicalDrivesBitmap drivesNeeded) noexcept
{
	drivesNeeded &= ~logicalDrivesOwned;
	const LogicalDrivesBitmap unavailableDrives = drivesNeeded & allLogicalDrivesOwned;
	if (!drivesNeeded.IsEmpty())
	{
		if (unavailableDrives.IsEmpty())
		{
			Move& move = reprap.GetMove();
			move.GetLastEndpoints(msNumber, logicalDrivesOwned, lastKnownEndpoints);
			const AxesBitmap axesAffected = move.GetKinematics().GetAffectedAxes(drivesNeeded, reprap.GetGCodes().GetVisibleAxes());
			allLogicalDrivesOwned |= drivesNeeded;
			logicalDrivesOwned |= drivesNeeded;
			axesAndExtrudersOwned |= axesAffected;
			move.SetLastEndpoints(msNumber, drivesNeeded, lastKnownEndpoints);
			move.MotorStepsToCartesian(lastKnownEndpoints, reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), coords);
			move.InverseAxisAndBedTransform(coords, currentTool);
		}
	}
	return unavailableDrives;
}

void MovementState::UpdateCoordinatesFromLastKnownEndpoints() noexcept
{
	float machinePosition[MaxAxes];
	Move& move = reprap.GetMove();
	move.MotorStepsToCartesian(lastKnownEndpoints, reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), machinePosition);
	memcpyf(coords, machinePosition, reprap.GetGCodes().GetTotalAxes());
	move.InverseAxisAndBedTransform(coords, currentTool);
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

// Adjust the motor endpoints without moving the motors. Called after auto-calibrating a linear delta or rotary delta machine.
// There must be no pending movement when calling this!
void MovementState::AdjustMotorPositions(const float adjustment[], size_t numMotors) noexcept
{
	SaveOwnDriveCoordinates();
	Move& move = reprap.GetMove();
	for (size_t i  = 0; i < numMotors; ++i)
	{
		lastKnownEndpoints[i] += lrintf(adjustment[i] * move.DriveStepsPerMm(i));
	}
	const LogicalDrivesBitmap drivesToAdjust = LogicalDrivesBitmap::MakeLowestNBits(numMotors);
	move.SetLastEndpoints(GetNumber(), drivesToAdjust, lastKnownEndpoints);
	move.SetMotorPositions(drivesToAdjust, lastKnownEndpoints);
}

/*static*/ void MovementState::SaveEndpointsBeforeSimulating() noexcept
{
	memcpyi32(endpointsAtSimulationStart, lastKnownEndpoints, ARRAY_SIZE(endpointsAtSimulationStart));
}

/*static*/ void MovementState::RestoreEndpointsAfterSimulating() noexcept
{
	memcpyi32(lastKnownEndpoints, endpointsAtSimulationStart, ARRAY_SIZE(lastKnownEndpoints));
}

/*static*/ void MovementState::DebugPrintLastKnownEndpoints(const char *_ecv_array str) noexcept
{
	debugPrintf(str);
	for (int32_t ep : lastKnownEndpoints)
	{
		debugPrintf(" %" PRIi32, ep);
	}
	debugPrintf("\n");
}

// End
