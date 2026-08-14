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
#include <PrintMonitor/PrintMonitor.h>

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(MovementState, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(MovementState, _condition, __VA_ARGS__)
#define OBJECT_MODEL_ARRAY_COUNT(_value)		OBJECT_MODEL_ARRAY_COUNT_BODY(MovementState, _value)
#define OBJECT_MODEL_ARRAY_VALUE(...)			OBJECT_MODEL_ARRAY_VALUE_BODY(MovementState, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry MovementState::objectModelArrayTable[] =
{
	// 0. User position
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(reprap.GetGCodes().GetTotalAxes()),
		OBJECT_MODEL_ARRAY_VALUE(self->raw.coords[context.GetLastIndex()], 3)
	},

	// 1. Restore points
	{
		nullptr,
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(NumVisibleRestorePoints),
		OBJECT_MODEL_ARRAY_VALUE(&self->restorePoints[context.GetLastIndex()])
	},

#if SUPPORT_COORDINATE_ROTATION
	// 2. Rotation centre coordinates
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(2),
		OBJECT_MODEL_ARRAY_VALUE(self->g68Centre[context.GetLastIndex()], 3)
	},
#endif
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(MovementState)

constexpr ObjectModelTableEntry MovementState::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. motionSystems[] members
	{ "currentMove",			OBJECT_MODEL_FUNC(self, 1),																	ObjectModelEntryFlags::liveNotPanelDue },
	{ "currentObject",			OBJECT_MODEL_FUNC_IF(reprap.GetPrintMonitor().IsPrinting(), (int32_t)self->currentObjectNumber),										ObjectModelEntryFlags::none },
	{ "currentTool",			OBJECT_MODEL_FUNC((int32_t)self->GetCurrentToolNumber()),									ObjectModelEntryFlags::live },
	{ "nextTool",				OBJECT_MODEL_FUNC((int32_t)self->newToolNumber),											ObjectModelEntryFlags::none },
	{ "previousTool",			OBJECT_MODEL_FUNC((int32_t)self->previousToolNumber),										ObjectModelEntryFlags::none },
	{ "printingAcceleration",	OBJECT_MODEL_FUNC(InverseConvertAcceleration(self->raw.maxPrintingAcceleration), 1),		ObjectModelEntryFlags::none },
	{ "restorePoints",			OBJECT_MODEL_FUNC_ARRAY(1),																	ObjectModelEntryFlags::none },
#if SUPPORT_COORDINATE_ROTATION
	{ "rotation",				OBJECT_MODEL_FUNC(self, 2),																	ObjectModelEntryFlags::liveNotPanelDue },
#endif
	{ "speedFactor",			OBJECT_MODEL_FUNC(self->speedFactor, 2),													ObjectModelEntryFlags::none },
	{ "travelAcceleration",		OBJECT_MODEL_FUNC(InverseConvertAcceleration(self->raw.maxTravelAcceleration), 1),			ObjectModelEntryFlags::none },
	{ "userPosition",			OBJECT_MODEL_FUNC_ARRAY(0),																	ObjectModelEntryFlags::live },
	{ "virtualEPos",			OBJECT_MODEL_FUNC(self->latestVirtualExtruderPosition, 5),									ObjectModelEntryFlags::live },
	{ "workplaceNumber",		OBJECT_MODEL_FUNC((int32_t)self->currentCoordinateSystem),									ObjectModelEntryFlags::none },

	// 1. currentMove members
	{ "acceleration",			OBJECT_MODEL_FUNC(reprap.GetMove().GetAccelerationMmPerSecSquared(self->GetNumber()), 1),	ObjectModelEntryFlags::liveNotPanelDue },
	{ "deceleration",			OBJECT_MODEL_FUNC(reprap.GetMove().GetDecelerationMmPerSecSquared(self->GetNumber()), 1),	ObjectModelEntryFlags::liveNotPanelDue },
	{ "distance",				OBJECT_MODEL_FUNC(reprap.GetMove().GetCurrentMoveDistance(self->GetNumber()), 2),			ObjectModelEntryFlags::liveNotPanelDue },
	{ "duration",				OBJECT_MODEL_FUNC(reprap.GetMove().GetCurrentMoveDuration(self->GetNumber()), 2),			ObjectModelEntryFlags::liveNotPanelDue },
	{ "extrusionRate",			OBJECT_MODEL_FUNC(reprap.GetMove().GetTotalExtrusionRate(self->GetNumber()), 2),			ObjectModelEntryFlags::liveNotPanelDue },
//# if SUPPORT_LASER
#if 0		// currently the laser support is global, not per motion system
	{ "laserPwm",				OBJECT_MODEL_FUNC_IF_NOSELF(reprap.GetGCodes().GetMachineType() == MachineType::laser,
															reprap.GetPlatform().GetLaserPwm(), 2),							ObjectModelEntryFlags::liveNotPanelDue },
# endif
	{ "requestedSpeed",			OBJECT_MODEL_FUNC(reprap.GetMove().GetRequestedSpeedMmPerSec(self->GetNumber()), 1),		ObjectModelEntryFlags::liveNotPanelDue },
	{ "topSpeed",				OBJECT_MODEL_FUNC(reprap.GetMove().GetTopSpeedMmPerSec(self->GetNumber()), 1),				ObjectModelEntryFlags::liveNotPanelDue },

#if SUPPORT_COORDINATE_ROTATION
	// 2. motionSystems[].rotation members
	{ "angle",					OBJECT_MODEL_FUNC(self->g68Angle, 2),														ObjectModelEntryFlags::liveNotPanelDue },
	{ "centre",					OBJECT_MODEL_FUNC_ARRAY(2),																	ObjectModelEntryFlags::liveNotPanelDue },
#endif
};

constexpr uint8_t MovementState::objectModelTableDescriptor[] =
{
	2 + SUPPORT_COORDINATE_ROTATION,
	12 + SUPPORT_COORDINATE_ROTATION,
	7,
#if SUPPORT_COORDINATE_ROTATION
	2
#endif
};

DEFINE_GET_OBJECT_MODEL_TABLE(MovementState)

// Set up some default values in the move buffer for special moves, e.g. for Z probing and firmware retraction. The movement tool is set to null.
void MovementState::SetDefaults(size_t firstDriveToZero) noexcept
{
	raw.moveType = 0;
	raw.isCoordinated = false;
	raw.applyM220M221 = false;
	raw.usingStandardFeedrate = false;
	raw.usePressureAdvance = false;
	doingArcMove = false;
	raw.checkEndstops = false;
	raw.reduceAcceleration = false;
	raw.hasPositiveExtrusion = false;
	raw.inverseTimeMode = false;
	raw.linearAxesMentioned = false;
	raw.rotationalAxesMentioned = false;
#if SUPPORT_SCANNING_PROBES
	raw.scanningProbeMove = false;
#endif
#if SUPPORT_LASER
	laserPixelData.Clear();
#endif
	raw.filePos = noFilePosition;
	raw.gCommandNumber = -1;
	raw.movementTool = nullptr;
	moveFractionToSkip = 0.0;
#if 0	// we don't use this yet
	cosXyAngle = 1.0;
#endif
	for (size_t drive = firstDriveToZero; drive < MaxAxesPlusExtruders; ++drive)
	{
		raw.coords[drive] = 0.0;				// clear extrusion
	}
}

void MovementState::ClearMove() noexcept
{
	TaskCriticalSectionLocker lock;				// make sure that other tasks sees a consistent memory state

	segmentsLeft = 0;
	segMoveState = SegmentedMoveState::inactive;
	doingArcMove = false;
	raw.checkEndstops = false;
	raw.reduceAcceleration = false;
	raw.moveType = 0;
	raw.applyM220M221 = false;
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
	move.SetMotorPositions(RawMove::allLogicalDrives, lastKnownEndpoints, true);
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
	raw.filePos = noFilePosition;
	raw.gCommandNumber = -1;
	codeQueue->Clear();
	currentCoordinateSystem = 0;

#if SUPPORT_COORDINATE_ROTATION
	g68Angle = g68Centre[0] = g68Centre[1] = 0.0;				// no coordinate rotation
#endif

	pausedInMacro = false;
	positionMayBeInaccurate = false;

#if SUPPORT_ASYNC_MOVES
	raw.axesAndExtrudersOwned.Clear();
	raw.logicalDrivesOwned.Clear();
	ownedAxisLetters.Clear();
#endif

	raw.maxPrintingAcceleration = ConvertAcceleration(DefaultPrintingAcceleration);
	raw.maxTravelAcceleration = ConvertAcceleration(DefaultTravelAcceleration);

	raw.movementTool = currentTool = nullptr;
	latestVirtualExtruderPosition = raw.moveStartVirtualExtruderPosition = 0.0;
	virtualFanSpeed = 0.0;
	speedFactor = 1.0;
	newToolNumber = -1;
	previousToolNumber = -1;

	ResetLaser();

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
	memcpyf(raw.coords, initialPosition, MaxAxesPlusExtruders);
	reprap.GetMove().SetLastEndpoints(msNumber, RawMove::allLogicalDrives, lastKnownEndpoints);
}

// Reset the laser parameters (also resets iobits because that is shared with laser)
void MovementState::ResetLaser() noexcept
{
#if SUPPORT_LASER
	laserPixelData.Clear();
#endif
#if SUPPORT_LASER || SUPPORT_IOBITS
	raw.laserPwmOrIoBits.Clear();
#endif
}

void MovementState::ChangeExtrusionFactor(unsigned int extruder, float multiplier) noexcept
{
	if (segmentsLeft != 0 && raw.applyM220M221)
	{
		raw.coords[ExtruderToLogicalDrive(extruder)] *= multiplier;		// last move not gone, so update it
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
	float coords[MaxAxesPlusExtruders];
	reprap.GetMove().UpdateLiveMachineCoordinates(coords, currentTool);
	return coords[axisOrExtruder];
}

void MovementState::Diagnostics(const StringRef& reply) const noexcept
{
	reply.lcatf("Segments left %u"
#if SUPPORT_ASYNC_MOVES
				", axes/extruders owned 0x%08" PRIx32 ", drives owned 0x%08" PRIx32
#endif
				,
				segmentsLeft
#if SUPPORT_ASYNC_MOVES
				, raw.axesAndExtrudersOwned.GetRaw(), raw.logicalDrivesOwned.GetRaw()
#endif
									);
	codeQueue->Diagnostics(reply);
}

void MovementState::SavePosition(unsigned int restorePointNumber, size_t numAxes, float p_feedRate, FilePosition p_filePos) noexcept
{
	RestorePoint& rp = restorePoints[restorePointNumber];
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		rp.moveCoords[axis] = currentUserPosition[axis];
	}

	rp.originalFeedRate = p_feedRate;
	rp.virtualExtruderPosition = latestVirtualExtruderPosition;
	rp.filePos = p_filePos;
	// Restore points saved this way (synchronous pauses via M226/M600/M601, tool changes, G60, simulation) are triggered
	// by a command that has already overwritten the modal motion command, so mark it unknown rather than saving a stale value
	rp.gCommandNumber = -1;
	rp.toolNumber = GetCurrentToolNumber();
	rp.fanSpeed = virtualFanSpeed;

#if SUPPORT_ASYNC_MOVES
	rp.axesAndExtrudersOwned = raw.axesAndExtrudersOwned;
#endif
#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = raw.laserPwmOrIoBits;
#endif
#if SUPPORT_LASER
	rp.laserPixelData = laserPixelData;
#endif
}

// Restore current values from the pause restore point
void MovementState::ResumeAfterPause() noexcept
{
	raw.moveStartVirtualExtruderPosition = latestVirtualExtruderPosition = GetPauseRestorePoint().virtualExtruderPosition;	// reset the extruder position in case we are receiving absolute extruder moves
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

// Set the motor positions to where the machine coordinates say they should be
void MovementState::SetNewPositionOfOwnedAxes() noexcept
{
	// First apply any skew compensation
	float ncoords[MaxAxes];
	memcpyf(ncoords, raw.coords, ARRAY_SIZE(ncoords));
	Move& move = reprap.GetMove();
	move.AxisAndBedTransform(ncoords, currentTool, true);

	// Now convert those coordinates to motor positions
	int32_t endpoints[MaxAxesPlusExtruders];
	memcpyi32(endpoints, lastKnownEndpoints, ARRAY_SIZE(endpoints));
	move.CartesianToMotorSteps(ncoords, endpoints, false);

	// Update the start coordinates in the DDA ring
	move.UpdateStartCoordinates(msNumber, ncoords);

	// Update the motor endpoints in the DDA ring, in the DMs, and in lastKnownEndpoints
	ChangeEndpointsAfterHoming(raw.logicalDrivesOwned, endpoints);
}

// Fetch lastKnownEndpoints from the motors for our owned drives and update the endpoints in our DDA ring
void MovementState::UpdateOwnedDriveEndpointsFromMotors() noexcept
{
	Move& move = reprap.GetMove();
	raw.logicalDrivesOwned.Iterate([&move](unsigned int drive, unsigned int count) noexcept
								{
									lastKnownEndpoints[drive] = move.GetLiveMotorPosition(drive) - move.GetCurrentBacklashSteps(drive);
								}
							  );
	move.SetLastEndpoints(msNumber, raw.logicalDrivesOwned, lastKnownEndpoints);
}

// Update lastKnownEndpoints for our owned drives. Called when pausing.
void MovementState::UpdateOwnedDriveLastEndpoints(const int32_t endpoints[MaxAxes]) noexcept
{
	raw.logicalDrivesOwned.Iterate([this, endpoints](unsigned int drive, unsigned int count) noexcept
								{
									lastKnownEndpoints[drive] = endpoints[drive];
								}
							  );
}

// Fetch the positions of currently owned drives and save them to lastKnownEndpoints
void MovementState::SaveOwnDriveCoordinates() const noexcept
{
	Move& move = reprap.GetMove();
	move.GetLastEndpoints(msNumber, raw.logicalDrivesOwned, lastKnownEndpoints);
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
	ReleaseAxesAndExtruders(raw.axesAndExtrudersOwned);
}

// Release some of the axes/extruders that we own. We must also clear the cache of owned axis letters.
// Called when we release a tool and when we release all axes and extruders.
void MovementState::ReleaseAxesAndExtruders(AxesBitmap axesToRelease) noexcept
{
	//debugPrintf("Release axes 0x%08" PRIx32, axesToRelease.GetRaw());
	SaveOwnDriveCoordinates();										// save the positions of the drives we own before we release them, otherwise we will get the wrong positions when we allocate them again
	LogicalDrivesBitmap axesAndExtrudersToRetain = raw.axesAndExtrudersOwned & ~axesToRelease;
	LogicalDrivesBitmap drivesToRetain;
	FormClosure(axesAndExtrudersToRetain, drivesToRetain);
	const LogicalDrivesBitmap drivesToRelease = raw.logicalDrivesOwned & ~drivesToRetain;
	raw.axesAndExtrudersOwned = axesAndExtrudersToRetain;
	raw.logicalDrivesOwned = drivesToRetain;
	allLogicalDrivesOwned.ClearBits(drivesToRelease);
	ownedAxisLetters.Clear();										// clear the cache of owned axis letters
	//debugPrintf(" still own 0x%08" PRIx32 " drives 0x%08" PRIx32 "\n", axesAndExtrudersOwned.GetRaw(), logicalDrivesOwned.GetRaw());
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
	AxesBitmap axesNeeded = axes & ~raw.axesAndExtrudersOwned;
	if (axesNeeded.IsEmpty())
	{
		ownedAxisLetters |= axisLetters;
		return axesNeeded;											// return empty bitmap
	}

	//debugPrintf("Allocate axes 0x%08" PRIx32, axes.GetRaw());
	// We don't need to check whether the axes needed are free because if any are already owned by the other MS, the corresponding logical drives will be owned by that MS too
	LogicalDrivesBitmap drivesNeeded;
	FormClosure(axesNeeded, drivesNeeded);
	//debugPrintf(" closure (0x%08" PRIx32 ", 0x%08" PRIx32, axesNeeded.GetRaw(), drivesNeeded.GetRaw());
	drivesNeeded &= ~raw.logicalDrivesOwned;

	const LogicalDrivesBitmap unavailableDrives = drivesNeeded & allLogicalDrivesOwned;
	if (unavailableDrives.IsEmpty())
	{
		// Update the cache of axis letters that we own
		ownedAxisLetters |= axisLetters;

		// Update the set of logical drives that we own
		Move& move = reprap.GetMove();
		move.GetLastEndpoints(msNumber, raw.logicalDrivesOwned, lastKnownEndpoints);
		allLogicalDrivesOwned |= drivesNeeded;
		raw.logicalDrivesOwned |= drivesNeeded;
		raw.axesAndExtrudersOwned |= axesNeeded;
		//debugPrintf(" now own 0x%08" PRIx32 " drives 0x%08" PRIx32 "\n", axesAndExtrudersOwned.GetRaw(), logicalDrivesOwned.GetRaw());

		// If we allocated any logical drives, get the last endpoints for those drives and update our Cartesian coordinates
		if (!drivesNeeded.IsEmpty())
		{
			move.SetLastEndpoints(msNumber, drivesNeeded, lastKnownEndpoints);
			move.MotorStepsToCartesian(lastKnownEndpoints, reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), raw.coords);
			move.InverseAxisAndBedTransform(raw.coords, currentTool);
		}
	}
	return unavailableDrives;
}

// Try to allocate logical drives directly, returning the bitmap of any logical drives we can't allocate
LogicalDrivesBitmap MovementState::AllocateDrives(LogicalDrivesBitmap drivesNeeded) noexcept
{
	AxesBitmap affectedAxes;
	FormClosure(affectedAxes, drivesNeeded);

	drivesNeeded &= ~raw.logicalDrivesOwned;
	const LogicalDrivesBitmap unavailableDrives = drivesNeeded & allLogicalDrivesOwned;
	if (!drivesNeeded.IsEmpty() && unavailableDrives.IsEmpty())
	{
		Move& move = reprap.GetMove();
		move.GetLastEndpoints(msNumber, raw.logicalDrivesOwned, lastKnownEndpoints);
		allLogicalDrivesOwned |= drivesNeeded;
		raw.logicalDrivesOwned |= drivesNeeded;
		raw.axesAndExtrudersOwned |= affectedAxes;
		move.SetLastEndpoints(msNumber, drivesNeeded, lastKnownEndpoints);
		move.MotorStepsToCartesian(lastKnownEndpoints, reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), raw.coords);
		move.InverseAxisAndBedTransform(raw.coords, currentTool);
	}
	return unavailableDrives;
}

// Given some axes/extruders and/or drives that we want to allocate, expand them to the closure of all connected axes and drives
void MovementState::FormClosure(AxesBitmap &axes, LogicalDrivesBitmap &drives) noexcept
{
	const GCodes& gcodes = reprap.GetGCodes();
	const Kinematics& kin = reprap.GetMove().GetKinematics();
	const AxesBitmap axesMask = AxesBitmap::MakeLowestNBits(MaxAxesPlusExtruders - gcodes.GetNumExtruders());
	const size_t numVisibleAxes = gcodes.GetVisibleAxes();
	drives |= axes & ~axesMask;										// allocate drives for the requested extruders
	while (true)
	{
		const AxesBitmap oldAxes = axes;
		const LogicalDrivesBitmap oldDrives = drives;
		drives |= kin.GetAllDrivesUsed(axes & axesMask);			// allocate the drives that control the requested axes
		axes |= kin.GetAffectedAxes(drives, numVisibleAxes);		// allocate the axes that are affected by the driver we now want
		if (axes == oldAxes && drives == oldDrives) { break; }
	}
}

void MovementState::UpdateCoordinatesFromLastKnownEndpoints() noexcept
{
	float machinePosition[MaxAxes];
	Move& move = reprap.GetMove();
	move.MotorStepsToCartesian(lastKnownEndpoints, reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), machinePosition);
	memcpyf(raw.coords, machinePosition, reprap.GetGCodes().GetTotalAxes());
	move.InverseAxisAndBedTransform(raw.coords, currentTool);
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
	move.SetMotorPositions(drivesToAdjust, lastKnownEndpoints, false);
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
