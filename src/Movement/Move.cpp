/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David

 A note on bed levelling:

 As at version 1.21 we support two types of bed compensation:
 1. The old 3, 4 and 5-point compensation using a RandomProbePointSet. We will probably discontinue this soon.
 2. Mesh bed levelling

 There is an interaction between using G30 to home Z or set a precise Z=0 height just before a print, and bed compensation.
 Consider the following sequence:
 1. Home Z, using either G30 or an endstop.
 2. Run G29 to generate a height map. If the Z=0 point has drifted off, the height map may have a Z offset.
 3. Use G30 to get an accurate Z=0 point. We want to keep the shape of the height map, but get rid of the offset.
 4. Run G29 to generate a height map. This should generate a height map with on offset at the point we just probed.
 5. Cancel bed compensation. The height at the point we just probed should be zero.

 So as well as maintaining a height map, we maintain a Z offset from it. The procedure is:
 1. Whenever bed compensation is not being used, the Z offset should be zero.
 2. Whenever we run G29 to probe the bed, we have a choice:
 (a) accept that the map may have a height offset; and set the Z offset to zero. This is what we do currently.
 (b) normalise the height map to zero, adjust the Z=0 origin, and set the Z offset to zero.
 3. When we run G30 to reset the Z=0 height, and we have a height map loaded, we adjust the Z offset to be the negative of the
    height map indication of that point.
 4. If we now cancel the height map, we also clear the Z offset, and the height at the point we probed remains correct.
 5. If we now run G29 to probe again, the height map should have near zero offset at the point we probed, if there has been no drift.

 Before we introduced the Z offset, at step 4 we would have a potentially large Z error as if the G30 hadn't been run,
 and at step 5 the new height map would have an offset again.

 */

#include "Move.h"
#include "StepTimer.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Tools/Tool.h"
#include "Endstops/ZProbe.h"
#include <TaskPriorities.h>

#if SUPPORT_IOBITS
# include <PortControl.h>
#endif

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanMotion.h>
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Move, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(Move, __VA_ARGS__)

static constexpr ObjectModelArrayDescriptor axesArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetVisibleAxes(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&reprap.GetPlatform(), 3); }
};

static constexpr ObjectModelArrayDescriptor extrudersArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetNumExtruders(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&reprap.GetPlatform(), 4); }
};

constexpr ObjectModelTableEntry Move::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Move members
	{ "axes",					OBJECT_MODEL_FUNC_NOSELF(&axesArrayDescriptor), 										ObjectModelEntryFlags::live },
	{ "calibration",			OBJECT_MODEL_FUNC(self, 4),																ObjectModelEntryFlags::none },
	{ "compensation",			OBJECT_MODEL_FUNC(self, 7),																ObjectModelEntryFlags::none },
	{ "currentMove",			OBJECT_MODEL_FUNC(self, 3),																ObjectModelEntryFlags::live },
	{ "daa",					OBJECT_MODEL_FUNC(self, 1),																ObjectModelEntryFlags::none },
	{ "extruders",				OBJECT_MODEL_FUNC_NOSELF(&extrudersArrayDescriptor),									ObjectModelEntryFlags::live },
	{ "idle",					OBJECT_MODEL_FUNC(self, 2),																ObjectModelEntryFlags::none },
	{ "kinematics",				OBJECT_MODEL_FUNC(self->kinematics),													ObjectModelEntryFlags::none },
	{ "printingAcceleration",	OBJECT_MODEL_FUNC(self->maxPrintingAcceleration, 1),									ObjectModelEntryFlags::none },
	{ "speedFactor",			OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetSpeedFactor(), 2),						ObjectModelEntryFlags::none },
	{ "travelAcceleration",		OBJECT_MODEL_FUNC(self->maxTravelAcceleration, 1),										ObjectModelEntryFlags::none },
	{ "workspaceNumber",		OBJECT_MODEL_FUNC_NOSELF((int32_t)reprap.GetGCodes().GetWorkplaceCoordinateSystemNumber()),	ObjectModelEntryFlags::none },

	// 1. Move.Daa members
	{ "enabled", 				OBJECT_MODEL_FUNC(self->drcEnabled), 													ObjectModelEntryFlags::none },
	{ "minimumAcceleration",	OBJECT_MODEL_FUNC(self->drcMinimumAcceleration, 1),										ObjectModelEntryFlags::none },
	{ "period",					OBJECT_MODEL_FUNC(self->drcPeriod, 1), 													ObjectModelEntryFlags::none },

	// 2. Move.Idle members
	{ "factor",					OBJECT_MODEL_FUNC_NOSELF(reprap.GetPlatform().GetIdleCurrentFactor(), 1),				ObjectModelEntryFlags::none },
	{ "timeout",				OBJECT_MODEL_FUNC(0.001f * (float)self->idleTimeout, 1),								ObjectModelEntryFlags::none },

	// 3. move.currentMove members
	{ "acceleration",			OBJECT_MODEL_FUNC(self->GetAcceleration(), 1),											ObjectModelEntryFlags::live },
	{ "deceleration",			OBJECT_MODEL_FUNC(self->GetDeceleration(), 1),											ObjectModelEntryFlags::live },
# if SUPPORT_LASER
	{ "laserPwm",				OBJECT_MODEL_FUNC_IF_NOSELF(reprap.GetGCodes().GetMachineType() == MachineType::laser,
															reprap.GetPlatform().GetLaserPwm(), 2),						ObjectModelEntryFlags::live },
# endif
	{ "requestedSpeed",			OBJECT_MODEL_FUNC(self->GetRequestedSpeed(), 1),										ObjectModelEntryFlags::live },
	{ "topSpeed",				OBJECT_MODEL_FUNC(self->GetTopSpeed(), 1),												ObjectModelEntryFlags::live },

	// 4. move.calibration members
	{ "final",					OBJECT_MODEL_FUNC(self, 6),																ObjectModelEntryFlags::none },
	{ "initial",				OBJECT_MODEL_FUNC(self, 5),																ObjectModelEntryFlags::none },
	{ "numFactors",				OBJECT_MODEL_FUNC((int32_t)self->numCalibratedFactors),									ObjectModelEntryFlags::none },

	// 5. move.calibration.initialDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->initialCalibrationDeviation.GetDeviationFromMean(), 3),			ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->initialCalibrationDeviation.GetMean(), 3),						ObjectModelEntryFlags::none },

	// 6. move.calibration.finalDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->latestCalibrationDeviation.GetDeviationFromMean(), 3),			ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->latestCalibrationDeviation.GetMean(), 3),						ObjectModelEntryFlags::none },

	// 7. move.compensation members
	{ "fadeHeight",				OBJECT_MODEL_FUNC((self->useTaper) ? self->taperHeight : std::numeric_limits<float>::quiet_NaN(), 1),	ObjectModelEntryFlags::none },
#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	{ "file",					OBJECT_MODEL_FUNC_IF(self->usingMesh, self->heightMap.GetFileName()),					ObjectModelEntryFlags::none },
#endif
	{ "meshDeviation",			OBJECT_MODEL_FUNC_IF(self->usingMesh, self, 8),											ObjectModelEntryFlags::none },
	{ "probeGrid",				OBJECT_MODEL_FUNC_NOSELF((const GridDefinition *)&reprap.GetGCodes().GetDefaultGrid()),	ObjectModelEntryFlags::none },
	{ "type",					OBJECT_MODEL_FUNC(self->GetCompensationTypeString()),									ObjectModelEntryFlags::none },

	// 8. move.compensation.meshDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->latestMeshDeviation.GetDeviationFromMean(), 3),					ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->latestMeshDeviation.GetMean(), 3),								ObjectModelEntryFlags::none },
};

constexpr uint8_t Move::objectModelTableDescriptor[] = { 9, 12, 3, 2, 4 + SUPPORT_LASER, 3, 2, 2, 4 + HAS_MASS_STORAGE, 2 };

DEFINE_GET_OBJECT_MODEL_TABLE(Move)

#endif

Move::Move() noexcept
	: active(false),
	  drcEnabled(false),											// disable dynamic ringing cancellation
	  maxPrintingAcceleration(10000.0), maxTravelAcceleration(10000.0),
	  drcPeriod(0.025),												// 40Hz
	  drcMinimumAcceleration(10.0),
	  jerkPolicy(0),
	  numCalibratedFactors(0)
{
	// Kinematics must be set up here because GCodes::Init asks the kinematics for the assumed initial position
	kinematics = Kinematics::Create(KinematicsType::cartesian);		// default to Cartesian
	mainDDARing.Init1(DdaRingLength);
#if SUPPORT_ASYNC_MOVES
	auxDDARing.Init1(AuxDdaRingLength);
#endif
	DriveMovement::InitialAllocate(NumDms);
}

void Move::Init() noexcept
{
	mainDDARing.Init2();

#if SUPPORT_ASYNC_MOVES
	auxDDARing.Init2();
	auxMoveAvailable = false;
	auxMoveLocked = false;
#endif

	// Clear the transforms
	SetIdentityTransform();
	tanXY = tanYZ = tanXZ = 0.0;

	usingMesh = useTaper = executeAllMoves = false;
	zShift = 0.0;

	idleTimeout = DefaultIdleTimeout;
	moveState = MoveState::idle;
	lastStateChangeTime = millis();
	idleCount = 0;

	simulationMode = 0;
	longestGcodeWaitInterval = 0;
	bedLevellingMoveAvailable = false;

	active = true;
}

void Move::Exit() noexcept
{
	StepTimer::DisableTimerInterrupt();
	mainDDARing.Exit();
#if SUPPORT_LASER || SUPPORT_IOBITS
	delete laserTask;
	laserTask = nullptr;
#endif
	active = false;												// don't accept any more moves
}

void Move::Spin() noexcept
{
	if (!active)
	{
		RawMove nextMove;
		(void) reprap.GetGCodes().ReadMove(nextMove);			// throw away any move that GCodes tries to pass us
		return;
	}

	if (idleCount < 1000)
	{
		++idleCount;
	}

	// Recycle the DDAs for completed moves, checking for DDA errors to print if Move debug is enabled
	mainDDARing.RecycleDDAs();
#if SUPPORT_ASYNC_MOVES
	auxDDARing.RecycleDDAs();
#endif

	// See if we can add another move to the ring
	bool canAddMove = (
#if SUPPORT_ROLAND
						  !reprap.GetRoland()->Active() &&
#endif
						  mainDDARing.CanAddMove()
					  );
	if (canAddMove)
	{
		// OK to add another move. First check if a special move is available.
		if (bedLevellingMoveAvailable)
		{
			if (simulationMode < 2)
			{
				if (mainDDARing.AddSpecialMove(reprap.GetPlatform().MaxFeedrate(Z_AXIS), specialMoveCoords))
				{
					if (moveState == MoveState::idle || moveState == MoveState::timing)
					{
						// We were previously idle, so we have a state change
						moveState = MoveState::collecting;
						const uint32_t now = millis();
						const uint32_t timeWaiting = now - lastStateChangeTime;
						if (timeWaiting > longestGcodeWaitInterval)
						{
							longestGcodeWaitInterval = timeWaiting;
						}
						lastStateChangeTime = now;
					}
				}
			}
			bedLevellingMoveAvailable = false;
		}
		else
		{
			// If there's a G Code move available, add it to the DDA ring for processing.
			RawMove nextMove;
			if (reprap.GetGCodes().ReadMove(nextMove))		// if we have a new move
			{
				if (simulationMode < 2)		// in simulation mode 2 and higher, we don't process incoming moves beyond this point
				{
					if (nextMove.moveType == 0)
					{
						AxisAndBedTransform(nextMove.coords, nextMove.tool, true);
					}

					if (mainDDARing.AddStandardMove(nextMove, !IsRawMotorMove(nextMove.moveType)))
					{
						idleCount = 0;
						if (moveState == MoveState::idle || moveState == MoveState::timing)
						{
							moveState = MoveState::collecting;
							const uint32_t now = millis();
							const uint32_t timeWaiting = now - lastStateChangeTime;
							if (timeWaiting > longestGcodeWaitInterval)
							{
								longestGcodeWaitInterval = timeWaiting;
							}
							lastStateChangeTime = now;
						}
					}
				}
			}
		}
	}

	mainDDARing.Spin(simulationMode, executeAllMoves || idleCount > 10);	// let the DDA ring process moves. Better to have a few moves in the queue so that we can do lookahead, hence the test on idleCount.

#if SUPPORT_ASYNC_MOVES
	if (auxMoveAvailable && auxDDARing.CanAddMove())
	{
		if (auxDDARing.AddAsyncMove(auxMove))
		{
			moveState = MoveState::collecting;
		}
		auxMoveAvailable = false;
	}
	auxDDARing.Spin(simulationMode, true);				// let the DDA ring process moves
#endif

	// Reduce motor current to standby if the rings have been idle for long enough
	if (   mainDDARing.IsIdle()
#if SUPPORT_ASYNC_MOVES
		&& auxDDARing.IsIdle()
#endif
	   )
	{
		if (moveState == MoveState::executing && !reprap.GetGCodes().IsPaused())
		{
			lastStateChangeTime = millis();				// record when we first noticed that the machine was idle
			moveState = MoveState::timing;
		}
		else if (moveState == MoveState::timing && millis() - lastStateChangeTime >= idleTimeout)
		{
			reprap.GetPlatform().SetDriversIdle();		// put all drives in idle hold
			moveState = MoveState::idle;
		}
	}
	else
	{
		moveState = MoveState::executing;
	}
}

// Return the number of currently used probe points
unsigned int Move::GetNumProbePoints() const noexcept
{
	return probePoints.GetNumBedCompensationPoints();
}

// Return the number of actually probed probe points
unsigned int Move::GetNumProbedProbePoints() const noexcept
{
	return probePoints.NumberOfProbePoints();
}

// Try to push some babystepping through the lookahead queue, returning the amount pushed
float Move::PushBabyStepping(size_t axis, float amount) noexcept
{
	return mainDDARing.PushBabyStepping(axis, amount);
}

// Change the kinematics to the specified type if it isn't already
// If it is already correct leave its parameters alone.
// This violates our rule on no dynamic memory allocation after the initialisation phase,
// however this function is normally called only when M665, M667 and M669 commands in config.g are processed.
bool Move::SetKinematics(KinematicsType k) noexcept
{
	if (kinematics->GetKinematicsType() != k)
	{
		Kinematics * const nk = Kinematics::Create(k);
		if (nk == nullptr)
		{
			return false;
		}
		delete kinematics;
		kinematics = nk;
		reprap.MoveUpdated();
	}
	return true;
}

// Return true if this is a raw motor move
bool Move::IsRawMotorMove(uint8_t moveType) const noexcept
{
	return moveType == 2 || ((moveType == 1 || moveType == 3) && kinematics->GetHomingMode() != HomingMode::homeCartesianAxes);
}

// Return true if the specified point is accessible to the Z probe
bool Move::IsAccessibleProbePoint(float x, float y) const noexcept
{
	const auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(reprap.GetGCodes().GetCurrentZProbeNumber());
	if (zp.IsNotNull())
	{
		x -= zp->GetXOffset();
		y -= zp->GetYOffset();
	}
	return kinematics->IsReachable(x, y, false);
}

// Pause the print as soon as we can, returning true if we are able to skip any moves and updating 'rp' to the first move we skipped.
bool Move::PausePrint(RestorePoint& rp) noexcept
{
	return mainDDARing.PauseMoves(rp);
}

#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT

// Pause the print immediately, returning true if we were able to skip or abort any moves and setting up to the move we aborted
bool Move::LowPowerOrStallPause(RestorePoint& rp) noexcept
{
	return mainDDARing.LowPowerOrStallPause(rp);
}

#endif

void Move::Diagnostics(MessageType mtype) noexcept
{
	Platform& p = reprap.GetPlatform();
	p.MessageF(mtype, "=== Move ===\nHiccups: %" PRIu32
#if SUPPORT_ASYNC_MOVES
						"(%" PRIu32 ")"
#endif
						", FreeDm: %d, MinFreeDm: %d, MaxWait: %" PRIu32 "ms\n",
						mainDDARing.GetClearNumHiccups(),
#if SUPPORT_ASYNC_MOVES
						auxDDARing.GetClearNumHiccups(),
#endif
						DriveMovement::NumFree(), DriveMovement::MinFree(), longestGcodeWaitInterval);
	longestGcodeWaitInterval = 0;
	DriveMovement::ResetMinFree();

#if defined(__ALLIGATOR__)
	// Motor Fault Diagnostic
	reprap.GetPlatform().MessageF(mtype, "Motor Fault status: %s\n", digitalRead(MotorFaultDetectPin) ? "none" : "FAULT detected!" );
#endif

	// Show the current probe position heights and type of bed compensation in use
	String<StringLength50> bedCompString;
	if (usingMesh)
	{
		bedCompString.copy("mesh");
	}
	else if (probePoints.GetNumBedCompensationPoints() != 0)
	{
		bedCompString.printf("%d point", probePoints.GetNumBedCompensationPoints());
	}
	else
	{
		bedCompString.copy("none");
	}
	p.MessageF(mtype, "Bed compensation in use: %s, comp offset %.3f\n", bedCompString.c_str(), (double)zShift);

	// Only print the probe point heights if we are using old-style compensation
	if (!usingMesh && probePoints.GetNumBedCompensationPoints() != 0)
	{
		// To keep the response short so that it doesn't get truncated when sending it via HTTP, we only show the first 5 bed probe points
		bedCompString.Clear();
		for (size_t i = 0; i < 5; ++i)
		{
			bedCompString.catf(" %.3f", (double)probePoints.GetZHeight(i));
		}
		p.MessageF(mtype, "Bed probe heights:%s\n", bedCompString.c_str());
	}

#if DDA_LOG_PROBE_CHANGES
	// Temporary code to print Z probe trigger positions
	p.Message(mtype, "Probe change coordinates:");
	char ch = ' ';
	for (size_t i = 0; i < DDA::numLoggedProbePositions; ++i)
	{
		float xyzPos[XYZ_AXES];
		MotorStepsToCartesian(DDA::loggedProbePositions + (XYZ_AXES * i), XYZ_AXES, XYZ_AXES, xyzPos);
		p.MessageF(mtype, "%c%.2f,%.2f", ch, xyzPos[X_AXIS], xyzPos[Y_AXIS]);
		ch = ',';
	}
	p.Message(mtype, "\n");
#endif

#if SUPPORT_ASYNC_MOVES
	mainDDARing.Diagnostics(mtype, "Main");
	auxDDARing.Diagnostics(mtype, "Aux");
#else
	mainDDARing.Diagnostics(mtype, "");
#endif
}

// Set the current position to be this
void Move::SetNewPosition(const float positionNow[MaxAxesPlusExtruders], bool doBedCompensation) noexcept
{
	float newPos[MaxAxesPlusExtruders];
	memcpy(newPos, positionNow, sizeof(newPos));			// copy to local storage because Transform modifies it
	AxisAndBedTransform(newPos, reprap.GetCurrentTool(), doBedCompensation);
	SetLiveCoordinates(newPos);
	mainDDARing.SetPositions(newPos);
}

// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered and DDA::SetPositions
void Move::EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const noexcept
{
	if (CartesianToMotorSteps(coords, ep, true))
	{
		for (size_t drive = reprap.GetGCodes().GetTotalAxes(); drive < numDrives; ++drive)
		{
			ep[drive] = MotorMovementToSteps(drive, coords[drive]);
		}
	}
}

// Convert distance to steps for a particular drive
int32_t Move::MotorMovementToSteps(size_t drive, float coord) noexcept
{
	return lrintf(coord * reprap.GetPlatform().DriveStepsPerUnit(drive));
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// This is computationally expensive on a delta or SCARA machine, so only call it when necessary, and never from the step ISR.
void Move::MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	kinematics->MotorStepsToCartesian(motorPos, reprap.GetPlatform().GetDriveStepsPerUnit(), numVisibleAxes, numTotalAxes, machinePos);
	if (reprap.Debug(moduleMove) && !inInterrupt())
	{
		debugPrintf("Forward transformed %" PRIi32 " %" PRIi32 " %" PRIi32 " to %.2f %.2f %.2f\n",
			motorPos[0], motorPos[1], motorPos[2], (double)machinePos[0], (double)machinePos[1], (double)machinePos[2]);
	}
}

// Convert Cartesian coordinates to motor steps, axes only, returning true if successful.
// Used to perform movement and G92 commands.
// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered, DDA::SetPositions and Move::EndPointToMachine
bool Move::CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const noexcept
{
	const bool b = kinematics->CartesianToMotorSteps(machinePos, reprap.GetPlatform().GetDriveStepsPerUnit(),
														reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), motorPos, isCoordinated);
	if (reprap.Debug(moduleMove) && !inInterrupt())
	{
		if (!b)
		{
			debugPrintf("Unable to transform");
			for (size_t i = 0; i < reprap.GetGCodes().GetVisibleAxes(); ++i)
			{
				debugPrintf(" %.2f", (double)machinePos[i]);
			}
			debugPrintf("\n");
		}
		else if (reprap.Debug(moduleDda))
		{
			debugPrintf("Transformed");
			for (size_t i = 0; i < reprap.GetGCodes().GetVisibleAxes(); ++i)
			{
				debugPrintf(" %.2f", (double)machinePos[i]);
			}
			debugPrintf(" to");
			for (size_t i = 0; i < reprap.GetGCodes().GetTotalAxes(); ++i)
			{
				debugPrintf(" %" PRIi32, motorPos[i]);
			}
			debugPrintf("\n");
		}
	}
	return b;
}

void Move::AxisAndBedTransform(float xyzPoint[MaxAxes], const Tool *tool, bool useBedCompensation) const noexcept
{
	AxisTransform(xyzPoint, tool);
	if (useBedCompensation)
	{
		BedTransform(xyzPoint, tool);
	}
}

void Move::InverseAxisAndBedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept
{
	InverseBedTransform(xyzPoint, tool);
	InverseAxisTransform(xyzPoint, tool);
}

// Do the Axis transform BEFORE the bed transform
void Move::AxisTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept
{
	// Identify the lowest Y axis
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	const AxesBitmap yAxes = Tool::GetYAxes(tool);
	const size_t lowestYAxis = yAxes.LowestSetBit();
	if (lowestYAxis < numVisibleAxes)
	{
		// Found a Y axis. Use this one when correcting the X coordinate.
		const AxesBitmap xAxes = Tool::GetXAxes(tool);
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (xAxes.IsBitSet(axis))
			{
				xyzPoint[axis] += tanXY*xyzPoint[lowestYAxis] + tanXZ*xyzPoint[Z_AXIS];
			}
			if (yAxes.IsBitSet(axis))
			{
				xyzPoint[axis] += tanYZ*xyzPoint[Z_AXIS];
			}
		}
	}
}

// Get the height error at a bed XY position
float Move::GetInterpolatedHeightError(float xCoord, float yCoord) const noexcept
{
	return (usingMesh) ? heightMap.GetInterpolatedHeightError(xCoord, yCoord) : probePoints.GetInterpolatedHeightError(xCoord, yCoord);
}

// Invert the Axis transform AFTER the bed transform
void Move::InverseAxisTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept
{
	// Identify the lowest Y axis
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	const AxesBitmap yAxes = Tool::GetYAxes(tool);
	const size_t lowestYAxis = yAxes.LowestSetBit();
	if (lowestYAxis < numVisibleAxes)
	{
		// Found a Y axis. Use this one when correcting the X coordinate.
		const AxesBitmap xAxes = Tool::GetXAxes(tool);
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (yAxes.IsBitSet(axis))
			{
				xyzPoint[axis] -= tanYZ*xyzPoint[Z_AXIS];
			}
			if (xAxes.IsBitSet(axis))
			{
				xyzPoint[axis] -= (tanXY*xyzPoint[lowestYAxis] + tanXZ*xyzPoint[Z_AXIS]);
			}
		}
	}
}

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept
{
	if (!useTaper || xyzPoint[Z_AXIS] < taperHeight)
	{
		float zCorrection = 0.0;
		const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
		const AxesBitmap xAxes = Tool::GetXAxes(tool);
		const AxesBitmap yAxes = Tool::GetYAxes(tool);
		unsigned int numCorrections = 0;

		// Transform the Z coordinate based on the average correction for each axis used as an X or Y axis.
		for (uint32_t xAxis = 0; xAxis < numAxes; ++xAxis)
		{
			if (xAxes.IsBitSet(xAxis))
			{
				const float xCoord = xyzPoint[xAxis] + Tool::GetOffset(tool, xAxis);
				for (uint32_t yAxis = 0; yAxis < numAxes; ++yAxis)
				{
					if (yAxes.IsBitSet(yAxis))
					{
						const float yCoord = xyzPoint[yAxis] + Tool::GetOffset(tool, yAxis);
						zCorrection += GetInterpolatedHeightError(xCoord, yCoord);
						++numCorrections;
					}
				}
			}
		}

		if (numCorrections > 1)
		{
			zCorrection /= numCorrections;			// take an average
		}

		zCorrection += zShift;
		xyzPoint[Z_AXIS] += (useTaper) ? (taperHeight - xyzPoint[Z_AXIS]) * recipTaperHeight * zCorrection : zCorrection;
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept
{
	float zCorrection = 0.0;
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	const AxesBitmap xAxes = Tool::GetXAxes(tool);
	const AxesBitmap yAxes = Tool::GetYAxes(tool);
	unsigned int numCorrections = 0;

	// Transform the Z coordinate based on the average correction for each axis used as an X or Y axis.
	for (uint32_t xAxis = 0; xAxis < numAxes; ++xAxis)
	{
		if (xAxes.IsBitSet(xAxis))
		{
			const float xCoord = xyzPoint[xAxis] + Tool::GetOffset(tool, xAxis);
			for (uint32_t yAxis = 0; yAxis < numAxes; ++yAxis)
			{
				if (yAxes.IsBitSet(yAxis))
				{
					const float yCoord = xyzPoint[yAxis] + Tool::GetOffset(tool, yAxis);
					zCorrection += GetInterpolatedHeightError(xCoord, yCoord);
					++numCorrections;
				}
			}
		}
	}

	if (numCorrections > 1)
	{
		zCorrection /= numCorrections;				// take an average
	}

	zCorrection += zShift;

	if (!useTaper || zCorrection >= taperHeight)	// need check on zCorrection to avoid possible divide by zero
	{
		xyzPoint[Z_AXIS] -= zCorrection;
	}
	else
	{
		const float zreq = (xyzPoint[Z_AXIS] - zCorrection)/(1.0 - (zCorrection * recipTaperHeight));
		if (zreq < taperHeight)
		{
			xyzPoint[Z_AXIS] = zreq;
		}
	}
}

// Normalise the bed transform to have zero height error at these bed coordinates
void Move::SetZeroHeightError(const float coords[MaxAxes]) noexcept
{
	float tempCoords[MaxAxes];
	memcpy(tempCoords, coords, sizeof(tempCoords));
	AxisTransform(tempCoords, nullptr);
	zShift = -GetInterpolatedHeightError(tempCoords[X_AXIS], tempCoords[Y_AXIS]);
}

void Move::SetIdentityTransform() noexcept
{
	probePoints.SetIdentity();
	heightMap.ClearGridHeights();
	heightMap.UseHeightMap(false);
	usingMesh = false;
	zShift = 0.0;
	reprap.MoveUpdated();
}

#if HAS_MASS_STORAGE

// Load the height map from file, returning true if an error occurred with the error reason appended to the buffer
bool Move::LoadHeightMapFromFile(FileStore *f, const char *fname, const StringRef& r) noexcept
{
	const bool ret = heightMap.LoadFromFile(f, fname, r);
	if (ret)
	{
		heightMap.ClearGridHeights();							// make sure we don't end up with a partial height map
	}
	else
	{
		zShift = 0.0;
	}
	reprap.MoveUpdated();
	return ret;
}

// Save the height map to a file returning true if an error occurred
bool Move::SaveHeightMapToFile(FileStore *f, const char *fname) noexcept
{
	return heightMap.SaveToFile(f, fname, zShift);
}

#endif

#if HAS_LINUX_INTERFACE

// Save the height map Z coordinates to an array
void Move::SaveHeightMapToArray(float *arr) const noexcept
{
	return heightMap.SaveToArray(arr, zShift);
}

#endif

void Move::SetTaperHeight(float h) noexcept
{
	useTaper = (h > 1.0);
	if (useTaper)
	{
		taperHeight = h;
		recipTaperHeight = 1.0/h;
	}
	reprap.MoveUpdated();
}

// Enable mesh bed compensation
bool Move::UseMesh(bool b) noexcept
{
	usingMesh = heightMap.UseHeightMap(b);
	reprap.MoveUpdated();
	return usingMesh;
}

float Move::AxisCompensation(unsigned int axis) const noexcept
{
	return (axis < ARRAY_SIZE(tangents)) ? tangents[axis] : 0.0;
}

void Move::SetAxisCompensation(unsigned int axis, float tangent) noexcept
{
	if (axis < ARRAY_SIZE(tangents))
	{
		tangents[axis] = tangent;
	}
}

// Calibrate or set the bed equation after probing, returning true if an error occurred
// sParam is the value of the S parameter in the G30 command that provoked this call.
// Caller already owns the GCode movement lock.
bool Move::FinishedBedProbing(int sParam, const StringRef& reply) noexcept
{
	bool error = false;
	const size_t numPoints = probePoints.NumberOfProbePoints();

	if (sParam < 0)
	{
		// A negative sParam just prints the probe heights
		probePoints.ReportProbeHeights(numPoints, reply);
	}
	else if (numPoints < (size_t)sParam)
	{
		reply.printf("Bed calibration : %d factor calibration requested but only %d points provided\n", sParam, numPoints);
		error = true;
	}
	else
	{
		if (reprap.Debug(moduleMove))
		{
			probePoints.DebugPrint(numPoints);
		}

		if (sParam == 0)
		{
			sParam = numPoints;
		}

		if (!probePoints.GoodProbePoints(numPoints))
		{
			reply.copy("Compensation or calibration cancelled due to probing errors");
			error = true;
		}
		else if (kinematics->SupportsAutoCalibration())
		{
			error = kinematics->DoAutoCalibration(sParam, probePoints, reply);
		}
		else
		{
			error = probePoints.SetProbedBedEquation(sParam, reply);
		}
	}

	// Clear out the Z heights so that we don't re-use old points.
	// This allows us to use different numbers of probe point on different occasions.
	probePoints.ClearProbeHeights();
	return error;
}

/*static*/ float Move::MotorStepsToMovement(size_t drive, int32_t endpoint) noexcept
{
	return ((float)(endpoint))/reprap.GetPlatform().DriveStepsPerUnit(drive);
}

// Return the transformed machine coordinates
void Move::GetCurrentUserPosition(float m[MaxAxes], uint8_t moveType, const Tool *tool) const noexcept
{
	GetCurrentMachinePosition(m, IsRawMotorMove(moveType));
	if (moveType == 0)
	{
		InverseAxisAndBedTransform(m, tool);
	}
}

// Get the accumulated extruder motor steps taken by an extruder since the last call. Used by the filament monitoring code.
// Returns the number of motor steps moves since the last call, and isPrinting is true unless we are currently executing an extruding but non-printing move
int32_t Move::GetAccumulatedExtrusion(size_t extruder, bool& isPrinting) noexcept
{
	if (extruder < reprap.GetGCodes().GetNumExtruders())
	{
		return mainDDARing.GetAccumulatedExtrusion(extruder, ExtruderToLogicalDrive(extruder), isPrinting);
	}

	isPrinting = false;
	return 0;
}

void Move::SetXYBedProbePoint(size_t index, float x, float y) noexcept
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform().Message(ErrorMessage, "Z probe point index out of range\n");
	}
	else
	{
		probePoints.SetXYBedProbePoint(index, x, y);
	}
}

void Move::SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError) noexcept
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform().Message(ErrorMessage, "Z probe point Z index out of range\n");
	}
	else
	{
		probePoints.SetZBedProbePoint(index, z, wasXyCorrected, wasError);
	}
}

// This returns the (X, Y) points to probe the bed at probe point count.  When probing, it returns false.
// If called after probing has ended it returns true, and the Z coordinate probed is also returned.
// If 'wantNozzlePosition is true then we return the nozzle position when the point is probed, else we return the probe point itself
float Move::GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const noexcept
{
	x = probePoints.GetXCoord(count);
	y = probePoints.GetYCoord(count);
	if (wantNozzlePosition)
	{
		const auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(reprap.GetGCodes().GetCurrentZProbeNumber());
		if (zp.IsNotNull())
		{
			x -= zp->GetXOffset();
			y -= zp->GetYOffset();
		}
	}
	return probePoints.GetZHeight(count);
}

// Enter or leave simulation mode
void Move::Simulate(uint8_t simMode) noexcept
{
	simulationMode = simMode;
	if (simMode != 0)
	{
		mainDDARing.ResetSimulationTime();
	}
}

// Adjust the leadscrews
// This is only ever called after bed probing, so we can assume that no such move is already pending.
void Move::AdjustLeadscrews(const floatc_t corrections[]) noexcept
{
	const size_t numZdrivers = reprap.GetPlatform().GetAxisDriversConfig(Z_AXIS).numDrivers;
	for (size_t i = 0; i < MaxDriversPerAxis; ++i)
	{
		specialMoveCoords[i] = (i < numZdrivers) ? (float)corrections[i] : 0.0;
	}
	bedLevellingMoveAvailable = true;
}

// Return the idle timeout in seconds
float Move::IdleTimeout() const noexcept
{
	return (float)idleTimeout * 0.001;
}

// Set the idle timeout in seconds
void Move::SetIdleTimeout(float timeout) noexcept
{
	idleTimeout = (uint32_t)lrintf(timeout * 1000.0);
	reprap.MoveUpdated();
}

#if HAS_MASS_STORAGE

// Write settings for resuming the print
// The GCodes module deals with the head position so all we need worry about is the bed compensation
// We don't handle random probe point bed compensation, and we assume that if a height map is being used it is the default one.
bool Move::WriteResumeSettings(FileStore *f) const noexcept
{
	return kinematics->WriteResumeSettings(f) && (!usingMesh || f->Write("G29 S1\n"));
}

#endif

// Process M204
GCodeResult Move::ConfigureAccelerations(GCodeBuffer&gb, const StringRef& reply) noexcept
{
	bool seen = false;
	if (gb.Seen('S'))
	{
		// For backwards compatibility with old versions of Marlin (e.g. for Cura and the Prusa fork of slic3r), set both accelerations
		seen = true;
		maxTravelAcceleration = maxPrintingAcceleration = gb.GetFValue();
	}
	if (gb.Seen('P'))
	{
		seen = true;
		maxPrintingAcceleration = gb.GetFValue();
	}
	if (gb.Seen('T'))
	{
		seen = true;
		maxTravelAcceleration = gb.GetFValue();
	}
	if (seen)
	{
		reprap.MoveUpdated();
	}
	else
	{
		reply.printf("Maximum printing acceleration %.1f, maximum travel acceleration %.1f", (double)maxPrintingAcceleration, (double)maxTravelAcceleration);
	}
	return GCodeResult::ok;
}

// Process M593
GCodeResult Move::ConfigureDynamicAcceleration(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	bool seen = false;
	if (gb.Seen('F'))
	{
		seen = true;
		const float f = gb.GetFValue();
		if (f >= 4.0 && f <= 10000.0)
		{
			drcPeriod = 1.0/f;
			drcEnabled = true;
		}
		else
		{
			drcEnabled = false;
		}
	}
	if (gb.Seen('L'))
	{
		seen = true;
		drcMinimumAcceleration = max<float>(gb.GetFValue(), 1.0);		// very low accelerations cause problems with the maths
	}

	if (seen)
	{
		reprap.MoveUpdated();
	}
	else
	{
		if (reprap.GetMove().IsDRCenabled())
		{
			reply.printf("Dynamic ringing cancellation at %.1fHz, min. acceleration %.1f", (double)(1.0/drcPeriod), (double)drcMinimumAcceleration);
		}
		else
		{
			reply.copy("Dynamic ringing cancellation is disabled");
		}
	}
	return GCodeResult::ok;
}

// Return the current live XYZ and extruder coordinates
// Interrupts are assumed enabled on entry
float Move::LiveCoordinate(unsigned int axisOrExtruder, const Tool *tool) noexcept
{
	if (mainDDARing.HaveLiveCoordinatesChanged())
	{
		mainDDARing.LiveCoordinates(latestLiveCoordinates);
		InverseAxisAndBedTransform(latestLiveCoordinates, tool);
	}
	return latestLiveCoordinates[axisOrExtruder];
}

void Move::SetLatestCalibrationDeviation(const Deviation& d, uint8_t numFactors) noexcept
{
	latestCalibrationDeviation = d;
	numCalibratedFactors = numFactors;
	reprap.MoveUpdated();
}

void Move::SetInitialCalibrationDeviation(const Deviation& d) noexcept
{
	initialCalibrationDeviation = d;
	reprap.MoveUpdated();
}

void Move::SetLatestMeshDeviation(const Deviation& d) noexcept
{
	latestMeshDeviation = d; reprap.MoveUpdated();
}

#if SUPPORT_OBJECT_MODEL

const char *Move::GetCompensationTypeString() const noexcept
{
	return (usingMesh) ? "mesh"
			: (probePoints.GetNumBedCompensationPoints() != 0) ? "legacy"
				: "none";
}

#endif

#if SUPPORT_LASER || SUPPORT_IOBITS

// Laser and IOBits support

Task<Move::LaserTaskStackWords> *Move::laserTask = nullptr;		// the task used to manage laser power or IOBits

extern "C" void LaserTaskStart(void * pvParameters) noexcept
{
	reprap.GetMove().LaserTaskRun();
}

// This is called when laser mode is selected or IOBits is enabled
void Move::CreateLaserTask() noexcept
{
	TaskCriticalSectionLocker lock;
	if (laserTask == nullptr)
	{
		laserTask = new Task<LaserTaskStackWords>;
		laserTask->Create(LaserTaskStart, "LASER", nullptr, TaskPriority::LaserPriority);
	}
}

// Wake up the laser task, if there is one (must check!). Call this at the start of a new move from standstill (not from an ISR)
void Move::WakeLaserTask() noexcept
{
	if (laserTask != nullptr)
	{
		laserTask->Give();
	}
}

// Wake up the laser task if there is one (must check!) from an ISR
void Move::WakeLaserTaskFromISR() noexcept
{
	if (laserTask != nullptr)
	{
		laserTask->GiveFromISR();
	}
}

void Move::LaserTaskRun() noexcept
{
	for (;;)
	{
		// Sleep until we are woken up by the start of a move
		(void)TaskBase::Take();

		if (reprap.GetGCodes().GetMachineType() == MachineType::laser)
		{
# if SUPPORT_LASER
			// Manage the laser power
			uint32_t ticks;
			while ((ticks = mainDDARing.ManageLaserPower()) != 0)
			{
				delay(ticks);
			}
# endif
		}
		else
		{
# if SUPPORT_IOBITS
			// Manage the IOBits
			uint32_t ticks;
			while ((ticks = reprap.GetPortControl().UpdatePorts()) != 0)
			{
				delay(ticks);
			}
# endif
		}
	}
}

#endif

#if SUPPORT_ASYNC_MOVES

// Get and lock the aux move buffer. If successful, return a pointer to the buffer.
// The caller must not attempt to lock the aux buffer more than once, and must call ReleaseAuxMove to release the buffer.
AsyncMove *Move::LockAuxMove() noexcept
{
	InterruptCriticalSectionLocker lock;
	if (!auxMoveLocked && !auxMoveAvailable)
	{
		auxMoveLocked = true;
		return &auxMove;
	}
	return nullptr;
}

// Release the aux move buffer and optionally signal that it contains a move
// The caller must have locked the buffer before calling this. If it calls with hasNewMove true, it must have populated the move buffer with the move details
void Move::ReleaseAuxMove(bool hasNewMove) noexcept
{
	auxMoveAvailable = hasNewMove;
	auxMoveLocked = false;
}

// Configure height following
GCodeResult Move::ConfigureHeightFollowing(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	if (heightController == nullptr)
	{
		heightController = new HeightController;
	}
	return heightController->Configure(gb, reply);
}

// Start/stop height following
GCodeResult Move::StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	if (heightController == nullptr)
	{
		reply.copy("Height following has not been configured");
		return GCodeResult::error;
	}
	return heightController->StartHeightFollowing(gb, reply);
}

#endif

// End
