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
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Tools/Tool.h>
#include <Endstops/ZProbe.h>
#include <Platform/TaskPriorities.h>

#if SUPPORT_IOBITS
# include <Platform/PortControl.h>
#endif

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanMotion.h>
# include <CAN/CanInterface.h>
#endif

Task<Move::MoveTaskStackWords> Move::moveTask;

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Move, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(Move, __VA_ARGS__)

static constexpr ObjectModelArrayDescriptor axesArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetTotalAxes(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&reprap.GetPlatform(), 3); }
};

static constexpr ObjectModelArrayDescriptor extrudersArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetNumExtruders(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&reprap.GetPlatform(), 4); }
};

constexpr ObjectModelArrayDescriptor Move::queueArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ARRAY_SIZE(rings); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(&((const Move*)self)->rings[context.GetLastIndex()]); }
};

#if SUPPORT_COORDINATE_ROTATION

constexpr ObjectModelArrayDescriptor Move::rotationCentreArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(reprap.GetGCodes().GetRotationCentre(context.GetLastIndex())); }
};

#endif

constexpr ObjectModelTableEntry Move::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Move members
	{ "axes",					OBJECT_MODEL_FUNC_NOSELF(&axesArrayDescriptor), 												ObjectModelEntryFlags::live },
	{ "calibration",			OBJECT_MODEL_FUNC(self, 3),																		ObjectModelEntryFlags::none },
	{ "compensation",			OBJECT_MODEL_FUNC(self, 6),																		ObjectModelEntryFlags::none },
	{ "currentMove",			OBJECT_MODEL_FUNC(self, 2),																		ObjectModelEntryFlags::live },
	{ "extruders",				OBJECT_MODEL_FUNC_NOSELF(&extrudersArrayDescriptor),											ObjectModelEntryFlags::live },
	{ "idle",					OBJECT_MODEL_FUNC(self, 1),																		ObjectModelEntryFlags::none },
	{ "kinematics",				OBJECT_MODEL_FUNC(self->kinematics),															ObjectModelEntryFlags::none },
	{ "limitAxes",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().LimitAxes()),										ObjectModelEntryFlags::none },
	{ "noMovesBeforeHoming",	OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().NoMovesBeforeHoming()),								ObjectModelEntryFlags::none },
	{ "printingAcceleration",	OBJECT_MODEL_FUNC(InverseConvertAcceleration(self->maxPrintingAcceleration), 1),				ObjectModelEntryFlags::none },
	{ "queue",					OBJECT_MODEL_FUNC_NOSELF(&queueArrayDescriptor),												ObjectModelEntryFlags::none },
#if SUPPORT_COORDINATE_ROTATION
	{ "rotation",				OBJECT_MODEL_FUNC(self, 44),																	ObjectModelEntryFlags::none },
#endif
	{ "shaping",				OBJECT_MODEL_FUNC(&self->axisShaper, 0),														ObjectModelEntryFlags::none },
	{ "speedFactor",			OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetSpeedFactor(), 2),								ObjectModelEntryFlags::none },
	{ "travelAcceleration",		OBJECT_MODEL_FUNC(InverseConvertAcceleration(self->maxTravelAcceleration), 1),					ObjectModelEntryFlags::none },
	{ "virtualEPos",			OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetVirtualExtruderPosition(), 5),					ObjectModelEntryFlags::live },
	{ "workplaceNumber",		OBJECT_MODEL_FUNC_NOSELF((int32_t)reprap.GetGCodes().GetWorkplaceCoordinateSystemNumber() - 1),	ObjectModelEntryFlags::none },
	{ "workspaceNumber",		OBJECT_MODEL_FUNC_NOSELF((int32_t)reprap.GetGCodes().GetWorkplaceCoordinateSystemNumber()),		ObjectModelEntryFlags::obsolete },

	// 1. Move.Idle members
	{ "factor",					OBJECT_MODEL_FUNC_NOSELF(reprap.GetPlatform().GetIdleCurrentFactor(), 1),						ObjectModelEntryFlags::none },
	{ "timeout",				OBJECT_MODEL_FUNC(0.001f * (float)self->idleTimeout, 1),										ObjectModelEntryFlags::none },

	// 2. move.currentMove members
	{ "acceleration",			OBJECT_MODEL_FUNC(self->GetAccelerationMmPerSecSquared(), 1),									ObjectModelEntryFlags::live },
	{ "deceleration",			OBJECT_MODEL_FUNC(self->GetDecelerationMmPerSecSquared(), 1),									ObjectModelEntryFlags::live },
# if SUPPORT_LASER
	{ "laserPwm",				OBJECT_MODEL_FUNC_IF_NOSELF(reprap.GetGCodes().GetMachineType() == MachineType::laser,
															reprap.GetPlatform().GetLaserPwm(), 2),								ObjectModelEntryFlags::live },
# endif
	{ "requestedSpeed",			OBJECT_MODEL_FUNC(self->GetRequestedSpeedMmPerSec(), 1),										ObjectModelEntryFlags::live },
	{ "topSpeed",				OBJECT_MODEL_FUNC(self->GetTopSpeedMmPerSec(), 1),												ObjectModelEntryFlags::live },

	// 3. move.calibration members
	{ "final",					OBJECT_MODEL_FUNC(self, 5),																		ObjectModelEntryFlags::none },
	{ "initial",				OBJECT_MODEL_FUNC(self, 4),																		ObjectModelEntryFlags::none },
	{ "numFactors",				OBJECT_MODEL_FUNC((int32_t)self->numCalibratedFactors),											ObjectModelEntryFlags::none },

	// 4. move.calibration.initialDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->initialCalibrationDeviation.GetDeviationFromMean(), 3),					ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->initialCalibrationDeviation.GetMean(), 3),								ObjectModelEntryFlags::none },

	// 5. move.calibration.finalDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->latestCalibrationDeviation.GetDeviationFromMean(), 3),					ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->latestCalibrationDeviation.GetMean(), 3),								ObjectModelEntryFlags::none },

	// 6. move.compensation members
	{ "fadeHeight",				OBJECT_MODEL_FUNC((self->useTaper) ? self->taperHeight : std::numeric_limits<float>::quiet_NaN(), 1),	ObjectModelEntryFlags::none },
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	{ "file",					OBJECT_MODEL_FUNC_IF(self->usingMesh, self->heightMap.GetFileName()),							ObjectModelEntryFlags::none },
#endif
	{ "liveGrid",				OBJECT_MODEL_FUNC_IF(self->usingMesh, (const GridDefinition *)&self->GetGrid()),				ObjectModelEntryFlags::none },
	{ "meshDeviation",			OBJECT_MODEL_FUNC_IF(self->usingMesh, self, 7),													ObjectModelEntryFlags::none },
	{ "probeGrid",				OBJECT_MODEL_FUNC_NOSELF((const GridDefinition *)&reprap.GetGCodes().GetDefaultGrid()),			ObjectModelEntryFlags::none },
	{ "skew",					OBJECT_MODEL_FUNC(self, 8),																		ObjectModelEntryFlags::none },
	{ "type",					OBJECT_MODEL_FUNC(self->GetCompensationTypeString()),											ObjectModelEntryFlags::none },

	// 7. move.compensation.meshDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->latestMeshDeviation.GetDeviationFromMean(), 3),							ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->latestMeshDeviation.GetMean(), 3),										ObjectModelEntryFlags::none },

	// 8. move.compensation.skew members
	{ "compensateXY",			OBJECT_MODEL_FUNC(self->compensateXY),															ObjectModelEntryFlags::none },
	{ "tanXY",					OBJECT_MODEL_FUNC(self->tanXY, 4),																ObjectModelEntryFlags::none },
	{ "tanXZ",					OBJECT_MODEL_FUNC(self->tanXZ, 4),																ObjectModelEntryFlags::none },
	{ "tanYZ",					OBJECT_MODEL_FUNC(self->tanYZ, 4),																ObjectModelEntryFlags::none },

#if SUPPORT_COORDINATE_ROTATION
	// 8. move.rotation members
	{ "angle",					OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetRotationAngle()),								ObjectModelEntryFlags::none },
	{ "centre",					OBJECT_MODEL_FUNC_NOSELF(&rotationCentreArrayDescriptor),										ObjectModelEntryFlags::none },
#endif
};

constexpr uint8_t Move::objectModelTableDescriptor[] =
{
	9 + SUPPORT_COORDINATE_ROTATION,
	17 + SUPPORT_WORKPLACE_COORDINATES,
	2,
	4 + SUPPORT_LASER,
	3,
	2,
	2,
	6 + (HAS_MASS_STORAGE || HAS_SBC_INTERFACE),
	2,
	4,
#if SUPPORT_COORDINATE_ROTATION
	2
#endif
};

DEFINE_GET_OBJECT_MODEL_TABLE(Move)

// The Move task starts executing here
[[noreturn]] static void MoveStart(void *param) noexcept
{
	static_cast<Move *>(param)->MoveLoop();
}

Move::Move() noexcept
	:
#if SUPPORT_ASYNC_MOVES
	  heightController(nullptr),
#endif
	  maxPrintingAcceleration(ConvertAcceleration(DefaultPrintingAcceleration)), maxTravelAcceleration(ConvertAcceleration(DefaultTravelAcceleration)),
	  jerkPolicy(0),
	  numCalibratedFactors(0)
{
	// Kinematics must be set up here because GCodes::Init asks the kinematics for the assumed initial position
	kinematics = Kinematics::Create(KinematicsType::cartesian);		// default to Cartesian
	mainDDARing.Init1(InitialDdaRingLength);
#if SUPPORT_ASYNC_MOVES
	auxDDARing.Init1(AuxDdaRingLength);
#endif
	DriveMovement::InitialAllocate(InitialNumDms);
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
	compensateXY = true;
	tanXY = tanYZ = tanXZ = 0.0;

	usingMesh = useTaper = false;
	zShift = 0.0;

	idleTimeout = DefaultIdleTimeout;
	moveState = MoveState::idle;
	whenLastMoveAdded = whenIdleTimerStarted = millis();

	simulationMode = SimulationMode::off;
	longestGcodeWaitInterval = 0;
	bedLevellingMoveAvailable = false;

	moveTask.Create(MoveStart, "Move", this, TaskPriority::MovePriority);
}

void Move::Exit() noexcept
{
	StepTimer::DisableTimerInterrupt();
	mainDDARing.Exit();
#if SUPPORT_ASYNC_MOVES
	auxDDARing.Exit();
#endif
#if SUPPORT_LASER || SUPPORT_IOBITS
	delete laserTask;
	laserTask = nullptr;
#endif
	moveTask.TerminateAndUnlink();
}

[[noreturn]] void Move::MoveLoop() noexcept
{
	for (;;)
	{
		// Recycle the DDAs for completed moves, checking for DDA errors to print if Move debug is enabled
		mainDDARing.RecycleDDAs();
#if SUPPORT_ASYNC_MOVES
		auxDDARing.RecycleDDAs();
#endif

		// See if we can add another move to the ring
		bool moveRead = false;
		const bool canAddMove = mainDDARing.CanAddMove();
		if (canAddMove)
		{
			// OK to add another move. First check if a special move is available.
			if (bedLevellingMoveAvailable)
			{
				moveRead = true;
				if (simulationMode < SimulationMode::partial)
				{
					if (mainDDARing.AddSpecialMove(reprap.GetPlatform().MaxFeedrate(Z_AXIS), specialMoveCoords))
					{
						const uint32_t now = millis();
						const uint32_t timeWaiting = now - whenLastMoveAdded;
						if (timeWaiting > longestGcodeWaitInterval)
						{
							longestGcodeWaitInterval = timeWaiting;
						}
						whenLastMoveAdded = now;
						moveState = MoveState::collecting;
					}
				}
				bedLevellingMoveAvailable = false;
			}
			else
			{
				// If there's a G Code move available, add it to the DDA ring for processing.
				RawMove nextMove;
				if (reprap.GetGCodes().ReadMove(nextMove))				// if we have a new move
				{
					moveRead = true;
					if (simulationMode < SimulationMode::partial)		// in simulation mode partial, we don't process incoming moves beyond this point
					{
						if (nextMove.moveType == 0)
						{
							AxisAndBedTransform(nextMove.coords, nextMove.tool, true);
						}

						if (mainDDARing.AddStandardMove(nextMove, !IsRawMotorMove(nextMove.moveType)))
						{
							const uint32_t now = millis();
							const uint32_t timeWaiting = now - whenLastMoveAdded;
							if (timeWaiting > longestGcodeWaitInterval)
							{
								longestGcodeWaitInterval = timeWaiting;
							}
							whenLastMoveAdded = now;
							moveState = MoveState::collecting;
						}
					}
				}
			}
		}

		// Let the DDA ring process moves. Better to have a few moves in the queue so that we can do lookahead, hence the test on idleCount and idleTime.
		uint32_t nextPrepareDelay = mainDDARing.Spin(simulationMode, !canAddMove, millis() - whenLastMoveAdded >= mainDDARing.GetGracePeriod());

#if SUPPORT_ASYNC_MOVES
		{
			bool waitingForAuxSpace = false;
			if (auxMoveAvailable)
			{
				if (auxDDARing.CanAddMove())
				{
					if (auxDDARing.AddAsyncMove(auxMove))
					{
						moveState = MoveState::collecting;
					}
					auxMoveAvailable = false;
				}
				else
				{
					waitingForAuxSpace = true;
				}
			}
			const uint32_t auxPrepareDelay = auxDDARing.Spin(simulationMode, waitingForAuxSpace, true);		// let the DDA ring process moves
			if (auxPrepareDelay < nextPrepareDelay)
			{
				nextPrepareDelay = auxPrepareDelay;
			}
		}
#endif

		// Reduce motor current to standby if the rings have been idle for long enough
		if (   mainDDARing.IsIdle()
#if SUPPORT_ASYNC_MOVES
			&& auxDDARing.IsIdle()
#endif
		   )
		{
			if (   moveState == MoveState::executing
				&& reprap.GetGCodes().GetPauseState() == PauseState::notPaused	// for now we don't go into idle hold when we are paused (is this sensible?)
			   )
			{
				whenIdleTimerStarted = millis();				// record when we first noticed that the machine was idle
				moveState = MoveState::timing;
			}
			else if (moveState == MoveState::timing && millis() - whenIdleTimerStarted >= idleTimeout)
			{
				reprap.GetPlatform().SetDriversIdle();			// put all drives in idle hold
				moveState = MoveState::idle;
			}
		}
		else
		{
			moveState = MoveState::executing;
		}

		// We need to be woken when one of the following is true:
		// 1. If moves are being executed and there are unprepared moves in the queue, when it is time to prepare more moves.
		// 2. If the queue was full and all moves in it were prepared, when we have completed one or more moves.
		// 3. In order to implement idle timeout, we must wake up regularly anyway, say every half second
		if (!moveRead && nextPrepareDelay != 0)
		{
			TaskBase::Take(min<uint32_t>(nextPrepareDelay, 500));
		}
	}
}

// This is called from GCodes to tell the Move task that a move is available
void Move::MoveAvailable() noexcept
{
	if (moveTask.IsRunning())
	{
		moveTask.Give();
	}
}

// Tell the lookahead ring we are waiting for it to empty and return true if it is
bool Move::WaitingForAllMovesFinished() noexcept
{
	return mainDDARing.SetWaitingToEmpty();
}

// Return the number of actually probed probe points
unsigned int Move::GetNumProbedProbePoints() const noexcept
{
	return probePoints.NumberOfProbePoints();
}

// Try to push some babystepping through the lookahead queue, returning the amount pushed
// This is called by the Main task, so we need to lock out the Move task while doing this
float Move::PushBabyStepping(size_t axis, float amount) noexcept
{
	TaskCriticalSectionLocker lock;						// lock out the Move task

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
bool Move::IsAccessibleProbePoint(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept
{
	return kinematics->IsReachable(axesCoords, axes);
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
	// Get the type of bed compensation in use
#if 0	// debug only
	String<StringLength256> scratchString;
#else
	String<StringLength50> scratchString;
#endif
	scratchString.copy(GetCompensationTypeString());

	Platform& p = reprap.GetPlatform();
	p.MessageF(mtype, "=== Move ===\nDMs created %u, segments created %u, maxWait %" PRIu32 "ms, bed compensation in use: %s, comp offset %.3f\n",
						DriveMovement::NumCreated(), MoveSegment::NumCreated(), longestGcodeWaitInterval, scratchString.c_str(), (double)zShift);
	longestGcodeWaitInterval = 0;

#if 0	// debug only
	scratchString.copy("Steps requested/done:");
	for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
	{
		scratchString.catf(" %" PRIu32 "/%" PRIu32, DDA::stepsRequested[driver], DDA::stepsDone[driver]);
		DDA::stepsRequested[driver] = DDA::stepsDone[driver] = 0;
	}
	scratchString.cat('\n');
	p.Message(mtype, scratchString.c_str());
#endif

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

	// DEBUG
#if 0
	extern uint32_t maxDelay;
	extern uint32_t maxDelayIncrease;
	p.MessageF(mtype, "Max delay %" PRIu32 ", increase %" PRIu32 "\n", maxDelay, maxDelayIncrease);
	maxDelay = maxDelayIncrease = 0;
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
	memcpyf(newPos, positionNow, ARRAY_SIZE(newPos));			// copy to local storage because Transform modifies it
	AxisAndBedTransform(newPos, reprap.GetCurrentTool(), doBedCompensation);

	mainDDARing.SetLiveCoordinates(newPos);
	mainDDARing.SetPositions(newPos);
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
		const size_t lowestXAxis = xAxes.LowestSetBit();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (xAxes.IsBitSet(axis))
			{
				xyzPoint[axis] += (compensateXY ? tanXY*xyzPoint[lowestYAxis] : 0.0) + tanXZ*xyzPoint[Z_AXIS];
			}
			if (yAxes.IsBitSet(axis))
			{
				xyzPoint[axis] += (compensateXY ? 0.0 : tanXY*xyzPoint[lowestXAxis]) + tanYZ*xyzPoint[Z_AXIS];
			}
		}
	}
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
		const size_t lowestXAxis = xAxes.LowestSetBit();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (yAxes.IsBitSet(axis))
			{
				xyzPoint[axis] -= ((compensateXY ? 0.0 : tanXY*xyzPoint[lowestXAxis]) + tanYZ*xyzPoint[Z_AXIS]);
			}
			if (xAxes.IsBitSet(axis))
			{
				xyzPoint[axis] -= ((compensateXY ? tanXY*xyzPoint[lowestYAxis] : 0.0) + tanXZ*xyzPoint[Z_AXIS]);
			}
		}
	}
}

// Compute the height correction needed at a point, ignoring taper
float Move::ComputeHeightCorrection(float xyzPoint[MaxAxes], const Tool *tool) const noexcept
{
	float zCorrection = 0.0;
	unsigned int numCorrections = 0;
	const GridDefinition& grid = GetGrid();
	const AxesBitmap axis1Axes = Tool::GetAxisMapping(tool, grid.GetAxisNumber(1));

	// Transform the Z coordinate based on the average correction for each axis used as an X or Y axis.
	Tool::GetAxisMapping(tool, grid.GetAxisNumber(0))
		.Iterate([this, xyzPoint, tool, axis1Axes, &zCorrection, &numCorrections](unsigned int axis0Axis, unsigned int)
					{
						const float axis0Coord = xyzPoint[axis0Axis] + Tool::GetOffset(tool, axis0Axis);
						axis1Axes.Iterate([this, xyzPoint, tool, axis0Coord, &zCorrection, &numCorrections](unsigned int axis1Axis, unsigned int)
											{
												const float axis1Coord = xyzPoint[axis1Axis] + Tool::GetOffset(tool, axis1Axis);
												zCorrection += heightMap.GetInterpolatedHeightError(axis0Coord, axis1Coord);
												++numCorrections;
											}
										);
					}
				);

	if (numCorrections > 1)
	{
		zCorrection /= numCorrections;			// take an average
	}

	return zCorrection + zShift;
}

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept
{
	if (usingMesh)
	{
		const float toolHeight = xyzPoint[Z_AXIS] + Tool::GetOffset(tool, Z_AXIS);
		if (!useTaper || toolHeight < taperHeight)
		{
			const float zCorrection = ComputeHeightCorrection(xyzPoint, tool);
			xyzPoint[Z_AXIS] += (useTaper && zCorrection < taperHeight) ? (taperHeight - toolHeight) * recipTaperHeight * zCorrection : zCorrection;
		}
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept
{
	if (usingMesh)
	{
		const float zCorrection = ComputeHeightCorrection(xyzPoint, tool);
		if (!useTaper || zCorrection >= taperHeight)	// need check on zCorrection to avoid possible divide by zero
		{
			xyzPoint[Z_AXIS] -= zCorrection;
		}
		else
		{
			const float toolZoffset = Tool::GetOffset(tool, Z_AXIS);
			const float zreq = (xyzPoint[Z_AXIS] - (taperHeight - toolZoffset) * zCorrection * recipTaperHeight)/(1.0 - zCorrection * recipTaperHeight);
			if (zreq + toolZoffset < taperHeight)
			{
				xyzPoint[Z_AXIS] = zreq;
			}
		}
	}
}

// Normalise the bed transform to have zero height error at these bed coordinates
void Move::SetZeroHeightError(const float coords[MaxAxes]) noexcept
{
	if (usingMesh)
	{
		float tempCoords[MaxAxes];
		memcpyf(tempCoords, coords, ARRAY_SIZE(tempCoords));
		AxisTransform(tempCoords, nullptr);
		const GridDefinition& grid = GetGrid();
		zShift = -heightMap.GetInterpolatedHeightError(tempCoords[grid.GetAxisNumber(0)], tempCoords[grid.GetAxisNumber(1)]);
	}
	else
	{
		zShift = 0.0;
	}
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

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Load the height map from file, returning true if an error occurred with the error reason appended to the buffer
bool Move::LoadHeightMapFromFile(FileStore *f, const char *fname, const StringRef& r) noexcept
{
	const bool err = heightMap.LoadFromFile(f, fname, r);
	if (err)
	{
		heightMap.ClearGridHeights();							// make sure we don't end up with a partial height map
	}
	else
	{
		zShift = 0.0;
	}
	float minError, maxError;
	(void)heightMap.GetStatistics(latestMeshDeviation, minError, maxError);
	reprap.MoveUpdated();
	return err;
}

// Save the height map to a file returning true if an error occurred
bool Move::SaveHeightMapToFile(FileStore *f, const char *fname) noexcept
{
	return heightMap.SaveToFile(f, fname, zShift);
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
		reprap.MoveUpdated();
	}
}

bool Move::IsXYCompensated() const
{
	return compensateXY;
}

void Move::SetXYCompensation(bool xyCompensation)
{
	compensateXY = xyCompensation;
	reprap.MoveUpdated();
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
			reply.copy("This kinematics does not support auto-calibration");
			error = true;
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
		reprap.GetPlatform().Message(ErrorMessage, "Z probe point index out of range\n");
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
			x -= zp->GetOffset(X_AXIS);
			y -= zp->GetOffset(Y_AXIS);
		}
	}
	return probePoints.GetZHeight(count);
}

// Enter or leave simulation mode
void Move::Simulate(SimulationMode simMode) noexcept
{
	simulationMode = simMode;
	if (simMode != SimulationMode::off)
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
	MoveAvailable();
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

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Write settings for resuming the print
// The GCodes module deals with the head position so all we need worry about is the bed compensation
// We don't handle random probe point bed compensation, and we assume that if a height map is being used it is the default one.
bool Move::WriteResumeSettings(FileStore *f) const noexcept
{
	return kinematics->WriteResumeSettings(f) && (!usingMesh || f->Write("G29 S1\n"));
}

#endif

// Process M204
GCodeResult Move::ConfigureAccelerations(GCodeBuffer&gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	if (gb.Seen('S'))
	{
		// For backwards compatibility with old versions of Marlin (e.g. for Cura and the Prusa fork of slic3r), set both accelerations
		seen = true;
		maxTravelAcceleration = maxPrintingAcceleration = gb.GetAcceleration();
	}
	if (gb.Seen('P'))
	{
		seen = true;
		maxPrintingAcceleration = gb.GetAcceleration();
	}
	if (gb.Seen('T'))
	{
		seen = true;
		maxTravelAcceleration = gb.GetAcceleration();
	}
	if (seen)
	{
		reprap.MoveUpdated();
	}
	else
	{
		reply.printf("Maximum printing acceleration %.1f, maximum travel acceleration %.1f mm/sec^2",
						(double)InverseConvertAcceleration(maxPrintingAcceleration), (double)InverseConvertAcceleration(maxTravelAcceleration));
	}
	return GCodeResult::ok;
}

// Process M595
GCodeResult Move::ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const size_t ringNumber = (gb.Seen('Q')) ? gb.GetLimitedUIValue('Q', ARRAY_SIZE(rings)) : 0;
	return rings[ringNumber].ConfigureMovementQueue(gb, reply);
}

// Process M572
GCodeResult Move::ConfigurePressureAdvance(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (gb.Seen('S'))
	{
		const float advance = gb.GetFValue();
		if (!reprap.GetGCodes().LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}

		GCodeResult rslt = GCodeResult::ok;

#if SUPPORT_CAN_EXPANSION
		CanDriversData<float> canDriversToUpdate;
#endif
		if (gb.Seen('D'))
		{
			uint32_t eDrive[MaxExtruders];
			size_t eCount = MaxExtruders;
			gb.GetUnsignedArray(eDrive, eCount, false);
#if SUPPORT_CAN_EXPANSION
			Platform& platform = reprap.GetPlatform();
#endif
			for (size_t i = 0; i < eCount; i++)
			{
				const uint32_t extruder = eDrive[i];
				if (extruder >= reprap.GetGCodes().GetNumExtruders())
				{
					reply.printf("Invalid extruder number '%" PRIu32 "'", extruder);
					rslt = GCodeResult::error;
					break;
				}
				extruderShapers[extruder].SetKseconds(advance);
#if SUPPORT_CAN_EXPANSION
				const DriverId did = platform.GetExtruderDriver(extruder);
				if (did.IsRemote())
				{
					canDriversToUpdate.AddEntry(did, advance);
				}
#endif
			}
		}
		else
		{
			const Tool * const ct = reprap.GetCurrentTool();
			if (ct == nullptr)
			{
				reply.copy("No tool selected");
				rslt = GCodeResult::error;
			}
			else
			{
#if SUPPORT_CAN_EXPANSION
				ct->IterateExtruders([this, advance, &canDriversToUpdate](unsigned int extruder)
										{
											extruderShapers[extruder].SetKseconds(advance);
											const DriverId did = reprap.GetPlatform().GetExtruderDriver(extruder);
											if (did.IsRemote())
											{
												canDriversToUpdate.AddEntry(did, advance);
											}
										}
									);
#else
				ct->IterateExtruders([this, advance](unsigned int extruder)
										{
											extruderShapers[extruder].SetKseconds(advance);
										}
									);
#endif
			}
		}

		reprap.MoveUpdated();

#if SUPPORT_CAN_EXPANSION
		return max(rslt, CanInterface::SetRemotePressureAdvance(canDriversToUpdate, reply));
#else
		return rslt;
#endif
	}

	reply.copy("Extruder pressure advance");
	char c = ':';
	for (size_t i = 0; i < reprap.GetGCodes().GetNumExtruders(); ++i)
	{
		reply.catf("%c %.3f", c, (double)extruderShapers[i].GetKseconds());
		c = ',';
	}
	return GCodeResult::ok;
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult Move::EutSetRemotePressureAdvance(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept
{
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDirectDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								extruderShapers[driver].SetKseconds(msg.values[count]);
							}
						}
				   );
	return rslt;
}

#endif

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

// Set the mesh deviation. Caller must call MoveUpdated() after calling this. We don't do that here because the caller may change Move in other ways first.
void Move::SetLatestMeshDeviation(const Deviation& d) noexcept
{
	latestMeshDeviation = d;
}

const char *Move::GetCompensationTypeString() const noexcept
{
	return (usingMesh) ? "mesh" : "none";
}

void Move::WakeMoveTaskFromISR() noexcept
{
	if (moveTask.IsRunning())
	{
		moveTask.GiveFromISR();
	}
}

#if SUPPORT_LASER || SUPPORT_IOBITS

// Laser and IOBits support

Task<Move::LaserTaskStackWords> *Move::laserTask = nullptr;		// the task used to manage laser power or IOBits

extern "C" [[noreturn]] void LaserTaskStart(void * pvParameters) noexcept
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
				(void)TaskBase::Take(ticks);
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
				(void)TaskBase::Take(ticks);
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
// The caller must have locked the buffer before calling this. If it calls this with hasNewMove true, it must have populated the move buffer with the move details
void Move::ReleaseAuxMove(bool hasNewMove) noexcept
{
	auxMoveAvailable = hasNewMove;
	auxMoveLocked = false;
	MoveAvailable();
}

// Configure height following
GCodeResult Move::ConfigureHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (heightController == nullptr)
	{
		heightController = new HeightController;
	}
	return heightController->Configure(gb, reply);
}

// Start/stop height following
GCodeResult Move::StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
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
