/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "DDA.h"
#include "RepRap.h"
#include "Platform.h"
#include "Move.h"
#include "StepTimer.h"
#include "Endstops/EndstopsManager.h"
#include "Kinematics/LinearDeltaKinematics.h"
#include "Tools/Tool.h"

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanMotion.h"
#endif

#ifdef DUET_NG
# define DDA_MOVE_DEBUG	(0)
#else
// On the wired Duets we don't have enough RAM to support this
# define DDA_MOVE_DEBUG	(0)
#endif

#if DDA_MOVE_DEBUG

// Structure to hold the essential parameters of a move, for debugging
struct MoveParameters
{
	float accelDistance;
	float steadyDistance;
	float decelDistance;
	float requestedSpeed;
	float startSpeed;
	float topSpeed;
	float endSpeed;
	float targetNextSpeed;
	uint32_t endstopChecks;
	uint16_t flags;

	MoveParameters() noexcept
	{
		accelDistance = steadyDistance = decelDistance = requestedSpeed = startSpeed = topSpeed = endSpeed = targetNextSpeed = 0.0;
		endstopChecks = 0;
		flags = 0;
	}

	void DebugPrint() const noexcept
	{
		reprap.GetPlatform().MessageF(DebugMessage, "%f,%f,%f,%f,%f,%f,%f,%f,%08" PRIX32 ",%04x\n",
								(double)accelDistance, (double)steadyDistance, (double)decelDistance, (double)requestedSpeed, (double)startSpeed, (double)topSpeed, (double)endSpeed,
								(double)targetNextSpeed, endstopChecks, flags);
	}

	static void PrintHeading() noexcept
	{
		reprap.GetPlatform().Message(DebugMessage,
									"accelDistance,steadyDistance,decelDistance,requestedSpeed,startSpeed,topSpeed,endSpeed,"
									"targetNextSpeed,endstopChecks,flags\n");
	}
};

const size_t NumSavedMoves = 128;

static MoveParameters savedMoves[NumSavedMoves];
static size_t savedMovePointer = 0;

// Print the saved moves in CSV format for analysis
/*static*/ void DDA::PrintMoves() noexcept
{
	// Print the saved moved in CSV format
	MoveParameters::PrintHeading();
	for (size_t i = 0; i < NumSavedMoves; ++i)
	{
		savedMoves[savedMovePointer].DebugPrint();
		savedMovePointer = (savedMovePointer + 1) % NumSavedMoves;
	}
}

#else

/*static*/ void DDA::PrintMoves() noexcept { }

#endif

#if DDA_LOG_PROBE_CHANGES

size_t DDA::numLoggedProbePositions = 0;
int32_t DDA::loggedProbePositions[XYZ_AXES * MaxLoggedProbePositions];
bool DDA::probeTriggered = false;

void DDA::LogProbePosition() noexcept
{
	if (numLoggedProbePositions < MaxLoggedProbePositions)
	{
		int32_t *p = loggedProbePositions + (numLoggedProbePositions * XYZ_AXES);
		for (size_t drive = 0; drive < XYZ_AXES; ++drive)
		{
			DriveMovement *dm = pddm[drive];
			if (dm != nullptr && dm->state == DMState::moving)
			{
				p[drive] = endPoint[drive] - dm->GetNetStepsLeft();
			}
			else
			{
				p[drive] = endPoint[drive];
			}
		}
		++numLoggedProbePositions;
	}
}

#endif

DDA::DDA(DDA* n) noexcept : next(n), prev(nullptr), state(empty)
{
	activeDMs = completedDMs = nullptr;
	tool = nullptr;						// needed in case we pause before any moves have been done

	// Set the endpoints to zero, because Move will ask for them.
	// They will be wrong if we are on a delta. We take care of that when we process the M665 command in config.g.
	for (int32_t& ep : endPoint)
	{
		ep = 0;
	}

	flags.all = 0;						// in particular we need to set endCoordinatesValid to false
	virtualExtruderPosition = 0.0;
	filePos = noFilePosition;

#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif
}

void DDA::ReleaseDMs() noexcept
{
	// Normally there should be no active DMs, but release any that there may be
	for (DriveMovement* dm = activeDMs; dm != nullptr; )
	{
		DriveMovement* const next = dm->nextDM;
		DriveMovement::Release(dm);
		dm = next;
	}
	for (DriveMovement* dm = completedDMs; dm != nullptr; )
	{
		DriveMovement* const next = dm->nextDM;
		DriveMovement::Release(dm);
		dm = next;
	}
	activeDMs = completedDMs = nullptr;
}

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
int32_t DDA::GetTimeLeft() const noexcept
pre(state == executing || state == frozen || state == completed)
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(afterPrepare.moveStartTime + clocksNeeded - StepTimer::GetTimerTicks())
			: (int32_t)clocksNeeded;
}

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance. Now that we generate step pulses
// for multiple motors simultaneously, there is no need to preserve round-robin order.
inline void DDA::InsertDM(DriveMovement *dm) noexcept
{
	DriveMovement **dmp = &activeDMs;
	while (*dmp != nullptr && (*dmp)->nextStepTime < dm->nextStepTime)
	{
		dmp = &((*dmp)->nextDM);
	}
	dm->nextDM = *dmp;
	*dmp = dm;
}

// Remove this drive from the list of drives with steps due and put it in the completed list
// Called from the step ISR only.
void DDA::DeactivateDM(size_t drive) noexcept
{
	DriveMovement **dmp = &activeDMs;
	while (*dmp != nullptr)
	{
		DriveMovement * const dm = *dmp;
		if (dm->drive == drive)
		{
			(*dmp) = dm->nextDM;
			dm->state = DMState::idle;
			dm->nextDM = completedDMs;
			completedDMs = dm;
			break;
		}
		dmp = &(dm->nextDM);
	}
}

void DDA::DebugPrintVector(const char *name, const float *vec, size_t len) const noexcept
{
	debugPrintf("%s=", name);
	for (size_t i = 0; i < len; ++i)
	{
		debugPrintf("%c%f", ((i == 0) ? '[' : ' '), (double)vec[i]);
	}
	debugPrintf("]");
}

// Print the text followed by the DDA only
void DDA::DebugPrint(const char *tag) const noexcept
{
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	debugPrintf("%s DDA:", tag);
	if (flags.endCoordinatesValid)
	{
		float startCoordinates[MaxAxes];
		for (size_t i = 0; i < numAxes; ++i)
		{
			startCoordinates[i] = endCoordinates[i] - (totalDistance * directionVector[i]);
		}
		DebugPrintVector(" start", startCoordinates, numAxes);
		DebugPrintVector(" end", endCoordinates, numAxes);
	}

	debugPrintf(" s=%f", (double)totalDistance);
	DebugPrintVector(" vec", directionVector, 5);
	debugPrintf("\n"
				"a=%f d=%f reqv=%f startv=%f topv=%f endv=%f\n"
				"cks=%" PRIu32 " sstcda=%" PRIu32 " tstcddpdsc=%" PRIu32 " exac=%" PRIi32 "\n",
				(double)acceleration, (double)deceleration, (double)requestedSpeed, (double)startSpeed, (double)topSpeed, (double)endSpeed, clocksNeeded,
				afterPrepare.startSpeedTimesCdivA, afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks, afterPrepare.extraAccelerationClocks);
}

// Print the DDA and active DMs
void DDA::DebugPrintAll(const char *tag) const noexcept
{
	DebugPrint(tag);
	for (DriveMovement* dm = activeDMs; dm != nullptr; dm = dm->nextDM)
	{
		dm->DebugPrint();
	}
	for (DriveMovement* dm = completedDMs; dm != nullptr; dm = dm->nextDM)
	{
		dm->DebugPrint();
	}
}

// Set up a real move. Return true if it represents real movement, else false.
// Either way, return the amount of extrusion we didn't do in the extruder coordinates of nextMove
bool DDA::InitStandardMove(DDARing& ring, const RawMove &nextMove, bool doMotorMapping) noexcept
{
	// 0. If there are more total axes than visible axes, then we must ignore any movement data in nextMove for the invisible axes.
	// The call to CartesianToMotorSteps may adjust the invisible axis endpoints for architectures such as CoreXYU and delta with >3 towers, so set them up here.
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	const int32_t * const positionNow = prev->DriveCoordinates();
	for (size_t axis = numVisibleAxes; axis < numTotalAxes; ++axis)
	{
		endPoint[axis] = positionNow[axis];
	}

	// 1. Compute the new endpoints and the movement vector
	const Move& move = reprap.GetMove();
	if (doMotorMapping)
	{
		if (!move.CartesianToMotorSteps(nextMove.coords, endPoint, nextMove.isCoordinated))		// transform the axis coordinates if on a delta or CoreXY printer
		{
			return false;												// throw away the move if it couldn't be transformed
		}
		flags.isDeltaMovement = move.IsDeltaMode()
							&& (endPoint[X_AXIS] != positionNow[X_AXIS] || endPoint[Y_AXIS] != positionNow[Y_AXIS] || endPoint[Z_AXIS] != positionNow[Z_AXIS]);
	}
	else
	{
		flags.isDeltaMovement = false;
	}

	flags.xyMoving = false;
	bool axesMoving = false;
	bool extruding = false;												// we set this true if extrusion was commanded, even if it is too small to do
	bool forwardExtruding = false;
	bool realMove = false;
	float accelerations[MaxAxesPlusExtruders];
	const float * const normalAccelerations = reprap.GetPlatform().Accelerations();
	const Kinematics& k = move.GetKinematics();

	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		accelerations[drive] = normalAccelerations[drive];
		endCoordinates[drive] = nextMove.coords[drive];

		if (drive < numTotalAxes)
		{
			if (!doMotorMapping && drive < numVisibleAxes)
			{
				endPoint[drive] = Move::MotorMovementToSteps(drive, nextMove.coords[drive]);
			}

			const int32_t delta = endPoint[drive] - positionNow[drive];
			if (doMotorMapping)
			{
				if (drive >= numVisibleAxes)
				{
					directionVector[drive] = 0.0;
				}
				else
				{
					const float positionDelta = endCoordinates[drive] - prev->GetEndCoordinate(drive, false);
					directionVector[drive] = positionDelta;
					if (positionDelta != 0.0 && (Tool::GetXAxes(nextMove.tool).IsBitSet(drive) || Tool::GetYAxes(nextMove.tool).IsBitSet(drive)))
					{
						flags.xyMoving = true;
					}
				}
			}
			else
			{
				directionVector[drive] = (float)delta/reprap.GetPlatform().DriveStepsPerUnit(drive);
			}

			if (delta != 0)
			{
				realMove = true;
				axesMoving = true;
			}
		}
		else if (LogicalDriveToExtruder(drive) < reprap.GetGCodes().GetNumExtruders())
		{
			// It's an extruder drive. We defer calculating the steps because they may be affected by nonlinear extrusion, which we can't calculate until we
			// know the speed of the move, and because extruder movement is relative so we need to accumulate fractions of a whole step between moves.
			const float movement = nextMove.coords[drive];
			directionVector[drive] = movement;
			if (movement != 0.0)
			{
				realMove = true;
				extruding = true;
				if (movement > 0.0)
				{
					forwardExtruding = true;
				}
				if (flags.xyMoving && nextMove.usePressureAdvance)
				{
					const float compensationTime = reprap.GetPlatform().GetPressureAdvance(LogicalDriveToExtruder(drive));
					if (compensationTime > 0.0)
					{
						// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
						accelerations[drive] = min<float>(accelerations[drive], reprap.GetPlatform().GetInstantDv(drive)/compensationTime);
					}
				}
			}
		}
		else
		{
			directionVector[drive] = 0.0;
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		// Update the end position in the previous move, so that on the next move we don't think there is XY movement when the user didn't ask for any
		if (doMotorMapping)
		{
			for (size_t drive = 0; drive < numTotalAxes; ++drive)
			{
				prev->endCoordinates[drive] = nextMove.coords[drive];
			}
		}
		return false;
	}

	// 3. Store some values
	tool = nextMove.tool;
	flags.checkEndstops = nextMove.checkEndstops;
	flags.reduceAcceleration = nextMove.reduceAcceleration;
	filePos = nextMove.filePos;
	virtualExtruderPosition = nextMove.virtualExtruderPosition;
	proportionDone = nextMove.proportionDone;
	initialUserX = nextMove.initialUserX;
	initialUserY = nextMove.initialUserY;

	flags.canPauseAfter = nextMove.canPauseAfter;
	flags.usingStandardFeedrate = nextMove.usingStandardFeedrate;
	flags.isPrintingMove = flags.xyMoving && forwardExtruding;				// require forward extrusion so that wipe-while-retracting doesn't count
	flags.isNonPrintingExtruderMove = extruding && !flags.isPrintingMove;	// flag used by filament monitors - we can ignore Z movement
	flags.usePressureAdvance = nextMove.usePressureAdvance;
	flags.hadLookaheadUnderrun = false;
	flags.isLeadscrewAdjustmentMove = false;
	flags.goingSlow = false;
	flags.controlLaser = nextMove.isCoordinated && nextMove.checkEndstops == 0;

	// The end coordinates will be valid at the end of this move if it does not involve endstop checks and is not a raw motor move
	flags.endCoordinatesValid = !nextMove.checkEndstops && doMotorMapping;
	flags.continuousRotationShortcut = (nextMove.moveType == 0);

#if SUPPORT_LASER || SUPPORT_IOBITS
	if (flags.controlLaser)
	{
		laserPwmOrIoBits = nextMove.laserPwmOrIoBits;
	}
	else
	{
		laserPwmOrIoBits.Clear();
	}
#endif

	// If it's a Z probing move, limit the Z acceleration to better handle nozzle-contact probes
	if (flags.reduceAcceleration && accelerations[Z_AXIS] > ZProbeMaxAcceleration)
	{
		accelerations[Z_AXIS] = ZProbeMaxAcceleration;
	}

	// 4. Normalise the direction vector and compute the amount of motion.
	if (flags.xyMoving)
	{
		// There is some XY movement, so normalise the direction vector so that the total XYZ movement has unit length and 'totalDistance' is the XYZ distance moved.
		// This means that the user gets the feed rate that he asked for. It also makes the delta calculations simpler.
		// First do the bed tilt compensation for deltas.
		directionVector[Z_AXIS] += (directionVector[X_AXIS] * k.GetTiltCorrection(X_AXIS)) + (directionVector[Y_AXIS] * k.GetTiltCorrection(Y_AXIS));
		totalDistance = NormaliseXYZ();
	}
	else if (axesMoving)
	{
		// Some axes are moving, but not axes that X or Y are mapped to. Normalise the movement to the vector sum of the axes that are moving.
		totalDistance = Normalise(directionVector, MaxAxesPlusExtruders, numTotalAxes);
	}
	else
	{
		// Extruder-only movement. Normalise so that the magnitude is the total absolute movement. This gives the correct feed rate for mixing extruders.
		totalDistance = 0.0;
		for (size_t d = 0; d < MaxAxesPlusExtruders; d++)
		{
			totalDistance += fabsf(directionVector[d]);
		}
		if (totalDistance > 0.0)		// should always be true
		{
			Scale(directionVector, 1.0/totalDistance, MaxAxesPlusExtruders);
		}
	}

	// 5. Compute the maximum acceleration available
	float normalisedDirectionVector[MaxAxesPlusExtruders];			// used to hold a unit-length vector in the direction of motion
	memcpy(normalisedDirectionVector, directionVector, sizeof(normalisedDirectionVector));
	Absolute(normalisedDirectionVector, MaxAxesPlusExtruders);
	acceleration = beforePrepare.maxAcceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations, MaxAxesPlusExtruders);
	if (flags.xyMoving)											// apply M204 acceleration limits to XY moves
	{
		acceleration = min<float>(acceleration, (flags.isPrintingMove) ? move.GetMaxPrintingAcceleration() : move.GetMaxTravelAcceleration());
	}
	deceleration = acceleration;

	// 6. Set the speed to the smaller of the requested and maximum speed.
	// Also enforce a minimum speed of 0.5mm/sec. We need a minimum speed to avoid overflow in the movement calculations.
	float reqSpeed = nextMove.feedRate;
	if (!doMotorMapping)
	{
		// Special case of a raw or homing move on a delta printer
		// We use the Cartesian motion system to implement these moves, so the feed rate will be interpreted in Cartesian coordinates.
		// This is wrong, we want the feed rate to apply to the drive that is moving the farthest.
		float maxDistance = 0.0;
		for (size_t axis = 0; axis < numTotalAxes; ++axis)
		{
			if (k.GetMotionType(axis) == MotionType::segmentFreeDelta && normalisedDirectionVector[axis] > maxDistance)
			{
				maxDistance = normalisedDirectionVector[axis];
			}
		}
		if (maxDistance != 0.0)				// should always be true
		{
			reqSpeed /= maxDistance;		// because normalisedDirectionVector is unit-normalised
		}
	}

	// Don't use the constrain function in the following, because if we have a very small XY movement and a lot of extrusion, we may have to make the
	// speed lower than the configured minimum movement speed. We must apply the minimum speed first and then limit it if necessary after that.
	requestedSpeed = min<float>(max<float>(reqSpeed, reprap.GetPlatform().MinMovementSpeed()),
								VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform().MaxFeedrates(), MaxAxesPlusExtruders));

	// On a Cartesian printer, it is OK to limit the X and Y speeds and accelerations independently, and in consequence to allow greater values
	// for diagonal moves. On other architectures, this is not OK and any movement in the XY plane should be limited on other ways.
	if (doMotorMapping)
	{
		k.LimitSpeedAndAcceleration(*this, normalisedDirectionVector, numVisibleAxes, flags.continuousRotationShortcut);	// give the kinematics the chance to further restrict the speed and acceleration
	}

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	endSpeed = 0.0;							// until the next move asks us to adjust it

	if (prev->state == provisional && (move.GetJerkPolicy() != 0 || (flags.isPrintingMove == prev->flags.isPrintingMove && flags.xyMoving == prev->flags.xyMoving)))
	{
		// Try to meld this move to the previous move to avoid stop/start
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		prev->beforePrepare.targetNextSpeed = min<float>(sqrtf(deceleration * totalDistance * 2.0), requestedSpeed);
		DoLookahead(ring, prev);
		startSpeed = prev->endSpeed;
	}
	else
	{
		// There is no previous move that we can adjust, so start at zero speed.
		startSpeed = 0.0;
	}

	RecalculateMove(ring);
	state = provisional;
	return true;
}

// Set up a leadscrew motor move returning true if the move does anything
bool DDA::InitLeadscrewMove(DDARing& ring, float feedrate, const float adjustments[MaxDriversPerAxis]) noexcept
{
	// 1. Compute the new endpoints and the movement vector
	bool realMove = false;

	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		endPoint[drive] = prev->endPoint[drive];				// adjusting leadscrews doesn't change the endpoint
		endCoordinates[drive] = prev->endCoordinates[drive];	// adjusting leadscrews doesn't change the position
		directionVector[drive] = 0.0;
	}

	for (size_t driver = 0; driver < MaxDriversPerAxis; ++driver)
	{
		directionVector[driver] = adjustments[driver];			// for leadscrew adjustment moves, store the adjustment needed in directionVector
		const int32_t delta = lrintf(adjustments[driver] * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));
		if (delta != 0)
		{
			realMove = true;
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		return false;
	}

	// 3. Store some values
	flags.isLeadscrewAdjustmentMove = true;
	flags.isDeltaMovement = false;
	flags.isPrintingMove = false;
	flags.xyMoving = false;
	flags.controlLaser = false;
	flags.checkEndstops = false;
	flags.reduceAcceleration = false;
	flags.canPauseAfter = true;
	flags.usingStandardFeedrate = false;
	flags.usePressureAdvance = false;
	flags.hadLookaheadUnderrun = false;
	flags.goingSlow = false;
	flags.continuousRotationShortcut = false;
	virtualExtruderPosition = prev->virtualExtruderPosition;
	tool = nullptr;
	filePos = prev->filePos;
	flags.endCoordinatesValid = prev->flags.endCoordinatesValid;
	acceleration = deceleration = reprap.GetPlatform().Accelerations()[Z_AXIS];

#if SUPPORT_LASER && SUPPORT_IOBITS
	if (reprap.GetGCodes().GetMachineType() == MachineType::laser)
	{
		laserPwmOrIoBits.Clear();
	}
	else
	{
		laserPwmOrIoBits = prev->laserPwmOrIoBits;
	}
#elif SUPPORT_LASER
	laserPwmOrIoBits.Clear();
#elif SUPPORT_IOBITS
	laserPwmOrIoBits = prev->laserPwmOrIoBits;
#endif

	// 4. Normalise the direction vector and compute the amount of motion.
	// Currently we normalise the vector sum of all Z motor movement to unit length.
	totalDistance = Normalise(directionVector, MaxAxesPlusExtruders, MaxAxesPlusExtruders);

	// 6. Set the speed to the smaller of the requested and maximum speed.
	requestedSpeed = feedrate;

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	startSpeed = endSpeed = 0.0;

	RecalculateMove(ring);
	state = provisional;
	return true;
}

# if SUPPORT_ASYNC_MOVES

// Set up an async motor move returning true if the move does anything.
// All async moves are relative and linear.
bool DDA::InitAsyncMove(DDARing& ring, const AsyncMove& nextMove) noexcept
{
	// 1. Compute the new endpoints and the movement vector
	bool realMove = false;

	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		// Note, the correspondence between endCoordinates and endPoint will not be exact because of rounding error.
		// This doesn't matter for the current application because we don't use either of these fields.

		// If it's a delta then we can only do async tower moves in the Z direction and on any additional linear axes
		const size_t axisToUse = (reprap.GetMove().GetKinematics().GetMotionType(drive) == MotionType::segmentFreeDelta) ? Z_AXIS : drive;
		directionVector[drive] = nextMove.movements[axisToUse];
		const int32_t delta = lrintf(nextMove.movements[axisToUse] * reprap.GetPlatform().DriveStepsPerUnit(drive));
		endPoint[drive] = prev->endPoint[drive] + delta;
		endCoordinates[drive] = prev->endCoordinates[drive];
		if (delta != 0)
		{
			realMove = true;
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		return false;
	}

	// 3. Store some values
	flags.isLeadscrewAdjustmentMove = false;
	flags.isDeltaMovement = false;
	flags.isPrintingMove = false;
	flags.xyMoving = false;
	flags.controlLaser = false;
	flags.checkEndstops = false;
	flags.reduceAcceleration = false;
	flags.canPauseAfter = true;
	flags.usingStandardFeedrate = false;
	flags.usePressureAdvance = false;
	flags.hadLookaheadUnderrun = false;
	flags.goingSlow = false;
	flags.continuousRotationShortcut = false;
	virtualExtruderPosition = 0;
	tool = nullptr;
	filePos = noFilePosition;
	flags.endCoordinatesValid = false;

	startSpeed = nextMove.startSpeed;
	endSpeed = nextMove.endSpeed;
	requestedSpeed = nextMove.requestedSpeed;
	acceleration = nextMove.acceleration;
	deceleration = nextMove.deceleration;

#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif

	// Currently we normalise the vector sum of all motor movements to unit length.
	totalDistance = Normalise(directionVector, MaxAxesPlusExtruders, MaxAxesPlusExtruders);

	RecalculateMove(ring);
	state = provisional;
	return true;
}

#endif

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
inline bool DDA::IsDecelerationMove() const noexcept
{
	return beforePrepare.decelDistance == totalDistance					// the simple case - is a deceleration-only move
			|| (topSpeed < requestedSpeed								// can't have been intended as deceleration-only if it reaches the requested speed
				&& beforePrepare.decelDistance > 0.98 * totalDistance	// rounding error can only go so far
			   );
}

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
inline bool DDA::IsAccelerationMove() const noexcept
{
	return beforePrepare.accelDistance == totalDistance					// the simple case - is an acceleration-only move
			|| (topSpeed < requestedSpeed								// can't have been intended as deceleration-only if it reaches the requested speed
				&& beforePrepare.accelDistance > 0.98 * totalDistance	// rounding error can only go so far
			   );
}

// Return true if there is no reason to delay preparing this move
bool DDA::IsGoodToPrepare() const noexcept
{
	return endSpeed >= topSpeed;							// if it never decelerates, we can't improve it
}

#if 0
#define LA_DEBUG	do { if (fabsf(fsquare(laDDA->endSpeed) - fsquare(laDDA->startSpeed)) > 2.02 * laDDA->acceleration * laDDA->totalDistance \
								|| laDDA->topSpeed > laDDA->requestedSpeed) { \
							debugPrintf("%s(%d) ", __FILE__, __LINE__);		\
							laDDA->DebugPrint();	\
						}	\
					} while(false)
#else
#define LA_DEBUG	do { } while(false)
#endif

// Try to increase the ending speed of this move to allow the next move to start at targetNextSpeed.
// Only called if this move and the next one are both printing moves.
/*static*/ void DDA::DoLookahead(DDARing& ring, DDA *laDDA) noexcept
pre(state == provisional)
{
//	if (reprap.Debug(moduleDda)) debugPrintf("Adjusting, %f\n", laDDA->targetNextSpeed);
	unsigned int laDepth = 0;
	bool goingUp = true;

	for(;;)					// this loop is used to nest lookahead without making recursive calls
	{
		if (goingUp)
		{
			// We have been asked to adjust the end speed of this move to match the next move starting at targetNextSpeed
			if (laDDA->beforePrepare.targetNextSpeed > laDDA->requestedSpeed)
			{
				laDDA->beforePrepare.targetNextSpeed = laDDA->requestedSpeed;			// don't try for an end speed higher than our requested speed
			}
			if (laDDA->topSpeed >= laDDA->requestedSpeed)
			{
				// This move already reaches its top speed, so we just need to adjust the deceleration part
				laDDA->MatchSpeeds();									// adjust it if necessary
				goingUp = false;
			}
			else if (   laDDA->IsDecelerationMove()
					 && laDDA->prev->state == DDA::provisional			// if we can't adjust the previous move then we don't care (and its figures may not be reliable if it has been recycled already)
					 && laDDA->prev->beforePrepare.decelDistance > 0.0	// if the previous move has no deceleration phase then no point in adjusting it
					)
			{
				// This is a deceleration-only move, so we may have to adjust the previous move as well to get optimum behaviour
				if (   laDDA->prev->state == provisional
					&& laDDA->prev->flags.xyMoving == laDDA->flags.xyMoving
					&& (   laDDA->prev->flags.isPrintingMove == laDDA->flags.isPrintingMove
						|| (laDDA->prev->flags.isPrintingMove && laDDA->prev->requestedSpeed == laDDA->requestedSpeed)	// special case to support coast-to-end
					   )
				   )
				{
					laDDA->MatchSpeeds();
					const float maxStartSpeed = sqrtf(fsquare(laDDA->beforePrepare.targetNextSpeed) + (2 * laDDA->deceleration * laDDA->totalDistance));
					laDDA->prev->beforePrepare.targetNextSpeed = min<float>(maxStartSpeed, laDDA->requestedSpeed);
					// leave 'recurse' true
				}
				else
				{
					// This move is a deceleration-only move but we can't adjust the previous one
					laDDA->flags.hadLookaheadUnderrun = true;
					const float maxReachableSpeed = sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->deceleration * laDDA->totalDistance));
					if (laDDA->beforePrepare.targetNextSpeed > maxReachableSpeed)
					{
						laDDA->beforePrepare.targetNextSpeed = maxReachableSpeed;
					}
					laDDA->MatchSpeeds();
					goingUp = false;
				}
			}
			else
			{
				// This move doesn't reach its requested speed, but it isn't a deceleration-only move
				// Set its end speed to the minimum of the requested speed and the highest we can reach
				const float maxReachableSpeed = sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
				if (laDDA->beforePrepare.targetNextSpeed > maxReachableSpeed)
				{
					// Looks like this is an acceleration segment, so to ensure smooth acceleration we should reduce targetNextSpeed to endSpeed as well
					laDDA->beforePrepare.targetNextSpeed = maxReachableSpeed;
				}
				laDDA->MatchSpeeds();
				goingUp = false;
			}
		}
		else
		{
			// Going back down the list
			// We have adjusted the end speed of the previous move as much as is possible. Adjust this move to match it.
			laDDA->startSpeed = laDDA->prev->endSpeed;
			const float maxEndSpeed = sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
			if (maxEndSpeed < laDDA->beforePrepare.targetNextSpeed)
			{
				laDDA->beforePrepare.targetNextSpeed = maxEndSpeed;
			}
		}

		if (goingUp)
		{
			// Still going up
			laDDA = laDDA->prev;
			++laDepth;
			if (reprap.Debug(moduleDda))
			{
				debugPrintf("Recursion start %u\n", laDepth);
			}
		}
		else
		{
			// Either just stopped going up, or going down
			if (laDDA->beforePrepare.targetNextSpeed < laDDA->endSpeed)
			{
				// This situation should not normally happen except by a small amount because of rounding error.
				// Don't reduce the end speed of the current move, because that may make the move infeasible.
				// Report a lookahead error if the change is too large to be accounted for by rounding error.
				if (laDDA->beforePrepare.targetNextSpeed < laDDA->endSpeed * 0.99)
				{
					ring.RecordLookaheadError();
					if (reprap.Debug(moduleMove))
					{
						debugPrintf("DDA.cpp(%d) tn=%f ", __LINE__, (double)laDDA->beforePrepare.targetNextSpeed);
						laDDA->DebugPrint("la");
					}
				}
			}
			else
			{
				laDDA->endSpeed = laDDA->beforePrepare.targetNextSpeed;
			}
LA_DEBUG;
			laDDA->RecalculateMove(ring);

			if (laDepth == 0)
			{
//				if (reprap.Debug(moduleDda)) debugPrintf("Complete, %f\n", laDDA->targetNextSpeed);
				return;
			}

			laDDA = laDDA->next;
			--laDepth;
		}
	}
}

// Try to push babystepping earlier in the move queue, returning the amount we pushed
//TODO this won't work for CoreXZ, rotary delta, Kappa, or SCARA with Z crosstalk
float DDA::AdvanceBabyStepping(DDARing& ring, size_t axis, float amount) noexcept
{
	if (axis != Z_AXIS)
	{
		return 0.0;				// only Z axis babystepping is supported at present
	}

	DDA *cdda = this;
	while (cdda->prev->state == DDAState::provisional)
	{
		cdda = cdda->prev;
	}

	// cdda addresses the earliest un-prepared move, which is the first one we can apply babystepping to
	// Allow babystepping Z speed up to 10% of the move top speed or up to half the Z jerk rate, whichever is lower
	float babySteppingDone = 0.0;
	while(cdda != this)
	{
		float babySteppingToDo = 0.0;
		if (amount != 0.0 && cdda->flags.xyMoving)
		{
			// Limit the babystepping Z speed to the lower of 0.1 times the original XYZ speed and 0.5 times the Z jerk
			const float maxBabySteppingAmount = cdda->totalDistance * min<float>(0.1, 0.5 * reprap.GetPlatform().GetInstantDv(Z_AXIS)/cdda->topSpeed);
			babySteppingToDo = constrain<float>(amount, -maxBabySteppingAmount, maxBabySteppingAmount);
			cdda->directionVector[Z_AXIS] += babySteppingToDo/cdda->totalDistance;
			cdda->totalDistance *= cdda->NormaliseXYZ();
			cdda->RecalculateMove(ring);
			babySteppingDone += babySteppingToDo;
			amount -= babySteppingToDo;
		}

		// Even if there is no babystepping to do this move, we may need to adjust the end coordinates
		cdda->endCoordinates[Z_AXIS] += babySteppingDone;
		if (cdda->flags.isDeltaMovement)
		{
			for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
			{
				if (reprap.GetMove().GetKinematics().GetMotionType(axis) == MotionType::segmentFreeDelta)
				{
					cdda->endPoint[axis] += (int32_t)(babySteppingDone * reprap.GetPlatform().DriveStepsPerUnit(axis));
				}
			}
		}
		else
		{
			cdda->endPoint[Z_AXIS] += (int32_t)(babySteppingDone * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));
		}

		// Now do the next move
		cdda = cdda->next;
	}

	return babySteppingDone;
}

// Recalculate the top speed, acceleration distance and deceleration distance, and whether we can pause after this move
// This may cause a move that we intended to be a deceleration-only move to have a tiny acceleration segment at the start
void DDA::RecalculateMove(DDARing& ring) noexcept
{
	const float twoA = 2 * acceleration;
	const float twoD = 2 * deceleration;
	beforePrepare.accelDistance = (fsquare(requestedSpeed) - fsquare(startSpeed))/twoA;
	beforePrepare.decelDistance = (fsquare(requestedSpeed) - fsquare(endSpeed))/twoD;
	if (beforePrepare.accelDistance + beforePrepare.decelDistance < totalDistance)
	{
		// This move reaches its top speed
		topSpeed = requestedSpeed;
	}
	else
	{
		// This move has no steady-speed phase, so it's accelerate-decelerate or accelerate-only or decelerate-only move.
		// If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - v^2)/2d = dist
		// So V^2(2a + 2d) = 2a.2d.dist + 2a.v^2 + 2d.u^2
		// So V^2 = (2a.2d.dist + 2a.v^2 + 2d.u^2)/(2a + 2d)
		const float vsquared = ((twoA * twoD * totalDistance) + (twoA * fsquare(endSpeed)) + twoD * fsquare(startSpeed))/(twoA + twoD);
		if (vsquared > fsquare(startSpeed) && vsquared > fsquare(endSpeed))
		{
			// It's an accelerate-decelerate move. Calculate accelerate distance from: V^2 = u^2 + 2as.
			beforePrepare.accelDistance = (vsquared - fsquare(startSpeed))/twoA;
			beforePrepare.decelDistance = (vsquared - fsquare(endSpeed))/twoD;
			topSpeed = sqrtf(vsquared);
		}
		else
		{
			// It's an accelerate-only or decelerate-only move.
			// Due to rounding errors and babystepping adjustments, we may have to adjust the acceleration or deceleration slightly.
			if (startSpeed < endSpeed)
			{
				beforePrepare.accelDistance = totalDistance;
				beforePrepare.decelDistance = 0.0;
				topSpeed = endSpeed;
				const float newAcceleration = (fsquare(endSpeed) - fsquare(startSpeed))/(2 * totalDistance);
				if (newAcceleration > 1.02 * acceleration)
				{
					// The acceleration increase is greater than we expect from rounding error, so record an error
					ring.RecordLookaheadError();
					if (reprap.Debug(moduleMove))
					{
						debugPrintf("DDA.cpp(%d) na=%f", __LINE__, (double)newAcceleration);
						DebugPrint("rm");
					}
				}
				acceleration = newAcceleration;
			}
			else
			{
				beforePrepare.accelDistance = 0.0;
				beforePrepare.decelDistance = totalDistance;
				topSpeed = startSpeed;
				const float newDeceleration = (fsquare(startSpeed) - fsquare(endSpeed))/(2 * totalDistance);
				if (newDeceleration > 1.02 * deceleration)
				{
					// The deceleration increase is greater than we expect from rounding error, so record an error
					ring.RecordLookaheadError();
					if (reprap.Debug(moduleMove))
					{
						debugPrintf("DDA.cpp(%d) nd=%f", __LINE__, (double)newDeceleration);
						DebugPrint("rm");
					}
				}
				deceleration = newDeceleration;
			}
		}
	}

	if (flags.canPauseAfter && endSpeed != 0.0)
	{
		const Platform& p = reprap.GetPlatform();
		for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
		{
			if (endSpeed * fabsf(directionVector[drive]) > p.GetInstantDv(drive))
			{
				flags.canPauseAfter = false;
				break;
			}
		}
	}

	// We need to set the number of clocks needed here because we use it before the move has been frozen
	const float totalTime = (topSpeed - startSpeed)/acceleration
							+ (topSpeed - endSpeed)/deceleration
							+ (totalDistance - beforePrepare.accelDistance - beforePrepare.decelDistance)/topSpeed;
	clocksNeeded = (uint32_t)(totalTime * StepTimer::StepClockRate);
}

// Decide what speed we would really like this move to end at.
// On entry, targetNextSpeed is the speed we would like the next move after this one to start at and this one to end at
// On return, targetNextSpeed is the actual speed we can achieve without exceeding the jerk limits.
void DDA::MatchSpeeds() noexcept
{
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
	{
		if (directionVector[drive] != 0.0 || next->directionVector[drive] != 0.0)
		{
			const float totalFraction = fabsf(directionVector[drive] - next->directionVector[drive]);
			const float jerk = totalFraction * beforePrepare.targetNextSpeed;
			const float allowedJerk = reprap.GetPlatform().GetInstantDv(drive);
			if (jerk > allowedJerk)
			{
				beforePrepare.targetNextSpeed = allowedJerk/totalFraction;
			}
		}
	}
}

// This is called by Move::CurrentMoveCompleted to update the live coordinates from the move that has just finished
bool DDA::FetchEndPosition(volatile int32_t ep[MaxAxesPlusExtruders], volatile float endCoords[MaxAxesPlusExtruders]) noexcept
{
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
	{
		ep[drive] = endPoint[drive];
	}
	if (flags.endCoordinatesValid)
	{
		const size_t visibleAxes = reprap.GetGCodes().GetVisibleAxes();
		for (size_t axis = 0; axis < visibleAxes; ++axis)
		{
			endCoords[axis] = endCoordinates[axis];
		}
	}

	// Extrusion amounts are always valid
	const size_t numExtruders = reprap.GetGCodes().GetNumExtruders();
	for (size_t extruder = 0; extruder < numExtruders; ++extruder)
	{
		endCoords[ExtruderToLogicalDrive(extruder)] += endCoordinates[ExtruderToLogicalDrive(extruder)];
	}

	return flags.endCoordinatesValid;
}

// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered
void DDA::SetPositions(const float move[MaxAxesPlusExtruders], size_t numDrives) noexcept
{
	reprap.GetMove().EndPointToMachine(move, endPoint, numDrives);
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		endCoordinates[axis] = move[axis];
	}
	flags.endCoordinatesValid = true;
}

// Get a Cartesian end coordinate from this move
float DDA::GetEndCoordinate(size_t drive, bool disableMotorMapping) noexcept
pre(disableDeltaMapping || drive < MaxAxes)
{
	if (disableMotorMapping)
	{
		return Move::MotorStepsToMovement(drive, endPoint[drive]);
	}
	else
	{
		const size_t visibleAxes = reprap.GetGCodes().GetVisibleAxes();
		if (drive < visibleAxes && !flags.endCoordinatesValid)
		{
			reprap.GetMove().MotorStepsToCartesian(endPoint, visibleAxes, reprap.GetGCodes().GetTotalAxes(), endCoordinates);
			flags.endCoordinatesValid = true;
		}
		return endCoordinates[drive];
	}
}

// Adjust the acceleration and deceleration to reduce ringing
// Only called if topSpeed > startSpeed & topSpeed > endSpeed
// This is only called once, so inlined for speed
inline void DDA::AdjustAcceleration() noexcept
{
	// Try to reduce the acceleration/deceleration of the move to cancel ringing
	const float idealPeriod = reprap.GetMove().GetDRCperiod();

	float proposedAcceleration = acceleration, proposedAccelDistance = beforePrepare.accelDistance;
	bool adjustAcceleration = false;
	if ((prev->state != DDAState::frozen && prev->state != DDAState::executing) || !prev->IsAccelerationMove())
	{
		const float accelTime = (topSpeed - startSpeed)/acceleration;
		if (accelTime < idealPeriod)
		{
			proposedAcceleration = (topSpeed - startSpeed)/idealPeriod;
			adjustAcceleration = true;
		}
		else if (accelTime < idealPeriod * 2)
		{
			proposedAcceleration = (topSpeed - startSpeed)/(idealPeriod * 2);
			adjustAcceleration = true;
		}
		if (adjustAcceleration)
		{
			proposedAccelDistance = (fsquare(topSpeed) - fsquare(startSpeed))/(2 * proposedAcceleration);
		}
	}

	float proposedDeceleration = deceleration, proposedDecelDistance = beforePrepare.decelDistance;
	bool adjustDeceleration = false;
	if (next->state != DDAState::provisional || !next->IsDecelerationMove())
	{
		const float decelTime = (topSpeed - endSpeed)/deceleration;
		if (decelTime < idealPeriod)
		{
			proposedDeceleration = (topSpeed - endSpeed)/idealPeriod;
			adjustDeceleration = true;
		}
		else if (decelTime < idealPeriod * 2)
		{
			proposedDeceleration = (topSpeed - endSpeed)/(idealPeriod * 2);
			adjustDeceleration = true;
		}
		if (adjustDeceleration)
		{
			proposedDecelDistance = (fsquare(topSpeed) - fsquare(endSpeed))/(2 * proposedDeceleration);
		}
	}

	if (adjustAcceleration || adjustDeceleration)
	{
		const float drcMinimumAcceleration = reprap.GetMove().GetDRCminimumAcceleration();
		if (proposedAccelDistance + proposedDecelDistance <= totalDistance)
		{
			if (proposedAcceleration < drcMinimumAcceleration || proposedDeceleration < drcMinimumAcceleration)
			{
				return;
			}
			acceleration = proposedAcceleration;
			deceleration = proposedDeceleration;
			beforePrepare.accelDistance = proposedAccelDistance;
			beforePrepare.decelDistance = proposedDecelDistance;
		}
		else
		{
			// We can't keep this as a trapezoidal move with the original top speed.
			// Try an accelerate-decelerate move with acceleration and deceleration times equal to the ideal period.
			const float twiceTotalDistance = 2 * totalDistance;
			float proposedTopSpeed = totalDistance/idealPeriod - (startSpeed + endSpeed)/2;
			if (proposedTopSpeed > startSpeed && proposedTopSpeed > endSpeed)
			{
				proposedAcceleration = (twiceTotalDistance - ((3 * startSpeed + endSpeed) * idealPeriod))/(2 * fsquare(idealPeriod));
				proposedDeceleration = (twiceTotalDistance - ((startSpeed + 3 * endSpeed) * idealPeriod))/(2 * fsquare(idealPeriod));
				if (   proposedAcceleration < drcMinimumAcceleration || proposedDeceleration < drcMinimumAcceleration
					|| proposedAcceleration > acceleration || proposedDeceleration > deceleration
				   )
				{
					return;
				}
				topSpeed = proposedTopSpeed;
				acceleration = proposedAcceleration;
				deceleration = proposedDeceleration;
				beforePrepare.accelDistance = startSpeed * idealPeriod + (acceleration * fsquare(idealPeriod))/2;
				beforePrepare.decelDistance = endSpeed * idealPeriod + (deceleration * fsquare(idealPeriod))/2;
			}
			else if (startSpeed < endSpeed)
			{
				// Change it into an accelerate-only move, accelerating as slowly as we can
				proposedAcceleration = (fsquare(endSpeed) - fsquare(startSpeed))/twiceTotalDistance;
				if (proposedAcceleration < drcMinimumAcceleration)
				{
					return;		// avoid very small accelerations because they can be problematic
				}
				acceleration = proposedAcceleration;
				topSpeed = endSpeed;
				beforePrepare.accelDistance = totalDistance;
				beforePrepare.decelDistance = 0.0;
			}
			else if (startSpeed > endSpeed)
			{
				// Change it into a decelerate-only move, decelerating as slowly as we can
				proposedDeceleration = (fsquare(startSpeed) - fsquare(endSpeed))/twiceTotalDistance;
				if (proposedDeceleration < drcMinimumAcceleration)
				{
					return;		// avoid very small accelerations because they can be problematic
				}
				deceleration = proposedDeceleration;
				topSpeed = startSpeed;
				beforePrepare.accelDistance = 0.0;
				beforePrepare.decelDistance = totalDistance;
			}
			else
			{
				// Start and end speeds are exactly the same, possibly zero, so give up trying to adjust this move
				return;
			}
		}

		const float totalTime =   (topSpeed - startSpeed)/acceleration
								+ (topSpeed - endSpeed)/deceleration
								+ (totalDistance - beforePrepare.accelDistance - beforePrepare.decelDistance)/topSpeed;
		clocksNeeded = (uint32_t)(totalTime * StepTimer::StepClockRate);
		if (reprap.Debug(moduleMove))
		{
			debugPrintf("New a=%.1f d=%.1f\n", (double)acceleration, (double)deceleration);
		}
	}
}

// Prepare this DDA for execution.
// This must not be called with interrupts disabled, because it calls Platform::EnableDrive.
void DDA::Prepare(uint8_t simMode, float extrusionPending[]) noexcept
{
	if (   flags.xyMoving
		&& reprap.GetMove().IsDRCenabled()
		&& topSpeed > startSpeed && topSpeed > endSpeed
		&& (fabsf(directionVector[X_AXIS]) > 0.5 || fabsf(directionVector[Y_AXIS]) > 0.5)
	   )
	{
		AdjustAcceleration();
	}

#if SUPPORT_LASER
	if (topSpeed < requestedSpeed && reprap.GetGCodes().GetMachineType() == MachineType::laser)
	{
		// Scale back the laser power according to the actual speed
		laserPwmOrIoBits.laserPwm = (laserPwmOrIoBits.laserPwm * topSpeed)/requestedSpeed;
	}
#endif

	PrepParams params;
	params.accelDistance = beforePrepare.accelDistance;
	params.decelDistance = beforePrepare.decelDistance;
	params.decelStartDistance = totalDistance - beforePrepare.decelDistance;

	if (simMode == 0)
	{
		if (flags.isDeltaMovement)
		{
			// This code assumes that the previous move in the DDA ring is the previously-executed move, because it fetches the X and Y end coordinates from that move.
			// Therefore the Move code must not store a new move in that entry until this one has been prepared! (It took me ages to track this down.)
			// Ideally we would store the initial X and Y coordinates in the DDA, but we need to be economical with memory in the Duet 06/085 build.
			afterPrepare.cKc = roundS32(directionVector[Z_AXIS] * DriveMovement::Kc);
			params.a2plusb2 = fsquare(directionVector[X_AXIS]) + fsquare(directionVector[Y_AXIS]);
			params.initialX = prev->GetEndCoordinate(X_AXIS, false);
			params.initialY = prev->GetEndCoordinate(Y_AXIS, false);
#if SUPPORT_CAN_EXPANSION
			params.finalX = GetEndCoordinate(X_AXIS, false);
			params.finalY = GetEndCoordinate(Y_AXIS, false);
			params.zMovement = GetEndCoordinate(Z_AXIS, false) - prev->GetEndCoordinate(Z_AXIS, false);
#endif
			params.dparams = static_cast<const LinearDeltaKinematics*>(&(reprap.GetMove().GetKinematics()));
		}

		// Convert the accelerate/decelerate distances to times
		const float accelStopTime = (topSpeed - startSpeed)/acceleration;
		const float steadyTime = (params.decelStartDistance - beforePrepare.accelDistance)/topSpeed;
#if SUPPORT_CAN_EXPANSION
		params.accelTime = accelStopTime;
		params.steadyTime = steadyTime;
		params.decelTime = (topSpeed - endSpeed)/acceleration;
		params.initialSpeedFraction = startSpeed/topSpeed;
		params.finalSpeedFraction = endSpeed/topSpeed;
		params.compFactor = 1.0 - params.initialSpeedFraction;
#else
		params.compFactor = (topSpeed - startSpeed)/topSpeed;
#endif
		const float decelStartTime = accelStopTime + steadyTime;
		afterPrepare.startSpeedTimesCdivA = (uint32_t)roundU32((startSpeed * StepTimer::StepClockRate)/acceleration);
		params.topSpeedTimesCdivD = (uint32_t)roundU32((topSpeed * StepTimer::StepClockRate)/deceleration);
		afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks = params.topSpeedTimesCdivD + (uint32_t)roundU32(decelStartTime * StepTimer::StepClockRate);
		afterPrepare.extraAccelerationClocks = roundS32((accelStopTime - (beforePrepare.accelDistance/topSpeed)) * StepTimer::StepClockRate);

		activeDMs = completedDMs = nullptr;

#if SUPPORT_CAN_EXPANSION
		CanMotion::StartMovement(*this);
#endif

		// Handle all drivers
		Platform& platform = reprap.GetPlatform();
		if (flags.isLeadscrewAdjustmentMove)
		{
			platform.EnableLocalDrivers(Z_AXIS);			// ensure all Z motors are enabled
		}

		AxesBitmap additionalAxisMotorsToEnable, axisMotorsEnabled;
#if SUPPORT_CAN_EXPANSION
		afterPrepare.drivesMoving.Clear();
#endif
		for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
		{
			if (flags.isLeadscrewAdjustmentMove)
			{
#if SUPPORT_CAN_EXPANSION
				afterPrepare.drivesMoving.SetBit(Z_AXIS);
#endif
				// For a leadscrew adjustment move, the first N elements of the direction vector are the adjustments to the N Z motors
				const AxisDriversConfig& config = platform.GetAxisDriversConfig(Z_AXIS);
				if (drive < config.numDrivers)
				{
					const int32_t delta = lrintf(directionVector[drive] * totalDistance * platform.DriveStepsPerUnit(Z_AXIS));
					const DriverId driver = config.driverNumbers[drive];
#if SUPPORT_CAN_EXPANSION
					if (driver.IsRemote())
					{
						CanMotion::AddMovement(*this, params, driver, delta, false);
					}
					else
#endif
					{
						if (delta != 0)
						{
							DriveMovement* const pdm = DriveMovement::Allocate(driver.localDriver + MaxAxesPlusExtruders, DMState::moving);
							pdm->totalSteps = labs(delta);
							pdm->direction = (delta >= 0);
							if (pdm->PrepareCartesianAxis(*this, params))
							{
								// Check for sensible values, print them if they look dubious
								if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
								{
									DebugPrintAll("pr");
								}
								InsertDM(pdm);
							}
							else
							{
								pdm->state = DMState::idle;
								pdm->nextDM = completedDMs;
								completedDMs = pdm;
							}
						}
					}
				}
			}
			else if (flags.isDeltaMovement && reprap.GetMove().GetKinematics().GetMotionType(drive) == MotionType::segmentFreeDelta)
			{
				// On a delta we need to move all towers even if some of them have no net movement
				const int32_t delta = endPoint[drive] - prev->endPoint[drive];
				if (platform.GetDriversBitmap(drive) != 0)					// if any of the drives is local
				{
					platform.EnableLocalDrivers(drive);
					DriveMovement* const pdm = DriveMovement::Allocate(drive, DMState::moving);
					pdm->totalSteps = labs(delta);
					pdm->direction = (delta >= 0);
					if (pdm->PrepareDeltaAxis(*this, params))
					{
						// Check for sensible values, print them if they look dubious
						if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
						{
							DebugPrintAll("pt");
						}
						InsertDM(pdm);
					}
					else
					{
						pdm->state = DMState::idle;
						pdm->nextDM = completedDMs;
						completedDMs = pdm;
					}
				}

#if SUPPORT_CAN_EXPANSION
				afterPrepare.drivesMoving.SetBit(drive);
				const AxisDriversConfig& config = platform.GetAxisDriversConfig(drive);
				for (size_t i = 0; i < config.numDrivers; ++i)
				{
					const DriverId driver = config.driverNumbers[i];
					if (driver.IsRemote())
					{
						CanMotion::AddMovement(*this, params, driver, delta, false);
					}
				}
#endif
				axisMotorsEnabled.SetBit(drive);
			}
			else if (drive < reprap.GetGCodes().GetTotalAxes())
			{
				// It's a linear drive
				int32_t delta = endPoint[drive] - prev->endPoint[drive];
				if (delta != 0)
				{
					if (flags.continuousRotationShortcut && reprap.GetMove().GetKinematics().IsContinuousRotationAxis(drive))
					{
						// This is a continuous rotation axis, so we may have adjusted the move to cross the 180 degrees position
						const int32_t stepsPerRotation = lrintf(360.0 * platform.DriveStepsPerUnit(drive));
						if (delta > stepsPerRotation/2)
						{
							delta -= stepsPerRotation;
						}
						else if (delta < -stepsPerRotation/2)
						{
							delta += stepsPerRotation;
						}
					}
					if (platform.GetDriversBitmap(drive) != 0)					// if any of the drives is local
					{
						platform.EnableLocalDrivers(drive);
						DriveMovement* const pdm = DriveMovement::Allocate(drive, DMState::moving);
						pdm->totalSteps = labs(delta);
						pdm->direction = (delta >= 0);
						if (pdm->PrepareCartesianAxis(*this, params))
						{
							// Check for sensible values, print them if they look dubious
							if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
							{
								DebugPrintAll("pr");
							}
							InsertDM(pdm);
						}
						else
						{
							pdm->state = DMState::idle;
							pdm->nextDM = completedDMs;
							completedDMs = pdm;
						}
					}

#if SUPPORT_CAN_EXPANSION
					afterPrepare.drivesMoving.SetBit(drive);
					const AxisDriversConfig& config = platform.GetAxisDriversConfig(drive);
					for (size_t i = 0; i < config.numDrivers; ++i)
					{
						const DriverId driver = config.driverNumbers[i];
						if (driver.IsRemote())
						{
							CanMotion::AddMovement(*this, params, driver, delta, false);
						}
					}
#endif
					axisMotorsEnabled.SetBit(drive);
					additionalAxisMotorsToEnable |= reprap.GetMove().GetKinematics().GetConnectedAxes(drive);
				}
			}
			else
			{
				// It's an extruder drive
				if (directionVector[drive] != 0.0)
				{
					// If there is any extruder jerk in this move, in theory that means we need to instantly extrude or retract some amount of filament.
					// Pass the speed change to PrepareExtruder
					float speedChange;
					if (flags.usePressureAdvance)
					{
						const float prevEndSpeed = (prev->flags.usePressureAdvance) ? prev->endSpeed * prev->directionVector[drive] : 0.0;
						speedChange = (startSpeed * directionVector[drive]) - prevEndSpeed;
					}
					else
					{
						speedChange = 0.0;
					}

					const size_t extruder = LogicalDriveToExtruder(drive);
#if SUPPORT_CAN_EXPANSION
					afterPrepare.drivesMoving.SetBit(drive);
					const DriverId driver = platform.GetExtruderDriver(extruder);
					if (driver.IsRemote())
					{
						const int32_t rawSteps = PrepareRemoteExtruder(drive, extrusionPending[extruder], speedChange);
						if (rawSteps != 0)
						{
							CanMotion::AddMovement(*this, params, driver, rawSteps, flags.usePressureAdvance);
						}
					}
					else
#endif
					{
						DriveMovement* const pdm = DriveMovement::Allocate(drive, DMState::moving);
						const bool stepsToDo = pdm->PrepareExtruder(*this, params, extrusionPending[extruder], speedChange, flags.usePressureAdvance);
						platform.EnableLocalDrivers(drive);

						if (stepsToDo)
						{
							// Check for sensible values, print them if they look dubious
							if (   reprap.Debug(moduleDda)
								&& (   pdm->totalSteps > 1000000
									|| pdm->reverseStartStep < pdm->mp.cart.decelStartStep
									|| (   pdm->reverseStartStep <= pdm->totalSteps
										&& pdm->mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD
													> (int64_t)(pdm->mp.cart.twoCsquaredTimesMmPerStepDivD * pdm->reverseStartStep)
									   )
								   )
							   )
							{
								DebugPrintAll("pr");
							}
							InsertDM(pdm);
						}
						else
						{
							pdm->state = DMState::idle;
							pdm->nextDM = completedDMs;
							completedDMs = pdm;
						}
					}
				}
			}
		}

		// On CoreXY and similar architectures, we also need to enable the motors controlling any connected axes
		additionalAxisMotorsToEnable &= ~axisMotorsEnabled;
		while (additionalAxisMotorsToEnable.IsNonEmpty())
		{
			const size_t drive = additionalAxisMotorsToEnable.LowestSetBit();
			additionalAxisMotorsToEnable.ClearBit(drive);
			if (platform.GetDriversBitmap(drive) != 0)									// if any of the connected axis drives is local
			{
				platform.EnableLocalDrivers(drive);
			}
#if SUPPORT_CAN_EXPANSION
			const AxisDriversConfig& config = platform.GetAxisDriversConfig(drive);
			for (size_t i = 0; i < config.numDrivers; ++i)
			{
				const DriverId driver = config.driverNumbers[i];
				if (driver.IsRemote())
				{
					CanMotion::AddMovement(*this, params, driver, 0, false);
				}
			}
#endif
		}

		const DDAState st = prev->state;
		afterPrepare.moveStartTime = (st == DDAState::executing || st == DDAState::frozen)
						? prev->afterPrepare.moveStartTime + prev->clocksNeeded			// this move will follow the previous one, so calculate the start time assuming no more hiccups
							: StepTimer::GetTimerTicks() + MovementStartDelayClocks;	// else this move is the first so start it after a short delay

		if (flags.checkEndstops)
		{
			// Before we send movement commands to remote drives, if any endstop switches we are monitoring are already set, make sure we don't start the motors concerned.
			// This is especially important when using CAN-connected motors or endstops, because we rely on receiving "endstop changed" messages.
			// Moves that check endstops are always run as isolated moves, so there can be no move in progress and the endstops must already be primed.
			platform.EnableAllSteppingDrivers();
			CheckEndstops(platform);									// this may modify pending CAN moves, and may set status 'completed'
		}

#if SUPPORT_CAN_EXPANSION
		CanMotion::FinishMovement(afterPrepare.moveStartTime);
#endif
		if (reprap.Debug(moduleDda) && reprap.Debug(moduleMove))		// temp show the prepared DDA if debug enabled for both modules
		{
			DebugPrintAll("pr");
		}

#if DDA_MOVE_DEBUG
		MoveParameters& m = savedMoves[savedMovePointer];
		m.accelDistance = accelDistance;
		m.decelDistance = decelDistance;
		m.steadyDistance = totalDistance - accelDistance - decelDistance;
		m.requestedSpeed = requestedSpeed;
		m.startSpeed = startSpeed;
		m.topSpeed = topSpeed;
		m.endSpeed = endSpeed;
		m.targetNextSpeed = targetNextSpeed;
		m.endstopChecks = endStopsToCheck;
		m.flags = flags;
		savedMovePointer = (savedMovePointer + 1) % NumSavedMoves;
#endif
	}

	if (state != completed)
	{
		state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
	}
}

// Take a unit positive-hyperquadrant vector, and return the factor needed to obtain
// length of the vector as projected to touch box[].
/*static*/ float DDA::VectorBoxIntersection(const float v[], const float box[], size_t dimensions) noexcept
{
	// Generate a vector length that is guaranteed to exceed the size of the box
	const float biggerThanBoxDiagonal = 2.0*Magnitude(box, dimensions);
	float magnitude = biggerThanBoxDiagonal;
	for (size_t d = 0; d < dimensions; d++)
	{
		if (biggerThanBoxDiagonal*v[d] > box[d])
		{
			const float a = box[d]/v[d];
			if (a < magnitude)
			{
				magnitude = a;
			}
		}
	}
	return magnitude;
}

// Normalise a vector with dim1 dimensions so that it is unit in the first dim2 dimensions, and also return its previous magnitude in dim2 dimensions
/*static*/ float DDA::Normalise(float v[], size_t dim1, size_t dim2) noexcept
{
	const float magnitude = Magnitude(v, dim2);
	if (magnitude <= 0.0)
	{
		return 0.0;
	}
	Scale(v, 1.0/magnitude, dim1);
	return magnitude;
}

// Make the direction vector unit-normal in XYZ and return the previous magnitude
float DDA::NormaliseXYZ() noexcept
{
	// First calculate the magnitude of the vector. If there is more than one X or Y axis, take an average of their movements (they should be equal).
	float xMagSquared = 0.0, yMagSquared = 0.0;
	unsigned int numXaxes = 0, numYaxes = 0;
	const AxesBitmap xAxes = Tool::GetXAxes(tool);
	const AxesBitmap yAxes = Tool::GetYAxes(tool);
	for (size_t d = 0; d < MaxAxes; ++d)
	{
		if (xAxes.IsBitSet(d))
		{
			xMagSquared += fsquare(directionVector[d]);
			++numXaxes;
		}
		if (yAxes.IsBitSet(d))
		{
			yMagSquared += fsquare(directionVector[d]);
			++numYaxes;
		}
	}
	if (numXaxes > 1)
	{
		xMagSquared /= numXaxes;
	}
	if (numYaxes > 1)
	{
		yMagSquared /= numYaxes;
	}
	const float magnitude = sqrtf(xMagSquared + yMagSquared + fsquare(directionVector[Z_AXIS]));
	if (magnitude <= 0.0)
	{
		return 0.0;
	}

	// Now normalise it
	Scale(directionVector, 1.0/magnitude, MaxAxesPlusExtruders);
	return magnitude;
}

// Return the magnitude of a vector
/*static*/ float DDA::Magnitude(const float v[], size_t dimensions) noexcept
{
	float magnitude = 0.0;
	for (size_t d = 0; d < dimensions; d++)
	{
		magnitude += v[d]*v[d];
	}
	return sqrtf(magnitude);
}

// Multiply a vector by a scalar
/*static*/ void DDA::Scale(float v[], float scale, size_t dimensions) noexcept
{
	for (size_t d = 0; d < dimensions; d++)
	{
		v[d] *= scale;
	}
}

// Move a vector into the positive hyperquadrant
/*static*/ void DDA::Absolute(float v[], size_t dimensions) noexcept
{
	for (size_t d = 0; d < dimensions; d++)
	{
		v[d] = fabsf(v[d]);
	}
}

// Check the endstops, given that we know that this move checks endstops.
// Either this move is currently executing (DDARing.currentDDA == this) and the state is 'executing', or we have almost finished preparing it and the state is 'provisional'.
void DDA::CheckEndstops(Platform& platform) noexcept
{
	const bool fromPrepare = (state == DDAState::provisional);		// determine this before anything sets the state to 'completed'

	for (;;)
	{
		const EndstopHitDetails hitDetails = platform.GetEndstops().CheckEndstops(flags.goingSlow);
		switch (hitDetails.GetAction())
		{
		case EndstopHitAction::stopAll:
			MoveAborted();											// set the state to completed and recalculate the endpoints
#if SUPPORT_CAN_EXPANSION
			CanMotion::StopAll(fromPrepare);
#endif
			if (hitDetails.isZProbe)
			{
				reprap.GetGCodes().MoveStoppedByZProbe();
			}
			else if (hitDetails.setAxisLow)
			{
				reprap.GetMove().GetKinematics().OnHomingSwitchTriggered(hitDetails.axis, false, platform.GetDriveStepsPerUnit(), *this);
				reprap.GetGCodes().SetAxisIsHomed(hitDetails.axis);
			}
			else if (hitDetails.setAxisHigh)
			{
				reprap.GetMove().GetKinematics().OnHomingSwitchTriggered(hitDetails.axis, true, platform.GetDriveStepsPerUnit(), *this);
				reprap.GetGCodes().SetAxisIsHomed(hitDetails.axis);
			}
			return;

		case EndstopHitAction::stopAxis:
			StopDrive(hitDetails.axis);								// we must stop the drive before we mess with its coordinates
#if SUPPORT_CAN_EXPANSION
			if (state == completed)									// if the call to StopDrive flagged the move as completed
			{
				CanMotion::StopAll(fromPrepare);
			}
			else
			{
				CanMotion::StopAxis(fromPrepare, hitDetails.axis);
			}
#endif
			if (hitDetails.setAxisLow)
			{
				reprap.GetMove().GetKinematics().OnHomingSwitchTriggered(hitDetails.axis, false, platform.GetDriveStepsPerUnit(), *this);
				reprap.GetGCodes().SetAxisIsHomed(hitDetails.axis);
			}
			else if (hitDetails.setAxisHigh)
			{
				reprap.GetMove().GetKinematics().OnHomingSwitchTriggered(hitDetails.axis, true, platform.GetDriveStepsPerUnit(), *this);
				reprap.GetGCodes().SetAxisIsHomed(hitDetails.axis);
			}
			break;

		case EndstopHitAction::stopDriver:
#if SUPPORT_CAN_EXPANSION
			if (hitDetails.driver.IsRemote())
			{
				CanMotion::StopDriver(fromPrepare, hitDetails.driver);
			}
			else
#endif
			{
				platform.DisableSteppingDriver(hitDetails.driver.localDriver);
			}
			if (hitDetails.setAxisLow)
			{
				reprap.GetMove().GetKinematics().OnHomingSwitchTriggered(hitDetails.axis, false, platform.GetDriveStepsPerUnit(), *this);
				reprap.GetGCodes().SetAxisIsHomed(hitDetails.axis);
			}
			else if (hitDetails.setAxisHigh)
			{
				reprap.GetMove().GetKinematics().OnHomingSwitchTriggered(hitDetails.axis, true, platform.GetDriveStepsPerUnit(), *this);
				reprap.GetGCodes().SetAxisIsHomed(hitDetails.axis);
			}
			break;

		case EndstopHitAction::reduceSpeed:
			if (!fromPrepare)										// don't mess with finish times etc. if the move hasn't started yet
			{
				ReduceHomingSpeed();								// must be just close
			}
			return;													// there can't be a higher priority endstop

		default:
			return;
		}
	}

#if DDA_LOG_PROBE_CHANGES
	else if ((endStopsToCheck & LogProbeChanges) != 0)
	{
		switch (platform.GetZProbeResult())
		{
		case EndStopHit::lowHit:
			if (!probeTriggered)
			{
				probeTriggered = true;
				LogProbePosition();
			}
			break;

		case EndStopHit::nearStop:
		case EndStopHit::noStop:
			if (probeTriggered)
			{
				probeTriggered = false;
				LogProbePosition();
			}
			break;

		default:
			break;
		}
	}
#endif
}

// The remaining functions are speed-critical, so use full optimisation
// The GCC optimize pragma appears to be broken, if we try to force O3 optimisation here then functions are never inlined

// Start executing this move, returning true if Step() needs to be called immediately. Must be called with interrupts disabled or basepri >= set interrupt priority, to avoid a race condition.
void DDA::Start(Platform& p, uint32_t tim) noexcept
pre(state == frozen)
{
	if ((int32_t)(tim - afterPrepare.moveStartTime ) > 25)
	{
		afterPrepare.moveStartTime = tim;			// this move is late starting, so record the actual start time
	}
	state = executing;

#if DDA_LOG_PROBE_CHANGES
	if ((endStopsToCheck & LogProbeChanges) != 0)
	{
		numLoggedProbePositions = 0;
		probeTriggered = false;
	}
#endif

	if (activeDMs != nullptr)
	{
		if (!flags.checkEndstops)
		{
			p.EnableAllSteppingDrivers();							// make sure that all drivers are enabled
		}
		const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
		unsigned int extrusions = 0, retractions = 0;				// bitmaps of extruding and retracting drives
		for (const DriveMovement* pdm = activeDMs; pdm != nullptr; pdm = pdm->nextDM)
		{
			const size_t drive = pdm->drive;
			p.SetDirection(drive, pdm->direction);
			if (drive >= numTotalAxes && drive < MaxAxesPlusExtruders)	// if it's an extruder
			{
				const size_t extruder = LogicalDriveToExtruder(drive);
				if (pdm->direction == FORWARDS)
				{
					extrusions |= (1u << extruder);
				}
				else
				{
					retractions |= (1u << extruder);
				}
			}
		}

		bool extruding = false;
		if (extrusions != 0 || retractions != 0)
		{
			// Check for trying to extrude or retract when the hot end temperature is too low
			const unsigned int prohibitedMovements = reprap.GetProhibitedExtruderMovements(extrusions, retractions);
			for (DriveMovement **dmpp = &activeDMs; *dmpp != nullptr; )
			{
				DriveMovement* const dm = *dmpp;
				const size_t drive = dm->drive;
				if (drive >= numTotalAxes && drive < MaxAxesPlusExtruders)
				{
					if ((prohibitedMovements & (1u << LogicalDriveToExtruder(drive))) != 0)
					{
						*dmpp = dm->nextDM;
						dm->nextDM = completedDMs;
						completedDMs = dm;
					}
					else
					{
						extruding = true;
						dmpp = &(dm->nextDM);
					}
				}
				else
				{
					dmpp = &(dm->nextDM);
				}
			}
		}

		if (extruding)
		{
			p.ExtrudeOn();
		}
		else
		{
			p.ExtrudeOff();
		}
	}
}

uint32_t DDA::lastStepLowTime = 0;
uint32_t DDA::lastDirChangeTime = 0;

// Generate the step pulses of internal drivers used by this DDA. Return true if the move is complete and the next move should be started.
void DDA::StepDrivers(Platform& p) noexcept
{
	// 1. Check endstop switches and Z probe if asked. This is not speed critical because fast moves do not use endstops or the Z probe.
	if (flags.checkEndstops)		// if any homing switches or the Z probe is enabled in this move
	{
		CheckEndstops(p);			// call out to a separate function because this may help cache usage in the more common case where we don't call it
		if (state == completed)		// we may have completed the move due to triggering an endstop switch or Z probe
		{
			return;
		}
	}

	uint32_t driversStepping = 0;
	DriveMovement* dm = activeDMs;
	uint32_t now = StepTimer::GetTimerTicks();
	const uint32_t elapsedTime = (now - afterPrepare.moveStartTime) + StepTimer::MinInterruptInterval;
	while (dm != nullptr && elapsedTime >= dm->nextStepTime)		// if the next step is due
	{
		driversStepping |= p.GetDriversBitmap(dm->drive);
		dm = dm->nextDM;
	}

	driversStepping &= p.GetSteppingEnabledDrivers();
	if ((driversStepping & p.GetSlowDriversBitmap()) == 0)			// if not using any slow drivers
	{
		// 3. Step the drivers
		StepPins::StepDriversHigh(driversStepping);					// generate the steps
	}
	else
	{
		// 3. Step the drivers
		uint32_t lastStepPulseTime = lastStepLowTime;
		while (now - lastStepPulseTime < p.GetSlowDriverStepLowClocks() || now - lastDirChangeTime < p.GetSlowDriverDirSetupClocks())
		{
			now = StepTimer::GetTimerTicks();
		}
		StepPins::StepDriversHigh(driversStepping);					// generate the steps
		lastStepPulseTime = StepTimer::GetTimerTicks();

		// 3a. Reset all step pins low. Do this now because some external drivers don't like the direction pins being changed before the end of the step pulse.
		while (StepTimer::GetTimerTicks() - lastStepPulseTime < p.GetSlowDriverStepHighClocks()) {}
		StepPins::StepDriversLow();									// set all step pins low
		lastStepLowTime = StepTimer::GetTimerTicks();
	}

	// 4. Remove those drives from the list, calculate the next step times, update the direction pins where necessary,
	//    and re-insert them so as to keep the list in step-time order.
	//    Note that the call to CalcNextStepTime may change the state of Direction pin.
	DriveMovement *dmToInsert = activeDMs;							// head of the chain we need to re-insert
	activeDMs = dm;													// remove the chain from the list
	while (dmToInsert != dm)										// note that both of these may be nullptr
	{
		const bool hasMoreSteps = (dmToInsert->isDelta)
				? dmToInsert->CalcNextStepTimeDelta(*this, true)
				: dmToInsert->CalcNextStepTimeCartesian(*this, true);
		DriveMovement * const nextToInsert = dmToInsert->nextDM;
		if (hasMoreSteps)
		{
			InsertDM(dmToInsert);
		}
		else
		{
			dmToInsert->nextDM = completedDMs;
			completedDMs = dmToInsert;
		}
		dmToInsert = nextToInsert;
	}

	// 5. Reset all step pins low. We already did this if we are using any external drivers, but doing it again does no harm.
	StepPins::StepDriversLow();										// set all step pins low

	// 6. If there are no more steps to do and the time for the move has nearly expired, flag the move as complete
	if (activeDMs == nullptr && StepTimer::GetTimerTicks() - afterPrepare.moveStartTime + WakeupTime >= clocksNeeded)
	{
		state = completed;
	}
}

// Stop a drive and re-calculate the corresponding endpoint.
// For extruder drivers, we need to be able to calculate how much of the extrusion was completed after calling this.
void DDA::StopDrive(size_t drive) noexcept
{
	DriveMovement* const pdm = FindActiveDM(drive);
	if (pdm != nullptr)
	{
		if (drive < reprap.GetGCodes().GetTotalAxes())
		{
			endPoint[drive] -= pdm->GetNetStepsLeft();
			flags.endCoordinatesValid = false;			// the XYZ position is no longer valid
		}
		DeactivateDM(drive);

#if !SUPPORT_CAN_EXPANSION
		if (activeDMs == nullptr)
		{
			state = completed;
		}
#endif
	}

#if SUPPORT_CAN_EXPANSION
	afterPrepare.drivesMoving.ClearBit(drive);
	if (afterPrepare.drivesMoving.IsEmpty())
	{
		state = completed;
	}
#endif
}

// This is called when we abort a move because we have hit an endstop.
// It stops all drives and adjusts the end points of the current move to account for how far through the move we got.
// The caller must call MoveCompleted at some point after calling this.
void DDA::MoveAborted() noexcept
{
	if (state == executing)
	{
		for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
		{
			StopDrive(drive);
		}
	}
	state = completed;
}

// Return the proportion of the complete multi-segment move that has already been done.
// The move was either not started or was aborted.
float DDA::GetProportionDone(bool moveWasAborted) const noexcept
{
	// Get the proportion of extrusion already done at the start of this segment
	float proportionDoneSoFar = (filePos != noFilePosition && filePos == prev->filePos)
									? prev->proportionDone
										: 0.0;
	if (moveWasAborted)
	{
		// The move was aborted, so subtract how much was done
		if (proportionDone > proportionDoneSoFar)
		{
			int32_t taken = 0, left = 0;
			for (size_t drive = MaxAxes; drive < NumDirectDrivers; ++drive)
			{
				const DriveMovement* const pdm = FindDM(drive);
				if (pdm != nullptr)								// if this extruder is active
				{
					taken += pdm->GetNetStepsTaken();
					left += pdm->GetNetStepsLeft();
				}
			}
			const int32_t total = taken + left;
			if (total > 0)										// if the move has net extrusion
			{
				proportionDoneSoFar += (((proportionDone - proportionDoneSoFar) * taken) + (total/2)) / total;
			}
		}
	}
	return proportionDoneSoFar;
}

// Reduce the speed of this move to the indicated speed.
// This is called from the ISR, so interrupts are disabled and nothing else can mess with us.
// As this is only called for homing moves and with very low speeds, we assume that we don't need acceleration or deceleration phases.
void DDA::ReduceHomingSpeed() noexcept
{
	if (!flags.goingSlow)
	{
		flags.goingSlow = true;

		topSpeed *= (1.0/ProbingSpeedReductionFactor);

		// Adjust extraAccelerationClocks so that step timing will be correct in the steady speed phase at the new speed
		const uint32_t clocksSoFar = StepTimer::GetTimerTicks() - afterPrepare. moveStartTime;
		afterPrepare.extraAccelerationClocks = (afterPrepare.extraAccelerationClocks * (int32_t)ProbingSpeedReductionFactor) - ((int32_t)clocksSoFar * (int32_t)(ProbingSpeedReductionFactor - 1));

		// We also need to adjust the total clocks needed, to prevent step errors being recorded
		if (clocksSoFar < clocksNeeded)
		{
			clocksNeeded += (clocksNeeded - clocksSoFar) * (ProbingSpeedReductionFactor - 1u);
		}

		// Adjust the speed in the DMs
		for (size_t drive = 0; drive < NumDirectDrivers; ++drive)
		{
			DriveMovement* const pdm = FindDM(drive);
			if (pdm != nullptr && pdm->state == DMState::moving)
			{
				pdm->ReduceSpeed(ProbingSpeedReductionFactor);
			}
		}
	}
}

bool DDA::HasStepError() const noexcept
{
#if 0	//debug
	if (hadHiccup)
	{
		return true;			// temporary for debugging DAA
	}
#endif

	for (size_t drive = 0; drive < NumDirectDrivers; ++drive)
	{
		const DriveMovement* const pdm = FindDM(drive);
		if (pdm != nullptr && pdm->state == DMState::stepError)
		{
			return true;
		}
	}
	return false;
}

// Free up this DDA, returning true if the lookahead underrun flag was set
bool DDA::Free() noexcept
{
	ReleaseDMs();
	state = empty;
	return flags.hadLookaheadUnderrun;
}

// Return the number of net steps already taken in this move by a particular drive
int32_t DDA::GetStepsTaken(size_t drive) const noexcept
{
	const DriveMovement * const dmp = FindDM(drive);
	return (dmp != nullptr) ? dmp->GetNetStepsTaken() : 0;
}

void DDA::LimitSpeedAndAcceleration(float maxSpeed, float maxAcceleration) noexcept
{
	if (requestedSpeed > maxSpeed)
	{
		requestedSpeed = maxSpeed;
	}
	if (acceleration > maxAcceleration)
	{
		acceleration = maxAcceleration;
	}
	if (deceleration > maxAcceleration)
	{
		deceleration = maxAcceleration;
	}
}

#if SUPPORT_CAN_EXPANSION

// Prepare a remote extruder, returning the number of steps we are going to do before allowing for pressure advance.
// This replicates some of the functionality that DriveMovement::PrepareExtruder does for local extruder drives.
int32_t DDA::PrepareRemoteExtruder(size_t drive, float& extrusionPending, float speedChange) const noexcept
{
	// Calculate the requested extrusion amount and a few other things
	float extrusionRequired = totalDistance * directionVector[drive];

#if SUPPORT_NONLINEAR_EXTRUSION
	// Add the nonlinear extrusion correction to totalExtrusion
	if (flags.isPrintingMove)
	{
		float a, b, limit;
		if (reprap.GetPlatform().GetExtrusionCoefficients(LogicalDriveToExtruder(drive), a, b, limit))
		{
			const float averageExtrusionSpeed = (extrusionRequired * StepTimer::StepClockRate)/clocksNeeded;
			const float factor = 1.0 + min<float>((averageExtrusionSpeed * a) + (averageExtrusionSpeed * averageExtrusionSpeed * b), limit);
			extrusionRequired *= factor;
		}
	}
#endif

	// Add on any fractional extrusion pending from the previous move
	extrusionRequired += extrusionPending;
	const float rawStepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const int32_t originalSteps = lrintf(extrusionRequired * rawStepsPerMm);
	int32_t netSteps;

	if (flags.usePressureAdvance && extrusionRequired >= 0.0)
	{
		// Calculate the pressure advance parameters
		const float compensationTime = reprap.GetPlatform().GetPressureAdvance(LogicalDriveToExtruder(drive));

		// Calculate the net total extrusion to allow for compensation. It may be negative.
		const float dv = extrusionRequired/totalDistance;
		extrusionRequired += (endSpeed - startSpeed) * compensationTime * dv;
		netSteps = lrintf(extrusionRequired * rawStepsPerMm);
	}
	else
	{
		netSteps = originalSteps;
	}

	extrusionPending = extrusionRequired - (float)netSteps/rawStepsPerMm;
	return originalSteps;
}

#endif

#if SUPPORT_LASER

// Manage the laser power. Return the number of ticks until we should be called again, or 0 to be called at the start of the next move.
uint32_t DDA::ManageLaserPower() const noexcept
{
	if (!flags.controlLaser || laserPwmOrIoBits.laserPwm == 0)
	{
		reprap.GetPlatform().SetLaserPwm(0);
		return 0;
	}

	const uint32_t clocksMoving = StepTimer::GetTimerTicks() - afterPrepare.moveStartTime;
	if (clocksMoving >= clocksNeeded)			// this also covers the case of now < startTime
	{
		// Something has gone wrong with the timing. Set zero laser power, but try again soon.
		reprap.GetPlatform().SetLaserPwm(0);
		return LaserPwmIntervalMillis;
	}

	const float timeMoving = (float)clocksMoving * (1.0/(float)StepTimer::StepClockRate);
	const float accelSpeed = startSpeed + acceleration * timeMoving;
	if (accelSpeed < topSpeed)
	{
		// Acceleration phase
		const Pwm_t pwm = (Pwm_t)((accelSpeed/topSpeed) * laserPwmOrIoBits.laserPwm);
		reprap.GetPlatform().SetLaserPwm(pwm);
		return LaserPwmIntervalMillis;
	}

	const uint32_t clocksLeft = clocksNeeded - clocksMoving;
	const float decelSpeed = endSpeed + deceleration * (float)clocksLeft * (1.0/(float)StepTimer::StepClockRate);
	if (decelSpeed < topSpeed)
	{
		// Deceleration phase
		const Pwm_t pwm = (Pwm_t)((decelSpeed/topSpeed) * laserPwmOrIoBits.laserPwm);
		reprap.GetPlatform().SetLaserPwm(pwm);
		return LaserPwmIntervalMillis;
	}

	// We must be in the constant speed phase
	reprap.GetPlatform().SetLaserPwm(laserPwmOrIoBits.laserPwm);
	const uint32_t decelClocks = ((topSpeed - endSpeed)/deceleration) * StepTimer::StepClockRate;
	if (clocksLeft <= decelClocks)
	{
		return LaserPwmIntervalMillis;
	}
	const uint32_t clocksToDecel = clocksLeft - decelClocks;
	return lrintf((float)clocksToDecel * StepTimer::StepClocksToMillis) + LaserPwmIntervalMillis;
}

#endif

// End
