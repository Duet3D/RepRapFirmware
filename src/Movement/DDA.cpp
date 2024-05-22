/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "DDA.h"
#include "MoveDebugFlags.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include "Move.h"
#include "StepTimer.h"
#include <Endstops/EndstopsManager.h>
#include "Kinematics/LinearDeltaKinematics.h"
#include <Tools/Tool.h>

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanMotion.h>
# include <CAN/CanInterface.h>
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

// Set up the parameters from the DDA, excluding steadyClocks because that may be affected by input shaping
void PrepParams::SetFromDDA(const DDA& dda) noexcept
{
	totalDistance = dda.totalDistance;
	decelStartDistance = dda.totalDistance - dda.beforePrepare.decelDistance;
	// Due to rounding error, for an accelerate-decelerate move we may have accelDistance+decelDistance slightly greater than totalDistance.
	// We need to make sure that accelDistance <= decelStartDistance for subsequent calculations to work.
	accelDistance = min<float>(dda.beforePrepare.accelDistance, decelStartDistance);
	acceleration = dda.acceleration;
	deceleration = dda.deceleration;
	accelClocks = (dda.topSpeed - dda.startSpeed)/dda.acceleration;
	decelClocks = (dda.topSpeed - dda.endSpeed)/dda.deceleration;
	topSpeed = dda.topSpeed;
	modified = false;
}

// Calculate the steady clocks and set the total clocks in the DDA
void PrepParams::Finalise() noexcept
{
	const float steadyDistance = decelStartDistance - accelDistance;
	steadyClocks = (steadyDistance <= 0.0) ? 0.0 : steadyDistance/topSpeed;
}

DDA::DDA(DDA* n) noexcept : next(n), prev(nullptr), state(empty)
{
	activeDMs = completedDMs = nullptr;
	segments = nullptr;
	tool = nullptr;						// needed in case we pause before any moves have been done

	// Set the endpoints to zero, because Move will ask for them.
	// They will be wrong if we are on a delta. We take care of that when we process the M665 command in config.g.
	for (int32_t& ep : endPoint)
	{
		ep = 0;
	}

	flags.all = 0;						// in particular we need to set endCoordinatesValid and usePressureAdvance to false, also checkEndstops false for the ATE build
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
		DriveMovement* const dnext = dm->nextDM;
		DriveMovement::Release(dm);
		dm = dnext;
	}
	for (DriveMovement* dm = completedDMs; dm != nullptr; )
	{
		DriveMovement* const dnext = dm->nextDM;
		DriveMovement::Release(dm);
		dm = dnext;
	}
	activeDMs = completedDMs = nullptr;
	ReleaseSegments();
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
	debugPrintf("%s %u ts=%" PRIu32 " DDA:", tag, (unsigned int)state, afterPrepare.moveStartTime);
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

	debugPrintf(" s=%.4e", (double)totalDistance);
#if 1
	DebugPrintVector(" vec", directionVector, numAxes);
	DebugPrintVector(" ext", directionVector + (MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders()), reprap.GetGCodes().GetNumExtruders());
#else
	DebugPrintVector(" vec", directionVector, MaxAxesPlusExtruders);
#endif
	debugPrintf("\n" "a=%.4e d=%.4e reqv=%.4e startv=%.4e topv=%.4e endv=%.4e cks=%" PRIu32 " fp=%" PRIu32 " fl=%04x\n",
				(double)acceleration, (double)deceleration, (double)requestedSpeed, (double)startSpeed, (double)topSpeed, (double)endSpeed, clocksNeeded, (uint32_t)filePos, flags.all);
	MoveSegment::DebugPrintList('S', segments);
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

	flags.all = 0;														// set all flags false

	// 1. Compute the new endpoints and the movement vector
	const Move& move = reprap.GetMove();
	if (doMotorMapping)
	{
		if (!move.CartesianToMotorSteps(nextMove.coords, endPoint, nextMove.isCoordinated))		// transform the axis coordinates if on a delta or CoreXY printer
		{
			return false;												// throw away the move if it couldn't be transformed
		}
#if SUPPORT_LINEAR_DELTA
		flags.isDeltaMovement = move.IsDeltaMode()
							&& (endPoint[X_AXIS] != positionNow[X_AXIS] || endPoint[Y_AXIS] != positionNow[Y_AXIS] || endPoint[Z_AXIS] != positionNow[Z_AXIS]);
#endif
	}

	bool linearAxesMoving = false;
	bool rotationalAxesMoving = false;
	bool extrudersMoving = false;
	bool forwardExtruding = false;
	float accelerations[MaxAxesPlusExtruders];

	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		accelerations[drive] = reprap.GetPlatform().Acceleration(drive, nextMove.reduceAcceleration);

		if (drive < numVisibleAxes)
		{
			if (doMotorMapping)
			{
				endCoordinates[drive] = nextMove.coords[drive];
				const float positionDelta = endCoordinates[drive] - prev->GetEndCoordinate(drive, false);
				directionVector[drive] = positionDelta;
				if (positionDelta != 0.0)
				{
					if (reprap.GetPlatform().IsAxisRotational(drive) && nextMove.rotationalAxesMentioned)
					{
						rotationalAxesMoving = true;
					}
					else if (nextMove.linearAxesMentioned)
					{
						linearAxesMoving = true;
					}
					if (Tool::GetXAxes(nextMove.movementTool).IsBitSet(drive) || Tool::GetYAxes(nextMove.movementTool).IsBitSet(drive))
					{
						flags.xyMoving = true;				// this move has XY movement in user space, before axis were mapped
					}
				}
			}
			else
			{
				// Raw motor move on a visible axis
				endPoint[drive] = Move::MotorMovementToSteps(drive, nextMove.coords[drive]);
				const int32_t delta = endPoint[drive] - positionNow[drive];
				directionVector[drive] = (float)delta/reprap.GetPlatform().DriveStepsPerUnit(drive);
				if (delta != 0)
				{
					if (reprap.GetPlatform().IsAxisRotational(drive))
					{
						rotationalAxesMoving = true;
					}
					else
					{
						linearAxesMoving = true;
					}
				}
			}
		}
		else if (LogicalDriveToExtruder(drive) < reprap.GetGCodes().GetNumExtruders())
		{
			// It's an extruder drive. We defer calculating the steps because they may be affected by nonlinear extrusion, which we can't calculate until we
			// know the speed of the move, and because extruder movement is relative so we need to accumulate fractions of a whole step between moves.
			const float movement = nextMove.coords[drive];
			endCoordinates[drive] = directionVector[drive] = movement;			// for an extruder, endCoordinates is the amount of movement
			if (movement != 0.0)
			{
				extrudersMoving = true;
				if (movement > 0.0)
				{
					forwardExtruding = true;
				}
				if (flags.xyMoving && nextMove.usePressureAdvance)
				{
					const float compensationClocks = reprap.GetMove().GetPressureAdvanceClocks(LogicalDriveToExtruder(drive));
					if (compensationClocks > 0.0)
					{
						// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
						accelerations[drive] = min<float>(accelerations[drive], reprap.GetPlatform().GetInstantDv(drive)/compensationClocks);
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
	if (!(linearAxesMoving || rotationalAxesMoving || extrudersMoving))
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
	tool = nextMove.movementTool;
	filePos = nextMove.filePos;
	virtualExtruderPosition = nextMove.moveStartVirtualExtruderPosition;
	proportionDone = nextMove.proportionDone;
	initialUserC0 = nextMove.initialUserC0;
	initialUserC1 = nextMove.initialUserC1;

	flags.checkEndstops = nextMove.checkEndstops;
	flags.canPauseAfter = nextMove.canPauseAfter;
	flags.usingStandardFeedrate = nextMove.usingStandardFeedrate;
	flags.isPrintingMove = flags.xyMoving && forwardExtruding;					// require forward extrusion so that wipe-while-retracting doesn't count
	flags.isNonPrintingExtruderMove = extrudersMoving && !flags.isPrintingMove;	// flag used by filament monitors - we can ignore Z movement
	flags.usePressureAdvance = nextMove.usePressureAdvance;
#if SUPPORT_SCANNING_PROBES
	flags.scanningProbeMove = nextMove.scanningProbeMove;
#endif
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

	// 4. Normalise the direction vector and compute the amount of motion.
	// NIST standard section 2.1.2.5 rule A: if any of XYZ is moving then the feed rate specifies the linear XYZ movement
	// We treat additional linear axes the same as XYZ
	const Kinematics& k = move.GetKinematics();
	if (linearAxesMoving)
	{
		// There is some linear axis movement, so normalise the direction vector so that the total linear movement has unit length and 'totalDistance' is the linear distance moved.
		// This means that the user gets the feed rate that he asked for. It also makes the delta calculations simpler.
		// First do the bed tilt compensation for deltas.
		directionVector[Z_AXIS] += (directionVector[X_AXIS] * k.GetTiltCorrection(X_AXIS)) + (directionVector[Y_AXIS] * k.GetTiltCorrection(Y_AXIS));
		totalDistance = NormaliseLinearMotion(reprap.GetPlatform().GetLinearAxes());
	}
	else if (rotationalAxesMoving)
	{
		// Some axes are moving, but not axes that X or Y are mapped to. Normalise the movement to the vector sum of the axes that are moving.
		totalDistance = Normalise(directionVector, reprap.GetPlatform().GetRotationalAxes());
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
			Scale(directionVector, 1.0/totalDistance);
		}
	}

	// 5. Compute the maximum acceleration available
	float normalisedDirectionVector[MaxAxesPlusExtruders];			// used to hold a unit-length vector in the direction of motion
	memcpyf(normalisedDirectionVector, directionVector, ARRAY_SIZE(normalisedDirectionVector));
	Absolute(normalisedDirectionVector, MaxAxesPlusExtruders);
	acceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations);
	if (flags.xyMoving)												// apply M204 acceleration limits to XY moves
	{
		acceleration = min<float>(acceleration, (flags.isPrintingMove) ? nextMove.maxPrintingAcceleration : nextMove.maxTravelAcceleration);
	}
	deceleration = acceleration;

	// 6. Set the speed to the smaller of the requested and maximum speed.
	// Also enforce a minimum speed of 0.5mm/sec. We need a minimum speed to avoid overflow in the movement calculations.
	float reqSpeed = (nextMove.inverseTimeMode) ? totalDistance/nextMove.feedRate : nextMove.feedRate;
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
		if (maxDistance != 0.0)				// should be true if we are homing a delta
		{
			reqSpeed /= maxDistance;		// because normalisedDirectionVector is unit-normalised
		}
	}

	// Don't use the constrain function in the following, because if we have a very small XY movement and a lot of extrusion, we may have to make the
	// speed lower than the configured minimum movement speed. We must apply the minimum speed first and then limit it if necessary after that.
	requestedSpeed = min<float>(max<float>(reqSpeed, reprap.GetPlatform().MinMovementSpeed()),
								VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform().MaxFeedrates()));

	// On a Cartesian printer, it is OK to limit the X and Y speeds and accelerations independently, and in consequence to allow greater values
	// for diagonal moves. On other architectures, this is not OK and any movement in the XY plane should be limited on other ways.
	if (doMotorMapping)
	{
		k.LimitSpeedAndAcceleration(*this, normalisedDirectionVector, numVisibleAxes, flags.continuousRotationShortcut);	// give the kinematics the chance to further restrict the speed and acceleration
	}

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	endSpeed = 0.0;							// until the next move asks us to adjust it

	if (   prev->state == provisional
		&& (   move.GetJerkPolicy() != 0
			|| (   flags.isPrintingMove == prev->flags.isPrintingMove
				&& flags.xyMoving == prev->flags.xyMoving
				&& flags.isNonPrintingExtruderMove == prev->flags.isNonPrintingExtruderMove		// this is to prevent extruder-only move being melded with Z-axis moves (issue 990)
			   )
		   )
	   )
	{
		// Try to meld this move to the previous move to avoid stop/start
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		prev->beforePrepare.targetNextSpeed = min<float>(fastSqrtf(deceleration * totalDistance * 2.0), requestedSpeed);
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
	flags.all = 0;
	flags.isLeadscrewAdjustmentMove = true;
	virtualExtruderPosition = prev->virtualExtruderPosition;
	tool = nullptr;
	filePos = prev->filePos;
	flags.endCoordinatesValid = prev->flags.endCoordinatesValid;
	acceleration = deceleration = reprap.GetPlatform().NormalAcceleration(Z_AXIS);

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
	//    Currently we normalise the vector sum of all Z motor movement to unit length.
	totalDistance = Normalise(directionVector);

	// 6. Set the speed to requested feed rate, which the caller must ensure is no more than the maximum speed for the Z axis.
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
	flags.all = 0;
	virtualExtruderPosition = 0;
	tool = nullptr;
	filePos = noFilePosition;

	startSpeed = nextMove.startSpeed;
	endSpeed = nextMove.endSpeed;
	requestedSpeed = nextMove.requestedSpeed;
	acceleration = nextMove.acceleration;
	deceleration = nextMove.deceleration;

# if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
# endif

	// Currently we normalise the vector sum of all motor movements to unit length.
	totalDistance = Normalise(directionVector);

	RecalculateMove(ring);
	state = provisional;
	return true;
}

#endif

#if SUPPORT_REMOTE_COMMANDS

// Set up a remote move. Return true if it represents real movement, else false.
// All values have already been converted to step clocks and the total distance has been normalised to 1.0.
// This one handles the old format movement message, used by older versions of RRF and the ATE
bool DDA::InitFromRemote(const CanMessageMovementLinear& msg) noexcept
{
	afterPrepare.moveStartTime = StepTimer::ConvertToLocalTime(msg.whenToExecute);
	clocksNeeded = msg.accelerationClocks + msg.steadyClocks + msg.decelClocks;
	flags.all = 0;
	flags.isRemote = true;
	flags.isPrintingMove = flags.usePressureAdvance = (msg.pressureAdvanceDrives != 0);

	// Normalise the move to unit distance
	totalDistance = 1.0;

	// Calculate the speeds and accelerations assuming unit movement length
	topSpeed = 2.0/(2 * msg.steadyClocks + (msg.initialSpeedFraction + 1.0) * msg.accelerationClocks + (msg.finalSpeedFraction + 1.0) * msg.decelClocks);
	startSpeed = topSpeed * msg.initialSpeedFraction;
	endSpeed = topSpeed * msg.finalSpeedFraction;

	acceleration = (msg.accelerationClocks == 0) ? 0.0 : (topSpeed * (1.0 - msg.initialSpeedFraction))/msg.accelerationClocks;
	deceleration = (msg.decelClocks == 0) ? 0.0 : (topSpeed * (1.0 - msg.finalSpeedFraction))/msg.decelClocks;

	// Prepare for movement
	PrepParams params;											// the default constructor clears params.plan to 'no shaping'

	// Set up the move parameters
	params.accelDistance = topSpeed * (1.0 + msg.initialSpeedFraction) * msg.accelerationClocks * 0.5;
	const float decelDistance = topSpeed * (1.0 + msg.finalSpeedFraction) * msg.decelClocks * 0.5;
	params.decelStartDistance = 1.0 - decelDistance;
	params.accelClocks = msg.accelerationClocks;
	params.steadyClocks = msg.steadyClocks;
	params.decelClocks = msg.decelClocks;
	params.acceleration = acceleration;
	params.deceleration = deceleration;

	segments = nullptr;
	activeDMs = completedDMs = nullptr;
	afterPrepare.drivesMoving.Clear();

	for (size_t drive = 0; drive < NumDirectDrivers; drive++)
	{
		endPoint[drive] = prev->endPoint[drive];				// the steps for this move will be added later
		const int32_t delta = (drive < msg.numDrivers) ? msg.perDrive[drive].steps : 0;
		directionVector[drive] = (float)delta;
		if (delta != 0)
		{
			EnsureSegments(params);								// we are going to need segments
			DriveMovement* const pdm = DriveMovement::Allocate(drive);
			pdm->totalSteps = labs(delta);						// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
			pdm->direction = (delta >= 0);						// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
			afterPrepare.drivesMoving.SetBit(drive);
			reprap.GetPlatform().EnableDrivers(drive, false);
			if ((msg.pressureAdvanceDrives & (1u << drive)) != 0)
			{
				pdm->PrepareExtruder(*this, (float)delta);
				pdm->nextDM = completedDMs;						// extruder DMs go in the completed list initially
				completedDMs = pdm;
			}
			else if (pdm->PrepareCartesianAxis(*this))
			{
				InsertDM(pdm);
				const int32_t netSteps = (pdm->reverseStartStep < pdm->totalSteps) ? (2 * pdm->reverseStartStep) - pdm->totalSteps : pdm->totalSteps;
				if (pdm->direction)
				{
					endPoint[drive] += netSteps;
				}
				else
				{
					endPoint[drive] -= netSteps;
				}

				// Check for sensible values, print them if they look dubious
				if (   reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintAllMoves)
					|| (pdm->totalSteps > 1000000 && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
				   )
				{
					DebugPrintAll("remu_err1");
				}
			}
			else
			{
				// No steps to do, so release the DM
				DriveMovement::Release(pdm);
			}
		}
	}

	// 2. Throw it away if there's no real movement.
	if (activeDMs == nullptr)
	{
		// We may have set up the segments, in which case we must recycle them
		ReleaseSegments();
		return false;
	}

	if (   reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintAllMoves)
		|| params.shapingPlan.debugPrint						// show the prepared DDA if debug enabled for both modules
	   )
	{
		DebugPrintAll("remu");
	}

	state = frozen;												// must do this last so that the ISR doesn't start executing it before we have finished setting it up
	return true;
}

// Set up a remote move. Return true if it represents real movement, else false.
// All values have already been converted to step clocks and the total distance has been normalised to 1.0.
// This version handles the new movement message that includes the input shaping plan and passes extruder movement as distance, not steps
bool DDA::InitFromRemote(const CanMessageMovementLinearShaped& msg) noexcept
{
	afterPrepare.moveStartTime = StepTimer::ConvertToLocalTime(msg.whenToExecute);
	flags.all = 0;
	flags.isRemote = true;
	flags.isPrintingMove = flags.usePressureAdvance = msg.usePressureAdvance;
	// TODO For now we treat any non-printing move as a non-printing extruder move. Better to pass a flag for it in the CAN message.
	flags.isNonPrintingExtruderMove = !flags.isPrintingMove;

	// Prepare for movement
	PrepParams params;
	params.shapingPlan.condensedPlan = msg.shapingPlan;

	// Normalise the move to unit distance
	params.totalDistance = totalDistance = 1.0;
	params.acceleration = acceleration = msg.acceleration;
	params.deceleration = deceleration = msg.deceleration;
	params.accelClocks = msg.accelerationClocks;
	params.steadyClocks = msg.steadyClocks;
	params.decelClocks = msg.decelClocks;
	clocksNeeded = msg.accelerationClocks + msg.steadyClocks + msg.decelClocks;

	// We occasionally receive a message with zero clocks needed. This messes up the calculations, so add one steady clock in this case.
	if (clocksNeeded == 0)
	{
		clocksNeeded = params.steadyClocks = 1;
	}

	// Set up the plan
	segments = nullptr;
	reprap.GetMove().GetAxisShaper().GetRemoteSegments(*this, params);

	activeDMs = completedDMs = nullptr;
	afterPrepare.drivesMoving.Clear();

	for (size_t drive = 0; drive < NumDirectDrivers; drive++)
	{
		endPoint[drive] = prev->endPoint[drive];						// the steps for this move will be added later
		if (drive >= msg.numDrivers)
		{
			directionVector[drive] = 0.0;
		}
		else if ((msg.extruderDrives & (1u << drive)) != 0)
		{
			// It's an extruder
			const float extrusionRequested = msg.perDrive[drive].extrusion;
			directionVector[drive] = extrusionRequested;
			if (extrusionRequested != 0.0)
			{
				DriveMovement* const pdm = DriveMovement::Allocate(drive);
				pdm->totalSteps = labs(extrusionRequested);				// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
				pdm->direction = (extrusionRequested >= 0.0);			// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
				afterPrepare.drivesMoving.SetBit(drive);
				reprap.GetPlatform().EnableDrivers(drive, false);
				pdm->PrepareExtruder(*this, extrusionRequested);
				pdm->nextDM = completedDMs;
				completedDMs = pdm;
			}
		}
		else
		{
			const int32_t delta = msg.perDrive[drive].steps;
			directionVector[drive] = (float)delta;
			if (delta != 0)
			{
				DriveMovement* const pdm = DriveMovement::Allocate(drive);
				pdm->totalSteps = labs(delta);					// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
				pdm->direction = (delta >= 0);					// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
				afterPrepare.drivesMoving.SetBit(drive);
				reprap.GetPlatform().EnableDrivers(drive, false);
				const bool stepsToDo = pdm->PrepareCartesianAxis(*this);
				if (stepsToDo)
				{
					InsertDM(pdm);
					const int32_t netSteps = (pdm->reverseStartStep < pdm->totalSteps) ? (2 * pdm->reverseStartStep) - pdm->totalSteps : pdm->totalSteps;
					if (pdm->direction)
					{
						endPoint[drive] += netSteps;
					}
					else
					{
						endPoint[drive] -= netSteps;
					}

					// Check for sensible values, print them if they look dubious
					if (pdm->totalSteps > 1000000 && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
					{
						DebugPrintAll("rems_err2");
					}
				}
				else
				{
					// No steps to do, so release the DM
					DriveMovement::Release(pdm);
				}
			}
		}
	}

	// 2. Throw it away if there's no real movement.
	if (activeDMs == nullptr)
	{
		// We may have set up the segments, in which case we must recycle them
		ReleaseSegments();
		return false;
	}

	if (   reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintAllMoves)
		|| params.shapingPlan.debugPrint
	   )
	{
		DebugPrintAll("rems");
	}

	state = frozen;												// must do this last so that the ISR doesn't start executing it before we have finished setting it up
	return true;
}

void DDA::StopDrivers(uint16_t whichDrives) noexcept
{
	if (state == executing)
	{
		for (size_t drive = 0; drive < NumDirectDrivers; ++drive)
		{
			if (whichDrives & (1u << drive))
			{
				StopDrive(drive);
			}
		}
	}
}

#endif	// SUPPORT_REMOTE_COMMANDS

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
bool DDA::IsDecelerationMove() const noexcept
{
	return beforePrepare.decelDistance == totalDistance					// the simple case - is a deceleration-only move
			|| (topSpeed < requestedSpeed								// can't have been intended as deceleration-only if it reaches the requested speed
				&& beforePrepare.decelDistance > 0.98 * totalDistance	// rounding error can only go so far
			   );
}

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
bool DDA::IsAccelerationMove() const noexcept
{
	return beforePrepare.accelDistance == totalDistance					// the simple case - is an acceleration-only move
			|| (topSpeed < requestedSpeed								// can't have been intended as deceleration-only if it reaches the requested speed
				&& beforePrepare.accelDistance > 0.98 * totalDistance	// rounding error can only go so far
			   );
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
				laDDA->MatchSpeeds();													// adjust it if necessary
				goingUp = false;
			}
			else if (   laDDA->IsDecelerationMove()
					 && laDDA->prev->beforePrepare.decelDistance > 0.0					// if the previous move has no deceleration phase then no point in adjusting it
					)
			{
				const DDAState st = laDDA->prev->state;
				// This is a deceleration-only move, and the previous one has a deceleration phase. We may have to adjust the previous move as well to get optimum behaviour.
				if (   st == provisional
					&& (   reprap.GetMove().GetJerkPolicy() != 0
						|| (   laDDA->prev->flags.xyMoving == laDDA->flags.xyMoving
							&& (   laDDA->prev->flags.isPrintingMove == laDDA->flags.isPrintingMove
								|| (laDDA->prev->flags.isPrintingMove && laDDA->prev->requestedSpeed == laDDA->requestedSpeed)	// special case to support coast-to-end
							   )
						   )
					   )
				   )
				{
					laDDA->MatchSpeeds();
					const float maxStartSpeed = fastSqrtf(fsquare(laDDA->beforePrepare.targetNextSpeed) + (2 * laDDA->deceleration * laDDA->totalDistance));
					laDDA->prev->beforePrepare.targetNextSpeed = min<float>(maxStartSpeed, laDDA->requestedSpeed);
					// leave 'goingUp' true
				}
				else
				{
					// This move is a deceleration-only move but we can't adjust the previous one
					if (st == frozen || st == executing)
					{
						laDDA->flags.hadLookaheadUnderrun = true;
					}
					const float maxReachableSpeed = fastSqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->deceleration * laDDA->totalDistance));
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
				const float maxReachableSpeed = fastSqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
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
			const float maxEndSpeed = fastSqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
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
#if 0
			if (reprap.Debug(moduleDda))
			{
				debugPrintf("Recursion start %u\n", laDepth);
			}
#endif
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
					if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
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
#if 0
				if (reprap.Debug(moduleDda))
				{
					debugPrintf("Complete, %f\n", laDDA->targetNextSpeed);
				}
#endif
				return;
			}

			laDDA = laDDA->next;
			--laDepth;
		}
	}
}

// Try to push babystepping earlier in the move queue, returning the amount we pushed
// Caution! Thus is called with scheduling locked, therefore it must make no FreeRTOS calls, or call anything that makes them
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
			Platform& platform = reprap.GetPlatform();
			const float maxBabySteppingAmount = cdda->totalDistance * min<float>(0.1, 0.5 * platform.GetInstantDv(Z_AXIS)/cdda->topSpeed);
			babySteppingToDo = constrain<float>(amount, -maxBabySteppingAmount, maxBabySteppingAmount);
			cdda->directionVector[Z_AXIS] += babySteppingToDo/cdda->totalDistance;
			cdda->totalDistance *= cdda->NormaliseLinearMotion(platform.GetLinearAxes());
			cdda->RecalculateMove(ring);
			babySteppingDone += babySteppingToDo;
			amount -= babySteppingToDo;
		}

		// Even if there is no babystepping to do this move, we may need to adjust the end coordinates
		cdda->endCoordinates[Z_AXIS] += babySteppingDone;
#if SUPPORT_LINEAR_DELTA
		if (cdda->flags.isDeltaMovement)
		{
			for (size_t motor = 0; motor < reprap.GetGCodes().GetTotalAxes(); ++motor)
			{
				if (reprap.GetMove().GetKinematics().GetMotionType(motor) == MotionType::segmentFreeDelta)
				{
					cdda->endPoint[motor] += (int32_t)(babySteppingDone * reprap.GetPlatform().DriveStepsPerUnit(motor));
				}
			}
		}
		else
#endif
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
		// It sometimes happens that we get a very short acceleration or deceleration segment. Remove any such segments by reducing the top speed to the start or end speed.
		// Don't do this if the cause is that the top speed is very low because that results in issues 989 and 994
		if (startSpeed >= endSpeed)
		{
			if (startSpeed + acceleration * MinimumAccelOrDecelClocks > requestedSpeed && startSpeed >= requestedSpeed * 0.9)
			{
				topSpeed = startSpeed;
				beforePrepare.accelDistance = 0.0;
			}
			else
			{
				topSpeed = requestedSpeed;
			}
		}
		else
		{
			if (endSpeed + deceleration * MinimumAccelOrDecelClocks > requestedSpeed && endSpeed >= requestedSpeed * 0.9)
			{
				topSpeed = endSpeed;
				beforePrepare.decelDistance = 0.0;
			}
			else
			{
				topSpeed = requestedSpeed;
			}
		}
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
			topSpeed = fastSqrtf(vsquared);
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
					if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
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
					if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
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
	clocksNeeded = (uint32_t)totalTime;
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

// This is called by DDARing::LiveCoordinates to get the endpoints of a move that is being executed
void DDA::FetchCurrentPositions(int32_t ep[MaxAxesPlusExtruders]) const noexcept
{
	memcpyi32(ep, prev->endPoint, MaxAxesPlusExtruders);
	if (!flags.isLeadscrewAdjustmentMove)			// driver numbers are out of the usual range for leadscrew adjustment moves
	{
		for (const DriveMovement* dm = activeDMs; dm != nullptr; dm = dm->nextDM)
		{
			ep[dm->drive] += dm->GetNetStepsTaken();
		}
	}
}

// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered
void DDA::SetPositions(const float move[MaxAxesPlusExtruders]) noexcept
{
	(void)reprap.GetMove().CartesianToMotorSteps(move, endPoint, true);
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

// Set up the segments (without input shaping) if we haven't done so already
void DDA::EnsureSegments(const PrepParams& params) noexcept
{
	if (segments == nullptr)
	{
		segments = AxisShaper::GetUnshapedSegments(*this, params);
	}
}

void DDA::ReleaseSegments() noexcept
{
	for (MoveSegment* seg = segments; seg != nullptr; )
	{
		MoveSegment* const nextSeg = seg->GetNext();
		MoveSegment::Release(seg);
		seg = nextSeg;
	}
	segments = nullptr;
}

// Prepare this DDA for execution.
// This must not be called with interrupts disabled, because it calls Platform::EnableDrive.
void DDA::Prepare(SimulationMode simMode) noexcept
{
	flags.wasAccelOnlyMove = IsAccelerationMove();			// save this for the next move to look at

#if SUPPORT_LASER
	if (topSpeed < requestedSpeed && reprap.GetGCodes().GetMachineType() == MachineType::laser)
	{
		// Scale back the laser power according to the actual speed
		laserPwmOrIoBits.laserPwm = (laserPwmOrIoBits.laserPwm * topSpeed)/requestedSpeed;
	}
#endif

	// Prepare for movement
	segments = nullptr;

	PrepParams params;										// the default constructor clears params.plan to 'no shaping'
	if (flags.xyMoving)
	{
		reprap.GetMove().GetAxisShaper().PlanShaping(*this, params, flags.xyMoving);	// this will set up shapedSegments if we are doing any shaping
	}
	else
	{
		params.SetFromDDA(*this);
		params.Finalise();
		clocksNeeded = params.TotalClocks();
	}

	// Copy the unshaped acceleration and deceleration back to the DDA because ManageLaserPower uses them
	//TODO change ManageLaserPower to work on the shaped segments instead
	acceleration = params.acceleration;
	deceleration = params.deceleration;

	if (simMode < SimulationMode::normal)
	{
#if SUPPORT_LINEAR_DELTA
		if (flags.isDeltaMovement)
		{
			// This code assumes that the previous move in the DDA ring is the previously-executed move, because it fetches the X and Y end coordinates from that move.
			// Therefore the Move code must not store a new move in that entry until this one has been prepared! (It took me ages to track this down.)
			// Ideally we would store the initial X and Y coordinates in the DDA, but we need to be economical with memory
			params.a2plusb2 = fsquare(directionVector[X_AXIS]) + fsquare(directionVector[Y_AXIS]);
			params.initialX = prev->GetEndCoordinate(X_AXIS, false);
			params.initialY = prev->GetEndCoordinate(Y_AXIS, false);
			params.dparams = static_cast<const LinearDeltaKinematics*>(&(reprap.GetMove().GetKinematics()));
# if SUPPORT_CAN_EXPANSION
			params.finalX = GetEndCoordinate(X_AXIS, false);
			params.finalY = GetEndCoordinate(Y_AXIS, false);
			params.zMovement = GetEndCoordinate(Z_AXIS, false) - prev->GetEndCoordinate(Z_AXIS, false);
# endif
		}
#endif

		activeDMs = completedDMs = nullptr;

#if SUPPORT_CAN_EXPANSION
		CanMotion::StartMovement();
#endif
		// Handle all drivers
		Platform& platform = reprap.GetPlatform();
		if (flags.isLeadscrewAdjustmentMove)
		{
			platform.EnableDrivers(Z_AXIS, false);			// ensure all Z motors are enabled
		}

		float extrusionFraction = 0.0;
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
					if (delta != 0)
					{
#if SUPPORT_CAN_EXPANSION
						if (driver.IsRemote())
						{
							CanMotion::AddAxisMovement(params, driver, delta);
						}
						else
#endif
						{
							EnsureSegments(params);
							DriveMovement* const pdm = DriveMovement::Allocate(driver.localDriver + MaxAxesPlusExtruders);
							pdm->direction = (delta >= 0);
							pdm->totalSteps = labs(delta);
							if (pdm->PrepareCartesianAxis(*this))
							{
								// Check for sensible values, print them if they look dubious
								if (pdm->totalSteps > 1000000 && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
								{
									DebugPrintAll("pr_err1");
								}
								InsertDM(pdm);
							}
							else
							{
								DriveMovement::Release(pdm);
							}
						}
					}
				}
			}
#if SUPPORT_LINEAR_DELTA
			else if (flags.isDeltaMovement && reprap.GetMove().GetKinematics().GetMotionType(drive) == MotionType::segmentFreeDelta)
			{
				// On a delta we need to move all towers even if some of them have no net movement
				platform.EnableDrivers(drive, false);
				EnsureSegments(params);

				const int32_t delta = endPoint[drive] - prev->endPoint[drive];
# if DDA_DEBUG_STEP_COUNT
				stepsRequested[drive] += delta;
# endif
				if (platform.GetDriversBitmap(drive) != 0						// if any of the drives is local
# if SUPPORT_CAN_EXPANSION
						|| flags.checkEndstops									// if checking endstops, create a DM even if there are no local drives involved
# endif
				   )
				{
					DriveMovement* const pdm = DriveMovement::Allocate(drive);
					pdm->direction = (delta >= 0);
					pdm->totalSteps = labs(delta);								// this is net steps for now
					if (pdm->PrepareDeltaAxis(*this, params))
					{
						// Check for sensible values, print them if they look dubious
						if (pdm->totalSteps > 1000000 && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
						{
							DebugPrintAll("pr_err2");
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

# if SUPPORT_CAN_EXPANSION
				afterPrepare.drivesMoving.SetBit(drive);
				const AxisDriversConfig& config = platform.GetAxisDriversConfig(drive);
				for (size_t i = 0; i < config.numDrivers; ++i)
				{
					const DriverId driver = config.driverNumbers[i];
					if (driver.IsRemote())
					{
						CanMotion::AddAxisMovement(params, driver, delta);
					}
				}
# endif
				axisMotorsEnabled.SetBit(drive);
			}
#endif	// SUPPORT_LINEAR_DELTA
			else if (drive < reprap.GetGCodes().GetTotalAxes())
			{
				// It's a linear axis
				int32_t delta = endPoint[drive] - prev->endPoint[drive];
				if (delta != 0)
				{
#if DDA_DEBUG_STEP_COUNT
					stepsRequested[drive] += delta;
#endif
					platform.EnableDrivers(drive, false);
					EnsureSegments(params);
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

					delta = platform.ApplyBacklashCompensation(drive, delta);

					if (   platform.GetDriversBitmap(drive) != 0				// if any of the drives is local
#if SUPPORT_CAN_EXPANSION
						|| flags.checkEndstops									// if checking endstops or a Z probe, create a DM even if there are no local drives involved
#endif
					   )
					{
						DriveMovement* const pdm = DriveMovement::Allocate(drive);
						pdm->direction = (delta >= 0);
						pdm->totalSteps = labs(delta);
						if (pdm->PrepareCartesianAxis(*this))
						{
							// Check for sensible values, print them if they look dubious
							if (pdm->totalSteps > 1000000 && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
							{
								DebugPrintAll("pr_err3");
							}
							InsertDM(pdm);
						}
						else
						{
							DriveMovement::Release(pdm);
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
							CanMotion::AddAxisMovement(params, driver, delta);
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
					const size_t extruder = LogicalDriveToExtruder(drive);

					// Check for cold extrusion/retraction. Do this now because we can't read temperatures from within the step ISR, also this works for CAN-connected extruders.
					// Don't check if it is a special move (indicated by flags.checkEndstops) because the 'tool' variable isn't valid for those moves
					if (flags.checkEndstops || Tool::ExtruderMovementAllowed(tool, directionVector[drive] > 0, extruder))
					{
						platform.EnableDrivers(drive, false);

						if (directionVector[drive] > 0.0)
						{
							extrusionFraction += directionVector[drive];			// accumulate the total extrusion fraction
						}

#if SUPPORT_NONLINEAR_EXTRUSION
						// Add the nonlinear extrusion correction to totalExtrusion.
						// If we are given a stupidly short move to execute then clocksNeeded can be zero, which leads to NaNs in this code; so we need to guard against that.
						if (flags.isPrintingMove && clocksNeeded != 0)
						{
							const NonlinearExtrusion& nl = platform.GetExtrusionCoefficients(extruder);
							float& dv = directionVector[drive];
							const float averageExtrusionSpeed = (totalDistance * dv * StepClockRate)/clocksNeeded;		// need speed in mm/sec for nonlinear extrusion calculation
							const float factor = 1.0 + min<float>((nl.A + (nl.B * averageExtrusionSpeed)) * averageExtrusionSpeed, nl.limit);
							dv *= factor;
						}
#endif

#if SUPPORT_CAN_EXPANSION
						afterPrepare.drivesMoving.SetBit(drive);
						const DriverId driver = platform.GetExtruderDriver(extruder);
						if (driver.IsRemote())
						{
							// The MovementLinearShaped message requires the extrusion amount in steps to be passed as a float. The remote board adds the PA and handles fractional steps.
							CanMotion::AddExtruderMovement(params, driver, totalDistance * directionVector[drive] * platform.DriveStepsPerUnit(drive), flags.usePressureAdvance);
						}
						else
#endif
						{
							EnsureSegments(params);
							DriveMovement* const pdm = DriveMovement::Allocate(drive);
							pdm->direction = (directionVector[drive] >= 0);
							pdm->PrepareExtruder(*this, platform.DriveStepsPerUnit(drive) * directionVector[drive]);
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
			platform.EnableDrivers(drive, false);
		}

		const DDAState st = prev->state;
		afterPrepare.moveStartTime = (st == DDAState::executing || st == DDAState::frozen)
						? prev->afterPrepare.moveStartTime + prev->clocksNeeded			// this move will follow the previous one, so calculate the start time assuming no more hiccups
							: StepTimer::GetTimerTicks() + AbsoluteMinimumPreparedTime;	// else this move is the first so start it after a short delay

		if (flags.checkEndstops)
		{
			// Before we send movement commands to remote drives, if any endstop switches we are monitoring are already set, make sure we don't start the motors concerned.
			// This is especially important when using CAN-connected motors or endstops, because we rely on receiving "endstop changed" messages.
			// Moves that check endstops are always run as isolated moves, so there can be no move in progress and the endstops must already be primed.
			platform.EnableAllSteppingDrivers();
			(void)CheckEndstops(platform);									// this may modify pending CAN moves, and may set status 'completed'
		}

#if SUPPORT_CAN_EXPANSION
		const uint32_t canClocksNeeded = CanMotion::FinishMovement(*this, afterPrepare.moveStartTime, simMode != SimulationMode::off);
		if (canClocksNeeded > clocksNeeded)
		{
			// Due to rounding error in the calculations, we quite often calculate the CAN move as being longer than our previously-calculated value, normally by just one clock.
			// Extend our move time in this case so that the expansion boards don't need to catch up.
			clocksNeeded = canClocksNeeded;
		}
#endif

		afterPrepare.averageExtrusionSpeed = (extrusionFraction * totalDistance * (float)StepClockRate)/(float)clocksNeeded;

		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintAllMoves) || params.shapingPlan.debugPrint)		// show the prepared DDA if debug enabled for both modules
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
/*static*/ float DDA::VectorBoxIntersection(const float v[], const float box[]) noexcept
{
	// Generate a vector length that is guaranteed to exceed the size of the box
	float magnitude = 0.0;
	for (size_t d = 0; d < MaxAxesPlusExtruders; d++)
	{
		magnitude += box[d];
	}

	// Now reduce the length until every axis fits
	for (size_t d = 0; d < MaxAxesPlusExtruders; d++)
	{
		if (magnitude * v[d] > box[d])
		{
			magnitude = box[d]/v[d];
		}
	}
	return magnitude;
}

// Get the magnitude measured over all axes and extruders
/*static*/ float DDA::Magnitude(const float v[]) noexcept
{
	float magnitudeSquared = 0.0;
	for (size_t d = 0; d < MaxAxesPlusExtruders; d++)
	{
		magnitudeSquared += fsquare(v[d]);
	}
	return fastSqrtf(magnitudeSquared);
}

// Normalise a vector with dim1 dimensions to unit length over the specified axes, and also return its previous magnitude in dim2 dimensions
/*static*/ float DDA::Normalise(float v[], AxesBitmap unitLengthAxes) noexcept
{
	const float magnitude = Magnitude(v, unitLengthAxes);
	if (magnitude <= 0.0)
	{
		return 0.0;
	}
	Scale(v, 1.0/magnitude);
	return magnitude;
}

// Normalise a vector to unit length over all axes
/*static*/ float DDA::Normalise(float v[]) noexcept
{
	const float magnitude = Magnitude(v);
	if (magnitude <= 0.0)
	{
		return 0.0;
	}
	Scale(v, 1.0/magnitude);
	return magnitude;
}

// Make the direction vector unit-normal in the linear axes, taking account of axis mapping, and return the previous magnitude
float DDA::NormaliseLinearMotion(AxesBitmap linearAxes) noexcept
{
	// First calculate the magnitude of the vector. If there is more than one X or Y axis, take an average of their movements (they should normally be equal).
	float xMagSquared = 0.0, yMagSquared = 0.0, magSquared = 0.0;
	unsigned int numXaxes = 0, numYaxes = 0;
	const AxesBitmap xAxes = Tool::GetXAxes(tool);
	const AxesBitmap yAxes = Tool::GetYAxes(tool);
	const float * const dv = directionVector;
	linearAxes.Iterate([&xMagSquared, &yMagSquared, &magSquared, &numXaxes, &numYaxes, xAxes, yAxes, dv](unsigned int axis, unsigned int count)
						{
							const float dv2 = fsquare(dv[axis]);
							if (xAxes.IsBitSet(axis))
							{
								xMagSquared += dv2;
								++numXaxes;
							}
							else if (yAxes.IsBitSet(axis))
							{
								yMagSquared += dv2;
								++numYaxes;
							}
							else
							{
								magSquared += dv2;
							}
						}
					  );
	if (numXaxes > 1)
	{
		xMagSquared /= numXaxes;
	}
	if (numYaxes > 1)
	{
		yMagSquared /= numYaxes;
	}
	const float magnitude = fastSqrtf(xMagSquared + yMagSquared + magSquared);
	if (magnitude <= 0.0)
	{
		return 0.0;
	}

	// Now normalise it
	Scale(directionVector, 1.0/magnitude);
	return magnitude;
}

// Return the magnitude of a vector over the specified orthogonal axes
/*static*/ float DDA::Magnitude(const float v[], AxesBitmap axes) noexcept
{
	float magnitude = 0.0;
	axes.Iterate([&magnitude, v](unsigned int axis, unsigned int count) { magnitude += fsquare(v[axis]); });
	return fastSqrtf(magnitude);
}

// Multiply a vector by a scalar
/*static*/ void DDA::Scale(float v[], float scale) noexcept
{
	for (size_t d = 0; d < MaxAxesPlusExtruders; d++)
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
// In the first case we may be called from the step ISR or from EndstopsManager::OnEndstopOrZProbeStatesChanged
#if SUPPORT_CAN_EXPANSION
// Returns true if the caller needs to wake the async sender task because CAN-connected drivers need to be stopped
bool DDA::CheckEndstops(Platform& platform) noexcept
#else
void DDA::CheckEndstops(Platform& platform) noexcept
#endif
{
#if SUPPORT_CAN_EXPANSION
	bool wakeAsyncSender = false;
#endif
	while (true)
	{
		const EndstopHitDetails hitDetails = platform.GetEndstops().CheckEndstops();
		switch (hitDetails.GetAction())
		{
		case EndstopHitAction::stopAll:
			MoveAborted();											// set the state to completed and recalculate the endpoints
#if SUPPORT_CAN_EXPANSION
			if (CanMotion::StopAll(*this)) { wakeAsyncSender = true; }
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
#if SUPPORT_CAN_EXPANSION
			return wakeAsyncSender;
#else
			return;
#endif

		case EndstopHitAction::stopAxis:
			StopDrive(hitDetails.axis);								// we must stop the drive before we mess with its coordinates
#if SUPPORT_CAN_EXPANSION
			if (CanMotion::StopAxis(*this, hitDetails.axis)) { wakeAsyncSender = true; }
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
				if (CanMotion::StopDriver(*this, hitDetails.axis, hitDetails.driver)) { wakeAsyncSender = true; }
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

		default:
#if SUPPORT_CAN_EXPANSION
			return wakeAsyncSender;
#else
			return;
#endif
		}
	}
}

// Start executing this move. Must be called with interrupts disabled or basepri >= set interrupt priority, to avoid a race condition.
void DDA::Start(Platform& p, uint32_t tim) noexcept
pre(state == frozen)
{
	if ((int32_t)(tim - afterPrepare.moveStartTime ) > 25)
	{
		afterPrepare.moveStartTime = tim;							// this move is late starting, so record the actual start time
	}
	state = executing;

#if DDA_LOG_PROBE_CHANGES
	if ((endStopsToCheck & LogProbeChanges) != 0)
	{
		numLoggedProbePositions = 0;
		probeTriggered = false;
	}
#endif

	LatePrepareExtruders();											// finish preparing any extruder drives

	if (activeDMs != nullptr)
	{
		if (!flags.checkEndstops)
		{
			p.EnableAllSteppingDrivers();							// make sure that all drivers are enabled
		}

		for (const DriveMovement* pdm = activeDMs; pdm != nullptr; pdm = pdm->nextDM)
		{
			p.SetDirection(pdm->drive, pdm->direction);
		}

#if SUPPORT_REMOTE_COMMANDS
		if (!flags.isRemote)
#endif
		{
			if (afterPrepare.averageExtrusionSpeed != 0.0)
			{
				p.ExtrudeOn();
				if (tool != nullptr)
				{
					// Pass the extrusion speed averaged over the whole move in mm/sec
					tool->ApplyFeedForward(afterPrepare.averageExtrusionSpeed);
				}
			}
			else
			{
				p.ExtrudeOff();
				if (tool != nullptr)
				{
					tool->StopFeedForward();
				}
			}
		}
	}
}

#ifdef DUET3_MB6XD
volatile uint32_t DDA::lastStepHighTime = 0;
#else
volatile uint32_t DDA::lastStepLowTime = 0;
#endif
volatile uint32_t DDA::lastDirChangeTime = 0;

#if DDA_DEBUG_STEP_COUNT
int32_t DDA::stepsRequested[NumDirectDrivers];
int32_t DDA::stepsDone[NumDirectDrivers];
#endif

#if 0	//debug
uint32_t lastDelay;
uint32_t maxDelay;
uint32_t maxDelayIncrease;
#endif

// Generate the step pulses of internal drivers used by this DDA
// Sets the status to 'completed' if the move is complete and the next move should be started
void DDA::StepDrivers(Platform& p, uint32_t now) noexcept
{
	// Check endstop switches and Z probe if asked. This is not speed critical because fast moves do not use endstops or the Z probe.
	if (flags.checkEndstops)		// if any homing switches or the Z probe is enabled in this move
	{
#if SUPPORT_CAN_EXPANSION
		if (CheckEndstops(p)) { CanInterface::WakeAsyncSender(); }
#else
		CheckEndstops(p);			// call out to a separate function because this may help cache usage in the more common and time-critical case where we don't call it
#endif
		if (state == completed)		// we may have completed the move due to triggering an endstop switch or Z probe
		{
			return;
		}
	}

	uint32_t driversStepping = 0;
	DriveMovement* dm = activeDMs;
	const uint32_t elapsedTime = (now - afterPrepare.moveStartTime) + StepTimer::MinInterruptInterval;
#if 0	//DEBUG
	if (dm != nullptr && elapsedTime >= dm->nextStepTime)
	{
		const uint32_t delay = elapsedTime - dm->nextStepTime;
		if (dm->nextStep != 1)
		{
			if (delay > maxDelay) { maxDelay = delay; }
			if (delay > lastDelay && (delay - lastDelay) > maxDelayIncrease) { maxDelayIncrease = delay - lastDelay; }
		}
		lastDelay = delay;
	}
#endif	//END DEBUG
	while (dm != nullptr && elapsedTime >= dm->nextStepTime)		// if the next step is due
	{
		driversStepping |= p.GetDriversBitmap(dm->drive);
#if DDA_DEBUG_STEP_COUNT
		stepsDone[dm->drive] += (dm->direction) ? 1 : -1;
#endif
		dm = dm->nextDM;
	}

	driversStepping &= p.GetSteppingEnabledDrivers();

#ifdef DUET3_MB6XD
	if (driversStepping != 0)
	{
		// Wait until step low and direction setup time have elapsed
		const uint32_t locLastStepPulseTime = lastStepHighTime;
		const uint32_t locLastDirChangeTime = lastDirChangeTime;
		while (now - locLastStepPulseTime < p.GetSlowDriverStepPeriodClocks() || now - locLastDirChangeTime < p.GetSlowDriverDirSetupClocks())
		{
			now = StepTimer::GetTimerTicks();
		}

		StepPins::StepDriversLow(StepPins::AllDriversBitmap & (~driversStepping));		// disable the step pins of the drivers we don't want to step
		StepPins::StepDriversHigh(driversStepping);										// set up the drivers that we do want to step

		// Trigger the TC so that it generates a step pulse
		STEP_GATE_TC->TC_CHANNEL[STEP_GATE_TC_CHAN].TC_CCR = TC_CCR_SWTRG;
		lastStepHighTime = StepTimer::GetTimerTicks();
	}

	// Calculate the next step times. We must do this even if no local drivers are stepping in case endstops or Z probes are active.
	for (DriveMovement *dm2 = activeDMs; dm2 != dm; dm2 = dm2->nextDM)
	{
		(void)dm2->CalcNextStepTime(*this);							// calculate next step times
	}
#else
# if SUPPORT_SLOW_DRIVERS											// if supporting slow drivers
	if ((driversStepping & p.GetSlowDriversBitmap()) != 0)			// if using some slow drivers
	{
		// Wait until step low and direction setup time have elapsed
		uint32_t lastStepPulseTime = lastStepLowTime;
		while (now - lastStepPulseTime < p.GetSlowDriverStepLowClocks() || now - lastDirChangeTime < p.GetSlowDriverDirSetupClocks())
		{
			now = StepTimer::GetTimerTicks();
		}

		StepPins::StepDriversHigh(driversStepping);					// step drivers high
		lastStepPulseTime = StepTimer::GetTimerTicks();

		for (DriveMovement *dm2 = activeDMs; dm2 != dm; dm2 = dm2->nextDM)
		{
			(void)dm2->CalcNextStepTime(*this);						// calculate next step times
		}

		while (StepTimer::GetTimerTicks() - lastStepPulseTime < p.GetSlowDriverStepHighClocks()) {}
		StepPins::StepDriversLow(driversStepping);					// step drivers low
		lastStepLowTime = StepTimer::GetTimerTicks();
	}
	else
# endif
	{
		StepPins::StepDriversHigh(driversStepping);					// step drivers high
# if SAME70
		__DSB();													// without this the step pulse can be far too short
# endif
		for (DriveMovement *dm2 = activeDMs; dm2 != dm; dm2 = dm2->nextDM)
		{
			(void)dm2->CalcNextStepTime(*this);						// calculate next step times
		}

		StepPins::StepDriversLow(driversStepping);					// step drivers low
	}
#endif

	// Remove those drives from the list, update the direction pins where necessary, and re-insert them so as to keep the list in step-time order.
	DriveMovement *dmToInsert = activeDMs;							// head of the chain we need to re-insert
	activeDMs = dm;													// remove the chain from the list
	while (dmToInsert != dm)										// note that both of these may be nullptr
	{
		DriveMovement * const nextToInsert = dmToInsert->nextDM;
		if (dmToInsert->state >= DMState::firstMotionState)
		{
			if (dmToInsert->directionChanged)
			{
				dmToInsert->directionChanged = false;
				p.SetDirection(dmToInsert->drive, dmToInsert->direction);
			}
			InsertDM(dmToInsert);
		}
		else
		{
			dmToInsert->nextDM = completedDMs;
			completedDMs = dmToInsert;
		}
		dmToInsert = nextToInsert;
	}

	// If there are no more steps to do and the time for the move has nearly expired, flag the move as complete
	if (activeDMs == nullptr)
	{
		// We set a move as current up to MovementStartDelayClocks (about 10ms) before it is due to start.
		// We need to make sure it has really started, or we can get arithmetic wrap round in the case that there are no local drivers stepping.
		const uint32_t timeRunning = StepTimer::GetTimerTicks() - afterPrepare.moveStartTime;
		if (   timeRunning + WakeupTime >= clocksNeeded				// if it looks like the move has almost finished
			&& timeRunning < 0 - AbsoluteMinimumPreparedTime		// and it really has started
			)
		{
			state = completed;
		}
	}
}

// Just before staring a move, we must call LatePrepareExtruder on any local extruders that are moving. These extruders have been put in the completedDMs list.
void DDA::LatePrepareExtruders() noexcept
{
	DriveMovement *pdm = completedDMs;
	completedDMs = nullptr;
	while (pdm != nullptr)
	{
		DriveMovement *const nextDm = pdm->nextDM;
		if (pdm->state == DMState::extruderPendingPreparation && pdm->LatePrepareExtruder(*this))
		{
			InsertDM(pdm);										// it has steps to do
		}
		else
		{
			pdm->nextDM = completedDMs;							// no steps to do, so put it back in the completed list
			completedDMs = pdm;
		}
		pdm = nextDm;
	}
}

// Simulate stepping the drivers, for debugging.
// This is basically a copy of DDA::StepDrivers except that instead of being called from the timer ISR and generating steps,
// it is called from the Move task and outputs info on the step timings. It ignores endstops.
void DDA::SimulateSteppingDrivers(Platform& p) noexcept
{
	static uint32_t lastStepTime;
	static bool checkTiming = false;
	static uint8_t lastDrive = 0;

	DriveMovement* dm = activeDMs;
	if (dm != nullptr)
	{
		const uint32_t dueTime = dm->nextStepTime;
		while (dm != nullptr && dueTime >= dm->nextStepTime)			// if the next step is due
		{
			const int32_t timeDiff = (int32_t)(dm->nextStepTime - lastStepTime);
			const bool badTiming = checkTiming && dm->drive == lastDrive && (timeDiff < 10 || timeDiff > 100000000);
			debugPrintf("%10" PRIu32 " D%u %c%s", dm->nextStepTime, dm->drive, (dm->direction) ? 'F' : 'B', (badTiming) ? " *\n" : "\n");
			lastDrive = dm->drive;
			dm = dm->nextDM;
		}
		lastStepTime = dueTime;
		checkTiming = true;

		for (DriveMovement *dm2 = activeDMs; dm2 != dm; dm2 = dm2->nextDM)
		{
			(void)dm2->CalcNextStepTime(*this);							// calculate next step times
		}

		// Remove those drives from the list, update the direction pins where necessary, and re-insert them so as to keep the list in step-time order.
		DriveMovement *dmToInsert = activeDMs;							// head of the chain we need to re-insert
		activeDMs = dm;													// remove the chain from the list
		while (dmToInsert != dm)										// note that both of these may be nullptr
		{
			DriveMovement * const nextToInsert = dmToInsert->nextDM;
			if (dmToInsert->state >= DMState::firstMotionState)
			{
				dmToInsert->directionChanged = false;
				InsertDM(dmToInsert);
			}
			else
			{
				dmToInsert->nextDM = completedDMs;
				completedDMs = dmToInsert;
			}
			dmToInsert = nextToInsert;
		}
	}

	// If there are no more steps to do, flag the move as complete
	if (activeDMs == nullptr)
	{
		checkTiming = false;		// don't check the timing of the first step in the next move
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
			endPoint[drive] = prev->endPoint[drive] + pdm->GetNetStepsTaken();
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

// Return the proportion of the extrusion in the complete multi-segment move that has already been done.
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
			int32_t stepsTaken = 0;
			float extrusionRequested = 0;
			for (size_t extruder = 0; extruder < reprap.GetGCodes().GetNumExtruders(); ++extruder)
			{
				const size_t logicalDrive = ExtruderToLogicalDrive(extruder);
				const DriveMovement* const pdm = FindDM(logicalDrive);
				if (pdm != nullptr)								// if this extruder is active
				{
					stepsTaken += pdm->GetNetStepsTaken();
					extrusionRequested += totalDistance * directionVector[logicalDrive];
				}
			}
			if (extrusionRequested > 0.0)										// if the move has net extrusion
			{
				proportionDoneSoFar += ((proportionDone - proportionDoneSoFar) * stepsTaken) / extrusionRequested;
			}
		}
	}
	return proportionDoneSoFar;
}

bool DDA::HasStepError() const noexcept
{
	for (const DriveMovement* dm = completedDMs; dm != nullptr; )
	{
		if (dm->state >= DMState::stepError1 && dm->state < DMState::firstMotionState)
		{
			return true;
		}
		dm = dm->nextDM;
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

// Update the movement accumulators to account for the move that has just finished.
// Only drives that correspond to extruders need to be updated, but it doesn't matter if we update them all.
// This is called with interrupts disabled.
void DDA::UpdateMovementAccumulators(volatile int32_t *accumulators) const noexcept
{
	// To identify all the extruder movement, we can either loop through extruder numbers and search both DM lists for a DM for that drive,
	// or we can iterate through both DM lists, checking whether the drive it is for is an extruder.
	// It's probably quicker to iterate through DMs.
	//
	// Loop through DMs, checking whether each associated drive is an extruder and updating the movement accumulator if so.
	// We could omit the check that the drive is an accumulator so that we update all accumulators, but we would still need to check for leadscrew adjustment moves.
	const unsigned int firstExtruderDrive =
#if SUPPORT_REMOTE_COMMANDS
		(CanInterface::InExpansionMode())
			? 0 :
#endif
				ExtruderToLogicalDrive(reprap.GetGCodes().GetNumExtruders() - 1);
	const unsigned int lastExtruderDrive =
#if SUPPORT_REMOTE_COMMANDS
		(CanInterface::InExpansionMode())
			? NumDirectDrivers - 1 :
#endif
				MaxAxesPlusExtruders - 1;
	if (firstExtruderDrive <= lastExtruderDrive)
	{
		for (const DriveMovement* dm = activeDMs; dm != nullptr; )
		{
			const uint8_t drv = dm->drive;
			if (   drv >= firstExtruderDrive						// check that it's an extruder (to save the call to GetStepsTaken)
				&& drv <= lastExtruderDrive							// check that it's not a direct leadscrew move
			   )
			{
				accumulators[drv] += dm->GetNetStepsTaken();
			}
			dm = dm->nextDM;
		}
		for (const DriveMovement* dm = completedDMs; dm != nullptr; )
		{
			const uint8_t drv = dm->drive;
			if (   drv >= firstExtruderDrive						// check that it's an extruder (to save the call to GetStepsTaken)
				&& drv <= lastExtruderDrive							// check that it's not a direct leadscrew move
			   )
			{
				accumulators[drv] += dm->GetNetStepsTaken();
			}
			dm = dm->nextDM;
		}
	}
}

float DDA::GetTotalExtrusionRate() const noexcept
{
	float fraction = 0.0;
	for (size_t i = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); i < MaxAxesPlusExtruders; ++i)
	{
		fraction += directionVector[i];
	}
	return fraction * InverseConvertSpeedToMmPerSec(topSpeed);
}

#if SUPPORT_LASER

// Manage the laser power. Return the number of ticks until we should be called again, or 0 to be called at the start of the next move.
uint32_t DDA::ManageLaserPower() const noexcept
{
	Platform& platform = reprap.GetPlatform();
	if (!flags.controlLaser || laserPwmOrIoBits.laserPwm == 0)
	{
		platform.SetLaserPwm(0);
		return 0;
	}

	const uint32_t clocksMoving = StepTimer::GetTimerTicks() - afterPrepare.moveStartTime;
	if (clocksMoving >= clocksNeeded)			// this also covers the case of now < startTime
	{
		// Something has gone wrong with the timing. Set zero laser power, but try again soon.
		platform.SetLaserPwm(0);
		return LaserPwmIntervalMillis;
	}

	const float accelSpeed = startSpeed + acceleration * clocksMoving;
	if (accelSpeed < topSpeed)
	{
		// Acceleration phase
		const Pwm_t pwm = (Pwm_t)((accelSpeed/topSpeed) * laserPwmOrIoBits.laserPwm);
		platform.SetLaserPwm(pwm);
		return LaserPwmIntervalMillis;
	}

	const uint32_t clocksLeft = clocksNeeded - clocksMoving;
	const float decelSpeed = endSpeed + deceleration * clocksLeft;
	if (decelSpeed < topSpeed)
	{
		// Deceleration phase
		const Pwm_t pwm = (Pwm_t)((decelSpeed/topSpeed) * laserPwmOrIoBits.laserPwm);
		platform.SetLaserPwm(pwm);
		return LaserPwmIntervalMillis;
	}

	// We must be in the constant speed phase
	platform.SetLaserPwm(laserPwmOrIoBits.laserPwm);
	const uint32_t decelClocks = (topSpeed - endSpeed)/deceleration;
	if (clocksLeft <= decelClocks)
	{
		return LaserPwmIntervalMillis;
	}
	const uint32_t clocksToDecel = clocksLeft - decelClocks;
	return lrintf((float)clocksToDecel * StepClocksToMillis) + LaserPwmIntervalMillis;
}

#endif

// End
