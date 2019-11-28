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
#include "Kinematics/LinearDeltaKinematics.h"		// for DELTA_AXES
#include "Tools/Tool.h"

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanInterface.h"
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

	MoveParameters()
	{
		accelDistance = steadyDistance = decelDistance = requestedSpeed = startSpeed = topSpeed = endSpeed = targetNextSpeed = 0.0;
		endstopChecks = 0;
		flags = 0;
	}

	void DebugPrint() const
	{
		reprap.GetPlatform().MessageF(DebugMessage, "%f,%f,%f,%f,%f,%f,%f,%f,%08" PRIX32 ",%04x\n",
								(double)accelDistance, (double)steadyDistance, (double)decelDistance, (double)requestedSpeed, (double)startSpeed, (double)topSpeed, (double)endSpeed,
								(double)targetNextSpeed, endstopChecks, flags);
	}

	static void PrintHeading()
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
/*static*/ void DDA::PrintMoves()
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

/*static*/ void DDA::PrintMoves() { }

#endif

#if DDA_LOG_PROBE_CHANGES

size_t DDA::numLoggedProbePositions = 0;
int32_t DDA::loggedProbePositions[XYZ_AXES * MaxLoggedProbePositions];
bool DDA::probeTriggered = false;

void DDA::LogProbePosition()
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

DDA::DDA(DDA* n) : next(n), prev(nullptr), state(empty)
{
	activeDMs = completedDMs = nullptr;

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

void DDA::ReleaseDMs()
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
int32_t DDA::GetTimeLeft() const
pre(state == executing || state == frozen || state == completed)
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(afterPrepare.moveStartTime + clocksNeeded - StepTimer::GetInterruptClocks())
			: (int32_t)clocksNeeded;
}

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance. Now that we generate step pulses
// for multiple motors simultaneously, there is no need to preserve round-robin order.
inline void DDA::InsertDM(DriveMovement *dm)
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
void DDA::DeactivateDM(size_t drive)
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

void DDA::DebugPrintVector(const char *name, const float *vec, size_t len) const
{
	debugPrintf("%s=", name);
	for (size_t i = 0; i < len; ++i)
	{
		debugPrintf("%c%f", ((i == 0) ? '[' : ' '), (double)vec[i]);
	}
	debugPrintf("]");
}

// Print the text followed by the DDA only
void DDA::DebugPrint(const char *tag) const
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
void DDA::DebugPrintAll(const char *tag) const
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
bool DDA::InitStandardMove(DDARing& ring, GCodes::RawMove &nextMove, bool doMotorMapping)
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
	float accelerations[MaxTotalDrivers];
	const float * const normalAccelerations = reprap.GetPlatform().Accelerations();
	const Kinematics& k = move.GetKinematics();

	for (size_t drive = 0; drive < MaxTotalDrivers; drive++)
	{
		accelerations[drive] = normalAccelerations[drive];
		endCoordinates[drive] = nextMove.coords[drive];

		if (drive < numTotalAxes)
		{
			if (!doMotorMapping && drive < numVisibleAxes)
			{
				endPoint[drive] = Move::MotorMovementToSteps(drive, nextMove.coords[drive]);
			}

			int32_t delta = endPoint[drive] - positionNow[drive];
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
					if (positionDelta != 0.0 && (IsBitSet(Tool::GetXAxes(nextMove.tool), drive) || IsBitSet(Tool::GetYAxes(nextMove.tool), drive)))
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
		else
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
					const float compensationTime = reprap.GetPlatform().GetPressureAdvance(drive - numTotalAxes);
					if (compensationTime > 0.0)
					{
						// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
						accelerations[drive] = min<float>(accelerations[drive], reprap.GetPlatform().GetInstantDv(drive)/compensationTime);
					}
				}
			}
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
	flags.usesEndstops = (nextMove.endStopsToCheck != 0);
	endStopsToCheck = nextMove.endStopsToCheck;					//TODO move this to DDARing
	filePos = nextMove.filePos;
	virtualExtruderPosition = nextMove.virtualExtruderPosition;
	proportionDone = nextMove.proportionDone;

	flags.canPauseAfter = nextMove.canPauseAfter;
	flags.usingStandardFeedrate = nextMove.usingStandardFeedrate;
	flags.isPrintingMove = flags.xyMoving && forwardExtruding;				// require forward extrusion so that wipe-while-retracting doesn't count
	flags.isNonPrintingExtruderMove = extruding && !flags.isPrintingMove;	// flag used by filament monitors - we can ignore Z movement
	flags.usePressureAdvance = nextMove.usePressureAdvance;
	flags.hadLookaheadUnderrun = false;
	flags.isLeadscrewAdjustmentMove = false;
	flags.goingSlow = false;

	// The end coordinates will be valid at the end of this move if it does not involve endstop checks and is not a raw motor move
	flags.endCoordinatesValid = (endStopsToCheck == 0) && doMotorMapping;
	flags.continuousRotationShortcut = (nextMove.moveType == 0);

#if SUPPORT_LASER || SUPPORT_IOBITS
	if (nextMove.isCoordinated && endStopsToCheck == 0)
	{
		laserPwmOrIoBits = nextMove.laserPwmOrIoBits;
	}
	else
	{
		laserPwmOrIoBits.Clear();
	}
#endif

	// If it's a Z probing move, limit the Z acceleration to better handle nozzle-contact probes
	if ((endStopsToCheck & ZProbeActive) != 0 && accelerations[Z_AXIS] > ZProbeMaxAcceleration)
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
		totalDistance = Normalise(directionVector, MaxTotalDrivers, numTotalAxes);
	}
	else
	{
		// Extruder-only movement. Normalise so that the magnitude is the total absolute movement. This gives the correct feed rate for mixing extruders.
		totalDistance = 0.0;
		for (size_t d = 0; d < MaxTotalDrivers; d++)
		{
			totalDistance += fabsf(directionVector[d]);
		}
		if (totalDistance > 0.0)		// should always be true
		{
			Scale(directionVector, 1.0/totalDistance, MaxTotalDrivers);
		}
	}

	// 5. Compute the maximum acceleration available
	float normalisedDirectionVector[MaxTotalDrivers];			// used to hold a unit-length vector in the direction of motion
	memcpy(normalisedDirectionVector, directionVector, sizeof(normalisedDirectionVector));
	Absolute(normalisedDirectionVector, MaxTotalDrivers);
	acceleration = beforePrepare.maxAcceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations, MaxTotalDrivers);
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
								VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform().MaxFeedrates(), MaxTotalDrivers));

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
bool DDA::InitLeadscrewMove(DDARing& ring, float feedrate, const float adjustments[MaxTotalDrivers])
{
	// 1. Compute the new endpoints and the movement vector
	bool realMove = false;

	for (size_t drive = 0; drive < MaxTotalDrivers; drive++)
	{
		endPoint[drive] = prev->endPoint[drive];				// adjusting leadscrews doesn't change the endpoint
		endCoordinates[drive] = prev->endCoordinates[drive];	// adjusting leadscrews doesn't change the position
		directionVector[drive] = adjustments[drive];			// for leadscrew adjustment moves, store the adjustment needed in directionVector
		const int32_t delta = lrintf(directionVector[drive] * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));
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
	flags.canPauseAfter = true;
	flags.usingStandardFeedrate = false;
	flags.usePressureAdvance = false;
	flags.hadLookaheadUnderrun = false;
	flags.goingSlow = false;
	flags.continuousRotationShortcut = false;
	endStopsToCheck = 0;
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
	totalDistance = Normalise(directionVector, MaxTotalDrivers, MaxTotalDrivers);

	// 6. Set the speed to the smaller of the requested and maximum speed.
	requestedSpeed = feedrate;

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	startSpeed = endSpeed = 0.0;

	RecalculateMove(ring);
	state = provisional;
	return true;
}

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
inline bool DDA::IsDecelerationMove() const
{
	return beforePrepare.decelDistance == totalDistance					// the simple case - is a deceleration-only move
			|| (topSpeed < requestedSpeed								// can't have been intended as deceleration-only if it reaches the requested speed
				&& beforePrepare.decelDistance > 0.98 * totalDistance	// rounding error can only go so far
			   );
}

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
inline bool DDA::IsAccelerationMove() const
{
	return beforePrepare.accelDistance == totalDistance					// the simple case - is an acceleration-only move
			|| (topSpeed < requestedSpeed								// can't have been intended as deceleration-only if it reaches the requested speed
				&& beforePrepare.accelDistance > 0.98 * totalDistance	// rounding error can only go so far
			   );
}

// Return true if there is no reason to delay preparing this move
bool DDA::IsGoodToPrepare() const
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
/*static*/ void DDA::DoLookahead(DDARing& ring, DDA *laDDA)
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
float DDA::AdvanceBabyStepping(DDARing& ring, size_t axis, float amount)
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
void DDA::RecalculateMove(DDARing& ring)
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
		for (size_t drive = 0; drive < MaxTotalDrivers; ++drive)
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
void DDA::MatchSpeeds()
{
	for (size_t drive = 0; drive < MaxTotalDrivers; ++drive)
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
bool DDA::FetchEndPosition(volatile int32_t ep[MaxTotalDrivers], volatile float endCoords[MaxTotalDrivers])
{
	for (size_t drive = 0; drive < MaxTotalDrivers; ++drive)
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
	for (size_t eDrive = reprap.GetGCodes().GetTotalAxes(); eDrive < MaxTotalDrivers; ++eDrive)
	{
		endCoords[eDrive] += endCoordinates[eDrive];
	}

	return flags.endCoordinatesValid;
}

// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered
void DDA::SetPositions(const float move[MaxTotalDrivers], size_t numDrives)
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
float DDA::GetEndCoordinate(size_t drive, bool disableMotorMapping)
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
inline void DDA::AdjustAcceleration()
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
void DDA::Prepare(uint8_t simMode, float extrusionPending[])
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
		CanInterface::StartMovement(*this);
#endif

		// Handle all drivers
		const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
		Platform& platform = reprap.GetPlatform();
		AxesBitmap additionalAxisMotorsToEnable = 0, axisMotorsEnabled = 0;
		for (size_t drive = 0; drive < NumDirectDrivers; ++drive)
		{
			if (flags.isLeadscrewAdjustmentMove)
			{
				const int32_t delta = lrintf(directionVector[drive] * totalDistance * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));
				if (delta != 0)
				{
					DriveMovement* const pdm = DriveMovement::Allocate(drive + MaxTotalDrivers, DMState::moving);
					pdm->totalSteps = labs(delta);
					pdm->direction = (delta >= 0);
					if (drive < NumDirectDrivers)							// if the drive is local
					{
						reprap.GetPlatform().EnableDrive(Z_AXIS);			// ensure all Z motors are enabled
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
					else
					{
						pdm->state = DMState::idle;								// no local drivers involved
#if SUPPORT_CAN_EXPANSION
						//TODO support leadscrew adjustment moves on remote drivers
#endif
						pdm->state = DMState::idle;
						pdm->nextDM = completedDMs;
						completedDMs = pdm;
					}
				}
			}
			else if (flags.isDeltaMovement && reprap.GetMove().GetKinematics().GetMotionType(drive) == MotionType::segmentFreeDelta)
			{
				// On a delta we need to allocate a DM for all towers even if there is no net movement
				DriveMovement* const pdm = DriveMovement::Allocate(drive, DMState::moving);
				const int32_t delta = endPoint[drive] - prev->endPoint[drive];
				pdm->totalSteps = labs(delta);
				pdm->direction = (delta >= 0);
				if (platform.GetDriversBitmap(drive) != 0)					// if any of the drives is local
				{
#if !SUPPORT_CAN_EXPANSION
					reprap.GetPlatform().EnableDrive(drive);
#endif
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
				else
				{
					pdm->state = DMState::idle;								// no local drivers involved
					pdm->nextDM = completedDMs;
					completedDMs = pdm;
				}

#if SUPPORT_CAN_EXPANSION
				const AxisDriversConfig& config = platform.GetAxisDriversConfig(drive);
				for (size_t i = 0; i < config.numDrivers; ++i)
				{
					const size_t driver = config.driverNumbers[i];
					if (driver >= NumDirectDrivers)
					{
						CanInterface::AddMovement(*this, params, driver - NumDirectDrivers, pdm->GetSteps());
					}
					else
					{
						platform.EnableDriver(driver);
					}
				}
#endif
			}
			else if (drive < numTotalAxes)
			{
				// It's a linear drive
				int32_t delta = endPoint[drive] - prev->endPoint[drive];
				if (delta != 0)
				{
					if (flags.continuousRotationShortcut && reprap.GetMove().GetKinematics().IsContinuousRotationAxis(drive))
					{
						// This is a continuous rotation axis, so we may have adjusted the move to cross the 180 degrees position
						const int32_t stepsPerRotation = lrintf(360.0 * reprap.GetPlatform().DriveStepsPerUnit(drive));
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
#if !SUPPORT_CAN_EXPANSION
						reprap.GetPlatform().EnableDrive(drive);
#endif
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
					const AxisDriversConfig& config = platform.GetAxisDriversConfig(drive);
					for (size_t i = 0; i < config.numDrivers; ++i)
					{
						const size_t driver = config.driverNumbers[i];
						if (driver >= NumDirectDrivers)
						{
							CanInterface::AddMovement(*this, params, driver - NumDirectDrivers, delta);
						}
						else
						{
							platform.EnableDriver(driver);
						}
					}
#endif
					SetBit(axisMotorsEnabled, drive);
					additionalAxisMotorsToEnable |= reprap.GetMove().GetKinematics().GetConnectedAxes(drive);
				}
			}
			else
			{
				// It's an extruder drive
				if (directionVector[drive] != 0.0)
				{
					DriveMovement* const pdm = DriveMovement::Allocate(drive, DMState::moving);
					if (platform.GetDriversBitmap(drive) != 0)					// if any of the drives is local
					{
#if !SUPPORT_CAN_EXPANSION
						reprap.GetPlatform().EnableDrive(drive);
#endif
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

						if (pdm->PrepareExtruder(*this, params, extrusionPending[drive - numTotalAxes], speedChange, flags.usePressureAdvance))
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
					else
					{
						pdm->state = DMState::idle;								// no local drivers involved
						pdm->nextDM = completedDMs;
						completedDMs = pdm;
					}

#if SUPPORT_CAN_EXPANSION
					const uint8_t driver = platform.GetExtruderDriver(drive - numTotalAxes);
					if (driver >= NumDirectDrivers)
					{
						CanInterface::AddMovement(*this, params, driver - NumDirectDrivers, pdm->GetSteps());
					}
					else
					{
						platform.EnableDriver(driver);
					}
#endif
				}
			}
		}

		// On CoreXY and similar architectures, we also need to enable the motors controlling any connected axes
		additionalAxisMotorsToEnable &= ~axisMotorsEnabled;
		for (size_t drive = 0; additionalAxisMotorsToEnable != 0; ++drive)
		{
			if (IsBitSet(additionalAxisMotorsToEnable, drive))
			{
				ClearBit(additionalAxisMotorsToEnable, drive);
#if SUPPORT_CAN_EXPANSION
				const AxisDriversConfig& config = platform.GetAxisDriversConfig(drive);
				for (size_t i = 0; i < config.numDrivers; ++i)
				{
					const size_t driver = config.driverNumbers[i];
					if (driver >= NumDirectDrivers)
					{
						CanInterface::AddMovement(*this, params, driver - NumDirectDrivers, 0);
					}
					else
					{
						platform.EnableDriver(driver);
					}
				}
#else
				if (platform.GetDriversBitmap(drive) != 0)		// if any of the connected axis drives is local
				{
					reprap.GetPlatform().EnableDrive(drive);
				}
#endif
			}
		}

		const DDAState st = prev->state;
		afterPrepare.moveStartTime = (st == DDAState::executing || st == DDAState::frozen)
						? prev->afterPrepare.moveStartTime + prev->clocksNeeded							// this move will follow the previous one, so calculate the start time assuming no more hiccups
							: StepTimer::GetInterruptClocks() + MovementStartDelayClocks;	// else this move is the first so start it after a short delay

#if SUPPORT_CAN_EXPANSION
		CanInterface::FinishMovement(afterPrepare.moveStartTime);
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

	state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
}

// Take a unit positive-hyperquadrant vector, and return the factor needed to obtain
// length of the vector as projected to touch box[].
/*static*/ float DDA::VectorBoxIntersection(const float v[], const float box[], size_t dimensions)
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
/*static*/ float DDA::Normalise(float v[], size_t dim1, size_t dim2)
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
float DDA::NormaliseXYZ()
{
	// First calculate the magnitude of the vector. If there is more than one X or Y axis, take an average of their movements (they should be equal).
	float xMagSquared = 0.0, yMagSquared = 0.0;
	unsigned int numXaxes = 0, numYaxes = 0;
	const AxesBitmap xAxes = Tool::GetXAxes(tool);
	const AxesBitmap yAxes = Tool::GetYAxes(tool);
	for (size_t d = 0; d < MaxAxes; ++d)
	{
		if (IsBitSet(xAxes, d))
		{
			xMagSquared += fsquare(directionVector[d]);
			++numXaxes;
		}
		if (IsBitSet(yAxes, d))
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
	Scale(directionVector, 1.0/magnitude, MaxTotalDrivers);
	return magnitude;
}

// Return the magnitude of a vector
/*static*/ float DDA::Magnitude(const float v[], size_t dimensions)
{
	float magnitude = 0.0;
	for (size_t d = 0; d < dimensions; d++)
	{
		magnitude += v[d]*v[d];
	}
	return sqrtf(magnitude);
}

// Multiply a vector by a scalar
/*static*/ void DDA::Scale(float v[], float scale, size_t dimensions)
{
	for (size_t d = 0; d < dimensions; d++)
	{
		v[d] *= scale;
	}
}

// Move a vector into the positive hyperquadrant
/*static*/ void DDA::Absolute(float v[], size_t dimensions)
{
	for (size_t d = 0; d < dimensions; d++)
	{
		v[d] = fabsf(v[d]);
	}
}

void DDA::CheckEndstops(Platform& platform)
{
	if ((endStopsToCheck & ZProbeActive) != 0)						// if the Z probe is enabled in this move
	{
		// Check whether the Z probe has been triggered. On a delta at least, this must be done separately from endstop checks,
		// because we have both a high endstop and a Z probe, and the Z motor is not the same thing as the Z axis.
		switch (platform.GetZProbeResult())
		{
		case EndStopHit::lowHit:
			MoveAborted();											// set the state to completed and recalculate the endpoints
			reprap.GetGCodes().MoveStoppedByZProbe();
			return;

		case EndStopHit::nearStop:
			ReduceHomingSpeed();
			break;

		default:
			break;
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

	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t drive = 0; drive <
#if HAS_STALL_DETECT
									NumDirectDrivers
#else
									((endStopsToCheck & UseSpecialEndstop) == 0 ? numAxes : NumDirectDrivers)
#endif
											; ++drive)
	{
		if (IsBitSet(endStopsToCheck, drive))
		{
			const bool esc = (endStopsToCheck & ActiveLowEndstop) == 0;
			const EndStopHit esh = ((endStopsToCheck & UseSpecialEndstop) != 0 && drive >= numAxes)
					? ((platform.EndStopInputState(drive) == esc) ? EndStopHit::lowHit : EndStopHit::noStop)
							: platform.Stopped(drive);
			switch (esh)
			{
			case EndStopHit::lowHit:
			case EndStopHit::highHit:
				if ((endStopsToCheck & UseSpecialEndstop) != 0)			// use non-default endstop while probing a tool offset
				{
					MoveAborted();
				}
				else
				{
					ClearBit(endStopsToCheck, drive);					// clear this check so that we can check for more
					const Kinematics& kin = reprap.GetMove().GetKinematics();
					if (drive < numAxes && kin.QueryTerminateHomingMove(drive))	// if not an extruder drive and this kinematics requires us to stop the move now
					{
						MoveAborted();									// this axis uses shared motors so stop the entire move
					}
					else
					{
						StopDrive(drive);								// we must stop the drive before we mess with its coordinates
					}
					if (drive < numAxes && IsHomingAxes())
					{
						kin.OnHomingSwitchTriggered(drive, esh == EndStopHit::highHit, reprap.GetPlatform().GetDriveStepsPerUnit(), *this);
						reprap.GetGCodes().SetAxisIsHomed(drive);
					}
				}
				break;

			case EndStopHit::nearStop:
				// Only reduce homing speed if there are no more axes to be homed. This allows us to home X and Y simultaneously.
				if (endStopsToCheck == MakeBitmap<EndstopsBitmap>(drive))
				{
					ReduceHomingSpeed();
				}
				break;

			default:
				break;
			}
		}
	}
}

// The remaining functions are speed-critical, so use full optimisation
// The GCC optimize pragma appears to be broken, if we try to force O3 optimisation here then functions are never inlined

// Start executing this move, returning true if Step() needs to be called immediately. Must be called with interrupts disabled or basepri >= set interrupt priority, to avoid a race condition.
void DDA::Start(Platform& p, uint32_t tim)
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

#if SUPPORT_LASER
	// Deal with laser power
	if (reprap.GetGCodes().GetMachineType() == MachineType::laser)
	{
		// Ideally we should ramp up the laser power as the machine accelerates, but for now we don't.
		p.SetLaserPwm(laserPwmOrIoBits.laserPwm);
	}
#endif

	if (activeDMs != nullptr)
	{
		unsigned int extrusions = 0, retractions = 0;			// bitmaps of extruding and retracting drives
		const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
		for (const DriveMovement* pdm = activeDMs; pdm != nullptr; pdm = pdm->nextDM)
		{
			const size_t drive = pdm->drive;
			p.SetDirection(drive, pdm->direction);
			if (drive >= numAxes && drive < MaxTotalDrivers)	// if it's an extruder
			{
				if (pdm->direction == FORWARDS)
				{
					extrusions |= (1 << (drive - numAxes));
				}
				else
				{
					retractions |= (1 << (drive - numAxes));
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
				if (drive >= numAxes && drive < NumDirectDrivers)
				{
					if ((prohibitedMovements & (1 << (drive - numAxes))) != 0)
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
void DDA::StepDrivers(Platform& p)
{
	// 1. Check endstop switches and Z probe if asked. This is not speed critical because fast moves do not use endstops or the Z probe.
	if (flags.usesEndstops)			// if any homing switches or the Z probe is enabled in this move
	{
		CheckEndstops(p);			// call out to a separate function because this may help cache usage in the more common case where we don't call it
		if (state == completed)		// we may have completed the move due to triggering an endstop switch or Z probe
		{
			return;
		}
	}

	uint32_t driversStepping = 0;
	DriveMovement* dm = activeDMs;
	uint32_t now = StepTimer::GetInterruptClocks();
	const uint32_t elapsedTime = (now - afterPrepare.moveStartTime) + MinInterruptInterval;
	while (dm != nullptr && elapsedTime >= dm->nextStepTime)		// if the next step is due
	{
		driversStepping |= p.GetDriversBitmap(dm->drive);
		dm = dm->nextDM;
	}

	if ((driversStepping & p.GetSlowDriversBitmap()) == 0)	// if not using any external drivers
	{
		// 3. Step the drivers
		Platform::StepDriversHigh(driversStepping);					// generate the steps
	}
	else
	{
		// 3. Step the drivers
		uint32_t lastStepPulseTime = lastStepLowTime;
		while (now - lastStepPulseTime < p.GetSlowDriverStepLowClocks() || now - lastDirChangeTime < p.GetSlowDriverDirSetupClocks())
		{
			now = StepTimer::GetInterruptClocks();
		}
		Platform::StepDriversHigh(driversStepping);					// generate the steps
		lastStepPulseTime = StepTimer::GetInterruptClocks();

		// 3a. Reset all step pins low. Do this now because some external drivers don't like the direction pins being changed before the end of the step pulse.
		while (StepTimer::GetInterruptClocks() - lastStepPulseTime < p.GetSlowDriverStepHighClocks()) {}
		Platform::StepDriversLow();									// set all step pins low
		lastStepLowTime = lastStepPulseTime = StepTimer::GetInterruptClocks();
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
	Platform::StepDriversLow();										// set all step pins low

	// If there are no more steps to do and the time for the move has nearly expired, flag the move as complete
	if (activeDMs == nullptr && StepTimer::GetInterruptClocks() - afterPrepare.moveStartTime + WakeupTime >= clocksNeeded)
	{
		state = completed;
	}
}

// Return the time that the next interrupt is needed. It may be earlier than the current time.
std::optional<uint32_t> DDA::GetNextInterruptTime() const
{
	return (state == executing)
			? std::optional<uint32_t>(((activeDMs != nullptr) ? activeDMs->nextStepTime : clocksNeeded - DDA::WakeupTime) + afterPrepare.moveStartTime)
				: std::optional<uint32_t>();
}

// Stop a drive and re-calculate the corresponding endpoint.
// For extruder drivers, we need to be able to calculate how much of the extrusion was completed after calling this.
void DDA::StopDrive(size_t drive)
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
		if (activeDMs == nullptr)
		{
			state = completed;
		}
	}
}

// This is called when we abort a move because we have hit an endstop.
// It stops all drives and adjusts the end points of the current move to account for how far through the move we got.
// The caller must call MoveCompleted at some point after calling this.
void DDA::MoveAborted()
{
	if (state == executing)
	{
		for (size_t drive = 0; drive < NumDirectDrivers; ++drive)
		{
			StopDrive(drive);
		}
	}
	state = completed;
}

// Return the proportion of extrusion for the complete multi-segment move that has already been done.
// The move was either not started or was aborted.
float DDA::GetProportionDone(bool moveWasAborted) const
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
			for (size_t drive = reprap.GetGCodes().GetTotalAxes(); drive < NumDirectDrivers; ++drive)
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
void DDA::ReduceHomingSpeed()
{
	if (!flags.goingSlow)
	{
		flags.goingSlow = true;

		topSpeed *= (1.0/ProbingSpeedReductionFactor);

		// Adjust extraAccelerationClocks so that step timing will be correct in the steady speed phase at the new speed
		const uint32_t clocksSoFar = StepTimer::GetInterruptClocks() -afterPrepare. moveStartTime;
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

bool DDA::HasStepError() const
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
bool DDA::Free()
{
	ReleaseDMs();
	state = empty;
	return flags.hadLookaheadUnderrun;
}

// Return the number of net steps already taken in this move by a particular drive
int32_t DDA::GetStepsTaken(size_t drive) const
{
	const DriveMovement * const dmp = FindDM(drive);
	return (dmp != nullptr) ? dmp->GetNetStepsTaken() : 0;
}

void DDA::LimitSpeedAndAcceleration(float maxSpeed, float maxAcceleration)
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

// End
