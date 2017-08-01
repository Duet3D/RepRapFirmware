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
#include "Kinematics/LinearDeltaKinematics.h"		// for DELTA_AXES

#ifdef DUET_NG
# define DDA_MOVE_DEBUG	(1)
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
	float startSpeed;
	float topSpeed;
	float endSpeed;

	MoveParameters()
	{
		accelDistance = steadyDistance = decelDistance = startSpeed = topSpeed = endSpeed = 0.0;
	}
};

const size_t NumSavedMoves = 256;

static MoveParameters savedMoves[NumSavedMoves];
static size_t savedMovePointer = 0;

// Print the saved moves in CSV format for analysis
/*static*/ void DDA::PrintMoves()
{
	// Print the saved moved in CSV format
	for (size_t i = 0; i < NumSavedMoves; ++i)
	{
		const MoveParameters& m = savedMoves[savedMovePointer];
		reprap.GetPlatform().MessageF(DEBUG_MESSAGE, "%f,%f,%f,%f,%f,%f\n", m.accelDistance, m.steadyDistance, m.decelDistance, m.startSpeed, m.topSpeed, m.endSpeed);
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
	for (DriveMovement*& p : pddm)
	{
		p = nullptr;
	}
}

void DDA::ReleaseDMs()
{
	for (DriveMovement*& p : pddm)
	{
		if (p != nullptr)
		{
			DriveMovement::Release(p);
			p = nullptr;
		}
	}
}

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
int32_t DDA::GetTimeLeft() const
pre(state == executing || state == frozen || state == completed)
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(moveStartTime + clocksNeeded - Platform::GetInterruptClocks())
			: (int32_t)clocksNeeded;
}

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance. Now that we generate step pulses
// for multiple motors simultaneously, there is no need to preserve round-robin order.
inline void DDA::InsertDM(DriveMovement *dm)
{
	DriveMovement **dmp = &firstDM;
	while (*dmp != nullptr && (*dmp)->nextStepTime < dm->nextStepTime)
	{
		dmp = &((*dmp)->nextDM);
	}
	dm->nextDM = *dmp;
	*dmp = dm;
}

// Remove this drive from the list of drives with steps due, and return its DM or nullptr if not there
// Called from the step ISR only.
DriveMovement *DDA::RemoveDM(size_t drive)
{
	DriveMovement **dmp = &firstDM;
	while (*dmp != nullptr)
	{
		DriveMovement *dm = *dmp;
		if (dm->drive == drive)
		{
			(*dmp) = dm->nextDM;
			return dm;
		}
		dmp = &(dm->nextDM);
	}
	return nullptr;
}

void DDA::DebugPrintVector(const char *name, const float *vec, size_t len) const
{
	debugPrintf("%s=", name);
	for (size_t i = 0; i < len; ++i)
	{
		debugPrintf("%c%f", ((i == 0) ? '[' : ' '), vec[i]);
	}
	debugPrintf("]");
}

void DDA::DebugPrint() const
{
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	debugPrintf("DDA:");
	if (endCoordinatesValid)
	{
		float startCoordinates[MaxAxes];
		for (size_t i = 0; i < numAxes; ++i)
		{
			startCoordinates[i] = endCoordinates[i] - (totalDistance * directionVector[i]);
		}
		DebugPrintVector(" start", startCoordinates, numAxes);
		DebugPrintVector(" end", endCoordinates, numAxes);
	}

	debugPrintf(" d=%f", totalDistance);
	DebugPrintVector(" vec", directionVector, 5);
	debugPrintf("\na=%f reqv=%f topv=%f startv=%f endv=%f\n"
				"daccel=%f ddecel=%f cks=%u\n",
				acceleration, requestedSpeed, topSpeed, startSpeed, endSpeed,
				accelDistance, decelDistance, clocksNeeded);
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		if (pddm[axis] != nullptr)
		{
			pddm[axis]->DebugPrint(reprap.GetGCodes().axisLetters[axis], isDeltaMovement);
		}
	}
	for (size_t i = numAxes; i < DRIVES; ++i)
	{
		if (pddm[i] != nullptr && pddm[i]->state != DMState::idle)
		{
			pddm[i]->DebugPrint((char)('0' + (i - numAxes)), false);
		}
	}
}

// This is called by Move to initialize all DDAs
void DDA::Init()
{
	// Set the endpoints to zero, because Move asks for them.
	// They will be wrong if we are on a delta. We take care of that when we process the M665 command in config.g.
	for (int32_t& ep : endPoint)
	{
		ep = 0;
	}
	state = empty;
	endCoordinatesValid = false;
	virtualExtruderPosition = 0;
	filePos = noFilePosition;

#if SUPPORT_IOBITS
	ioBits = 0;
#endif
}

// Set up a real move. Return true if it represents real movement, else false.
bool DDA::Init(const GCodes::RawMove &nextMove, bool doMotorMapping)
{
	// 1. Compute the new endpoints and the movement vector
	const int32_t * const positionNow = prev->DriveCoordinates();
	const Move& move = reprap.GetMove();
	if (doMotorMapping)
	{
		if (!move.CartesianToMotorSteps(nextMove.coords, endPoint))		// transform the axis coordinates if on a delta or CoreXY printer
		{
			return false;												// throw away the move if it couldn't be transformed
		}
		isDeltaMovement = move.IsDeltaMode()
							&& (endPoint[X_AXIS] != positionNow[X_AXIS] || endPoint[Y_AXIS] != positionNow[Y_AXIS] || endPoint[Z_AXIS] != positionNow[Z_AXIS]);
	}
	else
	{
		isDeltaMovement = false;
	}

	isPrintingMove = false;
	bool realMove = false, xyzMoving = false;
	xyMoving = false;
	const bool isSpecialDeltaMove = (move.IsDeltaMode() && !doMotorMapping);
	float accelerations[DRIVES];
	const float * const normalAccelerations = reprap.GetPlatform().Accelerations();
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		accelerations[drive] = normalAccelerations[drive];
		if (drive >= numAxes || !doMotorMapping)
		{
			endPoint[drive] = Move::MotorEndPointToMachine(drive, nextMove.coords[drive]);
		}

		endCoordinates[drive] = nextMove.coords[drive];
		const int32_t delta = (drive < numAxes) ? endPoint[drive] - positionNow[drive] : endPoint[drive];

		DriveMovement*& pdm = pddm[drive];
		if (drive < numAxes && !isSpecialDeltaMove)
		{
			const float positionDelta = nextMove.coords[drive] - prev->GetEndCoordinate(drive, false);
			directionVector[drive] = positionDelta;
			if (positionDelta != 0)
			{
				if (drive == Z_AXIS)
				{
					xyzMoving = true;
				}
				else if (drive == Y_AXIS || ((1 << drive) & nextMove.xAxes) != 0)
				{
					xyMoving = xyzMoving = true;
				}
			}
			if (isDeltaMovement || delta != 0)
			{
				pdm = DriveMovement::Allocate(drive, DMState::moving);	// on a delta printer, if one tower moves then we assume they all do
			}
		}
		else
		{
			directionVector[drive] = (float)delta/reprap.GetPlatform().DriveStepsPerUnit(drive);
			if (delta != 0)
			{
				pdm = DriveMovement::Allocate(drive, DMState::moving);
			}
		}

		if (pdm != nullptr)
		{
			pdm->totalSteps = labs(delta);				// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
			pdm->direction = (delta >= 0);				// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
			realMove = true;

			if (drive >= numAxes && xyMoving)
			{
				if (delta > 0)
				{
					isPrintingMove = true;				// we have both XY movement and extrusion
				}
				if (nextMove.usePressureAdvance)
				{
					const float compensationTime = reprap.GetPlatform().GetPressureAdvance(drive - numAxes);
					if (compensationTime > 0.0)
					{
						// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
						accelerations[drive] = min<float>(accelerations[drive], reprap.GetPlatform().ConfiguredInstantDv(drive)/compensationTime);
					}
				}
			}
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		ReleaseDMs();
		return false;
	}

	// 3. Store some values
	xAxes = nextMove.xAxes;
	yAxes = nextMove.yAxes;
	endStopsToCheck = nextMove.endStopsToCheck;
	canPauseBefore = nextMove.canPauseBefore;
	canPauseAfter = nextMove.canPauseAfter;
	filePos = nextMove.filePos;
	usePressureAdvance = nextMove.usePressureAdvance;
	virtualExtruderPosition = nextMove.virtualExtruderPosition;
	hadLookaheadUnderrun = false;
	isLeadscrewAdjustmentMove = false;
	goingSlow = false;

#if SUPPORT_IOBITS
	ioBits = nextMove.ioBits;
#endif

	// If it's a Z probing move, limit the Z acceleration to better handle nozzle-contact probes
	if ((endStopsToCheck & ZProbeActive) != 0 && accelerations[Z_AXIS] > ZProbeMaxAcceleration)
	{
		accelerations[Z_AXIS] = ZProbeMaxAcceleration;
	}

	// The end coordinates will be valid at the end of this move if it does not involve endstop checks and is not a special move on a delta printer
	endCoordinatesValid = (endStopsToCheck == 0) && (doMotorMapping || !move.IsDeltaMode());

	// 4. Normalise the direction vector and compute the amount of motion.
	if (xyzMoving)
	{
		// There is some XYZ movement, so normalise the direction vector so that the total XYZ movement has unit length and 'totalDistance' is the XYZ distance moved.
		// This means that the user gets the feed rate that he asked for. It also makes the delta calculations simpler.
		// First do the bed tilt compensation for deltas.
		const Kinematics& k = move.GetKinematics();
		directionVector[Z_AXIS] += (directionVector[X_AXIS] * k.GetTiltCorrection(X_AXIS)) + (directionVector[Y_AXIS] * k.GetTiltCorrection(Y_AXIS));

		totalDistance = NormaliseXYZ();
	}
	else
	{
		// Extruder-only movement.
		// Currently we normalise vector sum of all extruder movement to unit length.
		// Alternatives would be:
		// 1. Normalise the largest one to unit length. This means that when retracting multiple filaments, they all get the requested retract speed.
		// 2. Normalise the sum to unit length. This means that when we use mixing, we get the requested extrusion rate at the nozzle.
		// 3. Normalise the sum to the sum of the mixing coefficients (which we would have to include in the move details).
		totalDistance = Normalise(directionVector, DRIVES, DRIVES);
	}

	// 5. Compute the maximum acceleration available
	float normalisedDirectionVector[DRIVES];			// Used to hold a unit-length vector in the direction of motion
	memcpy(normalisedDirectionVector, directionVector, sizeof(normalisedDirectionVector));
	Absolute(normalisedDirectionVector, DRIVES);
	acceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations, DRIVES);
	if (xyMoving)
	{
		acceleration = min<float>(acceleration, (isPrintingMove) ? reprap.GetPlatform().GetMaxPrintingAcceleration() : reprap.GetPlatform().GetMaxTravelAcceleration());
	}

	// 6. Set the speed to the smaller of the requested and maximum speed.
	// Also enforce a minimum speed of 0.5mm/sec. We need a minimum speed to avoid overflow in the movement calculations.
	float reqSpeed = nextMove.feedRate;
	if (isSpecialDeltaMove)
	{
		// Special case of a raw or homing move on a delta printer
		// We use the Cartesian motion system to implement these moves, so the feed rate will be interpreted in Cartesian coordinates.
		// This is wrong, we want the feed rate to apply to the drive that is moving the farthest.
		float maxDistance = 0.0;
		for (size_t axis = 0; axis < DELTA_AXES; ++axis)
		{
			if (normalisedDirectionVector[axis] > maxDistance)
			{
				maxDistance = normalisedDirectionVector[axis];
			}
		}
		if (maxDistance != 0.0)				// should always be true
		{
			reqSpeed /= maxDistance;		// because normalisedDirectionVector is unit-normalised
		}
	}
	requestedSpeed = constrain<float>(reqSpeed, 0.5, VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform().MaxFeedrates(), DRIVES));

	// On a Cartesian printer, it is OK to limit the X and Y speeds and accelerations independently, and in consequence to allow greater values
	// for diagonal moves. On a delta, this is not OK and any movement in the XY plane should be limited to the X/Y axis values, which we assume to be equal.
	if (doMotorMapping)
	{
		switch (reprap.GetMove().GetKinematics().GetKinematicsType())
		{
		case KinematicsType::cartesian:
			// On a Cartesian printer the axes accelerate independently
			break;

		case KinematicsType::coreXY:
		case KinematicsType::coreXYU:
			// Preferably we would specialise these, but for now fall through to the default implementation
		default:
			// On other types of printer, the relationship between motor movement and head movement is complex.
			// Limit all moves to the lower of X and Y speeds and accelerations.
			{
				const float xyFactor = sqrtf(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[Y_AXIS]));
				const float * const maxSpeeds = reprap.GetPlatform().MaxFeedrates();
				const float maxSpeed = min<float>(maxSpeeds[X_AXIS], maxSpeeds[Y_AXIS]);
				if (requestedSpeed * xyFactor > maxSpeed)
				{
					requestedSpeed = maxSpeed/xyFactor;
				}

				const float maxAcceleration = min<float>(normalAccelerations[X_AXIS], normalAccelerations[Y_AXIS]);
				if (acceleration * xyFactor > maxAcceleration)
				{
					acceleration = maxAcceleration/xyFactor;
				}
			}
			break;
		}
	}

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	endSpeed = 0.0;					// until the next move asks us to adjust it

	if (prev->state != provisional)
	{
		// There is no previous move that we can adjust, so this move must start at zero speed.
		startSpeed = 0.0;
	}
	else
	{
		// Try to meld this move to the previous move to avoid stop/start
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		prev->targetNextSpeed = min<float>(sqrtf(acceleration * totalDistance * 2.0), requestedSpeed);
		DoLookahead(prev);
		startSpeed = prev->targetNextSpeed;
	}

	RecalculateMove();
	state = provisional;
	return true;
}

// Set up a raw (unmapped) motor move returning true if the move does anything
bool DDA::Init(const float_t adjustments[DRIVES])
{
	// 1. Compute the new endpoints and the movement vector
	const float ZAcceleration = reprap.GetPlatform().Accelerations()[Z_AXIS];
	const float ZSpeed = reprap.GetPlatform().MaxFeedrate(Z_AXIS);

	float accelerations[DRIVES];
	float maxSpeeds[DRIVES];
	bool realMove = false;

	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		accelerations[drive] = ZAcceleration;					// all motors moving are Z motors
		maxSpeeds[drive] = ZSpeed;								// all motors moving are Z motors
		endPoint[drive] = prev->endPoint[drive];				// adjusting leadscrews doesn't change the endpoint
		endCoordinates[drive] = prev->endCoordinates[drive];	// adjusting leadscrews doesn't change the position

		directionVector[drive] = adjustments[drive];
		const int32_t delta = lrintf(directionVector[drive] * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));

		if (delta != 0)
		{
			DriveMovement*& pdm = pddm[drive];
			pdm = DriveMovement::Allocate(drive + DRIVES, DMState::moving);
			pdm->totalSteps = labs(delta);
			pdm->direction = (delta >= 0);
			realMove = true;
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		ReleaseDMs();
		return false;
	}

	// 3. Store some values
	isLeadscrewAdjustmentMove = true;
	isDeltaMovement = false;
	isPrintingMove = false;
	xyMoving = false;
	endStopsToCheck = 0;
	canPauseBefore = true;
	canPauseAfter = true;
	usePressureAdvance = false;
	virtualExtruderPosition = prev->virtualExtruderPosition;
	hadLookaheadUnderrun = false;
	xAxes = prev->xAxes;
	yAxes = prev->yAxes;
	filePos = prev->filePos;
	endCoordinatesValid = prev->endCoordinatesValid;
	goingSlow = false;

#if SUPPORT_IOBITS
	ioBits = prev->ioBits;
#endif

	// 4. Normalise the direction vector and compute the amount of motion.
	// Currently we normalise the vector sum of all Z motor movement to unit length.
	totalDistance = Normalise(directionVector, DRIVES, DRIVES);

	// 5. Compute the maximum acceleration available
	float normalisedDirectionVector[DRIVES];			// Used to hold a unit-length vector in the direction of motion
	memcpy(normalisedDirectionVector, directionVector, sizeof(normalisedDirectionVector));
	Absolute(normalisedDirectionVector, DRIVES);
	acceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations, DRIVES);

	// 6. Set the speed to the smaller of the requested and maximum speed.
	requestedSpeed = VectorBoxIntersection(normalisedDirectionVector, maxSpeeds, DRIVES);

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	startSpeed = endSpeed = 0.0;

	RecalculateMove();
	state = provisional;
	return true;
}

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
// We declare this inline because it is only used once, in DDA::DoLookahead
inline bool DDA::IsDecelerationMove() const
{
	return decelDistance == totalDistance					// the simple case - is a deceleration-only move
			|| (topSpeed < requestedSpeed					// can't have been intended as deceleration-only if it reaches the requested speed
				&& decelDistance > 0.98 * totalDistance		// rounding error can only go so far
				&& prev->state == DDA::provisional			// if we can't adjust the previous move then we don't care (and its figures may not be reliable if it has been recycled already)
				&& prev->decelDistance > 0.0);				// if the previous move has no deceleration phase then no point in adjus6ting it
}

// Try to increase the ending speed of this move to allow the next move to start at targetNextSpeed
/*static*/ void DDA::DoLookahead(DDA *laDDA)
pre(state == provisional)
{
//	if (reprap.Debug(moduleDda)) debugPrintf("Adjusting, %f\n", laDDA->targetNextSpeed);
	unsigned int laDepth = 0;
	bool goingUp = true;

	for(;;)					// this loop is used to nest lookahead without making recursive calls
	{
		bool recurse = false;
		if (goingUp)
		{
			// We have been asked to adjust the end speed of this move to match the next move starting at targetNextSpeed
			if (laDDA->topSpeed >= laDDA->requestedSpeed)
			{
				// This move already reaches its top speed, so just need to adjust the deceleration part
				laDDA->endSpeed = laDDA->requestedSpeed;		// remove the deceleration phase
				laDDA->CalcNewSpeeds();							// put it back if necessary
			}
			else if (laDDA->IsDecelerationMove())
			{
				// This is a deceleration-only move, so we may have to adjust the previous move as well to get optimum behaviour
				if (laDDA->prev->state == provisional)
				{
					laDDA->endSpeed = laDDA->requestedSpeed;
					laDDA->CalcNewSpeeds();
					const float maxStartSpeed = sqrtf(fsquare(laDDA->endSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
					if (maxStartSpeed < laDDA->requestedSpeed)
					{
						laDDA->prev->targetNextSpeed = maxStartSpeed;
						// Still provisionally a decelerate-only move
					}
					else
					{
						laDDA->prev->targetNextSpeed = laDDA->requestedSpeed;
					}
					recurse = true;
				}
				else
				{
					// This move is a deceleration-only move but we can't adjust the previous one
					laDDA->hadLookaheadUnderrun = true;
					laDDA->endSpeed = min<float>(sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance)),
													laDDA->requestedSpeed);
					laDDA->CalcNewSpeeds();
				}
			}
			else
			{
				// This move doesn't reach its requested speed, but it isn't a deceleration-only move
				// Set its end speed to the minimum of the requested speed and the highest we can reach
				laDDA->endSpeed = min<float>(sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance)),
												laDDA->requestedSpeed);
				laDDA->CalcNewSpeeds();
			}
		}
		else
		{
			// Going back down the list
			// We have adjusted the end speed of the previous move as much as is possible and it has adjusted its targetNextSpeed accordingly.
			// Adjust this move to match it.
			laDDA->startSpeed = laDDA->prev->targetNextSpeed;
			const float maxEndSpeed = sqrtf(fsquare(laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
			if (maxEndSpeed < laDDA->endSpeed)
			{
				// Oh dear, we were too optimistic! Have another go.
				laDDA->endSpeed = maxEndSpeed;
				laDDA->CalcNewSpeeds();
			}
		}

		if (recurse)
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
			// Either just stopped going up. or going down
			laDDA->RecalculateMove();

			if (laDepth == 0)
			{
//				if (reprap.Debug(moduleDda)) debugPrintf("Complete, %f\n", laDDA->targetNextSpeed);
				return;
			}

			laDDA = laDDA->next;
			--laDepth;
			goingUp = false;
		}
	}
}

// Try to push babystepping earlier in the move queue, returning the amount we pushed
float DDA::AdvanceBabyStepping(float amount)
{
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
		if (amount != 0.0 && cdda->xyMoving)
		{
			// If not on a delta printer, check that we have a DM for the Z axis
			bool ok = (cdda->isDeltaMovement || cdda->pddm[Z_AXIS] != nullptr);
			if (!ok)
			{
				cdda->pddm[Z_AXIS] = DriveMovement::Allocate(Z_AXIS, DMState::idle);
				ok = (cdda->pddm[Z_AXIS] != nullptr);
			}

			if (ok)
			{
				// Limit the babystepping Z speed to the lower of 0.1 times the original XYZ speed and 0.5 times the Z jerk
				const float maxBabySteppingAmount = cdda->totalDistance * min<float>(0.1, 0.5 * reprap.GetPlatform().ConfiguredInstantDv(Z_AXIS)/cdda->topSpeed);
				babySteppingToDo = constrain<float>(amount, -maxBabySteppingAmount, maxBabySteppingAmount);
				cdda->directionVector[Z_AXIS] += babySteppingToDo/cdda->totalDistance;
				cdda->totalDistance *= cdda->NormaliseXYZ();
				cdda->RecalculateMove();
				babySteppingDone += babySteppingToDo;
				amount -= babySteppingToDo;
			}
		}

		// Even if there is no babystepping to do this move, we may need to adjust the end coordinates
		cdda->endCoordinates[Z_AXIS] += babySteppingDone;
		if (cdda->isDeltaMovement)
		{
			for (size_t tower = 0; tower < DELTA_AXES; ++tower)
			{
				cdda->endPoint[tower] += (int32_t)(babySteppingDone * reprap.GetPlatform().DriveStepsPerUnit(tower));
				if (babySteppingToDo != 0.0)
				{
					int32_t steps = (int32_t)(babySteppingToDo * reprap.GetPlatform().DriveStepsPerUnit(tower));
					DriveMovement* const pdm = cdda->pddm[tower];
					if (pdm != nullptr)
					{
						if (pdm->direction)		// if moving up
						{
							steps += (int32_t)pdm->totalSteps;
						}
						else
						{
							steps -= (int32_t)pdm->totalSteps;
						}
						if (steps >= 0)
						{
							pdm->direction = true;
							pdm->totalSteps = (uint32_t)steps;
						}
						else
						{
							pdm->direction = false;
							pdm->totalSteps = (uint32_t)(-steps);
						}
					}
				}
			}
		}
		else
		{
			cdda->endPoint[Z_AXIS] += (int32_t)(babySteppingDone * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));
			if (babySteppingToDo != 0.0)
			{
				int32_t steps = (int32_t)(babySteppingToDo * reprap.GetPlatform().DriveStepsPerUnit(Z_AXIS));
				DriveMovement* const pdm = cdda->pddm[Z_AXIS];			// must be non-null because we allocated one earlier if necessary
				if (pdm->state == DMState::moving)
				{
					if (pdm->direction)		// if moving up
					{
						steps += (int32_t)pdm->totalSteps;
					}
					else
					{
						steps -= (int32_t)pdm->totalSteps;
					}
				}
				else
				{
					pdm->state = DMState::moving;
				}

				if (pdm != nullptr)
				{
					if (steps >= 0)
					{
						pdm->direction = true;
						pdm->totalSteps = (uint32_t)steps;
					}
					else
					{
						pdm->direction = false;
						pdm->totalSteps = (uint32_t)(-steps);
					}
				}
			}
		}

		// Now do the next move
		cdda = cdda->next;
	}

	return babySteppingDone;
}

// Recalculate the top speed, acceleration distance and deceleration distance, and whether we can pause after this move
// This may cause a move that we intended to be a deceleration-only move to have a tiny acceleration segment at the start
void DDA::RecalculateMove()
{
	accelDistance = (fsquare(requestedSpeed) - fsquare(startSpeed))/(2.0 * acceleration);
	decelDistance = (fsquare(requestedSpeed) - fsquare(endSpeed))/(2.0 * acceleration);
	if (accelDistance + decelDistance >= totalDistance)
	{
		// This move has no steady-speed phase, so it's accelerate-decelerate or accelerate-only move.
		// If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - v^2)/2a = distance.
		// So (2V^2 - u^2 - v^2)/2a = distance
		// So V^2 = a * distance + 0.5(u^2 + v^2)
		float vsquared = (acceleration * totalDistance) + 0.5 * (fsquare(startSpeed) + fsquare(endSpeed));
		// Calculate accelerate distance from: V^2 = u^2 + 2as
		if (vsquared >= 0.0)
		{
			accelDistance = max<float>((vsquared - fsquare(startSpeed))/(2.0 * acceleration), 0.0);
			decelDistance = totalDistance - accelDistance;
			topSpeed = sqrtf(vsquared);
		}
		else
		{
			// It's an accelerate-only or decelerate-only move.
			// Due to rounding errors and babystepping adjustments, we may have to adjust the acceleration slightly.
			if (startSpeed < endSpeed)
			{
				// This would ideally never happen, but might because of rounding errors
				accelDistance = totalDistance;
				decelDistance = 0.0;
				topSpeed = endSpeed;
				acceleration = (fsquare(endSpeed) - fsquare(startSpeed))/(2.0 * totalDistance);
			}
			else
			{
				accelDistance = 0.0;
				decelDistance = totalDistance;
				topSpeed = startSpeed;
				acceleration = (fsquare(startSpeed) - fsquare(endSpeed))/(2.0 * totalDistance);
			}
		}
	}
	else
	{
		topSpeed = requestedSpeed;
	}

	if (canPauseAfter && endSpeed != 0.0)
	{
		const Platform& p = reprap.GetPlatform();
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			if (pddm[drive] != nullptr && pddm[drive]->state == DMState::moving && endSpeed * fabsf(directionVector[drive]) > p.ActualInstantDv(drive))
			{
				canPauseAfter = false;
				break;
			}
		}
	}
}

// Decide what speed we would really like this move to end at.
// On entry, endSpeed is our proposed ending speed and targetNextSpeed is the proposed starting speed of the next move
// On return, targetNextSpeed is the speed we would like the next move to start at, and endSpeed is the corresponding end speed of this move.
void DDA::CalcNewSpeeds()
{
	// We may have to make multiple passes, because reducing one of the speeds may solve some problems but actually make matters worse on another axis.
	bool limited;
	do
	{
//		debugPrintf("  Pass, start=%f end=%f\n", targetStartSpeed, endSpeed);
		limited = false;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			if (   (pddm[drive] != nullptr && pddm[drive]->state == DMState::moving)
				|| (next->pddm[drive] != nullptr && next->pddm[drive]->state == DMState::moving)
			   )
			{
				const float thisMoveFraction = directionVector[drive];
				const float nextMoveFraction = next->directionVector[drive];
				const float thisMoveSpeed = endSpeed * thisMoveFraction;
				const float nextMoveSpeed = targetNextSpeed * nextMoveFraction;
				const float idealDeltaV = fabsf(thisMoveSpeed - nextMoveSpeed);
				float maxDeltaV = reprap.GetPlatform().ActualInstantDv(drive);
				if (idealDeltaV > maxDeltaV)
				{
					// This drive can't change speed fast enough, so reduce the start and/or end speeds
					// This algorithm sometimes converges very slowly, requiring many passes.
					// To ensure it converges at all, and to speed up convergence, we over-adjust the speed to achieve an even lower deltaV.
					maxDeltaV *= 0.8;
					if ((thisMoveFraction >= 0.0) == (nextMoveFraction >= 0.0))
					{
						// Drive moving in the same direction for this move and the next one, so we must reduce speed of the faster one
						if (fabsf(thisMoveSpeed) > fabsf(nextMoveSpeed))
						{
							endSpeed = (fabsf(nextMoveSpeed) + maxDeltaV)/fabsf(thisMoveFraction);
						}
						else
						{
							targetNextSpeed = (fabsf(thisMoveSpeed) + maxDeltaV)/fabsf(nextMoveFraction);
						}
					}
					else if (fabsf(thisMoveSpeed) * 2 < maxDeltaV)
					{
						targetNextSpeed = (maxDeltaV - fabsf(thisMoveSpeed))/fabsf(nextMoveFraction);
					}
					else if (fabsf(nextMoveSpeed) * 2 < maxDeltaV)
					{
						endSpeed = (maxDeltaV - fabsf(nextMoveSpeed))/fabsf(thisMoveFraction);
					}
					else
					{
						targetNextSpeed = maxDeltaV/(2 * fabsf(nextMoveFraction));
						endSpeed = maxDeltaV/(2 * fabsf(thisMoveFraction));
					}
					limited = true;
					// Most conflicts are between X and Y. So if we just did Y, start another pass immediately to save time.
					if (drive == 1)
					{
						break;
					}
				}
			}
		}
	} while (limited);
}

// This is called by Move::CurrentMoveCompleted to update the live coordinates from the move that has just finished
bool DDA::FetchEndPosition(volatile int32_t ep[DRIVES], volatile float endCoords[DRIVES])
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		ep[drive] = endPoint[drive];
	}
	if (endCoordinatesValid)
	{
		const size_t visibleAxes = reprap.GetGCodes().GetVisibleAxes();
		for (size_t axis = 0; axis < visibleAxes; ++axis)
		{
			endCoords[axis] = endCoordinates[axis];
		}
	}

	// Extrusion amounts are always valid
	for (size_t eDrive = reprap.GetGCodes().GetTotalAxes(); eDrive < DRIVES; ++eDrive)
	{
		endCoords[eDrive] += endCoordinates[eDrive];
	}

	return endCoordinatesValid;
}

void DDA::SetPositions(const float move[DRIVES], size_t numDrives)
{
	reprap.GetMove().EndPointToMachine(move, endPoint, numDrives);
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numAxes; ++axis)
	{
		endCoordinates[axis] = move[axis];
	}
	endCoordinatesValid = true;
}

// Get a Cartesian end coordinate from this move
float DDA::GetEndCoordinate(size_t drive, bool disableDeltaMapping)
pre(disableDeltaMapping || drive < MaxAxes)
{
	if (disableDeltaMapping)
	{
		return Move::MotorEndpointToPosition(endPoint[drive], drive);
	}
	else
	{
		const size_t visibleAxes = reprap.GetGCodes().GetVisibleAxes();
		if (drive < visibleAxes && !endCoordinatesValid)
		{
			reprap.GetMove().MotorStepsToCartesian(endPoint, visibleAxes, reprap.GetGCodes().GetTotalAxes(), endCoordinates);
			endCoordinatesValid = true;
		}
		return endCoordinates[drive];
	}
}

// Calculate the time needed for this move. Called instead of Prepare when we are in simulation mode.
float DDA::CalcTime() const
{
	return (topSpeed - startSpeed)/acceleration								// acceleration time
			+ (totalDistance - accelDistance - decelDistance)/topSpeed		// steady speed time
			+ (topSpeed - endSpeed)/acceleration;							// deceleration time
}

// Prepare this DDA for execution.
// This must not be called with interrupts disabled, because it calls Platform::EnableDrive.
void DDA::Prepare()
{
	if (isDeltaMovement)
	{
		// This code used to be in DDA::Init but we moved it here so that we can implement babystepping in it,
		// also it speeds up simulation because we no longer execute this code when simulating.
		// However, this code assumes that the previous move in the DDA ring is the previously-executed move, because it fetches the X and Y end coordinates from that move.
		// Therefore the Move code must not store a new move in that entry until this one has been prepared! (It took me ages to track this down.)
		// Ideally we would store the initial X any Y coordinates in the DDA, but we need to be economical with memory in the Duet 06/085 build.
		a2plusb2 = fsquare(directionVector[X_AXIS]) + fsquare(directionVector[Y_AXIS]);
		cKc = (int32_t)(directionVector[Z_AXIS] * DriveMovement::Kc);

		const float initialX = prev->GetEndCoordinate(X_AXIS, false);
		const float initialY = prev->GetEndCoordinate(Y_AXIS, false);
		const LinearDeltaKinematics& dparams = (const LinearDeltaKinematics&)reprap.GetMove().GetKinematics();
		const float diagonalSquared = dparams.GetDiagonalSquared();
		const float a2b2D2 = a2plusb2 * diagonalSquared;

		for (size_t drive = 0; drive < DELTA_AXES; ++drive)
		{
			const float A = initialX - dparams.GetTowerX(drive);
			const float B = initialY - dparams.GetTowerY(drive);
			const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
			DriveMovement* const pdm = pddm[drive];
			if (pdm != nullptr)
			{
				const float aAplusbB = A * directionVector[X_AXIS] + B * directionVector[Y_AXIS];
				const float dSquaredMinusAsquaredMinusBsquared = diagonalSquared - fsquare(A) - fsquare(B);
				const float h0MinusZ0 = sqrtf(dSquaredMinusAsquaredMinusBsquared);
				pdm->mp.delta.hmz0sK = (int32_t)(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
				pdm->mp.delta.minusAaPlusBbTimesKs = -(int32_t)(aAplusbB * stepsPerMm * DriveMovement::K2);
				pdm->mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared =
						(int64_t)(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));

				// Calculate the distance at which we need to reverse direction.
				if (a2plusb2 <= 0.0)
				{
					// Pure Z movement. We can't use the main calculation because it divides by a2plusb2.
					pdm->direction = (directionVector[Z_AXIS] >= 0.0);
					pdm->reverseStartStep = pdm->totalSteps + 1;
				}
				else
				{
					// The distance to reversal is the solution to a quadratic equation. One root corresponds to the carriages being below the bed,
					// the other root corresponds to the carriages being above the bed.
					const float drev = ((directionVector[Z_AXIS] * sqrt(a2b2D2 - fsquare(A * directionVector[Y_AXIS] - B * directionVector[X_AXIS])))
										- aAplusbB)/a2plusb2;
					if (drev > 0.0 && drev < totalDistance)		// if the reversal point is within range
					{
						// Calculate how many steps we need to move up before reversing
						const float hrev = directionVector[Z_AXIS] * drev + sqrt(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - a2plusb2 * fsquare(drev));
						const int32_t numStepsUp = (int32_t)((hrev - h0MinusZ0) * stepsPerMm);

						// We may be almost at the peak height already, in which case we don't really have a reversal.
						if (numStepsUp < 1 || (pdm->direction && (uint32_t)numStepsUp <= pdm->totalSteps))
						{
							pdm->reverseStartStep = pdm->totalSteps + 1;
						}
						else
						{
							pdm->reverseStartStep = (uint32_t)numStepsUp + 1;

							// Correct the initial direction and the total number of steps
							if (pdm->direction)
							{
								// Net movement is up, so we will go up a bit and then down by a lesser amount
								pdm->totalSteps = (2 * numStepsUp) - pdm->totalSteps;
							}
							else
							{
								// Net movement is down, so we will go up first and then down by a greater amount
								pdm->direction = true;
								pdm->totalSteps = (2 * numStepsUp) + pdm->totalSteps;
							}
						}
					}
					else
					{
						pdm->reverseStartStep = pdm->totalSteps + 1;
					}
				}
			}
		}
	}

	PrepParams params;
	params.decelStartDistance = totalDistance - decelDistance;

	// Convert the accelerate/decelerate distances to times
	const float accelStopTime = (topSpeed - startSpeed)/acceleration;
	const float decelStartTime = accelStopTime + (params.decelStartDistance - accelDistance)/topSpeed;
	const float totalTime = decelStartTime + (topSpeed - endSpeed)/acceleration;

	clocksNeeded = (uint32_t)(totalTime * stepClockRate);

	params.startSpeedTimesCdivA = (uint32_t)((startSpeed * stepClockRate)/acceleration);
	params.topSpeedTimesCdivA = (uint32_t)((topSpeed * stepClockRate)/acceleration);
	params.decelStartClocks = (uint32_t)(decelStartTime * stepClockRate);
	params.topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivA + params.decelStartClocks;
	params.accelClocksMinusAccelDistanceTimesCdivTopSpeed = (uint32_t)((accelStopTime - (accelDistance/topSpeed)) * stepClockRate);
	params.compFactor = 1.0 - startSpeed/topSpeed;

	firstDM = nullptr;

	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		DriveMovement* const pdm = pddm[drive];
		if (pddm != nullptr && pdm->state == DMState::moving)
		{
			if (isLeadscrewAdjustmentMove)
			{
				reprap.GetPlatform().EnableDrive(Z_AXIS);			// ensure all Z motors are enabled
				pdm->PrepareCartesianAxis(*this, params);

				// Check for sensible values, print them if they look dubious
				if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
				{
					DebugPrint();
				}
			}
			else
			{
				reprap.GetPlatform().EnableDrive(drive);
				if (drive >= numAxes)
				{
					pdm->PrepareExtruder(*this, params, usePressureAdvance);

					// Check for sensible values, print them if they look dubious
					if (reprap.Debug(moduleDda)
						&& (   pdm->totalSteps > 1000000
							|| pdm->reverseStartStep < pdm->mp.cart.decelStartStep
							|| (pdm->reverseStartStep <= pdm->totalSteps
								&& pdm->mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA > (int64_t)(pdm->mp.cart.twoCsquaredTimesMmPerStepDivA * pdm->reverseStartStep))
						   )
					   )
					{
						DebugPrint();
					}
				}
				else if (isDeltaMovement && drive < DELTA_AXES)			// for now, additional axes are assumed to be not part of the delta mechanism
				{
					pdm->PrepareDeltaAxis(*this, params);

					// Check for sensible values, print them if they look dubious
					if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
					{
						DebugPrint();
					}
				}
				else
				{
					pdm->PrepareCartesianAxis(*this, params);

					// Check for sensible values, print them if they look dubious
					if (reprap.Debug(moduleDda) && pdm->totalSteps > 1000000)
					{
						DebugPrint();
					}
				}
			}

			// Prepare for the first step
			pdm->nextStep = 0;
			pdm->nextStepTime = 0;
			pdm->stepInterval = 999999;						// initialise to a large value so that we will calculate the time for just one step
			pdm->stepsTillRecalc = 0;							// so that we don't skip the calculation
			const bool stepsToDo = (isDeltaMovement && drive < numAxes)
									? pdm->CalcNextStepTimeDelta(*this, false)
									: pdm->CalcNextStepTimeCartesian(*this, false);
			if (stepsToDo)
			{
				InsertDM(pdm);
			}
			else
			{
				pdm->state = DMState::idle;
			}
		}
	}

	if (reprap.Debug(moduleDda) && reprap.Debug(moduleMove))		// temp show the prepared DDA if debug enabled for both modules
	{
		DebugPrint();
	}

#if DDA_MOVE_DEBUG
	MoveParameters& m = savedMoves[savedMovePointer];
	m.accelDistance = accelDistance;
	m.decelDistance = decelDistance;
	m.steadyDistance = totalDistance - accelDistance - decelDistance;
	m.startSpeed = startSpeed;
	m.topSpeed = topSpeed;
	m.endSpeed = endSpeed;
	savedMovePointer = (savedMovePointer + 1) % NumSavedMoves;
#endif

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
	// First calculate the magnitude of the vector. If there is more than one X axis, take an average of their movements (they should be equal).
	float xMagSquared = 0.0, yMagSquared = 0.0;
	unsigned int numXaxes = 0, numYaxes = 0;
	for (size_t d = 0; d < MaxAxes; ++d)
	{
		if (((1 << d) & xAxes) != 0)
		{
			xMagSquared += fsquare(directionVector[d]);
			++numXaxes;
		}
		if (((1 << d) & yAxes) != 0)
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
	Scale(directionVector, 1.0/magnitude, DRIVES);
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
			reprap.GetMove().ZProbeTriggered(this);
			break;

		case EndStopHit::lowNear:
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

		case EndStopHit::lowNear:
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
	for (size_t drive = 0; drive < numAxes; ++drive)
	{
		if ((endStopsToCheck & (1 << drive)) != 0)
		{
			switch(platform.Stopped(drive))
			{
			case EndStopHit::lowHit:
				endStopsToCheck &= ~(1 << drive);					// clear this check so that we can check for more
				if (endStopsToCheck == 0 || reprap.GetMove().GetKinematics().DriveIsShared(drive))
				{
					// No more endstops to check, or this axis uses shared motors, so stop the entire move
					MoveAborted();
				}
				else
				{
					StopDrive(drive);
				}
				reprap.GetMove().HitLowStop(drive, this);
				break;

			case EndStopHit::highHit:
				endStopsToCheck &= ~(1 << drive);					// clear this check so that we can check for more
				if (endStopsToCheck == 0 || reprap.GetMove().GetKinematics().DriveIsShared(drive))
				{
					// No more endstops to check, or this axis uses shared motors, so stop the entire move
					MoveAborted();
				}
				else
				{
					StopDrive(drive);
				}
				reprap.GetMove().HitHighStop(drive, this);
				break;

			case EndStopHit::lowNear:
				// Only reduce homing speed if there are no more axes to be homed.
				// This allows us to home X and Y simultaneously.
				if (endStopsToCheck == MakeBitmap<AxesBitmap>(drive))
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

// Start executing this move, returning true if Step() needs to be called immediately. Must be called with interrupts disabled, to avoid a race condition.
// Returns true if the caller needs to call the step ISR immediately.
bool DDA::Start(uint32_t tim)
pre(state == frozen)
{
	moveStartTime = tim;
	state = executing;

#if DDA_LOG_PROBE_CHANGES
	if ((endStopsToCheck & LogProbeChanges) != 0)
	{
		numLoggedProbePositions = 0;
		probeTriggered = false;
	}
#endif

	if (firstDM != nullptr)
	{
		unsigned int extrusions = 0, retractions = 0;		// bitmaps of extruding and retracting drives
		const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t i = 0; i < DRIVES; ++i)
		{
			DriveMovement* const pdm = pddm[i];
			if (pdm != nullptr && pdm->state == DMState::moving)
			{
				const size_t drive = pdm->drive;
				reprap.GetPlatform().SetDirection(drive, pdm->direction);
				if (drive >= numAxes && drive < DRIVES)
				{
					if (pdm->direction == FORWARDS)
					{
						extrusions |= (1 << (i - numAxes));
					}
					else
					{
						retractions |= (1 << (i - numAxes));
					}
				}
			}
		}

		bool extruding = false;
		if (extrusions != 0 || retractions != 0)
		{
			const unsigned int prohibitedMovements = reprap.GetProhibitedExtruderMovements(extrusions, retractions);
			for (DriveMovement **dmpp = &firstDM; *dmpp != nullptr; )
			{
				const size_t drive = (*dmpp)->drive;
				const bool thisDriveExtruding = drive >= numAxes && drive < DRIVES;
				if (thisDriveExtruding && (prohibitedMovements & (1 << (drive - numAxes))) != 0)
				{
					*dmpp = (*dmpp)->nextDM;
				}
				else
				{
					extruding = extruding || thisDriveExtruding;
					dmpp = &((*dmpp)->nextDM);
				}
			}
		}

		Platform& platform = reprap.GetPlatform();
		if (extruding)
		{
			platform.ExtrudeOn();
		}
		else
		{
			platform.ExtrudeOff();
		}

		if (firstDM != nullptr)
		{
			return platform.ScheduleStepInterrupt(firstDM->nextStepTime + moveStartTime);
		}
	}

	// No steps are pending. This should not happen, except perhaps for an extrude-only move when extrusion is prohibited
	return true;	// schedule another interrupt immediately
}

extern uint32_t maxReps;

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
bool DDA::Step()
{
	Platform& platform = reprap.GetPlatform();
	uint32_t lastStepPulseTime = platform.GetInterruptClocks();
	bool repeat;
	uint32_t numReps = 0;
	do
	{
		// Keep this loop as fast as possible, in the case that there are no endstops to check!

		// 1. Check endstop switches and Z probe if asked. This is not speed critical because fast moves do not use endstops or the Z probe.
		if (endStopsToCheck != 0)										// if any homing switches or the Z probe is enabled in this move
		{
			CheckEndstops(platform);	// Call out to a separate function because this may help cache usage in the more common case where we don't call it
			if (state == completed)		// we may have completed the move due to triggering an endstop switch or Z probe
			{
				break;
			}
		}

		// 2. Determine which drivers are due for stepping, overdue, or will be due very shortly
		DriveMovement* dm = firstDM;
		const uint32_t elapsedTime = (Platform::GetInterruptClocks() - moveStartTime) + minInterruptInterval;
		uint32_t driversStepping = 0;
		while (dm != nullptr && elapsedTime >= dm->nextStepTime)		// if the next step is due
		{
			++numReps;
			driversStepping |= platform.GetDriversBitmap(dm->drive);
			dm = dm->nextDM;

//uint32_t t3 = Platform::GetInterruptClocks() - t2;
//if (t3 > maxCalcTime) maxCalcTime = t3;
//if (t3 < minCalcTime) minCalcTime = t3;
		}

		// 3. Step the drivers
		if ((driversStepping & platform.GetSlowDrivers()) != 0)
		{
			while (Platform::GetInterruptClocks() - lastStepPulseTime < platform.GetSlowDriverClocks()) {}
			Platform::StepDriversHigh(driversStepping);					// generate the steps
			lastStepPulseTime = Platform::GetInterruptClocks();
		}
		else
		{
			Platform::StepDriversHigh(driversStepping);					// generate the steps
		}

		// 4. Remove those drives from the list, calculate the next step times, update the direction pins where necessary,
		//    and re-insert them so as to keep the list in step-time order. We assume that meeting the direction pin hold time
		//    is not a problem for any driver type. This is not necessarily true.
		DriveMovement *dmToInsert = firstDM;							// head of the chain we need to re-insert
		firstDM = dm;													// remove the chain from the list
		while (dmToInsert != dm)										// note that both of these may be nullptr
		{
			const bool hasMoreSteps = (isDeltaMovement && dmToInsert->drive < DELTA_AXES)
					? dmToInsert->CalcNextStepTimeDelta(*this, true)
					: dmToInsert->CalcNextStepTimeCartesian(*this, true);
			DriveMovement * const nextToInsert = dmToInsert->nextDM;
			if (hasMoreSteps)
			{
				InsertDM(dmToInsert);
			}
			dmToInsert = nextToInsert;
		}

		// 5. Reset all step pins low
		if ((driversStepping & platform.GetSlowDrivers()) != 0)
		{
			while (Platform::GetInterruptClocks() - lastStepPulseTime < platform.GetSlowDriverClocks()) {}
			Platform::StepDriversLow();									// set all step pins low
			lastStepPulseTime = Platform::GetInterruptClocks();
		}
		else
		{
			Platform::StepDriversLow();									// set all step pins low
		}

		// 6. Check for move completed
		if (firstDM == nullptr)
		{
			state = completed;
			break;
		}

		// 7. Schedule next interrupt, or if it would be too soon, generate more steps immediately
		repeat = platform.ScheduleStepInterrupt(firstDM->nextStepTime + moveStartTime);
	} while (repeat);

	if (numReps > maxReps)
	{
		maxReps = numReps;
	}

	if (state == completed)
	{
		const uint32_t finishTime = moveStartTime + clocksNeeded;	// calculate how long this move should take
		Move& move = reprap.GetMove();
		move.CurrentMoveCompleted();							// tell Move that the current move is complete
		return move.TryStartNextMove(finishTime);				// schedule the next move
	}
	return false;
}

// Stop a drive and re-calculate the corresponding endpoint
void DDA::StopDrive(size_t drive)
{
	DriveMovement* const pdm = pddm[drive];
	if (pdm->state == DMState::moving)
	{
		endPoint[drive] -= pdm->GetNetStepsLeft();
		pdm->state = DMState::idle;
		if (drive < reprap.GetGCodes().GetTotalAxes())
		{
			endCoordinatesValid = false;			// the XYZ position is no longer valid
		}
		RemoveDM(drive);
		if (firstDM == nullptr)
		{
			state = completed;
		}
	}
}

// This is called when we abort a move because we have hit an endstop.
// It adjusts the end points of the current move to account for how far through the move we got.
void DDA::MoveAborted()
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		StopDrive(drive);
	}
	state = completed;
}

// Reduce the speed of this move to the indicated speed.
// This is called from the ISR, so interrupts are disabled and nothing else can mess with us.
// As this is only called for homing moves and with very low speeds, we assume that we don't need acceleration or deceleration phases.
void DDA::ReduceHomingSpeed()
{
	if (!goingSlow)
	{
		goingSlow = true;
		const float factor = 3.0;				// the factor by which we are reducing the speed
		topSpeed /= factor;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			DriveMovement* const pdm = pddm[drive];
			if (pdm->state == DMState::moving)
			{
				pdm->ReduceSpeed(*this, factor);
				RemoveDM(pdm->drive);
				InsertDM(pdm);
			}
		}

		// We also need to adjust the total clocks needed, to prevent step errors being recorded
		const uint32_t clocksSoFar = Platform::GetInterruptClocks() - moveStartTime;
		if (clocksSoFar < clocksNeeded)
		{
			const uint32_t clocksLeft = clocksNeeded - clocksSoFar;
			clocksNeeded += (uint32_t)(clocksLeft * (factor - 1.0));
		}
	}
}

bool DDA::HasStepError() const
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		const DriveMovement* const pdm = pddm[drive];
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
	return hadLookaheadUnderrun;
}

// End
