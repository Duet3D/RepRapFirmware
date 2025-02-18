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
#include <GCodes/GCodes.h>

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
	const float steadyDistance = decelStartDistance - accelDistance;
	steadyClocks = (steadyDistance <= 0.0) ? 0 : lrintf(steadyDistance/dda.topSpeed);
	acceleration = dda.acceleration;
	deceleration = dda.deceleration;
	accelClocks = lrintf((dda.topSpeed - dda.startSpeed)/dda.acceleration);
	decelClocks = lrintf((dda.topSpeed - dda.endSpeed)/dda.deceleration);
	useInputShaping = dda.flags.xyMoving
					&& !(dda.flags.isolatedMove || dda.flags.isLeadscrewAdjustmentMove
#if SUPPORT_SCANNING_PROBES
						 || dda.flags.scanningProbeMove
#endif
						) ;
}

void PrepParams::DebugPrint() const noexcept
{
	debugPrintf("pp: td=%.3g ad=%.3g dsd=%.3g a=%.3g d=%.3g ac=%" PRIu32 " sc=%" PRIu32 " dc=%" PRIu32 "\n",
					(double)totalDistance, (double)accelDistance, (double)decelStartDistance, (double)acceleration, (double)deceleration, accelClocks, steadyClocks, decelClocks);
}

DDA::DDA(DDA *_ecv_null n) noexcept : next(n), prev(nullptr), state(empty)
{
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

// Return the number of clocks this DDA still needs to execute.
uint32_t DDA::GetTimeLeft() const noexcept
{
	switch (state)
	{
	case provisional:
		return clocksNeeded;
	case committed:
		{
			const int32_t timeExecuting = (int32_t)(StepTimer::GetMovementTimerTicks() - afterPrepare.moveStartTime);
			return (timeExecuting <= 0) ? clocksNeeded							// move has not started yet
					: ((uint32_t)timeExecuting > clocksNeeded) ? 0				// move has completed
						: clocksNeeded - (uint32_t)timeExecuting;				// move is part way through
		}
	default:
		return 0;
	}
}

void DDA::DebugPrintVector(const char *_ecv_array name, const float *_ecv_array vec, size_t len) const noexcept
{
	debugPrintf("%s=", name);
	for (size_t i = 0; i < len; ++i)
	{
		const char c = (i == 0) ? '[' : ' ';
		if (vec[i] == 0.0)
		{
			debugPrintf("%c0", c);						// just print 0 to save characters
		}
		else
		{
			debugPrintf("%c%.4g", c, (double)vec[i]);
		}
	}
	debugPrintf("]");
}

// Print the text followed by the DDA only
void DDA::DebugPrint(const char *_ecv_array tag) const noexcept
{
	debugPrintf("%s %u ts=%" PRIu32 " DDA: s=%.4g", tag, (unsigned int)state, afterPrepare.moveStartTime, (double)totalDistance);
	DebugPrintVector(" vec", directionVector, MaxAxesPlusExtruders);
	debugPrintf("\n" "a=%.4e d=%.4e reqv=%.4e startv=%.4e topv=%.4e endv=%.4e cks=%" PRIu32 " fp=%" PRIu32 " fl=%04x\n",
				(double)acceleration, (double)deceleration, (double)requestedSpeed, (double)startSpeed, (double)topSpeed, (double)endSpeed, clocksNeeded, (uint32_t)filePos, flags.all);
}

// Set up a real move. Return true if it represents real movement, else false.
// Either way, return the amount of extrusion we didn't do in the extruder coordinates of nextMove
bool DDA::InitStandardMove(DDARing& ring, const RawMove &nextMove, bool doMotorMapping) noexcept
{
	// 0. If there are more total axes than visible axes, then we must ignore any movement data in nextMove for the invisible axes.
	// Likewise we must ignore any movement data in nextMove for unowned axes.
	// The call to CartesianToMotorSteps may adjust the invisible axis endpoints for architectures such as CoreXYU and delta with >3 towers, so set them up here.
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	const Move& move = reprap.GetMove();

	// 1. Compute the new endpoints and the movement vector
#if SUPPORT_ASYNC_MOVES
	ownedDrives = nextMove.logicalDrivesOwned;
#endif

	flags.all = 0;														// set all flags false
	bool linearAxesMoving = false;
	bool rotationalAxesMoving = false;

	// Deal with axis movement
	if (doMotorMapping)
	{
		if (!move.CartesianToMotorSteps(nextMove.coords, endPoint, nextMove.isCoordinated))		// transform the axis coordinates to motor endpoints
		{
			return false;												// throw away the move if it couldn't be transformed
		}

		// Note, the following loop iterates over both axes and logical drives
		for (size_t axisOrDrive = 0; axisOrDrive < numTotalAxes; axisOrDrive++)
		{
#if SUPPORT_ASYNC_MOVES
			if (nextMove.axesAndExtrudersOwned.IsBitSet(axisOrDrive))
#endif
			{
				const float positionDelta = nextMove.coords[axisOrDrive] - ring.GetStartCoordinate(axisOrDrive);
				ring.SetStartCoordinate(axisOrDrive, nextMove.coords[axisOrDrive]);
				directionVector[axisOrDrive] = positionDelta;
				if (positionDelta != 0.0)
				{
					if (reprap.GetMove().IsAxisRotational(axisOrDrive))
					{
						if (nextMove.rotationalAxesMentioned)
						{
							rotationalAxesMoving = true;
						}
					}
					else if (nextMove.linearAxesMentioned)
					{
						linearAxesMoving = true;
						if (Tool::GetXAxes(nextMove.movementTool).IsBitSet(axisOrDrive) || Tool::GetYAxes(nextMove.movementTool).IsBitSet(axisOrDrive))
						{
							flags.xyMoving = true;				// this move has XY movement in user space, before axis were mapped
						}
					}
				}
			}
#if SUPPORT_ASYNC_MOVES
			else
			{
				// This is an axis we don't own, so make sure we don't move it
				directionVector[axisOrDrive] = 0.0;
			}

			if (!ownedDrives.IsBitSet(axisOrDrive))
			{
				endPoint[axisOrDrive] = prev->endPoint[axisOrDrive];
			}
#endif
		}
	}
	else
	{
		// Raw motor move
		for (size_t drive = 0; drive < numVisibleAxes; drive++)
		{
#if SUPPORT_ASYNC_MOVES
			if (ownedDrives.IsBitSet(drive))
#endif
			{
				// Raw motor move on a visible axis
				endPoint[drive] = move.MotorMovementToSteps(drive, nextMove.coords[drive]);
				const int32_t delta = endPoint[drive] - prev->endPoint[drive];
				directionVector[drive] = (float)delta/move.DriveStepsPerMm(drive);
				if (delta != 0)
				{
					if (reprap.GetMove().IsAxisRotational(drive))
					{
						rotationalAxesMoving = true;
					}
					else
					{
						linearAxesMoving = true;
					}
				}
			}
#if SUPPORT_ASYNC_MOVES
			else
			{
				// This is an axis we don't own, so make sure we don't move it
				directionVector[drive] = 0.0;
				endPoint[drive] = prev->endPoint[drive];
			}
#endif
		}

		// Set any invisible axis endpoints to the same positions as the previous move
		for (size_t drive = numVisibleAxes; drive < numTotalAxes; ++drive)
		{
			endPoint[drive] = prev->endPoint[drive];
			directionVector[drive] = 0.0;
		}
	}

	// Clear out unused logical drives
	for (size_t drive = numTotalAxes; drive < MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); ++drive)
	{
		directionVector[drive] = 0.0;
		endPoint[drive] = prev->endPoint[drive];
	}

	// Deal with extruder movement
	float accelerations[MaxAxesPlusExtruders];
	memcpyf(accelerations, reprap.GetMove().Accelerations(nextMove.reduceAcceleration), MaxAxesPlusExtruders);
	bool extrudersMoving = false;
	bool forwardExtruding = false;

	for (size_t drive = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); drive < MaxAxesPlusExtruders; ++drive)
	{
#if SUPPORT_ASYNC_MOVES
		if (ownedDrives.IsBitSet(drive))
#endif
		{
			// It's an extruder drive. We defer calculating the steps because they may be affected by nonlinear extrusion, which we can't calculate until we
			// know the speed of the move, and because extruder movement is relative so we need to accumulate fractions of a whole step between moves.
			const float movement = nextMove.coords[drive];
			directionVector[drive] = movement;							// for an extruder, endCoordinates is the amount of movement
			if (movement != 0.0)
			{
				extrudersMoving = true;
				if (movement > 0.0)
				{
					forwardExtruding = true;
				}
				if (flags.xyMoving && nextMove.usePressureAdvance)
				{
					const float compensationClocks = reprap.GetMove().GetPressureAdvanceClocksForLogicalDrive(drive);
					if (compensationClocks > 0.0)
					{
						// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
						accelerations[drive] = min<float>(accelerations[drive], reprap.GetMove().GetMaxInstantDv(drive)/compensationClocks);
					}
				}
			}
		}
#if SUPPORT_ASYNC_MOVES
		else
		{
			// This is an extruder we don't own, so make sure we don't move it
			directionVector[drive] = 0.0;
		}
#endif
	}

	// 2. Throw it away if there's no real movement.
	if (!(linearAxesMoving || rotationalAxesMoving || extrudersMoving))
	{
		// Update the end position in the previous move, so that on the next move we don't think there is XY movement when the user didn't ask for any
		if (doMotorMapping)
		{
			for (size_t drive = 0; drive < numTotalAxes; ++drive)
			{
				ring.SetStartCoordinate(drive, nextMove.coords[drive]);
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
	flags.isolatedMove = nextMove.checkEndstops || nextMove.moveType != 0;
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
	const Kinematics &_ecv_from k = move.GetKinematics();
	if (linearAxesMoving)
	{
		// There is some linear axis movement, so normalise the direction vector so that the total linear movement has unit length and 'totalDistance' is the linear distance moved.
		// This means that the user gets the feed rate that he asked for. It also makes the delta calculations simpler.
		// First do the bed tilt compensation for deltas.
		directionVector[Z_AXIS] += (directionVector[X_AXIS] * k.GetTiltCorrection(X_AXIS)) + (directionVector[Y_AXIS] * k.GetTiltCorrection(Y_AXIS));
		totalDistance = NormaliseLinearMotion(reprap.GetMove().GetLinearAxes());
	}
	else if (rotationalAxesMoving)
	{
		// Some axes are moving, but not axes that X or Y are mapped to. Normalise the movement to the vector sum of the axes that are moving.
		totalDistance = Normalise(directionVector, reprap.GetMove().GetRotationalAxes());
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
			if (k.GetKinematicsType() == KinematicsType::linearDelta && normalisedDirectionVector[axis] > maxDistance)
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
	requestedSpeed = min<float>(max<float>(reqSpeed, reprap.GetMove().MinMovementSpeed()),
								VectorBoxIntersection(normalisedDirectionVector, reprap.GetMove().MaxFeedrates()));

	// On a Cartesian printer, it is OK to limit the X and Y speeds and accelerations independently, and in consequence to allow greater values
	// for diagonal moves. On other architectures, this is not OK and any movement in the XY plane should be limited on other ways.
	if (doMotorMapping)
	{
		k.LimitSpeedAndAcceleration(*this, normalisedDirectionVector, numVisibleAxes, flags.continuousRotationShortcut);	// give the kinematics the chance to further restrict the speed and acceleration
	}

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
	endSpeed = 0.0;																// until the next move asks us to adjust it

	// See if we can meld this with the end of the previous one (which must currently have the end speed set to zero)
	if (   prev->state == provisional											// if previous move has not started yet
		&& (   move.GetJerkPolicy() != 0										// and melding is allowed
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
		directionVector[drive] = 0.0;
	}

	for (size_t driver = 0; driver < MaxDriversPerAxis; ++driver)
	{
		directionVector[driver] = adjustments[driver];			// for leadscrew adjustment moves, store the adjustment needed in directionVector
		const int32_t delta = lrintf(adjustments[driver] * reprap.GetMove().DriveStepsPerMm(Z_AXIS));
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
	flags.isolatedMove = true;
	virtualExtruderPosition = prev->virtualExtruderPosition;
	tool = nullptr;
	filePos = prev->filePos;
	acceleration = deceleration = reprap.GetMove().NormalAcceleration(Z_AXIS);

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
		const size_t axisToUse = (reprap.GetMove().GetKinematics().GetKinematicsType() == KinematicsType::linearDelta && drive <= Z_AXIS) ? Z_AXIS : drive;
		directionVector[drive] = nextMove.movements[axisToUse];
		const int32_t delta = lrintf(nextMove.movements[axisToUse] * reprap.GetMove().DriveStepsPerMm(drive));
		endPoint[drive] = prev->endPoint[drive] + delta;
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
// This version handles the new movement message that includes the input shaping plan and passes extruder movement as distance, not steps
bool DDA::InitFromRemote(DDARing& ring, const CanMessageMovementLinearShaped& msg) noexcept
{
	afterPrepare.moveStartTime = StepTimer::ConvertToLocalTime(msg.whenToExecute);
	flags.all = 0;
	flags.isPrintingMove = flags.usePressureAdvance = msg.usePressureAdvance;
	// TODO For now we treat any non-printing move as a non-printing extruder move. Better to pass a flag for it in the CAN message.
	flags.isNonPrintingExtruderMove = !flags.isPrintingMove;

	// Prepare for movement
	PrepParams params;

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

	const float accelDistanceExTopSpeed = -0.5 * params.acceleration * fsquare((float)params.accelClocks);
	const float decelDistanceExTopSpeed = -0.5 * params.deceleration * fsquare((float)params.decelClocks);
	topSpeed = (params.totalDistance - accelDistanceExTopSpeed - decelDistanceExTopSpeed)/clocksNeeded;

	params.accelDistance =      accelDistanceExTopSpeed + topSpeed * params.accelClocks;
	const float decelDistance = decelDistanceExTopSpeed + topSpeed * params.decelClocks;
	params.decelStartDistance =  1.0 - decelDistance;

	MovementFlags segFlags;
	segFlags.Clear();
	segFlags.nonPrintingMove = !msg.usePressureAdvance;
	segFlags.noShaping = !msg.useLateInputShaping;

	afterPrepare.drivesMoving.Clear();
	Move& move = reprap.GetMove();

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
				move.EnableDrivers(drive, false);
				move.AddLinearSegments(*this, drive, msg.whenToExecute, params, extrusionRequested, segFlags);
			}
		}
		else
		{
			const float delta = (float)msg.perDrive[drive].steps;
			directionVector[drive] = delta;
			if (delta != 0.0)
			{
				move.EnableDrivers(drive, false);
				move.AddLinearSegments(*this, drive, msg.whenToExecute, params, delta, segFlags);
				afterPrepare.drivesMoving.SetBit(drive);
			}
		}
	}

	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintAllMoves))
	{
		DebugPrint("rems");
	}

	state = committed;
	return true;
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
//pre(state == provisional)
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
					if (st == committed)
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
			laDDA = _ecv_not_null(laDDA->prev);
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

			laDDA = _ecv_not_null(laDDA->next);
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

	// Find the oldest un-prepared move
	DDA *cdda = this;
	while (cdda->prev->state == DDAState::provisional)
	{
		cdda = _ecv_not_null(cdda->prev);
	}

	// cdda addresses the earliest un-prepared move, which is the first one we can apply babystepping to
	// Allow babystepping Z speed up to 10% of the move top speed or up to half the Z jerk rate, whichever is lower
	float babySteppingDone = 0.0;
	while (cdda != this)
	{
		if (amount != 0.0 && cdda->flags.xyMoving)
		{
			// Limit the babystepping Z speed to the lower of 0.1 times the original XYZ speed and 0.5 times the Z jerk
			Move& move = reprap.GetMove();
			const float maxBabySteppingAmount = cdda->totalDistance * min<float>(0.1, 0.5 * move.GetMaxInstantDv(Z_AXIS)/cdda->topSpeed);
			const float babySteppingToDo = constrain<float>(amount, -maxBabySteppingAmount, maxBabySteppingAmount);
			cdda->directionVector[Z_AXIS] += babySteppingToDo/cdda->totalDistance;
			cdda->totalDistance *= cdda->NormaliseLinearMotion(move.GetLinearAxes());
			cdda->RecalculateMove(ring);
			babySteppingDone += babySteppingToDo;
			amount -= babySteppingToDo;
		}

		// Even if there is no babystepping to do this move, we may need to adjust the end coordinates
		cdda->endPoint[Z_AXIS] += (int32_t)(babySteppingDone * reprap.GetMove().DriveStepsPerMm(Z_AXIS));

		// Now do the next move
		cdda = _ecv_not_null(cdda->next);
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
		const Move& m = reprap.GetMove();
		for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
		{
			if (endSpeed * fabsf(directionVector[drive]) > m.GetMaxInstantDv(drive))
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
			const float allowedJerk = reprap.GetMove().GetPrintingInstantDv(drive);
			if (jerk > allowedJerk)
			{
				beforePrepare.targetNextSpeed = allowedJerk/totalFraction;
			}
		}
	}
}

// Force an end point. Called when a homing switch is triggered.
void DDA::SetDriveCoordinate(size_t drive, int32_t ep) noexcept
{
	endPoint[drive] = ep;
}

// Get a Cartesian end coordinate from this move
void DDA::GetEndCoordinates(float returnedCoords[MaxAxes]) noexcept
{
	const size_t totalAxes = reprap.GetGCodes().GetTotalAxes();
	reprap.GetMove().MotorStepsToCartesian(endPoint, reprap.GetGCodes().GetVisibleAxes(), totalAxes, returnedCoords);
}

// Dispatch this DDA to the move segment queue for execution.
// This must not be called with interrupts disabled, because it calls Platform::EnableDrive.
void DDA::Prepare(DDARing& ring, SimulationMode simMode) noexcept
{
#if SUPPORT_LASER
	if (topSpeed < requestedSpeed && reprap.GetGCodes().GetMachineType() == MachineType::laser)
	{
		// Scale back the laser power according to the actual speed
		laserPwmOrIoBits.laserPwm = (Pwm_t)((laserPwmOrIoBits.laserPwm * topSpeed)/requestedSpeed);
	}
#endif

	// Prepare for movement
	PrepParams params;
	params.SetFromDDA(*this);
	clocksNeeded = params.TotalClocks();

	// Decide when this move should start.
	// Avoid setting the move start time in the past or with very little time before it starts, because this can lead to us trying to modify a segment that is already executing
	const uint32_t now = StepTimer::GetMovementTimerTicks();
	if (prev->state == committed)
	{
		const uint32_t prevEndTime = prev->afterPrepare.moveStartTime + prev->clocksNeeded;
		if ((int32_t)(prevEndTime - now) >= (int32_t)MoveTiming::AbsoluteMinimumPreparedTime)
		{
			afterPrepare.moveStartTime = prevEndTime;		// start this move directly after the previous one
		}
		else if (startSpeed == 0.0)
		{
			afterPrepare.moveStartTime = now + MoveTiming::UsualMinimumPreparedTime;
		}
		else
		{
			afterPrepare.moveStartTime = now + MoveTiming::AbsoluteMinimumPreparedTime;
			reprap.GetMove().AddPrepareHiccup();		// move was supposed to follow the previous one directly, so record a hiccup
		}
	}
	else
	{
		afterPrepare.moveStartTime = now + MoveTiming::UsualMinimumPreparedTime;
	}

	if (simMode < SimulationMode::normal)
	{
#if SUPPORT_CAN_EXPANSION
		CanMotion::StartMovement();
#endif
		// Handle all drivers
		Move& move = reprap.GetMove();
		if (flags.isLeadscrewAdjustmentMove)
		{
			move.EnableDrivers(Z_AXIS, false);			// ensure all Z motors are enabled
		}

		float extrusionFraction = 0.0;
		AxesBitmap additionalAxisMotorsToEnable, axisMotorsEnabled;
		afterPrepare.drivesMoving.Clear();
		MovementFlags segFlags;
		segFlags.Clear();
		segFlags.checkEndstops = flags.checkEndstops;
		segFlags.noShaping = !params.useInputShaping;
		segFlags.nonPrintingMove = !flags.isPrintingMove;
		for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
		{
			if (flags.isLeadscrewAdjustmentMove)
			{
				// We don't set any bits in drivesMoving because setting the Z bit would be misleading, and setting individual driver bits isn't useful because it doesn't take account of CAN-connected drivers.
				// For a leadscrew adjustment move, the first N elements of the direction vector are the adjustments to the N Z motors
				const AxisDriversConfig& config = move.GetAxisDriversConfig(Z_AXIS);
				if (drive < config.numDrivers)
				{
					const int32_t delta = lrintf(directionVector[drive] * totalDistance * move.DriveStepsPerMm(Z_AXIS));
					const DriverId driver = config.driverNumbers[drive];
					if (delta != 0)
					{
#if SUPPORT_CAN_EXPANSION
						if (driver.IsRemote())
						{
							CanMotion::AddAxisMovement(params, driver, delta);
						}
						else		// we don't generate segments for leadscrew adjustment moves to remote drivers
#endif
						{
							move.AddLinearSegments(*this, driver.localDriver + MaxAxesPlusExtruders, afterPrepare.moveStartTime, params, (motioncalc_t)delta, segFlags);
						}
					}
				}
			}
			else
#if SUPPORT_ASYNC_MOVES
			if (ownedDrives.IsBitSet(drive))
#endif
			{
				if (drive < reprap.GetGCodes().GetTotalAxes())
				{
					// It's a linear axis
					int32_t delta = endPoint[drive] - prev->endPoint[drive];
					if (delta != 0)
					{
						move.EnableDrivers(drive, false);
						if (flags.continuousRotationShortcut && reprap.GetMove().GetKinematics().IsContinuousRotationAxis(drive))
						{
							// This is a continuous rotation axis, so we may have adjusted the move to cross the 180 degrees position
							const int32_t stepsPerRotation = lrintf(360.0 * move.DriveStepsPerMm(drive));
							if (delta > stepsPerRotation/2)
							{
								delta -= stepsPerRotation;
							}
							else if (delta < -stepsPerRotation/2)
							{
								delta += stepsPerRotation;
							}
						}

						delta = move.ApplyBacklashCompensation(drive, delta);

						// We generate segments even for nonlocal drivers so that the final position is correct and to track the position in near real time
						move.AddLinearSegments(*this, drive, afterPrepare.moveStartTime, params, (motioncalc_t)delta, segFlags);
						afterPrepare.drivesMoving.SetBit(drive);

#if SUPPORT_CAN_EXPANSION
						const AxisDriversConfig& config = move.GetAxisDriversConfig(drive);
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
						additionalAxisMotorsToEnable |= reprap.GetMove().GetKinematics().GetControllingDrives(drive, flags.checkEndstops);
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
						if (simMode != SimulationMode::off || flags.checkEndstops || Tool::ExtruderMovementAllowed(tool, directionVector[drive] > 0.0, extruder))
						{
							move.EnableDrivers(drive, false);

							if (flags.isPrintingMove && directionVector[drive] > 0.0)
							{
								extrusionFraction += directionVector[drive];					// accumulate the total extrusion fraction
							}

#if SUPPORT_NONLINEAR_EXTRUSION
							// Add the nonlinear extrusion correction to totalExtrusion.
							// If we are given a stupidly short move to execute then clocksNeeded can be zero, which leads to NaNs in this code; so we need to guard against that.
							if (flags.isPrintingMove && clocksNeeded != 0)
							{
								const NonlinearExtrusion& nl = move.GetExtrusionCoefficients(extruder);
								float& dv = directionVector[drive];
								const float averageExtrusionSpeed = (totalDistance * dv * StepClockRate)/(float)clocksNeeded;		// need speed in mm/sec for nonlinear extrusion calculation
								const float factor = 1.0 + min<float>((nl.A + (nl.B * averageExtrusionSpeed)) * averageExtrusionSpeed, nl.limit);
								dv *= factor;
							}
#endif

							const motioncalc_t delta = totalDistance * directionVector[drive] * move.DriveStepsPerMm(drive);

							// We generate segments even for nonlocal extruders in order to track extruder position
							move.AddLinearSegments(*this, drive, afterPrepare.moveStartTime, params, delta, segFlags);

#if SUPPORT_CAN_EXPANSION
							const DriverId driver = move.GetExtruderDriver(extruder);
							if (driver.IsRemote())
							{
								// The MovementLinearShaped message requires the extrusion amount in steps to be passed as a float. The remote board adds the PA and handles fractional steps.
								CanMotion::AddExtruderMovement(params, driver, delta, flags.usePressureAdvance);
							}
#endif
							afterPrepare.drivesMoving.SetBit(drive);
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
			move.EnableDrivers(drive, false);
		}

		afterPrepare.averageExtrusionSpeed = (extrusionFraction * totalDistance * (float)StepClockRate)/(float)clocksNeeded;

		state = committed;																// must do this before we call CheckEndstops
#if SUPPORT_SCANNING_PROBES
		if (flags.scanningProbeMove)
		{
			move.PrepareScanningProbeDataCollection(*this, params);
		}
		else
#endif
		if (flags.checkEndstops)
		{
			// Before we send movement commands to remote drives, if any endstop switches we are monitoring are already set, make sure we don't start the motors concerned.
			// This is especially important when using CAN-connected motors or endstops, because we rely on receiving "endstop changed" messages.
			// Moves that check endstops are always run as isolated moves, so there can be no move in progress and the endstops must already be primed.
			const uint32_t oldPrio = ChangeBasePriority(NvicPriorityStep);				// shut out the step interrupt
			(void)move.CheckEndstops(false);											// this may modify pending CAN moves
			RestoreBasePriority(oldPrio);
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

		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintAllMoves))		// show the prepared DDA if debug enabled
		{
			DebugPrint("pr");
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
	else
	{
		state = committed;
	}
}

// Check whether a committed move has finished
bool DDA::HasExpired() const noexcept
{
	// Note, for Z leadscrew adjustment moves (and any other individual motor moves that we may support in future), we must not use drivesMoving, because it doesn't describe the drivers that are moving.
	return (flags.checkEndstops)
			? reprap.GetMove().AreDrivesStopped(afterPrepare.drivesMoving)
				: (int32_t)(StepTimer::GetMovementTimerTicks() - GetMoveFinishTime()) >= 0;
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
	const float *_ecv_array const dv = directionVector;
	linearAxes.Iterate([&xMagSquared, &yMagSquared, &magSquared, &numXaxes, &numYaxes, xAxes, yAxes, dv](unsigned int axis, unsigned int count) noexcept
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
	axes.Iterate([&magnitude, v](unsigned int axis, unsigned int count) noexcept { magnitude += fsquare(v[axis]); });
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

// Return the proportion of the extrusion in the complete multi-segment move that has already been done.
// The move was either not started or was aborted.
float DDA::GetProportionDone() const noexcept
{
	// Get the proportion of extrusion already done at the start of this segment
	return (filePos != noFilePosition && filePos == prev->filePos)
									? prev->proportionDone
										: 0.0;
}

// Free up this DDA, returning true if the lookahead underrun flag was set
bool DDA::Free() noexcept
{
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

// Manage the laser power. Return the number of ticks until we should be called again, or portMAX_DELAY to be called at the start of the next move.
uint32_t DDA::ManageLaserPower() const noexcept
{
	Platform& platform = reprap.GetPlatform();
	if (!flags.controlLaser || laserPwmOrIoBits.laserPwm == 0)
	{
		platform.SetLaserPwm(0);
		return portMAX_DELAY;
	}

	const uint32_t clocksMoving = StepTimer::GetMovementTimerTicks() - afterPrepare.moveStartTime;
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
	const uint32_t decelClocks = (uint32_t)((topSpeed - endSpeed)/deceleration);
	if (clocksLeft <= decelClocks)
	{
		return LaserPwmIntervalMillis;
	}
	const uint32_t clocksToDecel = clocksLeft - decelClocks;
	return (uint32_t)lrintf((float)clocksToDecel * StepClocksToMillis) + LaserPwmIntervalMillis;
}

#endif

// End
