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

// Convert a float to a uint32_t, with negative values converted to zero
inline uint32_t floatToU32(float f) noexcept
{
	return (std::signbit(f)) ? 0 : (uint32_t)f;
}

// Set up the parameters from the DDA, excluding steadyClocks because that may be affected by input shaping
void PrepParams::SetFromDDA(const DDA& dda) noexcept
{
	totalDistance = dda.totalDistance;
	// Due to rounding error, for an accelerate-decelerate move we may have accelDistance+decelDistance slightly greater than totalDistance.
	// We need to make sure that accelDistance <= decelStartDistance for subsequent calculations to work.
#if SUPPORT_S_CURVE
	if (dda.flags.useScurve)
	{
		peakAcceleration = dda.profile.peakAcceleration;
		peakDeceleration = dda.profile.peakDeceleration;
		initialAcceleration = dda.profile.startAcceleration;
		initialDeceleration = (dda.profile.distances[0] + dda.profile.distances[1] + dda.profile.distances[2] + dda.profile.distances[3] == 0) ? initialAcceleration : 0.0;
		jerk = dda.jerk;

		memcpyf(distances, dda.profile.distances, ARRAY_SIZE(distances));		//TODO why not use the distances stored in he DDA instead of copying them?

		// Solve for the period of each phase
		float speed = dda.profile.startSpeed;
		float acc = dda.profile.startAcceleration;
		if (distances[0] == 0.0)
		{
			phase0Clocks = 0;
		}
		else
		{
			const float t0 = SmallestNonNegativeCubicSolution(jerk, 3 * acc, 6 * speed, -6 * distances[0]);
			phase0Clocks = floatToU32(t0);
			debugPrintf("Phase 0: %.3e %lu %.3e %.3e\n", (double)distances[0], phase0Clocks, (double)speed, (double)acc);
			speed += (acc + 0.5 * jerk * t0) * t0;
			acc += jerk * t0;
		}

		if (distances[1] == 0.0)
		{
			phase1Clocks = 0;
		}
		else
		{
			const float t1 = SmallestNonNegativeQuadraticSolution(0.5 * peakAcceleration, speed, -distances[1]);
			phase1Clocks = floatToU32(t1);
			debugPrintf("Phase 1: %.3e %lu %.3e %.3e (%.3e)\n", (double)distances[1], phase1Clocks, (double)speed, (double)peakAcceleration, (double)acc);
			speed += t1 * peakAcceleration;
			acc = peakAcceleration;
		}

		if (distances[2] == 0.0)
		{
			phase2Clocks = 0.0;
		}
		else
		{
			const float t2 = SmallestNonNegativeCubicSolution(-jerk, 3 * acc, 6 * speed, -6 * distances[2]);
			phase2Clocks = floatToU32(t2);
			debugPrintf("Phase 2: %.3e %lu %.3e %.3e\n", (double)distances[2], phase2Clocks, (double)speed, (double)acc);
			speed += (acc - 0.5 * jerk * t2) * t2;
			acc -= jerk * t2;
		}

		if (distances[3] == 0.0)
		{
			steadyClocks = 0.0;
		}
		else
		{
			const float t3 = distances[3]/speed;
			steadyClocks = floatToU32(t3);
			debugPrintf("Phase 3: %.3e %lu %.3e %.3e (%.3e)\n", (double)distances[3], steadyClocks, (double)speed, (double)0.0, (double)acc);
			acc = 0.0;
		}

		if (distances[4] == 0.0)
		{
			phase4Clocks = 0;
		}
		else
		{
			const float t4 = SmallestNonNegativeCubicSolution(-jerk, 3 * acc, 6 * speed, -6 * distances[4]);
			phase4Clocks = floatToU32(t4);
			debugPrintf("Phase 4: %.3e %lu %.3e %.3e\n", (double)distances[4], phase4Clocks, (double)speed, (double)acc);
			speed += (acc - 0.5 * jerk * t4) * t4;
			acc -= jerk * t4;
		}

		if (distances[5] == 0.0)
		{
			phase5Clocks = 0;
		}
		else
		{
			const float t5 = SmallestNonNegativeQuadraticSolution(0.5 * peakDeceleration, speed, -distances[5]);
			phase5Clocks = floatToU32(t5);
			debugPrintf("Phase 5: %.3e %lu %.3e %.3e (%.3e)\n", (double)distances[5], phase5Clocks, (double)speed, (double)peakDeceleration, (double)acc);
			speed += peakDeceleration * t5;
			acc = peakDeceleration;
		}

		if (distances[6] == 0.0)
		{
			phase6Clocks = 0;
		}
		else
		{
			// If we are ending at zero speed then we only just achieve the distance; and due to rounding error the cubic solution may fail.
			float t6 = SmallestNonNegativeCubicSolution(jerk, 3 * acc, 6 * speed, -6 * distances[6]);
			if (std::isnan(t6))
			{
				t6 = SmallestNonNegativeQuadraticSolution(0.5 * jerk, acc, speed);
				if (std::isnan(t6))
				{
					debugPrintf("Failed at %d\n", __LINE__);
					//TODO
				}
				else
				{
					const float actualDistance = (speed + 0.5 * acc * t6) * t6;
					if (fabsf(distances[6] - actualDistance) > fabsf(distances[6]) * 0.0001)
					{
						debugPrintf("Failed at %d\n", __LINE__);
						//TODO
					}
				}
			}
			phase6Clocks = floatToU32(t6);
			debugPrintf("Phase 6: %.3e %lu %.3e %.3e\n", (double)distances[6], phase6Clocks, (double)speed, (double)acc);
			speed += (acc + 0.5 * jerk * t6) * t6;
			acc += jerk * t6;
		}

		debugPrintf("final speed/acc: %.3e %.3e\n", (double)speed, (double)acc);

		if (dda.next->IsProvisional())
		{
			dda.next->profile.startSpeed = speed;
			dda.next->profile.startAcceleration = acc;
		}

		if (steadyClocks == 0.0)
		{
			// We have no steady speed phase (phase 4) so we can combine phases 3 and 5 because they must have the same jerk and acceleration is zero at the transition between them
			phase2Clocks += phase4Clocks;
			phase4Clocks = 0;
			distances[2] += distances[4];
			distances[4] = 0.0;
		}

		const float residualDistance = totalDistance - (distances[0] + distances[1] + distances[2] + distances[3] + distances[4] + distances[5] + distances[6]);

		// We may have a residual distance because of rounding error.
		// We want zero residual distance so that the move has the correct length, so add the residual distance to the longest phase that is present
		if (residualDistance != 0.0)
		{
			debugPrintf("totalDistance=%.4e residual=%.4e\n", (double)dda.totalDistance, (double)residualDistance);

			// Find the longest phase, preferring the steady speed phase if there is one
			unsigned int bestPhase = 3;
			if (distances[1] > distances[bestPhase]) { bestPhase = 1; }
			if (distances[5] > distances[bestPhase]) { bestPhase = 5; }
			if (distances[0] > distances[bestPhase]) { bestPhase = 0; }
			if (distances[6] > distances[bestPhase]) { bestPhase = 6; }
			if (distances[2] > distances[bestPhase]) { bestPhase = 2; }
			if (distances[4] > distances[bestPhase]) { bestPhase = 4; }

			distances[bestPhase] = max<float>(distances[bestPhase] + residualDistance, 0.0);
		}
	}
	else
	{
		peakAcceleration = dda.maxAcceleration;
		peakDeceleration = -dda.maxAcceleration;
		phase0Clocks = phase2Clocks = phase4Clocks = phase6Clocks = 0;
		phase1Clocks = lrintf((dda.profile.topSpeed - dda.profile.startSpeed)/peakAcceleration);
		phase5Clocks = lrintf((dda.profile.endSpeed - dda.profile.topSpeed)/peakDeceleration);
		distances[0] = distances[2] = distances[4] = distances[6] = 0.0;
		distances[5] = dda.beforePrepare.decelDistance;
		const float decelStartDistance = dda.totalDistance - dda.beforePrepare.decelDistance;
		distances[1] = min<float>(dda.beforePrepare.accelDistance, decelStartDistance);
		distances[3] = decelStartDistance - distances[1];
		steadyClocks = (distances[3] <= 0.0) ? 0 : lrintf(distances[3]/dda.profile.topSpeed);
		jerk = 0.0;							// this signals that we are not using S-curve acceleration
	}
#else
	decelStartDistance = dda.totalDistance - dda.beforePrepare.decelDistance;
	accelDistance = min<float>(dda.beforePrepare.accelDistance, decelStartDistance);
	acceleration = dda.maxAcceleration;
	deceleration = -dda.maxAcceleration;
	accelClocks = lrintf((dda.profile.topSpeed - dda.profile.startSpeed)/acceleration);
	decelClocks = lrintf((dda.profile.endSpeed - dda.profile.topSpeed)/deceleration);
	const float steadyDistance = decelStartDistance - accelDistance;
	steadyClocks = (steadyDistance <= 0.0) ? 0 : lrintf(steadyDistance/dda.profile.topSpeed);
#endif
	useInputShaping = dda.flags.xyMoving
					&& !(dda.flags.isolatedMove || dda.flags.isLeadscrewAdjustmentMove
#if SUPPORT_SCANNING_PROBES
						 || dda.flags.scanningProbeMove
#endif
						) ;
}

void PrepParams::DebugPrint() const noexcept
{
	debugPrintf("pp: td=%.3g"
#if SUPPORT_S_CURVE
				" ad=[%.3g %.3g %.3g] dd=[%.3g %.3g %.3g] a=[%.3g %.3g] d=[%.3g %.3g] ac=[%" PRIu32 " %" PRIu32 " %" PRIu32 "] sc=%" PRIu32 " dc=[%" PRIu32 " %" PRIu32 " %" PRIu32 "]"
#else
				" ad=%.3g dsd=%.3g a=%.3g d=%.3g ac=%" PRIu32 " sc=%" PRIu32 " dc=%" PRIu32
#endif
				"\n",
					(double)totalDistance,
#if SUPPORT_S_CURVE
					(double)distances[0], (double)distances[1], (double)distances[2],
					(double)distances[4], (double)distances[5], (double)distances[6],
					(double)initialAcceleration, (double)peakAcceleration,
					(double)initialDeceleration, (double)peakDeceleration,
					phase0Clocks, phase1Clocks, phase2Clocks, steadyClocks, phase4Clocks, phase5Clocks, phase6Clocks
#else
					(double)accelDistance, (double)decelStartDistance,
					(double)acceleration, (double)deceleration,
					accelClocks, steadyClocks, decelClocks
#endif
				);
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
	case planned:
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
	debugPrintf("\n"
#if SUPPORT_S_CURVE
				"a=[%.4e, %.4e, %.4e, %.4e] j=%.4e"
#else
				"a=%.4e"
#endif
				" reqv=%.4e startv=%.4e topv=%.4e endv=%.4e cks=%" PRIu32 " fp=%" PRIu32 " fl=x%04" PRIx32 "\n",
#if SUPPORT_S_CURVE
				(double)profile.startAcceleration, (double)profile.peakAcceleration, (double)profile.peakDeceleration, (double)profile.endAcceleration, (double)jerk,
#else
				(double)maxAcceleration,
#endif
				(double)requestedSpeed, (double)profile.startSpeed, (double)profile.topSpeed, (double)profile.endSpeed, clocksNeeded, (uint32_t)filePos, flags.all);
}

// Set up a real move. Return true if it represents real movement, else false.
// Either way, return the amount of extrusion we didn't do in the extruder coordinates of nextMove
MovementError DDA::InitStandardMove(DDARing& ring, const RawMove &nextMove, bool doMotorMapping) noexcept
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

	flags.all = 0;												// set all flags false
	bool linearAxesMoving = false;
	bool rotationalAxesMoving = false;

	// Deal with axis movement
	if (doMotorMapping)
	{
		const MovementError err = move.CartesianToMotorSteps(nextMove.coords, endPoint, nextMove.isCoordinated);	// transform the axis coordinates to motor endpoints
		if (err != MovementError::ok)
		{
			return err;											// throw away the move if it couldn't be transformed
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
					if (move.IsAxisRotational(axisOrDrive))
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
				const MovementError err = move.MotorMovementToSteps(drive, nextMove.coords[drive], endPoint[drive]);
				if (err != MovementError::ok)
				{
					return err;
				}
				const int32_t delta = endPoint[drive] - prev->endPoint[drive];
				directionVector[drive] = (float)delta/move.DriveStepsPerMm(drive);
				if (delta != 0)
				{
					if (move.IsAxisRotational(drive))
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
	memcpyf(accelerations, move.Accelerations(nextMove.reduceAcceleration), MaxAxesPlusExtruders);
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
					const float compensationClocks = move.GetPressureAdvanceClocksForLogicalDrive(drive);
					if (compensationClocks > 0.0)
					{
						// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
						accelerations[drive] = min<float>(accelerations[drive], move.GetMaxInstantDv(drive)/compensationClocks);
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
		return MovementError::noMovement;
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
	flags.controlLaserOrIoBits = nextMove.isCoordinated && !nextMove.checkEndstops;

	// The end coordinates will be valid at the end of this move if it does not involve endstop checks and is not a raw motor move
	flags.continuousRotationShortcut = (nextMove.moveType == 0);

#if SUPPORT_LASER || SUPPORT_IOBITS
	if (flags.controlLaserOrIoBits)
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
		totalDistance = NormaliseLinearMotion(move.GetLinearAxes());
	}
	else if (rotationalAxesMoving)
	{
		// Some axes are moving, but not axes that X or Y are mapped to. Normalise the movement to the vector sum of the axes that are moving.
		totalDistance = Normalise(directionVector, move.GetRotationalAxes());
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
	maxAcceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations);
	if (flags.xyMoving)												// apply M204 acceleration limits to XY moves
	{
		maxAcceleration = min<float>(maxAcceleration, (flags.isPrintingMove) ? nextMove.maxPrintingAcceleration : nextMove.maxTravelAcceleration);
	}

#if SUPPORT_S_CURVE
	if (move.IsUsingSCurve())
	{
		flags.useScurve = true;
		jerk = VectorBoxIntersection(normalisedDirectionVector, move.Jerks());
	}
#endif

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
			if (k.GetLegacyType() == KinematicsType::linearDelta && normalisedDirectionVector[axis] > maxDistance)
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
	requestedSpeed = min<float>(max<float>(reqSpeed, move.MinMovementSpeed()),
								VectorBoxIntersection(normalisedDirectionVector, move.MaxFeedrates()));

	// On a Cartesian printer, it is OK to limit the X and Y speeds and accelerations independently, and in consequence to allow greater values
	// for diagonal moves. On other architectures, this is not OK and any movement in the XY plane should be limited on other ways.
	if (doMotorMapping)
	{
		k.LimitSpeedAndAcceleration(*this, normalisedDirectionVector, numVisibleAxes, flags.continuousRotationShortcut);	// give the kinematics the chance to further restrict the speed and acceleration
	}

	// 7. Calculate the provisional accelerate and decelerate distances and the top speed
#if SUPPORT_S_CURVE
	if (   prev->IsProvisional()													// if previous move is queued but has not started yet
		&& flags.isPrintingMove == prev->flags.isPrintingMove
		&& flags.xyMoving == prev->flags.xyMoving
		&& flags.isNonPrintingExtruderMove == prev->flags.isNonPrintingExtruderMove	// this is to prevent extruder-only moves being melded with Z-axis moves (issue 990)
	   )
	{
		// We may be able to meld this move with the previous one
		if (flags.isPrintingMove)
		{
			SetSpeedRatioForPrintingMoves(move);
		}
		else
		{
			beforePrepare.startSpeedRatio = 1.0;
			beforePrepare.maxPrevEndSpeed = min<float>(requestedSpeed, prev->requestedSpeed);
		}
	}
	else
	{
		// This will be the first move after standstill
		profile.endAcceleration = 0.0;												// end acceleration is zero until we have a following move
		beforePrepare.startSpeedRatio = 1.0;
		beforePrepare.maxPrevEndSpeed = 0.0;
	}
#endif

	profile.endSpeed = 0.0;															// until we have a following move

	MovementError rslt;																// this will hold the return value

	// See if we can meld this with the end of the previous one (which must currently have the end speed set to zero)
#if SUPPORT_S_CURVE
	if (flags.useScurve)
	{
		profile.startSpeed = profile.startAcceleration = 0.0;						// in case there is no previous move
		state = DDAState::created;													// postpone planning this move until preparation
		rslt = MovementError::ok;
	}
	else
	{
		if (beforePrepare.maxPrevEndSpeed > 0.0)
		{
			// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = -2as limited to the requested speed
			prev->beforePrepare.targetNextSpeed = min<float>(fastSqrtf(maxAcceleration * totalDistance * 2.0), beforePrepare.maxPrevEndSpeed);
			DoLookahead(ring, prev);
			profile.startSpeed = prev->profile.endSpeed * beforePrepare.startSpeedRatio;
		}
		else
		{
			profile.startSpeed = 0.0;
		}
#else
	{
		if (   prev->IsProvisional()													// if previous move has not started yet
			&& (   move.GetJerkPolicy() != 0											// and melding is allowed
				|| (   flags.isPrintingMove == prev->flags.isPrintingMove
					&& flags.xyMoving == prev->flags.xyMoving
					&& flags.isNonPrintingExtruderMove == prev->flags.isNonPrintingExtruderMove		// this is to prevent extruder-only move being melded with Z-axis moves (issue 990)
				   )
			   )
		   )
		{
			// Try to meld this move to the previous move to avoid stop/start
			// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = -2as limited to the requested speed
			prev->beforePrepare.targetNextSpeed = min<float>(fastSqrtf(maxAcceleration * totalDistance * 2.0), requestedSpeed);
			DoLookahead(ring, prev);
			profile.startSpeed = prev->profile.endSpeed;
		}
		else
		{
			profile.startSpeed = 0.0;													// there is no previous move that we can adjust, so start at zero speed.
		}
#endif

		rslt = RecalculateMove(ring);
		state = planned;
	}
	return rslt;
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

	const Move& move = reprap.GetMove();
	for (size_t driver = 0; driver < MaxDriversPerAxis; ++driver)
	{
		directionVector[driver] = adjustments[driver];			// for leadscrew adjustment moves, store the adjustment needed in directionVector
		const int32_t delta = lrintf(adjustments[driver] * move.DriveStepsPerMm(Z_AXIS));
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
	maxAcceleration = move.NormalAcceleration(Z_AXIS);

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
	profile.startSpeed = profile.endSpeed = 0.0;

	RecalculateMove(ring);
	state = planned;
	return true;
}

# if SUPPORT_ASYNC_MOVES

// Set up an async motor move returning true if the move does anything.
// All async moves are relative and linear.
bool DDA::InitAsyncMove(DDARing& ring, const AsyncMove& nextMove) noexcept
{
	// 1. Compute the new endpoints and the movement vector
	bool realMove = false;

	const Move& move = reprap.GetMove();
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		// Note, the correspondence between endCoordinates and endPoint will not be exact because of rounding error.
		// This doesn't matter for the current application because we don't use either of these fields.

		// If it's a delta then we can only do async tower moves in the Z direction and on any additional linear axes
		const size_t axisToUse = (move.GetKinematics().GetLegacyType() == KinematicsType::linearDelta && drive <= Z_AXIS) ? Z_AXIS : drive;
		directionVector[drive] = nextMove.movements[axisToUse];
		const int32_t delta = lrintf(nextMove.movements[axisToUse] * move.DriveStepsPerMm(drive));
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

	profile.startSpeed = nextMove.startSpeed;
	profile.endSpeed = nextMove.endSpeed;
	requestedSpeed = nextMove.requestedSpeed;
	maxAcceleration = nextMove.accelDecel;

# if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
# endif

	// Currently we normalise the vector sum of all motor movements to unit length.
	totalDistance = Normalise(directionVector);

	RecalculateMove(ring);
	state = planned;
	return true;
}

#endif

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
bool DDA::IsDecelerationMove() const noexcept
{
	return beforePrepare.decelDistance == totalDistance					// the simple case - is a deceleration-only move
			|| (profile.topSpeed < requestedSpeed						// can't have been intended as deceleration-only if it reaches the requested speed
				&& beforePrepare.decelDistance > 0.98 * totalDistance	// rounding error can only go so far
			   );
}

// Return true if this move is or might have been intended to be a deceleration-only move
// A move planned as a deceleration-only move may have a short acceleration segment at the start because of rounding error
bool DDA::IsAccelerationMove() const noexcept
{
	return beforePrepare.accelDistance == totalDistance					// the simple case - is an acceleration-only move
			|| (profile.topSpeed < requestedSpeed						// can't have been intended as deceleration-only if it reaches the requested speed
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
// Only called if this move and the next one (which we have just added) are both printing moves, or both non-printing moves.
/*static*/ void DDA::DoLookahead(DDARing& ring, DDA *laDDA) noexcept
//pre(state == provisional)
{
//	if (reprap.Debug(moduleDda)) debugPrintf("Adjusting, %f\n", laDDA->targetNextSpeed);
	unsigned int laDepth = 0;

	// Iterate through the list towards earlier moves
	for (;;)
	{
		// We have been asked to adjust the end speed of this move to match the next move starting at targetNextSpeed
		if (laDDA->beforePrepare.targetNextSpeed > laDDA->requestedSpeed)
		{
			laDDA->beforePrepare.targetNextSpeed = laDDA->requestedSpeed;			// don't try for an end speed higher than our requested speed
		}
		if (laDDA->profile.topSpeed >= laDDA->requestedSpeed)
		{
			// This move already reaches its top speed, so we just need to adjust the deceleration part
			break;																	// stop going back to previous moves
		}
		if (   laDDA->IsDecelerationMove()
			 && laDDA->prev->beforePrepare.decelDistance > 0.0						// if the previous move has no deceleration phase then no point in adjusting it
			)
		{
			// This is a deceleration-only move, and the previous one has a deceleration phase. We may have to adjust the previous move as well to get optimum behaviour.
			if (   laDDA->prev->IsProvisional()
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
				const float maxStartSpeed = fastSqrtf(fsquare(laDDA->beforePrepare.targetNextSpeed) + (2 * laDDA->maxAcceleration * laDDA->totalDistance));
				laDDA->prev->beforePrepare.targetNextSpeed = min<float>(maxStartSpeed, laDDA->requestedSpeed);

				// Still going up
				laDDA = _ecv_not_null(laDDA->prev);
				++laDepth;
				continue;
			}

			// This move is a deceleration-only move but we can't adjust the previous one
			if (laDDA->prev->state == committed)
			{
				laDDA->flags.hadLookaheadUnderrun = true;
			}
		}

		// This move doesn't reach its requested speed, but either it isn't a deceleration-only move or we can't adjust the previous one
		// Set its target end speed to the minimum of the requested speed and the highest we can reach
		const float maxReachableSpeed = fastSqrtf(fsquare(laDDA->profile.startSpeed) + (2 * laDDA->maxAcceleration * laDDA->totalDistance));
		if (laDDA->beforePrepare.targetNextSpeed > maxReachableSpeed)
		{
			laDDA->beforePrepare.targetNextSpeed = maxReachableSpeed;
		}
		break;
	}

	laDDA->MatchSpeeds();													// adjust the target end speed if necessary

	// Iterate back through the list towards later moves
	for (;;)
	{
		if (laDDA->beforePrepare.targetNextSpeed < laDDA->profile.endSpeed)
		{
			// This situation should not normally happen except by a small amount because of rounding error.
			// Don't reduce the end speed of the current move, because that may make the move infeasible.
			// Report a lookahead error if the change is too large to be accounted for by rounding error.
			if (laDDA->beforePrepare.targetNextSpeed < laDDA->profile.endSpeed * 0.99)
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
			laDDA->profile.endSpeed = laDDA->beforePrepare.targetNextSpeed;
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

		// Going back down the list
		// We have adjusted the end speed of the previous move as much as is possible. Adjust this move to match it.
		laDDA->profile.startSpeed = laDDA->prev->profile.endSpeed;
		const float maxEndSpeed = fastSqrtf(fsquare(laDDA->profile.startSpeed) + (2 * laDDA->maxAcceleration * laDDA->totalDistance));
		if (maxEndSpeed < laDDA->beforePrepare.targetNextSpeed)
		{
			laDDA->beforePrepare.targetNextSpeed = maxEndSpeed;
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
	while (cdda->prev->IsProvisional())
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
			const float maxBabySteppingAmount = cdda->totalDistance * min<float>(0.1, 0.5 * move.GetMaxInstantDv(Z_AXIS)/cdda->profile.topSpeed);
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
// Check that the move will execute in less than 2^31 step clocks and return MovementError::ok if so
MovementError DDA::RecalculateMove(DDARing& ring) noexcept
{
	const float twoA = 2 * maxAcceleration;
	beforePrepare.accelDistance = (fsquare(requestedSpeed) - fsquare(profile.startSpeed))/twoA;
	beforePrepare.decelDistance = (fsquare(requestedSpeed) - fsquare(profile.endSpeed))/twoA;
	if (beforePrepare.accelDistance + beforePrepare.decelDistance < totalDistance)
	{
		// This move reaches its top speed
		// It sometimes happens that we get a very short acceleration or deceleration segment. Remove any such segments by reducing the top speed to the start or end speed.
		// Don't do this if the cause is that the top speed is very low because that results in issues 989 and 994
		if (profile.startSpeed >= profile.endSpeed)
		{
			if (profile.startSpeed + maxAcceleration * MinimumAccelOrDecelClocks > requestedSpeed && profile.startSpeed >= requestedSpeed * 0.9)
			{
				profile.topSpeed = profile.startSpeed;
				beforePrepare.accelDistance = 0.0;
			}
			else
			{
				profile.topSpeed = requestedSpeed;
			}
		}
		else
		{
			if (profile.endSpeed + maxAcceleration * MinimumAccelOrDecelClocks > requestedSpeed && profile.endSpeed >= requestedSpeed * 0.9)
			{
				profile.topSpeed = profile.endSpeed;
				beforePrepare.decelDistance = 0.0;
			}
			else
			{
				profile.topSpeed = requestedSpeed;
			}
		}
	}
	else
	{
		// This move has no steady-speed phase, so it's accelerate-decelerate or accelerate-only or decelerate-only move.
		// If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - v^2)/2d = dist
		// So V^2(2a + 2d) = 2a.2d.dist + 2a.v^2 + 2d.u^2
		// So V^2 = (2a.2d.dist + 2a.v^2 + 2d.u^2)/(2a + 2d)
		// We now always set a == d so the above reduces to: V^2 = (2a.dist + v^2 + u^2)/2
		const float vsquared = ((twoA * totalDistance) + fsquare(profile.endSpeed) + fsquare(profile.startSpeed)) * 0.5;
		if (vsquared > fsquare(profile.startSpeed) && vsquared > fsquare(profile.endSpeed))
		{
			// It's an accelerate-decelerate move. Calculate accelerate distance from: V^2 = u^2 + 2as.
			beforePrepare.accelDistance = (vsquared - fsquare(profile.startSpeed))/twoA;
			beforePrepare.decelDistance = (vsquared - fsquare(profile.endSpeed))/twoA;
			profile.topSpeed = fastSqrtf(vsquared);
		}
		else
		{
			// It's an accelerate-only or decelerate-only move.
			// Due to rounding errors and babystepping adjustments, we may have to adjust the acceleration or deceleration slightly.
			// It's OK to adjust maxAcceleration because if we get here then we have either an acceleration or a deceleration segment, not both
			if (profile.startSpeed < profile.endSpeed)
			{
				beforePrepare.accelDistance = totalDistance;
				beforePrepare.decelDistance = 0.0;
				profile.topSpeed = profile.endSpeed;
				const float newAcceleration = (fsquare(profile.endSpeed) - fsquare(profile.startSpeed))/(2 * totalDistance);
				if (newAcceleration > 1.02 * maxAcceleration)
				{
					// The acceleration increase is greater than we expect from rounding error, so record an error
					ring.RecordLookaheadError();
					if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
					{
						debugPrintf("DDA.cpp(%d) na=%f", __LINE__, (double)newAcceleration);
						DebugPrint("rm");
					}
				}
				maxAcceleration = newAcceleration;
			}
			else
			{
				beforePrepare.accelDistance = 0.0;
				beforePrepare.decelDistance = totalDistance;
				profile.topSpeed = profile.startSpeed;
				const float newDeceleration = (fsquare(profile.startSpeed) - fsquare(profile.endSpeed))/(2 * totalDistance);
				if (newDeceleration > 1.02 * maxAcceleration)
				{
					// The deceleration increase is greater than we expect from rounding error, so record an error
					ring.RecordLookaheadError();
					if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
					{
						debugPrintf("DDA.cpp(%d) nd=%f", __LINE__, (double)newDeceleration);
						DebugPrint("rm");
					}
				}
				maxAcceleration = newDeceleration;
			}
		}
	}

	// Set up flags.canPauseAfter
	if (flags.canPauseAfter && profile.endSpeed != 0.0)
	{
		const Move& m = reprap.GetMove();
		for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
		{
			if (profile.endSpeed * fabsf(directionVector[drive]) > m.GetMaxInstantDv(drive))
			{
				flags.canPauseAfter = false;
				break;
			}
		}
	}

	// We need to set the number of clocks needed here because we use it before the move has been frozen
	const float totalTime = (2 * profile.topSpeed - profile.startSpeed - profile.endSpeed)/maxAcceleration
							+ (totalDistance - beforePrepare.accelDistance - beforePrepare.decelDistance)/profile.topSpeed;
	clocksNeeded = (uint32_t)totalTime;
	return (totalTime < std::numeric_limits<int32_t>::max() - 100) ? MovementError::ok : MovementError::move_duration_too_long;
}

#if SUPPORT_S_CURVE

// If the extrusion mix hasn't changed, calculate the feed rate ratio needed to maintain constant extrusion speed and the maximum end speed of the previous move
void DDA::SetSpeedRatioForPrintingMoves(const Move& move) noexcept
{
	float extrusionRatio = 1.0;
	bool found = false;
	for (size_t drive = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); drive < MaxAxesPlusExtruders; ++drive)
	{
		if (directionVector[drive] < 0.0 || prev->directionVector[drive] < 0.0)
		{
			beforePrepare.startSpeedRatio = beforePrepare.maxPrevEndSpeed = 0.0;
			return;
		}
		else if (prev->directionVector[drive] != 0.0)
		{
			const float tempRatio = directionVector[drive]/prev->directionVector[drive];
			if (tempRatio < 0.2)
			{
				beforePrepare.startSpeedRatio = beforePrepare.maxPrevEndSpeed = 0.0;
				return;
			}

			if (!found)
			{
				extrusionRatio = tempRatio;
				found = true;
			}
			else if (fabsf(tempRatio - extrusionRatio) > extrusionRatio * 0.01)		// require the mix to be unchanged between extruders to within 1%
			{
				beforePrepare.startSpeedRatio = beforePrepare.maxPrevEndSpeed = 0.0;
				return;
			}
		}
		else if (directionVector[drive] != 0.0)
		{
			beforePrepare.startSpeedRatio = beforePrepare.maxPrevEndSpeed = 0.0;
			return;
		}
	}

	// If we get here then the extrusion ratio hasn't changed significantly
	beforePrepare.startSpeedRatio = extrusionRatio;

	// Now calculate the maximum previous move end speed that doesn't exceed the jerk limit for any axis
	float provisionalMaxEndSpeed = min<float>(requestedSpeed/beforePrepare.startSpeedRatio, prev->requestedSpeed);
	for (size_t axis = 0; axis < reprap.GetGCodes().GetVisibleAxes(); ++axis)
	{
		const float axisDv = beforePrepare.startSpeedRatio * directionVector[axis] - prev->directionVector[axis];
		if (fabsf(provisionalMaxEndSpeed * axisDv) > move.GetMaxInstantDv(axis))
		{
			provisionalMaxEndSpeed = move.GetMaxInstantDv(axis)/axisDv;
		}
	}
	beforePrepare.maxPrevEndSpeed = provisionalMaxEndSpeed;
}

// Struct used by the next two functions
struct MultipleMoveParameters
{
	float peakAcceleration;
	float t0;
	float t1;
	float t2;
	float s0;
	float s1;
	float s2;
	float totalDistance;
};

// Calculate the desired profile of a series of moves that starts from a given start speed and start acceleration and finishes at a higher end speed and zero acceleration
// Returns true if the move is feasible, false if we can't slow down to the requested speed and zero acceleration from the starting conditions.
// Outputs are peakAcceleration (which is <= maxAcceleration), the durations of the increasing acceleration and steady acceleration phases, and the distance covered
//
// From the wxMaxima worksheet:
// 	v = u + ap * t1 + ap^2/j - a0^2/(2*j)
// where u = initial speed, v = final speed, a0 = initial acceleration, ap = peak acceleration, j = jerk, t1 = time spent at peak acceleration
//
// If t1 == 0 this reduces to:
// 	v = u + ap^2/j - a0^2/(2*j)
// so:
//	(v - u) * j = ap^2 - a0^2/2
// so:
//	ap = sqrt((v - u) * j + a0^2/2)
//
// 	s =  t1 * (u + OneHalf * ap * t1)
//	   + (u * (2 * ap - a0) + t1 * (ThreeHalves * ap^2 - OneHalf * a0^2))/j
//	   + (a1^3 - a0^2 * ap + OneThird * a0^3)/j^2
//
// if t1 == 0 this reduces to:
// 	s =  (u * (2 * ap - a0))/j
//	   + (a1^3 - a0^2 * ap + OneThird * a0^3)/j^2
//
static bool CalculateMultipleMoveProfile(float startSpeed, float endSpeed, float startAcceleration, float maxAcceleration, float jerk, MultipleMoveParameters& rslt) noexcept
pre(endSpeed > startSpeed; jerk > 0; startAcceleration > 0; maxAcceleration > 0; startAcceleration <= maxAcceleration)
{
	// First, determine whether the maximum acceleration is limiting
	if ((endSpeed - startSpeed) * jerk > fsquare(maxAcceleration) - 0.5 * fsquare(startAcceleration))
	{
		// We need a constant acceleration phase
		rslt.peakAcceleration = maxAcceleration;
		rslt.t0 = (maxAcceleration - startAcceleration)/jerk;
		rslt.t1 = ((endSpeed - startSpeed) * jerk - fsquare(maxAcceleration) + 0.5 * fsquare(startAcceleration))/(maxAcceleration * jerk);
		rslt.t2 = rslt.peakAcceleration/jerk;
		rslt.s0 = (maxAcceleration - startAcceleration) * (startSpeed/jerk + (fsquare(maxAcceleration - startAcceleration) + 3 * startAcceleration)/(6 * fsquare(jerk)));
		rslt.s1 = rslt.t1 * (startSpeed + (maxAcceleration - startAcceleration) * 0.5 * (maxAcceleration + startAcceleration)/jerk + 0.5 * maxAcceleration);
		rslt.s2 = maxAcceleration * ((startSpeed + maxAcceleration * rslt.t1)/jerk + (5 * fsquare(maxAcceleration) - 3 * fsquare(startAcceleration))/(6 * fsquare(jerk)));

		rslt.totalDistance = rslt.s0 + rslt.s1 + rslt.s2;
		return true;
	}
	else
	{
		// We don't need a constant acceleration phase
		rslt.peakAcceleration = fastSqrtf((endSpeed - startSpeed) * jerk + 0.5 * fsquare(startAcceleration));
		if (rslt.peakAcceleration >= startAcceleration)
		{
			rslt.t0 = (rslt.peakAcceleration - startAcceleration)/jerk;
			rslt.t1 = 0.0;
			rslt.t2 = rslt.peakAcceleration/jerk;
			rslt.s0 = (rslt.peakAcceleration - startAcceleration) * (startSpeed/jerk + (fsquare(rslt.peakAcceleration - startAcceleration) + 3 * startAcceleration)/(6 * fsquare(jerk)));
			rslt.s1 = 0.0;
			rslt.s2 = rslt.peakAcceleration * (startSpeed/jerk + (5 * fsquare(rslt.peakAcceleration) - 3 * fsquare(startAcceleration))/(6 * fsquare(jerk)));

			rslt.totalDistance = rslt.s0 + rslt.s2;
			return true;
		}
	}

	rslt.totalDistance = 0.0;				// to avoid gcc warning about uninitialised variables
	return false;
}

// Given a movement profile that is viable, distribute it over the moves
/*static*/ void DDA::DistributePlanOverMoves(DDA *startMove, DDA *endMove, float distanceToPlan, const MultipleMoveParameters& accelParams, const MultipleMoveParameters& decelParams) noexcept
{
	if (startMove == endMove)
	{
		// The simple case - just copy the distances across
		startMove->profile.distances[0] = accelParams.s0;
		startMove->profile.distances[1] = accelParams.s1;
		startMove->profile.distances[2] = accelParams.s2;
		startMove->profile.distances[4] = decelParams.s2;
		startMove->profile.distances[5] = decelParams.s1;
		startMove->profile.distances[6] = decelParams.s0;
		startMove->profile.distances[3] = startMove->totalDistance - accelParams.totalDistance - decelParams.totalDistance;
	}
	else
	{
		// Allocate the t0 acceleration phase
		float distanceLeftAccelerating;
		{
			float s0Left = accelParams.s0;
			while (true)
			{
				if (startMove->totalDistance > s0Left)
				{
					startMove->profile.distances[0] = s0Left;
					distanceLeftAccelerating = startMove->totalDistance - s0Left;
					break;
				}

				// This whole move is part of the t0 segment of the multiple move acceleration phase
				startMove->profile.distances[0] = startMove->totalDistance;
				startMove->profile.distances[1] = startMove->profile.distances[2] = startMove->profile.distances[3] = startMove->profile.distances[4] = startMove->profile.distances[5] = startMove->profile.distances[6] = 0.0;
				s0Left -= startMove->totalDistance;
				startMove->flags.fullyPlanned = true;
				startMove = startMove->next;
			}
		}

		// Allocate the t1 acceleration phase
		{
			float s1Left = accelParams.s1;
			while (true)
			{
				startMove->profile.peakAcceleration = accelParams.peakAcceleration;
				if (distanceLeftAccelerating > s1Left)
				{
					startMove->profile.distances[1] = s1Left;
					distanceLeftAccelerating -= s1Left;
					break;
				}

				// The rest of this move is part of the t1 segment of the multiple move acceleration phase
				startMove->profile.distances[1] = distanceLeftAccelerating;
				startMove->profile.distances[2] = startMove->profile.distances[3] = startMove->profile.distances[4] = startMove->profile.distances[5] = startMove->profile.distances[6] = 0.0;
				s1Left -= distanceLeftAccelerating;
				startMove->flags.fullyPlanned = true;
				startMove = startMove->next;
				startMove->profile.distances[0] = 0.0;
				distanceLeftAccelerating = startMove->totalDistance;
			}
		}

		// Allocate the t2 acceleration phase
		{
			float s2Left = accelParams.s2;
			while (true)
			{
				if (distanceLeftAccelerating > s2Left)
				{
					startMove->profile.distances[2] = s2Left;
					break;
				}

				// The rest of this move is part of the t2 segment of the multiple move acceleration phase
				startMove->profile.distances[2] = startMove->totalDistance;
				startMove->profile.distances[3] = startMove->profile.distances[4] = startMove->profile.distances[5] = startMove->profile.distances[6] = 0.0;
				s2Left -= startMove->totalDistance;
				startMove = startMove->next;
				startMove->profile.distances[0] = startMove->profile.distances[1] = 0.0;
				distanceLeftAccelerating = startMove->totalDistance;
			}
		}

		// Allocate the t6 deceleration phase. This is the reverse of how we allocate the t0 phase.
		float distanceLeftDecelerating;
		{
			float s6Left = decelParams.s0;
			while (true)
			{
				if (endMove->totalDistance > s6Left)
				{
					endMove->profile.distances[6] = s6Left;
					distanceLeftDecelerating = endMove->totalDistance - s6Left;
					break;
				}

				// This whole move is part of the t6 segment of the multiple move deceleration phase
				endMove->profile.distances[6] = endMove->totalDistance;
				endMove->profile.distances[0] = endMove->profile.distances[1] = endMove->profile.distances[2] = endMove->profile.distances[3] = endMove->profile.distances[4] = endMove->profile.distances[5] = 0.0;
				s6Left -= endMove->totalDistance;
				endMove = endMove->prev;
			}
		}

		// Allocate the t5 deceleration phase. This is the reverse of how we allocate the t1 phase.
		{
			float s5Left = decelParams.s1;
			while (true)
			{
				endMove->profile.peakDeceleration = -decelParams.peakAcceleration;
				if (distanceLeftDecelerating > s5Left)
				{
					endMove->profile.distances[5] = s5Left;
					distanceLeftDecelerating -= s5Left;
					break;
				}

				// The rest of this move is part of the t5 segment of the multiple move deceleration phase
				endMove->profile.distances[5] = s5Left;
				endMove->profile.distances[0] = endMove->profile.distances[1] = endMove->profile.distances[2] = endMove->profile.distances[3] = endMove->profile.distances[4] = 0.0;
				s5Left -= endMove->totalDistance;
				endMove = endMove->prev;
				endMove->profile.distances[6] = 0.0;
				distanceLeftDecelerating = endMove->totalDistance;
			}
		}

		// Allocate the t4 deceleration phase. This is the reverse of how we allocate the t2 phase.
		{
			float s4Left = decelParams.s2;
			while (true)
			{
				if (distanceLeftDecelerating > s4Left)
				{
					endMove->profile.distances[2] = s4Left;
					break;
				}

				// The rest of this move is part of the t4 segment of the multiple move deceleration phase
				endMove->profile.distances[4] = distanceLeftDecelerating;
				if (endMove == startMove)
				{
					// I think this situation might arise because of rounding error
					// TODO do we need to take any other action here?
					break;
				}
				endMove->profile.distances[0] = endMove->profile.distances[1] = endMove->profile.distances[2] = endMove->profile.distances[3] = 0.0;
				s4Left -= endMove->totalDistance;
				endMove = endMove->prev;
				endMove->profile.distances[5] = endMove->profile.distances[6] = 0.0;
				distanceLeftDecelerating = endMove->totalDistance;
			}
		}

		// Allocate the t3 steady speed phase
		if (startMove == endMove)
		{
			// Just add a steady speed segment in the middle to make up the distance.
			startMove->profile.distances[3] = startMove->totalDistance
						- (startMove->profile.distances[0] + startMove->profile.distances[1] + startMove->profile.distances[2] + startMove->profile.distances[4] + startMove->profile.distances[5] + startMove->profile.distances[6]);
		}
		else
		{
			startMove->profile.distances[3] = startMove->totalDistance - (startMove->profile.distances[0] + startMove->profile.distances[1] + startMove->profile.distances[2]);
			endMove->profile.distances[3] = endMove->totalDistance - (endMove->profile.distances[4] + endMove->profile.distances[5] + endMove->profile.distances[6]);
			while (startMove->next != endMove)
			{
				startMove = startMove->next;
				startMove->profile.distances[3] = startMove->totalDistance;
				startMove->profile.distances[0] = startMove->profile.distances[1] = startMove->profile.distances[2] = startMove->profile.distances[4] = startMove->profile.distances[5] = startMove->profile.distances[6] = 0.0;
			}
		}
	}
}

// Plan some moves that haven't yet been committed.
// 'firstUnpreparedMove' is the oldest uncommitted move.
// 'stopping' is true if we want to stop in a controlled manner as quickly as possible; else we have added one or more moves so we may be able to increase the speed of already-planned moves.
/*static*/ void DDA::PlanMoves(DDA *firstUnpreparedMove, bool stopping) noexcept
{
	// For now we ignore 'stopping'. Need to implement it when we add support for feed hold.
	// Find a sequence of moves that have approximately the same requestedSpeed and maxEndSpeed, apart from the last one which may have a lower or zero maxPrevEndSpeed
	DDA *lastMoveToPlan = firstUnpreparedMove;
	DDA *nextMove;
	float distanceToPlan = firstUnpreparedMove->totalDistance;
	float maxReqSpeed = firstUnpreparedMove->requestedSpeed;
	float minJerk = firstUnpreparedMove->jerk;
	float minMaxAcc = firstUnpreparedMove->maxAcceleration;
	while ((nextMove = lastMoveToPlan->next)->GetState() != DDA::empty && nextMove->beforePrepare.maxPrevEndSpeed != 0.0)
	{
		distanceToPlan += nextMove->totalDistance;
		if (nextMove->jerk < minJerk) { minJerk = nextMove->jerk; }
		if (nextMove->maxAcceleration < minMaxAcc) { minMaxAcc = nextMove->maxAcceleration; }
		if (nextMove->requestedSpeed > maxReqSpeed) { maxReqSpeed = nextMove->requestedSpeed; }
		lastMoveToPlan = nextMove;
	}
	lastMoveToPlan->profile.endSpeed = lastMoveToPlan->profile.endAcceleration = 0.0;

	// If the sequence comprises a single move and the start speed and acceleration are both zero (e.g. we are adding the first move), this is the simplest case
	if (lastMoveToPlan == firstUnpreparedMove && firstUnpreparedMove->profile.startSpeed == 0.0 && firstUnpreparedMove->profile.startAcceleration == 0.0)
	{
		firstUnpreparedMove->CalculateIsolatedSCurveMove();
	}
	else
	{
		// Calculate a profile for this collection of moves that accelerates to the peak speed and then decelerates to zero
		// The constraints are:
		// - the start speed and start acceleration
		// - the maximum allowed acceleration and deceleration. We use the minimum value we found in this collection of moves.
		// - the allowed jerk (rate of change of acceleration). We use the minimum value we found in this collection of moves.
		// - the peak allowed speed
		// - the end speed and end acceleration must be zero
		// This is easier if we can constrain the XY max acceleration and jerk to be constant and isotropic (not necessarily true when bed compensation is in use)
		// The parameters we can adjust are:
		// - the duration we increase acceleration (up to max acceleration)
		// - if we reach max acceleration, he duration we maintain it
		// - the duration we maintain the peak speed
		// Start by seeing how much distance we use up if we accelerate to the peak requested speed
		float viablePeakSpeed, unviablePeakSpeed, peakSpeedToTry = maxReqSpeed;
		float viableDistanceNeeded, unviableDistanceNeeded;
		unsigned int numIterations = 0;
		int errorLine;
		while (true)
		{
			MultipleMoveParameters accelParams;
			if (!CalculateMultipleMoveProfile(firstUnpreparedMove->profile.startSpeed, peakSpeedToTry, firstUnpreparedMove->profile.startAcceleration, minMaxAcc, minJerk, accelParams))
			{
				errorLine = __LINE__;
				break;
			}

			MultipleMoveParameters decelParams;
			if (!CalculateMultipleMoveProfile(lastMoveToPlan->profile.endSpeed, peakSpeedToTry, lastMoveToPlan->profile.endAcceleration, minMaxAcc, minJerk, decelParams))
			{
				errorLine = __LINE__;
				break;
			}

			const float distanceNeeded = accelParams.totalDistance + decelParams.totalDistance;
			if (distanceNeeded <= distanceToPlan)
			{
				// This plan is viable
				if (numIterations == 0 || peakSpeedToTry >= unviablePeakSpeed * 0.95)
				{
					debugPrintf("Solved in %u iterations, match = %.2f\n", numIterations, (double)((numIterations == 0) ? 1.0 : peakSpeedToTry/unviablePeakSpeed));
					DistributePlanOverMoves(firstUnpreparedMove, lastMoveToPlan, distanceToPlan, accelParams, decelParams);
					return;
				}
				else
				{
					viablePeakSpeed = peakSpeedToTry;
					viableDistanceNeeded = distanceNeeded;
				}
			}
			else
			{
				// Here if we have insufficient distance to reach the requested speed
				unviableDistanceNeeded = distanceNeeded;
				if (numIterations == 0)
				{
					const float minViableSpeedAccel = firstUnpreparedMove->profile.startSpeed + 0.5 * fsquare(firstUnpreparedMove->profile.startAcceleration)/minJerk;
					const float minViableSpeedDecel = lastMoveToPlan->profile.endSpeed + 0.5 * fsquare(lastMoveToPlan->profile.endAcceleration)/minJerk;
					viablePeakSpeed = max<float>(minViableSpeedAccel, minViableSpeedDecel);
					if (viablePeakSpeed > 0.0)
					{
						if (!CalculateMultipleMoveProfile(lastMoveToPlan->profile.endSpeed, viablePeakSpeed, lastMoveToPlan->profile.endAcceleration, minMaxAcc, minJerk, decelParams))
						{
							errorLine = __LINE__;
							break;
						}
						if (!CalculateMultipleMoveProfile(firstUnpreparedMove->profile.startSpeed, viablePeakSpeed, firstUnpreparedMove->profile.startAcceleration, minMaxAcc, minJerk, accelParams))
						{
							errorLine = __LINE__;
							break;
						}
						viableDistanceNeeded = accelParams.totalDistance + decelParams.totalDistance;
						if (viableDistanceNeeded > distanceToPlan)
						{
							errorLine = __LINE__;
							break;
						}
					}
					else
					{
						viableDistanceNeeded = 0.0;
					}
				}

				// Try a speed somewhere between the known viable and unviable peak speeds
				//TODO instead of doing a simple binary chop, use the distances needed and available to make a better guess
				unviablePeakSpeed = peakSpeedToTry;
			}
			peakSpeedToTry = 0.5 * (viablePeakSpeed + unviablePeakSpeed);
			debugPrintf("Distances: viable %.3g unviable %.3g available %.3g, speeds: viable %.3e unviable %.3e trying %.3e\n",
							(double)viableDistanceNeeded, (double)unviableDistanceNeeded, (double)distanceToPlan, (double)viablePeakSpeed, (double)unviablePeakSpeed, (double)peakSpeedToTry);
			++numIterations;
		}

		// Here if there is no viable movement profile that satisfies the constraints
		debugPrintf("Move profile calc failed at line %d\n", errorLine);
	}
}

// Calculate the move to be added to the ring when the start speed and acceleration and the end speed and acceleration are all zero
// Caller has already set endSpeed and endDeceleration to zero
// For an S-curve acceleration phase which starts at speed u and acceleration a, spends time t1 accelerating with jerk j to peak acceleration ap, then spends time t2 at constant acceleration ap, then spends time t1 reducing acceleration back to a:
//	s = u * (2 * t1 + t2) + a * (2 * t1^2 + 2 * t1 * t2 + ½ * t2^2) + j * (t1^3 + (3/2) * t1^2 * t2 + ½ * t1 * t2^2)
//	v = u + a * (2 * t1 + t2) + j * (t1 * t2 + t1^2)
//	ap = a + j * t1
// Given u = 0 and a = 0 for the move we are constructing:
//	s = j * (t1^3 + (3/2) * t1^2 * t2 + ½ * t1 * t2^2)
//	v = j * (t1 * t2 + t1^2) = j * t1 * (t1 + t2)
//	ap = j * t1
// The deceleration phase is a mirror image of the acceleration phase. We add a steady speed phase between acceleration and deceleration if we need more distance.
void DDA::CalculateIsolatedSCurveMove() noexcept
{
	profile.startAcceleration = profile.endAcceleration = 0.0;
	do
	{
		// Determine whether the requested speed or the maximum acceleration is more limiting
		// The acceleration reached from a standing start is a = j * t and the speed reached is v = 0.5 * j * t^2.
		// So a^2 = j^2 * t^2 = 2 * v * j
		// The phase in which the acceleration is reducing will increase the speed by the same amount. Therefore we can reach acceleration a without exceeding speed v if a^2 >= v * j.
		if (fsquare(maxAcceleration) > requestedSpeed * jerk)
		{
			// In principle we can reach the requested speed without exceeding the maximum acceleration, without having to include a constant acceleration segment
			profile.distances[1] = profile.distances[5] = 0.0;
			const motioncalc_t halfTimeToReqSpeed = fastSqrtf(requestedSpeed/jerk);
			const motioncalc_t distanceToReqSpeed = requestedSpeed * halfTimeToReqSpeed;
			if (2 * distanceToReqSpeed < totalDistance)
			{
				// We can reach the requested speed and decelerate to zero again without exceeding the required distance. Generate a 5-phase move.
				profile.distances[0] = profile.distances[6] = OneSixth * jerk * fcube(halfTimeToReqSpeed);
				profile.distances[2] = profile.distances[4] = (requestedSpeed * halfTimeToReqSpeed) - profile.distances[0];
				profile.distances[3] = totalDistance - 2 * distanceToReqSpeed;
				profile.topSpeed = requestedSpeed;
				profile.peakAcceleration = jerk * halfTimeToReqSpeed;
				break;
			}
			// Else we can't reach the requested speed without exceeding required distance, or we can only just reach it and then we need to start decelerating immediately. Fall through to beyond the else-part of this if-statement.
		}
		else
		{
			// We can't reach the requested speed without inserting a constant acceleration segment to avoid exceeding maximum acceleration
			profile.peakAcceleration = maxAcceleration;
			const motioncalc_t basicDistance = 2 * fcube(maxAcceleration)/fsquare(jerk);	// distance if we reach max acceleration but have no constant acceleration segment
			if (basicDistance < totalDistance)
			{
				// We need to insert a constant acceleration segment. We may also need to limit the top speed.
				// Calculate t1 in the above equations
				// From the above equations:	t2^2 * (0.5 * t1) + t2 * (1.5 * t1^2) + (t1^3 - s/j) = 0
				// Solve for t2 to get:			t2 = [-1.5 * t1^2 +/- sqrt(2.25 * t1^4 - 4 * 0.5 * t1 * (t1^3 - s/j))]/t1
				// Rearrange:					t2 = -1.5 * t1 +/- sqrt(2.25 * t1^2 - 2 * (t1^2 - s/(j*t1))
				// Simplify:					t2 = -1.5 * t1 +/- sqrt(0.25 * t1^2 + 2 * s/(j*t1))
				// But j * t1 = ap, therefore:	t2 = -1.5 * t1 +/- sqrt(0.25 * t1^2 + 2 * s/ap)
				// s is half the total distance because we accelerate and decelerate again.
				const motioncalc_t timeToMaxAcceleration = maxAcceleration/jerk;
				profile.distances[0] = profile.distances[6] = OneSixth * jerk * fcube(timeToMaxAcceleration);
				const float constantAccelerationTime = -1.5 * timeToMaxAcceleration + fastSqrtf(0.25 * fsquare(timeToMaxAcceleration) + totalDistance/maxAcceleration);
				const float newTopSpeed = jerk * timeToMaxAcceleration * (timeToMaxAcceleration + constantAccelerationTime);
				if (newTopSpeed <= requestedSpeed)
				{
					// Generate a 6-phase move. The middle 2 phases could be combined.
					profile.topSpeed = newTopSpeed;
					profile.distances[1] = profile.distances[5] = 0.5 * constantAccelerationTime * profile.topSpeed;
					profile.distances[2] = profile.distances[4] = timeToMaxAcceleration * profile.topSpeed - profile.distances[0];
					profile.distances[3] = 0.0;
				}
				else
				{
					// We need to limit the constant acceleration time in order to limit the top speed, and add a constant speed phase. Generate a 7-phase move.
					profile.topSpeed = requestedSpeed;
					//	v = j * t1 * (t1 + t2) therefore t2 = v/(j * t1) - t1
					const motioncalc_t revisedConstantAccelerationTime = requestedSpeed/(jerk * timeToMaxAcceleration) - timeToMaxAcceleration;
					profile.distances[1] = profile.distances[5] = 0.5 * revisedConstantAccelerationTime * profile.topSpeed;
					profile.distances[2] = profile.distances[4] = timeToMaxAcceleration * profile.topSpeed - profile.distances[0];
					profile.distances[3] = totalDistance - 2 * (profile.distances[0] + profile.distances[2]);
				}
				break;
			}
			// Else fall through
		}

		// If we get here then we can reach neither requestedSpeed nor maxAcceleration without exceeding totalDistance. Generate a 4-phase move. The middle 2 phases could be combined.
		const motioncalc_t halfTimeToTopSpeed = fastCubeRootf(totalDistance * 0.5 / jerk);
		profile.distances[0] = profile.distances[6] = OneTwelfth * totalDistance;
		profile.distances[2] = profile.distances[4] = 0.5 * totalDistance - profile.distances[0];
		profile.distances[1] = profile.distances[3] = profile.distances[5] = 0.0;
		profile.topSpeed = jerk * fsquare(halfTimeToTopSpeed);
		profile.peakAcceleration = jerk * halfTimeToTopSpeed;
	} while (false);

	profile.peakDeceleration = -profile.peakAcceleration;
	flags.canPauseAfter = true;
}

#if 0	// we are probably not going to use this

// Add a new S-curve move to the ring when there is already at least one move there and we would like to meld them
// Returns 0 if successful and we are ready to do lookahead, else the line number at which a problem was detected
// Caller has already set endSpeed and endDeceleration to zero
int DDA::CalculateNewSCurveMove() noexcept
{
	// Calculate the ideal speed to transition from this move to the next. This may be limited by the following:
	// - our own requested speed
	// - the requested speed of the next move
	// - if the direction is changing, the instantaneous speed change limits of the axes involved
	float idealStartSpeed = min<float>(requestedSpeed, prev->requestedSpeed);

	// The following loop is similar to MatchSpeeds but it works on the previous move instead of the next. We already checked that the extrusion speeds match.
	for (size_t drive = 0; drive < reprap.GetGCodes().GetTotalAxes(); ++drive)
	{
		if (directionVector[drive] != 0.0 || next->directionVector[drive] != 0.0)
		{
			const float totalFraction = fabsf(directionVector[drive] - prev->directionVector[drive]);
			const float instantDv = totalFraction * idealStartSpeed;
			const float allowedInstantDv = reprap.GetMove().GetPrintingInstantDv(drive);
			if (instantDv > allowedInstantDv)
			{
				idealStartSpeed = allowedInstantDv/totalFraction;
			}
		}
	}

	startAcceleration = peakAcceleration = finalAcceleration = 0.0;				// we are not expecting an acceleration phase

	// Can we decelerate from idealEndSpeed and zero acceleration to standstill without exceeding distance?
	// First check whether we would need to include a constant deceleration segment in order to avoid exceeding the acceleration limit.
	// The acceleration reached from a standing start is a = j * t and the speed reached is v = 0.5 * j * t^2.
	// So a^2 = j^2 * t^2 = 2 * v * j
	// The phase in which the deceleration is reducing will reduce the speed by the same amount. Therefore we can reach deceleration a without exceeding speed v if a^2 >= v * j.
	if (fsquare(maxDeceleration) > idealStartSpeed * jerk)
	{
		// In principle we can decelerate from the requested speed of the next move without exceeding the maximum deceleration, without having to include a constant deceleration segment.
		// Would such a movement exceed the distance?
		const float t1 = fastSqrtf(idealStartSpeed/jerk);
		const float distanceFromIdealStartSpeed = idealStartSpeed * t1;
		if (distanceFromIdealStartSpeed <= totalDistance)
		{
			// We can decelerate from the ideal start speed and zero acceleration to zero/zero without exceeding the required distance.
			prev->beforePrepare.targetNextSpeed = startSpeed = topSpeed = idealStartSpeed;
			prev->beforePrepare.targetNextAcceleration = initialDeceleration = 0.0;

			// This is the 3-phase move we will generate if the proposal is accepted
			beforePrepare.phase0Time = beforePrepare.phase1Time = beforePrepare.phase2Time = beforePrepare.phase5Time = 0;
			beforePrepare.phase4Time = beforePrepare.phase6Time = t1;
			beforePrepare.phase3Time = (totalDistance - distanceFromIdealStartSpeed)/idealStartSpeed;
			peakDeceleration = jerk * t1;
		}
		else
		{
			// We can't decelerate from idealStartSpeed/zero to zero/zero without exceeding this move distance.
			// See what speed/acceleration we can decelerate from.
			const float distanceFromPeakDeceleration = OneSixth * jerk * fcube(t1);
			if (distanceFromPeakDeceleration <= totalDistance)
			{
				// We can execute all of the second part of the S and probably part of the first
				const float residualDistance = totalDistance = distanceFromPeakDeceleration;
				const float speedBeforeReducingDeceleration = 0.5 * idealStartSpeed;
				const float decelBeforeReducingDeceleration = t1 * jerk;
				const float t2 = SmallestNonNegativeCubicSolution(-jerk, 3 * decelBeforeReducingDeceleration, 6 * speedBeforeReducingDeceleration, -6 * residualDistance);
				if (std::isnan(t2))
				{
					return __LINE__;
				}
				else
				{
					prev->beforePrepare.targetNextSpeed = startSpeed = topSpeed = speedBeforeReducingDeceleration + (decelBeforeReducingDeceleration - 0.5 * jerk * t2) * t2;
					prev->beforePrepare.targetNextAcceleration = initialDeceleration = -(decelBeforeReducingDeceleration + jerk * t2);

					// This is the 2-phase move we will generate if the proposal is accepted. Save the values to avoid solving the equations again.
					beforePrepare.phase0Time = beforePrepare.phase1Time = beforePrepare.phase2Time = beforePrepare.phase3Time = beforePrepare.phase5Time = 0;
					beforePrepare.phase4Time = t2;
					beforePrepare.phase6Time = t1;
					peakDeceleration = -decelBeforeReducingDeceleration;
				}
			}
			else
			{
				// We can't execute all of the second part of the S
				const float timeToReachDistance = fastCubeRootf(6.0 * totalDistance/jerk);
				const float tempPeakDeceleration = jerk * timeToReachDistance;
				const float tempPeakSpeed = 0.5 * tempPeakDeceleration * timeToReachDistance;
				prev->beforePrepare.targetNextSpeed = startSpeed = topSpeed = tempPeakSpeed;
				prev->beforePrepare.targetNextAcceleration = initialDeceleration = peakDeceleration = -tempPeakDeceleration;

				// This is the 1-phase move we will generate if the proposal is accepted. Save the values to avoid solving the equations again.
				beforePrepare.phase0Time = beforePrepare.phase1Time = beforePrepare.phase2Time = beforePrepare.phase3Time = beforePrepare.phase4Time = beforePrepare.phase5Time = 0;
				beforePrepare.phase6Time = timeToReachDistance;
			}
		}
	}
	else
	{
		// Decelerating from idealStartSpeed/zero to zero/zero requires a constant deceleration segment
		// If a is the max deceleration then the speed change in variable deceleration segments is 2 * (a^2/2*j) = a^2/j
		// So the speed change required in the constant decel segment is idealStartSpeed - a^2/j
		// So the time that this segment takes is idealStartSpeed/a - a/j
		const float t1 = maxDeceleration/jerk;
		const float t2 = idealStartSpeed/maxDeceleration - t1;
		const float distanceFromIdealStartSpeed = jerk * t1 * (t1 * (t1 + 1.5 * t2) + 0.5 * fsquare(t2));
		if (distanceFromIdealStartSpeed <= totalDistance)
		{
			// We can decelerate from the ideal start speed and zero acceleration to zero/zero without exceeding the required distance.
			prev->beforePrepare.targetNextSpeed = startSpeed = topSpeed = idealStartSpeed;
			prev->beforePrepare.targetNextAcceleration = initialDeceleration = 0.0;

			// This is the 4-phase move we can generate if the proposal is accepted. Save the values to avoid solving the equations again.
			beforePrepare.phase0Time = beforePrepare.phase1Time = beforePrepare.phase2Time = 0;
			beforePrepare.phase4Time = beforePrepare.phase6Time = t1;
			beforePrepare.phase5Time = t2;
			beforePrepare.phase3Time = (totalDistance - distanceFromIdealStartSpeed)/idealStartSpeed;
			peakDeceleration = maxDeceleration;
		}
		else
		{
			// We can't decelerate from idealStartSpeed/zero to zero/zero without exceeding this move distance.
			// See what speed/acceleration we can decelerate from.
			const float distanceFromPeakDeceleration = OneSixth * jerk * fcube(t1);
			if (totalDistance < distanceFromPeakDeceleration)
			{
				// We can't execute the whole of the final part of the S
				const float timeToReachDistance = fastCubeRootf(6.0 * totalDistance/jerk);
				const float tempPeakDeceleration = jerk * timeToReachDistance;
				const float tempPeakSpeed = 0.5 * tempPeakDeceleration * timeToReachDistance;
				prev->beforePrepare.targetNextSpeed = startSpeed = topSpeed = tempPeakSpeed;
				prev->beforePrepare.targetNextAcceleration = initialDeceleration = peakDeceleration = -tempPeakDeceleration;

				// This is the 1-phase move we will generate if the proposal is accepted. Save the values to avoid solving the equations again.
				beforePrepare.phase0Time = beforePrepare.phase1Time = beforePrepare.phase2Time = beforePrepare.phase3Time = beforePrepare.phase4Time = beforePrepare.phase5Time = 0;
				beforePrepare.phase6Time = timeToReachDistance;
			}
			else
			{
				// We can execute all of the final part of the S. Can we execute all of the constant speed segment too?
				const float speedBeforeReducingDeceleration = 0.5 * maxDeceleration * t1;
				const float distanceAtPeakDeceleration = (speedBeforeReducingDeceleration + 0.5 * maxDeceleration * t2) * t2;
				const float distanceLeft = totalDistance - distanceFromPeakDeceleration - distanceAtPeakDeceleration;
				if (distanceLeft < 0.0)
				{
					// We can't execute all of the constant speed segment.
					const float distanceAvailable = totalDistance - distanceFromPeakDeceleration;
					const float t2a = (-speedBeforeReducingDeceleration + fastSqrtf(fsquare(speedBeforeReducingDeceleration) + 2 * maxDeceleration * distanceAvailable))/maxDeceleration;
					prev->beforePrepare.targetNextSpeed = startSpeed = topSpeed = speedBeforeReducingDeceleration + maxDeceleration * t2a;
					prev->beforePrepare.targetNextAcceleration = initialDeceleration = peakDeceleration = -maxDeceleration;

					// This is the 2-phase move we can generate of the proposal is accepted. Save the values to avoid solving the equations again.
					beforePrepare.phase0Time = beforePrepare.phase1Time = beforePrepare.phase2Time = beforePrepare.phase3Time = beforePrepare.phase4Time = 0;
					beforePrepare.phase5Time = t2a;
					beforePrepare.phase6Time = t1;
				}
				else
				{
					// We can execute all of the constant speed segment. See how much of the increasing-deceleration segment we can generate.
					const float speedAtStartOfConstantDeceleration = speedBeforeReducingDeceleration + t2 * maxDeceleration;
					const float t3 = SmallestNonNegativeCubicSolution(-jerk, -3 * maxDeceleration, 6 * speedAtStartOfConstantDeceleration, -6 * distanceLeft);
					if (std::isnan(t3))
					{
						return __LINE__;
					}
					else
					{
						prev->beforePrepare.targetNextSpeed = startSpeed = topSpeed = speedAtStartOfConstantDeceleration + (maxDeceleration - OneHalf * jerk * t3) * t3;
						prev->beforePrepare.targetNextAcceleration = initialDeceleration = -maxDeceleration + jerk * t3;

						// This is the 3-phase move we can generate of the proposal is accepted. Save the values to avoid solving the equations again.
						beforePrepare.phase0Time = beforePrepare.phase1Time = beforePrepare.phase2Time = beforePrepare.phase3Time = 0;
						beforePrepare.phase4Time = t3;
						beforePrepare.phase5Time = t2;
						beforePrepare.phase6Time = t1;
						peakDeceleration = -maxDeceleration;
					}
				}
			}
		}
	}
	return 0;
}

// Try to smooth out moves in the queue.
// laDDA is the move that we want to adjust. We have already set laDDA->beforePrepare.targetNextSpeed and laDDA->beforePrepare.targetNextAcceleration to the values that the following move would like to start at.
/*static*/ MovementError DDA::DoSCurveLookahead(DDARing& ring, DDA *laDDA) noexcept
{
	laDDA->next->DebugPrint("DDA: ");
	debugPrintf(" TNS %.3e, TNA %.3e\n", (double)laDDA->beforePrepare.targetNextSpeed, (double)laDDA->beforePrepare.targetNextAcceleration);

	unsigned int laDepth = 0;
	bool goingUp = true;
	while (true)
	{
		if (goingUp)
		{
			// We are iterating in the direction of older moves
			// See whether we can adjust this move to end at the speed and acceleration requested by the following move.
			// That requested speed won't be higher than our own requestedSpeed.
			// Check that the acceleration is within limits. TODO: should we have the following move ensure this in advance?
			if (laDDA->beforePrepare.targetNextAcceleration > 0.0)
			{
				if (laDDA->beforePrepare.targetNextAcceleration > laDDA->maxAcceleration)
				{
					laDDA->beforePrepare.targetNextAcceleration = laDDA->maxAcceleration;
					laDDA->flags.haveReducedAcceleration = true;								// tell the following move that we need to reduce acceleration
					goingUp = false;
					continue;
				}
			}
			else if (-laDDA->beforePrepare.targetNextAcceleration > laDDA->maxDeceleration)
			{
				laDDA->beforePrepare.targetNextAcceleration = -laDDA->maxDeceleration;
				laDDA->flags.haveReducedAcceleration = true;									// tell the following move that we need to reduce acceleration
				goingUp = false;
				continue;
			}

			// If we already reach our requested speed then see if we can reach the requested end speed and acceleration from it
			if (laDDA->topSpeed == laDDA->requestedSpeed)
			{
				// Can we go from top speed and zero acceleration to the requested speed and acceleration without exceeding jerk?
				if ((laDDA->topSpeed - laDDA->beforePrepare.targetNextSpeed) * laDDA->jerk * 2 > fsquare(laDDA->beforePrepare.targetNextAcceleration))
				{
					// No, so we need to end at a lower speed or a lower acceleration. Ask for a lower acceleration.
					laDDA->beforePrepare.targetNextAcceleration = fastSqrtf((laDDA->topSpeed - laDDA->beforePrepare.targetNextSpeed)/(2 * laDDA->jerk));
					laDDA->flags.haveReducedAcceleration = true;
					goingUp = false;
					continue;
				}

				// We must be at our top speed at the end of phase 3, during the whole of phase 4, and at the start of phase 5
				// Calculate the total distance remaining after any acceleration segments
				float distanceAvailable = laDDA->totalDistance;
				if (laDDA->beforePrepare.phase0Time != 0)
				{
					distanceAvailable -= (laDDA->startSpeed + (OneHalf * laDDA->startAcceleration + OneSixth * laDDA->jerk* laDDA->beforePrepare.phase0Time) * laDDA->beforePrepare.phase0Time) * laDDA->beforePrepare.phase0Time;
				}
				if (laDDA->beforePrepare.phase1Time != 0)
				{
					distanceAvailable -= (laDDA->startSpeed + (laDDA->startAcceleration + OneHalf * (laDDA->maxAcceleration + laDDA->jerk * laDDA->beforePrepare.phase1Time) * laDDA->beforePrepare.phase1Time)) * laDDA->beforePrepare.phase1Time;
				}

				// Phase 3 must end with zero acceleration. It may be absent if phases 1 and 2 are also absent.
				if (laDDA->beforePrepare.phase2Time != 0)
				{
					distanceAvailable -= (laDDA->topSpeed - OneSixth * laDDA->jerk * fsquare(laDDA->beforePrepare.phase2Time)) * laDDA->beforePrepare.phase2Time;
				}

				// Do we have enough distance available to decelerate to the requested values?
				// To reach the requested acceleration, t = requested_acc/max_jerk
				const float phase4Time = laDDA->beforePrepare.targetNextAcceleration/(-laDDA->jerk);
				const float distances[5] = (laDDA->topSpeed + OneSixth * (laDDA->beforePrepare.targetNextAcceleration * phase4Time)) * phase4Time;
				if (distances[5] <= distanceAvailable)
				{
					const float phase5EndSpeed = laDDA->topSpeed + OneHalf * (laDDA->beforePrepare.targetNextAcceleration * phase4Time);
					const float phase5Time = max<float>(0.0, (laDDA->beforePrepare.targetNextSpeed - phase5EndSpeed)/laDDA->beforePrepare.targetNextAcceleration);
					const float distances[6] = (phase5EndSpeed + OneHalf * (laDDA->beforePrepare.targetNextSpeed - phase5EndSpeed)) * phase5Time;
					const float distances[4] = distanceAvailable - (distances[5] + distances[6]);
					if (distances[4] >= 0)
					{
						// We do have enough distance
						laDDA->beforePrepare.phase4Time = phase4Time;
						laDDA->beforePrepare.phase5Time = phase5Time;
						laDDA->beforePrepare.phase3Time = distances[4]/laDDA->topSpeed;
						goingUp = false;
						continue;
					}
				}

				// Not enough distance to decelerate to the requested values, so we need to reduce laDDA->topSpeed
				// This should only occur if we reduce the requested start acceleration and/or start speed of the next move, which should not normally happen (but might when we introduce feed hold)
				qq;
			}

			// Currently, laDDA does not reach its requested speed. We may be able to increase its top speed if that would be helpful, but to do that we may need to adjust the previous move.
			// Calculate the top speed that we would like to start the deceleration from.
			// There are at least the following cases:
			// 1. If the previous move is already committed, we can't ask it to change its ending speed or acceleration anyway.
			// 2. Starting from targetNextSpeed and targetNextAcceleration, if we start reducing deceleration immediately then we will exceed our requested speed before we run out of distance.
			// 3. Starting from targetNextSpeed and targetNextAcceleration, if we start reducing deceleration immediately then we will run out of distance before we reach our requested speed.
			// 4. Starting from targetNextSpeed and targetNextAcceleration, if we start reducing deceleration immediately then we won't reach our requested speed, so we may need to introduce a constant acceleration segment
			qq;
		}
		else
		{
			// We are iterating towards newer moves
			qq;
			if (laDepth == 0) { return MovementError::ok; }		//TODO check move duration
		}
	}
}

#endif
#endif

// Decide what speed we would really like this move to end at and the next move to start at, assuming we want to use the same speed for both.
// On entry, targetNextSpeed is the speed we would like the next move after this one to start at and this one to end at
// On return, targetNextSpeed is the actual speed we can achieve without exceeding the jerk limits.
void DDA::MatchSpeeds() noexcept
{
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
	{
		if (directionVector[drive] != 0.0 || next->directionVector[drive] != 0.0)
		{
			const float totalFraction = fabsf(directionVector[drive] - next->directionVector[drive]);
			const float instantDv = totalFraction * beforePrepare.targetNextSpeed;
			const float allowedInstantDv = reprap.GetMove().GetPrintingInstantDv(drive);
			if (instantDv > allowedInstantDv)
			{
				beforePrepare.targetNextSpeed = allowedInstantDv/totalFraction;
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
void DDA::Prepare(DDARing& ring, uint32_t prepareAdvanceTime, SimulationMode simMode) noexcept
{
#if SUPPORT_LASER
	if (profile.topSpeed < requestedSpeed && reprap.GetGCodes().GetMachineType() == MachineType::laser)
	{
		// Scale back the laser power according to the actual speed
		laserPwmOrIoBits.laserPwm = (Pwm_t)((laserPwmOrIoBits.laserPwm * profile.topSpeed)/requestedSpeed);
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
		else if (profile.startSpeed == 0.0)
		{
			afterPrepare.moveStartTime = now + prepareAdvanceTime;
		}
		else
		{
			afterPrepare.moveStartTime = now + MoveTiming::AbsoluteMinimumPreparedTime;
			reprap.GetMove().AddPrepareHiccup();		// move was supposed to follow the previous one directly, so record a hiccup
		}
	}
	else
	{
		afterPrepare.moveStartTime = now + prepareAdvanceTime;
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
							move.AddLinearSegments(driver.localDriver + MaxAxesPlusExtruders, afterPrepare.moveStartTime, params, (motioncalc_t)delta, segFlags);
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
						move.AddLinearSegments(drive, afterPrepare.moveStartTime, params, (motioncalc_t)delta, segFlags);
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
							move.AddLinearSegments(drive, afterPrepare.moveStartTime, params, delta, segFlags.AddIsExtruder());

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
			BasePriorityBooster booster(NvicPriorityStep);								// shut out the step interrupt
			(void)move.CheckEndstops(false);											// this may modify pending CAN moves
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

void DDA::LimitSpeedAndAcceleration(float maxSpeed, float maxAllowedAcceleration) noexcept
{
	if (requestedSpeed > maxSpeed)
	{
		requestedSpeed = maxSpeed;
	}
	if (maxAcceleration > maxAllowedAcceleration)
	{
		maxAcceleration = maxAllowedAcceleration;
	}
}

float DDA::GetTotalExtrusionRate() const noexcept
{
	float fraction = 0.0;
	for (size_t i = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); i < MaxAxesPlusExtruders; ++i)
	{
		fraction += directionVector[i];
	}
	return fraction * InverseConvertSpeedToMmPerSec(profile.topSpeed);
}

#if SUPPORT_LASER

// Manage the laser power. Return the number of ticks until we should be called again, or portMAX_DELAY to be called at the start of the next move.
uint32_t DDA::ManageLaserPower(Platform& p) const noexcept
{
	const uint32_t clocksMoving = StepTimer::GetMovementTimerTicks() - afterPrepare.moveStartTime;
	if (clocksMoving >= clocksNeeded)			// this also covers the case of now < startTime
	{
		// Something has gone wrong with the timing. Set zero laser power, but try again soon.
		p.SetLaserPwm(0);
		return LaserPwmIntervalMillis;
	}

	const uint32_t clocksLeft = clocksNeeded - clocksMoving;
	if (!flags.controlLaserOrIoBits || laserPwmOrIoBits.laserPwm == 0)
	{
		p.SetLaserPwm(0);
		return (uint32_t)lrintf((float)clocksLeft * StepClocksToMillis);
	}

	const float accelSpeed = profile.startSpeed + maxAcceleration * clocksMoving;
	if (accelSpeed < profile.topSpeed)
	{
		// Acceleration phase
		const Pwm_t pwm = (Pwm_t)((accelSpeed/profile.topSpeed) * laserPwmOrIoBits.laserPwm);
		p.SetLaserPwm(pwm);
		return LaserPwmIntervalMillis;
	}

	const float decelSpeed = profile.endSpeed + maxAcceleration * clocksLeft;
	if (decelSpeed < profile.topSpeed)
	{
		// Deceleration phase
		const Pwm_t pwm = (Pwm_t)((decelSpeed/profile.topSpeed) * laserPwmOrIoBits.laserPwm);
		p.SetLaserPwm(pwm);
		return LaserPwmIntervalMillis;
	}

	// We must be in the constant speed phase
	p.SetLaserPwm(laserPwmOrIoBits.laserPwm);
	const uint32_t decelClocks = (uint32_t)((profile.topSpeed - profile.endSpeed)/maxAcceleration);
	if (clocksLeft <= decelClocks)
	{
		return LaserPwmIntervalMillis;
	}
	const uint32_t clocksToDecel = clocksLeft - decelClocks;
	return (uint32_t)lrintf((float)clocksToDecel * StepClocksToMillis) + LaserPwmIntervalMillis;
}

#endif

// End
