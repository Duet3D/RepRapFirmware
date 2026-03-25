/*
 * DDA_3rdOrder.cpp
 *
 *  Created on: 2 Sept 2025
 *      Author: David
 *
 *  This file contains functions used only to implement 3rd order motion control
 */

#include "DDA.h"

#if SUPPORT_S_CURVE

#include "DDARing.h"
#include "MovementProfile.h"
#include "MoveDebugFlags.h"
#include <Platform/RepRap.h>
#include "Move.h"
#include <GCodes/GCodes.h>

#define COPY_PLAN	(0)				// if enabled, keep a copy of the plan for debugging

// Convert a float to a uint32_t, with negative values converted to zero
static inline uint32_t doubleToU32(double& f, int line) noexcept
{
	// Check for invalid duration, for debugging
	if (std::isnan(f) || std::signbit(f))
	{
		debugPrintf("Calculated duration at line %d: %.4g\n", line, f);
		f = (double)0.0;
		return 0;
	}
	return (uint32_t)(f + (double)0.5);
}

// If the extrusion mix hasn't changed, calculate the feed rate ratio needed to maintain constant extrusion speed and the maximum end speed of the previous move
void DDA::SetSpeedRatioAndMaxJunctionSpeedForPrintingMoves(const Move& move) noexcept
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

void DDA::SetSpeedRatioAndMaxJunctionSpeedForNonPrintingMoves(const Move& move) noexcept
{
	beforePrepare.startSpeedRatio = 1.0;

	// Now calculate the maximum previous move end speed that doesn't exceed the jerk limit for any axis
	float provisionalMaxEndSpeed = min<float>(requestedSpeed, prev->requestedSpeed);
	for (size_t axis = 0; axis < reprap.GetGCodes().GetVisibleAxes(); ++axis)
	{
		const float axisDv = directionVector[axis] - prev->directionVector[axis];
		if (fabsf(provisionalMaxEndSpeed * axisDv) > move.GetMaxInstantDv(axis))
		{
			provisionalMaxEndSpeed = move.GetMaxInstantDv(axis)/axisDv;
		}
	}
	beforePrepare.maxPrevEndSpeed = provisionalMaxEndSpeed;
}

#if COPY_PLAN	//DEBUG
// WARNING: this variable is static, so it's not going to work with multiple motion systems.
// However, it's for debugging only, so as long as we only run one motion system while debugging with COPY_PLAN set then that's OK.
static MovementProfile debugProfile;
#endif

// Plan some moves that haven't yet been committed and store the plan in plannedProfile.
// 'firstUnpreparedMove' is the oldest uncommitted move.
// 'stopping' is true if we want to stop in a controlled manner as quickly as possible; else we have added one or more moves so we may be able to increase the speed of already-planned moves.
/*static*/ void DDA::PlanMoves(DDA *firstUnpreparedMove, MovementProfile& plannedProfile, bool stopping) noexcept
{
	// For now we ignore 'stopping'. Need to implement it when we add support for feed hold.
	// Find a sequence of moves that have approximately the same requestedSpeed and maxEndSpeed, apart from the last one which may have a lower or zero maxPrevEndSpeed
	DDA *lastMoveToPlan = firstUnpreparedMove;
	DDA *nextMove;
	double distanceToPlan = (double)(firstUnpreparedMove->totalDistance * firstUnpreparedMove->movementRatio);
	float fminReqSpeed = firstUnpreparedMove->requestedSpeed * firstUnpreparedMove->movementRatio;
	float maxReqSpeedForPlan = fminReqSpeed;
	float fminJerk = firstUnpreparedMove->jerk * firstUnpreparedMove->movementRatio;
	float fminMaxAcc = firstUnpreparedMove->maxAcceleration * firstUnpreparedMove->movementRatio;
	unsigned int numMoves = 1;
	while ((nextMove = lastMoveToPlan->next)->IsProvisional() && nextMove->IsSCurveMove() && nextMove->beforePrepare.maxPrevEndSpeed != 0.0)
	{
		// See whether we can reasonably include this move in the plan.
		// First test whether its requested speed is more or less the same as the requested speed of the move(s) so far.
		const float requestedSpeedNextMove = min<float>(nextMove->requestedSpeed, nextMove->beforePrepare.maxPrevEndSpeed) * nextMove->movementRatio;
		if (requestedSpeedNextMove < fminReqSpeed)
		{
			if (requestedSpeedNextMove < maxReqSpeedForPlan * 0.7) { break; }
			fminReqSpeed = requestedSpeedNextMove;
		}
		else if (requestedSpeedNextMove > maxReqSpeedForPlan)
		{
			if (requestedSpeedNextMove > fminReqSpeed * (1.0/0.7)) { break; }
			maxReqSpeedForPlan = requestedSpeedNextMove;
		}

		// We can include nextMove in the plan
		distanceToPlan += (double)(nextMove->totalDistance * nextMove->movementRatio);
		if (nextMove->jerk * nextMove->movementRatio < fminJerk) { fminJerk = nextMove->jerk * nextMove->movementRatio; }
		if (nextMove->maxAcceleration * nextMove->movementRatio < fminMaxAcc) { fminMaxAcc = nextMove->maxAcceleration * nextMove->movementRatio; }
		lastMoveToPlan = nextMove;
		++numMoves;
	}

	plannedProfile.usesAllMoves = (nextMove->GetState() == DDA::empty);
	plannedProfile.startSpeed = firstUnpreparedMove->startSpeed * firstUnpreparedMove->movementRatio;
	plannedProfile.startAcceleration = firstUnpreparedMove->startAcceleration * firstUnpreparedMove->movementRatio;
	plannedProfile.numberOfMovesCovered = numMoves;
	const double minJerk = (double)fminJerk;
	plannedProfile.jerk = minJerk;
	const double planReqSpeed = (double)fminReqSpeed;
	const double planMaxAcc = (double)fminMaxAcc;

	lastMoveToPlan->endSpeed = 0.0;					// for now we always end at zero speed (as well as zero acceleration)
	plannedProfile.endSpeed = 0.0;

	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
	{
		debugPrintf("Planning %u moves, dist %.3f maxSpeed %.4e maxAcc %.4e jerk %.4e ss %.3e sa %.3e\n",
					numMoves, distanceToPlan, planReqSpeed, planMaxAcc, minJerk, plannedProfile.startSpeed, plannedProfile.startAcceleration);
	}

	plannedProfile.topSpeed = planReqSpeed;									// speed we are aiming for, may get reduced
	plannedProfile.peakAcceleration = planMaxAcc;							// maximum acceleration allowed
	plannedProfile.peakDeceleration = -planMaxAcc;							// maximum deceleration allowed, for now always the same as maximum acceleration

	// If the sequence comprises a single move and the start speed and acceleration are both zero (e.g. we are adding the first move), this is the simplest case
	if (   plannedProfile.startSpeed == (double)0.0
		&& plannedProfile.startAcceleration == (double)0.0
		/*&& plannedProfile.endSpeed == (double)0.0*/						// currently we always set the end speed to 0 (see above), so we don't need to test it here
	   )
	{
		plannedProfile.CalculateSimpleSCurvePlan(distanceToPlan);
	}
	else
	{
		plannedProfile.CalculateGeneralSCurvePlan(distanceToPlan);
	}

	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
	{
		plannedProfile.DebugPrint();
	}
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
	{
		plannedProfile.CheckForShortSegments();
	}

#if COPY_PLAN	//DEBUG
	debugProfile = plannedProfile;
#endif
}

// Given a profile covering a number of moves, allocate one move from the profile.
// Update the profile to reflect what's left of it to execute.
/*static*/ void DDA::AllocateMoveFromPlan(MovementProfile& plannedProfile, PrepParams& params) noexcept
{
	constexpr uint32_t MinimumPhaseClocks = 10;

	params.totalDistance = totalDistance;
	double moveDistanceLeft = (double)totalDistance * (double)movementRatio;
	const double djerk = plannedProfile.jerk;
	const double recipMovementRatio = (double)1.0/(double)movementRatio;
	params.jerk = (motioncalc_t)(djerk * recipMovementRatio);
	double speed = plannedProfile.startSpeed;
	double acceleration = plannedProfile.startAcceleration;
	params.initialAcceleration = (motioncalc_t)(acceleration * recipMovementRatio);

	uint32_t totalClocks = 0;
	size_t lastPhaseNumber;

	do
	{
		if (plannedProfile.distances[0] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[0]);
			const double t0Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[0];
			params.distances[0] = (motioncalc_t)(t0Distance * recipMovementRatio);
			double t0 = MovementProfile::SmallestNonNegativeCubicSolution(plannedProfile.jerk, 3 * acceleration, 6 * speed, -6 * t0Distance);
			params.phaseClocks[0] = doubleToU32(t0, __LINE__);
			totalClocks += params.phaseClocks[0];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 0 %.4e %lu %.4e %.4e %.4e\n", t0Distance, params.phaseClocks[0], speed, acceleration, djerk);
			}
			speed += (acceleration + OneHalfDouble * (djerk * t0)) * t0;
			acceleration += djerk * t0;
			if (lastPhase)
			{
				plannedProfile.distances[0] -= t0Distance;
				params.topSpeed = (motioncalc_t)(speed * recipMovementRatio);
				topSpeed = (float)params.topSpeed;
				params.peakAcceleration = (motioncalc_t)(acceleration * recipMovementRatio);
				afterPrepare.peakAcceleration = (float)params.peakAcceleration;
				params.distances[1] = params.distances[2] = params.distances[3] = params.distances[4] = params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[1] = params.phaseClocks[2] = params.phaseClocks[3] = params.phaseClocks[4] = params.phaseClocks[5] = params.phaseClocks[6] = 0;
				params.initialDeceleration = params.peakDeceleration = 0.0;
				afterPrepare.peakDeceleration = 0.0;
				lastPhaseNumber = 0;
				break;
			}
			moveDistanceLeft -= t0Distance;
			plannedProfile.distances[0] = 0.0;
		}
		else
		{
			params.distances[0] = 0.0;
			params.phaseClocks[0] = 0;
		}

		if (plannedProfile.distances[1] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[1]);
			const double t1Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[1];
			params.distances[1] = (motioncalc_t)(t1Distance * recipMovementRatio);
			double t1 = MovementProfile::SmallestNonNegativeQuadraticSolution(OneHalfDouble * plannedProfile.peakAcceleration, speed, -t1Distance);
			params.phaseClocks[1] = doubleToU32(t1, __LINE__);
			totalClocks += params.phaseClocks[1];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 1 %.4e %lu %.4e %.4e %.4e %.4e\n", t1Distance, params.phaseClocks[1], speed, acceleration, (double)0.0, (double)plannedProfile.peakAcceleration);
			}
			params.peakAcceleration = (motioncalc_t)(acceleration * recipMovementRatio);
			speed += t1 * plannedProfile.peakAcceleration;
			acceleration = plannedProfile.peakAcceleration;
			if (lastPhase)
			{
				plannedProfile.distances[1] -= t1Distance;
				params.topSpeed = (motioncalc_t)(speed * recipMovementRatio);
				topSpeed = (float)params.topSpeed;
				afterPrepare.peakAcceleration = (float)params.peakAcceleration;
				params.distances[2] = params.distances[3] = params.distances[4] = params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[2] = params.phaseClocks[3] = params.phaseClocks[4] = params.phaseClocks[5] = params.phaseClocks[6] = 0;
				params.initialDeceleration = params.peakDeceleration = (motioncalc_t)0.0;
				afterPrepare.peakDeceleration = 0.0;
				lastPhaseNumber = 1;
				break;
			}
			moveDistanceLeft -= t1Distance;
			plannedProfile.distances[1] = 0.0;
		}
		else
		{
			params.distances[1] = 0.0;
			params.phaseClocks[1] = 0;
		}

		params.peakAcceleration = (motioncalc_t)(acceleration * recipMovementRatio);
		afterPrepare.peakAcceleration = max<float>((float)params.peakAcceleration, 0.0);			// params.peakAcceleration may be negative if we combined the t2 and t4 segments

		if (plannedProfile.distances[2] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[2]);
			const double t2Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[2];
			params.distances[2] = (motioncalc_t)(t2Distance * recipMovementRatio);
			double t2 = MovementProfile::SmallestNonNegativeCubicSolution(-djerk, 3 * acceleration, 6 * speed, -6 * t2Distance);
			params.phaseClocks[2] = doubleToU32(t2, __LINE__);
			totalClocks += params.phaseClocks[2];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 2 %.4e %lu %.4e %.4e %.4e\n", t2Distance, params.phaseClocks[2], speed, acceleration, -djerk);
			}
			speed += (acceleration - OneHalfDouble * (djerk * t2)) * t2;
			acceleration -= djerk * t2;
			if (lastPhase)
			{
				plannedProfile.distances[2] -= t2Distance;
				plannedProfile.t2NonDecelDistance = max<double>(plannedProfile.t2NonDecelDistance - t2Distance, (double)0.0);
				params.topSpeed = (motioncalc_t)(speed * recipMovementRatio);
				topSpeed = (float)params.topSpeed;
				params.distances[3] = params.distances[4] = params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[3] = params.phaseClocks[4] = params.phaseClocks[5] = params.phaseClocks[6] = 0;
				params.initialDeceleration = params.peakDeceleration = (motioncalc_t)0.0;
				afterPrepare.peakDeceleration = 0.0;
				lastPhaseNumber = 2;
				break;
			}
			moveDistanceLeft -= t2Distance;
			plannedProfile.distances[2] = plannedProfile.t2NonDecelDistance = 0.0;
		}
		else
		{
			params.distances[2] = 0.0;
			params.phaseClocks[2] = 0;
		}

		params.topSpeed = (motioncalc_t)(speed * recipMovementRatio);
		topSpeed = (float)params.topSpeed;

		if (plannedProfile.distances[3] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[3]);
			const double t3Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[3];
			params.distances[3] = (motioncalc_t)(t3Distance * recipMovementRatio);
			double t3 = t3Distance/speed;
			params.phaseClocks[3] = doubleToU32(t3, __LINE__);
			totalClocks += params.phaseClocks[3];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 3 %.4e %lu %.4e %.4e %.4e (%.4e)\n", t3Distance, params.phaseClocks[3], speed, (double)0.0, (double)0.0, acceleration);
			}
			acceleration = 0.0;
			if (lastPhase)
			{
				plannedProfile.distances[3] -= t3Distance;
				params.distances[4] = params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[4] = params.phaseClocks[5] = params.phaseClocks[6] = 0;
				params.initialDeceleration = params.peakDeceleration = (motioncalc_t)0.0;
				afterPrepare.peakDeceleration = 0.0;
				lastPhaseNumber = 3;
				break;
			}
			moveDistanceLeft -= t3Distance;
			plannedProfile.distances[3] = 0.0;
		}
		else
		{
			params.distances[3] = 0.0;
			params.phaseClocks[3] = 0;
		}

		params.initialDeceleration = (motioncalc_t)(acceleration * recipMovementRatio);

		if (plannedProfile.distances[4] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[4]);
			const double t4Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[4];
			params.distances[4] = (motioncalc_t)(t4Distance * recipMovementRatio);
			double t4 = MovementProfile::SmallestNonNegativeCubicSolution(-djerk, 3 * acceleration, 6 * speed, -6 * t4Distance);
			params.phaseClocks[4] = doubleToU32(t4, __LINE__);
			totalClocks += params.phaseClocks[4];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 4 %.4e %lu %.4e %.4e %.4e\n", t4Distance, params. phaseClocks[4], speed, acceleration, -djerk);
			}
			speed += (acceleration - OneHalfDouble * (djerk * t4)) * t4;
			acceleration -= djerk * t4;
			if (lastPhase)
			{
				plannedProfile.distances[4] -= t4Distance;
				params.distances[5] = params.distances[6] = 0.0;
				params.phaseClocks[5] = params.phaseClocks[6] = 0;
				params.peakDeceleration = (motioncalc_t)(acceleration * recipMovementRatio);
				afterPrepare.peakDeceleration = (float)params.peakDeceleration;
				lastPhaseNumber = 4;
				break;
			}
			moveDistanceLeft -= t4Distance;
			plannedProfile.distances[4] = 0.0;
		}
		else
		{
			params.distances[4] = 0.0;
			params.phaseClocks[4] = 0;
		}

		params.peakDeceleration = (motioncalc_t)(acceleration * recipMovementRatio);
		afterPrepare.peakDeceleration = (float)params.peakDeceleration;

		if (plannedProfile.distances[5] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[5]);
			const double t5Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[5];
			params.distances[5] = (motioncalc_t)(t5Distance * recipMovementRatio);
			double t5 = MovementProfile::SmallestNonNegativeQuadraticSolution(OneHalfDouble * plannedProfile.peakDeceleration, speed, -t5Distance);
			params.phaseClocks[5] = doubleToU32(t5, __LINE__);
			totalClocks += params.phaseClocks[5];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 5 %.4e %lu %.4e %.4e %.4e (%.4e)\n", t5Distance, params.phaseClocks[5], speed, acceleration, (double)0.0, plannedProfile.peakDeceleration);
			}
			speed += plannedProfile.peakDeceleration * t5;
			acceleration = plannedProfile.peakDeceleration;
			if (lastPhase)
			{
				plannedProfile.distances[5] -= t5Distance;
				params.distances[6] = 0.0;
				params.phaseClocks[6] = 0;
				lastPhaseNumber = 5;
				break;
			}
			moveDistanceLeft -= t5Distance;
			plannedProfile.distances[5] = 0.0;
		}
		else
		{
			params.distances[5] = 0.0;
			params.phaseClocks[5] = 0;
		}

		// Anything left must go in phase 6
		const double t6Distance = moveDistanceLeft;
		params.distances[6] = (motioncalc_t)(t6Distance * recipMovementRatio);

		// If this is the last move in the plan, calculate from the end so as to get the final speed and acceleration correct
		// This also avoids the cubic solution failing due to rounding error when the move ands at standstill.
		double t6;
		if (plannedProfile.numberOfMovesCovered == 1)
		{
			t6 = MovementProfile::SmallestNonNegativeCubicSolution(djerk, (double)0.0, 6 * plannedProfile.endSpeed, -6 * t6Distance);
			params.phaseClocks[6] = doubleToU32(t6, __LINE__);
		}
		else
		{
			t6 = MovementProfile::SmallestNonNegativeCubicSolution(djerk, 3 * acceleration, 6 * speed, -6 * t6Distance);
			params.phaseClocks[6] = doubleToU32(t6, __LINE__);
			speed += (acceleration + OneHalfDouble * djerk * t6) * t6;
			acceleration += djerk * t6;
			plannedProfile.distances[6] -= t6Distance;
		}
		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
		{
			debugPrintf("Phase 6 %.4e %lu %.4e %.4e %.4e\n", t6Distance, params.phaseClocks[6], speed, acceleration, djerk);
		}
		totalClocks += params.phaseClocks[6];
		lastPhaseNumber = 6;
	} while (false);

	SetState(planned);
	--plannedProfile.numberOfMovesCovered;
	plannedProfile.startSpeed = speed;
	plannedProfile.startAcceleration = acceleration;
	clocksNeeded = totalClocks;

	// Allocating phases between moves sometimes leads to some very short segments at the start and/or end of the move.
	// This is inefficient, and also messes up the initial speed calculation due to the rounding of duration to integer, resulting in large reported discontinuities.
	// So we eliminate very short phases by merging them with adjacent phases. This will result in very small speed discontinuities.
	// First find the first and non-empty segment number (we already have the last non-empty segment number)
	size_t firstPhaseNumber = 0;
	while (firstPhaseNumber < lastPhaseNumber && params.distances[firstPhaseNumber] == (motioncalc_t)0.0) { ++firstPhaseNumber; }

	// If the first non-empty segment is very short, move it into the next non-empty segment
	if (firstPhaseNumber < lastPhaseNumber && params.phaseClocks[firstPhaseNumber] < MinimumPhaseClocks)
	{
		// Find the next non-empty segment
		size_t nextPhaseNumber = firstPhaseNumber + 1;
		while (nextPhaseNumber < lastPhaseNumber && params.distances[nextPhaseNumber] == (motioncalc_t)0.0)
		{
			++nextPhaseNumber;
		}
		params.distances[nextPhaseNumber] += params.distances[firstPhaseNumber];
		params.phaseClocks[nextPhaseNumber] += params.phaseClocks[firstPhaseNumber];
		params.distances[firstPhaseNumber] = (motioncalc_t)0.0;
		params.phaseClocks[firstPhaseNumber] = 0;
		firstPhaseNumber = nextPhaseNumber;
	}

	// If the last non-empty segment is very short, move it into the previous non-empty segment
	if (lastPhaseNumber > firstPhaseNumber && params.phaseClocks[lastPhaseNumber] < MinimumPhaseClocks)
	{
		// Find the previous non-empty segment
		size_t prevPhaseNumber = lastPhaseNumber - 1;
		while (prevPhaseNumber > firstPhaseNumber && params.distances[prevPhaseNumber] == (motioncalc_t)0.0)
		{
			--prevPhaseNumber;
		}
		params.distances[prevPhaseNumber] += params.distances[lastPhaseNumber];
		params.phaseClocks[prevPhaseNumber] += params.phaseClocks[lastPhaseNumber];
		params.distances[lastPhaseNumber] = (motioncalc_t)0.0;
		params.phaseClocks[lastPhaseNumber] = 0;
	}

#if 0
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
	{
		params.DebugPrint();
	}
#endif
}

#endif

// End
