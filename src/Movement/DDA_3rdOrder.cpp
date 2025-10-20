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

#define COPY_PLAN	1

// Return the smallest non-negative root of the equation. Returns the largest solution if there is no nonnegative solution, or NaN if there are no solutions.
static double SmallestNonNegativeCubicSolution(double a, double b, double c, double d) noexcept
{
	double rslt[3];
	const size_t numSolutions = SolveCubic(a, b, c, d, rslt);
#if 0
	debugPrintf("%u solutions:", numSolutions);
	if (numSolutions >= 1) { debugPrintf(" %.3e", (double)rslt[0]); }
	if (numSolutions >= 2) { debugPrintf(" %.3e", (double)rslt[1]); }
	if (numSolutions >= 3) { debugPrintf(" %.3e", (double) rslt[2]); }
	debugPrintf("\n");
#endif
	if (unlikely(numSolutions == 0)) { return std::numeric_limits<double>::quiet_NaN(); }
	if (rslt[0] >= (double)0.0 || numSolutions == 1) { return rslt[0]; }
	if (rslt[1] >= (double)0.0 || numSolutions == 2) { return rslt[1]; }
	return rslt[2];
}

// Return the smallest non-negative root of the equation. Returns the greatest root if both roots are negative, or NaN if there re no roots.
static double SmallestNonNegativeQuadraticSolution(double a, double b, double c) noexcept
{
	if (a == (double)0.0)
	{
		return -c/b;
	}
	const double disc = dsquare(b) - 4 * a * c;
	if (disc < (double)0.0)
	{
		debugPrintf("No solutions: a=%.6g b=%.6g c=%.6g\n", a, b, c);
		return std::numeric_limits<double>::quiet_NaN();
	}
	const double temp = fastSqrtd(disc);
	if (a < 0)
	{
		a = -a; b = -b;
	}
	return ((b + temp <= 0) ? -(b + temp) : (temp - b))/(2 * a);
}

// Convert a float to a uint32_t, with negative values converted to zero
static inline uint32_t doubleToU32(double f) noexcept
{
	if (std::isnan(f) || std::signbit(f) || f > (double)(10 * StepClockRate))
	{
		debugPrintf("Calculated duration: %.4g\n", f);
	}
	return (std::signbit(f) || std::isnan(f)) ? 0 : (uint32_t)f;
}

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
		// See whether we can reasonably include this move in the plan
		const float requestedSpeedThisMove = min<float>(nextMove->requestedSpeed, nextMove->beforePrepare.maxPrevEndSpeed) * nextMove->movementRatio;
		if (requestedSpeedThisMove < fminReqSpeed)
		{
			if (requestedSpeedThisMove < maxReqSpeedForPlan * 0.7) { break; }
			fminReqSpeed = requestedSpeedThisMove;
		}
		else if (requestedSpeedThisMove > maxReqSpeedForPlan)
		{
			if (requestedSpeedThisMove > fminReqSpeed * (1.0/0.7)) { break; }
			maxReqSpeedForPlan = requestedSpeedThisMove;
		}

		// We can include nextMove in the plan
		distanceToPlan += (double)(nextMove->totalDistance * nextMove->movementRatio);
		if (nextMove->jerk * nextMove->movementRatio < fminJerk) { fminJerk = nextMove->jerk * nextMove->movementRatio; }
		if (nextMove->maxAcceleration * nextMove->movementRatio < fminMaxAcc) { fminMaxAcc = nextMove->maxAcceleration * nextMove->movementRatio; }
		lastMoveToPlan = nextMove;
		++numMoves;
	}

	plannedProfile.usesAllMoves = (nextMove->state == DDA::empty);
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
#if COPY_PLAN	//DEBUG
	debugProfile = plannedProfile;
#endif
}

// Given a profile covering a number of moves, allocate one move from the profile.
// Update the profile to reflect what's left of it to execute.
/*static*/ void DDA::AllocateMoveFromPlan(MovementProfile& plannedProfile, PrepParams& params) noexcept
{
	params.totalDistance = totalDistance;
	double moveDistanceLeft = (double)totalDistance * (double)movementRatio;
	const double djerk = plannedProfile.jerk;
	const double recipMovementRatio = (double)1.0/(double)movementRatio;
	params.jerk = (float)(djerk * recipMovementRatio);
	double speed = plannedProfile.startSpeed;
	double acceleration = plannedProfile.startAcceleration;
	params.initialAcceleration = (float)(acceleration * recipMovementRatio);

	uint32_t totalClocks = 0;

	do
	{
		if (plannedProfile.distances[0] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[0]);
			const double t0Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[0];
			params.distances[0] = (float)(t0Distance * recipMovementRatio);
			const double t0 = SmallestNonNegativeCubicSolution(plannedProfile.jerk, 3 * acceleration, 6 * speed, -6 * t0Distance);
			if (std::isnan(t0) || t0 < (double)0.0)
			{
				debugPrintf("Failed at %d, t0=%.7e\n", __LINE__, t0);
				//TODO
			}
			params.phaseClocks[0] = doubleToU32(t0);
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
				topSpeed = params.topSpeed = (float)(speed * recipMovementRatio);
				afterPrepare.peakAcceleration = params.peakAcceleration = (float)(acceleration * recipMovementRatio);
				params.distances[1] = params.distances[2] = params.distances[3] = params.distances[4] = params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[1] = params.phaseClocks[2] = params.phaseClocks[3] = params.phaseClocks[4] = params.phaseClocks[5] = params.phaseClocks[6] = 0;
				afterPrepare.peakDeceleration = params.initialDeceleration = params.peakDeceleration = 0.0;
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
			params.distances[1] = (float)(t1Distance * recipMovementRatio);
			const double t1 = SmallestNonNegativeQuadraticSolution(OneHalfDouble * plannedProfile.peakAcceleration, speed, -t1Distance);
			if (std::isnan(t1) || t1 < (double)0.0)
			{
				debugPrintf("Failed at %d, t1=%.7e\n", __LINE__, t1);
				//TODO
			}
			params.phaseClocks[1] = doubleToU32(t1);
			totalClocks += params.phaseClocks[1];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 1 %.4e %lu %.4e %.4e %.4e (%.4e)\n", t1Distance, params.phaseClocks[1], speed, acceleration, (double)0.0, (double)plannedProfile.peakAcceleration);
			}
			params.peakAcceleration = (float)(acceleration * recipMovementRatio);
			speed += t1 * plannedProfile.peakAcceleration;
			acceleration = plannedProfile.peakAcceleration;
			if (lastPhase)
			{
				plannedProfile.distances[1] -= t1Distance;
				topSpeed = params.topSpeed = (float)(speed * recipMovementRatio);
				afterPrepare.peakAcceleration = params.peakAcceleration;
				params.distances[2] = params.distances[3] = params.distances[4] = params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[2] = params.phaseClocks[3] = params.phaseClocks[4] = params.phaseClocks[5] = params.phaseClocks[6] = 0;
				afterPrepare.peakDeceleration = params.initialDeceleration = params.peakDeceleration = 0.0;
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

		params.peakAcceleration = (float)(acceleration * recipMovementRatio);
		afterPrepare.peakAcceleration = max<float>(params.peakAcceleration, 0.0);			// params.peakAcceleration may be negative if we combined the t2 and t4 segments

		if (plannedProfile.distances[2] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[2]);
			const double t2Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[2];
			params.distances[2] = (float)(t2Distance * recipMovementRatio);
			const double t2 = SmallestNonNegativeCubicSolution(-djerk, 3 * acceleration, 6 * speed, -6 * t2Distance);
			if (std::isnan(t2) || t2 < (double)0.0)
			{
				debugPrintf("Failed at %d, t2=%.7e\n", __LINE__, t2);
				//TODO
			}
			params.phaseClocks[2] = doubleToU32(t2);
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
				topSpeed = params.topSpeed = (float)(speed * recipMovementRatio);
				params.distances[3] = params.distances[4] = params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[3] = params.phaseClocks[4] = params.phaseClocks[5] = params.phaseClocks[6] = 0;
				afterPrepare.peakDeceleration = params.initialDeceleration = params.peakDeceleration = 0.0;
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

		topSpeed = params.topSpeed = (float)(speed * recipMovementRatio);

		if (plannedProfile.distances[3] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[3]);
			const double t3Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[3];
			params.distances[3] = (float)(t3Distance * recipMovementRatio);
			const double t3 = t3Distance/speed;
			params.phaseClocks[3] = doubleToU32(t3);
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
				afterPrepare.peakDeceleration = params.initialDeceleration = params.peakDeceleration = 0.0;
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

		params.initialDeceleration = (float)(acceleration * recipMovementRatio);

		if (plannedProfile.distances[4] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[4]);
			const double t4Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[4];
			params.distances[4] = (float)(t4Distance * recipMovementRatio);
			const double t4 = SmallestNonNegativeCubicSolution(-djerk, 3 * acceleration, 6 * speed, -6 * t4Distance);
			if (std::isnan(t4) || t4 < (double)0.0)
			{
				debugPrintf("Failed at %d, t4=%.7e\n", __LINE__, t4);
#if COPY_PLAN	//DEBUG
				debugPrintf("Original:  "); debugProfile.DebugPrint();
#endif
				debugPrintf("Remaining: "); plannedProfile.DebugPrint();
				//TODO
			}
			params.phaseClocks[4] = doubleToU32(t4);
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
				afterPrepare.peakDeceleration = params.peakDeceleration = (float)(acceleration * recipMovementRatio);
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

		afterPrepare.peakDeceleration = params.peakDeceleration = (float)(acceleration * recipMovementRatio);

		if (plannedProfile.distances[5] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[5]);
			const double t5Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[5];
			params.distances[5] = (float)(t5Distance * recipMovementRatio);
			const double t5 = SmallestNonNegativeQuadraticSolution(OneHalfDouble * plannedProfile.peakDeceleration, speed, -t5Distance);
			if (std::isnan(t5) || t5 < (double)0.0)
			{
				debugPrintf("Failed at %d, t5=%.7e dist=%.7e decl=%.7e speed=%.7e\n", __LINE__, t5, t5Distance, plannedProfile.peakDeceleration, speed);
#if COPY_PLAN	//DEBUG
				debugPrintf("Original:  "); debugProfile.DebugPrint();
#endif
				debugPrintf("Remaining: "); plannedProfile.DebugPrint();
				//TODO
			}
			params.phaseClocks[5] = doubleToU32(t5);
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
		params.distances[6] = (float)(t6Distance * recipMovementRatio);

		// If we are ending at zero speed then we only just achieve the distance, and due to rounding error the cubic solution may fail.
		double t6 = SmallestNonNegativeCubicSolution(djerk, 3 * acceleration, 6 * speed, -6 * t6Distance);
		if (std::isnan(t6))
		{
			t6 = SmallestNonNegativeQuadraticSolution(OneHalfDouble * djerk, acceleration, speed);
			if (std::isnan(t6) || t6 < (double)0.0)
			{
				debugPrintf("Failed at %d, t6=%.7e\n", __LINE__, t6);
				//TODO
			}
			else
			{
				const double actualDistance = (speed + OneHalfDouble * acceleration * t6) * t6;
				if (fabs(plannedProfile.distances[6] - actualDistance) > fabs(plannedProfile.distances[6]) * (double)0.0001)
				{
					debugPrintf("Failed at %d\n", __LINE__);
					//TODO
				}
			}
		}
		else if (t6 < (double)0.0)
		{
			debugPrintf("Failed at %d, t6=%.7e\n", __LINE__, t6);
			//TODO
		}
		params.phaseClocks[6] = doubleToU32(t6);
		totalClocks += params.phaseClocks[6];
		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
		{
			debugPrintf("Phase 6 %.4e %lu %.4e %.4e %.4e\n", t6Distance, params.phaseClocks[6], speed, acceleration, djerk);
		}
		speed += (acceleration + OneHalfDouble * djerk * t6) * t6;
		acceleration += djerk * t6;
		plannedProfile.distances[6] = max<double>(plannedProfile.distances[6] - t6Distance, 0.0);
	} while (false);

	state = DDA::planned;
	--plannedProfile.numberOfMovesCovered;
	plannedProfile.startSpeed = speed;
	plannedProfile.startAcceleration = acceleration;
	clocksNeeded = totalClocks;
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
	{
		params.DebugPrint();
	}
}

#endif

// End
