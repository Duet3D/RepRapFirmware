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

// Struct used by the next two functions
struct MultipleMoveParameters
{
	double peakAcceleration;
	double s0;
	double s1;
	double s2;
	double totalDistance;
};

#if COPY_PLAN	//DEBUG
// WARNING: this variable is static, so it's not going to work with multiple motion systems.
// However, it's for debugging only, so as long as we only run one motion system while debugging with COPY_PLAN set then that's OK.
static MovementProfile debugProfile;
#endif

#if 0

/*static*/ void DDA::PlanDeceleratingMoves(double distance, double acc, MovementProfile& plannedProfile) noexcept
{
	// This is called when we have insufficient distance to stop decelerating, so we can't use the normal planner.
	// Work out a plan to reach the end speed and zero acceleration from the current point.
	// Possible optimisation: because we are re-planning due to having added one or more moves, we probably already have such a plan.
	int errorLine;
	do
	{
		// If we increase deceleration to maximum and then reduce it to zero, will we undershoot the target speed?
		double t4 = (acc + plannedProfile.startAcceleration)/plannedProfile.jerk;
		const double u5 = plannedProfile.startSpeed + OneHalfDouble * (plannedProfile.startAcceleration - acc) * t4;
		double t6 = acc/plannedProfile.jerk;
		const double v6 = u5 - OneHalfDouble * acc * t6;			// calculate v6 assuming no t5 segment
		double d4, d5, d6;
		if (v6 >= plannedProfile.endSpeed)
		{
			// We need a constant deceleration segment to get the speed low enough
			d4 = (plannedProfile.startSpeed
					+ (OneHalfDouble * plannedProfile.startAcceleration - OneSixthDouble * plannedProfile.jerk * t4) * t4
				 ) * t4;
			const double t5 = (v6 - plannedProfile.endSpeed)/acc;
			d5 = (u5 - OneHalfDouble * acc * t5) * t5;
			d6 = (plannedProfile.endSpeed + OneSixthDouble * acc * t6) * t6;
		}
		else
		{
			// We don't need a constant deceleration segment
			const double disc = plannedProfile.jerk * (plannedProfile.startSpeed - plannedProfile.endSpeed) + OneHalfDouble * dsquare(plannedProfile.startAcceleration);
			if (disc < (double)0.0)
			{
				debugPrintf("too fast to decelerate\n");
				errorLine = __LINE__;
				break;
			}
			t6 = fastSqrtd(disc)/plannedProfile.jerk;
			t4 = t6 + plannedProfile.startAcceleration/plannedProfile.jerk;
			d4 = (plannedProfile.startSpeed + (OneHalfDouble * plannedProfile.startAcceleration - OneSixthDouble * plannedProfile.jerk * t4) * t4) * t4;
			d5 = 0.0;
			d6 = (plannedProfile.endSpeed + OneSixthDouble * disc/plannedProfile.jerk) * t6;
		}

		// We now have a plan that reaches the required end speed at zero acceleration, but it probably covers too little distance.
		// Check that it doesn't cover too much distance - this should never happen.
		const double minimumDistance = d4 + d5 + d6;
		const double excessDistance = distance - minimumDistance;
		if (excessDistance < (double)0.0)
		{
			debugPrintf("insufficient distance to decelerate, need %.4g available %.4g\n", minimumDistance, distance);
			errorLine = __LINE__;
			break;
		}

		// We need to modify this plan to use up the extra distance. We already know that reducing deceleration until we reach constant speed will use up too much distance.
		// To use up more distance, we need to decelerate more slowly on average. So we need to either stop decelerating at the start, or reduce deceleration at the start.
		// Add a t0 segment and increase the t4 segment by the same time to make up he required distance
		//TODO
		//plannedProfile.distances[0] = qq;
		plannedProfile.distances[1] = plannedProfile.distances[2] = plannedProfile.distances[3] = 0;
		plannedProfile.distances[4] = d4;
		plannedProfile.distances[5] = d5;
		plannedProfile.distances[6] = d6;
		plannedProfile.simple = false;
		plannedProfile.reachesRequestedSpeed = false;

		debugPrintf("unhandled case: can increase deceleration to maximum\n");
		errorLine = __LINE__;
		break;
	} while (false);

	// Here if there is no viable movement profile that satisfies the constraints
	debugPrintf("Move profile calc failed at line %d\n", errorLine);
}
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
	double distanceToPlan = (double)firstUnpreparedMove->totalDistance;
	float fmaxReqSpeed = firstUnpreparedMove->requestedSpeed;
	float fminJerk = firstUnpreparedMove->jerk;
	float fminMaxAcc = firstUnpreparedMove->maxAcceleration;
	unsigned int numMoves = 1;
	while ((nextMove = lastMoveToPlan->next)->IsProvisional() && nextMove->IsSCurveMove() && nextMove->beforePrepare.maxPrevEndSpeed != 0.0)
	{
		distanceToPlan += (double)nextMove->totalDistance;
		if (nextMove->jerk < fminJerk) { fminJerk = nextMove->jerk; }
		if (nextMove->maxAcceleration < fminMaxAcc) { fminMaxAcc = nextMove->maxAcceleration; }
		if (nextMove->requestedSpeed > fmaxReqSpeed) { fmaxReqSpeed = nextMove->requestedSpeed; }
		lastMoveToPlan = nextMove;
		++numMoves;
	}

	plannedProfile.usesAllMoves = (nextMove->state == DDA::empty);
	plannedProfile.startSpeed = firstUnpreparedMove->startSpeed;
	plannedProfile.startAcceleration = firstUnpreparedMove->startAcceleration;
	plannedProfile.numberOfMovesCovered = numMoves;
	const double minJerk = (double)fminJerk;
	plannedProfile.jerk = minJerk;
	const double maxReqSpeed = (double)fmaxReqSpeed;
	const double minMaxAcc = (double)fminMaxAcc;

	lastMoveToPlan->endSpeed = 0.0;					// for now we always end at zero speed (as well as zero acceleration)
	plannedProfile.endSpeed = 0.0;

	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
	{
		debugPrintf("Planning %u moves, dist %.3f maxSpeed %.4e maxAcc %.4e jerk %.4e ss %.3e sa %.3e\n",
					numMoves, distanceToPlan, maxReqSpeed, minMaxAcc, minJerk, plannedProfile.startSpeed, plannedProfile.startAcceleration);
	}

	plannedProfile.topSpeed = maxReqSpeed;									// speed we are aiming for, may get reduced
	plannedProfile.peakAcceleration = (double)fminMaxAcc;					// maximum acceleration allowed
	plannedProfile.peakDeceleration = -plannedProfile.peakAcceleration;		// maximum deceleration allowed, for now always the same as maximum acceleration

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
#if COPY_PLAN	//DEBUG
	debugProfile = plannedProfile;
#endif
}

#if 0
		double viablePeakSpeed, unviablePeakSpeed, peakSpeedToTry = maxReqSpeed;
		unsigned int numIterations = 0;
		int errorLine;
		while (true)
		{
			MultipleMoveParameters accelParams;
			if (!CalculateMultipleMoveProfile(plannedProfile.startSpeed, peakSpeedToTry, plannedProfile.startAcceleration, minMaxAcc, minJerk, accelParams))
			{
				errorLine = __LINE__;
				break;
			}

			MultipleMoveParameters decelParams;
			if (!CalculateMultipleMoveProfile(plannedProfile.endSpeed, peakSpeedToTry, 0.0, minMaxAcc, minJerk, decelParams))
			{
				errorLine = __LINE__;
				break;
			}

			const double distanceNeeded = accelParams.totalDistance + decelParams.totalDistance;
			if (distanceNeeded <= distanceToPlan)
			{
				// This plan is viable
				if (numIterations == 0 || distanceNeeded >= (double)0.98 * distanceToPlan)
				{
					plannedProfile.distances[0] = accelParams.s0;
					plannedProfile.distances[1] = accelParams.s1;
					plannedProfile.distances[2] = accelParams.s2;
					plannedProfile.distances[3] = distanceToPlan - distanceNeeded;
					plannedProfile.distances[4] = decelParams.s2;
					plannedProfile.distances[5] = decelParams.s1;
					plannedProfile.distances[6] = decelParams.s0;
					plannedProfile.topSpeed = peakSpeedToTry;
					plannedProfile.peakAcceleration = accelParams.peakAcceleration;
					plannedProfile.peakDeceleration = -decelParams.peakAcceleration;
					plannedProfile.reachesRequestedSpeed = true;
					if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
					{
						debugPrintf("Solved in %u iterations, match = %.2f\n", numIterations, (double)(distanceNeeded/distanceToPlan));
						plannedProfile.DebugPrint();
					}
#if COPY_PLAN	//DEBUG
					debugProfile = plannedProfile;
#endif
					return;
				}
				else
				{
//					DEBUG_HERE;
					viablePeakSpeed = peakSpeedToTry;
				}
			}
			else
			{
				// Here if we have insufficient distance to reach the requested speed
				if (numIterations == 0)
				{
					if (plannedProfile.startAcceleration < (double)0.0)
					{
						// We are decelerating to start with. We get here when we were executing a plan and started the deceleration phase, but more moves have been added.
						// There are two cases:
						// 1. We have enough distance to stop decelerating and then execute a standard profile. In that case we can use the standard planning code,
						//    however the "top speed" of that profile may be lower than our start speed.
						// 2. We don't have enough distance to stop the deceleration; so the best we can do is reduce the deceleration (phase 0) and then decelerate (phase 4, optionally 5, and 6).
						//    In this case, phase 4 doesn't start at zero acceleration.
						// Start by calculating the distance need to stop decelerating and the corresponding speed, then the distance needed to decelerate from that speed.
						const double timeToStopDecelerating = -plannedProfile.startAcceleration/minJerk;
						const double speedAfterStoppingDecelerating = plannedProfile.startSpeed + OneHalfDouble * plannedProfile.startAcceleration * timeToStopDecelerating;
						const double distanceToStopDecelerating = (speedAfterStoppingDecelerating - OneSixthDouble * plannedProfile.startAcceleration * timeToStopDecelerating) * timeToStopDecelerating;
						if (!CalculateMultipleMoveProfile(plannedProfile.endSpeed, speedAfterStoppingDecelerating, 0.0, minMaxAcc, minJerk, decelParams))
						{
							errorLine = __LINE__;
							break;
						}
						if (distanceToStopDecelerating + decelParams.totalDistance > distanceToPlan)
						{
							PlanDeceleratingMoves(distanceToPlan, minMaxAcc, plannedProfile);
							return;
						}

						// We can stop the deceleration, so we can use the normal planning algorithm. We just need to refine the target top speed.
						viablePeakSpeed = speedAfterStoppingDecelerating;
//						DEBUG_HERE;
					}
					else
					{
						if (plannedProfile.startAcceleration > (double)0.0)
						{
							// We start off accelerating, therefore we can't avoid an acceleration segment
							const double accelTime = plannedProfile.startAcceleration/minJerk;
							const double minViableSpeedAccel = plannedProfile.startSpeed + OneHalfDouble * plannedProfile.startAcceleration * accelTime;
							if (minViableSpeedAccel > plannedProfile.endSpeed)
							{
								// The minimum viable speed is determined by the initial acceleration and speed.
								// If we call CalculateMultipleMoveProfile to calculate the acceleration parameters, it may fail to find a solution because of rounding error.
								// Therefore we calculate the distances separately here.
								viablePeakSpeed = minViableSpeedAccel;
								accelParams.s0 = accelParams.s1 = 0.0;
								accelParams.totalDistance = accelParams.s2 = (plannedProfile.startSpeed + TwoThirdsDouble * plannedProfile.startAcceleration * accelTime) * accelTime;
							}
							else
							{
								viablePeakSpeed = plannedProfile.endSpeed;
								if (!CalculateMultipleMoveProfile(plannedProfile.startSpeed, viablePeakSpeed, plannedProfile.startAcceleration, minMaxAcc, minJerk, accelParams))
								{
									errorLine = __LINE__;
									break;
								}
							}
							if (!CalculateMultipleMoveProfile(plannedProfile.endSpeed, viablePeakSpeed, 0.0, minMaxAcc, minJerk, decelParams))
							{
								errorLine = __LINE__;
								break;
							}
						}
						else
						{
							// Start and end acceleration are both zero
							if (plannedProfile.startSpeed > plannedProfile.endSpeed)
							{
								viablePeakSpeed = plannedProfile.startSpeed;
								accelParams.totalDistance = accelParams.s0 = accelParams.s1 = accelParams.s2 = 0.0;
								if (!CalculateMultipleMoveProfile(plannedProfile.endSpeed, viablePeakSpeed, 0.0, minMaxAcc, minJerk, decelParams))
								{
									errorLine = __LINE__;
									break;
								}
							}
							else
							{
								viablePeakSpeed = plannedProfile.endSpeed;
								decelParams.totalDistance = decelParams.s0 = decelParams.s1 = decelParams.s2 = 0.0;
								if (!CalculateMultipleMoveProfile(plannedProfile.startSpeed, viablePeakSpeed, plannedProfile.startAcceleration, minMaxAcc, minJerk, accelParams))
								{
									errorLine = __LINE__;
									break;
								}
							}
						}

						const double viableDistanceNeeded = accelParams.totalDistance + decelParams.totalDistance;
						if (viableDistanceNeeded >= (double)0.98 * distanceToPlan)		// this test can save a lot of iterations when we re-plan something already planned
						{
							plannedProfile.distances[0] = accelParams.s0;
							plannedProfile.distances[1] = accelParams.s1;
							plannedProfile.distances[2] = accelParams.s2;
							plannedProfile.distances[3] = distanceToPlan - viableDistanceNeeded;
							plannedProfile.distances[4] = decelParams.s2;
							plannedProfile.distances[5] = decelParams.s1;
							plannedProfile.distances[6] = decelParams.s0;
							plannedProfile.topSpeed = viablePeakSpeed;
							plannedProfile.peakAcceleration = accelParams.peakAcceleration;
							plannedProfile.peakDeceleration = -decelParams.peakAcceleration;
							plannedProfile.reachesRequestedSpeed = false;
							if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
							{
								debugPrintf("Solved in 0.5 iterations, match = %.2f\n", viableDistanceNeeded/distanceToPlan);
								plannedProfile.DebugPrint();
							}
#if COPY_PLAN	//DEBUG
							debugProfile = plannedProfile;
#endif
							return;
						}
					}
				}

				// Try a speed somewhere between the known viable and unviable peak speeds
				//TODO instead of doing a simple binary chop, use the distances needed and available to make a better guess
				unviablePeakSpeed = peakSpeedToTry;
			}
			peakSpeedToTry = OneHalfDouble * (viablePeakSpeed + unviablePeakSpeed);
//			debugPrintf("Distances: viable %.6g unviable %.6g available %.6g, speeds: viable %.6e unviable %.6e trying %.6e\n",
//							(double)viableDistanceNeeded, (double)unviableDistanceNeeded, (double)distanceToPlan, (double)viablePeakSpeed, (double)unviablePeakSpeed, (double)peakSpeedToTry);
			delay(1);
			++numIterations;
			if (numIterations > 50) { errorLine = __LINE__; break; }
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Trying peak speed %.3e u=%.3e a=%.3e ma=%.3e j=%.3e s=%.3f\n",
							(double)peakSpeedToTry, plannedProfile.startSpeed, plannedProfile.startAcceleration, minMaxAcc, minJerk, distanceToPlan);
			}
		}

		// Here if there is no viable movement profile that satisfies the constraints
		debugPrintf("Move profile calc failed at line %d\n", errorLine);
	}
}
#endif

// Given a profile covering a number of moves, allocate one move from the profile.
// Update the profile to reflect what's left of it to execute.
/*static*/ void DDA::AllocateMoveFromPlan(MovementProfile& plannedProfile, PrepParams& params) noexcept
{
	params.totalDistance = totalDistance;
	double moveDistanceLeft = (double)totalDistance;
	const double djerk = plannedProfile.jerk;
	params.jerk = (float)djerk;
	double speed = plannedProfile.startSpeed;
	double acceleration = plannedProfile.startAcceleration;
	params.initialAcceleration = (float)acceleration;

	uint32_t totalClocks = 0;

	do
	{
		if (plannedProfile.distances[0] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[0]);
			const double t0Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[0];
			params.distances[0] = (float)t0Distance;
			const double t0 = SmallestNonNegativeCubicSolution(plannedProfile.jerk, 3 * acceleration, 6 * speed, -6 * t0Distance);
			if (std::isnan(t0) || t0 < (double)0.0)
			{
				debugPrintf("Failed at %d, t0=%.7e\n", __LINE__, t0);
				//TODO
			}
			params.phaseClocks[0] = doubleToU32(t0);
			totalClocks += params.phaseClocks[0];
			speed += (acceleration + OneHalfDouble * (djerk * t0)) * t0;
			acceleration += djerk * t0;
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 0: %.3e %lu %.3e %.3e %.3e\n", t0Distance, params.phaseClocks[0], speed, acceleration, djerk);
			}
			if (lastPhase)
			{
				plannedProfile.distances[0] -= t0Distance;
				topSpeed = params.topSpeed = (float)speed;
				afterPrepare.peakAcceleration = params.peakAcceleration = (float)acceleration;
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
			params.distances[1] = (float)t1Distance;
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
				debugPrintf("Phase 1: %.3e %lu %.3e %.3e (%.3e)\n", t1Distance, params.phaseClocks[1], speed, (double)plannedProfile.peakAcceleration, acceleration);
			}
			speed += t1 * plannedProfile.peakAcceleration;
			acceleration = plannedProfile.peakAcceleration;
			params.peakAcceleration = (float) acceleration;
			if (lastPhase)
			{
				plannedProfile.distances[1] -= t1Distance;
				topSpeed = params.topSpeed = speed;
				afterPrepare.peakAcceleration = params.peakAcceleration = acceleration;
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

		afterPrepare.peakAcceleration = params.peakAcceleration = max<float>(acceleration, 0.0);

		if (plannedProfile.distances[2] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[2]);
			const double t2Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[2];
			params.distances[2] = (float)t2Distance;
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
				debugPrintf("Phase 2: %.3e %lu %.3e %.3e %.3e\n", t2Distance, params.phaseClocks[2], speed, acceleration, -djerk);
			}
			speed += (acceleration - OneHalfDouble * (djerk * t2)) * t2;
			acceleration -= djerk * t2;
			if (lastPhase)
			{
				plannedProfile.distances[2] -= t2Distance;
				topSpeed = params.topSpeed = (float)speed;
				params.distances[3] = params.distances[4] = params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[3] = params.phaseClocks[4] = params.phaseClocks[5] = params.phaseClocks[6] = 0;
				afterPrepare.peakDeceleration = params.initialDeceleration = params.peakDeceleration = 0.0;
				break;
			}
			moveDistanceLeft -= t2Distance;
			plannedProfile.distances[2] = 0.0;
		}
		else
		{
			params.distances[2] = 0.0;
			params.phaseClocks[2] = 0;
		}

		topSpeed = params.topSpeed = (float)speed;

		if (plannedProfile.distances[3] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[3]);
			const double t3Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[3];
			params.distances[3] = t3Distance;
			const double t3 = (double)t3Distance/speed;
			params.phaseClocks[3] = doubleToU32(t3);
			totalClocks += params.phaseClocks[3];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 3: %.3e %lu %.3e %.3e (%.3e)\n", t3Distance, params.phaseClocks[3], speed, (double)0.0, acceleration);
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

		params.initialDeceleration = (float)acceleration;

		if (plannedProfile.distances[4] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[4]);
			const double t4Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[4];
			params.distances[4] = (float)t4Distance;
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
				debugPrintf("Phase 4: %.3e %lu %.3e %.3e %.3e\n", t4Distance, params. phaseClocks[4], speed, (double)0.0, -djerk);
			}
			speed += (acceleration - OneHalfDouble * (djerk * t4)) * t4;
			acceleration -= djerk * t4;
			if (lastPhase)
			{
				plannedProfile.distances[4] -= t4Distance;
				params.distances[5] = params.distances[6] = 0;
				params.phaseClocks[5] = params.phaseClocks[6] = 0;
				afterPrepare.peakDeceleration = params.peakDeceleration = acceleration;
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

		afterPrepare.peakDeceleration = params.peakDeceleration = acceleration;

		if (plannedProfile.distances[5] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[5]);
			const double t5Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[5];
			params.distances[5] = t5Distance;
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
				debugPrintf("Phase 5: %.3e %lu %.3e %.3e (%.3e)\n", t5Distance, params.phaseClocks[5], speed, plannedProfile.peakDeceleration, acceleration);
			}
			speed += plannedProfile.peakDeceleration * t5;
			acceleration = plannedProfile.peakDeceleration;
			if (lastPhase)
			{
				plannedProfile.distances[5] -= t5Distance;
				params.distances[6] = 0;
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
		params.distances[6] = t6Distance;

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
			debugPrintf("Phase 6: %.3e %lu %.3e %.3e %.3e\n", t6Distance, params.phaseClocks[6], speed, acceleration, djerk);
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

# if 0

// Given a movement profile that is viable, distribute it over the moves
/*static*/ void DDA::DistributePlanOverMoves(DDA *startMove, DDA *endMove, float distanceToPlan, const MultipleMoveParameters& accelParams, const MultipleMoveParameters& decelParams) noexcept
{
	debugPrintf("Distributing plan: %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
				(double)accelParams.s0, (double)accelParams.s1, (double)accelParams.s2,
				(double)(distanceToPlan - (accelParams.totalDistance + decelParams.totalDistance)),
				(double)decelParams.s2, (double)decelParams.s1, (double)decelParams.s0
			   );
	if (startMove == endMove)
	{
		// The simple case - just copy the distances across
		startMove->profile.distances[0] = accelParams.s0;
		startMove->profile.distances[1] = accelParams.s1;
		startMove->profile.distances[2] = accelParams.s2;
		startMove->profile.distances[4] = decelParams.s2;
		startMove->profile.distances[5] = decelParams.s1;
		startMove->profile.distances[6] = decelParams.s0;
		startMove->profile.distances[3] = distanceToPlan - (accelParams.totalDistance + decelParams.totalDistance);
	}
	else
	{
		// Allocate the t0 acceleration phase
		{
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
					startMove->state = planned;
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
					startMove->state = planned;
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
					startMove->profile.distances[2] = distanceLeftAccelerating;
					startMove->profile.distances[3] = startMove->profile.distances[4] = startMove->profile.distances[5] = startMove->profile.distances[6] = 0.0;
					s2Left -= distanceLeftAccelerating;
					startMove->state = planned;
					startMove = startMove->next;
					startMove->profile.distances[0] = startMove->profile.distances[1] = 0.0;
					distanceLeftAccelerating = startMove->totalDistance;
				}
			}
		}

		// Allocate the t6 deceleration phase. This is the reverse of how we allocate the t0 phase.
		{
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
					endMove->state = planned;
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
					endMove->profile.distances[5] = distanceLeftDecelerating;
					endMove->profile.distances[0] = endMove->profile.distances[1] = endMove->profile.distances[2] = endMove->profile.distances[3] = endMove->profile.distances[4] = 0.0;
					s5Left -= distanceLeftDecelerating;
					endMove->state = planned;
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
						endMove->profile.distances[4] = s4Left;
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

					// The rest of this move is part of the t4 segment of the multiple move deceleration phase
					endMove->profile.distances[0] = endMove->profile.distances[1] = endMove->profile.distances[2] = endMove->profile.distances[3] = 0.0;
					s4Left -= distanceLeftDecelerating;
					endMove->state = planned;
					endMove = endMove->prev;
					endMove->profile.distances[5] = endMove->profile.distances[6] = 0.0;
					distanceLeftDecelerating = endMove->totalDistance;
				}
			}
		}

		// Allocate the t3 steady speed phase
		if (startMove == endMove)
		{
			// Just add a steady speed segment in the middle to make up the distance.
			startMove->profile.distances[3] = startMove->totalDistance
						- (startMove->profile.distances[0] + startMove->profile.distances[1] + startMove->profile.distances[2] + startMove->profile.distances[4] + startMove->profile.distances[5] + startMove->profile.distances[6]);
			startMove->state = planned;
		}
		else
		{
			startMove->profile.distances[3] = startMove->totalDistance - (startMove->profile.distances[0] + startMove->profile.distances[1] + startMove->profile.distances[2]);
			startMove->profile.distances[4] = startMove->profile.distances[5] = startMove->profile.distances[6] = 0.0;
			startMove->state = planned;
			endMove->profile.distances[3] = endMove->totalDistance - (endMove->profile.distances[4] + endMove->profile.distances[5] + endMove->profile.distances[6]);
			endMove->profile.distances[0] = endMove->profile.distances[1] = endMove->profile.distances[2] = 0.0;
			endMove->state = planned;
			while (startMove->next != endMove)
			{
				startMove = startMove->next;
				startMove->profile.distances[3] = startMove->totalDistance;
				startMove->profile.distances[0] = startMove->profile.distances[1] = startMove->profile.distances[2] = startMove->profile.distances[4] = startMove->profile.distances[5] = startMove->profile.distances[6] = 0.0;
				startMove->state = planned;
			}
		}
	}
}

#endif

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

// End
