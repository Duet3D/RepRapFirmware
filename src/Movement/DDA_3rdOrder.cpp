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

constexpr double OneHalfDouble = (double)0.5;
constexpr double OneSixthDouble = (double)1.0/(double)6.0;
constexpr double OneTwelfthDouble = (double)1.0/(double)12.0;

// Convert a float to a uint32_t, with negative values converted to zero
static inline uint32_t doubleToU32(double f) noexcept
{
	return (std::signbit(f)) ? 0 : (uint32_t)f;
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
void DDA::CalculateIsolatedSCurveMove(MovementProfile& plannedProfile) const noexcept
{
	plannedProfile.jerk = (double)jerk;
	plannedProfile.numberOfMovesCovered = 1;
	do
	{
		// Determine whether the requested speed or the maximum acceleration is more limiting
		// The acceleration reached from a standing start is a = j * t and the speed reached is v = 0.5 * j * t^2.
		// So a^2 = j^2 * t^2 = 2 * v * j
		// The phase in which the acceleration is reducing will increase the speed by the same amount. Therefore we can reach acceleration a without exceeding speed v if a^2 >= v * j.
		if (fsquare(maxAcceleration) > requestedSpeed * jerk)
		{
			// In principle we can reach the requested speed without exceeding the maximum acceleration, without having to include a constant acceleration segment
			plannedProfile.distances[1] = plannedProfile.distances[5] = 0.0;
			const motioncalc_t halfTimeToReqSpeed = fastSqrtf(requestedSpeed/jerk);
			const motioncalc_t distanceToReqSpeed = requestedSpeed * halfTimeToReqSpeed;
			if (2 * distanceToReqSpeed < totalDistance)
			{
				// We can reach the requested speed and decelerate to zero again without exceeding the required distance. Generate a 5-phase move.
				plannedProfile.distances[0] = plannedProfile.distances[6] = OneSixthDouble * plannedProfile.jerk * dcube(halfTimeToReqSpeed);
				plannedProfile.distances[2] = plannedProfile.distances[4] = (double)(requestedSpeed * halfTimeToReqSpeed) - plannedProfile.distances[0];
				plannedProfile.distances[3] = totalDistance - 2 * distanceToReqSpeed;
				plannedProfile.topSpeed = requestedSpeed;
				plannedProfile.peakAcceleration = jerk * halfTimeToReqSpeed;
				break;
			}
			// Else we can't reach the requested speed without exceeding required distance, or we can only just reach it and then we need to start decelerating immediately. Fall through to beyond the else-part of this if-statement.
		}
		else
		{
			// We can't reach the requested speed without inserting a constant acceleration segment to avoid exceeding maximum acceleration
			plannedProfile.peakAcceleration = maxAcceleration;
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
				const double timeToMaxAcceleration = maxAcceleration/jerk;
				plannedProfile.distances[0] = plannedProfile.distances[6] = OneSixthDouble * plannedProfile.jerk * dcube(timeToMaxAcceleration);
				const double constantAccelerationTime = -(double)1.5 * timeToMaxAcceleration + fastSqrtd(0.25 * fsquare(timeToMaxAcceleration) + totalDistance/maxAcceleration);
				const double newTopSpeed = plannedProfile.jerk * timeToMaxAcceleration * (timeToMaxAcceleration + constantAccelerationTime);
				if (newTopSpeed <= (double)requestedSpeed)
				{
					// Generate a 6-phase move. The middle 2 phases could be combined.
					plannedProfile.topSpeed = newTopSpeed;
					plannedProfile.distances[1] = plannedProfile.distances[5] = OneHalfDouble * constantAccelerationTime * plannedProfile.topSpeed;
					plannedProfile.distances[2] = plannedProfile.distances[4] = timeToMaxAcceleration * plannedProfile.topSpeed - plannedProfile.distances[0];
					plannedProfile.distances[3] = (double)0.0;
				}
				else
				{
					// We need to limit the constant acceleration time in order to limit the top speed, and add a constant speed phase. Generate a 7-phase move.
					plannedProfile.topSpeed = (double)requestedSpeed;
					//	v = j * t1 * (t1 + t2) therefore t2 = v/(j * t1) - t1
					const double revisedConstantAccelerationTime = (double)requestedSpeed/(plannedProfile.jerk * timeToMaxAcceleration) - timeToMaxAcceleration;
					plannedProfile.distances[1] = plannedProfile.distances[5] = OneHalfDouble * revisedConstantAccelerationTime * plannedProfile.topSpeed;
					plannedProfile.distances[2] = plannedProfile.distances[4] = timeToMaxAcceleration * plannedProfile.topSpeed - plannedProfile.distances[0];
					plannedProfile.distances[3] = (double)totalDistance - 2 * (plannedProfile.distances[0] + plannedProfile.distances[2]);
				}
				break;
			}
			// Else fall through
		}

		// If we get here then we can reach neither requestedSpeed nor maxAcceleration without exceeding totalDistance. Generate a 4-phase move. The middle 2 phases could be combined.
		const double halfTimeToTopSpeed = fastCubeRootd((double)totalDistance * OneHalfDouble / plannedProfile.jerk);
		plannedProfile.distances[0] = plannedProfile.distances[6] = OneTwelfthDouble * (double)totalDistance;
		plannedProfile.distances[2] = plannedProfile.distances[4] = OneHalfDouble * (double)totalDistance - plannedProfile.distances[0];
		plannedProfile.distances[1] = plannedProfile.distances[3] = plannedProfile.distances[5] = 0.0;
		plannedProfile.topSpeed = plannedProfile.jerk * dsquare(halfTimeToTopSpeed);
		plannedProfile.peakAcceleration = plannedProfile.jerk * halfTimeToTopSpeed;
	} while (false);

	plannedProfile.peakDeceleration = -plannedProfile.peakAcceleration;
}

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
static bool CalculateMultipleMoveProfile(double startSpeed, double peakSpeed, double startAcceleration, double maxAcceleration, double jerk, MultipleMoveParameters& rslt) noexcept
pre(peakSpeed >= startSpeed; jerk > 0; startAcceleration > 0; maxAcceleration > 0; startAcceleration <= maxAcceleration)
{
	if (peakSpeed > startSpeed)
	{
		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
		{
			debugPrintf("Calculating MMP with accel/decel phase\n");
		}

		// We need an acceleration phase. First, determine whether the maximum acceleration is limiting.
		if ((peakSpeed - startSpeed) * jerk > dsquare(maxAcceleration) - OneHalfDouble * dsquare(startAcceleration))
		{
			// Maximum acceleration is limiting, so we need a constant acceleration phase
			rslt.peakAcceleration = maxAcceleration;
			rslt.s0 = (maxAcceleration - startAcceleration) * (startSpeed/jerk + (maxAcceleration - startAcceleration) * (2 * startAcceleration + maxAcceleration)/(6 * dsquare(jerk)));
			const double t1 = ((peakSpeed - startSpeed) * jerk - dsquare(maxAcceleration) + OneHalfDouble * dsquare(startAcceleration))/(maxAcceleration * jerk);
			rslt.s1 = t1 * (startSpeed + (maxAcceleration - startAcceleration) * OneHalfDouble * (maxAcceleration + startAcceleration)/jerk + OneHalfDouble * maxAcceleration);
			rslt.s2 = maxAcceleration * ((startSpeed + maxAcceleration * t1)/jerk + (5 * dsquare(maxAcceleration) - 3 * dsquare(startAcceleration))/(6 * dsquare(jerk)));
			rslt.totalDistance = rslt.s0 + rslt.s1 + rslt.s2;
		}
		else
		{
			// We don't need a constant acceleration phase
			rslt.peakAcceleration = fastSqrtd((peakSpeed - startSpeed) * jerk + OneHalfDouble * dsquare(startAcceleration));
			if (rslt.peakAcceleration >= startAcceleration)
			{
				rslt.s0 = (rslt.peakAcceleration - startAcceleration) * (startSpeed/jerk + (rslt.peakAcceleration - startAcceleration) * (2 * startAcceleration + rslt.peakAcceleration)/(6 * dsquare(jerk)));
				rslt.s1 = 0.0;
				rslt.s2 = rslt.peakAcceleration * (startSpeed/jerk + (5 * dsquare(rslt.peakAcceleration) - 3 * dsquare(startAcceleration))/(6 * dsquare(jerk)));
				rslt.totalDistance = rslt.s0 + rslt.s2;
			}
			else
			{
				debugPrintf("CalcMMP failed, sa=%.3e pa=%.3e ss=%.3e ps=%.3e\n", startAcceleration, rslt.peakAcceleration, startSpeed, peakSpeed);
				return false;
			}
		}
		if (rslt.s0 < 0 || rslt.s1 < 0 || rslt.s2 < 0)
		{
			debugPrintf("Error negative distance: s0,1,2 %.4e %.4e %.4e, ss %.4e ps =%.4e sa %.4e ma %.4e pa %.4e j %.4e\n",
							rslt.s0, rslt.s1, rslt.s2, startSpeed, peakSpeed, startAcceleration, maxAcceleration, rslt.peakAcceleration, jerk);
		}
	}
	else
	{
		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
		{
			debugPrintf("MMP has no accel/decel phase\n");
		}
		// We don't need an acceleration phase
		rslt.s0 = rslt.s1 = rslt.s2 = rslt.totalDistance = 0.0;
	}
	return true;
}

#if 0
// Calculate a deceleration-only profile returning true if successful
static bool CalculateDeceleratingMultipleMoveProfile(float startSpeed, float endSpeed, float startAcceleration, float endAcceleration, float maxAcceleration, float jerk, MultipleMoveParameters& rslt) noexcept
pre(startAcceleration <= 0.0; endAcceleration <= 0.0)
{
	debugPrintf("Calculating decel move\n");
	// The plan is to use negative jerk (increase deceleration) for time t2, decelerate steadily for time t1, then use positive jerk (reduce deceleration) for time t0
	// See Maxima for the derivation of this
	// First assume t1 is zero and see whether the peak acceleration is within limits
	const float discriminant = 2 * jerk * (startSpeed - endSpeed) + fsquare(startAcceleration) + fsquare(endAcceleration);
	if (discriminant < 0)
	{
		return false;					// no solutions
	}

	const float peakAcceleration = fastSqrtf(discriminant) * (0.5 * sqrtf(2.0));
	if (fabsf(peakAcceleration) <= maxAcceleration)
	{
		// We don't need a constant deceleration segment
		const float t0 = (peakAcceleration + endAcceleration)/jerk;
		rslt.s0 = (endSpeed - (0.5 * endAcceleration   + OneSixth * jerk * t0) * t0) * t0;
		rslt.s1 = 0.0;
		const float t2 = (peakAcceleration + startAcceleration)/jerk;
		rslt.s2 = (startSpeed + (0.5 * startAcceleration - OneSixth * jerk * t2) * t2) * t2;
		rslt.totalDistance = rslt.s0 + rslt.s2;
	}
	else
	{
		// We do need a constant deceleration segment, to avoid exceeding maximum deceleration
		const float t0 = (endAcceleration - maxAcceleration)/jerk;
		rslt.s0 = (endSpeed - (0.5 * endAcceleration + OneSixth * jerk * t0) * t0) * t0;
		const float t0EndSpeed = startSpeed + (startAcceleration - 0.5 * jerk * t0) * t0;
		const float t2 = (startAcceleration - maxAcceleration)/jerk;
		const float t2StartSpeed = endSpeed - (endAcceleration   - 0.5 * jerk * t2) * t2;
		const float t1 = (t0EndSpeed - t2StartSpeed)/maxAcceleration;
		rslt.s1 = (t0EndSpeed + t2StartSpeed) * t1 * 0.5;
		rslt.s2 = (startSpeed + (0.5 * startAcceleration - OneSixth * jerk * t2) * t2) * t2;
		rslt.totalDistance = rslt.s0 + rslt.s1 + rslt.s2;
	}
	return true;
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
	float maxReqSpeed = firstUnpreparedMove->requestedSpeed;
	float fminJerk = firstUnpreparedMove->jerk;
	float fminMaxAcc = firstUnpreparedMove->maxAcceleration;
	unsigned int numMoves = 1;
	while ((nextMove = lastMoveToPlan->next)->IsProvisional() && nextMove->beforePrepare.maxPrevEndSpeed != 0.0)
	{
		distanceToPlan += (double)nextMove->totalDistance;
		if (nextMove->jerk < fminJerk) { fminJerk = nextMove->jerk; }
		if (nextMove->maxAcceleration < fminMaxAcc) { fminMaxAcc = nextMove->maxAcceleration; }
		if (nextMove->requestedSpeed > maxReqSpeed) { maxReqSpeed = nextMove->requestedSpeed; }
		lastMoveToPlan = nextMove;
		++numMoves;
	}

	plannedProfile.usesAllMoves = (nextMove->state == DDA::empty);
	plannedProfile.startSpeed = firstUnpreparedMove->startSpeed;
	plannedProfile.startAcceleration = firstUnpreparedMove->startAcceleration;
	plannedProfile.numberOfMovesCovered = numMoves;
	const double minJerk = (double)fminJerk;
	plannedProfile.jerk = minJerk;
	const double minMaxAcc = (double)fminMaxAcc;

	lastMoveToPlan->endSpeed = 0.0;					// for now we always end at zero speed (as well as zero acceleration)
	plannedProfile.endSpeed = 0.0;

	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
	{
		debugPrintf("Planning %u moves, dist %.3f maxSpeed %.4e maxAcc %.4e jerk %.4e ss %.3e sa %.3e\n",
					numMoves, distanceToPlan, (double)maxReqSpeed, minMaxAcc, minJerk, plannedProfile.startSpeed, plannedProfile.startAcceleration);
	}

	// If the sequence comprises a single move and the start speed and acceleration are both zero (e.g. we are adding the first move), this is the simplest case
	if (numMoves == 1 && plannedProfile.startSpeed == (double)0.0 && plannedProfile.startAcceleration == (double)0.0)
	{
		firstUnpreparedMove->CalculateIsolatedSCurveMove(plannedProfile);
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
		// - if we reach max acceleration, the duration we maintain it
		// - the duration we maintain the peak speed
		// Start by seeing how much distance we use up if we accelerate to the peak requested speed
		double viablePeakSpeed, unviablePeakSpeed, peakSpeedToTry = maxReqSpeed;
		double viableDistanceNeeded;
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
					return;
				}
				else
				{
//					DEBUG_HERE;
					viablePeakSpeed = peakSpeedToTry;
					viableDistanceNeeded = distanceNeeded;
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
							// We have insufficient distance to stop decelerating.
							// Work out a plan to reach the end speed and zero acceleration from the current point.
							// If we increase deceleration to maximum and then reduce it to zero, will we undershoot the target speed?
							const double timeToMaxDeceleration = (minMaxAcc + plannedProfile.startAcceleration)/minJerk;
							const double speedAtMaxDeceleration = plannedProfile.startSpeed + OneHalfDouble * (plannedProfile.startAcceleration - minMaxAcc) * timeToMaxDeceleration;
							const double timeToEndDeceleration = minMaxAcc/minJerk;
							const double speedAtEndDeceleration = speedAtMaxDeceleration - OneHalfDouble * minMaxAcc * timeToEndDeceleration;
							double minimumDistance;
							if (speedAtEndDeceleration >= plannedProfile.endSpeed)
							{
								// We need a constant deceleration segment to get the speed low enough
								const double timeAtMaxDeceleration = speedAtEndDeceleration/(double)minMaxAcc;
								const double distanceToMaxDeceleration = (plannedProfile.startSpeed
																			+ (OneHalfDouble * plannedProfile.startAcceleration - OneSixthDouble * minJerk * timeToMaxDeceleration) * timeToMaxDeceleration
																		 ) * timeToMaxDeceleration;
								const double distanceAtMaxDeceleration = (speedAtMaxDeceleration - OneHalfDouble * minMaxAcc * timeAtMaxDeceleration) * timeAtMaxDeceleration;
								const double distanceToEndDeceleration = (plannedProfile.endSpeed + OneSixthDouble * minJerk * dsquare(timeToEndDeceleration)) * timeToEndDeceleration;
								minimumDistance = distanceToMaxDeceleration + distanceAtMaxDeceleration + distanceToEndDeceleration;
								if (minimumDistance > distanceToPlan)
								{
									debugPrintf("insufficient distance to decelerate, need %.4g available %.4g\n", minimumDistance, distanceToPlan);
									errorLine = __LINE__;
									break;
								}
							}
							else
							{
								// We can't increase deceleration to maximum
								//minimumDistance = qq;
								debugPrintf("unhandled case: can't increase deceleration to maximum\n");
								errorLine = __LINE__;
								break;
							}
							// TODO
							debugPrintf("unhandled case: time to stop decel %.4g, speed %.4g, dist %.4g, available %.4g\n",
										timeToStopDecelerating, speedAfterStoppingDecelerating, distanceToStopDecelerating, distanceToPlan);
							errorLine = __LINE__;
							break;
						}

						// We can stop the deceleration, so we can use the normal planning algorithm. We just need to refine the target top speed.
						viablePeakSpeed = speedAfterStoppingDecelerating;
						viableDistanceNeeded = distanceToStopDecelerating + decelParams.totalDistance;
//						DEBUG_HERE;
					}
					else
					{
						// We start off accelerating, therefore we can't avoid an acceleration segment
						const double minViableSpeedAccel = plannedProfile.startSpeed + OneHalfDouble * dsquare(plannedProfile.startAcceleration)/minJerk;
						const double minViableSpeedDecel = plannedProfile.endSpeed;
						viablePeakSpeed = max<float>(minViableSpeedAccel, minViableSpeedDecel);
						if (viablePeakSpeed > (double)0.0)
						{
							if (!CalculateMultipleMoveProfile(plannedProfile.endSpeed, viablePeakSpeed, 0.0, minMaxAcc, minJerk, decelParams))
							{
								errorLine = __LINE__;
								break;
							}
							if (!CalculateMultipleMoveProfile(plannedProfile.startSpeed, viablePeakSpeed, plannedProfile.startAcceleration, minMaxAcc, minJerk, accelParams))
							{
								errorLine = __LINE__;
								break;
							}
							viableDistanceNeeded = accelParams.totalDistance + decelParams.totalDistance;
							if (viableDistanceNeeded > distanceToPlan)
							{
								debugPrintf("accel %.4e decl %.4e viable %.4e available %.4e\n", accelParams.totalDistance, decelParams.totalDistance, viableDistanceNeeded, distanceToPlan);
								errorLine = __LINE__;
								break;
							}

							if (viableDistanceNeeded >= (double)0.98 * distanceToPlan)		// this test can save a lot of iterations when we re-plan something already planned
							{
								plannedProfile.distances[0] = accelParams.s0;
								plannedProfile.distances[1] = accelParams.s1;
								plannedProfile.distances[2] = accelParams.s2;
								plannedProfile.distances[3] = distanceToPlan - distanceNeeded;
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
								return;
							}
//							DEBUG_HERE;
						}
						else
						{
							viableDistanceNeeded = 0.0;
//							DEBUG_HERE;
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

// Given a profile covering a number of moves, allocate one move from the profile.
// Update the profile to reflect what's left of it to execute.
/*static*/ void DDA::AllocateMoveFromPlan(MovementProfile& plannedProfile, PrepParams& params) noexcept
{
	params.totalDistance = totalDistance;
	double moveDistanceLeft = (double)totalDistance;
	params.jerk = (float)plannedProfile.jerk;
	const double djerk = plannedProfile.jerk;
	double speed = plannedProfile.startSpeed;
	params.initialAcceleration = (float)plannedProfile.startAcceleration;
	double acceleration = plannedProfile.startAcceleration;

	uint32_t totalClocks = 0;

	do
	{
		if (plannedProfile.distances[0] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[0]);
			const double t0Distance = (lastPhase) ? moveDistanceLeft : (double)plannedProfile.distances[0];
			params.distances[0] = t0Distance;
			const double t0 = SmallestNonNegativeCubicSolution(plannedProfile.jerk, 3 * acceleration, 6 * speed, -6 * t0Distance);
			params.phaseClocks[0] = doubleToU32(t0);
			totalClocks += params.phaseClocks[0];
			speed += (acceleration + OneHalfDouble * djerk * t0) * t0;
			acceleration += djerk * t0;
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 0: %.3e %lu %.3e %.3e %.3e\n", t0Distance, params.phaseClocks[0], speed, acceleration, djerk);
			}
			if (lastPhase)
			{
				plannedProfile.distances[0] -= t0Distance;
				topSpeed = params.topSpeed = speed;
				afterPrepare.peakAcceleration = params.peakAcceleration = acceleration;
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
			params.distances[1] = t1Distance;
			const double t1 = SmallestNonNegativeQuadraticSolution(OneHalfDouble * plannedProfile.peakAcceleration, speed, -t1Distance);
			params.phaseClocks[1] = doubleToU32(t1);
			totalClocks += params.phaseClocks[2];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 1: %.3e %lu %.3e %.3e (%.3e)\n", t1Distance, params.phaseClocks[1], speed, (double)plannedProfile.peakAcceleration, acceleration);
			}
			speed += t1 * plannedProfile.peakAcceleration;
			acceleration = params.peakAcceleration = plannedProfile.peakAcceleration;
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
			params.distances[2] = t2Distance;
			const double t2 = SmallestNonNegativeCubicSolution(-djerk, 3 * acceleration, 6 * speed, -6 * t2Distance);
			params.phaseClocks[2] = doubleToU32(t2);
			totalClocks += params.phaseClocks[2];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 2: %.3e %lu %.3e %.3e %.3e\n", t2Distance, params.phaseClocks[2], speed, acceleration, -djerk);
			}
			speed += (acceleration - OneHalfDouble * djerk * t2) * t2;
			acceleration -= djerk * t2;
			if (lastPhase)
			{
				plannedProfile.distances[2] -= t2Distance;
				topSpeed = params.topSpeed = speed;
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

		topSpeed = params.topSpeed = speed;

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

		params.initialDeceleration = acceleration;

		if (plannedProfile.distances[4] > (double)0.0)
		{
			const bool lastPhase = (moveDistanceLeft <= plannedProfile.distances[4]);
			const double t4Distance = (lastPhase) ? moveDistanceLeft : plannedProfile.distances[4];
			params.distances[4] = t4Distance;
			const double t4 = SmallestNonNegativeCubicSolution(-djerk, 3 * acceleration, 6 * speed, -6 * t4Distance);
			params.phaseClocks[4] = doubleToU32(t4);
			totalClocks += params.phaseClocks[4];
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Lookahead))
			{
				debugPrintf("Phase 4: %.3e %lu %.3e %.3e %.3e\n", t4Distance, params. phaseClocks[4], speed, (double)0.0, -djerk);
			}
			speed += (acceleration - OneHalfDouble * djerk * t4) * t4;
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
				plannedProfile.distances[2] -= t5Distance;
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
			if (std::isnan(t6))
			{
				debugPrintf("Failed at %d\n", __LINE__);
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
