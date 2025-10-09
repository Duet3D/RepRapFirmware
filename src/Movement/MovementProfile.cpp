/*
 * MovementProfile.cpp
 *
 *  Created on: 10 Sept 2025
 *      Author: David
 */

#include "MovementProfile.h"

#if SUPPORT_S_CURVE

#include <Platform/RepRap.h>
#include "MoveDebugFlags.h"

void MovementProfile::DebugPrint() noexcept
{
	debugPrintf("Plan: d=[%.4g %.4g %.4g %.4g %.4g %.4g %.4g] v=[%.4g %.4g %.4g] a=[%.4g %.4g %.4g 0.0] j=%.4g num=%u all=%u rrs=%u simple=%u\n",
				distances[0], distances[1], distances[2], distances[3], distances[4], distances[5], distances[6],
				startSpeed, topSpeed, endSpeed,
				startAcceleration, peakAcceleration, peakDeceleration,
				jerk,
				numberOfMovesCovered, usesAllMoves, reachesRequestedSpeed, simple
			   );
}

// Calculate the parameters needed to accelerate to topSpeed from startSpeed and startAcceleration.
// Return the distance covered.
double MovementProfile::CalculateAccelerationParameters() noexcept
{
	if (startSpeed < topSpeed)
	{
		// We need an acceleration phase. First, determine whether the maximum acceleration is limiting.
		if ((topSpeed - startSpeed) * jerk > dsquare(peakAcceleration) - OneHalfDouble * dsquare(startAcceleration))
		{
			// Maximum acceleration is limiting, so we need a constant acceleration phase
			distances[0] = (peakAcceleration - startAcceleration) * (startSpeed/jerk + (peakAcceleration - startAcceleration) * (2 * startAcceleration + peakAcceleration)/(6 * dsquare(jerk)));
			// t1 = (v-u)/a + a0^2/2aj - a/j						[checked using Maxima]
			const double t1 = ((topSpeed - startSpeed) * jerk - dsquare(peakAcceleration) + OneHalfDouble * dsquare(startAcceleration))/(peakAcceleration * jerk);
			// s1 = t1 * (u + 0.5 * (a * t1 + (a^2 - a0^2)/j))		[checked using Maxima]
			distances[1] = t1 * (startSpeed + OneHalfDouble * (peakAcceleration * t1 + (dsquare(peakAcceleration - dsquare(startAcceleration))/jerk)));
			distances[2] = peakAcceleration * ((startSpeed + peakAcceleration * t1)/jerk + (5 * dsquare(peakAcceleration) - 3 * dsquare(startAcceleration))/(6 * dsquare(jerk)));
			return distances[0] + distances[1] + distances[2];
		}
		else
		{
			// We don't need a constant acceleration phase
			peakAcceleration = fastSqrtd((topSpeed - startSpeed) * jerk + OneHalfDouble * dsquare(startAcceleration));
			if (peakAcceleration >= startAcceleration)
			{
				distances[0] = (peakAcceleration - startAcceleration) * (startSpeed/jerk + (peakAcceleration - startAcceleration) * (2 * startAcceleration + peakAcceleration)/(6 * dsquare(jerk)));
				distances[1] = (double)0.0;
				distances[2] = peakAcceleration * (startSpeed/jerk + (5 * dsquare(peakAcceleration) - 3 * dsquare(startAcceleration))/(6 * dsquare(jerk)));
				return distances[0] + distances[2];
			}
			else
			{
				debugPrintf("CalcAccelParams failed, sa=%.4e pa=%.4e ss=%.4e ps=%.4e j=%.4e\n", startAcceleration, peakAcceleration, startSpeed, topSpeed, jerk);
				distances[0] = distances[1] = distances[2] = 0.0;
				return 0.0;
			}
		}
	}
	else
	{
		distances[0] = distances[1] = distances[2] = 0.0;
		return 0.0;
	}
}

// Calculate the parameters needed to decelerate from topSpeed to endSpeed and zeo acceleration.
// Return the distance covered.
// This is similar to the reverse of CalculateAccelerationParameters except that the end acceleration is zero.
double MovementProfile::CalculateDecelerationParameters() noexcept
{
	// We need a deceleration phase. First, determine whether the maximum acceleration is limiting.
	if (topSpeed > endSpeed)
	{
		if ((topSpeed - endSpeed) * jerk > dsquare(peakDeceleration))
		{
			// Maximum deceleration is limiting, so we need a constant deceleration phase
			distances[6] = -peakDeceleration * (endSpeed/jerk + dsquare(peakDeceleration)/(6 * dsquare(jerk)));
			// t1 = (v-u)/a + a0^2/2aj - a/j						[checked using Maxima]
			const double t5 = ((topSpeed - endSpeed) * jerk - dsquare(peakDeceleration))/(-peakDeceleration * jerk);
			// s1 = t1 * (u + 0.5 * (a * t1 + (a^2 - a0^2)/j))		[checked using Maxima]
			distances[5] = t5 * (endSpeed + OneHalfDouble * (-peakDeceleration * t5 + (dsquare(peakDeceleration)/jerk)));
			distances[4] = -peakDeceleration * ((endSpeed - peakDeceleration * t5)/jerk + (5 * dsquare(peakDeceleration))/(6 * dsquare(jerk)));
			return distances[4] + distances[5] + distances[6];
		}
		else
		{
			// We don't need a constant deceleration phase
			peakDeceleration = -fastSqrtd((topSpeed - endSpeed) * jerk);
			distances[6] = -peakDeceleration * (endSpeed/jerk + dsquare(peakDeceleration)/(6 * dsquare(jerk)));
			distances[5] = (double)0.0;
			distances[4] = -peakDeceleration * (endSpeed/jerk + (5 * dsquare(peakDeceleration))/(6 * dsquare(jerk)));
			return distances[4] + distances[6];
		}
	}
	else
	{
		distances[4] = distances[5] = distances[6] = 0.0;
		return 0.0;
	}
}

// Calculate the movement profile when the start speed and acceleration and the end speed and acceleration are all zero and the maximum deceleration is equal tp minus the maximum acceleration.
// This is provided to optimise a case that I think will be common.
// Caller has already set endSpeed and endDeceleration to zero
// For an S-curve acceleration phase which starts at speed u and acceleration a, spends time t0 accelerating with jerk j to peak acceleration ap, then spends time t1 at constant acceleration ap, then spends time t0 reducing acceleration back to a:
//	s = u * (2 * t0 + t1) + a * (2 * t0^2 + 2 * t0 * t1 + ½ * t1^2) + j * (t0^3 + (3/2) * t0^2 * t1 + ½ * t0 * t1^2)
//	v = u + a * (2 * t0 + t1) + j * (t0 * t1 + t0^2)
//	ap = a + j * t0
// Given u = 0 and a = 0 for the move we are constructing:
//	s = j * (t0^3 + (3/2) * t0^2 * t1 + ½ * t0 * t1^2)
//	v = j * (t0 * t1 + t0^2) = j * t0 * (t0 + t1)
//	ap = j * t0
// The deceleration phase is a mirror image of the acceleration phase. We add a steady speed phase between acceleration and deceleration if we need more distance.
/*static*/ void MovementProfile::CalculateSimpleSCurvePlan(double distance) noexcept
{
	simple = true;
	// Determine whether the requested speed or the maximum acceleration is more limiting
	// The acceleration reached from a standing start is a = j * t and the speed reached is v = 0.5 * j * t^2.
	// So a^2 = j^2 * t^2 = 2 * v * j
	// The phase in which the acceleration is reducing will increase the speed by the same amount. Therefore we can reach acceleration a without exceeding speed v if a^2 >= v * j.
	if (dsquare(peakAcceleration) > topSpeed * jerk)
	{
		// In principle we can reach the requested speed without exceeding the maximum acceleration, without having to include a constant acceleration segment
		const double halfTimeToReqSpeed = fastSqrtd(topSpeed/jerk);
		const double distanceToReqSpeed = topSpeed * halfTimeToReqSpeed;
		if (2 * distanceToReqSpeed < distance)
		{
			// We can reach the requested speed and decelerate to zero again without exceeding the required distance. Generate a 5-phase move.
			distances[0] = distances[6] = OneSixthDouble * jerk * dcube(halfTimeToReqSpeed);
			distances[1] = distances[5] = 0.0;
			distances[2] = distances[4] = topSpeed * halfTimeToReqSpeed - distances[0];
			distances[3] = distance - 2 * distanceToReqSpeed;
			peakAcceleration = jerk * halfTimeToReqSpeed;
			peakDeceleration = -peakAcceleration;
			reachesRequestedSpeed = true;
			debugPrintf("Ss, rrs, no cas\n");
			return;
		}
		// Else we can't reach the requested speed without exceeding required distance, or we can only just reach it and then we need to start decelerating immediately. Fall through to beyond the else-part of this if-statement.
	}
	else
	{
		// We can't reach the requested speed without inserting a constant acceleration segment to avoid exceeding maximum acceleration
		const double basicDistance = 2 * dcube(peakAcceleration)/dsquare(jerk);	// distance if we reach max acceleration but have no constant acceleration segment
		if (basicDistance < distance)
		{
			// We need to insert a constant acceleration segment. We may also need to limit the top speed.
			// Calculate t1 in the above equations
			// From the above equations:	t1^2 * (0.5 * t0) + t1 * (1.5 * t0^2) + (t0^3 - s/j) = 0
			// Solve for t2 to get:			t1 = [-1.5 * t0^2 +/- sqrt(2.25 * t0^4 - 4 * 0.5 * t0 * (t0^3 - s/j))]/t0
			// Rearrange:					t1 = -1.5 * t0 +/- sqrt(2.25 * t0^2 - 2 * (t0^2 - s/(j*t0))
			// Simplify:					t1 = -1.5 * t0 +/- sqrt(0.25 * t0^2 + 2 * s/(j*t0))
			// But j * t0 = a, therefore:	t1 = -1.5 * t0 +/- sqrt(0.25 * t0^2 + 2 * s/a)  [Verified using Maxima]
			// s is half the total distance because we accelerate and decelerate again.
			const double timeToMaxAcceleration = peakAcceleration/jerk;
			distances[0] = distances[6] = OneSixthDouble * jerk * dcube(timeToMaxAcceleration);
			const double constantAccelerationTime = -(double)1.5 * timeToMaxAcceleration + fastSqrtd(OneQuarterDouble * dsquare(timeToMaxAcceleration) + distance/peakAcceleration);
			const double newTopSpeed = jerk * timeToMaxAcceleration * (timeToMaxAcceleration + constantAccelerationTime);
			if (newTopSpeed <= topSpeed)
			{
				// Generate a 5-phase move
				topSpeed = newTopSpeed;
				distances[1] = distances[5] = OneHalfDouble * constantAccelerationTime * newTopSpeed;
				distances[2] = 2 * (timeToMaxAcceleration * newTopSpeed - distances[0]);
				distances[3] = distances[4] = (double)0.0;
				reachesRequestedSpeed = false;
				debugPrintf("Ss, not rrs, cas\n");
			}
			else
			{
				// We need to limit the constant acceleration time in order to limit the top speed, and add a constant speed phase. Generate a 7-phase move.
				//	v = j * t0 * (t0 + t1) therefore t1 = v/(j * t0) - t0 = v/a - t0
				const double revisedConstantAccelerationTime = topSpeed/peakAcceleration - timeToMaxAcceleration;
				distances[1] = distances[5] = OneHalfDouble * revisedConstantAccelerationTime * topSpeed;
				distances[2] = distances[4] = timeToMaxAcceleration * topSpeed - distances[0];
				distances[3] = distance - 2 * (distances[0] + distances[1] + distances[2]);
				reachesRequestedSpeed = true;
				debugPrintf("Ss, rrs, cas\n");
			}
			return;
		}
		// Else fall through
	}

	// If we get here then we can reach neither requestedSpeed nor maxAcceleration without exceeding totalDistance. Generate a 3-phase move.
	const double halfTimeToTopSpeed = fastCubeRootd(distance * OneHalfDouble / jerk);
	topSpeed = jerk * dsquare(halfTimeToTopSpeed);
	distances[0] = distances[6] = OneTwelfthDouble * distance;
	distances[1] = distances[3] = distances[4] = distances[5] = 0.0;
	distances[2] = distance - 2 * distances[0];
	peakAcceleration = jerk * halfTimeToTopSpeed;
	peakDeceleration = -peakAcceleration;
	reachesRequestedSpeed = false;
	debugPrintf("Ss, not rrs, no cas\n");
}

// Calculate a general 3rd order motion plan for a sequence of moves
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
// In all cases we have to achieve the required distance, end speed, and zero end acceleration (3 constraints).
// The possible profiles are:
// 1. Increase acceleration, decrease acceleration through and beyond the peak speed, reduce deceleration.
//    3 phases.
//	  Profile is viable of peak acceleration, peak deceleration and top speed are all within limits. The solution is quartic.
// 2. Increase acceleration to max, constant acceleration, reduce acceleration through and beyond the peak speed until we reach max deceleration, constant deceleration, reduce deceleration.
//	  5 phases; max acceleration and max deceleration are additional constraints.
//	  Profile is viable if peak speed does not exceed top speed.
// 3. As #2 but we don't reach max acceleration and we have no constant acceleration segment.
// 4. As #2 but we don't reach max deceleration and we have no constant deceleration segment.
// 5. Increase acceleration, decrease acceleration to zero at top speed, maintain top speed, increase deceleration, decrease deceleration.
//    4 phases; top speed is an additional constraint.
//    Profile is viable if peak acceleration and peak deceleration are within limits. The solution to this is ?quartic?.
// 6. Increase acceleration to max, constant acceleration, reduce acceleration reaching zero at top speed, constant speed, increase deceleration, constant deceleration, reduce deceleration.
//	  7 phases; top speed, max acceleration and max deceleration are additional constraints. ?missing constraint?
// 7. As #4 but we don't reach max acceleration and there is no max acceleration phase.
// 8. As #4 but we don't reach max deceleration and there is no max deceleration phase.
//
// For profiles #5,6,7,8 we can calculate the acceleration and deceleration parts of the profile separately, then we can adjust the constant speed profile to make the required distance.
// This has the simplest calculations, so we try it first. If it results in excessive distance then we have to use one of profiles #1,2,3,4 instead.
void MovementProfile::CalculateGeneralSCurvePlan(double distance) noexcept
{
	simple = false;

	// First make sure we don't have a top speed that is lower than the start or end speed, because that would lead to negative phase times
	if (topSpeed < startSpeed || topSpeed < startSpeed + OneHalfDouble * dsquare(startAcceleration)/jerk)
	{
		debugPrintf("Increasing top speed from %.4e to %.4e due to start speed/accel\n", topSpeed, startSpeed);
		topSpeed = startSpeed;
	}
	if (topSpeed < endSpeed)
	{
		debugPrintf("Increasing top speed from %.4e to %.4e due to end speed\n", topSpeed, endSpeed);
		topSpeed = endSpeed;
	}

	// See if any of plans 5,6,7,8 are feasible
	const double accelDistance = CalculateAccelerationParameters();
	if (accelDistance <= distance)
	{
		const double decelDistance = CalculateDecelerationParameters();
		const double excessDistance = distance - (accelDistance + decelDistance);
		if (excessDistance >= (double)0.0)
		{
			distances[3] = excessDistance;
			debugPrintf("Gs, rrs\n");
			return;
		}
	}

	// If we get here then we can't reach top speed without exceeding the available distance, so we must use one of plans 1,2,3,4.
	// Plans 2, 3 and 4 are simpler to calculate than 1 I suspect?
	//TODO

	// Try plan 1
#if 0
	debugPrintf("Invoking quartic solver, u=%.7e, v=%.7e, a=%.7e, j=%.7e, d=%.7e\n", startSpeed, endSpeed, startAcceleration, jerk, distance);

	// For debugging, first calculate the minimum distance, which is when either t0=0 or t6=0
	{
		const double square = jerk * (startSpeed - endSpeed) + OneHalfDouble * dsquare(startAcceleration);
		if (square >= (double)0.0)
		{
			// t0 = 0 is viable
			const double tempT6 = fastSqrtd(square)/jerk;
			const double minDistance = 2 * tempT6 * startSpeed - jerk * dcube(tempT6) + startAcceleration * (startSpeed + startAcceleration * tempT6)/jerk + dcube(startAcceleration)/(3 * dsquare(jerk));
			debugPrintf("Min distance %.3f when t0=0 t6=%.1f\n", minDistance, tempT6);
		}
		const double square2 = jerk * (endSpeed - startSpeed) + OneHalfDouble * dsquare(startAcceleration);
		if (square2 >= (double)0.0)
		{
			// t6 = 0 may be viable
			const double tempT0 = (fastSqrtd(square2) - startAcceleration)/jerk;
			if (tempT0 >= (double)0.0)
			{
				const double minDistance = 2 * tempT0 * startSpeed + jerk * dcube(tempT0) + startAcceleration * (startSpeed + 2 * startAcceleration * tempT0)/jerk + 3 * startAcceleration * dsquare(tempT0) + dcube(startAcceleration)/(3 * dsquare(jerk));
				debugPrintf("Min distance %.3f when t0=%.1f, t6=0\n", minDistance, tempT0);
			}
		}
	}
#endif

	// The following quartic equation solves for t0, see Maxima worksheet
	// The coefficients are a little simpler if we express them in terms of (endSpeed - startSpeed) instead of endSpeed
	const double speedDiff = endSpeed - startSpeed;

#if 1	// using speedDiff
	const double coeff0 = 144 * dcube(jerk) * startSpeed * (startAcceleration * distance - 2 * speedDiff * endSpeed)
						- 72 * dcube(jerk) * dcube(speedDiff)
						+ 12 * dsquare(jerk) * dsquare(startAcceleration) * (6 * dsquare(startSpeed) - 3 * dsquare(speedDiff) + 4 * startAcceleration * distance)
						- 72 * dsquare(dsquare(jerk)) * dsquare(distance)
						+ 6 * jerk * dsquare(dsquare(startAcceleration)) * (startSpeed + 3 * endSpeed)
						+ dsquare(dcube(startAcceleration));

//	+144*dcube(jerk)*startAcceleration*startSpeed*distance
//	-288*dcube(jerk)*speedDiff*dsquare(startSpeed)
//	-288*dcube(jerk)*dsquare(speedDiff)*startSpeed
//	-72*dcube(jerk)*dcube(speedDiff)
//	+72*dsquare(jerk)*dsquare(startAcceleration)*dsquare(startSpeed)
//	-36*dsquare(jerk)*dsquare(startAcceleration)*dsquare(speedDiff)
//	+48*dsquare(jerk)*dcube(startAcceleration)*distance
//	-72*dsquare(dsquare(jerk))*dsquare(distance)
//	+18*jerk*dsquare(dsquare(startAcceleration))*speedDiff
//	+24*jerk*dsquare(dsquare(startAcceleration))*startSpeed
//	+dsquare(dcube(startAcceleration))

#else	// not using speedDiff
	const double coeff0 = 72 * dcube(jerk) * startSpeed * endSpeed * (startSpeed - endSpeed)
						+ 72 * dcube(jerk) * (dcube(startSpeed) - dcube(endSpeed))
						+ 144 * dcube(jerk) * startAcceleration * startSpeed * distance
						+ 36 * dsquare(jerk) * dsquare(startAcceleration) * (dsquare(startSpeed) - dsquare(endSpeed) + 2 * startSpeed * endSpeed)
						+ 48 * dsquare(jerk) * dcube(startAcceleration) * distance
						- 72 * dsquare(dsquare(jerk)) * dsquare(distance)
						+ 6 * jerk * dsquare(dsquare(startAcceleration)) * (startSpeed + 3 * endSpeed)
						+ dsquare(dcube(startAcceleration));

//	+72*dcube(jerk)*dcube(startSpeed)
//	-72*dcube(jerk)*dcube(endSpeed)
//	+72*dcube(jerk)*dsquare(startSpeed)*endSpeed
//	-72*dcube(jerk)*startSpeed*dsquare(endSpeed)
//	+144*dcube(jerk)*startAcceleration*startSpeed*distance
//	+36*dsquare(jerk)*dsquare(startAcceleration)*dsquare(startSpeed)
//	-36*dsquare(jerk)*dsquare(startAcceleration)*dsquare(endSpeed)
//	+72*dsquare(jerk)*dsquare(startAcceleration)*startSpeed*endSpeed
//	+48*dsquare(jerk)*dcube(startAcceleration)*distance
//	-72*dsquare(dsquare(jerk))*dsquare(distance)
//	+6*jerk*dsquare(dsquare(startAcceleration))*startSpeed
//	+18*jerk*dsquare(dsquare(startAcceleration))*endSpeed
//	+dsquare(dcube(startAcceleration))

#endif

#if 1	// using speedDiff
	const double coeff1 = -12 * jerk * (  12 * dsquare(jerk) * (startAcceleration * (dsquare(speedDiff) - 2 * dsquare(startSpeed)) - 2 * distance * (jerk * startSpeed + dsquare(startAcceleration)))
										- 4 * jerk * dcube(startAcceleration) * (4 + startSpeed + 3 * speedDiff)
										- startAcceleration*dsquare(dsquare(startAcceleration))
									   );

//	coeff1=-(12*jerk*(	-24*dsquare(jerk)*startAcceleration*dsquare(startSpeed)
//						+12*dsquare(jerk)*startAcceleration*dsquare(speedDiff)
//						-24*dsquare(jerk)*dsquare(startAcceleration)*distance
//						-16*jerk*dcube(startAcceleration)*startSpeed
//						-12*jerk*dcube(startAcceleration)*speedDiff
//						-24*dcube(jerk)*startSpeed*distance
//						-startAcceleration*dsquare(dsquare(startAcceleration))
//					 )
//			)

#else	// not using speedDiff
	const double coeff1 = 12 * jerk * (  24 * dcube(jerk) * startSpeed * distance
									   - 12 * dsquare(jerk) * startAcceleration * (dsquare(endSpeed) - dsquare(startSpeed) - 2 * (startSpeed * endSpeed + startAcceleration * distance))
									   + 4 * jerk * dcube(startAcceleration) * (startSpeed + 3 * endSpeed)
									   + startAcceleration * dsquare(dsquare(startAcceleration))
									  );

//	-(12*jerk*
//			(
//				+12*dsquare(jerk)*startAcceleration*dsquare(endSpeed)
//				-12*dsquare(jerk)*startAcceleration*dsquare(startSpeed)
//				-24*dsquare(jerk)*startAcceleration*startSpeed*endSpeed
//				-24*dsquare(jerk)*dsquare(startAcceleration)*distance
//				-24*dcube(jerk)*startSpeed*distance
//				-4*jerk*dcube(startAcceleration)*startSpeed
//				-12*jerk*dcube(startAcceleration)*endSpeed
//				-startAcceleration*dsquare(dsquare(startAcceleration))
//			)
//	 )

#endif

#if 1	// using speedDiff
	const double coeff2 = -18 * dsquare(jerk) * (
												 + 4 * dsquare(jerk) * (dsquare(speedDiff) - 6 * startAcceleration * distance)
												 - 4 * jerk * dsquare(startAcceleration) * (startSpeed + 5 * endSpeed)
												 - 3 * dsquare(dsquare(startAcceleration))
												);

//	coeff2=-(18*dsquare(jerk)*(
//								-24*jerk*dsquare(startAcceleration)*startSpeed
//								-20*jerk*dsquare(startAcceleration)*speedDiff
//								-24*dsquare(jerk)*startAcceleration*distance
//								+4*dsquare(jerk)*dsquare(speedDiff)
//								-3*dsquare(dsquare(startAcceleration))
//							  )
//			)

#else	// not using speedDiff
	const double coeff2 = -(  18 * dsquare(jerk) * (  4 * dsquare(jerk) * (dsquare(startSpeed) + dsquare(endSpeed) - 2 * startSpeed * endSpeed)
													- 24 * dsquare(jerk) * startAcceleration * distance
													- 4 * jerk * dsquare(startAcceleration) * (startSpeed + 5 * endSpeed)
													- 3 * dsquare(dsquare(startAcceleration))
												   )
						   );

#endif

#if 1	// using speedDiff
	const double coeff3 = 48 * dcube(jerk) * (  2 * dcube(startAcceleration)
											  + 3 * jerk * startAcceleration * (endSpeed + speedDiff)
											  + 3 * distance * dsquare(jerk)
											 );
//	coeff3=48*dcube(jerk)*(  3*jerk*startAcceleration*startSpeed
//							+6*jerk*startAcceleration*speedDiff
//							+2*dcube(startAcceleration)
//							+3*distance*dsquare(jerk)
//						  )

#else	// not using speedDiff
	const double coeff3 = 48 * dcube(jerk) * (  2 * dcube(startAcceleration)
											  + 3 * jerk * startAcceleration * (2 * endSpeed - startSpeed)
											  + 3 * distance * dsquare(jerk)
											 );

#endif

#if 1	// using speedDiff
	const double coeff4 = 36 * dsquare(dsquare(jerk)) * (  dsquare(startAcceleration)
														 + 2 * jerk * speedDiff
														);
//	coeff4=36*dsquare(dsquare(jerk))*(dsquare(startAcceleration)+2*jerk*speedDiff)
#else
	const double coeff4 = 36 * dsquare(dsquare(jerk)) * (  dsquare(startAcceleration)
														 + 2 * jerk * (endSpeed - startSpeed)
														);
#endif

	double rslt[4];
	const size_t numSolutions = SolveQuartic(coeff4, coeff3, coeff2, coeff1, coeff0, rslt);

	debugPrintf("Coefficients %.7e %.7e %.7e %.7e %.7e, solutions", coeff4, coeff3, coeff2, coeff1, coeff0);
	for (size_t i = 0; i < numSolutions; ++i) { debugPrintf(" %.7e", rslt[i]); }
	debugPrintf("\n");

	// We want a solution in which t0, t2 and t6 are all non-negative. If there is more than one, we want the one with the lowest sum.
	double t0, t2, t6;
	bool foundSolution = false;
	double bestTime;
	for (size_t i = 0; i < numSolutions; ++i)
	{
		const double tempT0 = rslt[i];
		if (tempT0 >= (double)0.0)
		{
#if 0
			// Maxima gives t6 = sqrt((2 * jerk * (startSpeed - endSpeed + 2 * startAcceleration * tempT0) + startAcceleration^2)/(2 * jerk^2) + tempT0^2)
			const double square = (2 * jerk * (startSpeed - endSpeed + 2 * startAcceleration * tempT0) + dsquare(startAcceleration))/(2 * dsquare(jerk)) + dsquare(tempT0);
			if (square >= (double)0.0)
			{
				const double tempT6 = fastSqrtd(square);
#else
			// Unfortunately that doesn't tell us the sign of t6 and some solutions give a negative value.
			// So instead we have to use this:
			// t6=-((6*jerk^3*t0^3+18*jerk^2*startAcceleration*t0^2+startSpeed*(12*jerk^2*t0+6*jerk*startAcceleration)+12*jerk*startAcceleration^2*t0+2*startAcceleration^3-6*distance*jerk^2)
			//		/(6*jerk^3*t0^2+12*jerk^2*startAcceleration*t0+6*jerk^2*startSpeed+3*jerk*startAcceleration^2+6*endSpeed*jerk^2))
			const double tempT6 = -((  6 * dcube(jerk) * dcube(tempT0) + 18 * (dsquare(jerk) * startAcceleration) * dsquare(tempT0) + startSpeed * (12 * dsquare(jerk) * tempT0 + 6 * jerk * startAcceleration)
									 + 12 * jerk * dsquare(startAcceleration) * tempT0 + 2 * dcube(startAcceleration) - 6 * dsquare(jerk) * distance
								    )
								    / (6 * dcube(jerk) * dsquare(tempT0) + 12 * (dsquare(jerk) * startAcceleration) * tempT0 + 6 * dsquare(jerk) * (startSpeed + endSpeed) + 3 * jerk * dsquare(startAcceleration))
								   );
			if (tempT6 >= (double)0.0)
			{
#endif
				const double tempT2 = tempT6 + tempT0 + startAcceleration/jerk;
				const double totalTime = tempT0 + tempT2 + tempT6;
				if (!foundSolution || totalTime < bestTime)
				{
					bestTime = totalTime;
					t0 = tempT0;
					t2 = tempT2;
					t6 = tempT6;
					foundSolution = true;
				}
			}
		}
	}

	if (foundSolution)
	{
		const double dist0 = (startSpeed + (OneHalfDouble * startAcceleration + OneSixthDouble * jerk * t0) * t0) * t0;
		const double u2 = startSpeed + (startAcceleration + OneHalfDouble * jerk * t0) * t0;
		const double a2 = startAcceleration + jerk * t0;
		const double dist2 = (u2 + (OneHalfDouble * a2 - OneSixthDouble * jerk * t2) * t2) * t2;
		const double dist6 = (endSpeed + OneSixthDouble * jerk * dsquare(t6)) * t6;
		const double a6 = -OneHalfDouble * jerk * t6;

		debugPrintf("Quartic solution: t0,t2,t6 = %.1f %.1f %.1f, distances %.3f %.3f %.3f total %.3g, orig dist %.3g\n", t0, t2, t6, dist0, dist2, dist6, dist0 + dist2 + dist6, distance);

		// Check that max acceleration isn't exceeded
		if (a2 > peakAcceleration || a6 < peakDeceleration)
		{
			//TODO use another plan
			debugPrintf("Acceleration limits exceeded: %.4e %.4e vs. %.4e %.4e\n", a2, a6, peakAcceleration, peakDeceleration);
		}

		distances[0] = dist0;
		distances[1] = distances[3] = distances[4] = distances[5] = 0.0;
		distances[2] = dist2;
		distances[6] = dist6;
	}
	else
	{
		debugPrintf("No quartic solution\n");
		//TODO
		distances[0] = distances[1] = distances[3] = distances[4] = distances[5] = distances[6] = 0.0;
	}
}

#endif

// End
