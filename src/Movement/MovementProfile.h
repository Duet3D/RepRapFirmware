/*
 * MoveProfile.h
 *
 *  Created on: 10 Sept 2025
 *      Author: David
 */

#ifndef SRC_MOVEMENT_MOVEMENTPROFILE_H_
#define SRC_MOVEMENT_MOVEMENTPROFILE_H_

#include <RepRapFirmware.h>

#if SUPPORT_S_CURVE

// Constants that we use in double precision calculations
constexpr double OneHalfDouble = (double)0.5;
constexpr double OneQuarterDouble = (double)0.25;
constexpr double OneSixthDouble = (double)1.0/(double)6.0;
constexpr double OneTwelfthDouble = (double)1.0/(double)12.0;
constexpr double TwoThirdsDouble = (double)2.0/(double)3.0;

// Class to represent a movement profile that may cover several moves in a DDARing
// The end acceleration of a profile is always zero. Currently the end speed is always zero too, but that may change in future.
class MovementProfile
{
public:
	void Invalidate() noexcept { numberOfMovesCovered = 0; }

	double NonDecelDistance() const noexcept		// return the distance left excluding deceleration distance
		{ return distances[0] + distances[1] + t2NonDecelDistance + distances[3]; }
	bool ReducingDeceleration() const noexcept		// return true if we are in the reducing deceleration phase
		{ return distances[0] + distances[2] + distances[3] + distances[4] + distances[5] == (double)0.0; }

	double CalculateAccelerationParameters() noexcept;
	double CalculateDecelerationParameters() noexcept;
	void CalculateSimpleSCurvePlan(double distance) noexcept
		pre(startSpeed == 0.0; endSpeed == 0.0; topSpeed > 0.0; startAcceleration == 0.0; peakAcceleration > 0.0; peakDeceleration == -peakAcceleration);
	void CalculateSimpleFivePhasePlan(double distance) noexcept
		pre(startSpeed == 0.0; endSpeed == 0.0; topSpeed > 0.0; startAcceleration == 0.0; peakAcceleration > 0.0; peakDeceleration == -peakAcceleration);
	void CalculateGeneralSCurvePlan(double distance) noexcept
		pre(startSpeed >= 0.0; endSpeed >= 0.0; topSpeed > 0.0; peakAcceleration > 0.0);

	void DebugPrint() const noexcept;
	void CheckForShortSegments() const noexcept;

	static double SmallestNonNegativeCubicSolution(double a, double b, double c, double d) noexcept;
	static double SmallestNonNegativeQuadraticSolution(double a, double b, double c) noexcept;

	double startSpeed;								// the speed at the start of the profile
	double topSpeed;								// top speed of the profile
	double endSpeed;								// end speed of the profile
	double startAcceleration;						// the acceleration or deceleration at the start of this move, may be positive or negative. Valid for the first un-commited move in the queue.
	double peakAcceleration;						// the acceleration in the steady acceleration phase, if any. Valid if phase1Distance != 0.
	double peakDeceleration;						// the deceleration in the steady deceleration phase, if any. This is negative if there is a peak deceleration phase. Valid if phase5Distance != 0.
	double jerk;
	double distances[7];							// the distances of each phase
	double t2NonDecelDistance;						// that part of distances[2] that is not deceleration

	unsigned int numberOfMovesCovered = 0;			// if zero then the profile has not been calculated and the other fields are meaningless
	uint32_t scheduledMovesWhenCreated;				// number of moves in the ring when we created this plan
	bool reachesRequestedSpeed;						// true if this profile reaches the requested speed, so if more moves are added there is no point in recomputing the acceleration phase
	bool usesAllMoves;								// true if this profile covers all moves in the ring when created. If false then there is a stop-point before further moves in the ring.
	bool simple;									// for debugging only
};

#endif

#endif /* SRC_MOVEMENT_MOVEMENTPROFILE_H_ */
