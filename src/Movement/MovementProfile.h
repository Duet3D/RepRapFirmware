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

// Class to represent a movement profile that may cover several moves in a DDARing
class MovementProfile
{
public:
	void Invalidate() noexcept { numberOfMovesCovered = 0; }
    double NonDecelDistance() const noexcept		// return the distance left excluding deceleration distance
    	{ return distances[0] + distances[1] + distances[2] + distances[3]; }
    bool ReducingDeceleration() const noexcept		// return true if we are in the reducing deceleration phase
    	{ return distances[0] + distances[2] + distances[3] + distances[4] + distances[5] == (double)0.0; }

    void DebugPrint() noexcept;

	double startSpeed;								// the speed at the start of the move. Valid for the first un-commited move in the queue.
	double topSpeed;								// top speed of the move. Valid???
	double endSpeed;								// end speed of the move. Valid (and zero) for the last move in the queue
	double startAcceleration;						// the acceleration or deceleration at the start of this move, may be positive or negative. Valid for the first un-commited move in the queue.
	double peakAcceleration;						// the acceleration in the steady acceleration phase, if any. Valid if phase1Distance != 0.
	double peakDeceleration;						// the deceleration in the steady deceleration phase, if any. This is negative if there is a peak deceleration phase. Valid if phase5Distance != 0.
	double endAcceleration;							// the acceleration or deceleration at the end of the move. Valid (and zero) for the last move in the queue.
    double jerk;
    double distances[7];							// the distances of each phase

	unsigned int numberOfMovesCovered = 0;			// if zero then the profile has not been calculated and the other fields are meaningless
	uint32_t scheduledMovesWhenCreated;				// number of moves in the ring when we created this plan
	bool reachesRequestedSpeed;						// true if this profile reaches the requested speed, so if more moves are added there is no point in recomputing the acceleration phase
	bool usesAllMoves;								// true if this profile covers all moves in the ring when created. If false then there is a stop-point before further moves in the ring.
};

#endif

#endif /* SRC_MOVEMENT_MOVEMENTPROFILE_H_ */
