/*
 * MoveProfile.h
 *
 *  Created on: 10 Sept 2025
 *      Author: David
 */

#ifndef SRC_MOVEMENT_MOVEMENTPROFILE_H_
#define SRC_MOVEMENT_MOVEMENTPROFILE_H_

#include <RepRapFirmware.h>

// Class to represent the profile of a single move described by a DDA.
// Also used as the base class for MovementProfile which describes the profile of a sequence of moves.
struct MoveProfile
{
	float startSpeed;								// the speed at the start of the move. Valid for the first un-commited move in the queue.
	float topSpeed;									// top speed of the move. Valid???
	float endSpeed;									// end speed of the move. Valid (and zero) for the last move in the queue
#if SUPPORT_S_CURVE
	float startAcceleration;						// the acceleration or deceleration at the start of this move, may be positive or negative. Valid for the first un-commited move in the queue.
	float peakAcceleration;							// the acceleration in the steady acceleration phase, if any. Valid if phase1Distance != 0.
    float peakDeceleration;							// the deceleration in the steady deceleration phase, if any. This is negative if there is a peak deceleration phase. Valid if phase5Distance != 0.
    float endAcceleration;							// the acceleration or deceleration at the end of the move. Valid (and zero) for the last move in the queue.
    float jerk;
    float distances[7];								// the distances of each phase

    float TotalAccelDistance() const noexcept { return distances[0] + distances[1] + distances[2]; }
    float TotalDecelDistance() const noexcept { return distances[4] + distances[5] + distances[6]; }
#endif
};

#if SUPPORT_S_CURVE

// Class to represent a movement profile that may cover several moves in a DDARing
class MovementProfile : public MoveProfile
{
public:
	void Invalidate() noexcept { numberOfMovesCovered = 0; }

	float totalDistance;							// total distance to be covered
	unsigned int numberOfMovesCovered = 0;			// if zero then the profile has not been calculated and the other fields are meaningless
	uint32_t scheduledMovesWhenCreated;				// number of moves in the ring when we created this plan
	bool reachesRequestedSpeed;						// true if this profile reaches the requested speed, so if more moves are added there is no point in recomputing the acceleration phase
	bool usesAllMoves;								// true if this profile covers all moves in the ring when created. If false then there is a stop-point before further moves in the ring.
};

#endif

#endif /* SRC_MOVEMENT_MOVEMENTPROFILE_H_ */
