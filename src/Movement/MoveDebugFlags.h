/*
 * MoveDebugFlags.h
 *
 *  Created on: 20 Aug 2023
 *      Author: David
 */

#ifndef SRC_MOVEMENT_MOVEDEBUGFLAGS_H_
#define SRC_MOVEMENT_MOVEDEBUGFLAGS_H_

namespace MoveDebugFlags
{
	// Bit numbers in the move debug bitmap
	constexpr unsigned int PrintBadMoves = 0;
	constexpr unsigned int PrintAllMoves = 1;
	constexpr unsigned int SimulateSteppingDrivers = 2;
	constexpr unsigned int PrintTransforms = 3;
	constexpr unsigned int ZProbing = 4;
	constexpr unsigned int CollisionData = 5;
	constexpr unsigned int AxisAllocation = 6;
	constexpr unsigned int Lookahead = 7;
}

#endif /* SRC_MOVEMENT_MOVEDEBUGFLAGS_H_ */
