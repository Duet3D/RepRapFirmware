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
	// Bit numbers in the move debug bitmap. The lowest 8 bits are the default settings
	constexpr unsigned int PrintBadMoves = 0;
	constexpr unsigned int PrintAllMoves = 1;
	constexpr unsigned int CollisionData = 2;
	constexpr unsigned int PrintTransforms = 3;

	constexpr unsigned int Lookahead = 8;
	constexpr unsigned int ZProbing = 9;
	constexpr unsigned int AxisAllocation = 10;
	constexpr unsigned int SimulateSteppingDrivers = 11;
	constexpr unsigned int Segments = 12;
	constexpr unsigned int PhaseStep = 13;
}

#endif /* SRC_MOVEMENT_MOVEDEBUGFLAGS_H_ */
