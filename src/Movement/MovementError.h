/*
 * MovementError.h
 *
 *  Created on: 23 Apr 2025
 *      Author: David
 */

#ifndef SRC_MOVEMENT_MOVEMENTERROR_H_
#define SRC_MOVEMENT_MOVEMENTERROR_H_

#include <RepRapFirmware.h>

// Class to represent the types of movement error that can occur but can't easily be detected before the move is queued
enum class MovementError : uint8_t
{
	ok, 									// move can be executed
	noMovement,								// no significant movement was commanded so the move can be ignored
	microstep_position_too_large,			// exceeded about +/- 2^31 microsteps from zero position
	unreachable_position,					// the kinematics can't reach the requested position
	move_duration_too_long					// the move would take more than about 2^31 step clocks
};

const char *_ecv_array GetMovementErrorText(MovementError err) noexcept;

#endif /* SRC_MOVEMENT_MOVEMENTERROR_H_ */
