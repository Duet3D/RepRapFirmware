/*
 * MovementError.cpp
 *
 *  Created on: 23 Apr 2025
 *      Author: David
 */

#include "MovementError.h"

const char *_ecv_array GetMovementErrorText(MovementError err) noexcept
{
	switch (err)
	{
	case MovementError::microstep_position_too_large:	return "microstep position too large";
	case MovementError::unreachable_position:			return "unreachable position";
	case MovementError::move_duration_too_long:			return "move duration too long";

	case MovementError::ok:
	case MovementError::noMovement:
	default:
		return "no error";
	}
}

// End
