/*
 * MovementProfile.cpp
 *
 *  Created on: 10 Sept 2025
 *      Author: David
 */

#include "MovementProfile.h"

#if SUPPORT_S_CURVE

void MovementProfile::DebugPrint() noexcept
{
	debugPrintf("Plan: d=[%.4g %.4g %.4g %.4g %.4g %.4g %.4g] v=[%.4g %.4g %.4g] a=[%.4g %.4g %.4g 0.0] j=%.4g\n",
				distances[0], distances[1], distances[2], distances[3], distances[4], distances[5], distances[6],
				startSpeed, topSpeed, endSpeed,
				startAcceleration, peakAcceleration, peakDeceleration,
				jerk
			   );
}

#endif

// End
