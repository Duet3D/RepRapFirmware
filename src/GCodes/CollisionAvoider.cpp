/*
 * CollisionAvoider.cpp
 *
 *  Created on: 15 Aug 2022
 *      Author: David
 */

#include <GCodes/CollisionAvoider.h>

#if SUPPORT_ASYNC_MOVES

CollisionAvoider::CollisionAvoider() noexcept
{
	lowerAxis = upperAxis = -1;
}

bool CollisionAvoider::IsValid() const noexcept
{
	return lowerAxis >= 0 && upperAxis >= 0;
}

// Reset the position accumulators
void CollisionAvoider::ResetPositions(const float positions[]) noexcept
{
	if (IsValid())
	{
		lowerAxisMax = positions[lowerAxis];
		upperAxisMin = positions[upperAxis];
	}
}

// Set the parameters
void CollisionAvoider::Set(int axisL, int axisH, float sep, const float positions[]) noexcept
{
	lowerAxis = axisL;
	upperAxis = axisH;
	minSeparation = sep;
	ResetPositions(positions);
}

bool CollisionAvoider::UpdatePositions(const float axisPositions[]) noexcept
{
	const float newLowerMax = max<float>(axisPositions[lowerAxis], lowerAxisMax);
	const float newUpperMin = min<float>(axisPositions[upperAxis], upperAxisMin);
	if (newLowerMax + minSeparation > newUpperMin)
	{
		return false;
	}
	lowerAxisMax = newLowerMax;
	upperAxisMin = newUpperMin;
	return true;
}

#endif

// End
