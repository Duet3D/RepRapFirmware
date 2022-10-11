/*
 * CollisionAvoider.cpp
 *
 *  Created on: 15 Aug 2022
 *      Author: David
 */

#include <GCodes/CollisionAvoider.h>

#if SUPPORT_ASYNC_MOVES

#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

CollisionAvoider::CollisionAvoider() noexcept
{
	lowerAxis = upperAxis = -1;
}

bool CollisionAvoider::IsValid() const noexcept
{
	return lowerAxis >= 0 && upperAxis >= 0;
}

// Reset the position accumulators
void CollisionAvoider::ResetPositions(const float positions[], AxesBitmap whichPositions) noexcept
{
	if (IsValid())
	{
		if (whichPositions.IsBitSet(lowerAxis))
		{
			lowerAxisMax = positions[lowerAxis];
		}
		if (whichPositions.IsBitSet(upperAxis))
		{
			upperAxisMin = positions[upperAxis];
		}
	}
}

// Set the parameters
void CollisionAvoider::Set(int axisL, int axisH, float sep, const float positions[]) noexcept
{
	lowerAxis = axisL;
	upperAxis = axisH;
	minSeparation = sep;
	lowerAxisMax = positions[lowerAxis];
	upperAxisMin = positions[upperAxis];
}

bool CollisionAvoider::UpdatePositions(const float axisPositions[]) noexcept
{
	const float newLowerMax = max<float>(axisPositions[lowerAxis], lowerAxisMax);
	const float newUpperMin = min<float>(axisPositions[upperAxis], upperAxisMin);
	if (newLowerMax + minSeparation > newUpperMin)
	{
		if (reprap.Debug(moduleMove))
		{
			const char *const axisLetters = reprap.GetGCodes().GetAxisLetters();
			debugPrintf("Potential collision between axis %c at %.1f and axis %c at %.1f\n", axisLetters[lowerAxis], (double)newLowerMax, axisLetters[upperAxis], (double)newUpperMin);
		}
		return false;
	}
	lowerAxisMax = newLowerMax;
	upperAxisMin = newUpperMin;
	return true;
}

#endif

// End
