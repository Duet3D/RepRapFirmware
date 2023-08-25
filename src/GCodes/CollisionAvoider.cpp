/*
 * CollisionAvoider.cpp
 *
 *  Created on: 15 Aug 2022
 *      Author: David
 */

#include <GCodes/CollisionAvoider.h>
#include <Movement/MoveDebugFlags.h>

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
void CollisionAvoider::Set(int axisL, int axisH, float sep) noexcept
{
	lowerAxis = axisL;
	upperAxis = axisH;
	minSeparation = sep;
	lowerAxisMax = -std::numeric_limits<float>::infinity();
	upperAxisMin = std::numeric_limits<float>::infinity();
}

// If the new move doesn't risk a collision, update the position accumulators and return true; else return false
bool CollisionAvoider::UpdatePositions(const float axisPositions[], AxesBitmap axesHomed) noexcept
{
	if (IsValid() && axesHomed.IsBitSet(lowerAxis) && axesHomed.IsBitSet(upperAxis))
	{
		const float newLowerMax = max<float>(axisPositions[lowerAxis], lowerAxisMax);
		const float newUpperMin = min<float>(axisPositions[upperAxis], upperAxisMin);
		if (newLowerMax + minSeparation > newUpperMin)
		{
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::CollisionData))
			{
				const char *const axisLetters = reprap.GetGCodes().GetAxisLetters();
				debugPrintf("Potential collision between axis %c at %.1f and axis %c at %.1f\n", axisLetters[lowerAxis], (double)newLowerMax, axisLetters[upperAxis], (double)newUpperMin);
			}
			return false;
		}
		lowerAxisMax = newLowerMax;
		upperAxisMin = newUpperMin;
	}
	return true;
}

#endif

// End
