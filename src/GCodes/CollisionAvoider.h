/*
 * CollisionAvoider.h
 *
 *  Created on: 15 Aug 2022
 *      Author: David
 */

#ifndef SRC_GCODES_COLLISIONAVOIDER_H_
#define SRC_GCODES_COLLISIONAVOIDER_H_

#include <RepRapFirmware.h>

#if SUPPORT_ASYNC_MOVES

#include <Movement/RawMove.h>

class CollisionAvoider
{
public:
	CollisionAvoider() noexcept;

	bool IsValid() const noexcept;
	int GetLowerAxis() const noexcept { return lowerAxis; }
	int GetUpperAxis() const noexcept { return upperAxis; }
	float GetMinSeparation()const noexcept { return minSeparation; }

	// Reset the position accumulators
	void ResetPositions(const float positions[], AxesBitmap whichPositions) noexcept;

	// If the new move doesn't risk a collision, update the position accumulators and return true; else return false
	bool UpdatePositions(const float axisPositions[], AxesBitmap axesHomed) noexcept;

	// Set the parameters
	void Set(int axisL, int axisH, float sep) noexcept;

private:
	float minSeparation;
	float lowerAxisMax;
	float upperAxisMin;
	int lowerAxis;
	int upperAxis;
};

#endif

#endif /* SRC_GCODES_COLLISIONAVOIDER_H_ */
