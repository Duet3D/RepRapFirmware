/*
 * CollisionAvoider.cpp
 *
 *  Created on: 15 Aug 2022
 *      Author: David
 */

#include <GCodes/CollisionAvoider.h>

CollisionAvoider::CollisionAvoider() noexcept
{
	lowerAxis = upperAxis = -1;
}

bool CollisionAvoider::IsValid() const noexcept
{
	return lowerAxis >= 0 && upperAxis > lowerAxis;
}

// End
