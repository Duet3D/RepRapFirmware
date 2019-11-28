/*
 * RestorePoint.cpp
 *
 *  Created on: 14 Jun 2017
 *      Author: David
 */

#include "RestorePoint.h"

RestorePoint::RestorePoint()
{
	Init();
}

void RestorePoint::Init()
{
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		moveCoords[i] = 0.0;
	}

	feedRate = DefaultFeedRate * SecondsToMinutes;
	virtualExtruderPosition = 0.0;
	filePos = noFilePosition;
	proportionDone = 0.0;
	initialUserX = initialUserY = 0.0;
#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif
	toolNumber = -1;
}

// End
