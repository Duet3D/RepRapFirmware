/*
 * RestorePoint.cpp
 *
 *  Created on: 14 Jun 2017
 *      Author: David
 */

#include "RestorePoint.h"

RestorePoint::RestorePoint() noexcept
{
	Init();
}

void RestorePoint::Init() noexcept
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
	toolNumber = -1;

	for (size_t i = 0; i < MaxSpindles; ++i)
	{
		spindleSpeeds[i] = 0.0;
	}

#if SUPPORT_LASER || SUPPORT_IOBITS
	laserPwmOrIoBits.Clear();
#endif
}

// End
