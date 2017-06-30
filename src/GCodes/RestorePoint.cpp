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
	for (size_t i = 0; i < DRIVES; ++i)
	{
		moveCoords[i] = 0.0;
	}
	feedRate = DefaultFeedrate * SecondsToMinutes;
	virtualExtruderPosition = 0.0;
	filePos = noFilePosition;
#if SUPPORT_IOBITS
	ioBits = 0;
#endif
}

// End
