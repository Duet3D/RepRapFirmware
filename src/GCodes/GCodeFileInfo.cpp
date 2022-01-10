/*
 * GCodeFileInfo.cpp
 *
 *  Created on: 11 Jan 2020
 *      Author: David
 */

#include "GCodeFileInfo.h"

void GCodeFileInfo::Init() noexcept
{
	isValid = false;
	incomplete = true;
	objectHeight = 0.0;
	layerHeight = 0.0;
	printTime = simulatedTime = 0;
	numFilaments = 0;
	lastModifiedTime = 0;
	generatedBy.Clear();
	fileSize = 0;
	for (float& f : filamentNeeded)
	{
		f = 0.0;
	}
	for (ThumbnailInfo& th : thumbnails)
	{
		th.Invalidate();
	}
}

// End
