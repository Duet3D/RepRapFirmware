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
	firstLayerHeight = 0.0;
	objectHeight = 0.0;
	layerHeight = 0.0;
	printTime = simulatedTime = 0;
	numFilaments = 0;
	lastModifiedTime = 0;
	generatedBy.Clear();
	fileSize = 0;
	for (size_t extr = 0; extr < MaxExtruders; extr++)
	{
		filamentNeeded[extr] = 0.0;
	}
}

unsigned int GCodeFileInfo::GetNumLayers() const noexcept
{
	if (layerHeight <= 0.0) { return 0; }
	const float nl = (firstLayerHeight > 0.0)
						? (objectHeight - firstLayerHeight)/layerHeight + 1
							: objectHeight/layerHeight;
	return rintf(max<float>(nl, 0.0));
}

// End
