/*
 * GCodeFileInfo.h
 *
 *  Created on: 1 Apr 2019
 *      Author: Christian
 */

#ifndef SRC_GCODES_GCODEFILEINFO_H_
#define SRC_GCODES_GCODEFILEINFO_H_

#include "RepRapFirmware.h"

// Struct to hold Gcode file information
struct GCodeFileInfo
{
	GCodeFileInfo() { Init(); }
	void Init();

	FilePosition fileSize;
	time_t lastModifiedTime;
	float layerHeight;
	float firstLayerHeight;
	float objectHeight;
	float filamentNeeded[MaxExtruders];
	uint32_t printTime;
	uint32_t simulatedTime;
	unsigned int numFilaments;
	bool isValid;
	bool incomplete;
	String<50> generatedBy;
};

#endif /* SRC_GCODES_GCODEFILEINFO_H_ */
