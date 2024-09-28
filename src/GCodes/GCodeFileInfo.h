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
	struct ThumbnailInfo
	{
		NamedEnum(ImageFormat, uint8_t, png, qoi, jpeg);
		FilePosition offset;
		uint32_t size;
		uint16_t width, height;
		ImageFormat format;

		ThumbnailInfo() : width(0), format(ImageFormat::png) { }
		bool IsValid() const { return width != 0; }
		void Invalidate() { width = 0; }
	};

	GCodeFileInfo() noexcept { Init(); }
	void Init() noexcept;

	FilePosition fileSize;
	FilePosition headerSize;
	time_t lastModifiedTime;
	float layerHeight;
	unsigned int numLayers;
	float objectHeight;
	float filamentNeeded[MaxFilaments];
	uint32_t printTime;
	uint32_t simulatedTime;
	unsigned int numFilaments;
	bool isValid;
	bool incomplete;
	ThumbnailInfo thumbnails[MaxThumbnails];
	String<StringLength50> generatedBy;
};

#endif /* SRC_GCODES_GCODEFILEINFO_H_ */
