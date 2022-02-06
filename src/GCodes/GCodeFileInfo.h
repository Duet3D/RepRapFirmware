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
		NamedEnum(Format, uint8_t, png, qoi);
		FilePosition offset;
		uint32_t size;
		uint16_t width, height;
		Format format;

		ThumbnailInfo() : width(0), format(Format::png) { }
		bool IsValid() const { return width != 0; }
		void Invalidate() { width = 0; }
	};

	GCodeFileInfo() noexcept { Init(); }
	void Init() noexcept;

	static constexpr unsigned int MaxThumbnails = 3;

	FilePosition fileSize;
	time_t lastModifiedTime;
	float layerHeight;
	unsigned int numLayers;
	float objectHeight;
	float filamentNeeded[MaxExtruders];
	uint32_t printTime;
	uint32_t simulatedTime;
	unsigned int numFilaments;
	bool isValid;
	bool incomplete;
	ThumbnailInfo thumbnails[MaxThumbnails];
	String<StringLength50> generatedBy;
};

#endif /* SRC_GCODES_GCODEFILEINFO_H_ */
