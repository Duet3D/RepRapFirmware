/*
 * FileInfoParser.h
 *
 *  Created on: 31 Mar 2018
 *      Author: David
 */

#ifndef SRC_STORAGE_FILEINFOPARSER_H_
#define SRC_STORAGE_FILEINFOPARSER_H_

#include "RepRapFirmware.h"
#include "RTOSIface.h"

const FilePosition GCODE_HEADER_SIZE = 20000uL;		// How many bytes to read from the header - I (DC) have a Kisslicer file with a layer height comment 14Kb from the start
const FilePosition GCODE_FOOTER_SIZE = 400000uL;	// How many bytes to read from the footer

#if SAM4E || SAM4S || SAME70
const size_t GCODE_READ_SIZE = 2048;				// How many bytes to read in one go in GetFileInfo() (should be a multiple of 512 for read efficiency)
#else
const size_t GCODE_READ_SIZE = 1024;				// How many bytes to read in one go in GetFileInfo() (should be a multiple of 512 for read efficiency)
#endif

const size_t GCODE_OVERLAP_SIZE = 100;				// Size of the overlapping buffer for searching (should be a multiple of 4)

const uint32_t MAX_FILEINFO_PROCESS_TIME = 200;		// Maximum time to spend polling for file info in each call
const uint32_t MaxFileParseInterval = 4000;			// Maximum interval between repeat requests to parse a file

// Struct to hold Gcode file information
struct GCodeFileInfo
{
	FilePosition fileSize;
	time_t lastModifiedTime;
	float layerHeight;
	float firstLayerHeight;
	float objectHeight;
	float filamentNeeded[MaxExtruders];
	unsigned int numFilaments;
	bool isValid;
	bool incomplete;
	String<50> generatedBy;

	void Init();
};

enum FileParseState
{
	notParsing,
	parsingHeader,
	seeking,
	parsingFooter
};

class FileInfoParser
{
public:
	FileInfoParser();

	// The following method needs to be called until it returns true - this may take a few runs
	bool GetFileInfo(const char *directory, const char *fileName, GCodeFileInfo& info, bool quitEarly);

private:

	// G-Code parser methods
	bool FindHeight(const char* buf, size_t len, float& height) const;
	bool FindFirstLayerHeight(const char* buf, size_t len, float& layerHeight) const;
	bool FindLayerHeight(const char* buf, size_t len, float& layerHeight) const;
	bool FindSlicerInfo(const char* buf, size_t len, const StringRef& generatedBy) const;
	unsigned int FindFilamentUsed(const char* buf, size_t len, float *filamentUsed, size_t maxFilaments) const;

	// We parse G-Code files in multiple stages. These variables hold the required information
	MutexHandle parserMutexHandle;
	MutexStorage parserMutexStorage;

	FileParseState parseState;
	String<MaxFilenameLength> filenameBeingParsed;
	FileStore *fileBeingParsed;
	FilePosition nextSeekPos;
	GCodeFileInfo parsedFileInfo;
	uint32_t lastFileParseTime;
	uint32_t accumulatedParseTime, accumulatedReadTime, accumulatedSeekTime;

	size_t fileOverlapLength;
	char fileOverlap[GCODE_OVERLAP_SIZE];
};

#endif /* SRC_STORAGE_FILEINFOPARSER_H_ */
