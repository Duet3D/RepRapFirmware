/*
 * FileInfoParser.h
 *
 *  Created on: 31 Mar 2018
 *      Author: David
 */

#ifndef SRC_STORAGE_FILEINFOPARSER_H_
#define SRC_STORAGE_FILEINFOPARSER_H_

#include "RepRapFirmware.h"
#include "RTOSIface/RTOSIface.h"

const FilePosition GCODE_HEADER_SIZE = 20000uL;		// How many bytes to read from the header - I (DC) have a Kisslicer file with a layer height comment 14Kb from the start
const FilePosition GCODE_FOOTER_SIZE = 400000uL;	// How many bytes to read from the footer

#if SAM4E || SAM4S || SAME70
const size_t GCODE_READ_SIZE = 2048;				// How many bytes to read in one go in GetFileInfo() (should be a multiple of 512 for read efficiency)
#else
const size_t GCODE_READ_SIZE = 1024;				// How many bytes to read in one go in GetFileInfo() (should be a multiple of 512 for read efficiency)
#endif

const size_t GcodeFooterPrintTimeSearchSize = 4096;	// How much of the end of the file we search for the estimated print time in

const size_t GCODE_OVERLAP_SIZE = 100;				// Size of the overlapping buffer for searching (must be a multiple of 4)

const uint32_t MAX_FILEINFO_PROCESS_TIME = 200;		// Maximum time to spend polling for file info in each call
const uint32_t MaxFileParseInterval = 4000;			// Maximum interval between repeat requests to parse a file

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
	bool GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly);

	static constexpr const char* SimulatedTimeString = "\n; Simulated print time";	// used by FileInfoParser and MassStorage

private:

	// G-Code parser methods
	bool FindHeight(const char* buf, size_t len);
	bool FindFirstLayerHeight(const char* buf, size_t len);
	bool FindLayerHeight(const char* buf, size_t len);
	bool FindSlicerInfo(const char* buf, size_t len);
	bool FindPrintTime(const char* buf, size_t len);
	bool FindSimulatedTime(const char* buf, size_t len);
	unsigned int FindFilamentUsed(const char* buf, size_t len);

	// We parse G-Code files in multiple stages. These variables hold the required information
	Mutex parserMutex;

	FileParseState parseState;
	String<MaxFilenameLength> filenameBeingParsed;
	FileStore *fileBeingParsed;
	FilePosition nextSeekPos;
	GCodeFileInfo parsedFileInfo;
	uint32_t lastFileParseTime;
	uint32_t accumulatedParseTime, accumulatedReadTime, accumulatedSeekTime;
	size_t fileOverlapLength;

	// We used to allocate the following buffer on the stack; but now that this is called by more than one task
	// it is more economical to allocate it permanently because that lets us use smaller stacks.
	// Alternatively, we could allocate a FileBuffer temporarily.
	uint32_t buf32[(GCODE_READ_SIZE + GCODE_OVERLAP_SIZE + 3)/4 + 1];	// buffer must be 32-bit aligned for HSMCI. We need the +1 so we can add a null terminator.
};

#endif /* SRC_STORAGE_FILEINFOPARSER_H_ */
