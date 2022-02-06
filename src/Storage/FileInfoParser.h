/*
 * FileInfoParser.h
 *
 *  Created on: 31 Mar 2018
 *      Author: David
 */

#ifndef SRC_STORAGE_FILEINFOPARSER_H_
#define SRC_STORAGE_FILEINFOPARSER_H_

#include <RepRapFirmware.h>
#include <GCodes/GCodeFileInfo.h>

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

#include <RTOSIface/RTOSIface.h>

const FilePosition GCODE_HEADER_SIZE = 20000uL;		// How many bytes to read from the header - I (DC) have a Kisslicer file with a layer height comment 14Kb from the start
const FilePosition GCODE_FOOTER_SIZE = 400000uL;	// How many bytes to read from the footer

#if SAME70 || SAME5x
const size_t GCODE_READ_SIZE = 2048;				// How many bytes to read in one go in GetFileInfo() (should be a multiple of 512 for read efficiency)
#else
const size_t GCODE_READ_SIZE = 1024;				// How many bytes to read in one go in GetFileInfo() (should be a multiple of 512 for read efficiency)
#endif

const size_t GcodeFooterPrintTimeSearchSize = 4096;	// How much of the end of the file we search for the estimated print time in

const size_t GCODE_OVERLAP_SIZE = 100;				// Size of the overlapping buffer for searching (must be a multiple of 4)

const uint32_t MAX_FILEINFO_PROCESS_TIME = 200;		// Maximum time to spend polling for file info in each call
const uint32_t MaxFileParseInterval = 4000;			// Maximum interval between repeat requests to parse a file

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
	FileInfoParser() noexcept;

	// The following method needs to be called repeatedly until it doesn't return GCodeResult::notFinished - this may take a few runs
	GCodeResult GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly) noexcept;

	static constexpr const char *_ecv_array SimulatedTimeString = "\n; Simulated print time";	// used by FileInfoParser and MassStorage

private:

	// G-Code parser methods
	bool FindHeight(const char *_ecv_array bufp, size_t len) noexcept;
	bool FindNumLayers(const char *_ecv_array bufp, size_t len) noexcept;
	bool FindLayerHeight(const char *_ecv_array bufp) noexcept;
	bool FindSlicerInfo(const char *_ecv_array bufp) noexcept;
	bool FindPrintTime(const char *_ecv_array bufp) noexcept;
	bool FindSimulatedTime(const char *_ecv_array bufp) noexcept;
	unsigned int FindFilamentUsed(const char *_ecv_array bufp) noexcept;
	void FindFilamentUsedEmbedded(const char *_ecv_array p, const char *_ecv_array s1, const char *_ecv_array s2, unsigned int &filamentsFound) noexcept;
	bool FindThumbnails(const char *_ecv_array bufp, FilePosition bufferStartFilePosition) noexcept;

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
	alignas(4) char buf[GCODE_READ_SIZE + GCODE_OVERLAP_SIZE + 1];		// buffer must be 32-bit aligned for HSMCI. We need the +1 so we can add a null terminator.
};

#endif

#endif /* SRC_STORAGE_FILEINFOPARSER_H_ */
