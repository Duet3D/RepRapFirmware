/*
 * FileInfoParser.h
 *
 *  Created on: 31 Mar 2018
 *      Author: David
 */

#ifndef SRC_STORAGE_FILEINFOPARSER_H_
#define SRC_STORAGE_FILEINFOPARSER_H_

#include <RepRapFirmware.h>

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

#include <GCodes/GCodeFileInfo.h>
#include <RTOSIface/RTOSIface.h>

#if SAME70 || SAME5x
const size_t GCodeReadSize = 2048;							// How many bytes to read in one go in GetFileInfo(), must be a power to 2, should be >= 512 for file read efficiency
#else
const size_t GCodeReadSize = 1024;							// How many bytes to read in one go in GetFileInfo(), must be a power of 2, should be >= 512 for file read efficiency
#endif

static_assert ((GCodeReadSize & (GCodeReadSize - 1)) == 0);	// Check that it's a power of 2

const FilePosition GCodeFooterSize = 400 * 1024;			// How many bytes to read from the footer, must be a multiple of GCodeReadSize
static_assert(GCodeFooterSize % GCodeReadSize == 0);		// check it's a multiple of GCodeReadSize

const size_t GCodeOverlapSize = 100;						// Size of the overlapping buffer for searching, must be a multiple of 4
static_assert(GCodeOverlapSize % 4 == 0);

const uint32_t MaxFileinfoProcessTime = 200;				// Maximum time (ms) to spend polling for file info in each call
const uint32_t MaxFileParseInterval = 4000;					// Maximum interval (ms) between repeat requests to parse a file before we assume the request has been abandoned

enum class FileParseState : uint8_t
{
	notParsing,
	parsingHeader,
	seeking,
	parsingFooter
};

class GlobalVariables;

class FileInfoParser
{
public:
	FileInfoParser() noexcept;

	// The following method needs to be called repeatedly until it doesn't return GCodeResult::notFinished - this may take a few runs
	GCodeResult GetFileInfo(const char *_ecv_array filePath, GCodeFileInfo& p_info, bool quitEarly, GlobalVariables *_ecv_null customVariables) noexcept;

	static constexpr const char *_ecv_array SimulatedTimeString = "\n; Simulated print time";	// used by FileInfoParser and MassStorage

	static constexpr bool TableIsCorrectlyOrdered() noexcept;

private:
	// G-Code parser methods
	bool ReadAndProcessFileChunk(bool isParsingHeader, bool& reachedEnd) noexcept
		pre(fileBeingParsed != nullptr);
	bool FindEndComments() noexcept
		pre(fileBeingParsed != nullptr);
	const char *_ecv_array ScanBuffer(const char *_ecv_array pStart, const char *_ecv_array pEnd, bool isParsingHeader, bool& stopped) noexcept
		pre(pStart.base == pEnd.base; pStart < pEnd; atLineStart)
		post(_ecv_result.base == pStart.base; _ecv_result <= pEnd);

	// Parse table entry methods
	void ProcessGeneratedBy(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;
	void ProcessLayerHeight(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;
	void ProcessObjectHeight(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;
	void ProcessNumLayers(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;
	void ProcessJobTime(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;
	void ProcessSimulatedTime(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;
	void ProcessThumbnail(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;
	void ProcessFilamentUsed(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;
	void ProcessCustomInfo(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept;

	void ProcessFilamentUsedEmbedded(const char *_ecv_array p, const char *_ecv_array s2) noexcept
		pre(parsedFileInfo.numFilaments < MaxFilaments);

	struct ParseTableEntry
	{
		const char *_ecv_array key;												// the keyword at the start of the comment that we look for
		void (FileInfoParser::*func)(const char *_ecv_array, const char *_ecv_array, const char *_ecv_array, int) noexcept;		// the function used to process the rest of the comment that follows the keyword
		int param;																// a parameter we pass to that function
	};

	static const ParseTableEntry parseTable[];

	// We parse G-Code files in multiple stages. These variables hold the required information
	Mutex parserMutex;

	GlobalVariables *_ecv_null vars;
	String<MaxFilenameLength> filenameBeingParsed;
	FileStore *_ecv_null fileBeingParsed;
	GCodeFileInfo parsedFileInfo;
	uint32_t lastFileParseTime;
	size_t scanStartOffset;
	FilePosition bufferStartFilePosition;
	unsigned int numThumbnailsStored;
	FileParseState parseState;
	bool atLineStart;
	bool foundHeightComment;

	// Stats for performance monitoring
	uint32_t accumulatedParseTime, accumulatedReadTime, accumulatedSeekTime, prepTime;
	FilePosition trailerBytesProcessed;


	// We used to allocate the following buffer on the stack; but now that this is called by more than one task
	// it is more economical to allocate it permanently because that lets us use smaller stacks.
	// Alternatively, we could allocate a FileBuffer temporarily.
	alignas(4) char buf[GCodeReadSize + GCodeOverlapSize + 1];		// buffer must be 32-bit aligned for HSMCI. We need the +1 so we can add a null terminator.
};

#endif

#endif /* SRC_STORAGE_FILEINFOPARSER_H_ */
