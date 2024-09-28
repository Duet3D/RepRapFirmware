/*
 * FileInfoParser.cpp
 *
 *  Created on: 31 Mar 2018
 *      Author: David
 */

#include "FileInfoParser.h"
#include <Platform/OutputMemory.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <PrintMonitor/PrintMonitor.h>
#include <GCodes/GCodes.h>

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

const FileInfoParser::ParseTableEntry FileInfoParser::parseTable[] =
{
	// Note: if a key string in this table is a leading or embedded substring of another, the longer one must come first
	// Slicer identification keywords
	{ 	"KISSlicer",								&FileInfoParser::ProcessGeneratedBy,		2 },		// KISSlicer
	{	"Sliced at",								&FileInfoParser::ProcessGeneratedBy,		1 },		// Cura (old)
	{	"Fusion version",							&FileInfoParser::ProcessGeneratedBy,		2 },		// Fusion 360
	{	"generated by",								&FileInfoParser::ProcessGeneratedBy,		0 },		// slic3r and PrusaSlicer
	{	"G-Code generated by",						&FileInfoParser::ProcessGeneratedBy,		0 },		// S3D
	{	"Sliced by",								&FileInfoParser::ProcessGeneratedBy,		0 },		// ideaMaker
	{	"Generated with",							&FileInfoParser::ProcessGeneratedBy,		0 },		// Cura (new)
	{	"Generated by",								&FileInfoParser::ProcessGeneratedBy,		0 },		// kiri:moto
	{	"GENERATOR.NAME",							&FileInfoParser::ProcessGeneratedBy,		0 },		// Pathio (the version is separate, we don't include that)
	{	"Generated with",							&FileInfoParser::ProcessGeneratedBy,		0 },		// Matter Control

	// Layer height keywords
	{ 	"layer_height",								&FileInfoParser::ProcessLayerHeight,		0 },		// slic3r, PrusaSlicer, OrcaSlicer	"; layer_height = 0.2"
	{ 	"Layer height",								&FileInfoParser::ProcessLayerHeight,		0 },		// Cura
	{ 	"layerHeight",								&FileInfoParser::ProcessLayerHeight,		0 },		// S3D								";   layerHeight,0.2"
	{ 	"layer_thickness_mm",						&FileInfoParser::ProcessLayerHeight,		0 },		// Kisslicer
	{ 	"layerThickness",							&FileInfoParser::ProcessLayerHeight,		0 },		// Matter Control
	{ 	"sliceHeight",								&FileInfoParser::ProcessLayerHeight,		0 },		// kiri:moto

	// Number-of-layers keywords
	{ 	"num_layers",								&FileInfoParser::ProcessNumLayers,			0 },
	{ 	"NUM_LAYERS",								&FileInfoParser::ProcessNumLayers,			0 },
	{ 	"total layer number",						&FileInfoParser::ProcessNumLayers,			0 },		// OrcaSlicer						"; total layer number: 100"

	// Estimated job time keywords
	{ 	"estimated printing time (normal mode)",	&FileInfoParser::ProcessJobTime,			0 },		// PrusaSlicer later versions		"; estimated printing time (normal mode) = 2d 1h 5m 24s"
	{ 	"estimated printing time",					&FileInfoParser::ProcessJobTime,			0 },		// PrusaSlicer older versions		"; estimated printing time = 1h 5m 24s"
	{ 	"TIME",										&FileInfoParser::ProcessJobTime,			0 },		// Cura								";TIME:38846"
																											// Kiri Moto						";TIME 3720.97"
																											// Kiri Moto also					"; --- print time: 3721s ---"
	{ 	"Build time",								&FileInfoParser::ProcessJobTime,			0 },		// S3D								";   Build time: 0 hours 42 minutes"
																											// also REALvision					"; Build time: 2:11:47"
	{ 	"Estimated Build Time",						&FileInfoParser::ProcessJobTime,			0 },		// KISSlicer						"; Estimated Build Time:   332.83 minutes"
	{ 	"Calculated-during-export Build Time",		&FileInfoParser::ProcessJobTime,			0 },		// KISSSlicer 2 alpha				"; Calculated-during-export Build Time: 130.62 minutes"
	{ 	"Print Time",								&FileInfoParser::ProcessJobTime,			0 },		// Ideamaker
	{ 	"PRINT.TIME",								&FileInfoParser::ProcessJobTime,			0 },		// Patio
	{ 	"Print time",								&FileInfoParser::ProcessJobTime,			0 },		// Fusion 360
	{ 	"total print time (s)",						&FileInfoParser::ProcessJobTime,			0 },		// Matter Control

	// Simulated job time keyword
	{	"Simulated print time",						&FileInfoParser::ProcessSimulatedTime,		0 },		// appended to the file by RRF

	// Thumbnail keywords
	{	"thumbnail_JPG begin",						&FileInfoParser::ProcessThumbnail,			2 },		// thumbnail in JPEG format
	{	"thumbnail_QOI begin",						&FileInfoParser::ProcessThumbnail,			1 },		// thumbnail in QOI format
	{	"thumbnail begin",							&FileInfoParser::ProcessThumbnail,			0 },		// thumbnail in PNG format

	// Filament usage key phrases
	{	"filament used [mm]",						&FileInfoParser::ProcessFilamentUsed,		0 },		// Prusa slicer, OrcaSlicer			"; filament used [mm] = 1965.97"
	{	"filament used",							&FileInfoParser::ProcessFilamentUsed,		0 },		// Kiri Moto						"; --- filament used: 1657.31 mm ---"
	{	"Filament used",							&FileInfoParser::ProcessFilamentUsed,		0 },		// Cura								";Filament used: 0m"
	{	"Filament length",							&FileInfoParser::ProcessFilamentUsed,		1 },		// S3D v4							";   Filament length: 13572.2 mm (13.57 m)"
	{	"Material Length",							&FileInfoParser::ProcessFilamentUsed,		1 },		// S3D v5 ??? CHECK CASE OF FIRST LETTER
	{	"Ext",										&FileInfoParser::ProcessFilamentUsed,		2 },		// Kisslicer newer versions
	{	"Estimated Build Volume",					&FileInfoParser::ProcessFilamentUsed,		3 },		// Kisslicer older versions
	{	"Material#",								&FileInfoParser::ProcessFilamentUsed,		4 },		// Ideamaker						";Material#1 Used: 868.0"
	{	"Extruder",									&FileInfoParser::ProcessFilamentUsed,		5 },		// Fusion 360 						";Extruder 1 material used: 1811mm"

	// Object height keywords
	{	"max_z_height",								&FileInfoParser::ProcessObjectHeight,		0 },		// OrcaSlicer						"; max_z_height: 20.00"
};


FileInfoParser::FileInfoParser() noexcept
	: parseState(notParsing), fileBeingParsed(nullptr), accumulatedParseTime(0), accumulatedReadTime(0), accumulatedSeekTime(0)
{
	parsedFileInfo.Init();
	parserMutex.Create("FileInfoParser");
}

// This following method needs to be called repeatedly until it returns true - this may take a few runs
GCodeResult FileInfoParser::GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly) noexcept
{
	MutexLocker lock(parserMutex, MAX_FILEINFO_PROCESS_TIME);
	if (!lock.IsAcquired())
	{
		return GCodeResult::notFinished;
	}

	if (parseState != notParsing && !StringEqualsIgnoreCase(filePath, filenameBeingParsed.c_str()))
	{
		// We are already parsing a different file
		if (millis() - lastFileParseTime < MaxFileParseInterval)
		{
			return GCodeResult::notFinished;				// try again later
		}

		// Time this client out because it has probably disconnected
		fileBeingParsed->Close();
		parseState = notParsing;
	}

	if (parseState == notParsing)
	{
		// See if we can access the file
		// Webserver may call rr_fileinfo for a directory, check this case here
		if (MassStorage::DirectoryExists(filePath))
		{
			info.isValid = false;
			return GCodeResult::ok;
		}

		fileBeingParsed = MassStorage::OpenFile(filePath, OpenMode::read, 0);
		if (fileBeingParsed == nullptr)
		{
			// Something went wrong - we cannot open it
			info.isValid = false;
			return GCodeResult::error;
		}

		// File has been opened, let's start now
		filenameBeingParsed.copy(filePath);

		// Set up the info struct
		parsedFileInfo.Init();
		parsedFileInfo.fileSize = fileBeingParsed->Length();
#if HAS_MASS_STORAGE
		parsedFileInfo.lastModifiedTime = MassStorage::GetLastModifiedTime(filePath);
#endif
		parsedFileInfo.isValid = true;

		// Record some debug values here
		if (reprap.Debug(Module::PrintMonitor))
		{
			accumulatedReadTime = accumulatedParseTime = 0;
			reprap.GetPlatform().MessageF(UsbMessage, "-- Parsing file %s --\n", filePath);
		}

		// If the file is empty or not a G-Code file, we don't need to parse anything
		constexpr const char *GcodeFileExtensions[] = { ".gcode", ".g", ".gco", ".gc", ".nc" };
		bool isGcodeFile = false;
		for (const char *ext : GcodeFileExtensions)
		{
			if (StringEndsWithIgnoreCase(filePath, ext))
			{
				isGcodeFile = true;
				break;
			}
		}

		if (fileBeingParsed->Length() == 0 || !isGcodeFile)
		{
			fileBeingParsed->Close();
			parsedFileInfo.incomplete = false;
			info = parsedFileInfo;
			return GCodeResult::ok;
		}
		parseState = parsingHeader;
		scanStartOffset = GCODE_OVERLAP_SIZE;
		numThumbnailsStored = 0;
		numFilamentsFound = 0;
		atLineStart = true;
	}

	// Getting file information take a few runs. Speed it up when we are not printing by calling it several times.
	const uint32_t loopStartTime = millis();
	do
	{
#if 0
		size_t sizeToRead, sizeToScan;										// number of bytes we want to read and scan in this go
#endif

		switch (parseState)
		{
		case parsingHeader:
			{
				bool reachedEnd;
				if (!ReadAndProcessFileChunk(true, reachedEnd))
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Failed to read header of G-Code file \"%s\"\n", filePath);
					parseState = notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return GCodeResult::warning;
				}

				if (reachedEnd) { parseState = seeking; }
			}

#if 0
			{
				bool headerInfoComplete = true;

				// Read a chunk from the header. On the first run only process GCODE_READ_SIZE bytes, but use overlap next times.
				sizeToRead = (size_t)min<FilePosition>(fileBeingParsed->Length() - fileBeingParsed->Position(), GCODE_READ_SIZE);
				if (fileOverlapLength > 0)
				{
					sizeToScan = sizeToRead + fileOverlapLength;
				}
				else
				{
					sizeToScan = sizeToRead;
				}

				const FilePosition bufferStartFileOffset = fileBeingParsed->Position() - fileOverlapLength;
				const int nbytes = fileBeingParsed->Read(&buf[fileOverlapLength], sizeToRead);
				if (nbytes != (int)sizeToRead)
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Failed to read header of G-Code file \"%s\"\n", filePath);
					parseState = notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return GCodeResult::warning;
				}
				buf[sizeToScan] = 0;

				// Record performance data
				uint32_t now = millis();
				accumulatedReadTime += now - startTime;
				startTime = now;

				// Search for filament usage (Cura puts it at the beginning of a G-code file)
				if (parsedFileInfo.numFilaments == 0)
				{
					parsedFileInfo.numFilaments = FindFilamentUsed(buf);
					headerInfoComplete &= (parsedFileInfo.numFilaments != 0);
				}

				// Look for layer height
				if (parsedFileInfo.layerHeight == 0.0)
				{
					headerInfoComplete &= FindLayerHeight(buf);
				}

				// Look for slicer program
				if (parsedFileInfo.generatedBy.IsEmpty())
				{
					headerInfoComplete &= FindSlicerInfo(buf);
				}

				// Look for print time
				if (parsedFileInfo.printTime == 0)
				{
					headerInfoComplete &= FindPrintTime(buf);
				}

				// Look for thumbnail images
				headerInfoComplete &= FindThumbnails(buf, bufferStartFileOffset);

				// Keep track of the time stats
				accumulatedParseTime += millis() - startTime;

				// Can we proceed to the footer? Don't scan more than the first 4KB of the file
				FilePosition pos = fileBeingParsed->Position();
				if (headerInfoComplete || pos >= GCODE_HEADER_SIZE || pos == fileBeingParsed->Length())
				{
					// Yes - see if we need to output some debug info
					if (reprap.Debug(Module::PrintMonitor))
					{
						reprap.GetPlatform().MessageF(UsbMessage, "Header complete, processed %lu bytes, read time %.3fs, parse time %.3fs\n",
											fileBeingParsed->Position(), (double)((float)accumulatedReadTime/1000.0), (double)((float)accumulatedParseTime/1000.0));
					}

					// Go to the last chunk and proceed from there on
					const FilePosition seekFromEnd = ((fileBeingParsed->Length() - 1) % GCODE_READ_SIZE) + 1;
					nextSeekPos = fileBeingParsed->Length() - seekFromEnd;
					accumulatedSeekTime = accumulatedReadTime = accumulatedParseTime = 0;
					fileOverlapLength = 0;
					parseState = seeking;
				}
				else
				{
					// No - copy the last chunk of the buffer for overlapping search
					fileOverlapLength = min<size_t>(sizeToRead, GCODE_OVERLAP_SIZE);
					memcpy(buf, &buf[sizeToRead - fileOverlapLength], fileOverlapLength);
				}
			}
#endif
			break;

		case seeking:
			if (FindEndComments())
			{
				parseState = parsingFooter;
			}
			else
			{
				reprap.GetPlatform().MessageF(WarningMessage, "Could not find footer comments in file \"%s\"\n", filePath);
				parseState = notParsing;
				fileBeingParsed->Close();
				info = parsedFileInfo;
				return GCodeResult::warning;
			}
#if 0
			// Seeking into a large file can take a long time using the FAT file system, so do it in stages
			{
#if HAS_MASS_STORAGE
				FilePosition currentPos = fileBeingParsed->Position();
				const uint32_t clsize = fileBeingParsed->ClusterSize();
				if (currentPos/clsize > nextSeekPos/clsize)
				{
					// Seeking backwards over a cluster boundary, so in practice the seek will start from the start of the file
					currentPos = 0;
				}
				// Seek at most 512 clusters at a time
				const FilePosition maxSeekDistance = 512 * (FilePosition)clsize;
				const bool doFullSeek = (nextSeekPos <= currentPos + maxSeekDistance);
				const FilePosition thisSeekPos = (doFullSeek) ? nextSeekPos : currentPos + maxSeekDistance;
#elif HAS_EMBEDDED_FILES
				const bool doFullSeek = true;
				const FilePosition thisSeekPos = nextSeekPos;
#endif
				const uint32_t startTime = millis();
				if (!fileBeingParsed->Seek(thisSeekPos))
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Could not seek from end of file \"%s\"\n", filePath);
					parseState = notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return GCodeResult::warning;
				}
				accumulatedSeekTime += millis() - startTime;
				if (doFullSeek)
				{
					parseState = parsingFooter;
				}
			}
#endif
			break;

		case parsingFooter:
			{
				bool reachedEnd;
				if (!ReadAndProcessFileChunk(false, reachedEnd))
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Failed to read footer from G-Code file \"%s\"\n", filePath);
					parseState = notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return GCodeResult::warning;
				}

				if (reachedEnd)
				{
					parsedFileInfo.incomplete = false;
					info = parsedFileInfo;
					return GCodeResult::ok;
				}
			}
#if 0
			{
				// Processing the footer. See how many bytes we need to read and if we can reuse the overlap
				sizeToRead = (size_t)min<FilePosition>(fileBeingParsed->Length() - nextSeekPos, GCODE_READ_SIZE);
				if (fileOverlapLength > 0)
				{
					memcpy(&buf[sizeToRead], buf, fileOverlapLength);
					sizeToScan = sizeToRead + fileOverlapLength;
				}
				else
				{
					sizeToScan = sizeToRead;
				}

				// Read another chunk from the footer
				uint32_t startTime = millis();
				int nbytes = fileBeingParsed->Read(buf, sizeToRead);
				if (nbytes != (int)sizeToRead)
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Failed to read footer from G-Code file \"%s\"\n", filePath);
					parseState = notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return GCodeResult::warning;
				}
				buf[sizeToScan] = 0;

				// Record performance data
				uint32_t now = millis();
				accumulatedReadTime += now - startTime;
				startTime = now;

				bool footerInfoComplete = true;

				// Search for filament used
				if (parsedFileInfo.numFilaments == 0)
				{
					parsedFileInfo.numFilaments = FindFilamentUsed(buf);
					if (parsedFileInfo.numFilaments == 0)
					{
						footerInfoComplete = false;
					}
				}

				// Search for layer height
				if (parsedFileInfo.layerHeight == 0.0)
				{
					if (!FindLayerHeight(buf))
					{
						footerInfoComplete = false;
					}
				}

				// Search for object height
				if (parsedFileInfo.objectHeight == 0.0)
				{
					if (!FindHeight(buf, sizeToScan))
					{
						footerInfoComplete = false;
					}
				}

				// Search for number of layers
				if (parsedFileInfo.numLayers == 0)
				{
					// Number of layers should come before the object height
					(void)FindNumLayers(buf, sizeToScan);
				}

				// Look for print time
				if (parsedFileInfo.printTime == 0)
				{
					if (!FindPrintTime(buf) && fileBeingParsed->Length() - nextSeekPos <= GcodeFooterPrintTimeSearchSize)
					{
						footerInfoComplete = false;
					}
				}

				// Look for simulated print time. It will always be right at the end of the file, so don't look too far back
				if (parsedFileInfo.simulatedTime == 0)
				{
					if (!FindSimulatedTime(buf) && fileBeingParsed->Length() - nextSeekPos <= GcodeFooterPrintTimeSearchSize)
					{
						footerInfoComplete = false;
					}
				}

				// Keep track of the time stats
				accumulatedParseTime += millis() - startTime;

				// If we've collected all details, scanned the last 192K of the file or if we cannot go any further, stop here.
				if (footerInfoComplete || nextSeekPos == 0 || fileBeingParsed->Length() - nextSeekPos >= GCODE_FOOTER_SIZE)
				{
					if (reprap.Debug(Module::PrintMonitor))
					{
						reprap.GetPlatform().MessageF(UsbMessage, "Footer complete, processed %lu bytes, read time %.3fs, parse time %.3fs, seek time %.3fs\n",
											fileBeingParsed->Length() - fileBeingParsed->Position() + GCODE_READ_SIZE,
											(double)((float)accumulatedReadTime/1000.0), (double)((float)accumulatedParseTime/1000.0), (double)((float)accumulatedSeekTime/1000.0));
					}
					parseState = notParsing;
					fileBeingParsed->Close();
					if (parsedFileInfo.numLayers == 0 && parsedFileInfo.layerHeight > 0.0 && parsedFileInfo.objectHeight > 0.0)
					{
						parsedFileInfo.numLayers = lrintf(parsedFileInfo.objectHeight / parsedFileInfo.layerHeight);
					}
					parsedFileInfo.incomplete = false;
					info = parsedFileInfo;
					return GCodeResult::ok;
				}

				// Else go back further
				fileOverlapLength = (size_t)min<FilePosition>(sizeToScan, GCODE_OVERLAP_SIZE);
				nextSeekPos = (nextSeekPos <= GCODE_READ_SIZE) ? 0 : nextSeekPos - GCODE_READ_SIZE;
				parseState = seeking;
			}
#endif
			break;

		default:	// should not get here
			parsedFileInfo.incomplete = false;
			fileBeingParsed->Close();
			info = parsedFileInfo;
			parseState = notParsing;
			return GCodeResult::ok;
		}
		lastFileParseTime = millis();
	} while (!reprap.GetPrintMonitor().IsPrinting() && lastFileParseTime - loopStartTime < MAX_FILEINFO_PROCESS_TIME);

	if (quitEarly)
	{
		info = parsedFileInfo;				// note that the 'incomplete' flag is still set
		fileBeingParsed->Close();
		parseState = notParsing;
		return GCodeResult::ok;
	}
	return GCodeResult::notFinished;
}

// Read and process a chunk of the file.
// On entry, fileOverlapLength is the offset into the start of the buffer that we should read into, and if it is nonzero then there was an incomplete comment line already at the start of the buffer.
// The file position is correct for reading the next chunk of the file.
// If successful set reachedEnd true if we ran out of data in the file; otherwise set reachedEnd false and leave the buffer, fileOverlaplength and the file seek position ready for the next call to this function.
// Return true if success, false if there was a file read error.
bool FileInfoParser::ReadAndProcessFileChunk(bool parsingHeader, bool& reachedEnd) noexcept
{
	// Read a chunk of the file into the buffer after the data we have already.
	// For efficiency, read complete 512byte sectors on 512byte sector boundaries, and read them into 32-bit aligned memory - that allows the SD card driver to DMA directly into the buffer.
	bufferStartFilePosition = fileBeingParsed->Position() - GCODE_OVERLAP_SIZE;					// we need to keep this up to date so that we can record the file offsets of thumbnails
	const FilePosition sizeLeft = fileBeingParsed->Length() - fileBeingParsed->Position();
	const size_t sizeToRead = (size_t)min<FilePosition>(sizeLeft, GCODE_READ_SIZE);
	const int nbytes = fileBeingParsed->Read(buf + GCODE_OVERLAP_SIZE, sizeToRead);
	if (nbytes != (int)sizeToRead)
	{
		return false;
	}

	const char *_ecv_array bufp = buf + scanStartOffset;
	char *_ecv_array bufLim = buf + GCODE_OVERLAP_SIZE + (unsigned int)nbytes;
	reachedEnd = (sizeLeft <= GCODE_READ_SIZE);
	if (reachedEnd)
	{
		// This is the last read of the file, so append a '\n' terminator so that ScanBuffer can process the last line
		*bufLim = '\n';
		++bufLim;
	}

	if (!atLineStart)
	{
		// The last scan encountered a very long line. Skip the rest of it before resuming parsing.
		while (bufp < bufLim)
		{
			const char c = *bufp++;
			if (c == '\n' || c == '\r')
			{
				atLineStart = true;
				break;
			}
		}
	}
	const char *_ecv_array const pEnd = (bufp == bufLim) ? bufp : ScanBuffer(bufp, bufLim, parsingHeader, reachedEnd);

	if (!reachedEnd && pEnd < bufLim)
	{
		scanStartOffset = GCODE_READ_SIZE - (bufLim - pEnd);
		memcpy(buf + scanStartOffset, pEnd, bufLim - pEnd);
	}
	else
	{
		scanStartOffset = GCODE_OVERLAP_SIZE;
	}
	if (reachedEnd && parsingHeader)
	{
		parsedFileInfo.headerSize = bufferStartFilePosition + (pEnd - buf);
	}
	return true;
}

// Scan the buffer for data we are interested in.
// On entry, pStart is at the start of a line of the file.
// Return a pointer to the incomplete comment line at the end, if there is one, or pEnd if there isn't.
// If stopOnGCode is set then if we reach a line of GCode, set 'stopped'; otherwise leave 'stopped' alone.
const char *_ecv_array FileInfoParser::ScanBuffer(const char *_ecv_array pStart, const char *_ecv_array pEnd, bool stopOnGCode, bool& stopped) noexcept
{
	while (pStart < pEnd)
	{
		char c = *pStart++;
		switch (c)
		{
		case ';':
			// Found a whole-line comment
			{
				const char *_ecv_array commentStart = pStart - 1;
				while (pStart < pEnd && ((c = *pStart) == ' ' || c == '-'))
				{
					++pStart;					// skip spaces after the leading semicolon
				}
				if (pStart == pEnd)
				{
					return commentStart;
				}
				if (isAlpha(c))					// all keywords we are interested in start with a letter
				{
					// Find the length of the rest of the line and check that it ends within the buffer
					const char *_ecv_array kStart = pStart;
					while (pStart < pEnd && *pStart != '\r' && *pStart != '\n') { ++pStart; }
					if (pStart == pEnd)
					{
						// We didn't find a line terminator
						if (commentStart >= buf + GCODE_READ_SIZE)
						{
							// This comment starts within the last GCODE_OVERLAP_SIZE of the buffer, so we can safely leave processing it until the next buffer full
							return commentStart;
						}

						// This comment line is very long. Ignore it and return, flagging that we are not at a line start.
						atLineStart = false;
						return pEnd;
					}

					// pStart now points to the line terminator and kStart to the possible start of a key phrase.
					// There is definitely a line terminator, and as line terminators do not occur in key phrases, it is safe to call strcmp
					for (const ParseTableEntry& pte : parseTable)
					{
						if (StringStartsWith(kStart, pte.key))
						{
							// Found the key phrase. Check for a separator after it unless the key phrase ends with '#'.
							const char *_ecv_array argStart = kStart + strlen(pte.key);
							if (*(argStart - 1) != '#')
							{
								char c2 = *argStart;
								if (c2 != ' ' && c2 != '\t' && c2 != ':' && c2 != '=' && c2 != ',')
								{
									break;
								}

								// Skip further separators
								do
								{
									++argStart;
								} while ((c2 = *argStart) == ' ' || c2 == '\t' || c2 == ':' || c2 == '=');
							}
							(this->*pte.FileInfoParser::ParseTableEntry::func)(kStart, argStart, pte.param);
							break;
						}
					}
				}
				++pStart;					// skip the line end
			}
			break;

		case '\r':
		case '\n':
			break;							// skip the blank line or 2nd line terminator

		case 'N':
		case 'G':
		case 'M':
		case 'T':
			if (stopOnGCode)
			{
				stopped = true;
				return pStart;
			}
			// no break
		default:
			// Skip the rest of the line
			while (pStart < pEnd && ((c = *pStart) != '\n' && c != '\r'))
			{
				++pStart;
			}
			if (pStart == pEnd)
			{
				atLineStart = false;
				return pEnd;
			}
			++pStart;						// skip the line terminator
			break;
		}
	}
	return pEnd;
}

// Find the starting position of the file end comments, get the object height, set up the buffer ready to parse them.
// Return true if successful.
bool FileInfoParser::FindEndComments() noexcept
{
	// Temporary code until we find something better
	const FilePosition roundedDownLength = fileBeingParsed->Length() & ~(GCODE_READ_SIZE - 1);		// round down to a multiple of the read size
	FilePosition pos;
	if (roundedDownLength > parsedFileInfo.headerSize + GCODE_FOOTER_SIZE)
	{
		// Usual case when the file is long
		pos = roundedDownLength - GCODE_FOOTER_SIZE;
		scanStartOffset = GCODE_OVERLAP_SIZE;
	}
	else
	{
		// Start scanning from just after the header
		pos = parsedFileInfo.headerSize & ~(GCODE_READ_SIZE - 1);
		scanStartOffset = (parsedFileInfo.headerSize & (GCODE_READ_SIZE - 1)) + GCODE_OVERLAP_SIZE;
	}
	bufferStartFilePosition = pos - GCODE_OVERLAP_SIZE;
	atLineStart = false;
	return fileBeingParsed->Seek(pos);
}

// Parse table entry methods

// Process the slicer name and (if present) version
void FileInfoParser::ProcessGeneratedBy(const char *_ecv_array k, const char *_ecv_array p, int param) noexcept
{
	const char *_ecv_array introString = "";
	switch (param)
	{
	case 1:				// Cura (old)
		introString = "Cura at ";
		break;

	case 2:				// Kisslicer and Fusion 360 - the keyword is the generator name
		p = k;
		break;

	default:
		break;
	}

	parsedFileInfo.generatedBy.copy(introString);
	while (*p >= ' ')
	{
		parsedFileInfo.generatedBy.cat(*p++);
	}
}

// Process the layer height
void FileInfoParser::ProcessLayerHeight(const char *_ecv_array k, const char *_ecv_array p, int param) noexcept
{
	const char *tailPtr;
	const float val = SafeStrtof(p, &tailPtr);
	if (tailPtr != p && !std::isnan(val) && !std::isinf(val))	// if we found and converted a number
	{
		parsedFileInfo.layerHeight = val;
	}
}

void FileInfoParser::ProcessObjectHeight(const char *_ecv_array k, const char *_ecv_array p, int param) noexcept
{
	const char *tailPtr;
	const float val = SafeStrtof(p, &tailPtr);
	if (tailPtr != p && !std::isnan(val) && !std::isinf(val))	// if we found and converted a number
	{
		parsedFileInfo.objectHeight = val;
	}
}

// Process the number of layers
void FileInfoParser::ProcessNumLayers(const char *_ecv_array k, const char *_ecv_array p, int param) noexcept
{
	const unsigned int val = StrToU32(p);
	if (val > 0)
	{
		parsedFileInfo.numLayers = val;
	}
}

// Process the estimated job time
void FileInfoParser::ProcessJobTime(const char *_ecv_array k, const char *_ecv_array p, int param) noexcept
{
	const char *_ecv_array const q = p;
	float days = 0.0, hours = 0.0, minutes = 0.0;
	float secs = SafeStrtof(p, &p);
	if (q != p)
	{
		while (*p == ' ')
		{
			++p;
		}
		if (*p == ':')											// special code for REALvision
		{
			minutes = secs;
			secs = SafeStrtof(p + 1, &p);
			if (*p == ':')
			{
				hours = minutes;
				minutes = secs;
				secs = SafeStrtof(p + 1, &p);
				// I am assuming that it stops at hours
			}
		}
		else
		{
			if (*p == 'd')
			{
				days = secs;
				if (StringStartsWithIgnoreCase(p, "day"))			// not sure if any slicer needs this, but include it j.i.c.
				{
					p += 3;
					if (*p == 's')
					{
						++p;
					}
				}
				else
				{
					++p;
				}
				secs = SafeStrtof(p, &p);
				while (*p == ' ' || *p == ':')
				{
					++p;
				}
			}
			if (*p == 'h')
			{
				hours = secs;
				if (StringStartsWithIgnoreCase(p, "hour"))		// S3D
				{
					p += 4;
					if (*p == 's')
					{
						++p;
					}
				}
				else
				{
					++p;
				}
				secs = SafeStrtof(p, &p);
				while (*p == ' ' || *p == ':')					// Fusion 360 gives e.g. ";Print time: 40m:36s"
				{
					++p;
				}
			}
			if (*p== 'm')
			{
				minutes = secs;
				if (StringStartsWithIgnoreCase(p, "minute"))
				{
					p += 6;
					if (*p == 's')
					{
						++p;
					}
				}
				else if (StringStartsWithIgnoreCase(p, "min"))	// Fusion 360
				{
					p += 3;
				}
				else
				{
					++p;
				}
				secs = SafeStrtof(p, &p);
			}
		}
	}
	parsedFileInfo.printTime = lrintf(((days * 24.0 + hours) * 60.0 + minutes) * 60.0 + secs);
}

// Process the simulated time
void FileInfoParser::ProcessSimulatedTime(const char *_ecv_array k, const char *_ecv_array p, int param) noexcept
{
	const char *_ecv_array const q = p;
	const uint32_t secs = StrToU32(p, &p);
	if (q != p)
	{
		parsedFileInfo.simulatedTime = secs;
	}
}

// Process a thumbnail
// The overlap is small enough that we can discount the possibility of finding more than one thumbnail header in the overlap area.
// Thumbnail data is preceded by comment lines of the following form:
//	; thumbnail_QOI begin 32x32 2140
// or
//	; thumbnail begin 32x32 2140

void FileInfoParser::ProcessThumbnail(const char *_ecv_array k, const char *_ecv_array p, int param) noexcept
{
	if (numThumbnailsStored == MaxThumbnails)
	{
		return;		// no more space to store thumbnail info
	}

	GCodeFileInfo::ThumbnailInfo::ImageFormat fmt = (param == 2) ? GCodeFileInfo::ThumbnailInfo::ImageFormat::jpeg
													: (param == 1) ? GCodeFileInfo::ThumbnailInfo::ImageFormat::qoi
														:GCodeFileInfo::ThumbnailInfo::ImageFormat::png;
	// Store this thumbnail data
	const char *_ecv_array npos;
	const uint32_t w = StrToU32(p, &npos);
	if (w >= 16 && w <= 500 && *npos == 'x')
	{
		p = npos + 1;
		const uint32_t h = StrToU32(p, &npos);
		if (h >= 16 && h <= 500 && *npos == ' ')
		{
			p = npos + 1;
			const uint32_t size = StrToU32(p, &npos);
			if (size >= 10)
			{
				p = npos;
				while (*p == ' ' || *p == '\r' || *p == '\n')
				{
					++p;
				}
				if (*p == ';')
				{
					const FilePosition offset = bufferStartFilePosition + (p - buf);
					GCodeFileInfo::ThumbnailInfo& th = parsedFileInfo.thumbnails[numThumbnailsStored++];
					th.width = w;
					th.height = h;
					th.size = size;
					th.format = fmt;
					th.offset = offset;
				}
			}
		}
	}
}

// Scan the buffer for a 2-part filament used string. Return the number of filament found.
void FileInfoParser::ProcessFilamentUsedEmbedded(const char *_ecv_array p, const char *_ecv_array s2) noexcept
{
	const char *_ecv_array q1;
	uint32_t num = StrToU32(p, &q1);
	if (q1 != p && num < MaxFilaments && StringStartsWith(q1, s2))
	{
		p = q1 + strlen(s2);
		while (strchr(" :\t", *p) != nullptr)
		{
			++p;	// this allows for " Used: "
		}
		if (isDigit(*p))
		{
			float filamentLength = SafeStrtof(p, nullptr);
			if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
			{
				parsedFileInfo.filamentNeeded[numFilamentsFound] = filamentLength;
				++numFilamentsFound;
			}
		}
	}
}

// Process filament usage comment
void FileInfoParser::ProcessFilamentUsed(const char *_ecv_array k, const char *_ecv_array p, int param) noexcept
{
	if (numFilamentsFound < MaxFilaments)
	{
		switch (param)
		{
		case 0:
			while (isDigit(*p) && numFilamentsFound < MaxFilaments)
			{
				const char* q;
				const float filamentLength = SafeStrtof(p, &q);
				p = q;
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[numFilamentsFound] = filamentLength;
					if (*p == 'm')
					{
						++p;
						if (*p == 'm')
						{
							++p;
						}
						else
						{
							parsedFileInfo.filamentNeeded[numFilamentsFound] *= 1000.0;		// Cura outputs filament used in metres not mm
						}
					}
					++numFilamentsFound;
				}
				while (strchr(", \t", *p) != nullptr)
				{
					++p;
				}
			}
			break;

		case 1:			// S3D
			if (isDigit(*p))
			{
				const float filamentLength = SafeStrtof(p, nullptr);
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[numFilamentsFound] = filamentLength;
					++numFilamentsFound;
				}
			}
			break;

		case 2:			// Kisslicer newer versions
			if (*p == '#')
			{
				++p;				// later KISSlicer versions add a # here
			}
			while(isDigit(*p))
			{
				++p;
			}
			while(strchr(" :=\t", *p) != nullptr)
			{
				++p;
			}

			if (isDigit(*p))
			{
				const float filamentLength = SafeStrtof(p, nullptr);
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[numFilamentsFound] = filamentLength;
					++numFilamentsFound;
				}
			}
			break;

		case 3:			// Kisslicer older versions
			// Special case: Old KISSlicer and Pathio only generate the filament volume, so we need to calculate the length from it
			if (reprap.GetPlatform().GetFilamentWidth() > 0.0)
			{
				const float filamentCMM = SafeStrtof(p, nullptr) * 1000.0;
				if (!std::isnan(filamentCMM) && !std::isinf(filamentCMM))
				{
					parsedFileInfo.filamentNeeded[numFilamentsFound++] = filamentCMM / (Pi * fsquare(reprap.GetPlatform().GetFilamentWidth() * 0.5));
				}
			}
			break;

		case 4:			// Ideamaker e.g. ";Material#1 Used: 868.0"
			ProcessFilamentUsedEmbedded(p, " Used");
			break;

		case 5:			// Fusion 360 e.g. ";Extruder 1 material used: 1811mm"
			ProcessFilamentUsedEmbedded(p, " material used");
			break;
		}
	}
}

// Scan the buffer for a G1 Zxxx command. The buffer is null-terminated.
// This parsing algorithm needs to be fast. The old one sometimes took 5 seconds or more to parse about 120K of data.
// To speed up parsing, we now parse forwards from the start of the buffer. This means we can't stop when we have found a G1 Z command,
// we have to look for a later G1 Z command in the buffer. But it is faster in the (common) case that we don't find a match in the buffer at all.
bool FileInfoParser::FindHeight(const char* bufp, size_t len) noexcept
{
	bool foundHeight = false;
	bool inRelativeMode = false;
	for(;;)
	{
		// Skip to next newline
		char c;
		while (len >= 6 && (c = *bufp) != '\r' && c != '\n')
		{
			++bufp;
			--len;
		}

		// Skip the newline and any leading spaces
		do
		{
			++bufp;			// skip the newline
			--len;
			c = *bufp;
		} while (len >= 5 && (c == ' ' || c == '\t' || c == '\r' || c == '\n'));

		if (len < 5)
		{
			break;			// not enough characters left for a G1 Zx.x command
		}

		++bufp;				// move to 1 character beyond c
		--len;

		// In theory we should skip N and a line number here if they are present, but no slicers seem to generate line numbers
		if (c == 'G')
		{
			if (inRelativeMode)
			{
				// We have seen a G91 in this buffer already, so we are only interested in G90 commands that switch back to absolute mode
				if (bufp[0] == '9' && bufp[1] == '0' && (bufp[2] < '0' || bufp[2] > '9'))
				{
					// It's a G90 command so go back to absolute mode
					inRelativeMode = false;
				}
			}
			else if (*bufp == '1' || *bufp == '0')
			{
				// It could be a G0 or G1 command
				++bufp;
				--len;
				if (*bufp < '0' || *bufp > '9')
				{
					// It is a G0 or G1 command. See if it has a Z parameter.
					while (len >= 4)
					{
						c = *bufp;
						if (c == 'Z')
						{
							const char* zpos = bufp + 1;
							// Check special case of this code ending with ";E" or "; E" - ignore such codes
							while (len > 2 && *bufp != '\n' && *bufp != '\r' && *bufp != ';')
							{
								++bufp;
								--len;
							}
							if ((len >= 2 && StringStartsWith(bufp, ";E")) || (len >= 3 && StringStartsWith(bufp, "; E")))
							{
								// Ignore this G1 Z command
							}
							else
							{
								float objectHeight = SafeStrtof(zpos, nullptr);
								if (!std::isnan(objectHeight) && !std::isinf(objectHeight))
								{
									parsedFileInfo.objectHeight = objectHeight;
									foundHeight = true;
								}
							}
							break;		// carry on looking for a later G1 Z command
						}
						if (c == ';' || c == '\n' || c == '\r')
						{
							break;		// no Z parameter
						}
						++bufp;
						--len;
					}
				}
			}
			else if (bufp[0] == '9' && bufp[1] == '1' && (bufp[2] < '0' || bufp[2] > '9'))
			{
				// It's a G91 command
				inRelativeMode = true;
			}
		}
		else if (c == ';')
		{
			static const char kisslicerHeightString[] = " END_LAYER_OBJECT z=";
			if (len > 31 && StringStartsWithIgnoreCase(bufp, kisslicerHeightString))
			{
				float objectHeight = SafeStrtof(bufp + sizeof(kisslicerHeightString)/sizeof(char) - 1, nullptr);
				if (!std::isnan(objectHeight) && !std::isinf(objectHeight))
				{
					parsedFileInfo.objectHeight = objectHeight;
					return true;
				}
			}
		}
	}
	return foundHeight;
}

#endif

// End
