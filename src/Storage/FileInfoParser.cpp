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

FileInfoParser::FileInfoParser() noexcept
	: parseState(notParsing), fileBeingParsed(nullptr), accumulatedParseTime(0), accumulatedReadTime(0), accumulatedSeekTime(0), fileOverlapLength(0)
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
		fileOverlapLength = 0;

		// Set up the info struct
		parsedFileInfo.Init();
		parsedFileInfo.fileSize = fileBeingParsed->Length();
#if HAS_MASS_STORAGE
		parsedFileInfo.lastModifiedTime = MassStorage::GetLastModifiedTime(filePath);
#endif
		parsedFileInfo.isValid = true;

		// Record some debug values here
		if (reprap.Debug(modulePrintMonitor))
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
	}

	// Getting file information take a few runs. Speed it up when we are not printing by calling it several times.
	const uint32_t loopStartTime = millis();
	do
	{
		size_t sizeToRead, sizeToScan;										// number of bytes we want to read and scan in this go

		switch (parseState)
		{
		case parsingHeader:
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

				uint32_t startTime = millis();
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
					if (reprap.Debug(modulePrintMonitor))
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
			break;

		case seeking:
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
			break;

		case parsingFooter:
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
					if (reprap.Debug(modulePrintMonitor))
					{
						reprap.GetPlatform().MessageF(UsbMessage, "Footer complete, processed %lu bytes, read time %.3fs, parse time %.3fs, seek time %.3fs\n",
											fileBeingParsed->Length() - fileBeingParsed->Position() + GCODE_READ_SIZE,
											(double)((float)accumulatedReadTime/1000.0), (double)((float)accumulatedParseTime/1000.0), (double)((float)accumulatedSeekTime/1000.0));
					}
					parseState = notParsing;
					fileBeingParsed->Close();
					parsedFileInfo.incomplete = false;
					info = parsedFileInfo;
					return GCodeResult::ok;
				}

				// Else go back further
				fileOverlapLength = (size_t)min<FilePosition>(sizeToScan, GCODE_OVERLAP_SIZE);
				nextSeekPos = (nextSeekPos <= GCODE_READ_SIZE) ? 0 : nextSeekPos - GCODE_READ_SIZE;
				parseState = seeking;
			}
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

// Scan the buffer for the layer height. The buffer is null-terminated.
bool FileInfoParser::FindLayerHeight(const char *bufp) noexcept
{
	static const char* const layerHeightStrings[] =
	{
		"layer_height",			// slic3r
		"Layer height",			// Cura
		"layerHeight",			// S3D
		"layer_thickness_mm",	// Kisslicer
		"layerThickness",		// Matter Control
		"sliceHeight"			// kiri:moto
	};

	if (*bufp != 0)
	{
		++bufp;														// make sure we can look back 1 character after we find a match
		for (const char * lhStr : layerHeightStrings)				// search for each string in turn
		{
			const char *pos = bufp;
			for(;;)													// loop until success or strstr returns null
			{
				pos = strstr(pos, lhStr);
				if (pos == nullptr)
				{
					break;											// didn't find this string in the buffer, so try the next string
				}

				const char c = pos[-1];								// fetch the previous character
				pos += strlen(lhStr);								// skip the string we matched
				if (c == ' ' || c == ';' || c == '\t')				// check we are not in the middle of a word
				{
					while (strchr(" \t=:,", *pos) != nullptr)		// skip the possible separators
					{
						++pos;
					}
					const char *tailPtr;
					const float val = SafeStrtof(pos, &tailPtr);
					if (tailPtr != pos && !std::isnan(val) && !std::isinf(val))	// if we found and converted a number
					{
						parsedFileInfo.layerHeight = val;
						return true;
					}
				}
			}
		}
	}

	return false;
}

bool FileInfoParser::FindSlicerInfo(const char* bufp) noexcept
{
	static const char * const GeneratedByStrings[] =
	{
		"; KISSlicer",		// KISSlicer
		";Sliced at: ",		// Cura (old)
		";Fusion version:",	// Fusion 360
		"generated by ",	// slic3r and S3D
		";Sliced by ",		// ideaMaker
		";Generated with ",	// Cura (new)
		"; Generated by ",	// kiri:moto
		";GENERATOR.NAME:",	// Pathio (the version is separate, we don't include that)
		"; Generated with "	// Matter Control
	};

	size_t index = 0;
	const char* pos;
	do
	{
		pos = strstr(bufp, GeneratedByStrings[index]);
		if (pos != nullptr)
		{
			break;
		}
		++index;
	} while (index < ARRAY_SIZE(GeneratedByStrings));

	if (pos != nullptr)
	{
		const char* introString = "";
		switch (index)
		{
		default:
			pos += strlen(GeneratedByStrings[index]);
			break;

		case 0:		// KISSlicer
			pos += 2;
			break;

		case 1:		// Cura (old)
			introString = "Cura at ";
			pos += strlen(GeneratedByStrings[index]);
			break;

		case 2:		// Fusion 360
			pos += 1;
			break;
		}

		parsedFileInfo.generatedBy.copy(introString);
		while (*pos >= ' ')
		{
			parsedFileInfo.generatedBy.cat(*pos++);
		}
		return true;
	}
	return false;
}

// Scan the buffer for a 2-part filament used string. Return the number of filament found.
void FileInfoParser::FindFilamentUsedEmbedded(const char* p, const char *s1, const char *s2, unsigned int &filamentsFound) noexcept
{
	const size_t maxFilaments = reprap.GetGCodes().GetNumExtruders();
	while (filamentsFound < maxFilaments &&	(p = strstr(p, s1)) != nullptr)
	{
		p += strlen(s1);
		const char *q1, *q2;
		uint32_t num = StrToU32(p, &q1);
		if (q1 != p && num < maxFilaments && (q2 = strstr(q1, s2)) == q1)
		{
			p = q1 + strlen(s2);
			while(strchr(" :\t", *p) != nullptr)
			{
				++p;	// this allows for " Used: "
			}
			if (isDigit(*p))
			{
				float filamentLength = SafeStrtof(p, nullptr);
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[filamentsFound] = filamentLength;
					++filamentsFound;
				}
			}
		}
	}
}

// Scan the buffer for the filament used. The buffer is null-terminated.
// Returns the number of filaments found.
unsigned int FileInfoParser::FindFilamentUsed(const char* bufp) noexcept
{
	unsigned int filamentsFound = 0;
	const size_t maxFilaments = reprap.GetGCodes().GetNumExtruders();

	// Look for filament usage as generated by Slic3r and Cura
	const char* const filamentUsedStr1 = "ilament used";			// comment string used by slic3r and Cura, followed by filament used and "mm"
	const char* p = bufp;
	while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentUsedStr1)) != nullptr)
	{
		p += strlen(filamentUsedStr1);
		while(strchr(" [m]:=\t", *p) != nullptr)					// Prusa slicer now uses "; filament used [mm] = 4235.9"
		{
			++p;	// this allows for " = " from default slic3r comment and ": " from default Cura comment
		}
		while (isDigit(*p) && filamentsFound < maxFilaments)
		{
			const char* q;
			float filamentLength = SafeStrtof(p, &q);
			p = q;
			if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
			{
				parsedFileInfo.filamentNeeded[filamentsFound] = filamentLength;
				if (*p == 'm')
				{
					++p;
					if (*p == 'm')
					{
						++p;
					}
					else
					{
						parsedFileInfo.filamentNeeded[filamentsFound] *= 1000.0;		// Cura outputs filament used in metres not mm
					}
				}
				++filamentsFound;
			}
			while (strchr(", \t", *p) != nullptr)
			{
				++p;
			}
		}
	}

	// Look for filament usage strings generated by Ideamaker, e.g. ";Material#1 Used: 868.0"
	FindFilamentUsedEmbedded(bufp, ";Material#", " Used", filamentsFound);

	// Look for filament usage strings generated by Fusion 360, e.g. ";Extruder 1 material used: 1811mm"
	FindFilamentUsedEmbedded(bufp, ";Extruder ", " material used", filamentsFound);


	// Look for filament usage as generated by S3D
	if (filamentsFound == 0)
	{
		const char *filamentLengthStr = "ilament length";	// comment string used by S3D
		p = bufp;
		while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentLengthStr)) != nullptr)
		{
			p += strlen(filamentLengthStr);
			while(strchr(" :=\t", *p) != nullptr)
			{
				++p;
			}
			if (isDigit(*p))
			{
				float filamentLength = SafeStrtof(p, nullptr);
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[filamentsFound] = filamentLength;
					++filamentsFound;
				}
			}
		}
	}

	// Look for filament usage as generated by recent KISSlicer versions
	if (filamentsFound == 0)
	{
		const char *filamentLengthStr = ";    Ext ";
		p = bufp;
		while (filamentsFound < maxFilaments && (p = strstr(p, filamentLengthStr)) != nullptr)
		{
			p += strlen(filamentLengthStr);
			if (*p == '#')
			{
				++p;				// later KISSlicer versions add a # here
			}
			while(isdigit(*p))
			{
				++p;
			}
			while(strchr(" :=\t", *p) != nullptr)
			{
				++p;
			}

			if (isDigit(*p))
			{
				float filamentLength = SafeStrtof(p, nullptr);
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[filamentsFound] = filamentLength;
					++filamentsFound;
				}
			}
		}
	}

	// Special case: Old KISSlicer and Pathio only generate the filament volume, so we need to calculate the length from it
	if (filamentsFound == 0 && reprap.GetPlatform().GetFilamentWidth() > 0.0)
	{
		const char *filamentVolumeStr = "; Estimated Build Volume: ";				// old KISSlicer
		float multipler = 1000.0;													// volume is in cm^3
		p = strstr(bufp, filamentVolumeStr);
		if (p == nullptr)
		{
			filamentVolumeStr = ";EXTRUDER_TRAIN.0.MATERIAL.VOLUME_USED:";			// Pathio
			multipler = 1.0;														// volume is in mm^3
			p = strstr(bufp, filamentVolumeStr);
		}
		if (p != nullptr)
		{
			const float filamentCMM = SafeStrtof(p + strlen(filamentVolumeStr), nullptr) * multipler;
			if (!std::isnan(filamentCMM) && !std::isinf(filamentCMM))
			{
				parsedFileInfo.filamentNeeded[filamentsFound++] = filamentCMM / (Pi * fsquare(reprap.GetPlatform().GetFilamentWidth() * 0.5));
			}
		}
	}

	return filamentsFound;
}

// Scan the buffer for the estimated print time
bool FileInfoParser::FindPrintTime(const char* bufp) noexcept
{
	static const char* const PrintTimeStrings[] =
	{
		// Note: if a string in this table is a leading or embedded substring of another, the longer one must come first
		" estimated printing time (normal mode)",	// slic3r PE later versions			"; estimated printing time (normal mode) = 2d 1h 5m 24s"
		" estimated printing time",					// slic3r PE older versions			"; estimated printing time = 1h 5m 24s"
		";TIME",									// Cura								";TIME:38846"
		" Build time",								// S3D								";   Build time: 0 hours 42 minutes"
													// also REALvision					"; Build time: 2:11:47"
		" Build Time",								// KISSlicer						"; Estimated Build Time:   332.83 minutes"
													// also KISSSlicer 2 alpha			"; Calculated-during-export Build Time: 130.62 minutes"
		";Print Time:",								// Ideamaker
		";PRINT.TIME:",								// Patio
		";Print time:",								// Fusion 360
		"; total print time (s) ="					// Matter Control
	};

	for (const char * ptStr : PrintTimeStrings)
	{
		const char* pos = strstr(bufp, ptStr);
		if (pos != nullptr)
		{
			pos += strlen(ptStr);
			while (strchr(" \t=:", *pos))
			{
				++pos;
			}
			const char * const q = pos;
			float days = 0.0, hours = 0.0, minutes = 0.0;
			float secs = SafeStrtof(pos, &pos);
			if (q != pos)
			{
				while (*pos == ' ')
				{
					++pos;
				}
				if (*pos == ':')											// special code for REALvision
				{
					minutes = secs;
					secs = SafeStrtof(pos + 1, &pos);
					if (*pos == ':')
					{
						hours = minutes;
						minutes = secs;
						secs = SafeStrtof(pos + 1, &pos);
						// I am assuming that it stops at hours
					}
				}
				else
				{
					if (*pos == 'd')
					{
						days = secs;
						if (StringStartsWithIgnoreCase(pos, "day"))			// not sure if any slicer needs this, but include it j.i.c.
						{
							pos += 3;
							if (*pos == 's')
							{
								++pos;
							}
						}
						else
						{
							++pos;
						}
						secs = SafeStrtof(pos, &pos);
						while (*pos == ' ' || *pos == ':')
						{
							++pos;
						}
					}
					if (*pos == 'h')
					{
						hours = secs;
						if (StringStartsWithIgnoreCase(pos, "hour"))		// S3D
						{
							pos += 4;
							if (*pos == 's')
							{
								++pos;
							}
						}
						else
						{
							++pos;
						}
						secs = SafeStrtof(pos, &pos);
						while (*pos == ' ' || *pos == ':')					// Fusion 360 gives e.g. ";Print time: 40m:36s"
						{
							++pos;
						}
					}
					if (*pos == 'm')
					{
						minutes = secs;
						if (StringStartsWithIgnoreCase(pos, "minute"))
						{
							pos += 6;
							if (*pos == 's')
							{
								++pos;
							}
						}
						else if (StringStartsWithIgnoreCase(pos, "min"))	// Fusion 360
						{
							pos += 3;
						}
						else
						{
							++pos;
						}
						secs = SafeStrtof(pos, &pos);
					}
				}
			}
			parsedFileInfo.printTime = lrintf(((days * 24.0 + hours) * 60.0 + minutes) * 60.0 + secs);
			return true;
		}
	}
	return false;
}

// Scan the buffer for the simulated print time
bool FileInfoParser::FindSimulatedTime(const char* bufp) noexcept
{
	const char* pos = strstr(bufp, SimulatedTimeString);
	if (pos != nullptr)
	{
		pos += strlen(SimulatedTimeString);
		while (strchr(" \t=:", *pos))
		{
			++pos;
		}
		const char * const q = pos;
		const uint32_t secs = StrToU32(pos, &pos);
		if (q != pos)
		{
			parsedFileInfo.simulatedTime = secs;
			return true;
		}
	}
	return false;
}

// Search for embedded thumbnail images that start in the buffer
// Subsequent calls pass a buffer that overlaps with the previous one, so take care to avoid finding the same one twice.
// The overlap is small enough that we can discount the possibility of finding more than one thumbnail header in the overlap area.
// Return true if we have no room to store further thumbnails, or we are certain that we have found all the thumbnails in the file.
// Thumbnail data is preceded by comment lines of the following form:
//	; QOI thumbnail begin 32x32 2140
// or
//	; thumbnail begin 32x32 2140
bool FileInfoParser::FindThumbnails(const char *_ecv_array bufp, FilePosition bufferStartFilePosition) noexcept
{
	// Find the next free slot in which to store thumbnail data
	size_t thumbnailIndex = 0;
	for (thumbnailIndex = 0; parsedFileInfo.thumbnails[thumbnailIndex].IsValid(); )
	{
		++thumbnailIndex;
		if (thumbnailIndex == GCodeFileInfo::MaxThumbnails)
		{
			return true;		// no more space to store thumbnail info
		}
	}

	constexpr const char * PngThumbnailBegin = "; thumbnail begin ";
	constexpr const char * QoiThumbnailBegin = "; QOI thumbnail begin ";
	const char *_ecv_array pos = bufp;
	while (true)
	{
		const char *_ecv_array qoiPos = strstr(pos, QoiThumbnailBegin);
		const char *_ecv_array pngPos = strstr(pos, PngThumbnailBegin);
		GCodeFileInfo::ThumbnailInfo::Format fmt(GCodeFileInfo::ThumbnailInfo::Format::qoi);
		if (qoiPos != nullptr && (pngPos == nullptr || qoiPos < pngPos))
		{
			// found a QOI thumbnail
			pos = qoiPos + strlen(QoiThumbnailBegin);
			fmt = GCodeFileInfo::ThumbnailInfo::Format::qoi;
		}
		else if (pngPos != nullptr)
		{
			// found a PNG thumbnail
			pos = pngPos + strlen(PngThumbnailBegin);
			fmt = GCodeFileInfo::ThumbnailInfo::Format::png;
		}
		else
		{
			return false;		// no more thumbnails in this buffer, but we have room to save more thumbnail details
		}

		// Store this thumbnail data
		const char *_ecv_array npos;
		const uint32_t w = StrToU32(pos, &npos);
		if (w >= 16 && w <= 500 && *npos == 'x')
		{
			pos = npos + 1;
			const uint32_t h = StrToU32(pos, &npos);
			if (h >= 16 && h <= 500 && *npos == ' ')
			{
				pos = npos + 1;
				const uint32_t size = StrToU32(pos, &npos);
				if (size >= 10)
				{
					pos = npos;
					while (*pos == ' ' || *pos == '\r' || *pos == '\n')
					{
						++pos;
					}
					if (*pos == ';')
					{
						const FilePosition offset = bufferStartFilePosition + (pos - bufp);
						if (thumbnailIndex == 0 || offset != parsedFileInfo.thumbnails[thumbnailIndex - 1].offset)
						{
							GCodeFileInfo::ThumbnailInfo& th = parsedFileInfo.thumbnails[thumbnailIndex];
							th.width = w;
							th.height = h;
							th.size = size;
							th.format = fmt;
							th.offset = offset;
							++thumbnailIndex;
							if (thumbnailIndex == GCodeFileInfo::MaxThumbnails)
							{
								return true;		// no more space to store thumbnail info
							}
						}
					}
				}
			}
		}
	}
}

#endif

// End
