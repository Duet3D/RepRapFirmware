/*
 * FileInfoParser.cpp
 *
 *  Created on: 31 Mar 2018
 *      Author: David
 */

#include "FileInfoParser.h"
#include "OutputMemory.h"
#include "RepRap.h"
#include "Platform.h"
#include "PrintMonitor.h"
#include "GCodes/GCodes.h"

void GCodeFileInfo::Init()
{
	isValid = false;
	incomplete = true;
	firstLayerHeight = 0.0;
	objectHeight = 0.0;
	layerHeight = 0.0;
	printTime = simulatedTime = 0;
	numFilaments = 0;
	generatedBy.Clear();
	for (size_t extr = 0; extr < MaxExtruders; extr++)
	{
		filamentNeeded[extr] = 0.0;
	}
}

FileInfoParser::FileInfoParser()
	: parseState(notParsing), fileBeingParsed(nullptr), accumulatedParseTime(0), accumulatedReadTime(0), accumulatedSeekTime(0), fileOverlapLength(0)
{
	parsedFileInfo.Init();
	parserMutex.Create("FileInfoParser");
}

bool FileInfoParser::GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly)
{
	MutexLocker lock(parserMutex, MAX_FILEINFO_PROCESS_TIME);
	if (!lock)
	{
		return false;
	}

	if (parseState != notParsing && !StringEqualsIgnoreCase(filePath, filenameBeingParsed.c_str()))
	{
		// We are already parsing a different file
		if (millis() - lastFileParseTime < MaxFileParseInterval)
		{
			return false;				// try again later
		}

		// Time this client out because it has probably disconnected
		fileBeingParsed->Close();
		parseState = notParsing;
	}

	if (parseState == notParsing)
	{
		// See if we can access the file
		// Webserver may call rr_fileinfo for a directory, check this case here
		if (reprap.GetPlatform().GetMassStorage()->DirectoryExists(filePath))
		{
			info.isValid = false;
			return true;
		}

		fileBeingParsed = reprap.GetPlatform().GetMassStorage()->OpenFile(filePath, OpenMode::read, 0);
		if (fileBeingParsed == nullptr)
		{
			// Something went wrong - we cannot open it
			info.isValid = false;
			return true;
		}

		// File has been opened, let's start now
		filenameBeingParsed.copy(filePath);
		fileOverlapLength = 0;

		// Set up the info struct
		parsedFileInfo.Init();
		parsedFileInfo.fileSize = fileBeingParsed->Length();
		parsedFileInfo.lastModifiedTime = reprap.GetPlatform().GetMassStorage()->GetLastModifiedTime(filePath);
		parsedFileInfo.isValid = true;

		// Record some debug values here
		if (reprap.Debug(modulePrintMonitor))
		{
			accumulatedReadTime = accumulatedParseTime = 0;
			reprap.GetPlatform().MessageF(UsbMessage, "-- Parsing file %s --\n", filePath);
		}

		// If the file is empty or not a G-Code file, we don't need to parse anything
		if (fileBeingParsed->Length() == 0 || (!StringEndsWithIgnoreCase(filePath, ".gcode") && !StringEndsWithIgnoreCase(filePath, ".g")
					&& !StringEndsWithIgnoreCase(filePath, ".gco") && !StringEndsWithIgnoreCase(filePath, ".gc")))
		{
			fileBeingParsed->Close();
			parsedFileInfo.incomplete = false;
			info = parsedFileInfo;
			return true;
		}
		parseState = parsingHeader;
	}

	// Getting file information take a few runs. Speed it up when we are not printing by calling it several times.
	const uint32_t loopStartTime = millis();
	do
	{
		char* const buf = reinterpret_cast<char*>(buf32);
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
				const int nbytes = fileBeingParsed->Read(&buf[fileOverlapLength], sizeToRead);
				if (nbytes != (int)sizeToRead)
				{
					reprap.GetPlatform().MessageF(ErrorMessage, "Failed to read header of G-Code file \"%s\"\n", filePath);
					parseState = notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return true;
				}
				buf[sizeToScan] = 0;

				// Record performance data
				uint32_t now = millis();
				accumulatedReadTime += now - startTime;
				startTime = now;

				// Search for filament usage (Cura puts it at the beginning of a G-code file)
				if (parsedFileInfo.numFilaments == 0)
				{
					parsedFileInfo.numFilaments = FindFilamentUsed(buf, sizeToScan);
					headerInfoComplete &= (parsedFileInfo.numFilaments != 0);
				}

				// Look for first layer height
				if (parsedFileInfo.firstLayerHeight == 0.0)
				{
					headerInfoComplete &= FindFirstLayerHeight(buf, sizeToScan);
				}

				// Look for layer height
				if (parsedFileInfo.layerHeight == 0.0)
				{
					headerInfoComplete &= FindLayerHeight(buf, sizeToScan);
				}

				// Look for slicer program
				if (parsedFileInfo.generatedBy.IsEmpty())
				{
					headerInfoComplete &= FindSlicerInfo(buf, sizeToScan);
				}

				// Look for print time
				if (parsedFileInfo.printTime == 0)
				{
					headerInfoComplete &= FindPrintTime(buf, sizeToScan);
				}

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

				const uint32_t startTime = millis();
				if (!fileBeingParsed->Seek(thisSeekPos))
				{
					reprap.GetPlatform().Message(ErrorMessage, "Could not seek from end of file!\n");
					parseState = notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return true;
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
					reprap.GetPlatform().MessageF(ErrorMessage, "Failed to read footer from G-Code file \"%s\"\n", filePath);
					parseState = notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return true;
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
					parsedFileInfo.numFilaments = FindFilamentUsed(buf, sizeToScan);
					if (parsedFileInfo.numFilaments == 0)
					{
						footerInfoComplete = false;
					}
				}

				// Search for layer height
				if (parsedFileInfo.layerHeight == 0.0)
				{
					if (!FindLayerHeight(buf, sizeToScan))
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
					if (!FindPrintTime(buf, sizeToScan) && fileBeingParsed->Length() - nextSeekPos <= GcodeFooterPrintTimeSearchSize)
					{
						footerInfoComplete = false;
					}
				}

				// Look for simulated print time. It will always be right at the end of the file, so don't look too far back
				if (parsedFileInfo.simulatedTime == 0)
				{
					if (!FindSimulatedTime(buf, sizeToScan) && fileBeingParsed->Length() - nextSeekPos <= GcodeFooterPrintTimeSearchSize)
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
					return true;
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
			return true;
		}
		lastFileParseTime = millis();
	} while (!reprap.GetPrintMonitor().IsPrinting() && lastFileParseTime - loopStartTime < MAX_FILEINFO_PROCESS_TIME);

	if (quitEarly)
	{
		info = parsedFileInfo;				// note that the 'incomplete' flag is still set
		fileBeingParsed->Close();
		parseState = notParsing;
		return true;
	}
	return false;
}

// Scan the buffer for a G1 Zxxx command. The buffer is null-terminated.
bool FileInfoParser::FindFirstLayerHeight(const char* buf, size_t len)
{
	if (len < 4)
	{
		// Don't start if the buffer is not big enough
		return false;
	}
	parsedFileInfo.firstLayerHeight = 0.0;

	bool inComment = false, inRelativeMode = false, foundHeight = false;
	for(size_t i = 0; i < len - 4; i++)
	{
		if (buf[i] == ';')
		{
			inComment = true;
		}
		else if (inComment)
		{
			if (buf[i] == '\n')
			{
				inComment = false;
			}
		}
		else if (buf[i] == 'G')
		{
			// See if we can switch back to absolute mode
			if (inRelativeMode)
			{
				inRelativeMode = !(buf[i + 1] == '9' && buf[i + 2] == '0' && buf[i + 3] <= ' ');
			}
			// Ignore G0/G1 codes if in relative mode
			else if (buf[i + 1] == '9' && buf[i + 2] == '1' && buf[i + 3] <= ' ')
			{
				inRelativeMode = true;
			}
			// Look for "G0/G1 ... Z#HEIGHT#" command
			else if ((buf[i + 1] == '0' || buf[i + 1] == '1') && buf[i + 2] == ' ')
			{
				for(i += 3; i < len - 4; i++)
				{
					if (buf[i] == 'Z')
					{
						//debugPrintf("Found at offset %u text: %.100s\n", i, &buf[i + 1]);
						const float flHeight = SafeStrtof(&buf[i + 1], nullptr);
						if ((parsedFileInfo.firstLayerHeight == 0.0 || flHeight < parsedFileInfo.firstLayerHeight) && (flHeight <= reprap.GetPlatform().GetNozzleDiameter() * 3.0))
						{
							parsedFileInfo.firstLayerHeight = flHeight;				// Only report first Z height if it's somewhat reasonable
							foundHeight = true;
							// NB: Don't stop here, because some slicers generate two Z moves at the beginning
						}
						break;
					}
					else if (buf[i] == ';')
					{
						// Ignore comments
						break;
					}
				}
			}
		}
	}
	return foundHeight;
}

// Scan the buffer for a G1 Zxxx command. The buffer is null-terminated.
// This parsing algorithm needs to be fast. The old one sometimes took 5 seconds or more to parse about 120K of data.
// To speed up parsing, we now parse forwards from the start of the buffer. This means we can't stop when we have found a G1 Z command,
// we have to look for a later G1 Z command in the buffer. But it is faster in the (common) case that we don't find a match in the buffer at all.
bool FileInfoParser::FindHeight(const char* buf, size_t len)
{
	bool foundHeight = false;
	bool inRelativeMode = false;
	for(;;)
	{
		// Skip to next newline
		char c;
		while (len >= 6 && (c = *buf) != '\r' && c != '\n')
		{
			++buf;
			--len;
		}

		// Skip the newline and any leading spaces
		do
		{
			++buf;			// skip the newline
			--len;
			c = *buf;
		} while (len >= 5 && (c == ' ' || c == '\t' || c == '\r' || c == '\n'));

		if (len < 5)
		{
			break;			// not enough characters left for a G1 Zx.x command
		}

		++buf;				// move to 1 character beyond c
		--len;

		// In theory we should skip N and a line number here if they are present, but no slicers seem to generate line numbers
		if (c == 'G')
		{
			if (inRelativeMode)
			{
				// We have seen a G91 in this buffer already, so we are only interested in G90 commands that switch back to absolute mode
				if (buf[0] == '9' && buf[1] == '0' && (buf[2] < '0' || buf[2] > '9'))
				{
					// It's a G90 command so go back to absolute mode
					inRelativeMode = false;
				}
			}
			else if (*buf == '1' || *buf == '0')
			{
				// It could be a G0 or G1 command
				++buf;
				--len;
				if (*buf < '0' || *buf > '9')
				{
					// It is a G0 or G1 command. See if it has a Z parameter.
					while (len >= 4)
					{
						c = *buf;
						if (c == 'Z')
						{
							const char* zpos = buf + 1;
							// Check special case of this code ending with ";E" or "; E" - ignore such codes
							while (len > 2 && *buf != '\n' && *buf != '\r' && *buf != ';')
							{
								++buf;
								--len;
							}
							if ((len >= 2 && StringStartsWith(buf, ";E")) || (len >= 3 && StringStartsWith(buf, "; E")))
							{
								// Ignore this G1 Z command
							}
							else
							{
								parsedFileInfo.objectHeight = SafeStrtof(zpos, nullptr);
								foundHeight = true;
							}
							break;		// carry on looking for a later G1 Z command
						}
						if (c == ';' || c == '\n' || c == '\r')
						{
							break;		// no Z parameter
						}
						++buf;
						--len;
					}
				}
			}
			else if (buf[0] == '9' && buf[1] == '1' && (buf[2] < '0' || buf[2] > '9'))
			{
				// It's a G91 command
				inRelativeMode = true;
			}
		}
		else if (c == ';')
		{
			static const char kisslicerHeightString[] = " END_LAYER_OBJECT z=";
			if (len > 31 && StringStartsWithIgnoreCase(buf, kisslicerHeightString))
			{
				parsedFileInfo.objectHeight = SafeStrtof(buf + sizeof(kisslicerHeightString)/sizeof(char) - 1, nullptr);
				return true;
			}
		}
	}
	return foundHeight;
}

// Scan the buffer for the layer height. The buffer is null-terminated.
bool FileInfoParser::FindLayerHeight(const char *buf, size_t len)
{
	static const char* const layerHeightStrings[] =
	{
		"layer_height",			// slic3r
		"Layer height",			// Cura
		"layerHeight",			// S3D
		"layer_thickness_mm",	// Kisslicer
		"layerThickness"		// Matter Control
	};

	if (*buf != 0)
	{
		++buf;														// make sure we can look back 1 character after we find a match
		for (const char * lhStr : layerHeightStrings)				// search for each string in turn
		{
			const char *pos = buf;
			for(;;)													// loop until success or strstr returns null
			{
				pos = strstr(pos, lhStr);
				if (pos == nullptr)
				{
					break;											// didn't find this string in the buffer, so try the next string
				}

				const char c = pos[-1];								// fetch the previous character
				pos += strlen(lhStr);				// skip the string we matched
				if (c == ' ' || c == ';' || c == '\t')				// check we are not in the middle of a word
				{
					while (strchr(" \t=:,", *pos) != nullptr)		// skip the possible separators
					{
						++pos;
					}
					const char *tailPtr;
					const float val = SafeStrtof(pos, &tailPtr);
					if (tailPtr != pos)								// if we found and converted a number
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

bool FileInfoParser::FindSlicerInfo(const char* buf, size_t len)
{
	static const char * const GeneratedByStrings[] =
	{
		"generated by ",	// slic3r and S3D
		";Sliced by ",		// ideaMaker
		"; KISSlicer",		// KISSlicer
		";Sliced at: ",		// Cura (old)
		";Generated with "	// Cura (new)
	};

	size_t index = 0;
	const char* pos;
	do
	{
		pos = strstr(buf, GeneratedByStrings[index]);
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

		case 2:		// KISSlicer
			pos += 2;
			break;

		case 3:		// Cura (old)
			introString = "Cura at ";
			pos += strlen(GeneratedByStrings[index]);
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

// Scan the buffer for the filament used. The buffer is null-terminated.
// Returns the number of filaments found.
unsigned int FileInfoParser::FindFilamentUsed(const char* buf, size_t len)
{
	unsigned int filamentsFound = 0;
	const size_t maxFilaments = reprap.GetGCodes().GetNumExtruders();

	// Look for filament usage as generated by Slic3r and Cura
	const char* const filamentUsedStr1 = "ilament used";			// comment string used by slic3r and Cura, followed by filament used and "mm"
	const char* p = buf;
	while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentUsedStr1)) != nullptr)
	{
		p += strlen(filamentUsedStr1);
		while(strchr(" :=\t", *p) != nullptr)
		{
			++p;	// this allows for " = " from default slic3r comment and ": " from default Cura comment
		}
		while (isDigit(*p))
		{
			const char* q;
			parsedFileInfo.filamentNeeded[filamentsFound] = SafeStrtof(p, &q);
			p = q;
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
			while (strchr(", \t", *p) != nullptr)
			{
				++p;
			}
		}
	}

	// Look for filament usage string generated by Ideamaker
	const char* const filamentUsedStr2 = ";Material#";			// comment string used by Ideamaker, e.g. ";Material#1 Used: 868.0"
	p = buf;
	while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentUsedStr2)) != nullptr)
	{
		p += strlen(filamentUsedStr2);
		const char *q;
		unsigned long num = SafeStrtoul(p, &q);
		if (q != p && num < maxFilaments)
		{
			p = q;
			while(strchr(" Used:\t", *p) != nullptr)
			{
				++p;	// this allows for " Used: "
			}
			if (isDigit(*p))
			{
				parsedFileInfo.filamentNeeded[filamentsFound] = SafeStrtof(p, nullptr);
				++filamentsFound;
			}
		}
	}

	// Look for filament usage as generated by S3D
	if (filamentsFound == 0)
	{
		const char *filamentLengthStr = "ilament length";	// comment string used by S3D
		p = buf;
		while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentLengthStr)) != nullptr)
		{
			p += strlen(filamentLengthStr);
			while(strchr(" :=\t", *p) != nullptr)
			{
				++p;
			}
			if (isDigit(*p))
			{
				parsedFileInfo.filamentNeeded[filamentsFound] = SafeStrtof(p, nullptr); // S3D reports filament usage in mm, no conversion needed
				++filamentsFound;
			}
		}
	}

	// Look for filament usage as generated by recent KISSlicer versions
	if (filamentsFound == 0)
	{
		const char *filamentLengthStr = ";    Ext ";
		p = buf;
		while (filamentsFound < maxFilaments && (p = strstr(p, filamentLengthStr)) != nullptr)
		{
			p += strlen(filamentLengthStr);
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
				parsedFileInfo.filamentNeeded[filamentsFound] = SafeStrtof(p, nullptr);
				++filamentsFound;
			}
		}
	}

	// Special case: Old KISSlicer only generates the filament volume, so we need to calculate the length from it
	if (filamentsFound == 0)
	{
		const char *filamentVolumeStr = "; Estimated Build Volume: ";
		p = strstr(buf, filamentVolumeStr);
		if (p != nullptr)
		{
			const float filamentCMM = SafeStrtof(p + strlen(filamentVolumeStr), nullptr) * 1000.0;
			parsedFileInfo.filamentNeeded[filamentsFound++] = filamentCMM / (Pi * fsquare(reprap.GetPlatform().GetFilamentWidth() / 2.0));
		}
	}

	return filamentsFound;
}

// Scan the buffer for the estimated print time
bool FileInfoParser::FindPrintTime(const char* buf, size_t len)
{
	static const char* const PrintTimeStrings[] =
	{
		// Note: if a string in this table is a leading or embedded substring of another, the longer one must come first
		" estimated printing time (normal mode)",	// slic3r PE later versions	"; estimated printing time (normal mode) = 1h 5m 24s"
		" estimated printing time",					// slic3r PE older versions	"; estimated printing time = 1h 5m 24s"
		";TIME",									// Cura						";TIME:38846"
		" Build time"								// S3D						";   Build time: 0 hours 42 minutes"
													// also KISSlicer			"; Estimated Build Time:   332.83 minutes"
	};

	for (const char * ptStr : PrintTimeStrings)
	{
		const char* pos = strstr(buf, ptStr);
		if (pos != nullptr)
		{
			pos += strlen(ptStr);
			while (strchr(" \t=:", *pos))
			{
				++pos;
			}
			const char * const q = pos;
			float hours = 0.0, minutes = 0.0;
			float secs = SafeStrtod(pos, &pos);
			if (q != pos)
			{
				while (*pos == ' ')
				{
					++pos;
				}
				if (*pos == 'h')
				{
					hours = secs;
					if (StringStartsWithIgnoreCase(pos, "hours"))
					{
						pos += 5;
					}
					else
					{
						++pos;
					}
					secs = SafeStrtod(pos, &pos);
					while (*pos == ' ')
					{
						++pos;
					}
				}
				if (*pos == 'm')
				{
					minutes = secs;
					if (StringStartsWithIgnoreCase(pos, "minutes"))
					{
						pos += 7;
					}
					else
					{
						++pos;
					}
					secs = SafeStrtod(pos, &pos);
				}
			}
			parsedFileInfo.printTime = lrintf((hours * 60.0 + minutes) * 60.0 + secs);
			return true;
		}
	}
	return false;
}

// Scan the buffer for the simulated print time
bool FileInfoParser::FindSimulatedTime(const char* buf, size_t len)
{
	const char* pos = strstr(buf, SimulatedTimeString);
	if (pos != nullptr)
	{
		pos += strlen(SimulatedTimeString);
		while (strchr(" \t=:", *pos))
		{
			++pos;
		}
		const char * const q = pos;
		const uint32_t secs = SafeStrtoul(pos, &pos);
		if (q != pos)
		{
			parsedFileInfo.simulatedTime = secs;
			return true;
		}
	}
	return false;
}

// End
