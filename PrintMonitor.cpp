/****************************************************************************************************

RepRapFirmware - PrintMonitor

This class provides methods to obtain print end-time estimations and file information from generated
G-Code files, which may be reported to auxiliary devices and to the web interface using status responses.

-----------------------------------------------------------------------------------------------------

Version 0.1

Created on: Feb 24, 2015

Christian Hammacher

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"

PrintMonitor::PrintMonitor(Platform *p, GCodes *gc) : platform(p), gCodes(gc), isPrinting(false), isHeating(false),
	printStartTime(0), pauseStartTime(0.0), totalPauseTime(0.0), currentLayer(0), warmUpDuration(0.0),
	firstLayerDuration(0.0), firstLayerFilament(0.0), firstLayerProgress(0.0), lastLayerChangeTime(0.0),
	lastLayerFilament(0.0), numLayerSamples(0), layerEstimatedTimeLeft(0.0), parseState(notParsing),
	fileBeingParsed(nullptr), fileOverlapLength(0), printingFileParsed(false), accumulatedParseTime(0.0),
	accumulatedReadTime(0.0)
{
	filenameBeingPrinted[0] = 0;
}

void PrintMonitor::Init()
{
	longWait = platform->Time();
}

void PrintMonitor::Spin()
{
	// We might have started a file print while another G-Code file is being parsed.
	// So we need to start this process once the other file has been processed.
	if (filenameBeingPrinted[0] != 0 && !printingFileParsed)
	{
		printingFileParsed = GetFileInfo(platform->GetGCodeDir(), filenameBeingPrinted, printingFileInfo);
		if (!printingFileParsed)
		{
			platform->ClassReport(longWait);
			return;
		}
	}

	// Don't update the print time estimations if there is no file info or if the print has been paused
	if (gCodes->IsPausing() || gCodes->IsPaused() || gCodes->IsResuming())
	{
		if (pauseStartTime == 0.0)
		{
			pauseStartTime = platform->Time();
		}
		platform->ClassReport(longWait);
		return;
	}

	// Otherwise try to update them
	if (IsPrinting() && !reprap.GetRoland()->Active())
	{
		// We might need to adjust the actual print time if it was paused before
		if (pauseStartTime != 0.0)
		{
			totalPauseTime += platform->Time() - pauseStartTime;
			pauseStartTime = 0.0;
		}

		// Have we just started a print? See if we're heating up
		if (warmUpDuration == 0.0)
		{
			// Check if at least one nozzle heater is active and set
			bool heatersAtHighTemperature = false;
			for(size_t heater = E0_HEATER; heater < HEATERS; heater++)
			{
				if (reprap.GetHeat()->GetStatus(heater) == Heat::HS_active &&
					reprap.GetHeat()->GetActiveTemperature(heater) > TEMPERATURE_LOW_SO_DONT_CARE)
				{
					isHeating = true;
					if (reprap.GetHeat()->HeaterAtSetTemperature(heater))
					{
						heatersAtHighTemperature = true;
						isHeating = false;
						break;
					}
				}
			}

			// Yes - do we have live momement?
			if (heatersAtHighTemperature && !reprap.GetMove()->NoLiveMovement())
			{
				// Yes - we're actually starting the print
				WarmUpComplete();
			}
		}
		// Print is in progress...
		else if (currentLayer > 0 && !gCodes->DoingFileMacro())
		{
			float liveCoords[DRIVES + 1];
			reprap.GetMove()->LiveCoordinates(liveCoords);

			// See if we need to determine the first layer height (usually smaller than the nozzle diameter)
			if (printingFileInfo.firstLayerHeight == 0.0)
			{
				if (liveCoords[Z_AXIS] < platform->GetNozzleDiameter() * 1.5)
				{
					// This shouldn't be needed because we parse the first layer height anyway, but it won't harm
					printingFileInfo.firstLayerHeight = liveCoords[Z_AXIS];
				}
			}
			// Then check if we've finished the first layer
			else if (firstLayerDuration == 0.0)
			{
				if (HeightMatches(liveCoords[Z_AXIS], printingFileInfo.firstLayerHeight + printingFileInfo.layerHeight))
				{
					// First layer is complete
					FirstLayerComplete();
				}
			}
			// We have enough values to estimate the following layer heights
			else if (printingFileInfo.objectHeight > 0.0)
			{
				// Check for layer change
				float nextLayerZ = printingFileInfo.firstLayerHeight + currentLayer * printingFileInfo.layerHeight;
				if (HeightMatches(liveCoords[Z_AXIS], nextLayerZ))
				{
					// A new layer is now being printed
					LayerComplete();
				}
			}
		}
	}

	platform->ClassReport(longWait);
}

// Notifies this class that a file has been set for printing
void PrintMonitor::StartingPrint(const char* filename)
{
	printingFileParsed = GetFileInfo(platform->GetGCodeDir(), filename, printingFileInfo);
	strncpy(filenameBeingPrinted, filename, ARRAY_SIZE(filenameBeingPrinted));
	filenameBeingPrinted[ARRAY_UPB(filenameBeingPrinted)] = 0;
}

// Tell this class that the file set for printing is now actually processed
void PrintMonitor::StartedPrint()
{
	isPrinting = true;
	printStartTime = platform->Time();
}

// This is called as soon as the heaters are at temperature and the actual print has started
void PrintMonitor::WarmUpComplete()
{
	warmUpDuration = GetPrintDuration();
	if (printingFileInfo.layerHeight > 0.0) {
		currentLayer = 1;
	}
}

// Called when the first layer has been finished
void PrintMonitor::FirstLayerComplete()
{
	firstLayerFilament = gCodes->GetTotalRawExtrusion();
	firstLayerDuration = GetPrintDuration() - warmUpDuration;
	firstLayerProgress = gCodes->FractionOfFilePrinted();
}

// This is called whenever another layer has been finished
void PrintMonitor::LayerComplete()
{
	// Use untainted extruder positions for filament-based estimation
	const float extrRawTotal = gCodes->GetTotalRawExtrusion();

	// Record a new set of layer, filament and file stats
	if (currentLayer > 1)
	{
		// Record a new set
		if (numLayerSamples < MAX_LAYER_SAMPLES)
		{
			if (numLayerSamples == 0)
			{
				filamentUsagePerLayer[numLayerSamples] = extrRawTotal - firstLayerFilament;
				layerDurations[numLayerSamples] = GetPrintDuration() - warmUpDuration;
			}
			else
			{
				filamentUsagePerLayer[numLayerSamples] = extrRawTotal - lastLayerFilament;
				layerDurations[numLayerSamples] = GetPrintDuration() - lastLayerChangeTime;
			}
			fileProgressPerLayer[numLayerSamples] = gCodes->FractionOfFilePrinted();
			numLayerSamples++;
		}
		else
		{
			for(size_t i = 1; i < MAX_LAYER_SAMPLES; i++)
			{
				layerDurations[i - 1] = layerDurations[i];
				filamentUsagePerLayer[i - 1] = filamentUsagePerLayer[i];
				fileProgressPerLayer[i - 1] = fileProgressPerLayer[i];
			}

			layerDurations[MAX_LAYER_SAMPLES - 1] = GetPrintDuration() - lastLayerChangeTime;
			filamentUsagePerLayer[MAX_LAYER_SAMPLES - 1] = extrRawTotal - lastLayerFilament;
			fileProgressPerLayer[MAX_LAYER_SAMPLES - 1] = gCodes->FractionOfFilePrinted();
		}
	}

	// Update layer-based estimation times
	unsigned int remainingLayers;
	remainingLayers = round((printingFileInfo.objectHeight - printingFileInfo.firstLayerHeight) / printingFileInfo.layerHeight) + 1;
	remainingLayers -= currentLayer;

	float avgLayerTime, avgLayerDelta = 0.0;
	if (numLayerSamples)
	{
		avgLayerTime = 0.0;
		for(size_t layer = 0; layer < numLayerSamples; layer++)
		{
			avgLayerTime += layerDurations[layer];
			if (layer)
			{
				avgLayerDelta += layerDurations[layer] - layerDurations[layer - 1];
			}
		}
		avgLayerTime /= numLayerSamples;
		avgLayerDelta /= numLayerSamples;
	}
	else
	{
		avgLayerTime = firstLayerDuration * FIRST_LAYER_SPEED_FACTOR;
	}

	layerEstimatedTimeLeft = (avgLayerTime * remainingLayers) - (avgLayerDelta * remainingLayers);
	if (layerEstimatedTimeLeft < 0.0)
	{
		layerEstimatedTimeLeft = avgLayerTime * remainingLayers;
	}

	// Set new layer values
	currentLayer++;
	lastLayerChangeTime = GetPrintDuration();
	lastLayerFilament = extrRawTotal;
}

void PrintMonitor::StoppedPrint()
{
	isPrinting = printingFileParsed = false;
	currentLayer = numLayerSamples = 0;
	pauseStartTime = totalPauseTime = 0.0;
	firstLayerDuration = firstLayerFilament = firstLayerProgress = 0.0;
	layerEstimatedTimeLeft = printStartTime = warmUpDuration = 0.0;
	lastLayerChangeTime = lastLayerFilament = 0.0;
}

bool PrintMonitor::GetFileInfo(const char *directory, const char *fileName, GCodeFileInfo& info)
{
	// Webserver may call rr_fileinfo for a directory, check this case here
	if (reprap.GetPlatform()->GetMassStorage()->DirectoryExists(directory, fileName))
	{
		info.isValid = false;
		return true;
	}

	// Are we still parsing a file?
	if (parseState != notParsing)
	{
		if (!StringEquals(fileName, filenameBeingParsed))
		{
			// Yes - but it's not the file we're processing. Try again later
			return false;
		}
	}
	else if (parseState == notParsing)
	{
		// No - see if we can access the file
		fileBeingParsed = platform->GetFileStore(directory, fileName, false);
		if (fileBeingParsed == nullptr)
		{
			// Something went wrong - we cannot open it
			info.isValid = false;
			return true;
		}

		// File has been opened, let's start now
		strncpy(filenameBeingParsed, fileName, ARRAY_SIZE(filenameBeingParsed));
		filenameBeingParsed[ARRAY_UPB(filenameBeingParsed)] = 0;
		fileOverlapLength = 0;

		// Set up the info struct
		parsedFileInfo.isValid = true;
		parsedFileInfo.fileSize = fileBeingParsed->Length();
		parsedFileInfo.firstLayerHeight = 0.0;
		parsedFileInfo.objectHeight = 0.0;
		parsedFileInfo.layerHeight = 0.0;
		parsedFileInfo.numFilaments = 0;
		parsedFileInfo.generatedBy[0] = 0;
		for(size_t extr = 0; extr < DRIVES - AXES; extr++)
		{
			parsedFileInfo.filamentNeeded[extr] = 0.0;
		}

		// Record some debug values here
		if (reprap.Debug(modulePrintMonitor))
		{
			accumulatedReadTime = accumulatedParseTime = 0.0;
			platform->MessageF(GENERIC_MESSAGE, "-- Parsing file %s --\n", fileName);
		}

		// If the file is empty or no G-Code file, we don't need to parse anything
		if (fileBeingParsed->Length() == 0 || (!StringEndsWith(fileName, ".gcode") && !StringEndsWith(fileName, ".g")
					&& !StringEndsWith(fileName, ".gco") && !StringEndsWith(fileName, ".gc")))
		{
			fileBeingParsed->Close();
			info = parsedFileInfo;
			return true;
		}
		parseState = parsingHeader;
	}

	// First try to process the header of the file
	float startTime = platform->Time();
	uint32_t buf32[(GCODE_READ_SIZE + GCODE_OVERLAP_SIZE + 3)/4 + 1];	// buffer should be 32-bit aligned for HSMCI (need the +1 so we can add a null terminator)
	char* const buf = reinterpret_cast<char*>(buf32);
	size_t sizeToRead, sizeToScan;										// number of bytes we want to read and scan in this go

	if (parseState == parsingHeader)
	{
		bool headerInfoComplete = true;

		// Read a chunk from the header. On the first run only process 1024 bytes, but use overlap (total 1124 bytes) next times.
		sizeToRead = (size_t)min<FilePosition>(fileBeingParsed->Length() - fileBeingParsed->Position(), GCODE_READ_SIZE);
		if (fileOverlapLength > 0)
		{
			memcpy(buf, fileOverlap, fileOverlapLength);
			sizeToScan = sizeToRead + fileOverlapLength;
		}
		else
		{
			sizeToScan = sizeToRead;
		}

		int nbytes = fileBeingParsed->Read(&buf[fileOverlapLength], sizeToRead);
		if (nbytes != (int)sizeToRead)
		{
			platform->MessageF(HOST_MESSAGE, "Error: Failed to read header of G-Code file \"%s\"\n", fileName);
			parseState = notParsing;
			fileBeingParsed->Close();
			info = parsedFileInfo;
			return true;
		}
		buf[sizeToScan] = 0;

		// Record performance data
		if (reprap.Debug(modulePrintMonitor))
		{
			const float now = platform->Time();
			accumulatedReadTime += now - startTime;
			startTime = now;
		}

		// Search for filament usage (Cura puts it at the beginning of a G-code file)
		if (parsedFileInfo.numFilaments == 0)
		{
			parsedFileInfo.numFilaments = FindFilamentUsed(buf, sizeToScan, parsedFileInfo.filamentNeeded, DRIVES - AXES);
			headerInfoComplete &= (parsedFileInfo.numFilaments != 0);
		}

		// Look for first layer height
		if (parsedFileInfo.firstLayerHeight == 0.0)
		{
			headerInfoComplete &= FindFirstLayerHeight(buf, sizeToScan, parsedFileInfo.firstLayerHeight);
		}

		// Look for layer height
		if (parsedFileInfo.layerHeight == 0.0)
		{
			headerInfoComplete &= FindLayerHeight(buf, sizeToScan, parsedFileInfo.layerHeight);
		}

		// Look for slicer program
		if (parsedFileInfo.generatedBy[0] == 0)
		{
			// Slic3r and S3D
			const char* generatedByString = "generated by ";
			char* pos = strstr(buf, generatedByString);
			if (pos != nullptr)
			{
				pos += strlen(generatedByString);
				size_t i = 0;
				while (i < ARRAY_SIZE(parsedFileInfo.generatedBy) - 1 && *pos >= ' ')
				{
					char c = *pos++;
					if (c == '"' || c == '\\')
					{
						// Need to escape the quote-mark for JSON
						if (i > ARRAY_SIZE(parsedFileInfo.generatedBy) - 3)
						{
							break;
						}
						parsedFileInfo.generatedBy[i++] = '\\';
					}
					parsedFileInfo.generatedBy[i++] = c;
				}
				parsedFileInfo.generatedBy[i] = 0;
			}

			// Cura
			const char* slicedAtString = ";Sliced at: ";
			pos = strstr(buf, slicedAtString);
			if (pos != nullptr)
			{
				pos += strlen(slicedAtString);
				strcpy(parsedFileInfo.generatedBy, "Cura at ");
				size_t i = 8;
				while (i < ARRAY_SIZE(parsedFileInfo.generatedBy) - 1 && *pos >= ' ')
				{
					char c = *pos++;
					if (c == '"' || c == '\\')
					{
						if (i > ARRAY_SIZE(parsedFileInfo.generatedBy) - 3)
						{
							break;
						}
						parsedFileInfo.generatedBy[i++] = '\\';
					}
					parsedFileInfo.generatedBy[i++] = c;
				}
				parsedFileInfo.generatedBy[i] = 0;
			}

			// KISSlicer
			const char* kisslicerStart = "; KISSlicer";
			if (StringStartsWith(buf, kisslicerStart))
			{
				size_t stringLength = 0;
				for(size_t i = 2; i < ARRAY_UPB(parsedFileInfo.generatedBy); i++)
				{
					if (buf[i] == '\r' || buf[i] == '\n')
					{
						break;
					}

					parsedFileInfo.generatedBy[stringLength++] = buf[i];
				}
				parsedFileInfo.generatedBy[stringLength] = 0;
			}
		}
		headerInfoComplete &= (parsedFileInfo.generatedBy[0] != 0);

		// Keep track of the time stats
		if (reprap.Debug(modulePrintMonitor))
		{
			accumulatedParseTime += platform->Time() - startTime;
		}

		// Can we proceed to the footer? Don't scan more than the first 4KB of the file
		FilePosition pos = fileBeingParsed->Position();
		if (headerInfoComplete || pos >= GCODE_HEADER_SIZE || pos == fileBeingParsed->Length())
		{
			// Yes - see if we need to output some debug info
			if (reprap.Debug(modulePrintMonitor))
			{
				platform->MessageF(GENERIC_MESSAGE, "Header complete, processed %lu bytes total\n", fileBeingParsed->Position());
				platform->MessageF(GENERIC_MESSAGE, "Accumulated file read time: %fs, accumulated parsing time: %fs\n", accumulatedReadTime, accumulatedParseTime);
				accumulatedReadTime = accumulatedParseTime = 0.0;
			}

			// Go to the last sector and proceed from there on
			const FilePosition seekFromEnd = ((fileBeingParsed->Length() - 1) % GCODE_READ_SIZE) + 1;
			fileBeingParsed->Seek(fileBeingParsed->Length() - seekFromEnd);
			fileOverlapLength = 0;
			parseState = parsingFooter;
		}
		else
		{
			// No - copy the last chunk of the buffer for overlapping search
			fileOverlapLength = min<size_t>(sizeToRead, GCODE_OVERLAP_SIZE);
			memcpy(fileOverlap, &buf[sizeToRead - fileOverlapLength], fileOverlapLength);
		}
		return false;
	}

	// Processing the footer. See how many bytes we need to read and if we can reuse the overlap
	bool footerInfoComplete = true;
	FilePosition pos = fileBeingParsed->Position();
	sizeToRead = (size_t)min<FilePosition>(fileBeingParsed->Length() - pos, GCODE_READ_SIZE);
	if (fileOverlapLength > 0)
	{
		memcpy(&buf[sizeToRead], fileOverlap, fileOverlapLength);
		sizeToScan = sizeToRead + fileOverlapLength;
	}
	else
	{
		sizeToScan = sizeToRead;
	}

	// Read another chunk from the footer
	int nbytes = fileBeingParsed->Read(buf, sizeToRead);
	if (nbytes != (int)sizeToRead)
	{
		platform->MessageF(HOST_MESSAGE, "Error: Failed to read footer from G-Code file \"%s\"\n", fileName);
		parseState = notParsing;
		fileBeingParsed->Close();
		info = parsedFileInfo;
		return true;
	}
	buf[sizeToScan] = 0;

	// Record performance data
	if (reprap.Debug(modulePrintMonitor))
	{
		const float now = platform->Time();
		accumulatedReadTime += now - startTime;
		startTime = now;
	}

	// Search for filament used
	if (parsedFileInfo.numFilaments == 0)
	{
		parsedFileInfo.numFilaments = FindFilamentUsed(buf, sizeToScan, parsedFileInfo.filamentNeeded, DRIVES - AXES);
		footerInfoComplete &= (parsedFileInfo.numFilaments != 0);
	}

	// Search for layer height
	if (parsedFileInfo.layerHeight == 0.0)
	{
		footerInfoComplete &= FindLayerHeight(buf, sizeToScan, parsedFileInfo.layerHeight);
	}

	// Search for object height
	if (parsedFileInfo.objectHeight == 0.0)
	{
		footerInfoComplete &= FindHeight(buf, sizeToScan, parsedFileInfo.objectHeight);
	}

	// Keep track of the time stats
	if (reprap.Debug(modulePrintMonitor))
	{
		accumulatedParseTime += platform->Time() - startTime;
	}

	// If we've collected all details, scanned the last 128K of the file or if we cannot go any further, stop here.
	if (footerInfoComplete || pos == 0 || fileBeingParsed->Length() - pos >= GCODE_FOOTER_SIZE)
	{
		if (reprap.Debug(modulePrintMonitor))
		{
			platform->MessageF(GENERIC_MESSAGE, "Footer complete, processed %lu bytes total\n", fileBeingParsed->Length() - fileBeingParsed->Position() + GCODE_READ_SIZE);
			platform->MessageF(GENERIC_MESSAGE, "Accumulated file read time: %fs, accumulated parsing time: %fs\n", accumulatedReadTime, accumulatedParseTime);
		}
		parseState = notParsing;
		fileBeingParsed->Close();
		info = parsedFileInfo;
		return true;
	}

	// Else go back further
	size_t seekOffset = (size_t)min<FilePosition>(pos, GCODE_READ_SIZE);
	if (!fileBeingParsed->Seek(pos - seekOffset))
	{
		platform->Message(HOST_MESSAGE, "Error: Could not seek from end of file!\n");
		parseState = notParsing;
		fileBeingParsed->Close();
		info = parsedFileInfo;
		return true;
	}

	fileOverlapLength = (size_t)min<FilePosition>(sizeToScan, GCODE_OVERLAP_SIZE);
	memcpy(fileOverlap, buf, fileOverlapLength);
	return false;
}

// Get information for the specified file, or the currently printing file, in JSON format
bool PrintMonitor::GetFileInfoResponse(const char *filename, OutputBuffer *&response)
{
	// Poll file info for a specific file
	if (filename != nullptr)
	{
		GCodeFileInfo info;
		if (!GetFileInfo("0:/", filename, info))
		{
			// This may take a few runs...
			return false;
		}

		if (info.isValid)
		{
			if (!reprap.AllocateOutput(response))
			{
				// Should never happen
				return false;
			}

			response->printf("{\"err\":0,\"size\":%lu,\"height\":%.2f,\"firstLayerHeight\":%.2f,\"layerHeight\":%.2f,\"filament\":",
							info.fileSize, info.objectHeight, info.firstLayerHeight, info.layerHeight);
			char ch = '[';
			if (info.numFilaments == 0)
			{
				response->cat(ch);
			}
			else
			{
				for(size_t i = 0; i < info.numFilaments; ++i)
				{
					response->catf("%c%.1f", ch, info.filamentNeeded[i]);
					ch = ',';
				}
			}
			response->catf("],\"generatedBy\":\"%s\"}", info.generatedBy);
		}
		else
		{
			if (!reprap.AllocateOutput(response))
			{
				// Should never happen
				return false;
			}

			response->copy("{\"err\":1}");
		}
	}
	else if (IsPrinting())
	{
		if (!reprap.AllocateOutput(response))
		{
			// Should never happen
			return false;
		}

		// If we are still busy processing the file, return err code 2 so the web interface knows what's going on
		if (!printingFileParsed)
		{
			response->copy("{\"err\":2}");
			return true;
		}

		// Poll file info about a file currently being printed
		response->printf("{\"err\":0,\"size\":%lu,\"height\":%.2f,\"firstLayerHeight\":%.2f,\"layerHeight\":%.2f,\"filament\":",
						printingFileInfo.fileSize, printingFileInfo.objectHeight, printingFileInfo.firstLayerHeight, printingFileInfo.layerHeight);
		char ch = '[';
		if (printingFileInfo.numFilaments == 0)
		{
			response->cat(ch);
		}
		else
		{
			for (size_t i = 0; i < printingFileInfo.numFilaments; ++i)
			{
				response->catf("%c%.1f", ch, printingFileInfo.filamentNeeded[i]);
				ch = ',';
			}
		}
		response->catf("],\"generatedBy\":\"%s\",\"printDuration\":%d,\"fileName\":\"%s\"}",
				printingFileInfo.generatedBy, (int)GetPrintDuration(), filenameBeingPrinted);
	}
	else
	{
		if (!reprap.AllocateOutput(response))
		{
			// Should never happen
			return false;
		}

		response->copy("{\"err\":1}");
	}
	return true;
}

void PrintMonitor::StopParsing(const char *filename)
{
	if (parseState == notParsing)
	{
		// We're not parsing anything, stop here
		return;
	}

	if (filenameBeingPrinted[0] != 0 && !printingFileParsed)
	{
		if (StringEquals(filename, filenameBeingPrinted))
		{
			// If this is the file we're parsing for internal purposes, don't bother with this request
			return;
		}
	}

	if (StringEquals(filenameBeingParsed, filename))
	{
		parseState = notParsing;
		fileBeingParsed->Close();
	}
}

// Estimate the print time left in seconds on a preset estimation method
float PrintMonitor::EstimateTimeLeft(PrintEstimationMethod method) const
{
	// We can't provide an estimation if we're not printing (yet)
	if (!printingFileParsed && warmUpDuration == 0.0)
	{
		return 0.0;
	}

	// How long have we been printing continuously?
	float realPrintDuration = GetPrintDuration() - warmUpDuration;
	if (numLayerSamples != 0)
	{
		// Take into account the first layer time only if we haven't got any other samples
		realPrintDuration -= firstLayerDuration;
	}

	// Actual estimations
	switch (method)
	{
		case fileBased:
		{
			const float fractionPrinted = gCodes->FractionOfFilePrinted();

			// Provide rough estimation only if we haven't collected at least 2 layer samples
			if (numLayerSamples < 2 || !printingFileParsed || printingFileInfo.objectHeight == 0.0)
			{
				return (fractionPrinted < 0.01)
						? 0.0
						: realPrintDuration * (1.0 / fractionPrinted) - realPrintDuration;
			}

			// Work out how much progress we made in the layers we have data for, and how long it took.
			// Can't use the first layer sample in the table because we don't know the fraction printed at the start.
			float duration = 0.0;
			for(size_t layer = 1; layer < numLayerSamples; layer++)
			{
				duration += layerDurations[layer];
			}
			const float fractionPrintedInLayers = fileProgressPerLayer[numLayerSamples - 1] - fileProgressPerLayer[0];
			return (fractionPrintedInLayers < 0.01)
					? 0.0
					: duration * (1.0 - fractionPrinted)/fractionPrintedInLayers;
		}

		case filamentBased:
		{
			// Need some file information, otherwise this method won't work
			if (!printingFileParsed || printingFileInfo.numFilaments == 0 || reprap.GetRoland()->Active())
			{
				return 0.0;
			}

			// Sum up the filament usage and the filament needed
			float totalFilamentNeeded = 0.0;
			const float extrRawTotal = gCodes->GetTotalRawExtrusion();
			for(size_t extruder = 0; extruder < DRIVES - AXES; extruder++)
			{
				totalFilamentNeeded += printingFileInfo.filamentNeeded[extruder];
			}

			// If we have a reasonable amount of filament extruded, calculate estimated times left
			if (totalFilamentNeeded > 0.0 && extrRawTotal > totalFilamentNeeded * ESTIMATION_MIN_FILAMENT_USAGE)
			{
				if (firstLayerFilament == 0.0)
				{
					return realPrintDuration * (totalFilamentNeeded - extrRawTotal) / extrRawTotal;
				}
				if (extrRawTotal >= totalFilamentNeeded)
				{
					return 0.0;		// Avoid division by zero, else the web interface will report AJAX errors
				}

				float filamentRate;
				if (numLayerSamples)
				{
					filamentRate = 0.0;
					for(size_t i = 0; i < numLayerSamples; i++)
					{
						filamentRate += filamentUsagePerLayer[i] / layerDurations[i];
					}
					filamentRate /= numLayerSamples;
				}
				else
				{
					filamentRate = firstLayerFilament / firstLayerDuration;
				}

				return (totalFilamentNeeded - extrRawTotal) / filamentRate;
			}
			break;
		}

		case layerBased:
			if (layerEstimatedTimeLeft > 0.0)
			{
				float timeLeft = layerEstimatedTimeLeft - (GetPrintDuration() - lastLayerChangeTime);
				if (timeLeft > 0.0)
				{
					return timeLeft;
				}
			}
			break;
	}

	return 0.0;
}

// Scan the buffer for a G1 Zxxx command. The buffer is null-terminated.
bool PrintMonitor::FindFirstLayerHeight(const char* buf, size_t len, float& height) const
{
	if (len < 4)
	{
		// Don't start if the buffer is not big enough
		return false;
	}

//debugPrintf("Scanning %u bytes starting %.100s\n", len, buf);
	bool inComment = false, inRelativeMode = false;
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
			// Look for first "G0/G1 ... Z#HEIGHT#" command
			else if ((buf[i + 1] == '0' || buf[i + 1] == '1') && buf[i + 2] == ' ')
			{
				for(i += 3; i < len - 4; i++)
				{
					if (buf[i] == 'Z')
					{
						//debugPrintf("Found at offset %u text: %.100s\n", i, &buf[i + 1]);
						float flHeight = strtod(&buf[i + 1], nullptr);
						if (flHeight <= platform->GetNozzleDiameter() * 3.0)
						{
							height = flHeight;	// Only report first Z height if it's somewhat reasonable
							return true;
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
	return false;
}
// Scan the buffer in reverse for a G1 Zxxx command. The buffer is null-terminated.
bool PrintMonitor::FindHeight(const char* buf, size_t len, float& height) const
{
//debugPrintf("Scanning %u bytes starting %.100s\n", len, buf);
	bool inComment, inRelativeMode = false;
	unsigned int zPos;
	for(int i = (int)len - 5; i > 0; i--)
	{
		if (inRelativeMode)
		{
			inRelativeMode = !(buf[i] == 'G' && buf[i + 1] == '9' && buf[i + 2] == '1' && buf[i + 3] <= ' ');
		}
		else if (buf[i] == 'G')
		{
			// Ignore G0/G1 codes if absolute mode was switched back using G90 (typical for Cura files)
			if (buf[i + 1] == '9' && buf[i + 2] == '0' && buf[i + 3] <= ' ')
			{
				inRelativeMode = true;
			}
			// Look for last "G0/G1 ... Z#HEIGHT#" command as generated by common slicers
			else if ((buf[i + 1] == '0' || buf[i + 1] == '1') && buf[i + 2] == ' ')
			{
				// Looks like we found a controlled move, however it could be in a comment, especially when using slic3r 1.1.1
				inComment = false;
				size_t j = i;
				while (j != 0)
				{
					--j;
					char c = buf[j];
					if (c == '\n' || c == '\r')
					{
						// It's not in a comment
						break;
					}
					if (c == ';')
					{
						// It is in a comment, so give up on this one
						inComment = true;
						break;
					}
				}
				if (inComment)
					continue;

				// Find 'Z' position and grab that value
				zPos = 0;
				for(int j = i + 3; j < (int)len - 2; j++)
				{
					char c = buf[j];
					if (c < ' ')
					{
						// Skip all whitespaces...
						while (j < (int)len - 2 && c <= ' ')
						{
							c = buf[++j];
						}
						// ...to make sure ";End" doesn't follow G0 .. Z#HEIGHT#
						if (zPos != 0 && (buf[j] != ';' || buf[j + 1] != 'E'))
						{
							//debugPrintf("Found at offset %u text: %.100s\n", zPos, &buf[zPos + 1]);
							height = strtod(&buf[zPos + 1], nullptr);
							return true;
						}
						break;
					}
					else if (c == ';')
					{
						// Ignore comments
						break;
					}
					else if (c == 'Z')
					{
						zPos = j;
					}
				}
			}
		}
		// Special case: KISSlicer generates object height as a comment
		else
		{
			const char *kisslicerHeightString = "; END_LAYER_OBJECT z=";
			if (i < (int)len - 32 && StringStartsWith(buf + i, kisslicerHeightString))
			{
				height = strtod(buf + i + strlen(kisslicerHeightString), nullptr);
				return true;
			}
		}
	}
	return false;
}

// Scan the buffer for the layer height. The buffer is null-terminated.
bool PrintMonitor::FindLayerHeight(const char *buf, size_t len, float& layerHeight) const
{
	// Look for layer_height as generated by Slic3r
	const char* layerHeightStringSlic3r = "; layer_height ";
	const char *pos = strstr(buf, layerHeightStringSlic3r);
	if (pos != nullptr)
	{
		pos += strlen(layerHeightStringSlic3r);
		while (strchr(" \t=:", *pos))
		{
			++pos;
		}
		layerHeight = strtod(pos, nullptr);
		return true;
	}

	// Look for layer height as generated by Cura
	const char* layerHeightStringCura = "Layer height: ";
	pos = strstr(buf, layerHeightStringCura);
	if (pos != nullptr)
	{
		pos += strlen(layerHeightStringCura);
		while (strchr(" \t=:", *pos))
		{
			++pos;
		}
		layerHeight = strtod(pos, nullptr);
		return true;
	}

	// Look for layer height as generated by S3D
	const char* layerHeightStringS3D = "layerHeight,";
	pos = strstr(buf, layerHeightStringS3D);
	if (pos != nullptr)
	{
		pos += strlen(layerHeightStringS3D);
		layerHeight = strtod(pos, nullptr);
		return true;
	}

	// Look for layer height as generated by KISSlicer
	const char* layerHeightStringKisslicer = "layer_thickness_mm = ";
	pos = strstr(buf, layerHeightStringKisslicer);
	if (pos != nullptr)
	{
		pos += strlen(layerHeightStringKisslicer);
		layerHeight = strtod(pos, nullptr);
		return true;
	}

	return false;
}

// Scan the buffer for the filament used. The buffer is null-terminated.
// Returns the number of filaments found.
unsigned int PrintMonitor::FindFilamentUsed(const char* buf, size_t len, float *filamentUsed, unsigned int maxFilaments) const
{
	unsigned int filamentsFound = 0;

	// Look for filament usage as generated by Slic3r and Cura
	const char* filamentUsedStr = "ilament used";		// comment string used by slic3r and Cura, followed by filament used and "mm"
	const char* p = buf;
	while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentUsedStr)) != nullptr)
	{
		p += strlen(filamentUsedStr);
		while(strchr(" :=\t", *p) != nullptr)
		{
			++p;	// this allows for " = " from default slic3r comment and ": " from default Cura comment
		}
		if (isDigit(*p))
		{
			char* q;
			filamentUsed[filamentsFound] = strtod(p, &q);
			if (*q == 'm' && *(q + 1) != 'm')
			{
				filamentUsed[filamentsFound] *= 1000.0;		// Cura outputs filament used in metres not mm
			}
			++filamentsFound;
		}
	}

	// Look for filament usage as generated by S3D
	if (!filamentsFound)
	{
		const char *filamentLengthStr = "ilament length:";	// comment string used by S3D
		p = buf;
		while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentLengthStr)) != nullptr)
		{
			p += strlen(filamentLengthStr);
			while(strchr(" :=\t", *p) != nullptr)
			{
				++p;	// this allows for " = " from default slic3r comment and ": " from default Cura comment
			}
			if (isDigit(*p))
			{
				char* q;
				filamentUsed[filamentsFound] = strtod(p, &q); // S3D reports filament usage in mm, no conversion needed
				++filamentsFound;
			}
		}
	}

	// Special case: KISSlicer only generates the filament volume, so we need to calculate the length from it
	if (!filamentsFound)
	{
		const char *filamentVolumeStr = "; Estimated Build Volume: ";
		p = strstr(buf, filamentVolumeStr);
		if (p != nullptr)
		{
			float filamentCMM = strtod(p + strlen(filamentVolumeStr), nullptr) * 1000.0;
			filamentUsed[filamentsFound++] = filamentCMM / (PI * (platform->GetFilamentWidth() / 2.0) * (platform->GetFilamentWidth() / 2.0));
		}
	}

	return filamentsFound;
}

// This returns the amount of time the machine has printed without interruptions (i.e. pauses)
float PrintMonitor::GetPrintDuration() const
{
	if (!isPrinting)
	{
		// Can't provide a valid print duration if we don't know when it started
		return 0.0;
	}

	float printDuration = platform->Time() - printStartTime - totalPauseTime;
	if (pauseStartTime != 0.0)
	{
		// Take into account how long the machine has been paused for the time estimation
		printDuration -= platform->Time() - pauseStartTime;
	}
	return printDuration;
}

// vim: ts=4:sw=4
