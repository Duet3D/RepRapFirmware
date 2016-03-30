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

PrintMonitor::PrintMonitor(Platform *p, GCodes *gc) : platform(p), gCodes(gc), isPrinting(false),
	printStartTime(0), pauseStartTime(0.0), totalPauseTime(0.0), heatingUp(false), currentLayer(0), warmUpDuration(0.0),
	firstLayerDuration(0.0), firstLayerFilament(0.0), firstLayerProgress(0.0), lastLayerChangeTime(0.0),
	lastLayerFilament(0.0), lastLayerZ(0.0), numLayerSamples(0), layerEstimatedTimeLeft(0.0), parseState(notParsing),
	fileBeingParsed(nullptr), fileOverlapLength(0), printingFileParsed(false), accumulatedParseTime(0.0),
	accumulatedReadTime(0.0)
{
	filenameBeingPrinted[0] = 0;
}

void PrintMonitor::Init()
{
	longWait = platform->Time();
	lastUpdateTime = millis();
}

void PrintMonitor::Spin()
{
	// File information about the file being printed must be available before layer estimations can be made
	if (filenameBeingPrinted[0] != 0 && !printingFileParsed)
	{
		printingFileParsed = GetFileInfo(platform->GetGCodeDir(), filenameBeingPrinted, printingFileInfo);
		if (!printingFileParsed)
		{
			platform->ClassReport(longWait);
			return;
		}
	}

	// Don't do any updates if the print has been paused
	if (!gCodes->IsRunning())
	{
		if (pauseStartTime == 0.0)
		{
			pauseStartTime = platform->Time();
		}
		platform->ClassReport(longWait);
		return;
	}

	// Otherwise collect some stats after a certain period of time
	uint32_t now = millis();
	if (IsPrinting()
#if SUPPORT_ROLAND
		&& !reprap.GetRoland()->Active()
#endif
		&& now - lastUpdateTime > PRINTMONITOR_UPDATE_INTERVAL)
	{
		// Adjust the actual print time if the print was paused before
		if (pauseStartTime != 0.0)
		{
			totalPauseTime += platform->Time() - pauseStartTime;
			pauseStartTime = 0.0;
		}

		// Have we just started a print? See if we're heating up
		if (currentLayer == 0)
		{
			// Check if there are any active heaters
			bool nozzleAtHighTemperature = false;
			for(int heater = 0; heater < HEATERS; heater++)
			{
				if (reprap.GetHeat()->GetStatus(heater) == Heat::HS_active && reprap.GetHeat()->GetActiveTemperature(heater) > TEMPERATURE_LOW_SO_DONT_CARE)
				{
					heatingUp = true;

					// Check if this heater is assigned to a tool and if it has reached its set temperature yet
					if (reprap.IsHeaterAssignedToTool(heater))
					{
						if (!reprap.GetHeat()->HeaterAtSetTemperature(heater))
						{
							nozzleAtHighTemperature = false;
							break;
						}
						nozzleAtHighTemperature = true;
					}
				}
			}

			// Yes - do we have live movement?
			if (nozzleAtHighTemperature && !reprap.GetMove()->NoLiveMovement())
			{
				// Yes - we're actually starting the print
				WarmUpComplete();
				currentLayer = 1;
			}
		}
		// Print is in progress and filament is being extruded
		else if (!gCodes->DoingFileMacro() && reprap.GetMove()->IsExtruding())
		{
			float liveCoordinates[DRIVES];
			reprap.GetMove()->LiveCoordinates(liveCoordinates);

			// See if we need to determine the first layer height (usually smaller than the nozzle diameter)
			if (printingFileInfo.firstLayerHeight == 0.0)
			{
				if (liveCoordinates[Z_AXIS] < platform->GetNozzleDiameter() * 1.5)
				{
					// This shouldn't be needed because we parse the first layer height anyway, but it won't harm
					printingFileInfo.firstLayerHeight = liveCoordinates[Z_AXIS];
				}
			}
			// Check if we've finished the first layer
			else if (currentLayer == 1)
			{
				if (liveCoordinates[Z_AXIS] > printingFileInfo.firstLayerHeight + LAYER_HEIGHT_TOLERANCE)
				{
					FirstLayerComplete();
					currentLayer++;

					lastLayerZ = liveCoordinates[Z_AXIS];
					lastLayerChangeTime = GetPrintDuration();
				}
			}
			// Check for following layer changes
			else if (liveCoordinates[Z_AXIS] > lastLayerZ + LAYER_HEIGHT_TOLERANCE)
			{
				LayerComplete();
				currentLayer++;

				lastLayerZ = liveCoordinates[Z_AXIS];
				lastLayerChangeTime = GetPrintDuration();
			}
		}
		lastUpdateTime = now;
	}
	platform->ClassReport(longWait);
}

float PrintMonitor::GetWarmUpDuration() const
{
	if (currentLayer > 0)
	{
		return warmUpDuration;
	}
	return heatingUp ? GetPrintDuration() : 0.0;
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
	heatingUp = false;
	warmUpDuration = GetPrintDuration();
}

// Called when the first layer has been finished
void PrintMonitor::FirstLayerComplete()
{
	firstLayerFilament = RawFilamentExtruded();
	firstLayerDuration = GetPrintDuration() - warmUpDuration;
	firstLayerProgress = gCodes->FractionOfFilePrinted();

	// Update layer-based estimation time (if the object and layer heights are known)
	// This won't be very accurate, but at least something can be sent the web interface and to PanelDue
	if (printingFileInfo.objectHeight > 0.0 && printingFileInfo.layerHeight > 0.0)
	{
		unsigned int layersToPrint = round((printingFileInfo.objectHeight - printingFileInfo.firstLayerHeight) / printingFileInfo.layerHeight) + 1;
		layerEstimatedTimeLeft = firstLayerDuration * FIRST_LAYER_SPEED_FACTOR * (layersToPrint - 1);
	}
}

// This is called whenever a layer greater than 2 has been finished
void PrintMonitor::LayerComplete()
{
	// Record a new set of layer, filament and file stats
	const float extrRawTotal = RawFilamentExtruded();
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
	lastLayerFilament = extrRawTotal;

	// Update layer-based estimation time (if the object and layer heights are known)
	if (printingFileInfo.objectHeight > 0.0 && printingFileInfo.layerHeight > 0.0)
	{
		// Calculate the average layer time and include the first layer if possible
		float avgLayerTime = (numLayerSamples < MAX_LAYER_SAMPLES)
								? firstLayerDuration * FIRST_LAYER_SPEED_FACTOR
								: 0.0;
		for(size_t layer = 0; layer < numLayerSamples; layer++)
		{
			avgLayerTime += layerDurations[layer];
		}
		avgLayerTime /= (numLayerSamples < MAX_LAYER_SAMPLES) ? numLayerSamples + 1 : numLayerSamples;

		// Estimate the layer-based time left
		unsigned int totalLayers;
		totalLayers = round((printingFileInfo.objectHeight - printingFileInfo.firstLayerHeight) / printingFileInfo.layerHeight) + 1;
		if (currentLayer < totalLayers)
		{
			// Current layer is within reasonable boundaries, so an estimation can be made
			layerEstimatedTimeLeft = avgLayerTime * (totalLayers - currentLayer);
		}
		else
		{
			// Current layer is higher than the maximum number of layers. Assume the print has almost finished
			layerEstimatedTimeLeft = 0.1;
		}
	}
}

void PrintMonitor::StoppedPrint()
{
	isPrinting = heatingUp = printingFileParsed = false;
	currentLayer = numLayerSamples = 0;
	pauseStartTime = totalPauseTime = 0.0;
	firstLayerDuration = firstLayerFilament = firstLayerProgress = 0.0;
	layerEstimatedTimeLeft = printStartTime = warmUpDuration = 0.0;
	lastLayerChangeTime = lastLayerFilament = lastLayerZ = 0.0;
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
	if (filename != nullptr && filename[0] != 0)
	{
		GCodeFileInfo info;
		if (!GetFileInfo(FS_PREFIX, filename, info))
		{
			// This may take a few runs...
			return false;
		}

		if (info.isValid)
		{
			if (!OutputBuffer::Allocate(response))
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
			if (!OutputBuffer::Allocate(response))
			{
				// Should never happen
				return false;
			}

			response->copy("{\"err\":1}");
		}
	}
	else if (IsPrinting())
	{
		// If the file being printed hasn't been processed yet or if we
		// cannot write the response, try again later
		if (!printingFileParsed || !OutputBuffer::Allocate(response))
		{
			return false;
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
		if (!OutputBuffer::Allocate(response))
		{
			// Should never happen
			return false;
		}

		response->copy("{\"err\":1}");
	}
	return true;
}

// May be called from ISR
void PrintMonitor::StopParsing(const char *filename)
{
	if (parseState != notParsing && StringEquals(filenameBeingParsed, filename))
	{
		if (filenameBeingPrinted[0] != 0 && !printingFileParsed)
		{
			// If this is the file we're parsing for internal purposes, don't bother with this request
			return;
		}

		parseState = notParsing;
		fileBeingParsed->Close();
	}
}

// Estimate the print time left in seconds on a preset estimation method
float PrintMonitor::EstimateTimeLeft(PrintEstimationMethod method) const
{
	// We can't provide an estimation if we don't have any information about the file
	if (!printingFileParsed)
	{
		return 0.0;
	}

	// How long have we been printing continuously?
	float realPrintDuration = GetPrintDuration() - warmUpDuration;

	switch (method)
	{
		case fileBased:
		{
			// Can we provide an estimation at all?
			const float fractionPrinted = gCodes->FractionOfFilePrinted();
			if (fractionPrinted < ESTIMATION_MIN_FILE_USAGE || heatingUp)
			{
				// No, we haven't printed enough of the file yet. We can't provide an estimation at this moment
				return 0.0;
			}
			if (fractionPrinted == 1.0)
			{
				// No, but the file has been processed entirely. It won't take long until the print finishes
				return 0.1;
			}

			// See how long it takes per progress
			float duration, fractionPrintedInLayers;
			if (numLayerSamples == 0)
			{
				duration = firstLayerDuration;
				fractionPrintedInLayers = firstLayerProgress;
			}
			else if (numLayerSamples == 1)
			{
				duration = layerDurations[0];
				fractionPrintedInLayers = fileProgressPerLayer[0] - firstLayerProgress;
			}
			else if (numLayerSamples > 1)
			{
				duration = 0.0;
				for(size_t sample = 1; sample < numLayerSamples; sample++)
				{
					duration += layerDurations[sample];
				}
				fractionPrintedInLayers = fileProgressPerLayer[numLayerSamples - 1] - fileProgressPerLayer[0];
			}

			// Can we use these values?
			if (fractionPrintedInLayers < ESTIMATION_MIN_FILE_USAGE)
			{
				// No - only provide a rough estimation
				return max<float>(realPrintDuration * (1.0 / fractionPrinted) - realPrintDuration, 0.1);
			}

			// Yes...
			return max<float>(duration * (1.0 - fractionPrinted) / fractionPrintedInLayers, 0.1);
		}

		case filamentBased:
		{
			// Need some file information, otherwise this method won't work
			if (currentLayer == 0 || printingFileInfo.numFilaments == 0
#if SUPPORT_ROLAND
					|| reprap.GetRoland()->Active()
#endif
				)
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
				// Do we have more total filament extruded than reported by the file
				if (extrRawTotal >= totalFilamentNeeded)
				{
					// Yes - assume the print has almost finished
					return 0.1;
				}

				// Get filament usage per layer
				float filamentRate = 0.0;
				if (numLayerSamples > 0)
				{
					for(size_t i = 0; i < numLayerSamples; i++)
					{
						filamentRate += filamentUsagePerLayer[i] / layerDurations[i];
					}
					filamentRate /= numLayerSamples;
				}
				else if (firstLayerDuration > 0.0)
				{
					filamentRate = firstLayerFilament / firstLayerDuration;
				}

				// Can we provide a good estimation?
				if (filamentRate == 0.0)
				{
					// No - calculate time left based on the filament we have extruded so far
					return realPrintDuration * (totalFilamentNeeded - extrRawTotal) / extrRawTotal;
				}

				return (totalFilamentNeeded - extrRawTotal) / filamentRate;
			}
			break;
		}

		case layerBased:
			// Layer-based estimations are made after each layer change, only reflect this value
			if (layerEstimatedTimeLeft > 0.0)
			{
				float timeLeft = layerEstimatedTimeLeft - (GetPrintDuration() - lastLayerChangeTime);
				return (timeLeft > 0.0) ? timeLeft : 0.1;
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
	height = 0.0;

//debugPrintf("Scanning %u bytes starting %.100s\n", len, buf);
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
						float flHeight = strtod(&buf[i + 1], nullptr);
						if ((height == 0.0 || flHeight < height) && (flHeight <= platform->GetNozzleDiameter() * 3.0))
						{
							height = flHeight;				// Only report first Z height if it's somewhat reasonable
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

// Get the sum of extruded filament (in mm)
float PrintMonitor::RawFilamentExtruded() const
{
	return reprap.GetGCodes()->GetTotalRawExtrusion();
}

// Scan the buffer in reverse for a G1 Zxxx command. The buffer is null-terminated.
bool PrintMonitor::FindHeight(const char* buf, size_t len, float& height) const
{
	if (len < 5)
	{
		// Don't start if the buffer is not big enough
		return false;
	}

	//debugPrintf("Scanning %u bytes starting %.100s\n", len, buf);
	bool inComment, inRelativeMode = false;
	unsigned int zPos;
	for(size_t i = len - 5; i > 0; i--)
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
				for(size_t j = i + 3; j < len - 2; j++)
				{
					char c = buf[j];
					if (c == ';')
					{
						inComment = true;
					}
					else if (c == 'Z' && !inComment)
					{
						zPos = j;
					}
					else if (c == '\n')
					{
						if (zPos != 0)
						{
							// Check special case of this code ending with ";E" or "; E" - ignore such codes
							for(j = zPos + 1; j < len - 3; j++)
							{
								c = buf[j];
								if (c == ';' && (buf[j + 1] == 'E' || buf[j + 2] == 'E'))
								{
									zPos = 0;
									break;
								}
								if (c == '\n')
								{
									break;
								}
							}

							if (zPos != 0)
							{
								// Z position is valid - read it
								height = strtod(&buf[zPos + 1], nullptr);
								return true;
							}
						}
						break;
					}
				}
			}
		}
		// Special case: KISSlicer generates object height as a comment
		else
		{
			const char *kisslicerHeightString = "; END_LAYER_OBJECT z=";
			if (i + 32 < len && StringStartsWith(buf + i, kisslicerHeightString))
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
		// Take into account how long the machine has been paused
		printDuration -= platform->Time() - pauseStartTime;
	}
	return printDuration;
}

// vim: ts=4:sw=4
