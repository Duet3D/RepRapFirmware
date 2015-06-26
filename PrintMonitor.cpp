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

PrintMonitor::PrintMonitor(Platform *p, GCodes *gc) : platform(p), gCodes(gc), fileInfoDetected(false), isPrinting(false),
			printStartTime(0.0), currentLayer(0), firstLayerDuration(0.0), firstLayerHeight(0.0),
			firstLayerFilament(0.0), firstLayerProgress(0.0), warmUpDuration(0.0), layerEstimatedTimeLeft(0.0),
			lastLayerTime(0.0), lastLayerFilament(0.0), numLayerSamples(0)
{
}

void PrintMonitor::Init()
{
	longWait = platform->Time();
}

void PrintMonitor::Spin()
{
	if (gCodes->IsPausing() || gCodes->IsPaused() || gCodes->IsResuming())
	{
		// TODO: maybe incorporate pause durations in print estimations in the future?
		platform->ClassReport(longWait);
		return;
	}

	if (IsPrinting())
	{
		// May have just started a print, see if we're heating up
		if (warmUpDuration == 0.0)
		{
			// When a new print starts, the total (raw) extruder positions are zeroed
			float totalRawFilament = 0.0;
			for(size_t extruder=0; extruder < DRIVES - AXES; extruder++)
			{
				totalRawFilament += gCodes->GetRawExtruderPosition(extruder);
			}

			// See if at least one heater is active and set
			bool heatersAtHighTemperature = false;
			for(size_t heater=E0_HEATER; heater<HEATERS; heater++)
			{
				if (reprap.GetHeat()->GetStatus(heater) == Heat::HS_active &&
					reprap.GetHeat()->GetActiveTemperature(heater) > TEMPERATURE_LOW_SO_DONT_CARE &&
					reprap.GetHeat()->HeaterAtSetTemperature(heater))
				{
					heatersAtHighTemperature = true;
					break;
				}
			}

			if (heatersAtHighTemperature && totalRawFilament != 0.0)
			{
				lastLayerTime = platform->Time();
				warmUpDuration = lastLayerTime - printStartTime;

				if (fileInfoDetected && currentFileInfo.layerHeight > 0.0) {
					currentLayer = 1;
				}
			}
		}
		// Looks like the print has started
		else if (currentLayer > 0)
		{
			float liveCoords[DRIVES + 1];
			reprap.GetMove()->LiveCoordinates(liveCoords);

			// See if we can determine the first layer height (must be smaller than the nozzle diameter)
			if (firstLayerHeight == 0.0)
			{
				if (liveCoords[Z_AXIS] < platform->GetNozzleDiameter() * 1.1 && !gCodes->DoingFileMacro())
				{
					firstLayerHeight = liveCoords[Z_AXIS];
				}
			}
			// Then check if we've finished the first layer
			else if (firstLayerDuration == 0.0)
			{
				if (liveCoords[Z_AXIS] > firstLayerHeight * 1.05) // allow some tolerance for transform operations
				{
					firstLayerFilament = 0.0;
					for(size_t extruder=0; extruder<DRIVES - AXES; extruder++)
					{
						firstLayerFilament += gCodes->GetRawExtruderPosition(extruder);
					}
					firstLayerDuration = platform->Time() - lastLayerTime;
					firstLayerProgress = gCodes->FractionOfFilePrinted();
				}
			}
			// We have enough values to estimate the following layer heights
			else if (currentFileInfo.objectHeight > 0.0)
			{
				unsigned int estimatedLayer = round((liveCoords[Z_AXIS] - firstLayerHeight) / currentFileInfo.layerHeight) + 1;
				if (estimatedLayer == currentLayer + 1) // on layer change
				{
					// Record untainted extruder positions for filament-based estimation
					float extrRawTotal = 0.0;
					for(size_t extruder=0; extruder < DRIVES - AXES; extruder++)
					{
						extrRawTotal += gCodes->GetRawExtruderPosition(extruder);
					}

					const float now = platform->Time();
					unsigned int remainingLayers;
					remainingLayers = round((currentFileInfo.objectHeight - firstLayerHeight) / currentFileInfo.layerHeight) + 1;
					remainingLayers -= currentLayer;

					if (currentLayer > 1)
					{
						// Record a new set
						if (numLayerSamples < MAX_LAYER_SAMPLES)
						{
							layerDurations[numLayerSamples] = now - lastLayerTime;
							if (!numLayerSamples)
							{
								filamentUsagePerLayer[numLayerSamples] = extrRawTotal - firstLayerFilament;
							}
							else
							{
								filamentUsagePerLayer[numLayerSamples] = extrRawTotal - lastLayerFilament;
							}
							fileProgressPerLayer[numLayerSamples] = gCodes->FractionOfFilePrinted();
							numLayerSamples++;
						}
						else
						{
							for(size_t i=1; i<MAX_LAYER_SAMPLES; i++)
							{
								layerDurations[i - 1] = layerDurations[i];
								filamentUsagePerLayer[i - 1] = filamentUsagePerLayer[i];
								fileProgressPerLayer[i - 1] = fileProgressPerLayer[i];
							}

							layerDurations[MAX_LAYER_SAMPLES - 1] = now - lastLayerTime;
							filamentUsagePerLayer[MAX_LAYER_SAMPLES - 1] = extrRawTotal - lastLayerFilament;
							fileProgressPerLayer[MAX_LAYER_SAMPLES - 1] = gCodes->FractionOfFilePrinted();
						}
					}

					// Update layer-based estimation times
					float avgLayerTime, avgLayerDelta = 0.0;
					if (numLayerSamples)
					{
						avgLayerTime = 0.0;
						for(size_t layer=0; layer<numLayerSamples; layer++)
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
					currentLayer = estimatedLayer;
					lastLayerTime = now;
					lastLayerFilament = extrRawTotal;
				}
			}
		}
	}
	platform->ClassReport(longWait);
}

void PrintMonitor::StartingPrint(const char* filename)
{
	fileInfoDetected = GetFileInfo(platform->GetGCodeDir(), filename, currentFileInfo);
	strncpy(fileBeingPrinted, filename, ARRAY_SIZE(fileBeingPrinted));
	fileBeingPrinted[ARRAY_UPB(fileBeingPrinted)] = 0;
}

void PrintMonitor::StartedPrint()
{
	isPrinting = true;
	printStartTime = platform->Time();
}

void PrintMonitor::StoppedPrint()
{
	isPrinting = false;
	currentLayer = numLayerSamples = 0;
	firstLayerDuration = firstLayerHeight = firstLayerFilament = firstLayerProgress = 0.0;
	layerEstimatedTimeLeft = printStartTime = warmUpDuration = 0.0;
	lastLayerTime = lastLayerFilament = 0.0;
}

bool PrintMonitor::GetFileInfo(const char *directory, const char *fileName, GcodeFileInfo& info) const
{
	if (reprap.GetPlatform()->GetMassStorage()->PathExists(directory, fileName))
	{
		// Webserver can use this method to determine if a file was passed or not
		return false;
	}

	FileStore *f = reprap.GetPlatform()->GetFileStore(directory, fileName, false);
	if (f != NULL)
	{
		// Try to find the object height by looking for the last G1 Zxxx command in the file
		info.fileSize = f->Length();
		info.objectHeight = 0.0;
		info.layerHeight = 0.0;
		info.numFilaments = 0;
		info.generatedBy[0] = 0;
		for(size_t extr=0; extr<DRIVES - AXES; extr++)
		{
			info.filamentNeeded[extr] = 0.0;
		}

		if (info.fileSize != 0 && (StringEndsWith(fileName, ".gcode") || StringEndsWith(fileName, ".g") || StringEndsWith(fileName, ".gco") || StringEndsWith(fileName, ".gc")))
		{
			const size_t readSize = 512;					// read 512 bytes at a time (1K doesn't seem to work when we read from the end)
			const size_t overlap = 100;
			char buf[readSize + overlap + 1];				// need the +1 so we can add a null terminator

			bool foundLayerHeight = false;
			unsigned int filamentsFound = 0, nFilaments;
			float filaments[DRIVES - AXES];

			// Get slic3r settings by reading from the start of the file. We only read the first 4K or so, everything we are looking for should be there.
			for(uint8_t i=0; i<8; i++)
			{
				size_t sizeToRead = (size_t)min<unsigned long>(info.fileSize, readSize + overlap);
				int nbytes = f->Read(buf, sizeToRead);
				if (nbytes != (int)sizeToRead)
				{
					break;									// read failed so give up
				}
				else
				{
					buf[sizeToRead] = 0;

					// Search for filament usage (Cura puts it at the beginning of a G-code file)
					if (!filamentsFound)
					{
						nFilaments = FindFilamentUsed(buf, sizeToRead, filaments, DRIVES - AXES);
						if (nFilaments != 0 && nFilaments >= filamentsFound)
						{
							filamentsFound = nFilaments;
							for (unsigned int i = 0; i < filamentsFound; ++i)
							{
								info.filamentNeeded[i] = filaments[i];
							}
						}
					}

					// Look for layer height
					if (!foundLayerHeight)
					{
						foundLayerHeight = FindLayerHeight(buf, sizeToRead, info.layerHeight);
					}

					// Look for slicer program
					if (!info.generatedBy[0])
					{
						// Slic3r and S3D
						const char* generatedByString = "generated by ";
						char* pos = strstr(buf, generatedByString);
						size_t generatedByLength = ARRAY_SIZE(info.generatedBy);
						if (pos != NULL)
						{
							pos += strlen(generatedByString);
							size_t i = 0;
							while (i < ARRAY_SIZE(info.generatedBy) - 1 && *pos >= ' ')
							{
								char c = *pos++;
								if (c == '"' || c == '\\')
								{
									// Need to escape the quote-mark for JSON
									if (i > ARRAY_SIZE(info.generatedBy) - 3)
									{
										break;
									}
									info.generatedBy[i++] = '\\';
								}
								info.generatedBy[i++] = c;
							}
							info.generatedBy[i] = 0;
						}

						// Cura
						const char* slicedAtString = ";Sliced at: ";
						pos = strstr(buf, slicedAtString);
						if (pos != NULL)
						{
							pos += strlen(slicedAtString);
							strcpy(info.generatedBy, "Cura at ");
							size_t i = 8;
							while (i < ARRAY_SIZE(info.generatedBy) - 1 && *pos >= ' ')
							{
								char c = *pos++;
								if (c == '"' || c == '\\')
								{
									// Need to escape the quote-mark for JSON
									if (i > ARRAY_SIZE(info.generatedBy) - 3)
									{
										break;
									}
									info.generatedBy[i++] = '\\';
								}
								info.generatedBy[i++] = c;
							}
							info.generatedBy[i] = 0;
						}

						// KISSlicer
						const char* kisslicerStart = "; KISSlicer";
						if (StringStartsWith(buf, kisslicerStart))
						{
							size_t stringLength = 0;
							for(size_t i=2; i<ARRAY_UPB(info.generatedBy); i++)
							{
								if (buf[i] == '\r' || buf[i] == '\n')
								{
									break;
								}

								info.generatedBy[stringLength++] = buf[i];
							}
							info.generatedBy[stringLength] = 0;
						}
					}
				}

				// Have we collected everything?
				if (filamentsFound && foundLayerHeight && info.generatedBy[0])
				{
					break;
				}
			}

			// Now get the object height and filament used by reading the end of the file
			{
				size_t sizeToRead;
				if (info.fileSize <= readSize + overlap)
				{
					sizeToRead = info.fileSize;						// read the whole file in one go
				}
				else
				{
					sizeToRead = info.fileSize % readSize;
					if (sizeToRead <= overlap)
					{
						sizeToRead += readSize;
					}
				}
				unsigned long seekPos = info.fileSize - sizeToRead;	// read on a 512b boundary
				size_t sizeToScan = sizeToRead;
				for (;;)
				{
					if (!f->Seek(seekPos))
					{
						break;
					}
					int nbytes = f->Read(buf, sizeToRead);
					if (nbytes != (int)sizeToRead)
					{
						break;									// read failed so give up
					}

					// Search for filament used
					if (!filamentsFound)
					{
						nFilaments = FindFilamentUsed(buf, sizeToScan, filaments, DRIVES - AXES);
						if (nFilaments != 0 && nFilaments >= filamentsFound)
						{
							filamentsFound = nFilaments;
							for (unsigned int i = 0; i < filamentsFound; ++i)
							{
								info.filamentNeeded[i] = filaments[i];
							}
						}
					}

					// Search for layer height
					if (!foundLayerHeight)
					{
						foundLayerHeight = FindLayerHeight(buf, sizeToScan, info.layerHeight);
					}

					// Search for object height
					if (FindHeight(buf, sizeToScan, info.objectHeight))
					{
						break;		// quit if found height
					}

					if (seekPos == 0 || info.fileSize - seekPos >= 128000uL)	// scan up to about the last 128K of the file (32K wasn't enough)
					{
						break;
					}
					seekPos -= readSize;
					sizeToRead = readSize;
					sizeToScan = readSize + overlap;
					memcpy(buf + sizeToRead, buf, overlap);
				}
				info.numFilaments = filamentsFound;
			}
		}
		f->Close();
//debugPrintf("Set height %f and filament %f\n", height, filamentUsed);
		return true;
	}
	return false;
}

void PrintMonitor::GetFileInfoResponse(StringRef& response, const char* filename) const
{
	// Poll file info for a specific file
	if (filename != NULL)
	{
		GcodeFileInfo info;
		bool found = GetFileInfo("0:/", filename, info);
		if (found)
		{
			response.printf("{\"err\":0,\"size\":%lu,\"height\":%.2f,\"layerHeight\":%.2f,\"filament\":",
							info.fileSize, info.objectHeight, info.layerHeight);
			char ch = '[';
			if (info.numFilaments == 0)
			{
				response.catf("%c", ch);
			}
			else
			{
				for (unsigned int i = 0; i < info.numFilaments; ++i)
				{
					response.catf("%c%.1f", ch, info.filamentNeeded[i]);
					ch = ',';
				}
			}
			response.catf("],\"generatedBy\":\"%s\"}", info.generatedBy);
		}
		else
		{
			response.copy("{\"err\":1}");
		}
	}
	else if (IsPrinting() && fileInfoDetected)
	{
		// Poll file info about a file currently being printed
		response.printf("{\"err\":0,\"size\":%lu,\"height\":%.2f,\"layerHeight\":%.2f,\"filament\":",
						currentFileInfo.fileSize, currentFileInfo.objectHeight, currentFileInfo.layerHeight);
		char ch = '[';
		if (currentFileInfo.numFilaments == 0)
		{
			response.catf("%c", ch);
		}
		else
		{
			for (unsigned int i = 0; i < currentFileInfo.numFilaments; ++i)
			{
				response.catf("%c%.1f", ch, currentFileInfo.filamentNeeded[i]);
				ch = ',';
			}
		}
		response.catf("],\"generatedBy\":\"%s\",\"printDuration\":%d,\"fileName\":\"%s\"}",
				currentFileInfo.generatedBy, (int)((platform->Time() - printStartTime) * 1000.0), fileBeingPrinted);
	}
	else
	{
		response.copy("{\"err\":1}");
	}
}

float PrintMonitor::EstimateTimeLeft(PrintEstimationMethod method) const
{
	// We can't provide an estimation if we're not printing (yet)
	if (!IsPrinting() || (fileInfoDetected && currentFileInfo.numFilaments != 0 && warmUpDuration == 0.0))
	{
		return 0.0;
	}

	// Take into account the first layer time only if we haven't got any other samples
	float realPrintDuration = (platform->Time() - printStartTime) - warmUpDuration;
	if (numLayerSamples != 0)
	{
		realPrintDuration -= firstLayerDuration;
	}

	// Actual estimations
	switch (method)
	{
		case fileBased:
		{
			// Provide rough estimation only if we haven't collected any layer samples
			float fractionPrinted = gCodes->FractionOfFilePrinted();
			if (numLayerSamples == 0 || !fileInfoDetected || currentFileInfo.objectHeight == 0.0)
			{
				return realPrintDuration * (1.0 / fractionPrinted) - realPrintDuration;
			}

			// Each layer takes time to achieve more file progress, so take an average over our samples
			float avgSecondsByProgress = 0.0, lastLayerProgress = 0.0;
			for (unsigned int layer=0; layer<numLayerSamples; layer++)
			{
				avgSecondsByProgress += layerDurations[layer] / (fileProgressPerLayer[layer] - lastLayerProgress);
				lastLayerProgress = fileProgressPerLayer[layer];
			}
			avgSecondsByProgress /= numLayerSamples;

			// Then we know how many seconds it takes to finish 1% and we know how much file progress is left
			return avgSecondsByProgress * (1.0 - fractionPrinted);
		}

		case filamentBased:
		{
			// Need some file information, otherwise this method won't work
			if (!fileInfoDetected || currentFileInfo.numFilaments == 0)
			{
				return 0.0;
			}

			// Sum up the filament usage and the filament needed
			float totalFilamentNeeded = 0.0;
			float extrRawTotal = 0.0;
			for (size_t extruder=0; extruder < DRIVES - AXES; extruder++)
			{
				totalFilamentNeeded += currentFileInfo.filamentNeeded[extruder];
				extrRawTotal += gCodes->GetRawExtruderPosition(extruder);
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
				if (numLayerSamples != 0)
				{
					filamentRate = 0.0;
					for (unsigned int i=0; i<numLayerSamples; i++)
					{
						filamentRate += filamentUsagePerLayer[i] / layerDurations[i];
					}
					filamentRate /= numLayerSamples;
				}
				else
				{
					filamentRate = (firstLayerDuration > 0.0) ? firstLayerFilament / firstLayerDuration : 0.0;
				}

				return (filamentRate > 0.0) ? (totalFilamentNeeded - extrRawTotal) / filamentRate : 0.0;
			}
			break;
		}

		case layerBased:
			if (layerEstimatedTimeLeft > 0.0)
			{
				float timeLeft = layerEstimatedTimeLeft - (platform->Time() - lastLayerTime);
				if (timeLeft > 0.0)
				{
					return timeLeft;
				}
			}
			break;
	}

	return 0.0;
}

// Get information for the specified file, or the currently printing file, in JSON format
// Get information for a file on the SD card
// Scan the buffer for a G1 Zxxx command. The buffer is null-terminated.
bool PrintMonitor::FindHeight(const char* buf, size_t len, float& height) const
{
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
				for(int j=i +3; j < len - 2; j++)
				{
					char c = buf[j];
					if (c < ' ')
					{
						// Skip all whitespaces...
						while (j < len - 2 && c <= ' ')
						{
							c = buf[++j];
						}
						// ...to make sure ";End" doesn't follow G0 .. Z#HEIGHT#
						if (zPos != 0 && (buf[j] != ';' || buf[j + 1] != 'E'))
						{
							//debugPrintf("Found at offset %u text: %.100s\n", zPos, &buf[zPos + 1]);
							height = strtod(&buf[zPos + 1], NULL);
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
			if (i < len - 32 && StringStartsWith(buf + i, kisslicerHeightString))
			{
				height = strtod(buf + i + strlen(kisslicerHeightString), NULL);
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
	char *pos = strstr(buf, layerHeightStringSlic3r);
	if (pos != NULL)
	{
		pos += strlen(layerHeightStringSlic3r);
		while (strchr(" \t=:", *pos))
		{
			++pos;
		}
		layerHeight = strtod(pos, NULL);
		return true;
	}

	// Look for layer height as generated by Cura
	const char* layerHeightStringCura = "Layer height: ";
	pos = strstr(buf, layerHeightStringCura);
	if (pos != NULL)
	{
		pos += strlen(layerHeightStringCura);
		while (strchr(" \t=:", *pos))
		{
			++pos;
		}
		layerHeight = strtod(pos, NULL);
		return true;
	}

	// Look for layer height as generated by S3D
	const char* layerHeightStringS3D = "layerHeight,";
	pos = strstr(buf, layerHeightStringS3D);
	if (pos != NULL)
	{
		pos += strlen(layerHeightStringS3D);
		layerHeight = strtod(pos, NULL);
		return true;
	}

	// Look for layer height as generated by KISSlicer
	const char* layerHeightStringKisslicer = "layer_thickness_mm = ";
	pos = strstr(buf, layerHeightStringKisslicer);
	if (pos != NULL)
	{
		pos += strlen(layerHeightStringKisslicer);
		layerHeight = strtod(pos, NULL);
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
	while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentUsedStr)) != NULL)
	{
		p += strlen(filamentUsedStr);
		while(strchr(" :=\t", *p) != NULL)
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
		while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentLengthStr)) != NULL)
		{
			p += strlen(filamentLengthStr);
			while(strchr(" :=\t", *p) != NULL)
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
		if (p != NULL)
		{
			float filamentCMM = strtod(p + strlen(filamentVolumeStr), NULL) * 1000.0;
			filamentUsed[filamentsFound++] = filamentCMM / (PI * (platform->GetFilamentWidth() / 2.0) * (platform->GetFilamentWidth() / 2.0));
		}
	}

	return filamentsFound;
}
