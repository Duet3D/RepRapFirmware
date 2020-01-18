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

#include "PrintMonitor.h"

#include "GCodes/GCodes.h"
#include "Heating/Heat.h"
#include "Movement/Move.h"
#include "Platform.h"
#include "RepRap.h"

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(PrintMonitor, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(PrintMonitor, _condition,__VA_ARGS__)

const ObjectModelArrayDescriptor PrintMonitor::filamentArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t
			{ return ((const PrintMonitor*)self)->printingFileInfo.numFilaments; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
			{ return  ExpressionValue(((const PrintMonitor*)self)->printingFileInfo.filamentNeeded[context.GetIndex(0)], 1); }
};

constexpr ObjectModelTableEntry PrintMonitor::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. PrintMonitor members
	{ "file",				OBJECT_MODEL_FUNC(self, 1),							 																ObjectModelEntryFlags::none },
	{ "lastFileName",		OBJECT_MODEL_FUNC_IF(!self->filenameBeingPrinted.IsEmpty(), self->filenameBeingPrinted.c_str()), 					ObjectModelEntryFlags::none },
	{ "layer",				OBJECT_MODEL_FUNC((int32_t)self->currentLayer), 																	ObjectModelEntryFlags::none },
	{ "timesLeft",			OBJECT_MODEL_FUNC(self, 2),							 																ObjectModelEntryFlags::none },

	// 1. ParsedFileInfo members
	{ "filament",			OBJECT_MODEL_FUNC_NOSELF(&filamentArrayDescriptor),							 										ObjectModelEntryFlags::none },
	{ "firstLayerHeight",	OBJECT_MODEL_FUNC(self->printingFileInfo.firstLayerHeight, 2), 														ObjectModelEntryFlags::none },
	{ "generatedBy",		OBJECT_MODEL_FUNC_IF(!self->printingFileInfo.generatedBy.IsEmpty(), self->printingFileInfo.generatedBy.c_str()),	ObjectModelEntryFlags::none },
	{ "height",				OBJECT_MODEL_FUNC(self->printingFileInfo.objectHeight, 2), 															ObjectModelEntryFlags::none },
	{ "lastModified",		OBJECT_MODEL_FUNC(DateTime(self->printingFileInfo.lastModifiedTime)), 												ObjectModelEntryFlags::none },
	{ "layerHeight",		OBJECT_MODEL_FUNC(self->printingFileInfo.layerHeight, 2), 																ObjectModelEntryFlags::none },
	{ "numLayers",			OBJECT_MODEL_FUNC((int32_t)self->printingFileInfo.GetNumLayers()), 													ObjectModelEntryFlags::none },
	{ "printTime",			OBJECT_MODEL_FUNC_IF(self->printingFileInfo.printTime != 0, (int32_t)self->printingFileInfo.printTime), 			ObjectModelEntryFlags::none },
	{ "simulatedTime",		OBJECT_MODEL_FUNC_IF(self->printingFileInfo.simulatedTime != 0, (int32_t)self->printingFileInfo.simulatedTime), 	ObjectModelEntryFlags::none },
	{ "size",				OBJECT_MODEL_FUNC((int32_t)self->printingFileInfo.fileSize),	/* note, using int32_t limits us to 2Gb */			ObjectModelEntryFlags::none },

	// 2. TimesLeft members
	{ "filament",			OBJECT_MODEL_FUNC(self->EstimateTimeLeftAsExpression(filamentBased)),												ObjectModelEntryFlags::none },
	{ "file",				OBJECT_MODEL_FUNC(self->EstimateTimeLeftAsExpression(fileBased)),													ObjectModelEntryFlags::none },
	{ "layer",				OBJECT_MODEL_FUNC(self->EstimateTimeLeftAsExpression(layerBased)),													ObjectModelEntryFlags::none },
};

constexpr uint8_t PrintMonitor::objectModelTableDescriptor[] = { 3, 4, 10, 3 };

DEFINE_GET_OBJECT_MODEL_TABLE(PrintMonitor)

#endif

PrintMonitor::PrintMonitor(Platform& p, GCodes& gc) noexcept : platform(p), gCodes(gc), isPrinting(false), heatingUp(false),
	printStartTime(0), pauseStartTime(0), totalPauseTime(0), currentLayer(0), warmUpDuration(0.0),
	firstLayerDuration(0.0), firstLayerFilament(0.0), firstLayerProgress(0.0), lastLayerChangeTime(0.0),
	lastLayerFilament(0.0), lastLayerZ(0.0), numLayerSamples(0), layerEstimatedTimeLeft(0.0), printingFileParsed(false)
{
}

void PrintMonitor::Init() noexcept
{
	lastUpdateTime = millis();
}

bool PrintMonitor::GetPrintingFileInfo(GCodeFileInfo& info) noexcept
{
	if (IsPrinting())
	{
		if (!printingFileParsed)
		{
			return false;					// not ready yet
		}
		info = printingFileInfo;
	}
	else
	{
		info.isValid = false;
	}
	return true;
}

void PrintMonitor::SetPrintingFileInfo(const char *filename, GCodeFileInfo &info) noexcept
{
	filenameBeingPrinted.copy(filename);
	printingFileInfo = info;
	printingFileParsed = true;
}

void PrintMonitor::Spin() noexcept
{
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		if (!printingFileParsed)
		{
			return;
		}
	}
	else
#endif
	{
#if HAS_MASS_STORAGE
		// File information about the file being printed must be available before layer estimations can be made
		if (!filenameBeingPrinted.IsEmpty() && !printingFileParsed)
		{
			printingFileParsed = MassStorage::GetFileInfo(filenameBeingPrinted.c_str(), printingFileInfo, false);
			if (!printingFileParsed)
			{
				return;
			}
		}
#else
		return;
#endif
	}

	// Don't do any updates if the print has been paused
	if (!gCodes.IsRunning())
	{
		if (pauseStartTime == 0)
		{
			pauseStartTime = millis64();
		}
		return;
	}

	// Otherwise collect some stats after a certain period of time
	const uint32_t now = millis();
	if (IsPrinting()
#if SUPPORT_ROLAND
		&& !reprap.GetRoland()->Active()
#endif
		&& now - lastUpdateTime > PRINTMONITOR_UPDATE_INTERVAL)
	{
		// Adjust the actual print time if the print was paused before
		if (pauseStartTime != 0)
		{
			totalPauseTime += millis64() - pauseStartTime;
			pauseStartTime = 0;
		}

		if (gCodes.IsHeatingUp())
		{
			if (!heatingUp)
			{
				heatingUp = true;
				heatingStartedTime = millis64();
			}
		}
		else
		{
			if (heatingUp)
			{
				heatingUp = false;
				warmUpDuration += (millis64() - heatingStartedTime) * MillisToSeconds;
			}

			float currentZ;
			if (gCodes.GetLastPrintingHeight(currentZ))
			{
				// Print is in progress and filament is being extruded
				if (currentLayer == 0)
				{
					currentLayer = 1;

					// See if we need to determine the first layer height (usually smaller than the nozzle diameter)
					if (printingFileInfo.firstLayerHeight == 0.0 && currentZ < platform.GetNozzleDiameter() * 1.5)
					{
						printingFileInfo.firstLayerHeight = currentZ;
					}
				}
				else if (currentLayer == 1)
				{
					// Check if we've finished the first layer
					if (currentZ > printingFileInfo.firstLayerHeight + LAYER_HEIGHT_TOLERANCE)
					{
						FirstLayerComplete();
						currentLayer++;

						lastLayerZ = currentZ;
						lastLayerChangeTime = GetPrintDuration();
					}
				}
				// Else check for following layer changes
				else if (currentZ > lastLayerZ + LAYER_HEIGHT_TOLERANCE)
				{
					LayerComplete();
					currentLayer++;

					// If we know the layer height, compute what the current layer height should be. This is to handle slicers that use a different layer height for support.
					lastLayerZ = (printingFileInfo.layerHeight > 0.0)
									? printingFileInfo.firstLayerHeight + (currentLayer - 1) * printingFileInfo.layerHeight
										: currentZ;
					lastLayerChangeTime = GetPrintDuration();
				}
			}
		}
		lastUpdateTime = now;
	}
}

// Return the first layer print time
float PrintMonitor::GetFirstLayerDuration() const noexcept
{
	return (firstLayerDuration > 0.0) ? firstLayerDuration : ((currentLayer > 0) ? GetPrintDuration() - warmUpDuration : 0.0);
}

// Return the warm-up time
float PrintMonitor::GetWarmUpDuration() const noexcept
{
	return (heatingUp) ? warmUpDuration + (millis64() - heatingStartedTime) * MillisToSeconds : warmUpDuration;
}

// Notifies this class that a file has been set for printing
void PrintMonitor::StartingPrint(const char* filename) noexcept
{
#if HAS_MASS_STORAGE
	MassStorage::CombineName(filenameBeingPrinted.GetRef(), platform.GetGCodeDir(), filename);
	printingFileParsed = MassStorage::GetFileInfo(filenameBeingPrinted.c_str(), printingFileInfo, false);
#endif
}

// Tell this class that the file set for printing is now actually processed
void PrintMonitor::StartedPrint() noexcept
{
	isPrinting = true;
	heatingUp = false;
	printStartTime = millis64();
	warmUpDuration = 0.0;
}

// Called when the first layer has been finished
void PrintMonitor::FirstLayerComplete() noexcept
{
	firstLayerFilament = gCodes.GetTotalRawExtrusion();
	firstLayerDuration = GetPrintDuration() - warmUpDuration;
	firstLayerProgress = FractionOfFilePrinted();

	// Update layer-based estimation time (if the object and layer heights are known)
	// This won't be very accurate, but at least something can be sent the web interface and to PanelDue
	if (printingFileInfo.layerHeight > 0.0 && printingFileInfo.objectHeight > printingFileInfo.layerHeight)
	{
		unsigned int layersToPrint = lrintf((printingFileInfo.objectHeight - printingFileInfo.firstLayerHeight) / printingFileInfo.layerHeight) + 1;
		layerEstimatedTimeLeft = firstLayerDuration * FIRST_LAYER_SPEED_FACTOR * (layersToPrint - 1);
	}
}

// This is called whenever a layer greater than 2 has been finished
void PrintMonitor::LayerComplete() noexcept
{
	// Record a new set of layer, filament and file stats
	const float extrRawTotal = gCodes.GetTotalRawExtrusion();
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
		fileProgressPerLayer[numLayerSamples] = FractionOfFilePrinted();
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
		fileProgressPerLayer[MAX_LAYER_SAMPLES - 1] = FractionOfFilePrinted();
	}
	lastLayerFilament = extrRawTotal;

	// Update layer-based estimation time (if the object and layer heights are known)
	if (printingFileInfo.layerHeight > 0.0 && printingFileInfo.objectHeight > printingFileInfo.layerHeight)
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
		const unsigned int totalLayers = lrintf((printingFileInfo.objectHeight - printingFileInfo.firstLayerHeight) / printingFileInfo.layerHeight) + 1;
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

void PrintMonitor::StoppedPrint() noexcept
{
	isPrinting = heatingUp = printingFileParsed = false;
	currentLayer = numLayerSamples = 0;
	pauseStartTime = totalPauseTime = 0;
	firstLayerDuration = firstLayerFilament = firstLayerProgress = 0.0;
	layerEstimatedTimeLeft = printStartTime = warmUpDuration = 0.0;
	lastLayerChangeTime = lastLayerFilament = lastLayerZ = 0.0;
}

float PrintMonitor::FractionOfFilePrinted() const noexcept
{
	if (!printingFileInfo.isValid || printingFileInfo.fileSize == 0)
	{
		return -1.0;
	}
	return (float)reprap.GetGCodes().GetFilePosition() / (float)printingFileInfo.fileSize;
}

// Estimate the print time left in seconds on a preset estimation method
float PrintMonitor::EstimateTimeLeft(PrintEstimationMethod method) const noexcept
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
			const float fractionPrinted = FractionOfFilePrinted();
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
			const float extrRawTotal = gCodes.GetTotalRawExtrusion();
			for (size_t extruder = 0; extruder < MaxExtruders; extruder++)
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

#if SUPPORT_OBJECT_MODEL

// Return the estimated time remaining if we have it, else null
ExpressionValue PrintMonitor::EstimateTimeLeftAsExpression(PrintEstimationMethod method) const noexcept
{
	const float time = EstimateTimeLeft(method);
	return (time > 0.0) ? ExpressionValue(lrintf(time)) : ExpressionValue(nullptr);
}

#endif

// This returns the amount of time the machine has printed without interruptions (i.e. pauses)
float PrintMonitor::GetPrintDuration() const noexcept
{
	if (!isPrinting)
	{
		// Can't provide a valid print duration if we don't know when it started
		return 0.0;
	}

	const uint64_t printDuration = ((pauseStartTime != 0) ? pauseStartTime : millis64()) - printStartTime - totalPauseTime;
	return (float)printDuration * 0.001;
}

// vim: ts=4:sw=4
