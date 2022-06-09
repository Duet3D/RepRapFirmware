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

#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Heating/Heat.h>
#include <Movement/Move.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>

ReadWriteLock PrintMonitor::printMonitorLock;

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(PrintMonitor, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(PrintMonitor, _condition,__VA_ARGS__)

const ObjectModelArrayDescriptor PrintMonitor::filamentArrayDescriptor =
{
	&printMonitorLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t
			{ return ((const PrintMonitor*)self)->printingFileInfo.numFilaments; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
			{ return  ExpressionValue(((const PrintMonitor*)self)->printingFileInfo.filamentNeeded[context.GetIndex(0)], 1); }
};

const ObjectModelArrayDescriptor PrintMonitor::thumbnailArrayDescriptor =
{
	&printMonitorLock,
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t
			{
				size_t count = 0;
				while (count < MaxThumbnails && ((const PrintMonitor*)self)->printingFileInfo.thumbnails[count].IsValid())
				{
					count++;
				}
				return count;
			},
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(self, 2); }
};

constexpr ObjectModelTableEntry PrintMonitor::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Job members
#if TRACK_OBJECT_NAMES
	{ "build",				OBJECT_MODEL_FUNC_IF(self->IsPrinting(), self->gCodes.GetBuildObjects(), 0), 										ObjectModelEntryFlags::live },
#endif
	{ "duration",			OBJECT_MODEL_FUNC_IF(self->IsPrinting(), self->GetPrintOrSimulatedDuration()), 										ObjectModelEntryFlags::live },
	{ "file",				OBJECT_MODEL_FUNC(self, 1),							 																ObjectModelEntryFlags::none },
	{ "filePosition",		OBJECT_MODEL_FUNC(self->gCodes.GetFilePosition()),																	ObjectModelEntryFlags::live },
	{ "firstLayerDuration", OBJECT_MODEL_FUNC_NOSELF(nullptr), 																					ObjectModelEntryFlags::obsolete },
	{ "lastDuration",		OBJECT_MODEL_FUNC_IF(!self->IsPrinting(), (int32_t)self->gCodes.GetLastDuration()), 								ObjectModelEntryFlags::none },
	{ "lastFileName",		OBJECT_MODEL_FUNC_IF(!self->filenameBeingPrinted.IsEmpty() && !self->IsPrinting(), self->filenameBeingPrinted.c_str()), ObjectModelEntryFlags::none },
	// TODO Add enum about the last file print here (to replace lastFileAborted, lastFileCancelled, lastFileSimulated)
	{ "layer",				OBJECT_MODEL_FUNC_IF(self->IsPrinting() && self->currentLayer != 0, (int32_t)self->currentLayer), 					ObjectModelEntryFlags::live },
	{ "layerTime",			OBJECT_MODEL_FUNC_IF(self->IsPrinting() && self->currentLayer != 0, self->GetCurrentLayerTime(), 1), 				ObjectModelEntryFlags::live },
	{ "pauseDuration",		OBJECT_MODEL_FUNC_IF(self->IsPrinting(), lrintf(self->GetPauseDuration())),											ObjectModelEntryFlags::live },
	{ "rawExtrusion",		OBJECT_MODEL_FUNC_IF(self->IsPrinting(), ExpressionValue(self->gCodes.GetTotalRawExtrusion(), 1)),					ObjectModelEntryFlags::live },
	{ "timesLeft",			OBJECT_MODEL_FUNC(self, 3),							 																ObjectModelEntryFlags::live },
	{ "warmUpDuration",		OBJECT_MODEL_FUNC_IF(self->IsPrinting(), lrintf(self->GetWarmUpDuration())),										ObjectModelEntryFlags::live },

	// 1. ParsedFileInfo members
	{ "filament",			OBJECT_MODEL_FUNC_NOSELF(&filamentArrayDescriptor),							 										ObjectModelEntryFlags::none },
	{ "fileName",			OBJECT_MODEL_FUNC_IF(self->IsPrinting(), self->filenameBeingPrinted.c_str()),										ObjectModelEntryFlags::none },
	{ "generatedBy",		OBJECT_MODEL_FUNC_IF(!self->printingFileInfo.generatedBy.IsEmpty(), self->printingFileInfo.generatedBy.c_str()),	ObjectModelEntryFlags::none },
	{ "height",				OBJECT_MODEL_FUNC(self->printingFileInfo.objectHeight, 2), 															ObjectModelEntryFlags::none },
	{ "lastModified",		OBJECT_MODEL_FUNC(DateTime(self->printingFileInfo.lastModifiedTime)), 												ObjectModelEntryFlags::none },
	{ "layerHeight",		OBJECT_MODEL_FUNC(self->printingFileInfo.layerHeight, 2), 															ObjectModelEntryFlags::none },
	{ "numLayers",			OBJECT_MODEL_FUNC((uint32_t)self->printingFileInfo.numLayers), 														ObjectModelEntryFlags::none },
	{ "printTime",			OBJECT_MODEL_FUNC_IF(self->printingFileInfo.printTime != 0, (int32_t)self->printingFileInfo.printTime), 			ObjectModelEntryFlags::none },
	{ "simulatedTime",		OBJECT_MODEL_FUNC_IF(self->printingFileInfo.simulatedTime != 0, (int32_t)self->printingFileInfo.simulatedTime), 	ObjectModelEntryFlags::none },
	{ "size",				OBJECT_MODEL_FUNC(self->printingFileInfo.fileSize),																	ObjectModelEntryFlags::none },
	{ "thumbnails",			OBJECT_MODEL_FUNC_NOSELF(&thumbnailArrayDescriptor),																ObjectModelEntryFlags::none },

	// 2. ParsedFileInfo.thumbnails[] members
	{ "format",				OBJECT_MODEL_FUNC(self->printingFileInfo.thumbnails[context.GetLastIndex()].format.ToString()),						ObjectModelEntryFlags::none },
	{ "height",				OBJECT_MODEL_FUNC((int32_t)self->printingFileInfo.thumbnails[context.GetLastIndex()].height),						ObjectModelEntryFlags::none },
	{ "offset",				OBJECT_MODEL_FUNC(self->printingFileInfo.thumbnails[context.GetLastIndex()].offset),								ObjectModelEntryFlags::none },
	{ "size",				OBJECT_MODEL_FUNC(self->printingFileInfo.thumbnails[context.GetLastIndex()].size),									ObjectModelEntryFlags::none },
	{ "width",				OBJECT_MODEL_FUNC((int32_t)self->printingFileInfo.thumbnails[context.GetLastIndex()].width),						ObjectModelEntryFlags::none },

	// 3. TimesLeft members
	{ "filament",			OBJECT_MODEL_FUNC(self->EstimateTimeLeftAsExpression(filamentBased)),												ObjectModelEntryFlags::live },
	{ "file",				OBJECT_MODEL_FUNC(self->EstimateTimeLeftAsExpression(fileBased)),													ObjectModelEntryFlags::live },
	{ "layer", 				OBJECT_MODEL_FUNC_NOSELF(nullptr), 																					ObjectModelEntryFlags::obsolete },
	{ "slicer",				OBJECT_MODEL_FUNC(self->EstimateTimeLeftAsExpression(slicerBased)),													ObjectModelEntryFlags::live },
};

constexpr uint8_t PrintMonitor::objectModelTableDescriptor[] = { 4, 12 + TRACK_OBJECT_NAMES, 11, 5, 4 };

DEFINE_GET_OBJECT_MODEL_TABLE(PrintMonitor)

int32_t PrintMonitor::GetPrintOrSimulatedDuration() const noexcept
{
	return lrintf((gCodes.IsSimulating()) ? gCodes.GetSimulationTime() + reprap.GetMove().GetSimulationTime() : GetPrintDuration());
}

#endif

PrintMonitor::PrintMonitor(Platform& p, GCodes& gc) noexcept : platform(p), gCodes(gc), isPrinting(false), heatingUp(false), paused(false), printingFileParsed(false)
{
}

void PrintMonitor::Init() noexcept
{
	Reset();
	lastUpdateTime = millis();
}

// This is called at various times including when a print starts. Don't reset slicerTimeLeft or totalFilamentNeeded.
void PrintMonitor::Reset() noexcept
{
	WriteLocker locker(printMonitorLock);

	heatingUp = paused = false;
	currentLayer = 0;
	printStartTime = pauseStartTime = lastSnapshotTime = lastLayerChangeTime = heatingStartedTime = whenSlicerTimeLeftSet = millis64();
	totalPauseTime = warmUpDuration = lastSnapshotNonPrintingTime = lastLayerChangeNonPrintingTime = 0;
	lastLayerDuration = 0;
	lastSnapshotFileFraction = lastSnapshotFilamentUsed = 0.0;
	fileProgressRate = filamentProgressRate = 0.0;
	reprap.JobUpdated();
}

void PrintMonitor::UpdatePrintingFileInfo() noexcept
{
	totalFilamentNeeded = printingFileInfo.filamentNeeded[0];
	for (size_t extruder = 1; extruder < printingFileInfo.numFilaments; extruder++)
	{
		totalFilamentNeeded += printingFileInfo.filamentNeeded[extruder];
	}
	slicerTimeLeft = printingFileInfo.printTime;
	printingFileParsed = true;
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
	{
		WriteLocker locker(printMonitorLock);
		filenameBeingPrinted.copy(filename);
		printingFileInfo = info;
		UpdatePrintingFileInfo();
	}
	reprap.JobUpdated();
}

GCodeResult PrintMonitor::ProcessM73(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (gb.Seen('R'))
	{
		SetSlicerTimeLeft(gb.GetFValue() * MinutesToSeconds);
	}
	// M73 without P Q R or S parameters reports print progress in some implementations, but we don't currently do that
	return GCodeResult::ok;
}

void PrintMonitor::SetSlicerTimeLeft(float seconds) noexcept
{
	slicerTimeLeft = seconds;
	whenSlicerTimeLeftSet = millis64();
}

void PrintMonitor::Spin() noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		if (!printingFileParsed)
		{
			return;
		}
	}
	else
#endif
	{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		// File information about the file being printed must be available before layer estimations can be made
		if (!filenameBeingPrinted.IsEmpty() && !printingFileParsed)
		{
			WriteLocker locker(printMonitorLock);
			printingFileParsed = (MassStorage::GetFileInfo(filenameBeingPrinted.c_str(), printingFileInfo, false) != GCodeResult::notFinished);
			if (!printingFileParsed)
			{
				return;
			}
			UpdatePrintingFileInfo();
		}
#else
		return;
#endif
	}

	// Otherwise collect some stats after a certain period of time
	const uint64_t now = millis64();
	if (isPrinting
#if SUPPORT_ROLAND
		&& !reprap.GetRoland()->Active()
#endif
		&& (uint32_t)now - lastUpdateTime > UpdateIntervalMillis)
	{
		if (gCodes.GetPauseState() != PauseState::notPaused)
		{
			if (!paused)
			{
				pauseStartTime = now;
				paused = true;
			}
		}
		else
		{
			if (paused)
			{
				const uint64_t pauseTime = now - pauseStartTime;
				totalPauseTime += pauseTime;
				whenSlicerTimeLeftSet += pauseTime;
				paused = false;
			}

			if (gCodes.IsHeatingUp())
			{
				if (!heatingUp)
				{
					heatingUp = true;
					heatingStartedTime = now;
				}
			}
			else
			{
				if (heatingUp)
				{
					const uint64_t heatingTime = now - heatingStartedTime;
					warmUpDuration += heatingTime;
					whenSlicerTimeLeftSet += heatingTime;
					heatingUp = false;
				}

				const uint64_t totalNonPrintingTime = warmUpDuration + totalPauseTime;
				const uint32_t printTimeSinceLastSnapshot = (uint32_t)((now - lastSnapshotTime) - (totalNonPrintingTime - lastSnapshotNonPrintingTime));
				const uint32_t snaphotIntervalMillis = (gCodes.IsSimulating()) ? 1000 * SnapshotIntervalSecondsSimulating : 1000 * SnapshotIntervalSecondsPrinting;
				if (printTimeSinceLastSnapshot >= snaphotIntervalMillis)
				{
					// Take a new snapshot
					const float currentFraction = FractionOfFilePrinted();
					const float currentFilamentUsed = gCodes.GetTotalRawExtrusion();

					TaskCriticalSectionLocker lock;
					fileProgressRate = 1000.0 * (currentFraction - lastSnapshotFileFraction)/printTimeSinceLastSnapshot;
					filamentProgressRate = 1000.0 * (currentFilamentUsed - lastSnapshotFilamentUsed)/printTimeSinceLastSnapshot;
					lastSnapshotFileFraction = currentFraction;
					lastSnapshotFilamentUsed = currentFilamentUsed;
					lastSnapshotNonPrintingTime = totalNonPrintingTime;
					lastSnapshotTime = now;
				}
			}
		}
		lastUpdateTime = (uint32_t)now;
	}
}

// Return the warm-up time
float PrintMonitor::GetWarmUpDuration() const noexcept
{
	return (float)((heatingUp) ? warmUpDuration + (millis64() - heatingStartedTime) : warmUpDuration) * MillisToSeconds;
}

// Return the total pause time
float PrintMonitor::GetPauseDuration() const noexcept
{
	return (float)((paused) ? totalPauseTime + (millis64() - pauseStartTime) : totalPauseTime) * MillisToSeconds;
}

// Notifies this class that a file has been set for printing
void PrintMonitor::StartingPrint(const char* filename) noexcept
{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	WriteLocker locker(printMonitorLock);
	MassStorage::CombineName(filenameBeingPrinted.GetRef(), Platform::GetGCodeDir(), filename);
# if HAS_SBC_INTERFACE
	if (!reprap.UsingSbcInterface())
# endif
	{
		printingFileParsed = false;
		if (MassStorage::GetFileInfo(filenameBeingPrinted.c_str(), printingFileInfo, false) != GCodeResult::notFinished)
		{
			UpdatePrintingFileInfo();
		}
		else
		{
			totalFilamentNeeded = 0.0;
			slicerTimeLeft = 0.0;
		}
	}
	reprap.JobUpdated();
#endif
}

// Tell this class that the file set for printing is now actually processed
void PrintMonitor::StartedPrint() noexcept
{
	isPrinting = true;
	SetLayerNumber(0);
	Reset();
}

void PrintMonitor::StoppedPrint() noexcept
{
	isPrinting = printingFileParsed = false;
	Reset();
}

// Set the current layer number as given in a comment
// The Z move to the new layer probably hasn't been done yet, so just store the layer number.
void PrintMonitor::SetLayerNumber(uint32_t layerNumber) noexcept
{
	if (currentLayer != layerNumber)
	{
		currentLayer = layerNumber;
		lastLayerChangeTime = millis64();
		lastLayerChangeNonPrintingTime = GetWarmUpDuration() + GetPauseDuration();
	}
}

// Set the printing height of the new layer
// The Z move to the new layer probably hasn't been done yet, so just store the layer print height.
void PrintMonitor::SetLayerZ(float layerZ) noexcept
{
	// Currently we don't use the layerZ value
}

// Report that a new layer has started
void PrintMonitor::LayerChange() noexcept
{
	++currentLayer;
	lastLayerChangeTime = millis64();
	lastLayerChangeNonPrintingTime = GetWarmUpDuration() + GetPauseDuration();
}

float PrintMonitor::FractionOfFilePrinted() const noexcept
{
	ReadLocker locker(printMonitorLock);

	if (!printingFileInfo.isValid || printingFileInfo.fileSize == 0)
	{
		return -1.0;
	}
	return (float)gCodes.GetFilePosition() / (float)printingFileInfo.fileSize;
}

// Estimate the print time left in seconds on a preset estimation method
float PrintMonitor::EstimateTimeLeft(PrintEstimationMethod method) const noexcept
{
	ReadLocker locker(printMonitorLock);

	// We can't provide an estimation if we don't have any information about the file
	if (!printingFileParsed || !isPrinting)
	{
		return 0.0;
	}

	switch (method)
	{
		case fileBased:
			if (lastSnapshotTime != printStartTime && fileProgressRate > 0.0)
			{
				return (1.0 - FractionOfFilePrinted())/fileProgressRate;
			}
			break;

		case filamentBased:
			if (lastSnapshotTime != printStartTime && filamentProgressRate > 0.0)
			{
				// Sum up the filament usage and the filament needed
				const float extrRawTotal = gCodes.GetTotalRawExtrusion();

				// If we have a reasonable amount of filament extruded, calculate estimated times left
				if (totalFilamentNeeded > 0.0 && extrRawTotal > totalFilamentNeeded * MinFilamentUsageForEstimation)
				{
					// Do we have more total filament extruded than reported by the file
					if (extrRawTotal >= totalFilamentNeeded)
					{
						// Yes - assume the print has almost finished
						return 1.0;
					}

					return (totalFilamentNeeded - extrRawTotal)/filamentProgressRate;
				}
			}
			break;

		case slicerBased:
			if (slicerTimeLeft > 0.0 && !gCodes.IsSimulating())				// don't report slicer time if we are simulating
			{
				const int64_t now = millis64();
				int64_t adjustment = (int64_t)(now - whenSlicerTimeLeftSet);			// add the time since we stored the slicer time left
				if (heatingUp)
				{
					adjustment -= (int64_t)(now - heatingStartedTime);					// subtract any recent heating time
				}
				if (paused)
				{
					adjustment -= (int64_t)(now - pauseStartTime);						// subtract any current pause time
				}
				return max<float>(1.0, slicerTimeLeft - adjustment * MillisToSeconds);
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

	const uint64_t now = millis64();
	const uint64_t pauseTime = (paused) ? totalPauseTime + (now - pauseStartTime) : totalPauseTime;
	return (float)(now - printStartTime - pauseTime) * MillisToSeconds;
}

// Get the time since starting the current layer in seconds
float PrintMonitor::GetCurrentLayerTime() const noexcept
{
	return (float)(millis64() - lastLayerChangeTime + lastLayerChangeNonPrintingTime - (GetWarmUpDuration() + GetPauseDuration())) * MillisToSeconds;
}

// End
