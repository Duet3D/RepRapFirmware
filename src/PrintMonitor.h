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

#ifndef PRINTMONITOR_H
#define PRINTMONITOR_H

#include "RepRapFirmware.h"
#include "Storage/FileInfoParser.h"	// for struct GCodeFileInfo

const float LAYER_HEIGHT_TOLERANCE = 0.015;			// Tolerance for comparing two Z heights (in mm)

const size_t MAX_LAYER_SAMPLES = 5;					// Number of layer samples for end-time estimation (except for first layer)
const float ESTIMATION_MIN_FILAMENT_USAGE = 0.01;	// Minimum per cent of filament to be printed before the filament-based estimation returns values
const float ESTIMATION_MIN_FILE_USAGE = 0.001;		// Minimum per cent of the file to be processed before any file-based estimations are made
const float FIRST_LAYER_SPEED_FACTOR = 0.25;		// First layer speed factor compared to other layers (only for layer-based estimation)

const uint32_t PRINTMONITOR_UPDATE_INTERVAL = 200;	// Update interval in milliseconds

enum PrintEstimationMethod
{
	filamentBased,
	fileBased,
	layerBased
};

class PrintMonitor
{
	public:
		PrintMonitor(Platform& p, GCodes& gc);
		void Spin();
		void Init();

		bool IsPrinting() const;						// Is a file being printed?
		void StartingPrint(const char *filename);		// Called to indicate a file will be printed (see M23)
		void StartedPrint();							// Called whenever a new live print starts (see M24)
		void StoppedPrint();							// Called whenever a file print has stopped

		// Return an estimate in seconds based on a specific estimation method
		float EstimateTimeLeft(PrintEstimationMethod method) const;

		// Provide some information about the file being printed
		unsigned int GetCurrentLayer() const;
		float GetCurrentLayerTime() const;
		float GetPrintDuration() const;
		float GetWarmUpDuration() const;
		float GetFirstLayerDuration() const;
		float GetFirstLayerHeight() const;

		const char *GetPrintingFilename() const { return (isPrinting) ? filenameBeingPrinted.c_str() : nullptr; }
		bool GetPrintingFileInfo(GCodeFileInfo& info);

	private:
		Platform& platform;
		GCodes& gCodes;
		uint32_t lastUpdateTime;

		// Information/Events concerning the file being printed
		void FirstLayerComplete();
		void LayerComplete();

		bool isPrinting;
		bool heatingUp;
		uint64_t printStartTime;
		uint64_t heatingStartedTime;
		uint64_t pauseStartTime, totalPauseTime;

		unsigned int currentLayer;
		float warmUpDuration, firstLayerDuration;
		float firstLayerFilament, firstLayerProgress;
		float lastLayerChangeTime, lastLayerFilament, lastLayerZ;

		unsigned int numLayerSamples;
		float layerDurations[MAX_LAYER_SAMPLES];
		float filamentUsagePerLayer[MAX_LAYER_SAMPLES];
		float fileProgressPerLayer[MAX_LAYER_SAMPLES];
		float layerEstimatedTimeLeft;

		bool printingFileParsed;
		GCodeFileInfo printingFileInfo;
		String<MaxFilenameLength> filenameBeingPrinted;
};

inline bool PrintMonitor::IsPrinting() const { return isPrinting; }
inline unsigned int PrintMonitor::GetCurrentLayer() const { return currentLayer; }
inline float PrintMonitor::GetCurrentLayerTime() const { return (lastLayerChangeTime > 0.0) ? (GetPrintDuration() - lastLayerChangeTime) : 0.0; }
inline float PrintMonitor::GetFirstLayerHeight() const { return printingFileParsed ? printingFileInfo.firstLayerHeight : 0.0; }

#endif /* PRINTMONITOR_H */

// vim: ts=4:sw=4
