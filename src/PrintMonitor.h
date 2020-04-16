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

#include <RepRapFirmware.h>
#include <GCodes/GCodeFileInfo.h>
#include <ObjectModel/ObjectModel.h>

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

class PrintMonitor INHERIT_OBJECT_MODEL
{
public:
	PrintMonitor(Platform& p, GCodes& gc) noexcept;
	void Spin() noexcept;
	void Init() noexcept;

	bool IsPrinting() const noexcept;						// Is a file being printed?
	void StartingPrint(const char *filename) noexcept;		// Called to indicate a file will be printed (see M23)
	void StartedPrint() noexcept;							// Called whenever a new live print starts (see M24)
	void StoppedPrint() noexcept;							// Called whenever a file print has stopped
	void SetLayerNumber(uint32_t layerNumber) noexcept;		// Set the current layer number
	void SetLayerZ(float layerZ) noexcept;					// Set the printing height of the new layer
	float FractionOfFilePrinted() const noexcept;			// Return the fraction printed (0..1)

	// Return an estimate in seconds based on a specific estimation method
	float EstimateTimeLeft(PrintEstimationMethod method) const noexcept;
#if SUPPORT_OBJECT_MODEL
	ExpressionValue EstimateTimeLeftAsExpression(PrintEstimationMethod method) const noexcept;
#endif

	// Provide some information about the file being printed
	unsigned int GetCurrentLayer() const noexcept;
	float GetCurrentLayerTime() const noexcept;
	float GetPrintDuration() const noexcept;
	float GetWarmUpDuration() const noexcept;
	float GetFirstLayerDuration() const noexcept;
	float GetFirstLayerHeight() const noexcept;

	const char *GetPrintingFilename() const noexcept { return (isPrinting) ? filenameBeingPrinted.c_str() : nullptr; }
	bool GetPrintingFileInfo(GCodeFileInfo& info) noexcept;
	void SetPrintingFileInfo(const char *filename, GCodeFileInfo& info) noexcept;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(filament)

private:
	Platform& platform;
	GCodes& gCodes;
	uint32_t lastUpdateTime;

	// Information/Events concerning the file being printed
	void FirstLayerComplete() noexcept;
	void LayerComplete() noexcept;
	void Reset() noexcept;

#if SUPPORT_OBJECT_MODEL
	int32_t GetPrintOrSimulatedDuration() const noexcept;
#endif

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

	unsigned int lastLayerNumberNotified;
	float lastLayerStartHeightNotified;

	bool printingFileParsed;
	GCodeFileInfo printingFileInfo;
	String<MaxFilenameLength> filenameBeingPrinted;
};

inline bool PrintMonitor::IsPrinting() const noexcept { return isPrinting; }
inline unsigned int PrintMonitor::GetCurrentLayer() const noexcept { return currentLayer; }

inline float PrintMonitor::GetCurrentLayerTime() const noexcept
{
	return (currentLayer == 0) ? 0.0
			: (currentLayer == 1) ? GetPrintDuration()
				: GetPrintDuration() - lastLayerChangeTime;
}

inline float PrintMonitor::GetFirstLayerHeight() const noexcept
{
	return printingFileParsed ? printingFileInfo.firstLayerHeight : 0.0;
}

#endif /* PRINTMONITOR_H */

// vim: ts=4:sw=4
