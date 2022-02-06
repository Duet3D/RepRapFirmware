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

enum PrintEstimationMethod
{
	filamentBased,
	fileBased,
	slicerBased
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
	void LayerChange() noexcept;							// Report that a new layer has started
	float FractionOfFilePrinted() const noexcept;			// Return the fraction printed (0..1)

	// Return an estimate in seconds based on a specific estimation method
	float EstimateTimeLeft(PrintEstimationMethod method) const noexcept;

	// Provide some information about the file being printed
	unsigned int GetCurrentLayer() const noexcept;
	float GetCurrentLayerTime() const noexcept;				// Return the number of seconds printing the current layer
	float GetPrintDuration() const noexcept;
	float GetWarmUpDuration() const noexcept;
	float GetPauseDuration() const noexcept;

	const char *GetPrintingFilename() const noexcept { return (isPrinting) ? filenameBeingPrinted.c_str() : nullptr; }
	bool GetPrintingFileInfo(GCodeFileInfo& info) noexcept;
	void SetPrintingFileInfo(const char *filename, GCodeFileInfo& info) noexcept;

	GCodeResult ProcessM73(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void SetSlicerTimeLeft(float seconds) noexcept;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(filament)
	OBJECT_MODEL_ARRAY(thumbnail)

private:
	static constexpr float MinFilamentUsageForEstimation = 0.01;		// Minimum per cent of filament to be printed before the filament-based estimation returns values
	static constexpr uint32_t UpdateIntervalMillis = 200;				// Update interval in milliseconds
	static constexpr uint32_t SnapshotIntervalSecondsPrinting = 30;		// Snapshot interval in seconds
	static constexpr uint32_t SnapshotIntervalSecondsSimulating = 1;	// Snapshot interval in seconds

	void Reset() noexcept;
	void UpdatePrintingFileInfo() noexcept;

#if SUPPORT_OBJECT_MODEL
	ExpressionValue EstimateTimeLeftAsExpression(PrintEstimationMethod method) const noexcept;
	int32_t GetPrintOrSimulatedDuration() const noexcept;
#endif

	Platform& platform;
	GCodes& gCodes;
	uint32_t lastUpdateTime;

	bool isPrinting;
	bool heatingUp;
	bool paused;

	uint64_t printStartTime;
	uint64_t heatingStartedTime;
	uint64_t warmUpDuration, printDuration;
	uint64_t pauseStartTime, totalPauseTime;
	uint64_t lastSnapshotTime;
	uint64_t lastSnapshotNonPrintingTime;
	uint64_t lastLayerChangeTime;
	uint64_t lastLayerChangeNonPrintingTime;
	uint64_t whenSlicerTimeLeftSet;

	uint32_t lastLayerDuration;

	unsigned int currentLayer;
	float lastSnapshotFileFraction, lastSnapshotFilamentUsed;
	float fileProgressRate, filamentProgressRate;
	float totalFilamentNeeded;
	float slicerTimeLeft;						// time left in seconds as reported by slicer

	unsigned int lastLayerNumberNotified;
	float lastLayerStartHeightNotified;

	static ReadWriteLock printMonitorLock;

	bool printingFileParsed;
	GCodeFileInfo printingFileInfo;
	String<MaxFilenameLength> filenameBeingPrinted;
};

inline bool PrintMonitor::IsPrinting() const noexcept { return isPrinting; }
inline unsigned int PrintMonitor::GetCurrentLayer() const noexcept { return currentLayer; }

#endif /* PRINTMONITOR_H */

// vim: ts=4:sw=4
