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

const FilePosition GCODE_HEADER_SIZE = 20000uL;		// How many bytes to read from the header - I (DC) have a Kisslicer file with a layer height comment 14Kb from the start
const FilePosition GCODE_FOOTER_SIZE = 400000uL;	// How many bytes to read from the footer

#if SAM4E || SAM4S
const size_t GCODE_READ_SIZE = 4096;				// How many bytes to read in one go in GetFileInfo() (should be a multiple of 512 for read efficiency)
#else
const size_t GCODE_READ_SIZE = 1024;				// How many bytes to read in one go in GetFileInfo() (should be a multiple of 512 for read efficiency)
#endif

const size_t GCODE_OVERLAP_SIZE = 100;				// Size of the overlapping buffer for searching (should be a multiple of 4)

const float LAYER_HEIGHT_TOLERANCE = 0.015;			// Tolerance for comparing two Z heights (in mm)

const size_t MAX_LAYER_SAMPLES = 5;					// Number of layer samples for end-time estimation (except for first layer)
const float ESTIMATION_MIN_FILAMENT_USAGE = 0.01;	// Minimum per cent of filament to be printed before the filament-based estimation returns values
const float ESTIMATION_MIN_FILE_USAGE = 0.001;		// Minimum per cent of the file to be processed before any file-based estimations are made
const float FIRST_LAYER_SPEED_FACTOR = 0.25;		// First layer speed factor compared to other layers (only for layer-based estimation)

const uint32_t PRINTMONITOR_UPDATE_INTERVAL = 200;	// Update interval in milliseconds
const uint32_t MAX_FILEINFO_PROCESS_TIME = 200;		// Maximum time to spend polling for file info in each call

enum PrintEstimationMethod
{
	filamentBased,
	fileBased,
	layerBased
};

// Struct to hold Gcode file information
struct GCodeFileInfo
{
	bool isValid;
	FilePosition fileSize;
	time_t lastModifiedTime;
	float firstLayerHeight;
	float objectHeight;
	float filamentNeeded[MaxExtruders];
	unsigned int numFilaments;
	float layerHeight;
	char generatedBy[50];
};

enum FileParseState
{
	notParsing,
	parsingHeader,
	parsingFooter
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

		// The following two methods need to be called until they return true - this may take a few runs
		bool GetFileInfo(const char *directory, const char *fileName, GCodeFileInfo& info);
		bool GetFileInfoResponse(const char *filename, OutputBuffer *&response);
		void StopParsing(const char *filename);

		// Return an estimate in seconds based on a specific estimation method
		float EstimateTimeLeft(PrintEstimationMethod method) const;

		// Provide some information about the file being printed
		unsigned int GetCurrentLayer() const;
		float GetCurrentLayerTime() const;
		float GetPrintDuration() const;
		float GetWarmUpDuration() const;
		float GetFirstLayerDuration() const;
		float GetFirstLayerHeight() const;

		const char *GetPrintingFilename() const { return (isPrinting) ? filenameBeingPrinted : nullptr; }

	private:
		Platform& platform;
		GCodes& gCodes;
		uint32_t longWait;
		uint32_t lastUpdateTime;

		// Information/Events concerning the file being printed
		void WarmUpComplete();
		void FirstLayerComplete();
		void LayerComplete();

		bool isPrinting;
		uint64_t printStartTime;
		uint64_t pauseStartTime, totalPauseTime;

		bool heatingUp;
		unsigned int currentLayer;
		float warmUpDuration, firstLayerDuration;
		float firstLayerFilament, firstLayerProgress;
		float lastLayerChangeTime, lastLayerFilament, lastLayerZ;

		unsigned int numLayerSamples;
		float layerDurations[MAX_LAYER_SAMPLES];
		float filamentUsagePerLayer[MAX_LAYER_SAMPLES];
		float fileProgressPerLayer[MAX_LAYER_SAMPLES];
		float layerEstimatedTimeLeft;

		// We parse G-Code files in multiple stages. These variables hold the required information
		volatile FileParseState parseState;
		char filenameBeingParsed[MaxFilenameLength];
		FileStore *fileBeingParsed;
		GCodeFileInfo parsedFileInfo;

		char fileOverlap[GCODE_OVERLAP_SIZE];
		size_t fileOverlapLength;

		bool printingFileParsed;
		GCodeFileInfo printingFileInfo;
		char filenameBeingPrinted[MaxFilenameLength];

		// G-Code parser methods
		bool FindHeight(const char* buf, size_t len, float& height) const;
		bool FindFirstLayerHeight(const char* buf, size_t len, float& layerHeight) const;
		bool FindLayerHeight(const char* buf, size_t len, float& layerHeight) const;
		unsigned int FindFilamentUsed(const char* buf, size_t len, float *filamentUsed, unsigned int maxFilaments) const;

		uint32_t accumulatedParseTime, accumulatedReadTime, accumulatedSeekTime;
};

inline bool PrintMonitor::IsPrinting() const { return isPrinting; }
inline unsigned int PrintMonitor::GetCurrentLayer() const { return currentLayer; }
inline float PrintMonitor::GetCurrentLayerTime() const { return (lastLayerChangeTime > 0.0) ? (GetPrintDuration() - lastLayerChangeTime) : 0.0; }
inline float PrintMonitor::GetFirstLayerDuration() const { return (firstLayerDuration > 0.0) ? firstLayerDuration : ((currentLayer > 0) ? GetPrintDuration() - warmUpDuration : 0.0); }
inline float PrintMonitor::GetFirstLayerHeight() const { return printingFileParsed ? printingFileInfo.firstLayerHeight : 0.0; }

#endif /* PRINTMONITOR_H */

// vim: ts=4:sw=4
