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

enum PrintEstimationMethod {
	filamentBased,
	fileBased,
	layerBased
};

// Struct to hold Gcode file information
struct GcodeFileInfo
{
	unsigned long fileSize;
	float objectHeight;
	float filamentNeeded[DRIVES - AXES];
	unsigned int numFilaments;
	float layerHeight;
	char generatedBy[50];
};

class PrintMonitor
{
	public:
		PrintMonitor(Platform *p, GCodes *gc);
		void Spin();
		void Init();

		bool IsPrinting() const;						// Is a file being printed?
		void StartingPrint(const char *filename);		// Called to indicate a file will be printed (see M23)
		void StartedPrint();							// Called whenever a new live print starts (see M24)
		void StoppedPrint();							// Called whenever a file print has stopped

	    bool GetFileInfo(const char *directory, const char *fileName, GcodeFileInfo& info) const;
		void GetFileInfoResponse(StringRef& response, const char* filename) const;

		float EstimateTimeLeft(PrintEstimationMethod method) const;

		const char *GetPrintFilename() const;
		unsigned int GetCurrentLayer() const;
		float GetCurrentLayerTime() const;
		float GetPrintDuration() const;
		float GetWarmUpDuration() const;
		float GetFirstLayerDuration() const;
		float GetFirstLayerHeight() const;

	private:
		Platform *platform;
		GCodes *gCodes;
		float longWait;

	    bool fileInfoDetected;
	    char fileBeingPrinted[MaxFilenameLength];
	    GcodeFileInfo currentFileInfo;

		bool isPrinting;
	    float printStartTime;
	    unsigned int currentLayer;
	    float warmUpDuration;

	    float firstLayerDuration;
	    float firstLayerHeight;
	    float firstLayerFilament;
	    float firstLayerProgress;

	    float lastLayerTime, lastLayerFilament;
	    unsigned int numLayerSamples;
	    float layerDurations[MAX_LAYER_SAMPLES];
	    float filamentUsagePerLayer[MAX_LAYER_SAMPLES];
	    float fileProgressPerLayer[MAX_LAYER_SAMPLES];
	    float layerEstimatedTimeLeft;

	    bool FindHeight(const char* buf, size_t len, float& height) const;
	    bool FindLayerHeight(const char* buf, size_t len, float& layerHeight) const;
	    unsigned int FindFilamentUsed(const char* buf, size_t len, float *filamentUsed, unsigned int maxFilaments) const;
};

inline const char *PrintMonitor::GetPrintFilename() const { return fileBeingPrinted; }
inline bool PrintMonitor::IsPrinting() const { return isPrinting; }
inline unsigned int PrintMonitor::GetCurrentLayer() const { return currentLayer; }
inline float PrintMonitor::GetCurrentLayerTime() const { return (lastLayerTime > 0.0) ? (platform->Time() - lastLayerTime) : 0.0; }
inline float PrintMonitor::GetPrintDuration() const { return (printStartTime > 0.0) ? (platform->Time() - printStartTime) : 0.0; }
inline float PrintMonitor::GetWarmUpDuration() const { return warmUpDuration; }
inline float PrintMonitor::GetFirstLayerDuration() const { return firstLayerDuration; }
inline float PrintMonitor::GetFirstLayerHeight() const { return firstLayerHeight; }

#endif /* PRINTMONITOR_H */
