/****************************************************************************************************

RepRapFirmware - Reprap

RepRap is a simple class that acts as a container for an instance of all the others.

-----------------------------------------------------------------------------------------------------

Version 0.1

21 May 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef REPRAP_H
#define REPRAP_H

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include <RTOSIface/RTOSIface.h>
#include <General/function_ref.h>
#include <ObjectModel/GlobalVariables.h>
#include "MessageBox.h"

#if SUPPORT_CAN_EXPANSION
# include <CAN/ExpansionManager.h>
#endif

enum class ResponseSource
{
	HTTP,
	AUX,
	Generic
};

typedef Bitmap<uint32_t> DebugFlags;

class RepRap INHERIT_OBJECT_MODEL
{
public:
	RepRap() noexcept;
	RepRap(const RepRap&) = delete;

	void EmergencyStop() noexcept;
 	void Init() noexcept;
	void Spin() noexcept;
	void Exit() noexcept;
	void Diagnostics(MessageType mtype) noexcept;
	void DeferredDiagnostics(MessageType mtype) noexcept { diagnosticsDestination = mtype; }
	void Timing(MessageType mtype) noexcept;

	bool Debug(Module module) const noexcept { return debugMaps[module.ToBaseType()].IsNonEmpty(); }
	DebugFlags GetDebugFlags(Module m) const noexcept { return debugMaps[m.ToBaseType()]; }
	void SetDebug(Module m, uint32_t flags) noexcept;
	void ClearDebug() noexcept;
	void PrintDebug(MessageType mt) noexcept;
	Module GetSpinningModule() const noexcept;

	const char *GetName() const noexcept;
	void SetName(const char* nm) noexcept;
	bool NoPasswordSet() const noexcept;
	bool CheckPassword(const char* pw) const noexcept;
	void SetPassword(const char* pw) noexcept;

	Platform& GetPlatform() const noexcept { return *platform; }
	Move& GetMove() const noexcept { return *move; }
	Heat& GetHeat() const noexcept { return *heat; }
	GCodes& GetGCodes() const noexcept { return *gCodes; }
	Network& GetNetwork() const noexcept { return *network; }
	PrintMonitor& GetPrintMonitor() const noexcept { return *printMonitor; }
	FansManager& GetFansManager() const noexcept { return *fansManager; }

	// Message box functions
	uint32_t SendAlert(MessageType mt, const char *_ecv_array p_message, const char *_ecv_array title, int sParam, float tParam, AxesBitmap controls, MessageBoxLimits *_ecv_null limits = nullptr) noexcept;
	void SendSimpleAlert(MessageType mt, const char *_ecv_array p_message, const char *_ecv_array title) noexcept;

#if SUPPORT_IOBITS
 	PortControl& GetPortControl() const noexcept { return *portControl; }
#endif
#if SUPPORT_DIRECT_LCD
 	Display& GetDisplay() const noexcept { return *display; }
 	const char *GetLatestMessage(uint16_t& sequence) const noexcept;
#endif
#if HAS_SBC_INTERFACE
 	bool UsingSbcInterface() const noexcept { return usingSbcInterface; }
 	SbcInterface& GetSbcInterface() const noexcept { return *sbcInterface; }
#endif
#if SUPPORT_CAN_EXPANSION
 	ExpansionManager& GetExpansion() const noexcept { return *expansion; }
#endif

#if SUPPORT_REMOTE_COMMANDS
 	void ScheduleReset() noexcept { whenDeferredCommandScheduled = millis(); deferredCommand = DeferredCommand::reboot; }
 	void ScheduleFirmwareUpdateOverCan() noexcept { whenDeferredCommandScheduled = millis(); deferredCommand = DeferredCommand::updateFirmware; }
#endif

	void Tick() noexcept;
	bool SpinTimeoutImminent() const noexcept;
	bool IsStopped() const noexcept;

	OutputBuffer *GetStatusResponse(uint8_t type, ResponseSource source) const noexcept;
	OutputBuffer *GetConfigResponse() noexcept;
	OutputBuffer *GetLegacyStatusResponse(uint8_t type, int seq) const noexcept;

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	OutputBuffer *GetFilesResponse(const char* dir, unsigned int startAt, bool flagsDirs) noexcept;
	OutputBuffer *GetFilelistResponse(const char* dir, unsigned int startAt) noexcept;
	OutputBuffer *GetThumbnailResponse(const char *filename, FilePosition offset, bool forM31point1) noexcept;
#endif

	GCodeResult GetFileInfoResponse(const char *filename, OutputBuffer *&response, bool quitEarly) noexcept;
	OutputBuffer *GetModelResponse(const GCodeBuffer *_ecv_null gb, const char *key, const char *flags) const THROWS(GCodeException);
	Mutex& GetObjectModelReportMutex() noexcept { return objectModelReportMutex; }

	void Beep(unsigned int freq, unsigned int ms) noexcept;
	void SetMessage(const char *msg) noexcept;

	bool IsProcessingConfig() const noexcept { return processingConfig; }

	// Firmware update operations
	bool CheckFirmwareUpdatePrerequisites(const StringRef& reply, const StringRef& filenameRef) noexcept;
#if HAS_MASS_STORAGE
	void UpdateFirmware(const char *iapFilename, const char *iapParam) noexcept;
#endif
	void PrepareToLoadIap() noexcept;
	[[noreturn]] void StartIap(const char *filename) noexcept;

	void ReportInternalError(const char *file, const char *func, int line) const noexcept;	// report an internal error

	static uint32_t DoDivide(uint32_t a, uint32_t b) noexcept;			// helper function for diagnostic tests
	static void GenerateBusFault() noexcept;							// helper function for diagnostic tests
	static float SinfCosf(float angle) noexcept;						// helper function for diagnostic tests
	static float FastSqrtf(float f) noexcept;							// helper function for diagnostic tests

	void KickHeatTaskWatchdog() noexcept { heatTaskIdleTicks = 0; }

	void SaveConfigError(const char *filename, unsigned int lineNumber, const char *errorMessage) noexcept;

	void BoardsUpdated() noexcept { ++boardsSeq; }
	void DirectoriesUpdated() noexcept { ++directoriesSeq; }
	void FansUpdated() noexcept { ++fansSeq; }
	void GlobalUpdated() noexcept { ++globalSeq; }
	void HeatUpdated() noexcept { ++heatSeq; }
	void InputsUpdated() noexcept { ++inputsSeq; }
	void LedStripsUpdated() noexcept { ++ledStripsSeq; }
	void JobUpdated() noexcept { ++jobSeq; }
	void MoveUpdated() noexcept { ++moveSeq; }
	void NetworkUpdated() noexcept { ++networkSeq; }
	void ScannerUpdated() noexcept { ++scannerSeq; }
	void SensorsUpdated() noexcept { ++sensorsSeq; }
	void SpindlesUpdated() noexcept { ++spindlesSeq; }
	void StateUpdated() noexcept { ++stateSeq; }
	void ToolsUpdated() noexcept { ++toolsSeq; }
	void VolumesUpdated() noexcept { ++volumesSeq; }

	ReadLockedPointer<const VariableSet> GetGlobalVariablesForReading() noexcept { return globalVariables.GetForReading(); }
	WriteLockedPointer<VariableSet> GetGlobalVariablesForWriting() noexcept { return globalVariables.GetForWriting(); }

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

	ReadWriteLock *_ecv_null GetObjectLock(unsigned int tableNumber) const noexcept override;

private:
	static void EncodeString(StringRef& response, const char* src, size_t spaceToLeave, bool allowControlChars = false, char prefix = 0) noexcept;
	static void AppendFloatArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref_noexcept<float(size_t) noexcept> func, unsigned int numDecimalDigits) noexcept;
	static void AppendIntArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref_noexcept<int(size_t) noexcept> func) noexcept;
	static void AppendStringArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref_noexcept<const char *(size_t) noexcept> func) noexcept;

	size_t GetStatusIndex() const noexcept;
	char GetStatusCharacter() const noexcept;
	const char* GetStatusString() const noexcept;
	bool RunStartupFile(const char *filename, bool isMainConfigFile) noexcept;

	static constexpr uint32_t MaxHeatTaskTicksInSpinState = 4000;	// timeout before we reset the processor if the heat task doesn't run
	static constexpr uint32_t MaxMainTaskTicksInSpinState = 20000;	// timeout before we reset the processor if the main task doesn't run
	static constexpr uint32_t HighMainTaskTicksInSpinState = 16000;	// how long before we warn that timeout is approaching

	Platform* platform;
	Network* network;
	Move* move;
	Heat* heat;
	GCodes* gCodes;
 	PrintMonitor* printMonitor;
 	FansManager* fansManager;

 	Mutex objectModelReportMutex;									// mutex used to limit concurrent reporting of object model, which may result in output buffer starvation

 	// Recording the first error message encountered in config.g
 	AutoStringHandle configErrorFilename;
 	unsigned int configErrorLine;
 	AutoStringHandle configErrorMessage;

#if SUPPORT_IOBITS
 	PortControl *portControl;
#endif

#if SUPPORT_DIRECT_LCD
 	Display *display;
#endif

#if HAS_SBC_INTERFACE
 	SbcInterface *sbcInterface;
#endif

#if SUPPORT_CAN_EXPANSION
 	ExpansionManager *expansion;
#endif

	uint16_t boardsSeq, directoriesSeq, fansSeq, heatSeq, inputsSeq, jobSeq, ledStripsSeq, moveSeq, globalSeq;
	uint16_t networkSeq, scannerSeq, sensorsSeq, spindlesSeq, stateSeq, toolsSeq, volumesSeq;

	GlobalVariables globalVariables;

	uint32_t lastWarningMillis;					// when we last sent a warning message for things that can happen very often

	uint16_t ticksInSpinState;
	uint16_t heatTaskIdleTicks;
	uint32_t fastLoop, slowLoop;

#if SUPPORT_REMOTE_COMMANDS
	enum class DeferredCommand : uint8_t { none, reboot, updateFirmware };
	volatile uint32_t whenDeferredCommandScheduled;
	volatile DeferredCommand deferredCommand;
#endif

	DebugFlags debugMaps[NumRealModules];

	String<RepRapPasswordLength> password;
	String<MachineNameLength> myName;

	unsigned int beepFrequency, beepDuration;
	uint32_t beepTimer;
	String<MaxMessageLength> message;
#if SUPPORT_DIRECT_LCD
	uint16_t messageSequence;					// used by 12864 display to detect when there is a new message
#endif

	// Deferred diagnostics
	MessageType diagnosticsDestination;
	bool justSentDiagnostics;

	// State flags
	Module spinningModule;
	bool stopped;
	bool active;
	bool processingConfig;
#if HAS_SBC_INTERFACE
 	bool usingSbcInterface;
#endif
};

// A single instance of the RepRap class contains all the others
extern RepRap reprap;

inline Module RepRap::GetSpinningModule() const noexcept { return spinningModule; }
inline bool RepRap::IsStopped() const noexcept { return stopped; }

#endif


