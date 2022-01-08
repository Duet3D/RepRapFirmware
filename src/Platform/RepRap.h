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

#if SUPPORT_CAN_EXPANSION
# include <CAN/ExpansionManager.h>
#endif

enum class ResponseSource
{
	HTTP,
	AUX,
	Generic
};

// Message box data
struct MessageBox
{
	bool active;
	String<MaxMessageLength> message;
	String<MaxTitleLength> title;
	int mode;
	uint32_t seq;
	uint32_t timer, timeout;
	AxesBitmap controls;

	MessageBox() noexcept : active(false), seq(0) { }
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

	bool Debug(Module module) const noexcept { return debugMaps[module].IsNonEmpty(); }
	DebugFlags GetDebugFlags(Module m) const noexcept { return debugMaps[m]; }
	void SetDebug(Module m, uint32_t flags) noexcept;
	void ClearDebug() noexcept;
	void PrintDebug(MessageType mt) noexcept;
	Module GetSpinningModule() const noexcept;

	const char *GetName() const noexcept;
	void SetName(const char* nm) noexcept;
	bool NoPasswordSet() const noexcept;
	bool CheckPassword(const char* pw) const noexcept;
	void SetPassword(const char* pw) noexcept;

	// Tool management
	void AddTool(Tool* t) noexcept;
	void DeleteTool(int toolNumber) noexcept;
	void SelectTool(int toolNumber, bool simulating) noexcept;
	void StandbyTool(int toolNumber, bool simulating) noexcept;
	int GetCurrentToolNumber() const noexcept;
	void SetPreviousToolNumber() noexcept;
	Tool *GetCurrentTool() const noexcept;
	ReadLockedPointer<Tool> GetLockedCurrentTool() const noexcept;
	ReadLockedPointer<Tool> GetTool(int toolNumber) const noexcept;
	ReadLockedPointer<Tool> GetCurrentOrDefaultTool() const noexcept;
	ReadLockedPointer<Tool> GetFirstTool() const noexcept;										// Return the lowest-numbered tool
	AxesBitmap GetCurrentXAxes() const noexcept;												// Get the current axes used as X axes
	AxesBitmap GetCurrentYAxes() const noexcept;												// Get the current axes used as Y axes
	AxesBitmap GetCurrentAxisMapping(unsigned int axis) const noexcept;
	bool IsHeaterAssignedToTool(int8_t heater) const noexcept;
	unsigned int GetNumberOfContiguousTools() const noexcept;
	void ReportAllToolTemperatures(const StringRef& reply) const noexcept;
	GCodeResult SetAllToolsFirmwareRetraction(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& outBuf) THROWS(GCodeException);

	unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions) noexcept;
	void PrintTool(int toolNumber, const StringRef& reply) const noexcept;
	void FlagTemperatureFault(int8_t dudHeater) noexcept;
	GCodeResult ClearTemperatureFault(int8_t wasDudHeater, const StringRef& reply) noexcept;

	Platform& GetPlatform() const noexcept { return *platform; }
	Move& GetMove() const noexcept { return *move; }
	Heat& GetHeat() const noexcept { return *heat; }
	GCodes& GetGCodes() const noexcept { return *gCodes; }
	Network& GetNetwork() const noexcept { return *network; }
	Scanner& GetScanner() const noexcept { return *scanner; }
	PrintMonitor& GetPrintMonitor() const noexcept { return *printMonitor; }
	FansManager& GetFansManager() const noexcept { return *fansManager; }

#if SUPPORT_ROLAND
	Roland& GetRoland() const noexcept { return *roland; }
#endif
#if SUPPORT_IOBITS
 	PortControl& GetPortControl() const noexcept { return *portControl; }
#endif
#if SUPPORT_12864_LCD
 	Display& GetDisplay() const noexcept { return *display; }
 	const char *GetLatestMessage(uint16_t& sequence) const noexcept;
 	const MessageBox& GetMessageBox() const noexcept { return mbox; }
#endif
#if HAS_SBC_INTERFACE
 	bool UsingSbcInterface() const noexcept { return usingSbcInterface; }
 	SbcInterface& GetSbcInterface() const noexcept { return *sbcInterface; }
#endif
#if SUPPORT_CAN_EXPANSION
 	ExpansionManager& GetExpansion() const noexcept { return *expansion; }
#endif

	void Tick() noexcept;
	bool SpinTimeoutImminent() const noexcept;
	bool IsStopped() const noexcept;

	uint16_t GetExtrudersInUse() const noexcept;
	uint16_t GetToolHeatersInUse() const noexcept;

	OutputBuffer *GetStatusResponse(uint8_t type, ResponseSource source) const noexcept;
	OutputBuffer *GetConfigResponse() noexcept;
	OutputBuffer *GetLegacyStatusResponse(uint8_t type, int seq) const noexcept;

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	OutputBuffer *GetFilesResponse(const char* dir, unsigned int startAt, bool flagsDirs) noexcept;
	OutputBuffer *GetFilelistResponse(const char* dir, unsigned int startAt) noexcept;
	OutputBuffer *GetThumbnailResponse(const char *filename, FilePosition offset) noexcept;
#endif

	GCodeResult GetFileInfoResponse(const char *filename, OutputBuffer *&response, bool quitEarly) noexcept;

#if SUPPORT_OBJECT_MODEL
	OutputBuffer *GetModelResponse(const char *key, const char *flags) const THROWS(GCodeException);
#endif

	void Beep(unsigned int freq, unsigned int ms) noexcept;
	void SetMessage(const char *msg) noexcept;
	void SetAlert(const char *msg, const char *title, int mode, float timeout, AxesBitmap controls) noexcept;
	void ClearAlert() noexcept;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteToolSettings(FileStore *f) noexcept;						// save some information for the resume file
	bool WriteToolParameters(FileStore *f, const bool forceWriteOffsets) noexcept;	// save some information in config-override.g
#endif

	bool IsProcessingConfig() const noexcept { return processingConfig; }

	// Firmware update operations
	bool CheckFirmwareUpdatePrerequisites(const StringRef& reply, const StringRef& filenameRef) noexcept;
	void UpdateFirmware(const StringRef& filenameRef) noexcept;
	void PrepareToLoadIap() noexcept;
	[[noreturn]] void StartIap(const char *filename) noexcept;

	void ReportInternalError(const char *file, const char *func, int line) const noexcept;	// report an internal error

	static uint32_t DoDivide(uint32_t a, uint32_t b) noexcept;			// helper function for diagnostic tests
	static void GenerateBusFault() noexcept;							// helper function for diagnostic tests
	static float SinfCosf(float angle) noexcept;						// helper function for diagnostic tests
	static float FastSqrtf(float f) noexcept;							// helper function for diagnostic tests

	void KickHeatTaskWatchdog() noexcept { heatTaskIdleTicks = 0; }

	void BoardsUpdated() noexcept { ++boardsSeq; }
	void DirectoriesUpdated() noexcept { ++directoriesSeq; }
	void FansUpdated() noexcept { ++fansSeq; }
	void GlobalUpdated() noexcept { ++globalSeq; }
	void HeatUpdated() noexcept { ++heatSeq; }
	void InputsUpdated() noexcept { ++inputsSeq; }
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
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(boards)
	OBJECT_MODEL_ARRAY(fans)
	OBJECT_MODEL_ARRAY(gpout)
	OBJECT_MODEL_ARRAY(inputs)
	OBJECT_MODEL_ARRAY(spindles)
	OBJECT_MODEL_ARRAY(tools)
	OBJECT_MODEL_ARRAY(restorePoints)
	OBJECT_MODEL_ARRAY(volumes)
	OBJECT_MODEL_ARRAY(volChanges)

private:
	static void EncodeString(StringRef& response, const char* src, size_t spaceToLeave, bool allowControlChars = false, char prefix = 0) noexcept;
	static void AppendFloatArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref<float(size_t)> func, unsigned int numDecimalDigits) noexcept;
	static void AppendIntArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref<int(size_t)> func) noexcept;
	static void AppendStringArray(OutputBuffer *buf, const char *name, size_t numValues, function_ref<const char *(size_t)> func) noexcept;

	size_t GetStatusIndex() const noexcept;
	char GetStatusCharacter() const noexcept;
	const char* GetStatusString() const noexcept;
	void ReportToolTemperatures(const StringRef& reply, const Tool *tool, bool includeNumber) const noexcept;
	bool RunStartupFile(const char *filename) noexcept;

	static constexpr uint32_t MaxTicksInSpinState = 20000;	// timeout before we reset the processor
	static constexpr uint32_t HighTicksInSpinState = 16000;	// how long before we warn that timeout is approaching

	static ReadWriteLock toolListLock;

	Platform* platform;
	Network* network;
	Move* move;
	Heat* heat;
	GCodes* gCodes;
	Scanner* scanner;
 	PrintMonitor* printMonitor;
 	FansManager* fansManager;

#if SUPPORT_IOBITS
 	PortControl *portControl;
#endif

#if SUPPORT_12864_LCD
 	Display *display;
#endif

#if HAS_SBC_INTERFACE
 	SbcInterface *sbcInterface;
#endif

#if SUPPORT_ROLAND
	Roland* roland;
#endif

#if SUPPORT_CAN_EXPANSION
 	ExpansionManager *expansion;
#endif

 	mutable Mutex messageBoxMutex;				// mutable so that we can lock and release it in const functions

	uint16_t boardsSeq, directoriesSeq, fansSeq, heatSeq, inputsSeq, jobSeq, moveSeq, globalSeq;
	uint16_t networkSeq, scannerSeq, sensorsSeq, spindlesSeq, stateSeq, toolsSeq, volumesSeq;

	GlobalVariables globalVariables;

	Tool* toolList;								// the tool list is sorted in order of increasing tool number
	Tool* currentTool;
	uint32_t lastWarningMillis;					// when we last sent a warning message for things that can happen very often

	uint16_t activeExtruders;
	uint16_t activeToolHeaters;
	uint16_t numToolsToReport;

	uint16_t ticksInSpinState;
	uint16_t heatTaskIdleTicks;
	uint32_t fastLoop, slowLoop;

	DebugFlags debugMaps[Module::numModules];

	String<RepRapPasswordLength> password;
	String<MachineNameLength> myName;

	unsigned int beepFrequency, beepDuration;
	uint32_t beepTimer;
	String<MaxMessageLength> message;
#if SUPPORT_12864_LCD
	uint16_t messageSequence;					// used by 12864 display to detect when there is a new message
#endif

	MessageBox mbox;							// message box data

	int16_t previousToolNumber;					// the tool number we were using before the last tool change, or -1 if we weren't using a tool

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

inline Tool* RepRap::GetCurrentTool() const noexcept { return currentTool; }
inline uint16_t RepRap::GetExtrudersInUse() const noexcept { return activeExtruders; }
inline uint16_t RepRap::GetToolHeatersInUse() const noexcept { return activeToolHeaters; }
inline bool RepRap::IsStopped() const noexcept { return stopped; }

#define REPORT_INTERNAL_ERROR do { reprap.ReportInternalError((__FILE__), (__func__), (__LINE__)); } while(0)

#endif


