/****************************************************************************************************

RepRapFirmware - G Codes

This class interprets G Codes from one or more sources, and calls the functions in Move, Heat etc
that drive the machine to do what the G Codes command.

-----------------------------------------------------------------------------------------------------

Version 0.1

13 February 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef GCODES_H
#define GCODES_H

#include "RepRapFirmware.h"
#include <Platform/RepRap.h>			// for type ResponseSource
#include "ObjectTracker.h"
#include <Movement/RawMove.h>
#include <Libraries/sha1/sha1.h>
#include <Platform/Platform.h>		// for type EndStopHit
#include <Platform/PrintPausedReason.h>
#include "GCodeChannel.h"
#include "GCodeInput.h"
#include "GCodeMachineState.h"
#include "CollisionAvoider.h"
#include "KeepoutZone.h"
#include "TriggerItem.h"
#include <Tools/Filament.h>
#include <FilamentMonitors/FilamentMonitor.h>
#include "RestorePoint.h"
#include "StraightProbeSettings.h"
#include <Movement/BedProbing/Grid.h>

const char feedrateLetter = 'F';						// GCode feedrate
const char extrudeLetter = 'E'; 						// GCode extrude

// Machine type enumeration. The numeric values must be in the same order as the corresponding M451..M453 commands.
enum class MachineType : uint8_t
{
	fff = 0,
	laser = 1,
	cnc = 2
};

enum class StopPrintReason : uint8_t
{
	normalCompletion,
	userCancelled,
	abort
};

enum class PauseState : uint8_t
{
	// Do not change the order of these! We rely on notPaused < pausing < { paused, resuming, cancelling }
	notPaused = 0,
	pausing,
	paused,
	resuming,
	cancelling
};

struct M585Settings
{
	size_t axisNumber;			// the axis we are moving
	float feedRate;
	float probingLimit;
	float offset;
	bool useProbe;				// true if using a probe (M585 only because M675 always uses a probe)
};

struct M675Settings
{
	size_t axisNumber;			// the axis we are moving
	float feedRate;
	float backoffDistance;		// back off distance
	float minDistance;			// the position we reached when probing towards minimum
};

enum class SimulationMode : uint8_t
{	off = 0,				// not simulating
	debug,					// simulating step generation
	normal,					// not generating steps, just timing
	partial,				// generating DDAs but doing nothing with them
	highest = partial
};

class SbcInterface;

// The GCode interpreter

class GCodes
{
public:
	GCodes(Platform& p) noexcept;
	void Spin() noexcept;														// Called in a tight loop to make this class work
	void Init() noexcept;														// Set it up
	void Exit() noexcept;														// Shut it down
	void Reset() noexcept;														// Reset some parameter to defaults
	bool ReadMove(MovementSystemNumber queueNumber, RawMove& m) noexcept
		pre(queueNumber < ARRAY_SIZE(moveStates));								// Called by the Move class to get a movement set by the last G Code
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	bool QueueFileToPrint(const char* fileName, const StringRef& reply) noexcept;	// Open a file of G Codes to run
#endif
	void AbortPrint(GCodeBuffer& gb) noexcept;									// Cancel any print in progress
	void HandleM114(GCodeBuffer& gb, const StringRef& s) const noexcept;		// Write where we are into a string
	bool DoingFileMacro() const noexcept;										// Is a macro file being processed by any input channel?
	bool GetMacroRestarted() const noexcept;									// Return true if the macro being executed by fileGCode was restarted
	bool WaitingForAcknowledgement() const noexcept;							// Is an input waiting for a message to be acknowledged?

	FilePosition GetPrintingFilePosition() const noexcept;						// Return the current position of the file being printed in bytes. May return noFilePosition if allowNoFilePos is true
	void Diagnostics(MessageType mtype) noexcept;								// Send helpful information out

	bool RunConfigFile(const char* fileName, bool isMainConfigFile) noexcept;	// Start running a configuration file
	bool IsTriggerBusy() const noexcept;										// Return true if the trigger G-code buffer is busy running config.g or a trigger file

	bool IsAxisHomed(unsigned int axis) const noexcept							// Has the axis been homed?
		{ return axesVirtuallyHomed.IsBitSet(axis); }
	void SetAxisIsHomed(unsigned int axis) noexcept;							// Tell us that the axis is now homed
	void SetAxisNotHomed(unsigned int axis) noexcept;							// Tell us that the axis is not homed
	void SetAllAxesNotHomed() noexcept;											// Flag all axes as not homed

	float GetPrimarySpeedFactor() const noexcept { return moveStates[0].speedFactor; }	// Return the current speed factor as a fraction
	float GetExtrusionFactor(size_t extruder) noexcept;							// Return the current extrusion factor for the specified extruder
	float GetFilamentDiameter(size_t extruder) const noexcept
		pre(extruder < MaxExtruders) { return filamentDiameters[extruder]; }
	float GetRawExtruderTotalByDrive(size_t extruder) const noexcept;			// Get the total extrusion since start of print, for one drive
	float GetTotalRawExtrusion() const noexcept { return rawExtruderTotal; }	// Get the total extrusion since start of print, all drives
	float GetTotalBabyStepOffset(size_t axis) const noexcept
		pre(axis < maxAxes);
	float GetUserCoordinate(const MovementState& ms, size_t axis) const noexcept;	// Get the current user coordinate in the current workspace coordinate system

	bool CheckNetworkCommandAllowed(GCodeBuffer& gb, const StringRef& reply, GCodeResult& result) noexcept;
#if HAS_NETWORKING
	NetworkGCodeInput *GetHTTPInput() const noexcept { return httpInput; }
	NetworkGCodeInput *GetTelnetInput() const noexcept { return telnetInput; }
#endif

	PauseState GetPauseState() const noexcept { return pauseState; }
	bool IsFlashing() const noexcept { return isFlashing; }						// Is a new firmware binary going to be flashed?

	bool IsFlashingPanelDue() const noexcept
#if SUPPORT_PANELDUE_FLASH
		{ return isFlashingPanelDue; }
#else
		{ return false; }
#endif

	bool IsReallyPrinting() const noexcept;										// Return true if we are printing from SD card and not pausing, paused or resuming
	bool IsReallyPrintingOrResuming() const noexcept;
	bool IsCancellingPrint() const noexcept;
	bool IsSimulating() const noexcept { return simulationMode != SimulationMode::off; }
	bool IsDoingToolChange() const noexcept { return doingToolChange; }
	bool IsHeatingUp() const noexcept;											// Return true if the SD card print is waiting for a heater to reach temperature
	void CancelWaitForTemperatures(bool onlyInPrintFiles) noexcept;
	bool IsRunningConfigFile() const noexcept { return runningConfigFile; }
	bool SawM501InConfigFile() const noexcept { return m501SeenInConfigFile; }

	uint32_t GetLastDuration() const noexcept { return lastDuration; }			// The most recent print time or simulated time
	float GetSimulationTime() const noexcept { return simulationTime; }

	bool AllAxesAreHomed() const noexcept;										// Return true if all axes are homed
	bool LimitAxes() const noexcept { return limitAxes; }
	bool NoMovesBeforeHoming() const noexcept { return noMovesBeforeHoming; }

	void MoveStoppedByZProbe() noexcept { zProbeTriggered = true; }				// Called from the step ISR when the Z probe is triggered, causing the move to be aborted

	size_t GetTotalAxes() const noexcept { return numTotalAxes; }
	size_t GetVisibleAxes() const noexcept { return numVisibleAxes; }
	size_t GetNumExtruders() const noexcept { return numExtruders; }

	const char* GetMachineModeString() const noexcept;							// Get the name of the current machine mode

	void HandleHeaterFault() noexcept;											// Respond to a heater fault

#if HAS_VOLTAGE_MONITOR
	bool LowVoltagePause() noexcept;
	bool LowVoltageResume() noexcept;
#endif

	const char *GetAxisLetters() const noexcept { return axisLetters; }			// Return a null-terminated string of axis letters indexed by drive
	size_t GetAxisNumberForLetter(const char axisLetter) const noexcept;
	MachineType GetMachineType() const noexcept { return machineType; }
	bool LockMovementSystemAndWaitForStandstill(GCodeBuffer& gb, MovementSystemNumber msNumber) noexcept;	// Lock a movement system and wait for pending moves to finish
	bool LockCurrentMovementSystemAndWaitForStandstill(GCodeBuffer& gb) noexcept;	// Lock movement and wait for pending moves to finish
	bool LockAllMovementSystemsAndWaitForStandstill(GCodeBuffer& gb) noexcept;	// Lock movement and wait for all motion systems to reach standstill
	uint16_t GetMotorBrakeOnDelay() const noexcept { return 200; }				// Get the delay between brake on and motors off, in milliseconds TODO make this configurable
	uint16_t GetMotorBrakeOffDelay() const noexcept { return 25; }				// Get the delay between motors on and brake off, in milliseconds

#if SUPPORT_DIRECT_LCD
	void SetPrimarySpeedFactor(float factor) noexcept;							// Set the speed factor
	void SetExtrusionFactor(size_t extruder, float factor) noexcept;			// Set an extrusion factor for the specified extruder
	void SelectPrimaryTool(int toolNumber, bool simulating) noexcept { moveStates[0].SelectTool(toolNumber, simulating); }
	bool ProcessCommandFromLcd(const char *cmd) noexcept;						// Process a GCode command from the 12864 LCD returning true if the command was accepted
	float GetItemCurrentTemperature(unsigned int itemNumber) const noexcept;
	float GetItemActiveTemperature(unsigned int itemNumber) const noexcept;
	float GetItemStandbyTemperature(unsigned int itemNumber) const noexcept;
	void SetItemActiveTemperature(unsigned int itemNumber, float temp) noexcept;
	void SetItemStandbyTemperature(unsigned int itemNumber, float temp) noexcept;
	bool EvaluateConditionForDisplay(const char *_ecv_array str) const noexcept;
	bool EvaluateValueForDisplay(const char *_ecv_array str, ExpressionValue& expr) const noexcept;
#endif

	void SetMappedFanSpeed(const GCodeBuffer *null gb, float f) noexcept;				// Set the speeds of fans mapped for the current tool
	void HandleReply(GCodeBuffer& gb, GCodeResult rslt, const char *reply) noexcept;	// Handle G-Code replies
	void EmergencyStop() noexcept;													// Cancel everything

	const GridDefinition& GetDefaultGrid() const { return defaultGrid; };			// Get the default grid definition
	void ActivateHeightmap(bool activate) noexcept;									// (De-)Activate the height map

	size_t GetCurrentZProbeNumber() const noexcept { return currentZProbeNumber; }

#if SUPPORT_SCANNING_PROBES
	void TakeScanningProbeReading() noexcept;										// Take and store a reading from a scanning Z probe
	GCodeResult HandleM558Point1or2(GCodeBuffer& gb, const StringRef &reply, unsigned int probeNumber) THROWS(GCodeException);	// Calibrate a scanning Z probe
#endif

	// These next two are public because they are used by class SbcInterface
	void UnlockAll(const GCodeBuffer& gb) noexcept;									// Release all locks
	GCodeBuffer *GetGCodeBuffer(GCodeChannel channel) const noexcept { return gcodeSources[channel.ToBaseType()]; }

#if HAS_MASS_STORAGE
	GCodeResult StartSDTiming(GCodeBuffer& gb, const StringRef& reply) noexcept;	// Start timing SD card file writing
#endif

	void SavePosition(const GCodeBuffer& gb, unsigned int restorePointNumber) noexcept
		pre(restorePointNumber < NumTotalRestorePoints);							// Save position etc. to a restore point
	void StartToolChange(GCodeBuffer& gb, MovementState& ms, uint8_t param) noexcept;

	unsigned int GetPrimaryWorkplaceCoordinateSystemNumber() const noexcept { return GetPrimaryMovementState().currentCoordinateSystem + 1; }

#if SUPPORT_COORDINATE_ROTATION
	void RotateCoordinates(float angleDegrees, float coords[2]) const noexcept;		// Account for coordinate rotation
#endif

	// This function is called by other functions to account correctly for workplace coordinates
	float GetWorkplaceOffset(const GCodeBuffer& gb, size_t axis) const noexcept
	{
		return workplaceCoordinates[GetConstMovementState(gb).currentCoordinateSystem][axis];
	}

#if SUPPORT_KEEPOUT_ZONES
	size_t GetNumKeepoutZones() const noexcept { return (keepoutZone.IsDefined()) ? 1 : 0; }
	bool IsKeepoutZoneDefined(size_t n) const noexcept { return n == 0 && keepoutZone.IsDefined(); }
	const KeepoutZone *GetKeepoutZone(size_t) const noexcept { return &keepoutZone; }
#endif

#if SUPPORT_OBJECT_MODEL
	float GetWorkplaceOffset(size_t axis, size_t workplaceNumber) const noexcept
	{
		return workplaceCoordinates[workplaceNumber][axis];
	}
	float GetPrimaryMaxPrintingAcceleration() const noexcept { return moveStates[0].maxPrintingAcceleration; }
	float GetPrimaryMaxTravelAcceleration() const noexcept { return moveStates[0].maxTravelAcceleration; }

# if SUPPORT_COORDINATE_ROTATION
	float GetRotationAngle() const noexcept { return g68Angle; }
	float GetRotationCentre(size_t index) const noexcept pre(index < 2) { return g68Centre[index]; }
# endif

	size_t GetNumInputs() const noexcept { return NumGCodeChannels; }
	const GCodeBuffer* GetInput(size_t n) const noexcept { return gcodeSources[n]; }
	const GCodeBuffer* GetInput(GCodeChannel n) const noexcept { return gcodeSources[n.RawValue()]; }

	const ObjectTracker *GetBuildObjects() const noexcept { return &buildObjects; }

	const MovementState& GetMovementState(unsigned int queueNumber) const noexcept pre(queueNumber < NumMovementSystems) { return moveStates[queueNumber]; }
	const MovementState& GetPrimaryMovementState() const noexcept { return moveStates[0]; }		// Temporary support for object model and status report values that only handle a single movement system
	const MovementState& GetCurrentMovementState(const ObjectExplorationContext& context) const noexcept;
	const MovementState& GetConstMovementState(const GCodeBuffer& gb) const noexcept;			// Get a reference to the movement state associated with the specified GCode buffer (there is a private non-const version)
	bool IsHeaterUsedByDifferentCurrentTool(int heaterNumber, const Tool *tool) const noexcept;	// Check if the specified heater is used by a current tool other than the specified one
	void MessageBoxClosed(bool cancelled, bool m292, uint32_t seq, ExpressionValue rslt) noexcept;

# if HAS_VOLTAGE_MONITOR
	const char *_ecv_array null GetPowerFailScript() const noexcept { return powerFailScript; }
# endif

# if SUPPORT_LASER
	// Return laser PWM in 0..1. Only the primary movement queue is permitted to control the laser.
	float GetLaserPwm() const noexcept
	{
		return (float)moveStates[0].laserPwmOrIoBits.laserPwm * (1.0/65535);
	}
# endif
#endif

#if !HAS_MASS_STORAGE && !HAS_EMBEDDED_FILES && defined(DUET_NG)
	// Function called by RepRap.cpp to enable PanelDue by default in the Duet 2 SBC build
	void SetAux0CommsProperties(uint32_t mode) const noexcept;
#endif

#if SUPPORT_REMOTE_COMMANDS
	void SwitchToExpansionMode() noexcept;
	void SetRemotePrinting(bool isPrinting) noexcept { isRemotePrinting = isPrinting; }
#endif

	static constexpr const char *AllowedAxisLetters =
#if defined(DUET3)
						"XYZUVWABCDabcdefghijklmnopqrstuvwxyz";
#else
						"XYZUVWABCDabcdef";
#endif

	// Standard macro filenames
#define DEPLOYPROBE		"deployprobe"
#define RETRACTPROBE	"retractprobe"
#define FILAMENT_ERROR	"filament-error"
#define TPRE			"tpre"
#define TPOST			"tpost"
#define TFREE			"tfree"

	static constexpr const char* CONFIG_FILE = "config.g";
	static constexpr const char* CONFIG_BACKUP_FILE = "config.g.bak";
	static constexpr const char* BED_EQUATION_G = "bed.g";
	static constexpr const char* MESH_G = "mesh.g";
	static constexpr const char* PAUSE_G = "pause.g";
	static constexpr const char* RESUME_G = "resume.g";
	static constexpr const char* CANCEL_G = "cancel.g";
	static constexpr const char* START_G = "start.g";
	static constexpr const char* STOP_G = "stop.g";
	static constexpr const char* CONFIG_OVERRIDE_G = "config-override.g";
	static constexpr const char* DefaultHeightMapFile = "heightmap.csv";
	static constexpr const char* LOAD_FILAMENT_G = "load.g";
	static constexpr const char* CONFIG_FILAMENT_G = "config.g";
	static constexpr const char* UNLOAD_FILAMENT_G = "unload.g";
	static constexpr const char* RESUME_AFTER_POWER_FAIL_G = "resurrect.g";
	static constexpr const char* RESUME_PROLOGUE_G = "resurrect-prologue.g";
	static constexpr const char* FILAMENT_CHANGE_G = "filament-change.g";
	static constexpr const char* DAEMON_G = "daemon.g";
	static constexpr const char* RUNONCE_G = "runonce.g";
#if SUPPORT_PROBE_POINTS_FILE
	static constexpr const char* DefaultProbeProbePointsFile = "probePoints.csv";
#endif

private:
	GCodes(const GCodes&) = delete;

	// Resources that can be locked.
	// To avoid deadlock, if you need multiple resources then you must lock them in increasing numerical order.
	typedef uint32_t Resource;
	static const Resource MoveResourceBase = 0;										// Movement system and associated variables
	static const Resource FileSystemResource = MoveResourceBase + NumMovementSystems;	// Non-sharable parts of the file system
	static const size_t NumResources = FileSystemResource + 1;

	static_assert(NumResources <= sizeof(Resource) * CHAR_BIT, "Too many resources to keep a bitmap of them in class GCodeMachineState");

	// Codes passed to DoFileMacro
	static constexpr int ToolChangeMacroCode = -1;								// A macro that is being called because of a tool change
	static constexpr int SystemHelperMacroCode = -2;							// Another system macro that is being called to help execute the current command
	static constexpr int AsyncSystemMacroCode = -3;								// A macro that is not being executed as part of a commend being executed, e.g. due to a trigger, filament out etc.

	bool LockResource(const GCodeBuffer& gb, Resource r) noexcept;				// Lock the resource, returning true if success
	bool LockFileSystem(const GCodeBuffer& gb) noexcept;						// Lock the unshareable parts of the file system
	bool LockMovement(const GCodeBuffer& gb) noexcept;							// Lock the movement system we are using
	bool LockMovement(const GCodeBuffer& gb, MovementSystemNumber msNumber) noexcept;	// Lock a particular movement system
	bool LockAllMovement(const GCodeBuffer& gb) noexcept;						// Lock all movement systems
	void GrabResource(const GCodeBuffer& gb, Resource r) noexcept;				// Grab a resource even if it is already owned
	void GrabMovement(const GCodeBuffer& gb) noexcept;							// Grab all movement locks even if they are already owned
	void UnlockResource(const GCodeBuffer& gb, Resource r) noexcept;			// Unlock the resource if we own it
	void UnlockMovement(const GCodeBuffer& gb) noexcept;						// Unlock the movement system we are using, if we own it
	void UnlockMovement(const GCodeBuffer& gb, MovementSystemNumber msNumber) noexcept;	// Unlock a particular movement system, if we own it
#if SUPPORT_ASYNC_MOVES
	void UnlockMovementFrom(const GCodeBuffer& gb, MovementSystemNumber firstMsNumber) noexcept;	// Release movement locks greater or equal to than the specified one
#endif

	bool SpinGCodeBuffer(GCodeBuffer& gb) noexcept;								// Do some work on an input channel
	bool StartNextGCode(GCodeBuffer& gb, const StringRef& reply) noexcept;		// Fetch a new or old GCode and process it
	void RunStateMachine(GCodeBuffer& gb, const StringRef& reply) noexcept;		// Execute a step of the state machine
	void DoStraightManualProbe(GCodeBuffer& gb, const StraightProbeSettings& sps);

	void StartPrinting(bool fromStart) noexcept;								// Start printing the file already selected
	void StopPrint(GCodeBuffer *gbp, StopPrintReason reason) noexcept;			// Stop the current print

	bool DoFilePrint(GCodeBuffer& gb, const StringRef& reply) noexcept;					// Get G Codes from a file and print them
	bool DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, int codeRunning, VariableSet& initialVariables) noexcept;
	bool DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, int codeRunning) noexcept;	// Run a GCode macro file, optionally report error if not found
	bool DoFileMacroWithParameters(GCodeBuffer& gb, const char* fileName, bool reportMissing, int codeRunning) THROWS(GCodeException);

	void FileMacroCyclesReturn(GCodeBuffer& gb) noexcept;								// End a macro

	bool ActOnCode(GCodeBuffer& gb, const StringRef& reply) noexcept;					// Do a G, M or T Code
	bool HandleGcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Do a G code
	bool HandleMcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Do an M code
	bool HandleTcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Do a T code
	bool HandleQcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Do an internal code
	bool HandleResult(GCodeBuffer& gb, GCodeResult rslt, const StringRef& reply, OutputBuffer *outBuf) noexcept
		pre(outBuf == nullptr || rslt == GCodeResult::ok);

	void HandleReply(GCodeBuffer& gb, OutputBuffer *reply) noexcept;
	void HandleReplyPreserveResult(GCodeBuffer& gb, GCodeResult rslt, const char *reply) noexcept;	// Handle G-Code replies

	GCodeResult TryMacroFile(GCodeBuffer& gb) THROWS(GCodeException);								// Try to find a macro file that implements a G or M command

	bool DoStraightMove(GCodeBuffer& gb, bool isCoordinated) THROWS(GCodeException) SPEED_CRITICAL;	// Execute a straight move
	bool DoArcMove(GCodeBuffer& gb, bool clockwise) THROWS(GCodeException)							// Execute an arc move
		pre(segmentsLeft == 0; resourceOwners[MoveResource] == &gb);
	void FinaliseMove(GCodeBuffer& gb, MovementState& ms) noexcept;									// Adjust the move parameters to account for segmentation and/or part of the move having been done already
	bool CheckEnoughAxesHomed(AxesBitmap axesToMove) noexcept;										// Check that enough axes have been homed
	bool TravelToStartPoint(GCodeBuffer& gb) noexcept;												// Set up a move to travel to the resume point

	GCodeResult DoDwell(GCodeBuffer& gb) THROWS(GCodeException);														// Wait for a bit
	GCodeResult DoHome(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);									// Home some axes
	GCodeResult SetOrReportOffsets(GCodeBuffer& gb, const StringRef& reply, int code) THROWS(GCodeException);			// Deal with a G10/M568
	GCodeResult SetPositions(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);							// Deal with a G92
	GCodeResult StraightProbe(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);							// Deal with a G38.x
	GCodeResult DoDriveMapping(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);							// Deal with a M584
	GCodeResult ProbeTool(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);								// Deal with a M585
	GCodeResult FindCenterOfCavity(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);						// Deal with a M675
	GCodeResult SetDateTime(GCodeBuffer& gb,const StringRef& reply) THROWS(GCodeException);								// Deal with a M905
	GCodeResult SavePosition(GCodeBuffer& gb,const StringRef& reply) THROWS(GCodeException);							// Deal with G60
	GCodeResult ConfigureDriver(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);						// Deal with M569
	GCodeResult ConfigureLocalDriver(GCodeBuffer& gb, const StringRef& reply, uint8_t drive) THROWS(GCodeException)
		pre(drive < platform.GetNumActualDirectDrivers());																// Deal with M569 for one local driver
	GCodeResult ConfigureLocalDriverBasicParameters(GCodeBuffer& gb, const StringRef& reply, uint8_t drive) THROWS(GCodeException)
		pre(drive < platform.GetNumActualDirectDrivers());																// Deal with M569.0 for one local driver
	GCodeResult ConfigureAccelerations(GCodeBuffer&gb, const StringRef& reply) THROWS(GCodeException);					// process M204
	GCodeResult DoMessageBox(GCodeBuffer&gb, const StringRef& reply) THROWS(GCodeException);							// process M291
	GCodeResult AcknowledgeMessage(GCodeBuffer&gb, const StringRef& reply) THROWS(GCodeException);						// process M292

#if SUPPORT_ACCELEROMETERS
	GCodeResult ConfigureAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);					// Deal with M955
	GCodeResult StartAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);						// Deal with M956
#endif
#if SUPPORT_CAN_EXPANSION
	GCodeResult StartClosedLoopDataCollection(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// Deal with M569.5
#endif

	bool SetupM675ProbingMove(GCodeBuffer& gb, bool towardsMin) noexcept;
	void SetupM675BackoffMove(GCodeBuffer& gb, float position) noexcept;
	bool SetupM585ProbingMove(GCodeBuffer& gb) noexcept;
	size_t FindAxisLetter(GCodeBuffer& gb) THROWS(GCodeException);									// Search for and return an axis, throw if none found

	bool ProcessWholeLineComment(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Process a whole-line comment

	void LoadExtrusionAndFeedrateFromGCode(GCodeBuffer& gb, MovementState& ms, bool isPrintingMove) THROWS(GCodeException);	// Set up the extrusion of a move

	bool Push(GCodeBuffer& gb, bool withinSameFile) noexcept;										// Push feedrate etc on the stack
	void Pop(GCodeBuffer& gb, bool withinSameFile) noexcept;										// Pop feedrate etc
	void DisableDrives() noexcept;																	// Turn the motors off
	bool SendConfigToLine();																		// Deal with M503

	GCodeResult OffsetAxes(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// Set/report offsets

	GCodeResult GetSetWorkplaceCoordinates(GCodeBuffer& gb, const StringRef& reply, bool compute) THROWS(GCodeException);	// Set workspace coordinates
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteWorkplaceCoordinates(FileStore *f) const noexcept;
#endif

	ReadLockedPointer<Tool> GetSpecifiedOrCurrentTool(GCodeBuffer& gb) THROWS(GCodeException);
	GCodeResult ManageTool(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Create a new tool definition
	void SetToolHeaters(const GCodeBuffer& gb, Tool *tool, float temperature) THROWS(GCodeException);	// Set all a tool's heaters active and standby temperatures, for M104/M109
	bool ToolHeatersAtSetTemperatures(const Tool *tool, bool waitWhenCooling, float tolerance, bool waitOnFault) const noexcept;
																							// Wait for the heaters associated with the specified tool to reach their set temperatures
	void GenerateTemperatureReport(const GCodeBuffer& gb, const StringRef& reply) const noexcept;	// Store a standard-format temperature report in reply
	OutputBuffer *GenerateJsonStatusResponse(int type, int seq, ResponseSource source) const noexcept;	// Generate a M408 response
	void CheckReportDue(GCodeBuffer& gb, const StringRef& reply) const noexcept;			// Check whether we need to report temperatures or status

	void RestorePosition(const RestorePoint& rp, GCodeBuffer *gb) noexcept;					// Restore user position from a restore point

	void UpdateCurrentUserPosition(const GCodeBuffer& gb) noexcept;							// Get the machine position from the Move class and transform it to the user position
	void UpdateUserPositionFromMachinePosition(const GCodeBuffer& gb, MovementState& ms) noexcept;	// Update the user position from the machine position
	void ToolOffsetTransform(MovementState& ms, AxesBitmap explicitAxes = AxesBitmap()) const noexcept;
																							// Convert user coordinates to head reference point coordinates
	void ToolOffsetTransform(const MovementState& ms, const float coordsIn[MaxAxes], float coordsOut[MaxAxes], AxesBitmap explicitAxes = AxesBitmap()) const noexcept;
																							// Convert user coordinates to head reference point coordinates
	void ToolOffsetInverseTransform(MovementState& ms) const noexcept;						// Convert head reference point coordinates to user coordinates
	void ToolOffsetInverseTransform(const MovementState& ms, const float coordsIn[MaxAxes], float coordsOut[MaxAxes]) const noexcept;
																							// Convert head reference point coordinates to user coordinates
	// Tool management
	void ReportToolTemperatures(const StringRef& reply, const Tool *tool, bool includeNumber) const noexcept;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteToolSettings(FileStore *f, const StringRef& buf) const noexcept;				// save some information for the resume file
	bool WriteToolParameters(FileStore *f, const bool forceWriteOffsets) const noexcept;	// save some information in config-override.g
#endif

	GCodeResult RetractFilament(GCodeBuffer& gb, bool retract) THROWS(GCodeException);			// Retract or un-retract filaments
	GCodeResult LoadFilament(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Load the specified filament into a tool
	GCodeResult UnloadFilament(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Unload the current filament from a tool
	bool ChangeMicrostepping(size_t axisOrExtruder, unsigned int microsteps, bool interp, const StringRef& reply) const noexcept; // Change microstepping on the specified drive
	void CheckTriggers() noexcept;															// Check for and execute triggers
	void DoEmergencyStop() noexcept;														// Execute an emergency stop

	bool DoSynchronousPause(GCodeBuffer& gb, PrintPausedReason reason, GCodeState newState) noexcept;	// Pause the print due to a command in the file itself
	bool DoAsynchronousPause(GCodeBuffer& gb, PrintPausedReason reason, GCodeState newState) noexcept;	// Pause the print returning true if successful, false if we can't yet
	void CheckForDeferredPause(GCodeBuffer& gb) noexcept;									// Check if a pause is pending, action it if so
	void ProcessEvent(GCodeBuffer& gb) noexcept;											// Start processing a new event

#if HAS_VOLTAGE_MONITOR || HAS_SMART_DRIVERS
	bool DoEmergencyPause() noexcept;														// Do an emergency pause following loss of power or a motor stall
#endif

	// Bed probing
	GCodeResult DefineGrid(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);	// Define the probing grid, returning true if error
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	GCodeResult LoadHeightMap(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Load the height map from file
	bool TrySaveHeightMap(const char *filename, const StringRef& reply) const noexcept;				// Save the height map to the specified file
	GCodeResult SaveHeightMap(GCodeBuffer& gb, const StringRef& reply) const THROWS(GCodeException);	// Save the height map to the file specified by P parameter
# if SUPPORT_PROBE_POINTS_FILE
	GCodeResult LoadProbePointsMap(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Load map of reachable probe points from file
	void ClearProbePointsInvalid() noexcept;
# endif
#endif
	void ClearBedMapping() noexcept;																// Stop using bed compensation
	bool IsUsingMeshCompensation(const MovementState& ms, ParameterLettersBitmap axesMentioned) const noexcept;	// Test whether a set of axes and a current Z height involves mesh bed compensation
	GCodeResult ProbeGrid(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// Start probing the grid, returning true if we didn't because of an error
	ReadLockedPointer<ZProbe> SetZProbeNumber(GCodeBuffer& gb, char probeLetter) THROWS(GCodeException);	// Set up currentZProbeNumber and return the probe
	GCodeResult ExecuteG30(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// Probes at a given position - see the comment at the head of the function itself
	void InitialiseTaps(bool fastThenSlow) noexcept;												// Set up to do the first of a possibly multi-tap probe

	GCodeResult ConfigureTrigger(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Handle M581
	GCodeResult CheckTrigger(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Handle M582
	GCodeResult SendI2c(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);			// Handle M260
	GCodeResult ReceiveI2c(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);			// Handle M261
	GCodeResult WaitForPin(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);			// Handle M577
	GCodeResult RaiseEvent(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);			// Handle M957

	// Object cancellation support
	GCodeResult HandleM486(GCodeBuffer& gb, const StringRef &reply, OutputBuffer*& buf) THROWS(GCodeException);
	void StartObject(GCodeBuffer& gb, const char *_ecv_array label) noexcept;
	void StopObject(GCodeBuffer& gb) noexcept;
	void ChangeToObject(GCodeBuffer& gb, int i) noexcept;

#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES || HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	GCodeResult UpdateFirmware(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);		// Handle M997
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	GCodeResult SimulateFile(GCodeBuffer& gb, const StringRef &reply, const StringRef& file, bool updateFile) THROWS(GCodeException);	// Handle M37 to simulate a whole file
	GCodeResult ChangeSimulationMode(GCodeBuffer& gb, const StringRef &reply, SimulationMode newSimMode) THROWS(GCodeException);		// Handle M37 to change the simulation mode
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	GCodeResult WriteConfigOverrideFile(GCodeBuffer& gb, const StringRef& reply) const noexcept; // Write the config-override file
	bool WriteConfigOverrideHeader(FileStore *f) const noexcept;				// Write the config-override header
#endif

	void CheckFinishedRunningConfigFile(GCodeBuffer& gb) noexcept;				// Copy the feed rate etc. from the daemon to the input channels

	MessageType GetMessageBoxDevice(GCodeBuffer& gb) const;						// Decide which device to display a message box on

	// Z probe
	void DoManualProbe(GCodeBuffer&, const char *message, const char *title, const AxesBitmap); // Do manual probe in arbitrary direction
	void DoManualBedProbe(GCodeBuffer& gb);										// Do a manual bed probe
	void DeployZProbe(GCodeBuffer& gb) noexcept;
	void RetractZProbe(GCodeBuffer& gb) noexcept;
	void CheckIfMoreTapsNeeded(GCodeBuffer& gb, const ZProbe& zp) noexcept;		// Decide whether we have probed the current point sufficient times

	void AppendAxes(const StringRef& reply, AxesBitmap axes) const noexcept;	// Append a list of axes to a string

	void EndSimulation(GCodeBuffer *null gb) noexcept;							// Restore positions etc. when exiting simulation mode

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	void SaveResumeInfo(bool wasPowerFailure) noexcept;
	bool SaveMoveStateResumeInfo(const MovementState& ms, FileStore * const f, const char *printingFilename, const StringRef& buf) noexcept;
#endif

	void NewSingleSegmentMoveAvailable(MovementState& ms) noexcept;				// Flag that a new move is available
	void NewMoveAvailable(MovementState& ms) noexcept;							// Flag that a new move is available

	void SetMoveBufferDefaults(MovementState& ms) noexcept;						// Set up default values in the move buffer
	void ChangeExtrusionFactor(unsigned int extruder, float factor) noexcept;	// Change a live extrusion factor

	MovementState& GetMovementState(const GCodeBuffer& gb) noexcept;			// Get a reference to the movement state associated with the specified GCode buffer

#if SUPPORT_ASYNC_MOVES
	GCodeResult SelectMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Handle M596
	GCodeResult CollisionAvoidance(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// Handle M597
	GCodeResult SyncMovementSystems(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Handle M598
	GCodeResult ForkInputReader(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// Handle M606
	GCodeResult ExecuteM400(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);				// Handle M400
	void AllocateAxes(const GCodeBuffer& gb, MovementState& ms, AxesBitmap axes, ParameterLettersBitmap axLetters) THROWS(GCodeException);	// allocate axes to a movement state
	void AllocateAxisLetters(const GCodeBuffer& gb, MovementState& ms, ParameterLettersBitmap axLetters) THROWS(GCodeException);
																											// allocate axes by letter
	void AllocateAxesDirectFromLetters(const GCodeBuffer& gb, MovementState& ms, ParameterLettersBitmap axLetters) THROWS(GCodeException);
																											// allocate axes by letter for a special move
	bool IsAxisFree(unsigned int axis) const noexcept;														// test whether an axis is unowned
	bool DoSync(GCodeBuffer& gb) noexcept;																	// sync with the other stream returning true if done, false if we need to wait for it
	bool SyncWith(GCodeBuffer& thisGb, const GCodeBuffer& otherGb) noexcept;								// synchronise motion systems
	void UpdateAllCoordinates(const GCodeBuffer& gb) noexcept;
#endif

#if SUPPORT_KEEPOUT_ZONES
	GCodeResult DefineKeepoutZone(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// Handle M599
#endif

#if SUPPORT_COORDINATE_ROTATION
	GCodeResult HandleG68(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Handle G68
#endif

#if SUPPORT_DIRECT_LCD
	int GetHeaterNumber(unsigned int itemNumber) const noexcept;
#endif
	Pwm_t ConvertLaserPwm(float reqVal) const noexcept;

#if HAS_AUX_DEVICES
# if ALLOW_ARBITRARY_PANELDUE_PORT
	uint8_t serialChannelForPanelDueFlashing;
# else
	static constexpr uint8_t serialChannelForPanelDueFlashing = 1;
# endif
	static bool emergencyStopCommanded;
	static void CommandEmergencyStop(AsyncSerial *p) noexcept;
#endif

	Platform& platform;													// The RepRap machine

#if HAS_NETWORKING || HAS_SBC_INTERFACE
	NetworkGCodeInput* httpInput;										// These cache incoming G-codes...
	NetworkGCodeInput* telnetInput;										// ...
#endif

	GCodeBuffer* gcodeSources[NumGCodeChannels];						// The various sources of gcodes

	GCodeBuffer* HttpGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::HTTP)]; }
	GCodeBuffer* TelnetGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Telnet)]; }
	GCodeBuffer* FileGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::File)]; }
	GCodeBuffer* UsbGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::USB)]; }
	GCodeBuffer* AuxGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux)]; }					// This one is for the PanelDue on the async serial interface
	GCodeBuffer* TriggerGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Trigger)]; }			// Used for executing config.g and trigger macro files
	GCodeBuffer* QueuedGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Queue)]; }
	GCodeBuffer* LcdGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::LCD)]; }					// This one for the 12864 LCD
	GCodeBuffer* SpiGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::SBC)]; }
	GCodeBuffer* DaemonGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Daemon)]; }
	GCodeBuffer* Aux2GCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux2)]; }					// This one is reserved for the second async serial interface
	GCodeBuffer* AutoPauseGCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Autopause)]; }		// GCode state machine used to run macros on power fail, heater faults and filament out
#if SUPPORT_ASYNC_MOVES
	GCodeBuffer* File2GCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::File2)]; }
	GCodeBuffer* Queue2GCode() const noexcept { return gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Queue2)]; }
	GCodeBuffer* GetFileGCode(unsigned int msNumber) const noexcept;
#else
	GCodeBuffer* GetFileGCode(unsigned int msNumber) const noexcept { return FileGCode(); }
#endif

	size_t nextGcodeSource;												// The one to check next, using round-robin scheduling

	const GCodeBuffer* resourceOwners[NumResources];					// Which gcode buffer owns each resource

	StraightProbeSettings straightProbeSettings;						// G38 straight probe settings
	union
	{
		M675Settings m675Settings;
		M585Settings m585Settings;
	};

	MachineType machineType;					// whether FFF, laser or CNC
	bool active;								// Live and running?
	bool stopped;								// set after emergency stop has been executed
	const char *_ecv_array null deferredPauseCommandPending;
	PauseState pauseState;						// whether the machine is running normally or is pausing, paused or resuming
	bool runningConfigFile;						// We are running config.g during the startup process
	bool doingToolChange;						// We are running tool change macros

#if HAS_VOLTAGE_MONITOR
	bool isPowerFailPaused;						// true if the print was paused automatically because of a power failure
	char *_ecv_array null powerFailScript;		// the commands run when there is a power failure
#endif

	// The following contain the details of moves that the Move module fetches
	MovementState moveStates[NumMovementSystems];	// Move details

	size_t numTotalAxes;						// How many axes we have
	size_t numVisibleAxes;						// How many axes are visible
	size_t numExtruders;						// How many extruders we have, or may have
	float axisScaleFactors[MaxAxes];			// Scale XYZ coordinates by this factor
	float rawExtruderTotalByDrive[MaxExtruders]; // Extrusion amount in the last G1 command with an E parameter when in absolute extrusion mode
	float rawExtruderTotal;						// Total extrusion amount fed to Move class since starting print, before applying extrusion factor, summed over all drives

	float workplaceCoordinates[NumCoordinateSystems][MaxAxes];	// Workplace coordinate offsets

#if SUPPORT_COORDINATE_ROTATION
	float g68Angle;								// the G68 rotation angle in radians
	float g68Centre[2];							// the XY coordinates of the centre to rotate about
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	FileData fileToPrint;						// The next file to print
#endif

	ParameterLettersBitmap allAxisLetters;		// Which axis letters are in use
	char axisLetters[MaxAxes + 1];				// The names of the axes, with a null terminator
	bool limitAxes;								// Don't think outside the box
	bool noMovesBeforeHoming;					// Don't allow movement prior to homing the associates axes

	AxesBitmap toBeHomed;						// Bitmap of axes still to be homed
	AxesBitmap axesHomed;						// Bitmap of which axes have been homed
	AxesBitmap axesVirtuallyHomed;				// same as axesHomed except all bits are set when simulating

	float extrusionFactors[MaxExtruders];		// extrusion factors (normally 1.0)
	float filamentDiameters[MaxExtruders];		// diameter of the filament in each extruder
	float volumetricExtrusionFactors[MaxExtruders]; // Volumetric extrusion factors
	float currentBabyStepOffsets[MaxAxes];		// The accumulated axis offsets due to baby stepping requests

	// Z probe
	GridDefinition defaultGrid;					// The grid defined by the M557 command in config.g
	int32_t g30ProbePointIndex;					// the index of the point we are probing (G30 P parameter), or -1 if none
	int g30SValue;								// S parameter in the G30 command, or -2 if there wasn't one
	float g30HValue;							// H parameter in the G30 command, or 0.0 if there wasn't on
	float g30zHeightError;						// the height error last time we probed
	float g30PrevHeightError;					// the height error the previous time we probed
	float g30zHeightErrorSum;					// the sum of the height errors for the current probe point
	float g30zHeightErrorLowestDiff;			// the lowest difference we have seen between consecutive readings

	// Scanning Z probe calibration
	float calibrationStartingHeight;			// how much above and below the trigger height we go when calibrating a scanning probe
	float heightChangePerPoint;					// the height different between adjacent points
	size_t numPointsToCollect;					// how many readings to take
	size_t numCalibrationReadingsTaken;			// the number of calibration readings we have taken
	int32_t calibrationReadings[MaxScanningProbeCalibrationPoints];

	uint32_t lastProbedTime;					// time in milliseconds that the probe was last triggered
	volatile bool zProbeTriggered;				// Set by the step ISR when a move is aborted because the Z probe is triggered
	size_t gridAxis0Index, gridAxis1Index;		// Which grid probe point is next
	size_t lastAxis0Index;						// The last grid probe point in this row to scan
	bool doingManualBedProbe;					// true if we are waiting for the user to jog the nozzle until it touches the bed
	bool hadProbingError;						// true if there was an error probing the last point
	GCodeResult scanningResult;					// set if we had a bad value when scanning the bed
	bool zDatumSetByProbing;					// true if the Z position was last set by probing, not by an endstop switch or by G92
	int8_t tapsDone;							// how many times we tapped the current point at the slow speed
	bool acceptReading;							// true if we are going to accept the reading of the previous tap
	uint8_t currentZProbeNumber;				// which Z probe a G29 or G30 command is using
#if SUPPORT_PROBE_POINTS_FILE
	bool probePointsFileLoaded;					// true if we loaded a probe point file since we last cleared the grid
#endif

	// Simulation and print time
	float simulationTime;						// Accumulated simulation time
	uint32_t lastDuration;						// Time or simulated time of the last successful print or simulation, in seconds
	SimulationMode simulationMode;				// see description of enum SimulationMode
	bool exitSimulationWhenFileComplete;		// true if simulating a file
	bool updateFileWhenSimulationComplete;		// true if simulated time should be appended to the file

	// Triggers
	TriggerItem triggers[MaxTriggers];				// Trigger conditions
	TriggerNumbersBitmap triggersPending;		// Bitmap of triggers pending but not yet executed

	// Firmware update
	Bitmap<uint8_t> firmwareUpdateModuleMap;	// Bitmap of firmware modules to be updated
	bool isFlashing;							// Is a new firmware binary going to be flashed?
#if SUPPORT_PANELDUE_FLASH
	bool isFlashingPanelDue;					// Are we in the process of flashing PanelDue?
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	// SHA1 hashing
	FileStore *fileBeingHashed;
	SHA1Context hash;
	bool StartHash(const char* filename) noexcept;
	GCodeResult AdvanceHash(const StringRef &reply) noexcept;
#endif

	// Laser
	float laserMaxPower;
	bool laserPowerSticky;						// true if G1 S parameters are remembered across G1 commands

	// Object cancellation
	ObjectTracker buildObjects;

	// Misc
	uint32_t lastWarningMillis;					// When we last sent a warning message for things that can happen very often
	AxesBitmap axesToSenseLength;				// The axes on which we are performing axis length sensing

#if SUPPORT_ASYNC_MOVES
	CollisionAvoider collisionChecker;			// currently we support just one collision avoider
	MovementSystemNumber pausedMovementSystemNumber;
#endif
#if SUPPORT_KEEPOUT_ZONES
	KeepoutZone keepoutZone;					// currently we support just one keepout zone
#endif

#if HAS_MASS_STORAGE
	static constexpr uint32_t SdTimingByteIncrement = 8 * 1024;	// how many timing bytes we write at a time
	static constexpr const char *TimingFileName = "test.tst";	// the name of the file we write
	FileStore *sdTimingFile;					// file handle being used for SD card write timing
	uint32_t timingBytesRequested;				// how many bytes we were asked to write
	uint32_t timingBytesWritten;				// how many timing bytes we have written so far
	uint32_t timingStartMillis;
#endif

#if SUPPORT_REMOTE_COMMANDS
	bool isRemotePrinting;
#endif

	bool displayNoToolWarning;					// True if we need to display a 'no tool selected' warning
	bool m501SeenInConfigFile;					// true if M501 was executed from config.g
	bool daemonRunning;
	char filamentToLoad[FilamentNameLength];	// Name of the filament being loaded

	static constexpr const float MinServoPulseWidth = 544.0, MaxServoPulseWidth = 2400.0;

	static constexpr int8_t ObjectModelAuxStatusReportType = 100;		// A non-negative value distinct from any M408 report type
};

// Get the total baby stepping offset for an axis
inline float GCodes::GetTotalBabyStepOffset(size_t axis) const noexcept
{
	return currentBabyStepOffsets[axis];
}

// Lock a particular movement system
inline bool GCodes::LockMovement(const GCodeBuffer& gb, MovementSystemNumber msNumber) noexcept
{
	return LockResource(gb, MoveResourceBase + msNumber);
}

// Unlock a particular movement system, if we own it
inline void GCodes::UnlockMovement(const GCodeBuffer& gb, MovementSystemNumber msNumber) noexcept
{
	return UnlockResource(gb, MoveResourceBase + msNumber);
}

#if !SUPPORT_ASYNC_MOVES

inline bool GCodes::LockCurrentMovementSystemAndWaitForStandstill(GCodeBuffer& gb) noexcept
{
	return LockMovementSystemAndWaitForStandstill(gb, 0);
}

inline bool GCodes::LockAllMovementSystemsAndWaitForStandstill(GCodeBuffer& gb) noexcept
{
	return LockMovementSystemAndWaitForStandstill(gb, 0);
}

// Get a reference to the movement state associated with the specified GCode buffer
inline MovementState& GCodes::GetMovementState(const GCodeBuffer& gb) noexcept
{
	return moveStates[0];
}

inline const MovementState& GCodes::GetConstMovementState(const GCodeBuffer& gb) const noexcept
{
	return moveStates[0];
}

inline const MovementState& GCodes::GetCurrentMovementState(const ObjectExplorationContext& context) const noexcept
{
	return moveStates[0];
}

// Lock the movement system we are using
inline bool GCodes::LockMovement(const GCodeBuffer& gb) noexcept
{
	return LockResource(gb, MoveResourceBase);
}

// Lock all movement systems
inline bool GCodes::LockAllMovement(const GCodeBuffer& gb) noexcept
{
	return LockResource(gb, MoveResourceBase);
}

// Unlock the movement system we are using, if we own it
inline void GCodes::UnlockMovement(const GCodeBuffer& gb) noexcept
{
	return UnlockResource(gb, MoveResourceBase);
}

// Grab all movement locks even if they are already owned
inline void GCodes::GrabMovement(const GCodeBuffer& gb) noexcept
{
	GrabResource(gb, MoveResourceBase);
}

#endif

//*****************************************************************************************************

#endif
