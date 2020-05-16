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
#include "RepRap.h"			// for type ResponseSource
#include "GCodeResult.h"
#include "ObjectTracker.h"
#include "Movement/RawMove.h"
#include "Libraries/sha1/sha1.h"
#include "Platform.h"		// for type EndStopHit
#include "GCodeChannel.h"
#include "GCodeInput.h"
#include "Trigger.h"
#include "Tools/Filament.h"
#include "FilamentMonitors/FilamentMonitor.h"
#include "RestorePoint.h"
#include "Movement/BedProbing/Grid.h"

const char feedrateLetter = 'F';						// GCode feedrate
const char extrudeLetter = 'E'; 						// GCode extrude

// Bits for T-code P-parameter to specify which macros are supposed to be run
constexpr uint8_t TFreeBit = 1 << 0;
constexpr uint8_t TPreBit = 1 << 1;
constexpr uint8_t TPostBit = 1 << 2;
constexpr uint8_t DefaultToolChangeParam = TFreeBit | TPreBit | TPostBit;

// Machine type enumeration. The numeric values must be in the same order as the corresponding M451..M453 commands.
enum class MachineType : uint8_t
{
	fff = 0,
	laser = 1,
	cnc = 2
};

enum class PauseReason
{
	user,			// M25 command received
	gcode,			// M25 or M226 command encountered in the file being printed
	filamentChange,	// M600 command
	trigger,		// external switch
	heaterFault,	// heater fault detected
	filament,		// filament monitor
#if HAS_SMART_DRIVERS
	stall,			// motor stall detected
#endif
#if HAS_VOLTAGE_MONITOR
	lowVoltage		// VIN voltage dropped below configured minimum
#endif
};

// Keep this in sync with PrintStopReason in Linux/MessageFormats.h
enum class StopPrintReason
{
	normalCompletion,
	userCancelled,
	abort
};

//****************************************************************************************************

class LinuxInterface;
class StraightProbeSettings;

// The GCode interpreter

class GCodes
{
public:
	GCodes(Platform& p) noexcept;
	void Spin() noexcept;														// Called in a tight loop to make this class work
	void Init() noexcept;														// Set it up
	void Exit() noexcept;														// Shut it down
	void Reset() noexcept;														// Reset some parameter to defaults
	bool ReadMove(RawMove& m) noexcept;											// Called by the Move class to get a movement set by the last G Code
	void ClearMove() noexcept;
#if HAS_MASS_STORAGE
	bool QueueFileToPrint(const char* fileName, const StringRef& reply) noexcept;	// Open a file of G Codes to run
#endif
	void StartPrinting(bool fromStart) noexcept;								// Start printing the file already selected
	void AbortPrint(GCodeBuffer& gb) noexcept;									// Cancel any print in progress
	void GetCurrentCoordinates(const StringRef& s) const noexcept;				// Write where we are into a string
	bool DoingFileMacro() const noexcept;										// Is a macro file being processed?
	bool WaitingForAcknowledgement() const noexcept;							// Is an input waiting for a message to be acknowledged?

	FilePosition GetFilePosition() const noexcept;								// Return the current position of the file being printed in bytes
	void Diagnostics(MessageType mtype) noexcept;								// Send helpful information out

	bool RunConfigFile(const char* fileName) noexcept;							// Start running the config file
	bool IsDaemonBusy() const noexcept;											// Return true if the daemon is busy running config.g or a trigger file

	bool IsAxisHomed(unsigned int axis) const noexcept							// Has the axis been homed?
		{ return axesHomed.IsBitSet(axis); }
	void SetAxisIsHomed(unsigned int axis) noexcept;							// Tell us that the axis is now homed
	void SetAxisNotHomed(unsigned int axis) noexcept;							// Tell us that the axis is not homed
	void SetAllAxesNotHomed() noexcept;											// Flag all axes as not homed

	float GetSpeedFactor() const noexcept { return speedFactor; }				// Return the current speed factor as a fraction
	float GetExtrusionFactor(size_t extruder) noexcept;							// Return the current extrusion factor for the specified extruder
#if SUPPORT_12864_LCD
	void SetSpeedFactor(float factor) noexcept;									// Set the speed factor
	void SetExtrusionFactor(size_t extruder, float factor) noexcept;			// Set an extrusion factor for the specified extruder
#endif

	float GetRawExtruderTotalByDrive(size_t extruder) const noexcept;			// Get the total extrusion since start of print, for one drive
	float GetTotalRawExtrusion() const noexcept { return rawExtruderTotal; }	// Get the total extrusion since start of print, all drives
	float GetTotalBabyStepOffset(size_t axis) const noexcept
		pre(axis < maxAxes);
	float GetUserCoordinate(size_t axis) const noexcept;						// Get the current user coordinate in the current workspace coordinate system

#if HAS_NETWORKING
	NetworkGCodeInput *GetHTTPInput() const noexcept { return httpInput; }
	NetworkGCodeInput *GetTelnetInput() const noexcept { return telnetInput; }
#endif

	bool IsFlashing() const noexcept { return isFlashing; }						// Is a new firmware binary going to be flashed?

	bool IsPaused() const noexcept;
	bool IsPausing() const noexcept;
	bool IsResuming() const noexcept;
	bool IsRunning() const noexcept;
	bool IsReallyPrinting() const noexcept;										// Return true if we are printing from SD card and not pausing, paused or resuming
	bool IsSimulating() const noexcept { return simulationMode != 0; }
	bool IsDoingToolChange() const noexcept { return doingToolChange; }
	bool IsHeatingUp() const noexcept;											// Return true if the SD card print is waiting for a heater to reach temperature

	uint32_t GetLastDuration() const noexcept { return lastDuration; }			// The most recent print time or simulated time
	float GetSimulationTime() const noexcept { return simulationTime; }

	bool AllAxesAreHomed() const noexcept;										// Return true if all axes are homed

	void StopPrint(StopPrintReason reason) noexcept;							// Stop the current print

	void MoveStoppedByZProbe() noexcept { zProbeTriggered = true; }				// Called from the step ISR when the Z probe is triggered, causing the move to be aborted

	size_t GetTotalAxes() const noexcept { return numTotalAxes; }
	size_t GetVisibleAxes() const noexcept { return numVisibleAxes; }
	size_t GetNumExtruders() const noexcept { return numExtruders; }

	const char* GetMachineModeString() const noexcept;							// Get the name of the current machine mode

	void FilamentError(size_t extruder, FilamentSensorStatus fstat) noexcept;
	void HandleHeaterFault() noexcept;											// Respond to a heater fault

#if HAS_VOLTAGE_MONITOR
	bool LowVoltagePause() noexcept;
	bool LowVoltageResume() noexcept;
#endif

#if HAS_SMART_DRIVERS
	bool PauseOnStall(DriversBitmap stalledDrivers) noexcept;
	bool ReHomeOnStall(DriversBitmap stalledDrivers) noexcept;
#endif

	const char *GetAxisLetters() const noexcept { return axisLetters; }			// Return a null-terminated string of axis letters indexed by drive
	MachineType GetMachineType() const noexcept { return machineType; }
	bool LockMovementAndWaitForStandstill(GCodeBuffer& gb) noexcept;			// Lock movement and wait for pending moves to finish

#if SUPPORT_12864_LCD
	bool ProcessCommandFromLcd(const char *cmd) noexcept;						// Process a GCode command from the 12864 LCD returning true if the command was accepted
	float GetItemCurrentTemperature(unsigned int itemNumber) const noexcept;
	float GetItemActiveTemperature(unsigned int itemNumber) const noexcept;
	float GetItemStandbyTemperature(unsigned int itemNumber) const noexcept;
	void SetItemActiveTemperature(unsigned int itemNumber, float temp) noexcept;
	void SetItemStandbyTemperature(unsigned int itemNumber, float temp) noexcept;
#endif

	float GetMappedFanSpeed() const noexcept { return lastDefaultFanSpeed; }	// Get the mapped fan speed
	void SetMappedFanSpeed(float f) noexcept;									// Set the speeds of fans mapped for the current tool
	void HandleReply(GCodeBuffer& gb, GCodeResult rslt, const char *reply) noexcept;	// Handle G-Code replies
	void EmergencyStop() noexcept;												// Cancel everything
	bool GetLastPrintingHeight(float& height) const noexcept;					// Get the height in user coordinates of the last printing move
	bool AtxPowerControlled() const noexcept { return atxPowerControlled; }

	const GridDefinition& GetDefaultGrid() const { return defaultGrid; };		// Get the default grid definition
	void AssignGrid(float xRange[2], float yRange[2], float radius, float spacing[2]) noexcept;	// Assign the heightmap using the given parameters
	void ActivateHeightmap(bool activate) noexcept;								// (De-)Activate the height map

	int GetNewToolNumber() const noexcept { return newToolNumber; }
	size_t GetCurrentZProbeNumber() const noexcept { return currentZProbeNumber; }

	// These next two are public because they are used by class LinuxInterface
	void UnlockAll(const GCodeBuffer& gb) noexcept;								// Release all locks
	GCodeBuffer *GetGCodeBuffer(GCodeChannel channel) const noexcept { return gcodeSources[channel.ToBaseType()]; }

#if HAS_MASS_STORAGE
	GCodeResult StartSDTiming(GCodeBuffer& gb, const StringRef& reply) noexcept;	// Start timing SD card file writing
#endif

	void SavePosition(RestorePoint& rp, const GCodeBuffer& gb) const noexcept;		// Save position etc. to a restore point
	void SaveSpindleSpeeds(RestorePoint& rp) const noexcept;						// Save spindle speeds to a restore point
	void StartToolChange(GCodeBuffer& gb, int toolNum, uint8_t toolChangeParam) noexcept;

	unsigned int GetWorkplaceCoordinateSystemNumber() const noexcept { return currentCoordinateSystem + 1; }

	// This function is called by other functions to account correctly for workplace coordinates
	float GetWorkplaceOffset(size_t axis) const noexcept
	{
		return workplaceCoordinates[currentCoordinateSystem][axis];
	}

#if SUPPORT_OBJECT_MODEL
	float GetWorkplaceOffset(size_t axis, size_t workplaceNumber) const noexcept
	{
		return workplaceCoordinates[workplaceNumber][axis];
	}

	size_t GetNumInputs() const noexcept { return NumGCodeChannels; }
	const GCodeBuffer* GetInput(size_t n) const noexcept { return gcodeSources[n]; }
	const GCodeBuffer* GetInput(GCodeChannel n) const noexcept { return gcodeSources[n.RawValue()]; }
	const ObjectTracker *GetBuildObjects() const noexcept { return &buildObjects; }
	const RestorePoint *GetRestorePoint(size_t n) const pre(n < NumRestorePoints) { return &numberedRestorePoints[n]; }
	float GetVirtualExtruderPosition() const noexcept { return virtualExtruderPosition; }

# if HAS_VOLTAGE_MONITOR
	const char *GetPowerFailScript() const noexcept { return powerFailScript; }
# endif

# if SUPPORT_LASER
	// Return laser PWM in 0..1
	float GetLaserPwm() const noexcept
	{
		return (float)moveBuffer.laserPwmOrIoBits.laserPwm * (1.0/65535);
	}
# endif
#endif

	static constexpr const char *AllowedAxisLetters = "XYZUVWABCD";

	// Standard macro filenames
#define DEPLOYPROBE		"deployprobe"
#define RETRACTPROBE	"retractprobe"
#define TPRE			"tpre"
#define TPOST			"tpost"
#define TFREE			"tfree"

	static constexpr const char* CONFIG_FILE = "config.g";
	static constexpr const char* CONFIG_BACKUP_FILE = "config.g.bak";
	static constexpr const char* BED_EQUATION_G = "bed.g";
	static constexpr const char* PAUSE_G = "pause.g";
	static constexpr const char* RESUME_G = "resume.g";
	static constexpr const char* CANCEL_G = "cancel.g";
	static constexpr const char* START_G = "start.g";
	static constexpr const char* STOP_G = "stop.g";
	static constexpr const char* SLEEP_G = "sleep.g";
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
#if HAS_SMART_DRIVERS
	static constexpr const char* REHOME_G = "rehome.g";
#endif

private:
	GCodes(const GCodes&) = delete;

	enum class HeaterFaultState : uint8_t { noFault, pausePending, timing, stopping, stopped };

	// Resources that can be locked.
	// To avoid deadlock, if you need multiple resources then you must lock them in increasing numerical order.
	typedef uint32_t Resource;
	static const Resource MoveResource = 0;										// Movement system, including canned cycle variables
	static const Resource FileSystemResource = 1;								// Non-sharable parts of the file system
	static const Resource HeaterResourceBase = 2;
	static const size_t NumResources = HeaterResourceBase + 1;

	static_assert(NumResources <= sizeof(Resource) * CHAR_BIT, "Too many resources to keep a bitmap of them in class GCodeMachineState");

	bool LockResource(const GCodeBuffer& gb, Resource r) noexcept;				// Lock the resource, returning true if success
	bool LockFileSystem(const GCodeBuffer& gb) noexcept;						// Lock the unshareable parts of the file system
	bool LockMovement(const GCodeBuffer& gb) noexcept;							// Lock movement
	void GrabResource(const GCodeBuffer& gb, Resource r) noexcept;				// Grab a resource even if it is already owned
	void GrabMovement(const GCodeBuffer& gb) noexcept;							// Grab the movement lock even if it is already owned
	void UnlockResource(const GCodeBuffer& gb, Resource r) noexcept;			// Unlock the resource if we own it
	void UnlockMovement(const GCodeBuffer& gb) noexcept;						// Unlock the movement resource if we own it

	void StartNextGCode(GCodeBuffer& gb, const StringRef& reply) noexcept;		// Fetch a new or old GCode and process it
	void RunStateMachine(GCodeBuffer& gb, const StringRef& reply) noexcept;		// Execute a step of the state machine
	void DoStraightManualProbe(GCodeBuffer& gb, const StraightProbeSettings& sps);

	void DoFilePrint(GCodeBuffer& gb, const StringRef& reply) noexcept;					// Get G Codes from a file and print them
	bool DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, int codeRunning = -1) noexcept;
		// Run a GCode macro file, optionally report error if not found
	void FileMacroCyclesReturn(GCodeBuffer& gb) noexcept;								// End a macro

	bool ActOnCode(GCodeBuffer& gb, const StringRef& reply) noexcept;					// Do a G, M or T Code
	bool HandleGcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Do a G code
	bool HandleMcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Do an M code
	bool HandleTcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Do a T code
	bool HandleQcode(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Do an internal code
	bool HandleResult(GCodeBuffer& gb, GCodeResult rslt, const StringRef& reply, OutputBuffer *outBuf)
		pre(outBuf == nullptr || rslt == GCodeResult::ok) noexcept;

	void HandleReply(GCodeBuffer& gb, OutputBuffer *reply) noexcept;
	void HandleReplyPreserveResult(GCodeBuffer& gb, GCodeResult rslt, const char *reply) noexcept;	// Handle G-Code replies

	bool DoStraightMove(GCodeBuffer& gb, bool isCoordinated, const char *& err) __attribute__((hot));	// Execute a straight move
	bool DoArcMove(GCodeBuffer& gb, bool clockwise, const char *& err)				// Execute an arc move
		pre(segmentsLeft == 0; resourceOwners[MoveResource] == &gb);
	void FinaliseMove(GCodeBuffer& gb) noexcept;									// Adjust the move parameters to account for segmentation and/or part of the move having been done already
	bool CheckEnoughAxesHomed(AxesBitmap axesMoved) noexcept;						// Check that enough axes have been homed
	bool TravelToStartPoint(GCodeBuffer& gb) noexcept;								// Set up a move to travel to the resume point

	GCodeResult DoDwell(GCodeBuffer& gb) THROWS(GCodeException);									// Wait for a bit
	GCodeResult DoHome(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);				// Home some axes
	GCodeResult SetOrReportOffsets(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Deal with a G10
	GCodeResult SetPositions(GCodeBuffer& gb) THROWS(GCodeException);								// Deal with a G92
	GCodeResult StraightProbe(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Deal with a G38.x
	GCodeResult DoDriveMapping(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Deal with a M584
	GCodeResult ProbeTool(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// Deal with a M585
	GCodeResult FindCenterOfCavity(GCodeBuffer& gb, const StringRef& reply, const bool towardsMin = true) THROWS(GCodeException);	// Deal with a M675
	GCodeResult SetDateTime(GCodeBuffer& gb,const StringRef& reply) THROWS(GCodeException);			// Deal with a M905
	GCodeResult SavePosition(GCodeBuffer& gb,const StringRef& reply) THROWS(GCodeException);		// Deal with G60
	GCodeResult ConfigureDriver(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Deal with M569

	bool ProcessWholeLineComment(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Process a whole-line comment

	const char *LoadExtrusionAndFeedrateFromGCode(GCodeBuffer& gb, bool isPrintingMove);	// Set up the extrusion of a move

	bool Push(GCodeBuffer& gb, bool withinSameFile);								// Push feedrate etc on the stack
	void Pop(GCodeBuffer& gb, bool withinSameFile);									// Pop feedrate etc
	void DisableDrives() noexcept;													// Turn the motors off
	bool SendConfigToLine();														// Deal with M503

	GCodeResult OffsetAxes(GCodeBuffer& gb, const StringRef& reply);				// Set/report offsets

#if SUPPORT_WORKPLACE_COORDINATES
	GCodeResult GetSetWorkplaceCoordinates(GCodeBuffer& gb, const StringRef& reply, bool compute);	// Set workspace coordinates
# if HAS_MASS_STORAGE
	bool WriteWorkplaceCoordinates(FileStore *f) const noexcept;
# endif
#endif

	GCodeResult ManageTool(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Create a new tool definition
	void SetToolHeaters(Tool *tool, float temperature, bool both) THROWS(GCodeException);	// Set all a tool's heaters to the temperature, for M104/M109
	bool ToolHeatersAtSetTemperatures(const Tool *tool, bool waitWhenCooling, float tolerance) const noexcept;
																				// Wait for the heaters associated with the specified tool to reach their set temperatures
	void GenerateTemperatureReport(const StringRef& reply) const noexcept;		// Store a standard-format temperature report in reply
	OutputBuffer *GenerateJsonStatusResponse(int type, int seq, ResponseSource source) const noexcept;	// Generate a M408 response
	void CheckReportDue(GCodeBuffer& gb, const StringRef& reply) const;			// Check whether we need to report temperatures or status

	void RestorePosition(const RestorePoint& rp, GCodeBuffer *gb) noexcept;		// Restore user position from a restore point

	void UpdateCurrentUserPosition() noexcept;									// Get the current position from the Move class
	void ToolOffsetTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes], AxesBitmap explicitAxes = AxesBitmap()) const noexcept;
																				// Convert user coordinates to head reference point coordinates
	void ToolOffsetInverseTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes]) const noexcept;	// Convert head reference point coordinates to user coordinates
	float GetCurrentToolOffset(size_t axis) const noexcept;						// Get an axis offset of the current tool

	GCodeResult RetractFilament(GCodeBuffer& gb, bool retract);					// Retract or un-retract filaments
	GCodeResult LoadFilament(GCodeBuffer& gb, const StringRef& reply);			// Load the specified filament into a tool
	GCodeResult UnloadFilament(GCodeBuffer& gb, const StringRef& reply);		// Unload the current filament from a tool
	bool ChangeMicrostepping(size_t drive, unsigned int microsteps, bool interp, const StringRef& reply) const noexcept; // Change microstepping on the specified drive
	void CheckTriggers() noexcept;												// Check for and execute triggers
	void CheckFilament() noexcept;												// Check for and respond to filament errors
	void CheckHeaterFault() noexcept;											// Check for and respond to a heater fault, returning true if we should exit
	void DoEmergencyStop() noexcept;											// Execute an emergency stop

	void DoPause(GCodeBuffer& gb, PauseReason reason, const char *msg) noexcept	// Pause the print
		pre(resourceOwners[movementResource] = &gb);
	void CheckForDeferredPause(GCodeBuffer& gb) noexcept;						// Check if a pause is pending, action it if so

#if HAS_VOLTAGE_MONITOR || HAS_SMART_DRIVERS
	bool DoEmergencyPause() noexcept;											// Do an emergency pause following loss of power or a motor stall
#endif

	bool IsMappedFan(unsigned int fanNumber) noexcept;							// Return true if this fan number is currently being used as a print cooling fan
	void SaveFanSpeeds() noexcept;												// Save the speeds of all fans

	GCodeResult DefineGrid(GCodeBuffer& gb, const StringRef &reply);			// Define the probing grid, returning true if error
#if HAS_MASS_STORAGE
	GCodeResult LoadHeightMap(GCodeBuffer& gb, const StringRef& reply);			// Load the height map from file
	bool TrySaveHeightMap(const char *filename, const StringRef& reply) const noexcept;	// Save the height map to the specified file
	GCodeResult SaveHeightMap(GCodeBuffer& gb, const StringRef& reply) const;	// Save the height map to the file specified by P parameter
#endif
	void ClearBedMapping();														// Stop using bed compensation
	GCodeResult ProbeGrid(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Start probing the grid, returning true if we didn't because of an error
	ReadLockedPointer<ZProbe> SetZProbeNumber(GCodeBuffer& gb) THROWS(GCodeException);		// Set up currentZProbeNumber and return the probe
	GCodeResult ExecuteG30(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Probes at a given position - see the comment at the head of the function itself
	void InitialiseTaps() noexcept;												// Set up to do the first of a possibly multi-tap probe
	void SetBedEquationWithProbe(int sParam, const StringRef& reply);			// Probes a series of points and sets the bed equation

	GCodeResult ConfigureTrigger(GCodeBuffer& gb, const StringRef& reply);		// Handle M581
	GCodeResult CheckTrigger(GCodeBuffer& gb, const StringRef& reply);			// Handle M582
	GCodeResult UpdateFirmware(GCodeBuffer& gb, const StringRef &reply);		// Handle M997
	GCodeResult SendI2c(GCodeBuffer& gb, const StringRef &reply);				// Handle M260
	GCodeResult ReceiveI2c(GCodeBuffer& gb, const StringRef &reply);			// Handle M261
#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	GCodeResult SimulateFile(GCodeBuffer& gb, const StringRef &reply, const StringRef& file, bool updateFile);	// Handle M37 to simulate a whole file
	GCodeResult ChangeSimulationMode(GCodeBuffer& gb, const StringRef &reply, uint32_t newSimulationMode);		// Handle M37 to change the simulation mode
#endif
	GCodeResult WaitForPin(GCodeBuffer& gb, const StringRef &reply);			// Handle M577

#if HAS_MASS_STORAGE
	GCodeResult WriteConfigOverrideFile(GCodeBuffer& gb, const StringRef& reply) const noexcept; // Write the config-override file
	bool WriteConfigOverrideHeader(FileStore *f) const noexcept;				// Write the config-override header
#endif

	void CheckFinishedRunningConfigFile(GCodeBuffer& gb) noexcept;						// Copy the feed rate etc. from the daemon to the input channels

	MessageType GetMessageBoxDevice(GCodeBuffer& gb) const;						// Decide which device to display a message box on
	void DoManualProbe(GCodeBuffer&, const char *message, const char *title, const AxesBitmap); // Do manual probe in arbitrary direction
	void DoManualBedProbe(GCodeBuffer& gb);										// Do a manual bed probe
	void DeployZProbe(GCodeBuffer& gb, int code) noexcept;
	void RetractZProbe(GCodeBuffer& gb, int code) noexcept;

	void AppendAxes(const StringRef& reply, AxesBitmap axes) const noexcept;	// Append a list of axes to a string

	void EndSimulation(GCodeBuffer *gb) noexcept;								// Restore positions etc. when exiting simulation mode
	bool IsCodeQueueIdle() const noexcept;										// Return true if the code queue is idle

#if HAS_MASS_STORAGE
	void SaveResumeInfo(bool wasPowerFailure) noexcept;
#endif

	void NewMoveAvailable(unsigned int sl) noexcept;							// Flag that a new move is available
	void NewMoveAvailable() noexcept;											// Flag that a new move is available

	void SetMoveBufferDefaults() noexcept;										// Set up default values in the move buffer
	void ChangeExtrusionFactor(unsigned int extruder, float factor) noexcept;	// Change a live extrusion factor

#if SUPPORT_12864_LCD
	int GetHeaterNumber(unsigned int itemNumber) const noexcept;
#endif
	Pwm_t ConvertLaserPwm(float reqVal) const noexcept;

#ifdef SERIAL_AUX_DEVICE
	static bool emergencyStopCommanded;
	static void CommandEmergencyStop(UARTClass *p) noexcept;
#endif

	Platform& platform;													// The RepRap machine

#if HAS_NETWORKING
	NetworkGCodeInput* httpInput;										// These cache incoming G-codes...
	NetworkGCodeInput* telnetInput;										// ...
#endif

	GCodeBuffer* gcodeSources[NumGCodeChannels];						// The various sources of gcodes

	GCodeBuffer*& httpGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::HTTP)];
	GCodeBuffer*& telnetGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Telnet)];
	GCodeBuffer*& fileGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::File)];
	GCodeBuffer*& usbGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::USB)];
	GCodeBuffer*& auxGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux)];					// This one is for the PanelDue on the async serial interface
	GCodeBuffer*& triggerGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Trigger)];			// Used for executing config.g and trigger macro files
	GCodeBuffer*& queuedGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Queue)];
	GCodeBuffer*& lcdGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::LCD)];					// This one for the 12864 LCD
	GCodeBuffer*& spiGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::SBC)];
	GCodeBuffer*& daemonGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Daemon)];
	GCodeBuffer*& aux2GCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Aux2)];				// This one is reserved for the second async serial interface
	GCodeBuffer*& autoPauseGCode = gcodeSources[GCodeChannel::ToBaseType(GCodeChannel::Autopause)];		// ***THIS ONE MUST BE LAST*** GCode state machine used to run macros on power fail, heater faults and filament out

	size_t nextGcodeSource;												// The one to check next, using round-robin scheduling

	const GCodeBuffer* resourceOwners[NumResources];					// Which gcode buffer owns each resource

	MachineType machineType;					// whether FFF, laser or CNC
	bool active;								// Live and running?
#if HAS_LINUX_INTERFACE
	FilePosition lastFilePosition;				// Last known file position
#endif
	bool isPaused;								// true if the print has been paused manually or automatically
	bool pausePending;							// true if we have been asked to pause but we are running a macro
	bool filamentChangePausePending;			// true if we have been asked to pause for a filament change but we are running a macro
	bool runningConfigFile;						// We are running config.g during the startup process
	bool doingToolChange;						// We are running tool change macros

#if HAS_VOLTAGE_MONITOR
	bool isPowerFailPaused;						// true if the print was paused automatically because of a power failure
	char *powerFailScript;						// the commands run when there is a power failure
#endif

	// The current user position now holds the requested user position after applying workplace coordinate offsets.
	// So we must subtract the workplace coordinate offsets when we want to display them.
	// We have chosen this approach because it allows us to switch workplace coordinates systems or turn off applying workplace offsets without having to update currentUserPosition.
	float currentUserPosition[MaxAxes];			// The current position of the axes as commanded by the input gcode, after accounting for workplace offset, before accounting for tool offset and Z hop
	float currentZHop;							// The amount of Z hop that is currently applied
	float lastPrintingMoveHeight;				// the Z coordinate in the last printing move, or a negative value if we don't know it

	// The following contain the details of moves that the Move module fetches
	// CAUTION: segmentsLeft should ONLY be changed from 0 to not 0 by calling NewMoveAvailable()!
	RawMove moveBuffer;							// Move details to pass to Move class
	unsigned int segmentsLeft;					// The number of segments left to do in the current move, or 0 if no move available
	unsigned int totalSegments;					// The total number of segments left in the complete move

	unsigned int segmentsLeftToStartAt;
	float moveFractionToSkip;
	float firstSegmentFractionToSkip;

	float restartMoveFractionDone;				// how much of the next move was printed before the pause or power failure (from M26)
	float restartInitialUserX;					// if the print was paused during an arc move, the user X coordinate at the start of that move (from M26)
	float restartInitialUserY;					// if the print was paused during an arc move, the user X coordinate at the start of that move (from M26)

	float arcCentre[MaxAxes];
	float arcRadius;
	float arcCurrentAngle;
	float arcAngleIncrement;
	bool doingArcMove;

	enum class SegmentedMoveState : uint8_t
	{
		inactive = 0,
		active,
		aborted
	};
	SegmentedMoveState segMoveState;

	AxesBitmap axesHomedBeforeSimulation;		// axes that were homed when we started the simulation
	RestorePoint simulationRestorePoint;		// The position and feed rate when we started a simulation

	RestorePoint numberedRestorePoints[NumRestorePoints];				// Restore points accessible using the R parameter in the G0/G1 command
	RestorePoint& pauseRestorePoint = numberedRestorePoints[1];			// The position and feed rate when we paused the print
	RestorePoint& toolChangeRestorePoint = numberedRestorePoints[2];	// The position and feed rate when we freed a tool
	RestorePoint& findCenterOfCavityRestorePoint = numberedRestorePoints[3];	// The position and feed rate when we found the lower boundary of cavity

	size_t numTotalAxes;						// How many axes we have
	size_t numVisibleAxes;						// How many axes are visible
	size_t numExtruders;						// How many extruders we have, or may have
	float axisScaleFactors[MaxAxes];			// Scale XYZ coordinates by this factor
	float virtualExtruderPosition;				// Virtual extruder position of the last move fed into the Move class
	float rawExtruderTotalByDrive[MaxExtruders]; // Extrusion amount in the last G1 command with an E parameter when in absolute extrusion mode
	float rawExtruderTotal;						// Total extrusion amount fed to Move class since starting print, before applying extrusion factor, summed over all drives

	unsigned int currentCoordinateSystem;		// This is zero-based, where as the P parameter in the G10 command is 1-based
	float workplaceCoordinates[NumCoordinateSystems][MaxAxes];	// Workplace coordinate offsets

#if HAS_MASS_STORAGE
	FileData fileToPrint;						// The next file to print
	FilePosition fileOffsetToPrint;				// The offset to print from
#endif

	// Tool change. These variables can be global because movement is locked while doing a tool change, so only one can take place at a time.
	int16_t newToolNumber;
	uint8_t toolChangeParam;

	char axisLetters[MaxAxes + 1];				// The names of the axes, with a null terminator
	bool limitAxes;								// Don't think outside the box
	bool noMovesBeforeHoming;					// Don't allow movement prior to homing the associates axes

	AxesBitmap toBeHomed;						// Bitmap of axes still to be homed
	AxesBitmap axesHomed;						// Bitmap of which axes have been homed

	float pausedFanSpeeds[MaxFans];				// Fan speeds when the print was paused or a tool change started
	float lastDefaultFanSpeed;					// Last speed given in a M106 command with on fan number
	float pausedDefaultFanSpeed;				// The speed of the default print cooling fan when the print was paused or a tool change started
	float speedFactor;							// speed factor as a fraction (normally 1.0)
	float extrusionFactors[MaxExtruders];		// extrusion factors (normally 1.0)
	float volumetricExtrusionFactors[MaxExtruders]; // Volumetric extrusion factors
	float currentBabyStepOffsets[MaxAxes];		// The accumulated axis offsets due to baby stepping requests

	// Z probe
	GridDefinition defaultGrid;					// The grid defined by the M557 command in config.g
	int32_t g30ProbePointIndex;					// the index of the point we are probing (G30 P parameter), or -1 if none
	int g30SValue;								// S parameter in the G30 command, or -2 if there wasn't one
	float g30HValue;							// H parameter in the G30 command, or 0.0 if there wasn't on
	float g30zStoppedHeight;					// the height to report after running G30 S-1
	float g30zHeightError;						// the height error last time we probed
	float g30PrevHeightError;					// the height error the previous time we probed
	float g30zHeightErrorSum;					// the sum of the height errors for the current probe point
	float g30zHeightErrorLowestDiff;			// the lowest difference we have seen between consecutive readings
	uint32_t lastProbedTime;					// time in milliseconds that the probe was last triggered
	volatile bool zProbeTriggered;				// Set by the step ISR when a move is aborted because the Z probe is triggered
	size_t gridXindex, gridYindex;				// Which grid probe point is next
	bool doingManualBedProbe;					// true if we are waiting for the user to jog the nozzle until it touches the bed
	bool hadProbingError;						// true if there was an error probing the last point
	bool zDatumSetByProbing;					// true if the Z position was last set by probing, not by an endstop switch or by G92
	uint8_t tapsDone;							// how many times we tapped the current point
	uint8_t currentZProbeNumber;				// which Z probe a G29 or G30 command is using

	// Simulation and print time
	float simulationTime;						// Accumulated simulation time
	uint32_t lastDuration;						// Time or simulated time of the last successful print or simulation, in seconds
	uint8_t simulationMode;						// 0 = not simulating, 1 = simulating, >1 are simulation modes for debugging
	bool exitSimulationWhenFileComplete;		// true if simulating a file
	bool updateFileWhenSimulationComplete;		// true if simulated time should be appended to the file

	// Triggers
	Trigger triggers[MaxTriggers];				// Trigger conditions
	TriggerNumbersBitmap triggersPending;		// Bitmap of triggers pending but not yet executed

	// Firmware update
	uint8_t firmwareUpdateModuleMap;			// Bitmap of firmware modules to be updated
	bool isFlashing;							// Is a new firmware binary going to be flashed?

	// Code queue
	GCodeQueue *codeQueue;						// Stores certain codes for deferred execution

#if HAS_MASS_STORAGE
	// SHA1 hashing
	FileStore *fileBeingHashed;
	SHA1Context hash;
	bool StartHash(const char* filename) noexcept;
	GCodeResult AdvanceHash(const StringRef &reply) noexcept;
#endif

	// Filament monitoring
	FilamentSensorStatus lastFilamentError;
	size_t lastFilamentErrorExtruder;

	// Laser
	float laserMaxPower;
	bool laserPowerSticky;						// true if G1 S parameters are remembered across G1 commands

	// Heater fault handler
	HeaterFaultState heaterFaultState;			// whether there is a heater fault and what we have done about it so far
	uint32_t heaterFaultTime;					// when the heater fault occurred
	uint32_t heaterFaultTimeout;				// how long we wait for the user to fix it before turning everything off

	// Object cancellation
	ObjectTracker buildObjects;

	// Misc
	uint32_t lastWarningMillis;					// When we last sent a warning message for things that can happen very often
	AxesBitmap axesToSenseLength;				// The axes on which we are performing axis length sensing
	bool atxPowerControlled;

#if HAS_MASS_STORAGE
	static constexpr uint32_t SdTimingByteIncrement = 8 * 1024;	// how many timing bytes we write at a time
	static constexpr const char *TimingFileName = "test.tst";	// the name of the file we write
	FileStore *sdTimingFile;					// file handle being used for SD card write timing
	uint32_t timingBytesRequested;				// how many bytes we were asked to write
	uint32_t timingBytesWritten;				// how many timing bytes we have written so far
	uint32_t timingStartMillis;
#endif

	int8_t lastAuxStatusReportType;				// The type of the last status report requested by PanelDue
	bool isWaiting;								// True if waiting to reach temperature
	bool cancelWait;							// Set true to cancel waiting
	bool displayNoToolWarning;					// True if we need to display a 'no tool selected' warning
	bool m501SeenInConfigFile;					// true if M501 was executed form config.g
	char filamentToLoad[FilamentNameLength];	// Name of the filament being loaded

	static constexpr const float MinServoPulseWidth = 544.0, MaxServoPulseWidth = 2400.0;
};

// Flag that a new move is available for consumption by the Move subsystem
// Code that sets up a new move should ensure that segmentsLeft is zero, then set up all the move parameters,
// then call this function to update SegmentsLeft safely in a multi-threaded environment
inline void GCodes::NewMoveAvailable(unsigned int sl) noexcept
{
	totalSegments = sl;
	__DMB();					// make sure that all the move details have been written first
	segmentsLeft = sl;			// set the number of segments to indicate that a move is available to be taken
}

// Flag that a new move is available for consumption by the Move subsystem
// This version is for when totalSegments has already be set up.
inline void GCodes::NewMoveAvailable() noexcept
{
	const unsigned int sl = totalSegments;
	__DMB();					// make sure that the move details have been written first
	segmentsLeft = sl;			// set the number of segments to indicate that a move is available to be taken
}

// Get the total baby stepping offset for an axis
inline float GCodes::GetTotalBabyStepOffset(size_t axis) const noexcept
{
	return currentBabyStepOffsets[axis];
}

//*****************************************************************************************************

#endif
