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

// The GCode interpreter

class GCodes INHERIT_OBJECT_MODEL
{
public:
	friend class LinuxInterface;

	GCodes(Platform& p);
	void Spin();														// Called in a tight loop to make this class work
	void Init();														// Set it up
	void Exit();														// Shut it down
	void Reset();														// Reset some parameter to defaults
	bool ReadMove(RawMove& m);											// Called by the Move class to get a movement set by the last G Code
	void ClearMove();
#if HAS_MASS_STORAGE
	bool QueueFileToPrint(const char* fileName, const StringRef& reply);	// Open a file of G Codes to run
#endif
	void StartPrinting(bool fromStart);									// Start printing the file already selected
	void GetCurrentCoordinates(const StringRef& s) const;				// Write where we are into a string
	bool DoingFileMacro() const;										// Or still busy processing a macro file?
	FilePosition GetFilePosition() const;								// Return the current position of the file being printed in bytes
	void Diagnostics(MessageType mtype);								// Send helpful information out

	bool RunConfigFile(const char* fileName);							// Start running the config file
	bool IsDaemonBusy() const;											// Return true if the daemon is busy running config.g or a trigger file

	bool IsAxisHomed(unsigned int axis) const							// Has the axis been homed?
		{ return IsBitSet(axesHomed, axis); }
	void SetAxisIsHomed(unsigned int axis);								// Tell us that the axis is now homed
	void SetAxisNotHomed(unsigned int axis);							// Tell us that the axis is not homed
	void SetAllAxesNotHomed();											// Flag all axes as not homed

	float GetSpeedFactor() const;										// Return the current speed factor
#if SUPPORT_12864_LCD
	void SetSpeedFactor(float factor);									// Set the speed factor
#endif
	float GetExtrusionFactor(size_t extruder);							// Return the current extrusion factors
	void SetExtrusionFactor(size_t extruder, float factor);				// Set an extrusion factor

	float GetRawExtruderTotalByDrive(size_t extruder) const;			// Get the total extrusion since start of print, for one drive
	float GetTotalRawExtrusion() const { return rawExtruderTotal; }		// Get the total extrusion since start of print, all drives
	float GetTotalBabyStepOffset(size_t axis) const
		pre(axis < maxAxes);
	float GetUserCoordinate(size_t axis) const;							// Get the current user coordinate in the current workspace coordinate system

#if HAS_NETWORKING
	NetworkGCodeInput *GetHTTPInput() const { return httpInput; }
	NetworkGCodeInput *GetTelnetInput() const { return telnetInput; }
#endif

	bool IsFlashing() const { return isFlashing; }						// Is a new firmware binary going to be flashed?

	bool IsPaused() const;
	bool IsPausing() const;
	bool IsResuming() const;
	bool IsRunning() const;
	bool IsReallyPrinting() const;										// Return true if we are printing from SD card and not pausing, paused or resuming
	bool IsSimulating() const { return simulationMode != 0; }
	bool IsDoingToolChange() const { return doingToolChange; }
	bool IsHeatingUp() const;											// Return true if the SD card print is waiting for a heater to reach temperature

	bool AllAxesAreHomed() const;										// Return true if all axes are homed

	void StopPrint(StopPrintReason reason);								// Stop the current print

	void MoveStoppedByZProbe() { zProbeTriggered = true; }				// Called from the step ISR when the Z probe is triggered, causing the move to be aborted

	size_t GetTotalAxes() const { return numTotalAxes; }
	size_t GetVisibleAxes() const { return numVisibleAxes; }
	size_t GetNumExtruders() const { return numExtruders; }

	const char* GetMachineModeString() const;							// Get the name of the current machine mode

	void FilamentError(size_t extruder, FilamentSensorStatus fstat);
	void HandleHeaterFault(int heater);									// Respond to a heater fault

#if HAS_VOLTAGE_MONITOR
	bool LowVoltagePause();
	bool LowVoltageResume();
#endif

#if HAS_SMART_DRIVERS
	bool PauseOnStall(DriversBitmap stalledDrivers);
	bool ReHomeOnStall(DriversBitmap stalledDrivers);
#endif

	const char *GetAxisLetters() const { return axisLetters; }			// Return a null-terminated string of axis letters indexed by drive
	MachineType GetMachineType() const { return machineType; }
	bool LockMovementAndWaitForStandstill(const GCodeBuffer& gb);		// Lock movement and wait for pending moves to finish

#if SUPPORT_12864_LCD
	bool ProcessCommandFromLcd(const char *cmd);						// Process a GCode command from the 12864 LCD returning true if the command was accepted
	float GetItemCurrentTemperature(unsigned int itemNumber) const;
	float GetItemActiveTemperature(unsigned int itemNumber) const;
	float GetItemStandbyTemperature(unsigned int itemNumber) const;
	void SetItemActiveTemperature(unsigned int itemNumber, float temp);
	void SetItemStandbyTemperature(unsigned int itemNumber, float temp);
#endif

	float GetMappedFanSpeed() const { return lastDefaultFanSpeed; }		// Get the mapped fan speed
	void SetMappedFanSpeed(float f);									// Set the speeds of fans mapped for the current tool
	void HandleReply(GCodeBuffer& gb, GCodeResult rslt, const char *reply);	// Handle G-Code replies
	void EmergencyStop();												// Cancel everything
	bool GetLastPrintingHeight(float& height) const;					// Get the height in user coordinates of the last printing move
	bool AtxPowerControlled() const { return atxPowerControlled; }

	void AssignGrid(float xRange[2], float yRange[2], float radius, float spacing[2]);	// Assign the heightmap using the given parameters
	void ActivateHeightmap(bool activate);										// (De-)Activate the height map

#if HAS_MASS_STORAGE
	GCodeResult StartSDTiming(GCodeBuffer& gb, const StringRef& reply);	// Start timing SD card file writing
#endif

#if SUPPORT_WORKPLACE_COORDINATES
	unsigned int GetWorkplaceCoordinateSystemNumber() const { return currentCoordinateSystem + 1; }
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	GCodes(const GCodes&);												// private copy constructor to prevent copying

	enum class HeaterFaultState : uint8_t { noFault, pausePending, timing, stopping, stopped };

	// Resources that can be locked.
	// To avoid deadlock, if you need multiple resources then you must lock them in increasing numerical order.
	typedef unsigned int Resource;
	static const Resource MoveResource = 0;								// Movement system, including canned cycle variables
	static const Resource FileSystemResource = 1;						// Non-sharable parts of the file system
	static const Resource HeaterResourceBase = 2;
	static const Resource FanResourceBase = HeaterResourceBase + MaxHeaters;
	static const size_t NumResources = FanResourceBase + NumTotalFans;

	static_assert(NumResources <= 32, "Too many resources to keep a bitmap of them in class GCodeMachineState");

	bool LockResource(const GCodeBuffer& gb, Resource r);				// Lock the resource, returning true if success
	bool LockHeater(const GCodeBuffer& gb, int heater);
	bool LockFan(const GCodeBuffer& gb, int fan);
	bool LockFileSystem(const GCodeBuffer& gb);							// Lock the unshareable parts of the file system
	bool LockMovement(const GCodeBuffer& gb);							// Lock movement
	void GrabResource(const GCodeBuffer& gb, Resource r);				// Grab a resource even if it is already owned
	void GrabMovement(const GCodeBuffer& gb);							// Grab the movement lock even if it is already owned
	void UnlockResource(const GCodeBuffer& gb, Resource r);				// Unlock the resource if we own it
	void UnlockMovement(const GCodeBuffer& gb);							// Unlock the movement resource if we own it
	void UnlockAll(const GCodeBuffer& gb);								// Release all locks

	GCodeBuffer *GetGCodeBuffer(GCodeChannel channel) const { return gcodeSources[(size_t)channel]; }
	void StartNextGCode(GCodeBuffer& gb, const StringRef& reply);		// Fetch a new or old GCode and process it
	void RunStateMachine(GCodeBuffer& gb, const StringRef& reply);		// Execute a step of the state machine
	void DoFilePrint(GCodeBuffer& gb, const StringRef& reply);			// Get G Codes from a file and print them
	bool DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, int codeRunning = -1);
																		// Run a GCode macro file, optionally report error if not found
	void FileMacroCyclesReturn(GCodeBuffer& gb);						// End a macro

	bool ActOnCode(GCodeBuffer& gb, const StringRef& reply);			// Do a G, M or T Code
	bool HandleGcode(GCodeBuffer& gb, const StringRef& reply);			// Do a G code
	bool HandleMcode(GCodeBuffer& gb, const StringRef& reply);			// Do an M code
	bool HandleTcode(GCodeBuffer& gb, const StringRef& reply);			// Do a T code
	bool HandleResult(GCodeBuffer& gb, GCodeResult rslt, const StringRef& reply, OutputBuffer *outBuf)
		pre(outBuf == nullptr || rslt == GCodeResult::ok);

	void HandleReply(GCodeBuffer& gb, OutputBuffer *reply);

	const char* DoStraightMove(GCodeBuffer& gb, bool isCoordinated) __attribute__((hot));	// Execute a straight move returning any error message
	const char* DoArcMove(GCodeBuffer& gb, bool clockwise)						// Execute an arc move returning any error message
		pre(segmentsLeft == 0; resourceOwners[MoveResource] == &gb);
	void FinaliseMove(GCodeBuffer& gb);											// Adjust the move parameters to account for segmentation and/or part of the move having been done already
	bool CheckEnoughAxesHomed(AxesBitmap axesMoved);							// Check that enough axes have been homed
	void AbortPrint(GCodeBuffer& gb);											// Cancel any print in progress

	GCodeResult DoDwell(GCodeBuffer& gb);										// Wait for a bit
	GCodeResult DoHome(GCodeBuffer& gb, const StringRef& reply);				// Home some axes
	GCodeResult ExecuteG30(GCodeBuffer& gb, const StringRef& reply);			// Probes at a given position - see the comment at the head of the function itself
	void InitialiseTaps();														// Set up to do the first of a possibly multi-tap probe
	void SetBedEquationWithProbe(int sParam, const StringRef& reply);			// Probes a series of points and sets the bed equation
	GCodeResult SetOrReportOffsets(GCodeBuffer& gb, const StringRef& reply);	// Deal with a G10
	GCodeResult SetPositions(GCodeBuffer& gb);									// Deal with a G92
	GCodeResult StraightProbe(GCodeBuffer& gb, const StringRef& reply);			// Deal with a G38.x
	GCodeResult DoDriveMapping(GCodeBuffer& gb, const StringRef& reply);		// Deal with a M584
	GCodeResult ProbeTool(GCodeBuffer& gb, const StringRef& reply);				// Deal with a M585
	GCodeResult FindCenterOfCavity(GCodeBuffer& gb, const StringRef& reply, const bool towardsMin = true);	// Deal with a M675
	GCodeResult SetDateTime(GCodeBuffer& gb,const  StringRef& reply);			// Deal with a M905
	GCodeResult SavePosition(GCodeBuffer& gb,const  StringRef& reply);			// Deal with G60
	GCodeResult ConfigureDriver(GCodeBuffer& gb,const  StringRef& reply);		// Deal with M569

	bool LoadExtrusionAndFeedrateFromGCode(GCodeBuffer& gb, bool isPrintingMove);	// Set up the extrusion of a move

	bool Push(GCodeBuffer& gb, bool preserveLineNumber);						// Push feedrate etc on the stack
	void Pop(GCodeBuffer& gb, bool preserveLineNumber);							// Pop feedrate etc
	void DisableDrives();														// Turn the motors off
																				// Start saving GCodes in a file
	bool SendConfigToLine();													// Deal with M503

	GCodeResult OffsetAxes(GCodeBuffer& gb, const StringRef& reply);			// Set/report offsets

#if SUPPORT_WORKPLACE_COORDINATES
	GCodeResult GetSetWorkplaceCoordinates(GCodeBuffer& gb, const StringRef& reply, bool compute);	// Set workspace coordinates
# if HAS_MASS_STORAGE
	bool WriteWorkplaceCoordinates(FileStore *f) const;
# endif
#endif

	GCodeResult ManageTool(GCodeBuffer& gb, const StringRef& reply);			// Create a new tool definition
	void SetToolHeaters(Tool *tool, float temperature, bool both);				// Set all a tool's heaters to the temperature, for M104/M109
	bool ToolHeatersAtSetTemperatures(const Tool *tool, bool waitWhenCooling, float tolerance) const;
																				// Wait for the heaters associated with the specified tool to reach their set temperatures
	void ReportToolTemperatures(const StringRef& reply, const Tool *tool, bool includeNumber) const;
	void GenerateTemperatureReport(const StringRef& reply) const;				// Store a standard-format temperature report in reply
	OutputBuffer *GenerateJsonStatusResponse(int type, int seq, ResponseSource source) const;	// Generate a M408 response
	void CheckReportDue(GCodeBuffer& gb, const StringRef& reply) const;			// Check whether we need to report temperatures or status

	void SavePosition(RestorePoint& rp, const GCodeBuffer& gb) const;			// Save position to a restore point
	void RestorePosition(const RestorePoint& rp, GCodeBuffer *gb);				// Restore user position from a restore point

	void SetMachinePosition(const float positionNow[MaxAxesPlusExtruders], bool doBedCompensation = true); // Set the current position to be this
	void UpdateCurrentUserPosition();											// Get the current position from the Move class
	void ToolOffsetTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes], AxesBitmap explicitAxes = 0) const;
																				// Convert user coordinates to head reference point coordinates
	void ToolOffsetInverseTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes]) const;	// Convert head reference point coordinates to user coordinates
	float GetCurrentToolOffset(size_t axis) const;								// Get an axis offset of the current tool

	GCodeResult RetractFilament(GCodeBuffer& gb, bool retract);					// Retract or un-retract filaments
	GCodeResult LoadFilament(GCodeBuffer& gb, const StringRef& reply);			// Load the specified filament into a tool
	GCodeResult UnloadFilament(GCodeBuffer& gb, const StringRef& reply);		// Unload the current filament from a tool
	bool ChangeMicrostepping(size_t drive, unsigned int microsteps, bool interp, const StringRef& reply) const; // Change microstepping on the specified drive
	void CheckTriggers();														// Check for and execute triggers
	void CheckFilament();														// Check for and respond to filament errors
	void CheckHeaterFault();													// Check for and respond to a heater fault, returning true if we should exit
	void DoEmergencyStop();														// Execute an emergency stop

	void DoPause(GCodeBuffer& gb, PauseReason reason, const char *msg)			// Pause the print
		pre(resourceOwners[movementResource] = &gb);

#if HAS_VOLTAGE_MONITOR || HAS_SMART_DRIVERS
	bool DoEmergencyPause();													// Do an emergency pause following loss of power or a motor stall
#endif

	bool IsMappedFan(unsigned int fanNumber);									// Return true if this fan number is currently being used as a print cooling fan
	void SaveFanSpeeds();														// Save the speeds of all fans

	GCodeResult DefineGrid(GCodeBuffer& gb, const StringRef &reply);			// Define the probing grid, returning true if error
#if HAS_MASS_STORAGE
	GCodeResult LoadHeightMap(GCodeBuffer& gb, const StringRef& reply);			// Load the height map from file
	bool TrySaveHeightMap(const char *filename, const StringRef& reply) const;	// Save the height map to the specified file
	GCodeResult SaveHeightMap(GCodeBuffer& gb, const StringRef& reply) const;	// Save the height map to the file specified by P parameter
#endif
	void ClearBedMapping();														// Stop using bed compensation
	GCodeResult ProbeGrid(GCodeBuffer& gb, const StringRef& reply);				// Start probing the grid, returning true if we didn't because of an error
	GCodeResult ConfigureTrigger(GCodeBuffer& gb, const StringRef& reply, int code);	// Handle M581
	GCodeResult CheckTrigger(GCodeBuffer& gb, const StringRef& reply, int code);		// Handle M582
	GCodeResult UpdateFirmware(GCodeBuffer& gb, const StringRef &reply);		// Handle M997
	GCodeResult SendI2c(GCodeBuffer& gb, const StringRef &reply);				// Handle M260
	GCodeResult ReceiveI2c(GCodeBuffer& gb, const StringRef &reply);			// Handle M261
#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	GCodeResult SimulateFile(GCodeBuffer& gb, const StringRef &reply, const StringRef& file, bool updateFile);	// Handle M37 to simulate a whole file
	GCodeResult ChangeSimulationMode(GCodeBuffer& gb, const StringRef &reply, uint32_t newSimulationMode);		// Handle M37 to change the simulation mode
#endif
	GCodeResult WaitForPin(GCodeBuffer& gb, const StringRef &reply);			// Handle M577

#if HAS_MASS_STORAGE
	GCodeResult WriteConfigOverrideFile(GCodeBuffer& gb, const StringRef& reply) const; // Write the config-override file
	bool WriteConfigOverrideHeader(FileStore *f) const;							// Write the config-override header
#endif

	void CopyConfigFinalValues(GCodeBuffer& gb);								// Copy the feed rate etc. from the daemon to the input channels

	MessageType GetMessageBoxDevice(GCodeBuffer& gb) const;						// Decide which device to display a message box on
	void DoManualProbe(GCodeBuffer&, const char *message, const char *title, const AxesBitmap); 			// Do manual probe in arbitrary direction
	void DoManualBedProbe(GCodeBuffer& gb);										// Do a manual bed probe

	void AppendAxes(const StringRef& reply, AxesBitmap axes) const;				// Append a list of axes to a string

	void EndSimulation(GCodeBuffer *gb);								// Restore positions etc. when exiting simulation mode
	bool IsCodeQueueIdle() const;										// Return true if the code queue is idle

#if HAS_MASS_STORAGE
	void SaveResumeInfo(bool wasPowerFailure);
#endif

	void NewMoveAvailable(unsigned int sl);								// Flag that a new move is available
	void NewMoveAvailable();											// Flag that a new move is available

	void SetMoveBufferDefaults();										// Set up default values in the move buffer
	void ChangeExtrusionFactor(unsigned int extruder, float factor);	// Change a live extrusion factor

#if SUPPORT_12864_LCD
	int GetHeaterNumber(unsigned int itemNumber) const;
#endif
	Pwm_t ConvertLaserPwm(float reqVal) const;

	// This function is called by other functions to account correctly for workplace coordinates, depending on whether the build configuration supports them.
	inline float GetWorkplaceOffset(size_t axis) const
	{
#if SUPPORT_WORKPLACE_COORDINATES
		return workplaceCoordinates[currentCoordinateSystem][axis];
#else
		return axisOffsets[axis];
#endif
	}

#ifdef SERIAL_AUX_DEVICE
	static bool emergencyStopCommanded;
	static void CommandEmergencyStop(UARTClass *p);
#endif

	Platform& platform;													// The RepRap machine

#if HAS_NETWORKING
	NetworkGCodeInput* httpInput;										// These cache incoming G-codes...
	NetworkGCodeInput* telnetInput;										// ...
#endif

	GCodeBuffer* gcodeSources[NumGCodeChannels];						// The various sources of gcodes

	GCodeBuffer*& httpGCode = gcodeSources[0];
	GCodeBuffer*& telnetGCode = gcodeSources[1];
	GCodeBuffer*& fileGCode = gcodeSources[2];
	GCodeBuffer*& usbGCode = gcodeSources[3];
	GCodeBuffer*& auxGCode = gcodeSources[4];							// This one is for the PanelDue on the async serial interface
	GCodeBuffer*& daemonGCode = gcodeSources[5];						// Used for executing config.g and trigger macro files
	GCodeBuffer*& queuedGCode = gcodeSources[6];
	GCodeBuffer*& lcdGCode = gcodeSources[7];							// This one for the 12864 LCD
	GCodeBuffer*& spiGCode = gcodeSources[8];
	GCodeBuffer*& autoPauseGCode = gcodeSources[9];						// ***THIS ONE MUST BE LAST*** GCode state machine used to run macros on power fail, heater faults and filament out

	const GCodeBuffer* resourceOwners[NumResources];					// Which gcode buffer owns each resource

	MachineType machineType;					// whether FFF, laser or CNC
	bool active;								// Live and running?
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
	float moveFractionToStartAt;				// how much of the next move was printed before the power failure
	float moveFractionToSkip;
	float firstSegmentFractionToSkip;

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

#if SUPPORT_WORKPLACE_COORDINATES
	static const size_t NumCoordinateSystems = 9;
	unsigned int currentCoordinateSystem;		// This is zero-based, where as the P parameter in the G10 command is 1-based
	float workplaceCoordinates[NumCoordinateSystems][MaxAxes];	// Workplace coordinate offsets
#else
	float axisOffsets[MaxAxes];					// M206 axis offsets
#endif

#if HAS_MASS_STORAGE
	FileData fileToPrint;						// The next file to print
	FilePosition fileOffsetToPrint;				// The offset to print from
#endif

	char axisLetters[MaxAxes + 1];				// The names of the axes, with a null terminator
	bool limitAxes;								// Don't think outside the box
	bool noMovesBeforeHoming;					// Don't allow movement prior to homing the associates axes

	AxesBitmap toBeHomed;						// Bitmap of axes still to be homed
	AxesBitmap axesHomed;						// Bitmap of which axes have been homed

	float pausedFanSpeeds[NumTotalFans];			// Fan speeds when the print was paused or a tool change started
	float lastDefaultFanSpeed;					// Last speed given in a M106 command with on fan number
	float pausedDefaultFanSpeed;				// The speed of the default print cooling fan when the print was paused or a tool change started
	float speedFactor;							// speed factor as a percentage (normally 100.0)
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
	bool probeIsDeployed;						// true if M401 has been used to deploy the probe and M402 has not yet been used t0 retract it
	bool hadProbingError;						// true if there was an error probing the last point
	bool zDatumSetByProbing;					// true if the Z position was last set by probing, not by an endstop switch or by G92
	uint8_t tapsDone;							// how many times we tapped the current point

	float simulationTime;						// Accumulated simulation time
	uint8_t simulationMode;						// 0 = not simulating, 1 = simulating, >1 are simulation modes for debugging
	bool exitSimulationWhenFileComplete;		// true if simulating a file
	bool updateFileWhenSimulationComplete;		// true if simulated time should be appended to the file

	// Firmware retraction settings
	float retractLength, retractExtra;			// retraction length and extra length to un-retract
	float retractSpeed;							// retract speed in mm/min
	float unRetractSpeed;						// un=retract speed in mm/min
	float retractHop;							// Z hop when retracting
	bool isRetracted;							// true if filament has been firmware-retracted

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
	bool StartHash(const char* filename);
	GCodeResult AdvanceHash(const StringRef &reply);
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

	// Standard macro filenames
	static constexpr const char* BED_EQUATION_G = "bed.g";
	static constexpr const char* PAUSE_G = "pause.g";
	static constexpr const char* RESUME_G = "resume.g";
	static constexpr const char* CANCEL_G = "cancel.g";
	static constexpr const char* START_G = "start.g";
	static constexpr const char* STOP_G = "stop.g";
	static constexpr const char* SLEEP_G = "sleep.g";
	static constexpr const char* CONFIG_OVERRIDE_G = "config-override.g";
	static constexpr const char* DEPLOYPROBE_G = "deployprobe.g";
	static constexpr const char* RETRACTPROBE_G = "retractprobe.g";
	static constexpr const char* DefaultHeightMapFile = "heightmap.csv";
	static constexpr const char* LOAD_FILAMENT_G = "load.g";
	static constexpr const char* CONFIG_FILAMENT_G = "config.g";
	static constexpr const char* UNLOAD_FILAMENT_G = "unload.g";
	static constexpr const char* RESUME_AFTER_POWER_FAIL_G = "resurrect.g";
	static constexpr const char* RESUME_PROLOGUE_G = "resurrect-prologue.g";
	static constexpr const char* FILAMENT_CHANGE_G = "filament-change.g";
#if HAS_SMART_DRIVERS
	static constexpr const char* REHOME_G = "rehome.g";
#endif

	static constexpr const float MinServoPulseWidth = 544.0, MaxServoPulseWidth = 2400.0;
};

// Flag that a new move is available for consumption by the Move subsystem
// Code that sets up a new move should ensure that segmentsLeft is zero, then set up all the move parameters,
// then call this function to update SegmentsLeft safely in a multi-threaded environment
inline void GCodes::NewMoveAvailable(unsigned int sl)
{
	totalSegments = sl;
	__DMB();					// make sure that all the move details have been written first
	segmentsLeft = sl;			// set the number of segments to indicate that a move is available to be taken
}

// Flag that a new move is available for consumption by the Move subsystem
// This version is for when totalSegments has already be set up.
inline void GCodes::NewMoveAvailable()
{
	const unsigned int sl = totalSegments;
	__DMB();					// make sure that the move details have been written first
	segmentsLeft = sl;			// set the number of segments to indicate that a move is available to be taken
}

// Get the total baby stepping offset for an axis
inline float GCodes::GetTotalBabyStepOffset(size_t axis) const
{
	return currentBabyStepOffsets[axis];
}

//*****************************************************************************************************

#endif
