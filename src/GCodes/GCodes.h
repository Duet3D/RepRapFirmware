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
#include "Libraries/sha1/sha1.h"
#include "Platform.h"		// for type EndStopHit
#include "GCodeInput.h"
#include "Tools/Filament.h"
#include "FilamentSensors/FilamentSensor.h"
#include "RestorePoint.h"
#include "Movement/BedProbing/Grid.h"

const char feedrateLetter = 'F';						// GCode feedrate
const char extrudeLetter = 'E'; 						// GCode extrude

// Type for specifying which endstops we want to check
typedef AxesBitmap EndstopChecks;						// must be large enough to hold a bitmap of drive numbers or ZProbeActive
const EndstopChecks ZProbeActive = 1 << 31;				// must be distinct from 1 << (any drive number)
const EndstopChecks HomeAxes = 1 << 30;					// must be distinct from 1 << (any drive number)
const EndstopChecks LogProbeChanges = 1 << 29;			// must be distinct from 1 << (any drive number)

typedef uint32_t TriggerInputsBitmap;					// Bitmap of input pins that a single trigger number responds to
typedef uint32_t TriggerNumbersBitmap;					// Bitmap of trigger numbers

struct Trigger
{
	TriggerInputsBitmap rising;
	TriggerInputsBitmap falling;
	uint8_t condition;

	void Init()
	{
		rising = falling = 0;
		condition = 0;
	}

	bool IsUnused() const
	{
		return rising == 0 && falling == 0;
	}
};

// Bits for T-code P-parameter to specify which macros are supposed to be run
const int TFreeBit = 1 << 0;
const int TPreBit = 1 << 1;
const int TPostBit = 1 << 2;
const int DefaultToolChangeParam = TFreeBit | TPreBit | TPostBit;

// Machine type enumeration. The numeric values must be in the same order as the corresponding M451..M453 commands.
enum class MachineType : uint8_t
{
	fff = 0,
	laser = 1,
	cnc = 2
};

//****************************************************************************************************

// The GCode interpreter

class GCodes
{   
public:
	struct RawMove
	{
		float coords[DRIVES];											// new positions for the axes, amount of movement for the extruders
		float initialCoords[MaxAxes];									// the initial positions of the axes
		float feedRate;													// feed rate of this move
		float virtualExtruderPosition;									// the virtual extruder position at the start of this move
		FilePosition filePos;											// offset in the file being printed at the start of reading this move
		AxesBitmap xAxes;												// axes that X is mapped to
		AxesBitmap yAxes;												// axes that Y is mapped to
		EndstopChecks endStopsToCheck;									// endstops to check
#if SUPPORT_IOBITS
		IoBits_t ioBits;												// I/O bits to set/clear at the start of this move
#endif
		uint8_t moveType;												// the S parameter from the G0 or G1 command, 0 for a normal move
		uint8_t isFirmwareRetraction : 1;								// true if this is a firmware retraction/un-retraction move
		uint8_t usePressureAdvance : 1;									// true if we want to us extruder pressure advance, if there is any extrusion
		uint8_t canPauseBefore : 1;										// true if we can pause before this move
		uint8_t canPauseAfter : 1;										// true if we can pause just after this move and successfully restart
		uint8_t hasExtrusion : 1;										// true if the move includes extrusion - only valid if the move was set up by SetupMove
		uint8_t isCoordinated : 1;										// true if this is a coordinates move
	};
  
	GCodes(Platform& p);
	void Spin();														// Called in a tight loop to make this class work
	void Init();														// Set it up
	void Exit();														// Shut it down
	void Reset();														// Reset some parameter to defaults
	bool ReadMove(RawMove& m);											// Called by the Move class to get a movement set by the last G Code
	void ClearMove();
	bool QueueFileToPrint(const char* fileName, StringRef& reply);		// Open a file of G Codes to run
	void StartPrinting();												// Start printing the file already selected
	void GetCurrentCoordinates(StringRef& s) const;						// Write where we are into a string
	bool DoingFileMacro() const;										// Or still busy processing a macro file?
	float FractionOfFilePrinted() const;								// Get fraction of file printed
	void Diagnostics(MessageType mtype);								// Send helpful information out

	bool RunConfigFile(const char* fileName);							// Start running the config file
	bool IsDaemonBusy() const;											// Return true if the daemon is busy running config.g or a trigger file

	bool GetAxisIsHomed(unsigned int axis) const						// Has the axis been homed?
		{ return IsBitSet(axesHomed, axis); }
	void SetAxisIsHomed(unsigned int axis)								// Tell us that the axis is now homed
		{ SetBit(axesHomed, axis); }
	void SetAxisNotHomed(unsigned int axis)								// Tell us that the axis is not homed
		{ ClearBit(axesHomed, axis); }

	float GetSpeedFactor() const { return speedFactor * MinutesToSeconds; }	// Return the current speed factor
	float GetExtrusionFactor(size_t extruder) { return extrusionFactors[extruder]; } // Return the current extrusion factors
	float GetRawExtruderTotalByDrive(size_t extruder) const;			// Get the total extrusion since start of print, for one drive
	float GetTotalRawExtrusion() const { return rawExtruderTotal; }		// Get the total extrusion since start of print, all drives
	float GetBabyStepOffset() const { return currentBabyStepZOffset; }	// Get the current baby stepping Z offset

	RegularGCodeInput *GetHTTPInput() const { return httpInput; }
	RegularGCodeInput *GetTelnetInput() const { return telnetInput; }

	void WriteGCodeToFile(GCodeBuffer& gb);								// Write this GCode into a file
	void WriteHTMLToFile(GCodeBuffer& gb, char b);						// Save an HTML file (usually to upload a new web interface)

	bool IsFlashing() const { return isFlashing; }						// Is a new firmware binary going to be flashed?

	bool IsPaused() const;
	bool IsPausing() const;
	bool IsResuming() const;
	bool IsRunning() const;
	bool IsDoingToolChange() const { return doingToolChange; }

	bool AllAxesAreHomed() const;										// Return true if all axes are homed

	void CancelPrint(bool printStats, bool deleteResumeFile);			// Cancel the current print

	void MoveStoppedByZProbe() { zProbeTriggered = true; }				// Called from the step ISR when the Z probe is triggered, causing the move to be aborted

	size_t GetTotalAxes() const { return numTotalAxes; }
	size_t GetVisibleAxes() const { return numVisibleAxes; }
	size_t GetNumExtruders() const { return numExtruders; }

	void FilamentError(size_t extruder, FilamentSensorStatus fstat);

#ifdef DUET_NG
	bool AutoPause();
	bool AutoShutdown();
	bool AutoResume();
	bool AutoResumeAfterShutdown();
#endif

	static const char axisLetters[MaxAxes];

private:
	GCodes(const GCodes&);												// private copy constructor to prevent copying

	enum class CannedMoveType : uint8_t { none, relative, absolute };

	// Resources that can be locked.
	// To avoid deadlock, if you need multiple resources then you must lock them in increasing numerical order.
	typedef unsigned int Resource;
	static const Resource MoveResource = 0;								// Movement system, including canned cycle variables
	static const Resource FileSystemResource = 1;						// Non-sharable parts of the file system
	static const Resource HeaterResourceBase = 2;
	static const Resource FanResourceBase = HeaterResourceBase + Heaters;
	static const size_t NumResources = FanResourceBase + NUM_FANS;

	static_assert(NumResources <= 32, "Too many resources to keep a bitmap of them in class GCodeMachineState");

	bool LockResource(const GCodeBuffer& gb, Resource r);				// Lock the resource, returning true if success
	bool LockHeater(const GCodeBuffer& gb, int heater);
	bool LockFan(const GCodeBuffer& gb, int fan);
	bool LockFileSystem(const GCodeBuffer& gb);							// Lock the unshareable parts of the file system
	bool LockMovement(const GCodeBuffer& gb);							// Lock movement
	bool LockMovementAndWaitForStandstill(const GCodeBuffer& gb);		// Lock movement and wait for pending moves to finish
	void UnlockAll(const GCodeBuffer& gb);								// Release all locks

	void StartNextGCode(GCodeBuffer& gb, StringRef& reply);				// Fetch a new or old GCode and process it
	void DoFilePrint(GCodeBuffer& gb, StringRef& reply);				// Get G Codes from a file and print them
	bool DoFileMacro(GCodeBuffer& gb, const char* fileName, bool reportMissing, bool runningM502 = false);
																		// Run a GCode macro file, optionally report error if not found
	void FileMacroCyclesReturn(GCodeBuffer& gb);						// End a macro

	bool ActOnCode(GCodeBuffer& gb, StringRef& reply);					// Do a G, M or T Code
	bool HandleGcode(GCodeBuffer& gb, StringRef& reply);				// Do a G code
	bool HandleMcode(GCodeBuffer& gb, StringRef& reply);				// Do an M code
	bool HandleTcode(GCodeBuffer& gb, StringRef& reply);				// Do a T code

	bool DoStraightMove(GCodeBuffer& gb, StringRef& reply, bool isCoordinated) __attribute__((hot));	// Execute a straight move returning true if an error was written to 'reply'
	bool DoArcMove(GCodeBuffer& gb, bool clockwise)						// Execute an arc move returning true if it was badly-formed
		pre(segmentsLeft == 0; resourceOwners[MoveResource] == &gb);

	GCodeResult DoDwell(GCodeBuffer& gb);								// Wait for a bit
	GCodeResult DoDwellTime(GCodeBuffer& gb, uint32_t dwellMillis);		// Really wait for a bit
	GCodeResult DoHome(GCodeBuffer& gb, StringRef& reply);				// Home some axes
	GCodeResult ExecuteG30(GCodeBuffer& gb, StringRef& reply);			// Probes at a given position - see the comment at the head of the function itself
	void SetBedEquationWithProbe(int sParam, StringRef& reply);			// Probes a series of points and sets the bed equation
	GCodeResult SetPrintZProbe(GCodeBuffer& gb, StringRef& reply);		// Either return the probe value, or set its threshold
	GCodeResult SetOrReportOffsets(GCodeBuffer& gb, StringRef& reply);	// Deal with a G10

	GCodeResult SetPositions(GCodeBuffer& gb);							// Deal with a G92
	bool LoadExtrusionAndFeedrateFromGCode(GCodeBuffer& gb, int moveType); // Set up the extrusion and feed rate of a move for the Move class

	bool Push(GCodeBuffer& gb);											// Push feedrate etc on the stack
	void Pop(GCodeBuffer& gb);											// Pop feedrate etc
	void DisableDrives();												// Turn the motors off
	void SetMACAddress(GCodeBuffer& gb);								// Deals with an M540
	void HandleReply(GCodeBuffer& gb, bool error, const char *reply);	// Handle G-Code replies
	void HandleReply(GCodeBuffer& gb, bool error, OutputBuffer *reply);
	bool OpenFileToWrite(GCodeBuffer& gb, const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32);
																		// Start saving GCodes in a file
	void FinishWrite(GCodeBuffer& gb);									// Finish writing to the file and respond
	bool SendConfigToLine();											// Deal with M503
	GCodeResult OffsetAxes(GCodeBuffer& gb);							// Set offsets - deprecated, use G10
	void SetPidParameters(GCodeBuffer& gb, int heater, StringRef& reply); // Set the P/I/D parameters for a heater
	GCodeResult SetHeaterParameters(GCodeBuffer& gb, StringRef& reply);	// Set the thermistor and ADC parameters for a heater, returning true if an error occurs
	bool ManageTool(GCodeBuffer& gb, StringRef& reply);					// Create a new tool definition, returning true if an error was reported
	void SetToolHeaters(Tool *tool, float temperature);					// Set all a tool's heaters to the temperature, for M104
	bool ToolHeatersAtSetTemperatures(const Tool *tool, bool waitWhenCooling) const; // Wait for the heaters associated with the specified tool to reach their set temperatures
	void GenerateTemperatureReport(StringRef& reply) const;				// Store a standard-format temperature report in reply
	OutputBuffer *GenerateJsonStatusResponse(int type, int seq, ResponseSource source) const;	// Generate a M408 response
	void CheckReportDue(GCodeBuffer& gb, StringRef& reply) const;		// Check whether we need to report temperatures or status

	void SavePosition(RestorePoint& rp, const GCodeBuffer& gb) const;	// Save position to a restore point
	void RestorePosition(const RestorePoint& rp, GCodeBuffer *gb);		// Restore user position from a restore point

	void SetAllAxesNotHomed();											// Flag all axes as not homed
	void SetMachinePosition(const float positionNow[DRIVES], bool doBedCompensation = true); // Set the current position to be this
	void GetCurrentUserPosition();										// Get the current position form the Move class
	void ToolOffsetTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes], AxesBitmap explicitAxes = 0);	// Convert user coordinates to head reference point coordinates
	void ToolOffsetInverseTransform(const float coordsIn[MaxAxes], float coordsOut[MaxAxes]);	// Convert head reference point coordinates to user coordinates
	const char *TranslateEndStopResult(EndStopHit es);					// Translate end stop result to text
	GCodeResult RetractFilament(GCodeBuffer& gb, bool retract);			// Retract or un-retract filaments
	GCodeResult LoadFilament(GCodeBuffer& gb, StringRef& reply);		// Load the specified filament into a tool
	GCodeResult UnloadFilament(GCodeBuffer& gb, StringRef& reply);		 // Unload the current filament from a tool
	bool ChangeMicrostepping(size_t drive, int microsteps, int mode) const;	// Change microstepping on the specified drive
	void ListTriggers(StringRef reply, TriggerInputsBitmap mask);		// Append a list of trigger inputs to a message
	void CheckTriggers();												// Check for and execute triggers
	void DoEmergencyStop();												// Execute an emergency stop

	void DoPause(GCodeBuffer& gb, bool isAuto)							// Pause the print
	pre(resourceOwners[movementResource] = &gb);

	void SetMappedFanSpeed();											// Set the speeds of fans mapped for the current tool
	void SaveFanSpeeds();												// Save the speeds of all fans

	bool DefineGrid(GCodeBuffer& gb, StringRef &reply);					// Define the probing grid, returning true if error
	bool LoadHeightMap(GCodeBuffer& gb, StringRef& reply) const;		// Load the height map from file
	bool SaveHeightMap(GCodeBuffer& gb, StringRef& reply) const;		// Save the height map to file
	GCodeResult ProbeGrid(GCodeBuffer& gb, StringRef& reply);			// Start probing the grid, returning true if we didn't because of an error

	bool WriteConfigOverrideFile(StringRef& reply, const char *fileName) const; // Write the config-override file
	void CopyConfigFinalValues(GCodeBuffer& gb);						// Copy the feed rate etc. from the daemon to the input channels

	void ClearBabyStepping() { currentBabyStepZOffset = 0.0; }

	MessageType GetMessageBoxDevice(GCodeBuffer& gb) const;				// Decide which device to display a message box on
	void DoManualProbe(GCodeBuffer& gb);								// Do a manual bed probe

	void AppendAxes(StringRef& reply, AxesBitmap axes) const;			// Append a list of axes to a string

	void EndSimulation(GCodeBuffer *gb);								// Restore positions etc. when exiting simulation mode
	bool IsCodeQueueIdle() const;										// Return true if the code queue is idle

	void SaveResumeInfo();

	const char* GetMachineModeString() const;							// Get the name of the current machine mode

	Platform& platform;													// The RepRap machine

	RegularGCodeInput* httpInput;										// These cache incoming G-codes...
	RegularGCodeInput* telnetInput;										// ...
	FileGCodeInput* fileInput;											// ...
	StreamGCodeInput* serialInput;										// ...
	StreamGCodeInput* auxInput;											// ...for the GCodeBuffers below

#ifdef DUET_NG
	GCodeBuffer* gcodeSources[8];										// The various sources of gcodes
	GCodeBuffer*& autoPauseGCode = gcodeSources[7];						// GCode state machine used to run pause.g and resume.g
#else
	GCodeBuffer* gcodeSources[7];										// The various sources of gcodes
#endif

	GCodeBuffer*& httpGCode = gcodeSources[0];
	GCodeBuffer*& telnetGCode = gcodeSources[1];
	GCodeBuffer*& fileGCode = gcodeSources[2];
	GCodeBuffer*& serialGCode = gcodeSources[3];
	GCodeBuffer*& auxGCode = gcodeSources[4];							// This one is for the LCD display on the async serial interface
	GCodeBuffer*& daemonGCode = gcodeSources[5];						// Used for executing config.g and trigger macro files
	GCodeBuffer*& queuedGCode = gcodeSources[6];
	size_t nextGcodeSource;												// The one to check next

	const GCodeBuffer* resourceOwners[NumResources];					// Which gcode buffer owns each resource

	MachineType machineType;					// whether FFF, laser or CNC
	bool active;								// Live and running?
	bool isPaused;								// true if the print has been paused manually or automatically
	bool pausePending;							// true if we have been asked to pause but we are running a macro
#ifdef DUET_NG
	bool isAutoPaused;							// true if the print was paused automatically
#endif
	bool runningConfigFile;						// We are running config.g during the startup process
	bool doingToolChange;						// We are running tool change macros

	float currentUserPosition[MaxAxes];			// The current position of the axes as commanded by the input gcode, before accounting for tool offset and Z hop
	float currentZHop;							// The amount of Z hop that is currently applied

	// The following contain the details of moves that the Move module fetches
	RawMove moveBuffer;							// Move details to pass to Move class
	unsigned int segmentsLeft;					// The number of segments left to do in the current move, or 0 if no move available
	float arcCentre[MaxAxes];
	float arcRadius;
	float arcCurrentAngle;
	float arcAngleIncrement;
	bool doingArcMove;

	RestorePoint simulationRestorePoint;		// The position and feed rate when we started a simulation
	RestorePoint pauseRestorePoint;				// The position and feed rate when we paused the print
	RestorePoint toolChangeRestorePoint;		// The position and feed rate when we freed a tool
	size_t numTotalAxes;						// How many axes we have
	size_t numVisibleAxes;						// How many axes are visible
	size_t numExtruders;						// How many extruders we have, or may have
	float axisOffsets[MaxAxes];					// M206 axis offsets
	float axisScaleFactors[MaxAxes];			// Scale XYZ coordinates by this factor (for Delta configurations)
	float virtualExtruderPosition;				// Virtual extruder position of the last move fed into the Move class
	float rawExtruderTotalByDrive[MaxExtruders]; // Extrusion amount in the last G1 command with an E parameter when in absolute extrusion mode
	float rawExtruderTotal;						// Total extrusion amount fed to Move class since starting print, before applying extrusion factor, summed over all drives
	float record[DRIVES];						// Temporary store for move positions
	float distanceScale;						// MM or inches
	float arcSegmentLength;						// Length of segments that we split arc moves into

	FileData fileToPrint;						// The next file to print
	FilePosition fileOffsetToPrint;				// The offset to print from

	FileStore* fileBeingWritten;				// A file to write G Codes (or sometimes HTML) to
	FilePosition fileSize;						// Size of the file being written

	int oldToolNumber, newToolNumber;			// Tools being changed
	int toolChangeParam;						// Bitmap of all the macros to be run during a tool change

	const char* eofString;						// What's at the end of an HTML file?
	uint8_t eofStringCounter;					// Check the...
	uint8_t eofStringLength;					// ... EoF string as we read.
	bool limitAxes;								// Don't think outside the box.

	AxesBitmap toBeHomed;						// Bitmap of axes still to be homed
	AxesBitmap axesHomed;						// Bitmap of which axes have been homed

	float pausedFanSpeeds[NUM_FANS];			// Fan speeds when the print was paused or a tool change started
	float lastDefaultFanSpeed;					// Last speed given in a M106 command with on fan number
	float pausedDefaultFanSpeed;				// The speed of the default print cooling fan when the print was paused or a tool change started
	float speedFactor;							// speed factor, including the conversion from mm/min to mm/sec, normally 1/60
	float extrusionFactors[MaxExtruders];		// extrusion factors (normally 1.0)
	float volumetricExtrusionFactors[MaxExtruders]; // Volumetric extrusion factors
	float currentBabyStepZOffset;				// The accumulated Z offset due to baby stepping requests

	// Z probe
	GridDefinition defaultGrid;					// The grid defined by the M557 command in config.g
	int32_t g30ProbePointIndex;					// the index of the point we are probing (G30 P parameter), or -1 if none
	int g30SValue;								// S parameter in the G30 command, or -2 if there wasn't one
	float g30zStoppedHeight;					// the height to report after running G30 S-1
	float g30zHeightError;						// the height error last time we probed
	uint32_t lastProbedTime;					// time in milliseconds that the probe was last triggered
	volatile bool zProbeTriggered;				// Set by the step ISR when a move is aborted because the Z probe is triggered
	size_t gridXindex, gridYindex;				// Which grid probe point is next
	bool doingManualBedProbe;					// true if we are waiting for the user to jog the nozzle until it touches the bed
	bool probeIsDeployed;						// true if M401 has been used to deploy the probe and M402 has not yet been used t0 retract it

	float simulationTime;						// Accumulated simulation time
	uint8_t simulationMode;						// 0 = not simulating, 1 = simulating, >1 are simulation modes for debugging
	bool exitSimulationWhenFileComplete;		// true if simulating a file

	// Firmware retraction settings
	float retractLength, retractExtra;			// retraction length and extra length to un-retract
	float retractSpeed;							// retract speed in mm/min
	float unRetractSpeed;						// un=retract speed in mm/min
	float retractHop;							// Z hop when retracting
	bool isRetracted;							// true if filament has been firmware-retracted

	// Triggers
	Trigger triggers[MaxTriggers];				// Trigger conditions
	TriggerInputsBitmap lastEndstopStates;		// States of the trigger inputs last time we looked
	static_assert(MaxTriggers <= 32, "Too many triggers");
	TriggerNumbersBitmap triggersPending;		// Bitmap of triggers pending but not yet executed

	// Firmware update
	uint8_t firmwareUpdateModuleMap;			// Bitmap of firmware modules to be updated
	bool isFlashing;							// Is a new firmware binary going to be flashed?

	// Code queue
	GCodeQueue *codeQueue;						// Stores certain codes for deferred execution

	// SHA1 hashing
	FileStore *fileBeingHashed;
	SHA1Context hash;
	bool StartHash(const char* filename);
	GCodeResult AdvanceHash(StringRef &reply);

	// Filament monitoring
	FilamentSensorStatus lastFilamentError;
	size_t lastFilamentErrorExtruder;

	// CNC and laser
	float spindleMaxRpm;
	float laserMaxPower;

	// Misc
	uint32_t longWait;							// Timer for things that happen occasionally (seconds)
	uint32_t lastWarningMillis;					// When we last sent a warning message for things that can happen very often
	AxesBitmap axesToSenseLength;				// The axes on which we are performing axis length sensing
	int8_t lastAuxStatusReportType;				// The type of the last status report requested by PanelDue
	bool isWaiting;								// True if waiting to reach temperature
	bool cancelWait;							// Set true to cancel waiting
	bool displayNoToolWarning;					// True if we need to display a 'no tool selected' warning
	bool displayDeltaNotHomedWarning;			// True if we need to display a 'attempt to move before homing on a delta printer' message
	char filamentToLoad[FilamentNameLength];	// Name of the filament being loaded

	static constexpr const char* BED_EQUATION_G = "bed.g";
	static constexpr const char* RESUME_G = "resume.g";
	static constexpr const char* CANCEL_G = "cancel.g";
	static constexpr const char* STOP_G = "stop.g";
	static constexpr const char* SLEEP_G = "sleep.g";
	static constexpr const char* CONFIG_OVERRIDE_G = "config-override.g";
	static constexpr const char* DEPLOYPROBE_G = "deployprobe.g";
	static constexpr const char* RETRACTPROBE_G = "retractprobe.g";
	static constexpr const char* RESUME_PROLOGUE_G = "resurrect-prologue.g";
	static constexpr const char* PAUSE_G = "pause.g";
	static constexpr const char* HOME_ALL_G = "homeall.g";
	static constexpr const char* HOME_DELTA_G = "homedelta.g";
	static constexpr const char* DefaultHeightMapFile = "heightmap.csv";
	static constexpr const char* LOAD_FILAMENT_G = "load.g";
	static constexpr const char* UNLOAD_FILAMENT_G = "unload.g";

#ifdef DUET_NG
	static constexpr const char* POWER_FAIL_G = "powerfail.g";
	static constexpr const char* POWER_RESTORE_G = "powerrestore.g";
#endif

	static constexpr const char* RESUME_AFTER_POWER_FAIL_G = "resurrect.g";

	static constexpr const float MinServoPulseWidth = 544.0, MaxServoPulseWidth = 2400.0;
	static const uint16_t ServoRefreshFrequency = 50;
};

//*****************************************************************************************************

#endif
