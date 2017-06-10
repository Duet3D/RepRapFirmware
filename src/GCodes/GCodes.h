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
#include "Libraries/sha1/sha1.h"
#include "Platform.h"		// for type EndStopHit
#include "GCodeInput.h"

class GCodeBuffer;
class GCodeQueue;

const char feedrateLetter = 'F';						// GCode feedrate
const char extrudeLetter = 'E'; 						// GCode extrude

// Type for specifying which endstops we want to check
typedef uint16_t EndstopChecks;							// must be large enough to hold a bitmap of drive numbers or ZProbeActive
const EndstopChecks ZProbeActive = 1 << 15;				// must be distinct from 1 << (any drive number)
const EndstopChecks LogProbeChanges = 1 << 14;			// must be distinct from 1 << (any drive number)

typedef uint16_t TriggerMask;

struct Trigger
{
	TriggerMask rising;
	TriggerMask falling;
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

// Bits for T-code P-parameter to specify which macros are supposted to be run
const int TFreeBit = 1 << 0;
const int TPreBit = 1 << 1;
const int TPostBit = 1 << 2;
const int DefaultToolChangeParam = TFreeBit | TPreBit | TPostBit;

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
		FilePosition filePos;											// offset in the file being printed at the end of reading this move
		uint32_t xAxes;													// axes that X is mapped to
		float newBabyStepping;											// the adjustment we made to the Z offset in this move
		EndstopChecks endStopsToCheck;									// endstops to check
		uint8_t moveType;												// the S parameter from the G0 or G1 command, 0 for a normal move
		bool isFirmwareRetraction;										// true if this is a firmware retraction/un-retraction move
		bool usePressureAdvance;										// true if we want to us extruder pressure advance, if there is any extrusion
		bool canPauseAfter;												// true if we can pause just after this move and successfully restart
		bool hasExtrusion;												// true if the move includes extrusion - only valid if the move was set up by SetupMove
	};
  
	GCodes(Platform& p);
	void Spin();														// Called in a tight loop to make this class work
	void Init();														// Set it up
	void Exit();														// Shut it down
	void Reset();														// Reset some parameter to defaults
	bool ReadMove(RawMove& m);											// Called by the Move class to get a movement set by the last G Code
	void ClearMove();
	void QueueFileToPrint(const char* fileName);						// Open a file of G Codes to run
	void DeleteFile(const char* fileName);								// Does what it says
	void GetCurrentCoordinates(StringRef& s) const;						// Write where we are into a string
	bool DoingFileMacro() const;										// Or still busy processing a macro file?
	float FractionOfFilePrinted() const;								// Get fraction of file printed
	void Diagnostics(MessageType mtype);								// Send helpful information out

	bool RunConfigFile(const char* fileName);							// Start running the config file
	bool IsDaemonBusy() const;											// Return true if the daemon is busy running config.g or a trigger file

	bool GetAxisIsHomed(unsigned int axis) const						// Has the axis been homed?
		{ return (axesHomed & (1 << axis)) != 0; }
	void SetAxisIsHomed(unsigned int axis)								// Tell us that the axis is now homed
		{ axesHomed |= (1 << axis); }
	void SetAxisNotHomed(unsigned int axis)								// Tell us that the axis is not homed
		{ axesHomed &= ~(1 << axis); }

	float GetSpeedFactor() const { return speedFactor * MinutesToSeconds; }	// Return the current speed factor
	float GetExtrusionFactor(size_t extruder) { return extrusionFactors[extruder]; } // Return the current extrusion factors
	float GetRawExtruderPosition(size_t drive) const;					// Get the actual extruder position, after adjusting the extrusion factor
	float GetRawExtruderTotalByDrive(size_t extruder) const;			// Get the total extrusion since start of print, for one drive
	float GetTotalRawExtrusion() const { return rawExtruderTotal; }		// Get the total extrusion since start of print, all drives
	float GetBabyStepOffset() const;									// Get the current baby stepping Z offset

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

	void CancelPrint();													// Cancel the current print

	void MoveStoppedByZProbe() { zProbeTriggered = true; }				// Called from the step ISR when the Z probe is triggered, causing the move to be aborted

	size_t GetTotalAxes() const { return numTotalAxes; }
	size_t GetVisibleAxes() const { return numVisibleAxes; }
	size_t GetNumExtruders() const { return numExtruders; }

	static const char axisLetters[MaxAxes]; 							// 'X', 'Y', 'Z'

private:
	GCodes(const GCodes&);												// private copy constructor to prevent copying

	enum class CannedMoveType : uint8_t { none, relative, absolute };

	struct RestorePoint
	{
		float moveCoords[DRIVES];
		float feedRate;
		RestorePoint() { Init(); }
		void Init();
	};

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
	bool DoCannedCycleMove(GCodeBuffer& gb, EndstopChecks ce);			// Do a move from an internally programmed canned cycle
	void FileMacroCyclesReturn(GCodeBuffer& gb);						// End a macro
	bool ActOnCode(GCodeBuffer& gb, StringRef& reply);					// Do a G, M or T Code
	bool HandleGcode(GCodeBuffer& gb, StringRef& reply);				// Do a G code
	bool HandleMcode(GCodeBuffer& gb, StringRef& reply);				// Do an M code
	bool HandleTcode(GCodeBuffer& gb, StringRef& reply);				// Do a T code
	int SetUpMove(GCodeBuffer& gb, StringRef& reply);					// Pass a move on to the Move module
	bool DoDwell(GCodeBuffer& gb);										// Wait for a bit
	bool DoDwellTime(GCodeBuffer& gb, uint32_t dwellMillis);			// Really wait for a bit
	bool DoHome(GCodeBuffer& gb, StringRef& reply, bool& error);		// Home some axes
	bool DoSingleZProbeAtPoint(GCodeBuffer& gb, size_t probePointIndex, float heightAdjust); // Probe at a given point
	bool DoSingleZProbe(GCodeBuffer& gb, StringRef& reply, bool reportOnly, float heightAdjust); // Probe where we are
	int DoZProbe(GCodeBuffer& gb, float distance);						// Do a Z probe cycle up to the maximum specified distance
	bool SetSingleZProbeAtAPosition(GCodeBuffer& gb, StringRef& reply);	// Probes at a given position - see the comment at the head of the function itself
	void SetBedEquationWithProbe(int sParam, StringRef& reply);			// Probes a series of points and sets the bed equation
	bool SetPrintZProbe(GCodeBuffer& gb, StringRef& reply);				// Either return the probe value, or set its threshold
	bool SetOrReportOffsets(GCodeBuffer& gb, StringRef& reply);			// Deal with a G10

	bool SetPositions(GCodeBuffer& gb);									// Deal with a G92
	bool LoadExtrusionAndFeedrateFromGCode(GCodeBuffer& gb, int moveType); // Set up the extrusion and feed rate of a move for the Move class
	unsigned int LoadMoveBufferFromGCode(GCodeBuffer& gb, int moveType); // Set up the axis coordinates of a move for the Move class
	bool DoArcMove(GCodeBuffer& gb, bool clockwise)						// Execute an arc move returning true if it was badly-formed
		pre(segmentsLeft == 0; resourceOwners[MoveResource] == &gb);

	bool Push(GCodeBuffer& gb);											// Push feedrate etc on the stack
	void Pop(GCodeBuffer& gb);											// Pop feedrate etc
	void DisableDrives();												// Turn the motors off
	void SetMACAddress(GCodeBuffer& gb);								// Deals with an M540
	void HandleReply(GCodeBuffer& gb, bool error, const char *reply);	// Handle G-Code replies
	void HandleReply(GCodeBuffer& gb, bool error, OutputBuffer *reply);
	bool OpenFileToWrite(GCodeBuffer& gb, const char* directory, const char* fileName);	// Start saving GCodes in a file
	bool SendConfigToLine();											// Deal with M503
	bool OffsetAxes(GCodeBuffer& gb);									// Set offsets - deprecated, use G10
	void SetPidParameters(GCodeBuffer& gb, int heater, StringRef& reply); // Set the P/I/D parameters for a heater
	bool SetHeaterParameters(GCodeBuffer& gb, StringRef& reply);		// Set the thermistor and ADC parameters for a heater, returning true if an error occurs
	void ManageTool(GCodeBuffer& gb, StringRef& reply);					// Create a new tool definition
	void SetToolHeaters(Tool *tool, float temperature);					// Set all a tool's heaters to the temperature.  For M104...
	bool ToolHeatersAtSetTemperatures(const Tool *tool, bool waitWhenCooling) const; // Wait for the heaters associated with the specified tool to reach their set temperatures
	void GenerateTemperatureReport(StringRef& reply) const;				// Store a standard-format temperature report in reply
	OutputBuffer *GenerateJsonStatusResponse(int type, int seq, ResponseSource source) const;	// Generate a M408 response
	void CheckReportDue(GCodeBuffer& gb, StringRef& reply) const;		// Check whether we need to report temperatures or status

	void SetAllAxesNotHomed();											// Flag all axes as not homed
	void SetPositions(const float positionNow[DRIVES], bool doBedCompensation = true); // Set the current position to be this
	const char *TranslateEndStopResult(EndStopHit es);					// Translate end stop result to text
	bool RetractFilament(GCodeBuffer& gb, bool retract);				// Retract or un-retract filaments
	bool ChangeMicrostepping(size_t drive, int microsteps, int mode) const;	// Change microstepping on the specified drive
	void ListTriggers(StringRef reply, TriggerMask mask);				// Append a list of trigger endstops to a message
	void CheckTriggers();												// Check for and execute triggers
	void DoEmergencyStop();												// Execute an emergency stop

	void DoPause(GCodeBuffer& gb)										// Pause the print
	pre(resourceOwners[movementResource] = &gb);

	void SetMappedFanSpeed();											// Set the speeds of fans mapped for the current tool
	void SaveFanSpeeds();												// Save the speeds of all fans

	bool DefineGrid(GCodeBuffer& gb, StringRef &reply);					// Define the probing grid, returning true if error
	bool ProbeGrid(GCodeBuffer& gb, StringRef& reply);					// Start probing the grid, returning true if we didn't because of an error
	bool LoadHeightMap(GCodeBuffer& gb, StringRef& reply) const;		// Load the height map from file
	bool SaveHeightMap(GCodeBuffer& gb, StringRef& reply) const;		// Save the height map to file

	bool WriteConfigOverrideFile(StringRef& reply, const char *fileName) const; // Write the config-override file
	void CopyConfigFinalValues(GCodeBuffer& gb);						// Copy the feed rate etc. from the daemon to the input channels

	void ClearBabyStepping();

	static uint32_t LongArrayToBitMap(const long *arr, size_t numEntries);	// Convert an array of longs to a bit map

	Platform& platform;													// The RepRap machine

	RegularGCodeInput* httpInput;										// These cache incoming G-codes...
	RegularGCodeInput* telnetInput;										// ...
	FileGCodeInput* fileInput;											// ...
	StreamGCodeInput* serialInput;										// ...
	StreamGCodeInput* auxInput;											// ...for the GCodeBuffers below

	GCodeBuffer* gcodeSources[7];										// The various sources of gcodes

	GCodeBuffer*& httpGCode = gcodeSources[0];
	GCodeBuffer*& telnetGCode = gcodeSources[1];
	GCodeBuffer*& fileGCode = gcodeSources[2];
	GCodeBuffer*& serialGCode = gcodeSources[3];
	GCodeBuffer*& auxGCode = gcodeSources[4];							// This one is for the LCD display on the async serial interface
	GCodeBuffer*& daemonGCode = gcodeSources[5];						// Used for executing config.g and trigger macro files
	GCodeBuffer*& queuedGCode = gcodeSources[6];
	size_t nextGcodeSource;												// The one to check next

	const GCodeBuffer* resourceOwners[NumResources];					// Which gcode buffer owns each resource

	bool active;								// Live and running?
	bool isPaused;								// true if the print has been paused
	bool runningConfigFile;						// We are running config.g during the startup process
	bool doingToolChange;						// We are running tool change macros

	// The following contain the details of moves that the Move module fetches
	RawMove moveBuffer;							// Move details to pass to Move class
	unsigned int segmentsLeft;					// The number of segments left to do in the current move, or 0 if no move available
	float arcCentre[MaxAxes];
	float arcRadius;
	float arcCurrentAngle;
	float arcAngleIncrement;
	uint32_t arcAxesMoving;
	bool doingArcMove;

	RestorePoint simulationRestorePoint;		// The position and feed rate when we started a simulation
	RestorePoint pauseRestorePoint;				// The position and feed rate when we paused the print
	RestorePoint toolChangeRestorePoint;		// The position and feed rate when we freed a tool
	size_t numTotalAxes;						// How many axes we have
	size_t numVisibleAxes;						// How many axes are visible
	size_t numExtruders;						// How many extruders we have, or may have
	float axisScaleFactors[MaxAxes];			// Scale XYZ coordinates by this factor (for Delta configurations)
	float lastRawExtruderPosition[MaxExtruders]; // Extruder position of the last move fed into the Move class
	float rawExtruderTotalByDrive[MaxExtruders]; // Total extrusion amount fed to Move class since starting print, before applying extrusion factor, per drive
	float rawExtruderTotal;						// Total extrusion amount fed to Move class since starting print, before applying extrusion factor, summed over all drives
	float record[DRIVES];						// Temporary store for move positions
	float cannedMoveCoords[DRIVES];				// Where to go or how much to move by in a canned cycle move, last is feed rate
	float cannedFeedRate;						// How fast to do it
	CannedMoveType cannedMoveType[DRIVES];		// Is this drive involved in a canned cycle move?
	bool offSetSet;								// Are any axis offsets non-zero?
	float distanceScale;						// MM or inches
	float arcSegmentLength;						// Length of segments that we split arc moves into
	FileData fileToPrint;
	FileStore* fileBeingWritten;				// A file to write G Codes (or sometimes HTML) to
	uint16_t toBeHomed;							// Bitmap of axes still to be homed
	int oldToolNumber, newToolNumber;			// Tools being changed
	int toolChangeParam;						// Bitmap of all the macros to be run during a tool change

	const char* eofString;						// What's at the end of an HTML file?
	uint8_t eofStringCounter;					// Check the...
	uint8_t eofStringLength;					// ... EoF string as we read.
	size_t probeCount;							// Counts multiple probe points
	int8_t cannedCycleMoveCount;				// Counts through internal (i.e. not macro) canned cycle moves
	bool cannedCycleMoveQueued;					// True if a canned cycle move has been set
	bool limitAxes;								// Don't think outside the box.
	uint32_t axesHomed;							// Bitmap of which axes have been homed
	float pausedFanSpeeds[NUM_FANS];			// Fan speeds when the print was paused or a tool change started
	float lastDefaultFanSpeed;					// Last speed given in a M106 command with on fan number
	float pausedDefaultFanSpeed;				// The speed of the default print cooling fan when the print was paused or a tool change started
	float speedFactor;							// speed factor, including the conversion from mm/min to mm/sec, normally 1/60
	float extrusionFactors[MaxExtruders];		// extrusion factors (normally 1.0)
	float currentBabyStepZOffset;				// The accumulated Z offset due to baby stepping requests
	float pendingBabyStepZOffset;				// The amount of additional baby stepping requested but not yet acted on

	// Z probe
	float lastProbedZ;							// the last height at which the Z probe stopped
	uint32_t lastProbedTime;					// time in milliseconds that the probe was last triggered
	volatile bool zProbeTriggered;				// Set by the step ISR when a move is aborted because the Z probe is triggered
	size_t gridXindex, gridYindex;				// Which grid probe point is next

	float simulationTime;						// Accumulated simulation time
	uint8_t simulationMode;						// 0 = not simulating, 1 = simulating, >1 are simulation modes for debugging

	// Firmware retraction settings
	float retractLength, retractExtra;			// retraction length and extra length to un-retract
	float retractSpeed;							// retract speed in mm/min
	float unRetractSpeed;						// un=retract speed in mm/min
	float retractHop;							// Z hop when retracting
	bool isRetracted;							// true if filament has been firmware-retracted

	// Triggers
	Trigger triggers[MaxTriggers];				// Trigger conditions
	TriggerMask lastEndstopStates;				// States of the endstop inputs last time we looked
	static_assert(MaxTriggers <= 32, "Too many triggers");
	uint32_t triggersPending;					// Bitmap of triggers pending but not yet executed

	// Firmware update
	uint8_t firmwareUpdateModuleMap;			// Bitmap of firmware modules to be updated
	bool isFlashing;							// Is a new firmware binary going to be flashed?

	// Code queue
	GCodeQueue *codeQueue;						// Stores certain codes for deferred execution

	// SHA1 hashing
	FileStore *fileBeingHashed;
	SHA1Context hash;
	bool StartHash(const char* filename);
	bool AdvanceHash(StringRef &reply);

	// Misc
	float longWait;								// Timer for things that happen occasionally (seconds)
	uint32_t lastWarningMillis;					// When we last sent a warning message for things that can happen very often
	int8_t lastAuxStatusReportType;				// The type of the last status report requested by PanelDue
	bool isWaiting;								// True if waiting to reach temperature
	bool cancelWait;							// Set true to cancel waiting
	bool displayNoToolWarning;					// True if we need to display a 'no tool selected' warning
	bool displayDeltaNotHomedWarning;			// True if we need to display a 'attempt to move before homing on a delta printer' message
};

//*****************************************************************************************************

#endif
