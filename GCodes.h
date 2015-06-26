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

#include "GCodeBuffer.h"

const unsigned int StackSize = 5;

const char feedrateLetter = 'F';						// GCode feedrate
const char extrudeLetter = 'E'; 						// GCode extrude

// Type for specifying which endstops we want to check
typedef uint16_t EndstopChecks;							// must be large enough to hold a bitmap of drive numbers or ZProbeActive
const EndstopChecks ZProbeActive = 1 << 15;				// must be distinct from 1 << (any drive number)

const float minutesToSeconds = 60.0;
const float secondsToMinutes = 1.0/minutesToSeconds;

// Enumeration to list all the possible states that the Gcode processing machine may be in
enum class GCodeState
{
	normal,												// not doing anything and ready to process a new GCode
	waitingForMoveToComplete,							// doing a homing move, so we must wait for it to finish before processing another GCode
	homing,
	setBed1,
	setBed2,
	setBed3,
	toolChange1,
	toolChange2,
	toolChange3,
	pausing1,
	pausing2,
	resuming1,
	resuming2,
	resuming3
};

// Small class to stack the state when we execute a macro file
class GCodeMachineState
{
public:
	GCodeState state;
	GCodeBuffer *gb;									// this may be null when executing config.g
	float feedrate;
	FileData fileState;
	bool drivesRelative;
	bool axesRelative;
	bool doingFileMacro;
};

//****************************************************************************************************

// The GCode interpreter

class GCodes
{   
  public:
  
    GCodes(Platform* p, Webserver* w);
    void Spin();														// Called in a tight loop to make this class work
    void Init();														// Set it up
    void Exit();														// Shut it down
    void Reset();														// Reset some parameter to defaults
    bool ReadMove(float* m, EndstopChecks& ce, uint8_t& rMoveType, FilePosition& fPos);	// Called by the Move class to get a movement set by the last G Code
    void ClearMove();
    void QueueFileToPrint(const char* fileName);						// Open a file of G Codes to run
    void DeleteFile(const char* fileName);								// Does what it says
    bool GetProbeCoordinates(int count, float& x, float& y, float& z) const;	// Get pre-recorded probe coordinates
    void GetCurrentCoordinates(StringRef& s) const;						// Write where we are into a string
    bool DoingFileMacro() const;										// Or still busy processing a macro file?
    float FractionOfFilePrinted() const;								// Get fraction of file printed
    void Diagnostics();													// Send helpful information out
    bool HaveIncomingData() const;										// Is there something that we have to do?
    bool GetAxisIsHomed(uint8_t axis) const { return axisIsHomed[axis]; } // Is the axis at 0?
    void SetAxisIsHomed(uint8_t axis) { axisIsHomed[axis] = true; }		// Tell us that the axis is now homed
    bool CoolingInverted() const;										// Is the current fan value inverted?

    void PauseSDPrint();												// Pause the current print from SD card
    float GetSpeedFactor() const { return speedFactor * minutesToSeconds; }	// Return the current speed factor
    const float *GetExtrusionFactors() const { return extrusionFactors; } // Return the current extrusion factors
    float GetRawExtruderPosition(size_t drive) const;					// Get the actual extruder position, after adjusting the extrusion factor
    
    bool HaveAux() const { return auxDetected; }						// Any device on the AUX line?

    bool IsPaused() const;
    bool IsPausing() const;
    bool IsResuming() const;

  private:
  
    void StartNextGCode(StringRef& reply);								// Fetch a new GCode and process it
    void DoFilePrint(GCodeBuffer* gb, StringRef& reply);				// Get G Codes from a file and print them
    bool AllMovesAreFinishedAndMoveBufferIsLoaded();					// Wait for move queue to exhaust and the current position is loaded
    bool DoCannedCycleMove(EndstopChecks ce);							// Do a move from an internally programmed canned cycle
    bool DoFileMacro(const char* fileName, bool reportMissing = true);	// Run a GCode macro in a file, optionally report error if not found
    void FileMacroCyclesReturn();										// End a macro
    bool ActOnCode(GCodeBuffer* gb, StringRef& reply);					// Do a G, M or T Code
    bool HandleGcode(GCodeBuffer* gb, StringRef& reply);				// Do a G code
    bool HandleMcode(GCodeBuffer* gb, StringRef& reply);				// Do an M code
    bool HandleTcode(GCodeBuffer* gb, StringRef& reply);				// Do a T code
    void CancelPrint();													// Cancel the current print
    int SetUpMove(GCodeBuffer* gb, StringRef& reply);					// Pass a move on to the Move module
    bool DoDwell(GCodeBuffer *gb);										// Wait for a bit
    bool DoDwellTime(float dwell);										// Really wait for a bit
    bool DoSingleZProbeAtPoint(int probePointIndex);					// Probe at a given point
    bool DoSingleZProbe();												// Probe where we are
    int DoZProbe(float distance);										// Do a Z probe cycle up to the maximum specified distance
    bool SetSingleZProbeAtAPosition(GCodeBuffer *gb, StringRef& reply);	// Probes at a given position - see the comment at the head of the function itself
    void SetBedEquationWithProbe(int sParam, StringRef& reply);			// Probes a series of points and sets the bed equation
    bool SetPrintZProbe(GCodeBuffer *gb, StringRef& reply);				// Either return the probe value, or set its threshold
    void SetOrReportOffsets(StringRef& reply, GCodeBuffer *gb);			// Deal with a G10
    bool SetPositions(GCodeBuffer *gb);									// Deal with a G92
    bool LoadMoveBufferFromGCode(GCodeBuffer *gb,  						// Set up a move for the Move class
    		bool doingG92, bool applyLimits);
    bool NoHome() const;												// Are we homing and not finished?
    void Push();														// Push feedrate etc on the stack
    void Pop();															// Pop feedrate etc
    void DisableDrives();												// Turn the motors off
    void SetEthernetAddress(GCodeBuffer *gb, int mCode);				// Does what it says
    void SetMACAddress(GCodeBuffer *gb);								// Deals with an M540
    void HandleReply(bool error, const GCodeBuffer *gb, 				// If the GCode is from the serial interface, reply to it
    		 const char* reply, char gMOrT, int code, bool resend);
    bool OpenFileToWrite(const char* directory,							// Start saving GCodes in a file
    		const char* fileName, GCodeBuffer *gb);
    void WriteGCodeToFile(GCodeBuffer *gb);								// Write this GCode into a file
    bool SendConfigToLine();											// Deal with M503
    void WriteHTMLToFile(char b, GCodeBuffer *gb);						// Save an HTML file (usually to upload a new web interface)
    bool OffsetAxes(GCodeBuffer *gb);									// Set offsets - deprecated, use G10
    void SetPidParameters(GCodeBuffer *gb, int heater, StringRef& reply);	// Set the P/I/D parameters for a heater
    void SetHeaterParameters(GCodeBuffer *gb, StringRef& reply);		 // Set the thermistor and ADC parameters for a heater
    int8_t Heater(int8_t head) const;									// Legacy G codes start heaters at 0, but we use 0 for the bed.  This sorts that out.
    void ManageTool(GCodeBuffer *gb, StringRef& reply);					// Create a new tool definition
    void SetToolHeaters(Tool *tool, float temperature);					// Set all a tool's heaters to the temperature.  For M104...
    bool ToolHeatersAtSetTemperatures(const Tool *tool) const;			// Wait for the heaters associated with the specified tool to reach their set temperatures
    bool AllAxesAreHomed() const;										// Return true if all axes are homed
    void SetAllAxesNotHomed();											// Flag all axes as not homed
    void SetPositions(float positionNow[DRIVES]);						// Set the current position to be this

    Platform* platform;							// The RepRap machine
    bool active;								// Live and running?
    bool isPaused;								// true if the print has been paused
    Webserver* webserver;						// The webserver class
    float dwellTime;							// How long a pause for a dwell (seconds)?
    bool dwellWaiting;							// We are in a dwell
    GCodeBuffer* webGCode;						// The sources...
    GCodeBuffer* fileGCode;						// ...
    GCodeBuffer* serialGCode;					// ...
    GCodeBuffer* auxGCode;						// this one is for the LCD display on the async serial interface
    GCodeBuffer* fileMacroGCode;				// ...
    GCodeBuffer *gbCurrent;
    bool moveAvailable;							// Have we seen a move G Code and set it up?
    float moveBuffer[DRIVES+1]; 				// Move coordinates; last is feed rate
    float savedMoveBuffer[DRIVES+1];			// The position and feedrate when we started the current simulation
    float pausedMoveBuffer[DRIVES+1]; 			// Move coordinates; last is feed rate
    EndstopChecks endStopsToCheck;				// Which end stops we check them on the next move
    uint8_t moveType;							// 0 = normal move, 1 = homing move, 2 = direct motor move
    GCodeState state;							// The main state variable of the GCode state machine
	bool drivesRelative;
	bool axesRelative;
    GCodeMachineState stack[StackSize];			// State that we save when calling macro files
    unsigned int stackPointer;					// Push and Pop stack pointer
    static const char axisLetters[AXES]; 		// 'X', 'Y', 'Z'
    float lastRawExtruderPosition[DRIVES - AXES];	// Extruder position of the last move fed into the Move class
	float record[DRIVES+1];						// Temporary store for move positions
	float moveToDo[DRIVES+1];					// Where to go set by G1 etc
	bool activeDrive[DRIVES+1];					// Is this drive involved in a move?
	bool offSetSet;								// Are any axis offsets non-zero?
    float distanceScale;						// MM or inches
    FileData fileBeingPrinted;
    FileData fileToPrint;
    FileStore* fileBeingWritten;				// A file to write G Codes (or sometimes HTML) in
    FileStore* configFile;						// A file containing a macro
    uint16_t toBeHomed;							// Bitmap of axes still to be homed
    bool doingFileMacro;						// Are we executing a macro file?
    int oldToolNumber, newToolNumber;			// Tools being changed
    const char* eofString;						// What's at the end of an HTML file?
    uint8_t eofStringCounter;					// Check the...
    uint8_t eofStringLength;					// ... EoF string as we read.
    int probeCount;								// Counts multiple probe points
    int8_t cannedCycleMoveCount;				// Counts through internal (i.e. not macro) canned cycle moves
    bool cannedCycleMoveQueued;					// True if a canned cycle move has been set
    bool zProbesSet;							// True if all Z probing is done and we can set the bed equation
    float longWait;								// Timer for things that happen occasionally (seconds)
    bool limitAxes;								// Don't think outside the box.
    bool axisIsHomed[AXES];						// These record which of the axes have been homed
    bool coolingInverted;
    float pausedFanValue;
    float speedFactor;							// speed factor, including the conversion from mm/min to mm/sec, normally 1/60
    float speedFactorChange;					// factor by which we changed the speed factor since the last move
    float extrusionFactors[DRIVES - AXES];		// extrusion factors (normally 1.0)
    float lastProbedZ;							// the last height at which the Z probe stopped

    bool auxDetected;							// Have we processed at least one G-Code from an AUX device?
    bool simulating;
    float simulationTime;
    FilePosition filePos;						// The position we got up to in the file being printed
    FilePosition moveFilePos;					// Saved version of filePos for the next real move to be processed
};

//*****************************************************************************************************

inline bool GCodes::DoingFileMacro() const
{
	return doingFileMacro;
}

inline bool GCodes::HaveIncomingData() const
{
	return fileBeingPrinted.IsLive() ||
			webserver->GCodeAvailable() ||
			(platform->GetLine()->Status() & (uint8_t)IOStatus::byteAvailable) ||
			(platform->GetAux()->Status() & (uint8_t)IOStatus::byteAvailable);
}

// This function takes care of the fact that the heater and head indices don't match because the bed is heater 0.
inline int8_t GCodes::Heater(int8_t head) const
{
   return head+1; 
}

inline bool GCodes::CoolingInverted() const
{
	return coolingInverted;
}

inline bool GCodes::AllAxesAreHomed() const
{
	return axisIsHomed[X_AXIS] && axisIsHomed[Y_AXIS] && axisIsHomed[Z_AXIS];
}

inline void GCodes::SetAllAxesNotHomed()
{
	axisIsHomed[X_AXIS] = axisIsHomed[Y_AXIS] = axisIsHomed[Z_AXIS] = false;
}

#endif
