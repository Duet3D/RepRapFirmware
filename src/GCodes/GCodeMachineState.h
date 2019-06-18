/*
 * GCodeMachineState.h
 *
 *  Created on: 15 Nov 2016
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEMACHINESTATE_H_
#define SRC_GCODES_GCODEMACHINESTATE_H_

#include "RepRapFirmware.h"
#include "Storage/FileData.h"

// Enumeration to list all the possible states that the Gcode processing machine may be in
enum class GCodeState : uint8_t
{
	normal,												// not doing anything and ready to process a new GCode

	waitingForSpecialMoveToComplete,					// doing a special move, so we must wait for it to finish before processing another GCode
	waitingForSegmentedMoveToGo,						// doing an arc move, so we must check whether it completes normally

	probingToolOffset,

	findCenterOfCavityMin,
	findCenterOfCavityR,
	findCenterOfCavityMax,

	homing1,
	homing2,

	// These next 4 must be contiguous
	toolChange0,
	toolChange1,
	toolChange2,
	toolChangeComplete,

	// These next 5 must be contiguous
	m109ToolChange0,
	m109ToolChange1,
	m109ToolChange2,
	m109ToolChangeComplete,
	m109WaitForTemperature,

	pausing1,
	pausing2,

	filamentChangePause1,
	filamentChangePause2,

	resuming1,
	resuming2,
	resuming3,

	flashing1,
	flashing2,

	stoppingWithHeatersOff,
	stoppingWithHeatersOn,

	// These next 9 must be contiguous
	gridProbing1,
	gridProbing2a,
	gridProbing2b,
	gridProbing3,
	gridProbing4,
	gridProbing4a,
	gridProbing5,
	gridProbing6,
	gridProbing7,

	// These next 10 must be contiguous
	probingAtPoint0,
	probingAtPoint1,
	probingAtPoint2a,
	probingAtPoint2b,
	probingAtPoint3,
	probingAtPoint4,
	probingAtPoint4a,
	probingAtPoint5,
	probingAtPoint6,
	probingAtPoint7,

	doingFirmwareRetraction,
	doingFirmwareUnRetraction,
	loadingFilament,
	unloadingFilament,

#if HAS_HIGH_SPEED_SD
	timingSDwrite,
#endif

#if HAS_VOLTAGE_MONITOR
	powerFailPausing1
#endif
};

// Class to hold the state of gcode execution for some input source
class GCodeMachineState
{
public:
	typedef uint32_t ResourceBitmap;
	GCodeMachineState();

	GCodeMachineState *previous;
	float feedRate;
#if HAS_HIGH_SPEED_SD
	FileData fileState;
#elif HAS_LINUX_INTERFACE
	uint32_t fileId;							// virtual file ID to deal with stack push/pops when a file is being cancelled or finished in the wrong stack level
#endif
	ResourceBitmap lockedResources;
	const char *errorMessage;
	GCodeState state;
	uint8_t toolChangeParam;
	int16_t newToolNumber;
	unsigned int
		drivesRelative : 1,
		axesRelative : 1,
#if HAS_LINUX_INTERFACE
		isFileFinished : 1,
#endif
		doingFileMacro : 1,
		waitWhileCooling : 1,
		runningM501 : 1,
		runningM502 : 1,
		volumetricExtrusion : 1,
		g53Active : 1,							// true if seen G53 on this line of GCode
		runningSystemMacro : 1,					// true if running a system macro file
		usingInches : 1,						// true if units are inches not mm
		waitingForAcknowledgement : 1,
		messageAcknowledged : 1,
		messageCancelled : 1;

	static GCodeMachineState *Allocate()
	post(!result.IsLive(); result.state == GCodeState::normal);

#if HAS_HIGH_SPEED_SD
	bool DoingFile() const { return fileState.IsLive(); }
#elif HAS_LINUX_INTERFACE
	bool DoingFile() const { return fileId == 0; }

	void SetFileExecuting();
	void SetFileFinished();
#endif
	void CloseFile();
	bool UsingMachineCoordinates() const { return g53Active || runningSystemMacro; }

	// Copy values that may have been altered by config.g into this state record
	void CopyStateFrom(const GCodeMachineState& other)
	{
		drivesRelative = other.drivesRelative;
		axesRelative = other.axesRelative;
		feedRate = other.feedRate;
		volumetricExtrusion = other.volumetricExtrusion;
		usingInches = other.usingInches;
	}

	static void Release(GCodeMachineState *ms);
	static unsigned int GetNumAllocated() { return numAllocated; }
	static unsigned int GetNumInUse();

private:
	static GCodeMachineState *freeList;
	static unsigned int numAllocated;
};

#endif /* SRC_GCODES_GCODEMACHINESTATE_H_ */
