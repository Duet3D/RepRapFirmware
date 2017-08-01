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
	waitingForMoveToComplete,							// doing a homing move, so we must wait for it to finish before processing another GCode
	homing,

	// These next 4 must be contiguous
	toolChange0,
	toolChange1,
	toolChange2,
	toolChangeComplete,

	// These next 4 must be contiguous
	m109ToolChange0,
	m109ToolChange1,
	m109ToolChange2,
	m109ToolChangeComplete,

	pausing1,
	pausing2,
	resuming1,
	resuming2,
	resuming3,
	flashing1,
	flashing2,
	stopping,
	sleeping,

	// These next 6 must be contiguous
	gridProbing1,
	gridProbing2,
	gridProbing3,
	gridProbing4,
	gridProbing5,
	gridprobing6,

	// These next 8 must be contiguous
	probingAtPoint0,
	probingAtPoint1,
	probingAtPoint2,
	probingAtPoint3,
	probingAtPoint4,
	probingAtPoint5,
	probingAtPoint6,
	probingAtPoint7,

	doingFirmwareRetraction,
	doingFirmwareUnRetraction,
	loadingFilament,
	unloadingFilament
};

// Class to hold the state of gcode execution for some input source
class GCodeMachineState
{
public:
	typedef uint32_t ResourceBitmap;
	GCodeMachineState();

	GCodeMachineState *previous;
	float feedrate;
	FileData fileState;
	ResourceBitmap lockedResources;
	GCodeState state;
	unsigned int
		drivesRelative : 1,
		axesRelative : 1,
		doingFileMacro : 1,
		waitWhileCooling : 1,
		runningM502 : 1,
		// Caution: these next 3 will be modified out-of-process when we use RTOS, so they will need to be individual bool variables
		waitingForAcknowledgement : 1,
		messageAcknowledged : 1,
		messageCancelled : 1;

	static GCodeMachineState *Allocate()
	post(!result.IsLive(); result.state == GCodeState::normal);

	// Copy values that may have been altered by config.g into this state record
	void CopyStateFrom(const GCodeMachineState& other)
	{
		drivesRelative = other.drivesRelative;
		axesRelative = other.axesRelative;
		feedrate = other.feedrate;
	}

	static void Release(GCodeMachineState *ms);
	static unsigned int GetNumAllocated() { return numAllocated; }
	static unsigned int GetNumInUse();

private:
	static GCodeMachineState *freeList;
	static unsigned int numAllocated;
};

#endif /* SRC_GCODES_GCODEMACHINESTATE_H_ */
