/*
 * GCodeMachineState.cpp
 *
 *  Created on: 15 Nov 2016
 *      Author: David
 */

#include <limits>

#include "GCodeMachineState.h"

GCodeMachineState *GCodeMachineState::freeList = nullptr;
unsigned int GCodeMachineState::numAllocated = 0;

#if HAS_LINUX_INTERFACE
static unsigned int LastFileId = 1;
#endif

// Create a default initialised GCodeMachineState
GCodeMachineState::GCodeMachineState()
	: previous(nullptr), feedRate(DefaultFeedRate * SecondsToMinutes), lockedResources(0), errorMessage(nullptr), lineNumber(0),
	  drivesRelative(false), axesRelative(false), doingFileMacro(false), runningM501(false), runningM502(false),
	  volumetricExtrusion(false), g53Active(false), runningSystemMacro(false), usingInches(false),
	  waitingForAcknowledgement(false), messageAcknowledged(false),
	  indentLevel(0), state(GCodeState::normal)
{
#if HAS_LINUX_INTERFACE
	fileId = 0;
	isFileFinished = false;
#endif
}

#if HAS_LINUX_INTERFACE
// Set the state to indicate a file is being processed
void GCodeMachineState::SetFileExecuting()
{
	fileId = LastFileId++;
	isFileFinished = false;
}

// Mark the currently executing file as finished
void GCodeMachineState::SetFileFinished()
{
	isFileFinished = true;
}
#endif

// Close the currently executing file
void GCodeMachineState::CloseFile()
{
#if HAS_HIGH_SPEED_SD
	fileState.Close();
#elif HAS_LINUX_INTERFACE
	for (GCodeMachineState *ms = this; ms != nullptr; ms = ms->previous)
	{
		if (ms->fileId == fileId)
		{
			ms->isFileFinished = false;
			ms->fileId = 0;
		}
	}
#endif
}

// Allocate a new GCodeMachineState
/*static*/ GCodeMachineState *GCodeMachineState::Allocate()
{
	GCodeMachineState *ms = freeList;
	if (ms != nullptr)
	{
		freeList = ms->previous;
		ms->lockedResources = 0;
		ms->errorMessage = nullptr;
		ms->state = GCodeState::normal;
	}
	else
	{
		ms = new GCodeMachineState();
		++numAllocated;
	}
	return ms;
}

/*static*/ void GCodeMachineState::Release(GCodeMachineState *ms)
{
#if HAS_HIGH_SPEED_SD
	ms->fileState.Close();
#elif HAS_LINUX_INTERFACE
	ms->fileId = 0;
	ms->isFileFinished = false;
#endif
	ms->previous = freeList;
	freeList = ms;
}

/*static*/ unsigned int GCodeMachineState::GetNumInUse()
{
	unsigned int inUse = numAllocated;
	for (GCodeMachineState *ms = freeList; ms != nullptr; ms = ms->previous)
	{
		--inUse;
	}
	return inUse;
}

GCodeMachineState::BlockState& GCodeMachineState::CurrentBlockState()
{
	return blockStates[min<size_t>(indentLevel, ARRAY_SIZE(blockStates) - 1)];
}

void GCodeMachineState::CreateBlock()
{
	++indentLevel;
	CurrentBlockState().SetPlainBlock();
}

void GCodeMachineState::EndBlock()
{
	if (indentLevel != 0)
	{
		--indentLevel;
	}
}

// End
