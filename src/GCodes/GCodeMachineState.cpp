/*
 * GCodeMachineState.cpp
 *
 *  Created on: 15 Nov 2016
 *      Author: David
 */

#include "GCodeMachineState.h"
#include "RepRap.h"

#include <limits>

#if HAS_LINUX_INTERFACE
static unsigned int LastFileId = 1;
#endif

// Create a default initialised GCodeMachineState
GCodeMachineState::GCodeMachineState() noexcept
	: previous(nullptr), feedRate(DefaultFeedRate * SecondsToMinutes),
#if HAS_LINUX_INTERFACE
	  fileId(0),
#endif
	  errorMessage(nullptr),
	  lineNumber(0),
	  compatibility(Compatibility::reprapFirmware), drivesRelative(false), axesRelative(false),
#if HAS_LINUX_INTERFACE
	  isFileFinished(false), fileError(false),
#endif
	  doingFileMacro(false), waitWhileCooling(false), runningM501(false), runningM502(false),
	  volumetricExtrusion(false), g53Active(false), runningSystemMacro(false), usingInches(false),
	  waitingForAcknowledgement(false), messageAcknowledged(false), blockNesting(0), state(GCodeState::normal)
{
	blockStates[0].SetPlainBlock(0);
}

// Copy constructor. This chains the new one to the previous one.
GCodeMachineState::GCodeMachineState(GCodeMachineState& prev, bool withinSameFile) noexcept
	: previous(&prev), feedRate(prev.feedRate),
#if HAS_MASS_STORAGE
	  fileState(prev.fileState),
#endif
#if HAS_LINUX_INTERFACE
	  fileId(prev.fileId),
#endif
	  lockedResources(prev.lockedResources), errorMessage(nullptr),
	  lineNumber((withinSameFile) ? prev.lineNumber : 0),
	  compatibility(prev.compatibility), drivesRelative(prev.drivesRelative), axesRelative(prev.axesRelative),
#if HAS_LINUX_INTERFACE
	  isFileFinished(prev.isFileFinished), fileError(false),
#endif
	  doingFileMacro(prev.doingFileMacro), waitWhileCooling(prev.waitWhileCooling), runningM501(prev.runningM501),  runningM502(prev.runningM502),
	  volumetricExtrusion(false), g53Active(false), runningSystemMacro(prev.runningSystemMacro), usingInches(prev.usingInches),
	  waitingForAcknowledgement(false), messageAcknowledged(false), blockNesting((withinSameFile) ? prev.blockNesting : 0), state(GCodeState::normal)
{
	if (withinSameFile)
	{
		for (size_t i = 0; i <= blockNesting; ++i)
		{
			blockStates[i] = prev.blockStates[i];
		}
	}
	else
	{
		blockStates[0].SetPlainBlock(0);
	}
}

GCodeMachineState::~GCodeMachineState() noexcept
{
#if HAS_MASS_STORAGE
# if HAS_LINUX_INTERFACE
	if (!reprap.UsingLinuxInterface())
# endif
	{
		fileState.Close();
	}
#endif
}

#if HAS_LINUX_INTERFACE

// Set the state to indicate a file is being processed
void GCodeMachineState::SetFileExecuting() noexcept
{
	fileId = LastFileId++;
	isFileFinished = fileError = false;
}

// Mark the currently executing file as finished
void GCodeMachineState::SetFileFinished(bool error) noexcept
{
	isFileFinished = true;
	fileError = error;
}

#endif

bool GCodeMachineState::DoingFile() const noexcept
{
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		return fileId != 0;
	}
#endif
#if HAS_MASS_STORAGE
	return fileState.IsLive();
#else
	return false;
#endif
}

// Close the currently executing file
void GCodeMachineState::CloseFile() noexcept
{
#if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		for (GCodeMachineState *ms = this; ms != nullptr; ms = ms->previous)
		{
			if (ms->fileId == fileId)
			{
				ms->isFileFinished = false;
				ms->fileId = 0;
			}
		}
	}
	else
#endif
	{
#if HAS_MASS_STORAGE
		fileState.Close();
#endif
	}
}

void GCodeMachineState::CopyStateFrom(const GCodeMachineState& other) noexcept
{
	drivesRelative = other.drivesRelative;
	axesRelative = other.axesRelative;
	feedRate = other.feedRate;
	volumetricExtrusion = other.volumetricExtrusion;
	usingInches = other.usingInches;
}

GCodeMachineState::BlockState& GCodeMachineState::CurrentBlockState() noexcept
{
	return blockStates[blockNesting];
}

const GCodeMachineState::BlockState& GCodeMachineState::CurrentBlockState() const noexcept
{
	return blockStates[blockNesting];
}

// Get the number of iterations of the closest enclosing loop in the current file, or -1 if there is on enclosing loop
int32_t GCodeMachineState::GetIterations() const noexcept
{
	uint8_t i = blockNesting;
	while (true)
	{
		if (blockStates[i].GetType() == BlockType::loop)
		{
			return blockStates[i].GetIterations();
		}
		if (i == 0)
		{
			return -1;
		}
		--i;
	}
}

// Create a new block returning true if successful, false if maximum indent level exceeded
bool GCodeMachineState::CreateBlock(uint16_t indentLevel) noexcept
{
	if (blockNesting + 1 == ARRAY_SIZE(blockStates))
	{
		return false;
	}

	++blockNesting;
	CurrentBlockState().SetPlainBlock(indentLevel);
	return true;
}

void GCodeMachineState::EndBlock() noexcept
{
	if (blockNesting != 0)
	{
		--blockNesting;
	}
}

// End
