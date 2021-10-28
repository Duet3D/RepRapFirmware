/*
 * GCodeMachineState.cpp
 *
 *  Created on: 15 Nov 2016
 *      Author: David
 */

#include "GCodeMachineState.h"
#include <Platform/RepRap.h>

#include <limits>

// Create a default initialised GCodeMachineState
GCodeMachineState::GCodeMachineState() noexcept
	: feedRate(ConvertSpeedFromMmPerMin(DefaultFeedRate)),
#if HAS_SBC_INTERFACE
	  fileId(NoFileId),
#endif
	  lineNumber(0),
	  selectedPlane(0), drivesRelative(false), axesRelative(false),
	  doingFileMacro(false), waitWhileCooling(false), runningM501(false), runningM502(false),
	  volumetricExtrusion(false), g53Active(false), runningSystemMacro(false), usingInches(false),
	  waitingForAcknowledgement(false), messageAcknowledged(false), localPush(false), macroRestartable(false), firstCommandAfterRestart(false), commandRepeated(false),
#if HAS_SBC_INTERFACE
	  lastCodeFromSbc(false), macroStartedByCode(false), fileFinished(false),
#endif
	  compatibility(Compatibility::RepRapFirmware),
	  previous(nullptr), errorMessage(nullptr),
	  blockNesting(0), state(GCodeState::normal), stateMachineResult(GCodeResult::ok)
{
	blockStates[0].SetPlainBlock(0);
}

// Copy constructor. This chains the new one to the previous one.
GCodeMachineState::GCodeMachineState(GCodeMachineState& prev, bool withinSameFile) noexcept
	: feedRate(prev.feedRate),
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	  fileState(prev.fileState),
#endif
#if HAS_SBC_INTERFACE
	  fileId(prev.fileId),
#endif
	  lockedResources(prev.lockedResources),
	  lineNumber(0),
	  selectedPlane(prev.selectedPlane), drivesRelative(prev.drivesRelative), axesRelative(prev.axesRelative),
	  doingFileMacro(prev.doingFileMacro), waitWhileCooling(prev.waitWhileCooling), runningM501(prev.runningM501), runningM502(prev.runningM502),
	  volumetricExtrusion(false), g53Active(false), runningSystemMacro(prev.runningSystemMacro), usingInches(prev.usingInches),
	  waitingForAcknowledgement(false), messageAcknowledged(false), localPush(withinSameFile), firstCommandAfterRestart(prev.firstCommandAfterRestart), commandRepeated(false),
#if HAS_SBC_INTERFACE
	  lastCodeFromSbc(prev.lastCodeFromSbc), macroStartedByCode(prev.macroStartedByCode), fileFinished(prev.fileFinished),
#endif
	  compatibility(prev.compatibility),
	  previous(&prev), errorMessage(nullptr),
	  blockNesting((withinSameFile) ? prev.blockNesting : 0), state(GCodeState::normal), stateMachineResult(GCodeResult::ok)
{
	if (withinSameFile)
	{
		for (size_t i = 0; i <= blockNesting; ++i)
		{
			blockStates[i] = prev.blockStates[i];
		}
		macroRestartable = prev.macroRestartable;
	}
	else
	{
		blockStates[0].SetPlainBlock(0);
		macroRestartable = false;
	}
}

GCodeMachineState::~GCodeMachineState() noexcept
{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	fileState.Close();
#endif
}

// Return true if all nested macros we are running are restartable
bool GCodeMachineState::CanRestartMacro() const noexcept
{
	const GCodeMachineState *p = this;
	do
	{
		if (p->doingFileMacro && !p->macroRestartable)
		{
			return false;
		}
		p = p->previous;
	} while (p != nullptr);
	return true;
}

#if HAS_SBC_INTERFACE

// Set the state to indicate a file is being processed
void GCodeMachineState::SetFileExecuting() noexcept
{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	if (!fileState.IsLive())
#endif
	{
		fileId = (GetPrevious() != nullptr ? GetPrevious()->fileId : NoFileId) + 1;
		if (fileId == NoFileId)
		{
			// In case the ID overlapped increase it once more (should never happen)
			fileId++;
		}
		fileFinished = false;
	}
}

#endif

// Return true if we are reading GCode commands from a file or macro
bool GCodeMachineState::DoingFile() const noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface() && fileId != NoFileId)
	{
		return true;
	}
#endif
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	return fileState.IsLive();
#else
	return false;
#endif
}

// Close the currently executing file
void GCodeMachineState::CloseFile() noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		if (fileId != NoFileId)
		{
			const FileId lastFileId = fileId;
			for (GCodeMachineState *ms = this; ms != nullptr; ms = ms->GetPrevious())
			{
				if (ms->fileId == lastFileId)
				{
					ms->fileId = NoFileId;
					ms->fileFinished = false;
				}
			}
		}
	}
	else
#endif
	{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		fileState.Close();
#endif
	}
}

void GCodeMachineState::WaitForAcknowledgement() noexcept
{
	waitingForAcknowledgement = true;
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	if (fileState.IsLive())
	{
		// Stop reading from the current file
		CloseFile();
	}
#endif
}

void GCodeMachineState::CopyStateFrom(const GCodeMachineState& other) noexcept
{
	selectedPlane = other.selectedPlane;
	drivesRelative = other.drivesRelative;
	axesRelative = other.axesRelative;
	feedRate = other.feedRate;
	volumetricExtrusion = other.volumetricExtrusion;
	usingInches = other.usingInches;
}

// Set the error message and associated state
void GCodeMachineState::SetError(const char *msg) noexcept
{
	if (stateMachineResult != GCodeResult::error)
	{
		errorMessage = msg;
		stateMachineResult = GCodeResult::error;
	}
}

void GCodeMachineState::SetWarning(const char *msg) noexcept
{
	if (stateMachineResult == GCodeResult::ok)
	{
		errorMessage = msg;
		stateMachineResult = GCodeResult::warning;
	}
}

// Retrieve the result and error message if it is worse than the one we already have
void GCodeMachineState::RetrieveStateMachineResult(GCodeResult& rslt, const StringRef& reply) const noexcept
{
	if (stateMachineResult >= rslt)
	{
		rslt = stateMachineResult;
		if (errorMessage != nullptr)
		{
			reply.copy(errorMessage);
		}
	}
}

GCodeMachineState *GCodeMachineState::Pop() const noexcept
{
	GCodeMachineState * const rslt = GetPrevious();
	if (errorMessage != nullptr)
	{
		rslt->errorMessage = errorMessage;
		rslt->stateMachineResult = stateMachineResult;
	}
	return rslt;
}

void GCodeMachineState::SetState(GCodeState newState) noexcept
{
	if (state == GCodeState::normal && newState != GCodeState::normal)
	{
		stateMachineResult = GCodeResult::ok;
		errorMessage = nullptr;
	}
	state = newState;
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
		variables.EndScope(blockNesting);
	}
}

// End
