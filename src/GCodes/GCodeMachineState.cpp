/*
 * GCodeMachineState.cpp
 *
 *  Created on: 15 Nov 2016
 *      Author: David
 */

#include "GCodeMachineState.h"
#include <Platform/RepRap.h>
#include <Storage/MassStorage.h>

#include <climits>

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
	  waitingForAcknowledgement(false), messageAcknowledged(false), localPush(false), macroRestartable(false), firstCommandAfterRestart(false), commandRepeated(false), inverseTimeMode(false),
#if HAS_SBC_INTERFACE
	  lastCodeFromSbc(false), macroStartedByCode(false), fileFinished(false),
#endif
	  stateParameter(0),
	  compatibility(Compatibility::RepRapFirmware),
	  previous(nullptr), currentBlockState(new BlockState(nullptr)), errorMessage(nullptr),
	  blockNesting(0), state(GCodeState::normal), stateMachineResult(GCodeResult::ok)
#if SUPPORT_ASYNC_MOVES
	  , commandedQueueNumber(0), ownQueueNumber(0), executeAllCommands(true)
#endif
{
	currentBlockState->SetPlainBlock(0);
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
	  lineNumber((withinSameFile) ? prev.lineNumber : 0),
	  selectedPlane(prev.selectedPlane), drivesRelative(prev.drivesRelative), axesRelative(prev.axesRelative),
	  doingFileMacro(prev.doingFileMacro), waitWhileCooling(prev.waitWhileCooling), runningM501(prev.runningM501), runningM502(prev.runningM502),
	  volumetricExtrusion(false), g53Active(false), runningSystemMacro(prev.runningSystemMacro), usingInches(prev.usingInches),
	  waitingForAcknowledgement(false), messageAcknowledged(false), localPush(withinSameFile), firstCommandAfterRestart(prev.firstCommandAfterRestart), commandRepeated(false),
	  inverseTimeMode(withinSameFile && prev.inverseTimeMode),
#if HAS_SBC_INTERFACE
	  lastCodeFromSbc(prev.lastCodeFromSbc), macroStartedByCode(prev.macroStartedByCode), fileFinished(prev.fileFinished),
#endif
	  compatibility(prev.compatibility),
	  previous(&prev), currentBlockState(new BlockState(nullptr)), errorMessage(nullptr),
	  blockNesting((withinSameFile) ? prev.blockNesting : 0),
	  state(GCodeState::normal), stateMachineResult(GCodeResult::ok)
#if SUPPORT_ASYNC_MOVES
	  , commandedQueueNumber(prev.commandedQueueNumber), ownQueueNumber(prev.ownQueueNumber), executeAllCommands(prev.executeAllCommands)
#endif
{
	if (withinSameFile)
	{
		// Copy the block states from the previous MachineState to the new one
		const BlockState *src = prev.currentBlockState;
		BlockState *dst = currentBlockState;
		for (;;)
		{
			*dst = *src;
			src = src->GetPrevious();
			if (src == nullptr)
			{
				break;
			}
			BlockState *const newDst = new BlockState(nullptr);
			dst->SetPrevious(newDst);
			dst = newDst;
		}
		macroRestartable = prev.macroRestartable;
	}
	else
	{
		currentBlockState->SetPlainBlock(0);
		macroRestartable = false;
	}
}

#if SUPPORT_ASYNC_MOVES

GCodeMachineState::GCodeMachineState(GCodeMachineState& copyFrom, GCodeMachineState *prev, unsigned int oldExecuteQueue, unsigned int newExecuteQueue) noexcept
	: feedRate(copyFrom.feedRate),
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	  fileState(),
#endif
#if HAS_SBC_INTERFACE
	  fileId(copyFrom.fileId),
#endif
	  lockedResources(copyFrom.lockedResources),
	  lineNumber(copyFrom.lineNumber),
	  selectedPlane(copyFrom.selectedPlane), drivesRelative(copyFrom.drivesRelative), axesRelative(copyFrom.axesRelative),
	  doingFileMacro(copyFrom.doingFileMacro), waitWhileCooling(copyFrom.waitWhileCooling), runningM501(copyFrom.runningM501), runningM502(copyFrom.runningM502),
	  volumetricExtrusion(false), g53Active(false), runningSystemMacro(copyFrom.runningSystemMacro), usingInches(copyFrom.usingInches),
	  waitingForAcknowledgement(false), messageAcknowledged(false), localPush(copyFrom.localPush),
	  macroRestartable(copyFrom.macroRestartable), firstCommandAfterRestart(copyFrom.firstCommandAfterRestart), commandRepeated(copyFrom.commandRepeated),
	  inverseTimeMode(copyFrom.inverseTimeMode),
#if HAS_SBC_INTERFACE
	  lastCodeFromSbc(copyFrom.lastCodeFromSbc), macroStartedByCode(copyFrom.macroStartedByCode), fileFinished(copyFrom.fileFinished),
#endif
	  compatibility(copyFrom.compatibility),
	  previous(prev), currentBlockState(new BlockState(nullptr)), errorMessage(nullptr),
	  blockNesting(copyFrom.blockNesting),
	  state(GCodeState::normal), stateMachineResult(GCodeResult::ok),
	  commandedQueueNumber(copyFrom.commandedQueueNumber), ownQueueNumber(newExecuteQueue), executeAllCommands(false)
{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	if (copyFrom.fileState.IsLive())
	{
		if (localPush)
		{
			fileState.CopyFrom(prev->fileState);
		}
		else
		{
			fileState.Set(MassStorage::DuplicateOpenHandle(copyFrom.fileState.GetUnderlyingFile()));
		}
	}
#endif
	copyFrom.ownQueueNumber = oldExecuteQueue;
	copyFrom.executeAllCommands = false;

	// Copy the block states from the previous MachineState to the new one
	const BlockState *src = copyFrom.currentBlockState;
	BlockState *dst = currentBlockState;
	for (;;)
	{
		*dst = *src;
		src = src->GetPrevious();
		if (src == nullptr)
		{
			break;
		}
		BlockState *const newDst = new BlockState(nullptr);
		dst->SetPrevious(newDst);
		dst = newDst;
	}
}

#endif

// Destructor. This deletes any associated BlockState objects but not any linked MachineState objects.
GCodeMachineState::~GCodeMachineState() noexcept
{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	fileState.Close();
#endif
	while (currentBlockState != nullptr)
	{
		BlockState *const tempBs = currentBlockState;
		currentBlockState = currentBlockState->GetPrevious();
		delete tempBs;
	}
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

void GCodeMachineState::WaitForAcknowledgement(uint32_t seq) noexcept
{
	msgBoxSeq = seq;
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	if (fileState.IsLive())
	{
		// Stop reading from the current file
		CloseFile();
	}
#endif
	waitingForAcknowledgement = true;
}

// This is called only after running config.g and when using M26/M23 to resume a print
void GCodeMachineState::CopyStateFrom(const GCodeMachineState& other) noexcept
{
	selectedPlane = other.selectedPlane;
	drivesRelative = other.drivesRelative;
	axesRelative = other.axesRelative;
	feedRate = other.feedRate;
	volumetricExtrusion = other.volumetricExtrusion;
	usingInches = other.usingInches;
	inverseTimeMode = other.inverseTimeMode;
}

// Set the error message and associated state
void GCodeMachineState::SetError(const char *msg) noexcept
{
	if (stateMachineResult != GCodeResult::error)
	{
		errorMessage = GCodeException(msg);
		stateMachineResult = GCodeResult::error;
	}
}

void GCodeMachineState::SetError(const GCodeException& exc) noexcept
{
	if (stateMachineResult != GCodeResult::error)
	{
		errorMessage = exc;
		stateMachineResult = GCodeResult::error;
	}
}

void GCodeMachineState::SetWarning(const char *msg) noexcept
{
	if (stateMachineResult == GCodeResult::ok)
	{
		errorMessage = GCodeException(msg);
		stateMachineResult = GCodeResult::warning;
	}
}

// Retrieve the result and error message if it is worse than the one we already have
void GCodeMachineState::RetrieveStateMachineResult(const GCodeBuffer& gb, const StringRef& reply, GCodeResult& rslt) const noexcept
{
	if (stateMachineResult >= rslt)
	{
		rslt = stateMachineResult;
		if (!errorMessage.IsNull())
		{
			errorMessage.GetMessage(reply, &gb);
		}
	}
}

GCodeMachineState *GCodeMachineState::Pop() const noexcept
{
	GCodeMachineState * const rslt = GetPrevious();
	if (!errorMessage.IsNull())
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
		errorMessage = GCodeException();
	}
	state = newState;
}

// Get the number of iterations of the closest enclosing loop in the current file, or -1 if there is on enclosing loop
int32_t GCodeMachineState::GetIterations() const noexcept
{
	for (const BlockState *bs = currentBlockState; bs != nullptr; bs = bs->GetPrevious())
	{
		if (bs->GetType() == BlockType::loop)
		{
			return bs->GetIterations();
		}
	}
	return -1;
}

// Create a new block returning true if successful, false if maximum indent level exceeded
void GCodeMachineState::CreateBlock(uint16_t indentLevel) noexcept
{
	currentBlockState = new BlockState(currentBlockState);
	++blockNesting;
	currentBlockState->SetPlainBlock(indentLevel);
}

void GCodeMachineState::EndBlock() noexcept
{
	if (blockNesting != 0)
	{
		BlockState *const oldBs = currentBlockState;
		currentBlockState = currentBlockState->GetPrevious();
		delete oldBs;
		--blockNesting;
		variables.EndScope(blockNesting);
	}
}

void GCodeMachineState::ClearBlocks() noexcept
{
	while (blockNesting != 0)
	{
		EndBlock();
	}
	currentBlockState->SetPlainBlock();
	variables.Clear();
}

#if SUPPORT_ASYNC_MOVES

// Clone a whole chain of these objects, ensuring that the file positions are correct
// This is recursive but we don't expect the recursion to be very deep.
GCodeMachineState *GCodeMachineState::ForkChain() noexcept
{
	GCodeMachineState *newPrev = (previous == nullptr) ? nullptr : previous->ForkChain();
	return new GCodeMachineState(*this, newPrev, 0, 1);
}

#endif

// End
