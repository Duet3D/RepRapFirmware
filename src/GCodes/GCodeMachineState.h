/*
 * GCodeMachineState.h
 *
 *  Created on: 15 Nov 2016
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEMACHINESTATE_H_
#define SRC_GCODES_GCODEMACHINESTATE_H_

#include <RepRapFirmware.h>
#include <Storage/FileData.h>
#include <General/FreelistManager.h>
#include <General/NamedEnum.h>
#include <ObjectModel/Variable.h>

// Enumeration to list all the possible states that the Gcode processing machine may be in
enum class GCodeState : uint8_t
{
	normal,												// not doing anything and ready to process a new GCode

	abortWhenMovementFinished,							// abort the print when all current moves have completed

	waitingForSpecialMoveToComplete,					// doing a special move, so we must wait for it to finish before processing another GCode
	waitingForSegmentedMoveToGo,						// doing an arc move, so we must check whether it completes normally

	// This group must be contiguous
	probingToolOffset1,
	probingToolOffset2,
	probingToolOffset3,
	probingToolOffset4,

	// These next 7 must be contiguous
	findCenterOfCavity1,
	findCenterOfCavity2,
	findCenterOfCavity3,
	findCenterOfCavity4,
	findCenterOfCavity5,
	findCenterOfCavity6,

	homing1,
	homing2,

	// These next 5 must be contiguous
	toolChange0,
	toolChange1,
	toolChange2,
	toolChange3,
	toolChangeComplete,

	// These next 6 must be contiguous
	m109ToolChange0,
	m109ToolChange1,
	m109ToolChange2,
	m109ToolChange3,
	m109ToolChangeComplete,
	m109WaitForTemperature,

	pausing1,
	pausing2,

	filamentChangePause1,
	filamentChangePause2,

	filamentErrorPause1,
	filamentErrorPause2,

	resuming1,
	resuming2,
	resuming3,

	cancelling,

	flashing1,
	flashing2,

	stopping,

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

	// These next 4 must be contiguous
	straightProbe0,
	straightProbe1,
	straightProbe2,
	straightProbe3,

	doingFirmwareRetraction,
	doingFirmwareUnRetraction,
	loadingFilament,
	unloadingFilament,

	checkError,						// go to this state after doing a macro when we need to check for a stored error message

#if HAS_MASS_STORAGE
	timingSDwrite,
	timingSDread,
#endif

#if HAS_SBC_INTERFACE
	waitingForAcknowledgement,
#endif

#if HAS_VOLTAGE_MONITOR
	powerFailPausing1,
#endif
};

// Other firmware that we might switch to be compatible with. The ordering is as for M555 starting with 0.
NamedEnum(Compatibility, uint8_t, Default, RepRapFirmware, Marlin, Teacup, Sprinter, Repetier, NanoDLP);

// Type of the block we are in when processing conditional GCode
enum class BlockType : uint8_t
{
	plain,						// a normal block
	ifTrue,						// the block immediately after 'if' when the condition was true, or after 'elif' when the condition was true and previous conditions were false
	ifFalseNoneTrue,			// the block immediately after 'if' when the condition was false, or after 'elif' when all conditions so far were false
	ifFalseHadTrue,				// the block immediately after 'elif' when we have already seem a true condition
	loop						// block inside a 'while' command
};

#if HAS_SBC_INTERFACE
typedef uint8_t FileId;

constexpr FileId NoFileId = 0;
#endif

// Class to hold the state of gcode execution for some input source
class GCodeMachineState
{
public:
	typedef Bitmap<uint32_t> ResourceBitmap;

	// Class to record the state of blocks when using conditional GCode
	class BlockState
	{
	public:
		BlockType GetType() const noexcept { return blockType; }
		uint32_t GetIterations() const noexcept { return iterationsDone; }
		uint32_t GetLineNumber() const noexcept { return lineNumber; }
		FilePosition GetFilePosition() const noexcept { return fpos; }
		uint16_t GetIndent() const noexcept { return indentLevel; }

		void SetLoopBlock(FilePosition filePos, uint32_t lineNum) noexcept { blockType = BlockType::loop; fpos = filePos; lineNumber = lineNum; iterationsDone = 0; }
		void SetPlainBlock() noexcept { blockType = BlockType::plain; iterationsDone = 0; }
		void SetPlainBlock(uint16_t p_indentLevel) noexcept { blockType = BlockType::plain; iterationsDone = 0; indentLevel = p_indentLevel; }
		void SetIfTrueBlock() noexcept { blockType = BlockType::ifTrue; iterationsDone = 0; }
		void SetIfFalseNoneTrueBlock() noexcept { blockType = BlockType::ifFalseNoneTrue; iterationsDone = 0; }
		void SetIfFalseHadTrueBlock() noexcept { blockType = BlockType::ifFalseHadTrue; iterationsDone = 0; }

		void IncrementIterations() noexcept { ++iterationsDone; }

	private:
		FilePosition fpos;											// the file offset at which the current block started
		uint32_t lineNumber;										// the line number at which the current block started
		uint32_t iterationsDone;
		uint16_t indentLevel;										// the indentation of this block
		BlockType blockType;										// the type of this block
	};

	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<GCodeMachineState>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<GCodeMachineState>(p); }

	GCodeMachineState() noexcept;
	GCodeMachineState(GCodeMachineState&, bool withinSameFile) noexcept;	// this chains the new one to the previous one
	GCodeMachineState(const GCodeMachineState&) = delete;			// copying these would be a bad idea

	~GCodeMachineState() noexcept;

	GCodeState GetState() const noexcept { return state; }
	void SetState(GCodeState newState) noexcept;
	inline void AdvanceState() noexcept { state = static_cast<GCodeState>(static_cast<uint8_t>(state) + 1); }

	GCodeMachineState *GetPrevious() const noexcept { return previous; }
	GCodeMachineState *Pop() const noexcept;
	uint8_t GetBlockNesting() const noexcept { return blockNesting; }

	void SetMacroRestartable(bool b) noexcept { macroRestartable = b; }
	bool CanRestartMacro() const noexcept;

	VariableSet variables;											// local variables and parameters
	float feedRate;
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	FileData fileState;
#endif
#if HAS_SBC_INTERFACE
	FileId fileId;													// virtual ID to distinguish files in different stack levels (only unique per GB)
#endif
	ResourceBitmap lockedResources;
	BlockState blockStates[MaxBlockIndent];
	uint32_t lineNumber;

	uint32_t
		selectedPlane : 2,
		drivesRelative : 1,
		axesRelative : 1,
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
		messageCancelled : 1,
		localPush : 1,							// true if this stack frame was created by M120, so we use the parent variables
		macroRestartable : 1,					// true if the current macro has used M98 R1 to say that it can be interrupted and restarted
		firstCommandAfterRestart : 1,			// true if this is the first command after restarting a macro that was interrupted
		commandRepeated : 1						// true if the current command is being repeated because it returned GCodeResult::notFinished the first time
#if HAS_SBC_INTERFACE
		, lastCodeFromSbc : 1,
		macroStartedByCode : 1,
		fileFinished : 1
#endif
		;

	Compatibility compatibility;
	uint16_t stateParameter;					// a parameter, the meaning of which depends on what state we are in

	bool DoingFile() const noexcept;
	void CloseFile() noexcept;

	void WaitForAcknowledgement() noexcept;

#if HAS_SBC_INTERFACE
	void SetFileExecuting() noexcept;
#endif

	bool UsingMachineCoordinates() const noexcept { return g53Active || runningSystemMacro; }

	// Set the error message and associated state
	void SetError(const char *msg) noexcept;
	void SetWarning(const char *msg) noexcept;
	void RetrieveStateMachineResult(GCodeResult& rslt, const StringRef& reply) const noexcept;

	// Copy values that may have been altered into this state record
	// Called after running config.g and after running resurrect.g
	void CopyStateFrom(const GCodeMachineState& other) noexcept;

	BlockState& CurrentBlockState() noexcept;
	const BlockState& CurrentBlockState() const noexcept;
	int32_t GetIterations() const noexcept;

	bool CreateBlock(uint16_t indentLevel) noexcept;
	void EndBlock() noexcept;

private:
	GCodeMachineState *previous;
	const char *errorMessage;
	uint8_t blockNesting;
	GCodeState state;
	GCodeResult stateMachineResult;				// the worst status (ok, warning or error) that we encountered while running the state machine
};

#endif /* SRC_GCODES_GCODEMACHINESTATE_H_ */
