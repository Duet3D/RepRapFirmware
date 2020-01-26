/*
 * GCodeBuffer.h
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H
#define SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H

#include "BinaryParser.h"
#include "StringParser.h"

#include "RepRapFirmware.h"
#include "GCodes/GCodeChannel.h"
#include "GCodes/GCodeMachineState.h"
#include "GCodes/GCodeResult.h"
#include "Linux/MessageFormats.h"
#include "MessageType.h"
#include "ObjectModel/ObjectModel.h"

class FileGCodeInput;

// The processing state of each buffer
enum class GCodeBufferState : uint8_t
{
	parseNotStarted,								// we haven't started parsing yet
	parsingLineNumber,								// we saw N at the start and we are parsing the line number
	parsingWhitespace,								// parsing whitespace after the line number
	parsingGCode,									// parsing GCode words
	parsingBracketedComment,						// inside a (...) comment
	parsingQuotedString,							// inside a double-quoted string
	parsingChecksum,								// parsing the checksum after '*'
	discarding,										// discarding characters after the checksum or an end-of-line comment
	ready,											// we have a complete gcode but haven't started executing it
	executing										// we have a complete gcode and have started executing it
};

// Class to hold an individual GCode and provide functions to allow it to be parsed
class GCodeBuffer
{
public:
	friend class BinaryParser;
	friend class StringParser;

	GCodeBuffer(GCodeChannel channel, GCodeInput *normalIn, FileGCodeInput *fileIn, MessageType mt, Compatibility c = Compatibility::reprapFirmware) noexcept;
	void Reset() noexcept;														// Reset it to its state after start-up
	void Init() noexcept;														// Set it up to parse another G-code
	void Diagnostics(MessageType mtype) noexcept;								// Write some debug info

	bool IsBinary() const noexcept { return isBinaryBuffer; }					// Return true if the code is in binary format
	bool Put(char c) noexcept __attribute__((hot));								// Add a character to the end
	void PutAndDecode(const char *data, size_t len, bool isBinary) noexcept;	// Add an entire G-Code, overwriting any existing content
	void PutAndDecode(const char *str) noexcept;								// Add a null-terminated string, overwriting any existing content
	void StartNewFile() noexcept;												// Called when we start a new file
	bool FileEnded() noexcept;													// Called when we reach the end of the file we are reading from
	void DecodeCommand() noexcept;												// Decode the command in the buffer when it is complete
	bool CheckMetaCommand(const StringRef& reply) THROWS_GCODE_EXCEPTION;		// Check whether the current command is a meta command, or we are skipping a block

	char GetCommandLetter() const noexcept;
	bool HasCommandNumber() const noexcept;
	int GetCommandNumber() const noexcept;
	int8_t GetCommandFraction() const noexcept;
	int32_t GetLineNumber() const noexcept { return machineState->lineNumber; }
	GCodeResult GetLastResult() const noexcept { return lastResult; }
	void SetLastResult(GCodeResult r) noexcept { lastResult = r; }

	bool Seen(char c) noexcept __attribute__((hot));								// Is a character present?
	void MustSee(char c) THROWS_GCODE_EXCEPTION;									// Test for character present, throw error if not

	float GetFValue() THROWS_GCODE_EXCEPTION __attribute__((hot));					// Get a float after a key letter
	float GetDistance() THROWS_GCODE_EXCEPTION;										// Get a distance or coordinate and convert it from inches to mm if necessary
	int32_t GetIValue() THROWS_GCODE_EXCEPTION __attribute__((hot));				// Get an integer after a key letter
	uint32_t GetUIValue() THROWS_GCODE_EXCEPTION;									// Get an unsigned integer value
	void GetIPAddress(IPAddress& returnedIp) THROWS_GCODE_EXCEPTION;				// Get an IP address quad after a key letter
	void GetMacAddress(MacAddress& mac) THROWS_GCODE_EXCEPTION;						// Get a MAC address sextet after a key letter
	PwmFrequency GetPwmFrequency() THROWS_GCODE_EXCEPTION;							// Get a PWM frequency
	float GetPwmValue() THROWS_GCODE_EXCEPTION;										// Get a PWM value
	DriverId GetDriverId() THROWS_GCODE_EXCEPTION;									// Get a driver ID
	void GetUnprecedentedString(const StringRef& str, bool allowEmpty = false) THROWS_GCODE_EXCEPTION;	// Get a string with no preceding key letter
	void GetQuotedString(const StringRef& str) THROWS_GCODE_EXCEPTION;				// Get and copy a quoted string
	void GetPossiblyQuotedString(const StringRef& str) THROWS_GCODE_EXCEPTION;		// Get and copy a string which may or may not be quoted
	void GetReducedString(const StringRef& str) THROWS_GCODE_EXCEPTION;				// Get and copy a quoted string, removing certain characters
	void GetFloatArray(float arr[], size_t& length, bool doPad) THROWS_GCODE_EXCEPTION __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length, bool doPad) THROWS_GCODE_EXCEPTION;		// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) THROWS_GCODE_EXCEPTION;	// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length) THROWS_GCODE_EXCEPTION;	// Get a :-separated list of drivers after a key letter

	bool TryGetFValue(char c, float& val, bool& seen) THROWS_GCODE_EXCEPTION;
	bool TryGetIValue(char c, int32_t& val, bool& seen) THROWS_GCODE_EXCEPTION;
	bool TryGetUIValue(char c, uint32_t& val, bool& seen) THROWS_GCODE_EXCEPTION;
	bool TryGetBValue(char c, bool& val, bool& seen) THROWS_GCODE_EXCEPTION;
	bool TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad = false) THROWS_GCODE_EXCEPTION;
	bool TryGetUIArray(char c, size_t numVals, uint32_t vals[], const StringRef& reply, bool& seen, bool doPad = false) THROWS_GCODE_EXCEPTION;
	bool TryGetQuotedString(char c, const StringRef& str, bool& seen) THROWS_GCODE_EXCEPTION;
	bool TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen) THROWS_GCODE_EXCEPTION;

	bool IsIdle() const noexcept;
	bool IsCompletelyIdle() const noexcept;
	bool IsReady() const noexcept;								// Return true if a gcode is ready but hasn't been started yet
	bool IsExecuting() const noexcept;							// Return true if a gcode has been started and is not paused
	void SetFinished(bool f) noexcept;							// Set the G Code executed (or not)
	void SetCommsProperties(uint32_t arg) noexcept;

	GCodeMachineState& MachineState() const noexcept { return *machineState; }
	GCodeMachineState& OriginalMachineState() const noexcept;
	float ConvertDistance(float distance) const noexcept;
	float InverseConvertDistance(float distance) const noexcept;
	bool PushState(bool withinSameFile) noexcept;				// Push state returning true if successful (i.e. stack not overflowed)
	bool PopState(bool withinSameFile) noexcept;				// Pop state returning true if successful (i.e. no stack underrun)

	void AbortFile(bool abortAll, bool requestAbort = true) noexcept;
	bool IsDoingFile() const noexcept;							// Return true if this source is executing a file
	bool IsDoingFileMacro() const noexcept;						// Return true if this source is executing a file macro
	FilePosition GetFilePosition() const noexcept;				// Get the file position at the start of the current command

#if HAS_LINUX_INTERFACE
	void SetPrintFinished() noexcept;							// Mark the print file as finished
	bool IsFileFinished() const noexcept;						// Return true if this source has finished execution of a file

	bool IsMacroRequested() const noexcept { return !requestedMacroFile.IsEmpty(); }					// Indicates if a macro file is being requested
	void RequestMacroFile(const char *filename, bool reportMissing, bool fromCode) noexcept;	// Request execution of a file macro
	const char *GetRequestedMacroFile(bool& reportMissing, bool &fromCode) const noexcept;		// Return requested macro file or nullptr if none

	bool IsAbortRequested() const noexcept;						// Is the cancellation of the current file requested?
	bool IsAbortAllRequested() const noexcept;					// Is the cancellation of all files being executed on this channel requested?
	void AcknowledgeAbort() noexcept;							// Indicates that the current macro file is being cancelled

	void ReportStack() noexcept { reportStack = true; }			// Flags current stack details to be reported
	bool IsStackEventFlagged() const noexcept;					// Did the stack change?
	void AcknowledgeStackEvent() noexcept;						// Indicates that the last stack event has been written

	bool IsInvalidated() const noexcept { return invalidated; }	// Indicates if the channel is invalidated
	void Invalidate(bool i = true) noexcept { invalidated = i; }	// Invalidate this channel (or not)
#endif

	GCodeState GetState() const noexcept;
	void SetState(GCodeState newState) noexcept;
	void SetState(GCodeState newState, const char *err) noexcept;
	void AdvanceState() noexcept;
	void MessageAcknowledged(bool cancelled) noexcept;

	GCodeChannel GetChannel() const noexcept { return codeChannel; }
	const char *GetIdentity() const noexcept { return gcodeChannelName[(size_t)codeChannel]; }
	bool CanQueueCodes() const noexcept;
	MessageType GetResponseMessageType() const noexcept;

	int GetToolNumberAdjust() const noexcept { return toolNumberAdjust; }
	void SetToolNumberAdjust(int arg) noexcept { toolNumberAdjust = arg; }

#if HAS_MASS_STORAGE
	bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) noexcept;
																// open a file to write to
	bool IsWritingFile() const noexcept;						// Returns true if writing a file
	void WriteToFile() noexcept;								// Write the current GCode to file

	bool IsWritingBinary() const noexcept;						// Returns true if writing binary
	void WriteBinaryToFile(char b) noexcept;					// Write a byte to the file
	void FinishWritingBinary() noexcept;
#endif

	const char* DataStart() const noexcept;						// Get the start of the current command
	size_t DataLength() const noexcept;							// Get the length of the current command

	void PrintCommand(const StringRef& s) const noexcept;
	void AppendFullCommand(const StringRef &s) const noexcept;

	bool IsTimerRunning() const noexcept { return timerRunning; }
	uint32_t WhenTimerStarted() const noexcept { return whenTimerStarted; }
	void StartTimer() noexcept;
	void StopTimer() noexcept { timerRunning = false; }
	bool DoDwellTime(uint32_t dwellMillis) noexcept;			// Execute a dwell returning true if it has finished

	void RestartFrom(FilePosition pos) noexcept;

#if HAS_MASS_STORAGE
	FileGCodeInput *GetFileInput() const noexcept { return fileInput; }	//TEMPORARY!
#endif
	GCodeInput *GetNormalInput() const noexcept { return normalInput; }	//TEMPORARY!

private:
	const GCodeChannel codeChannel;						// Channel number of this instance
	GCodeInput *normalInput;							// Our normal input stream, or nullptr if there isn't one

#if HAS_MASS_STORAGE
	FileGCodeInput *fileInput;							// Our file input stream for when we are reading from a print file or a macro file, may be shared with other GCodeBuffers
#endif

	const MessageType responseMessageType;				// The message type we use for responses to string codes coming from this channel

	int toolNumberAdjust;								// The adjustment to tool numbers in commands we receive

#if HAS_LINUX_INTERFACE
	char buffer[MaxCodeBufferSize];
#else
	char buffer[GCODE_LENGTH];
#endif

	GCodeResult lastResult;
	bool isBinaryBuffer;
	BinaryParser binaryParser;
	StringParser stringParser;

	GCodeBufferState bufferState;						// Idle, executing or paused
	GCodeMachineState *machineState;					// Machine state for this gcode source

	uint32_t whenTimerStarted;							// When we started waiting
	bool timerRunning;									// True if we are waiting

#if HAS_LINUX_INTERFACE
	String<MaxFilenameLength> requestedMacroFile;
	uint8_t
		reportMissingMacro : 1,
		isMacroFromCode: 1,
		abortFile : 1,
		abortAllFiles : 1,
		invalidated : 1,
		reportStack : 1;
#endif
};

inline bool GCodeBuffer::IsDoingFileMacro() const noexcept
{
	return machineState->doingFileMacro;
}

inline GCodeState GCodeBuffer::GetState() const noexcept
{
	return machineState->state;
}

inline void GCodeBuffer::SetState(GCodeState newState) noexcept
{
	machineState->state = newState;
}

inline void GCodeBuffer::SetState(GCodeState newState, const char *err) noexcept
{
	machineState->state = newState;
	machineState->errorMessage = err;
}

inline void GCodeBuffer::AdvanceState() noexcept
{
	machineState->state = static_cast<GCodeState>(static_cast<uint8_t>(machineState->state) + 1);
}

// Return true if we can queue gcodes from this source. This is the case if a file is being executed
inline bool GCodeBuffer::CanQueueCodes() const noexcept
{
	return IsDoingFile();
}

inline bool GCodeBuffer::IsDoingFile() const noexcept
{
	return machineState->DoingFile();
}

#if HAS_LINUX_INTERFACE

inline bool GCodeBuffer::IsAbortRequested() const noexcept
{
	return abortFile;
}

inline bool GCodeBuffer::IsAbortAllRequested() const noexcept
{
	return abortAllFiles;
}

inline void GCodeBuffer::AcknowledgeAbort() noexcept
{
	abortFile = abortAllFiles = false;
}

inline bool GCodeBuffer::IsStackEventFlagged() const noexcept
{
	return reportStack;
}

inline void GCodeBuffer::AcknowledgeStackEvent() noexcept
{
	reportStack = false;
}

#endif

#endif /* SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H */
