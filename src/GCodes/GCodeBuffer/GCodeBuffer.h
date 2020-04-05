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
#include "Linux/LinuxMessageFormats.h"
#include "MessageType.h"
#include "ObjectModel/ObjectModel.h"

class FileGCodeInput;

// The processing state of each buffer
enum class GCodeBufferState : uint8_t
{
	parseNotStarted,								// we haven't started parsing yet
	parsingLineNumber,								// we saw N at the start and we are parsing the line number
	parsingWhitespace,								// parsing whitespace after the line number
	parsingComment,									// parsing a whole-line comment that we may be interested in
	parsingGCode,									// parsing GCode words
	parsingBracketedComment,						// inside a (...) comment
	parsingQuotedString,							// inside a double-quoted string
	parsingChecksum,								// parsing the checksum after '*'
	discarding,										// discarding characters after the checksum or an end-of-line comment
	ready,											// we have a complete gcode but haven't started executing it
	executing										// we have a complete gcode and have started executing it
};

// Class to hold an individual GCode and provide functions to allow it to be parsed
class GCodeBuffer INHERIT_OBJECT_MODEL
{
public:
	friend class BinaryParser;
	friend class StringParser;

	GCodeBuffer(GCodeChannel::RawType channel, GCodeInput *normalIn, FileGCodeInput *fileIn, MessageType mt, Compatibility::RawType c = Compatibility::RepRapFirmware) noexcept;
	void Reset() noexcept;														// Reset it to its state after start-up
	void Init() noexcept;														// Set it up to parse another G-code
	void Diagnostics(MessageType mtype) noexcept;								// Write some debug info

	bool Put(char c) noexcept __attribute__((hot));								// Add a character to the end
#if HAS_LINUX_INTERFACE
	void PutAndDecode(const char *data, size_t len, bool isBinary = false) noexcept;	// Add an entire G-Code, overwriting any existing content
#else
	void PutAndDecode(const char *data, size_t len) noexcept;					// Add an entire G-Code, overwriting any existing content
#endif
	void PutAndDecode(const char *str) noexcept;								// Add a null-terminated string, overwriting any existing content
	void StartNewFile() noexcept;												// Called when we start a new file
	bool FileEnded() noexcept;													// Called when we reach the end of the file we are reading from
	void DecodeCommand() noexcept;												// Decode the command in the buffer when it is complete
	bool CheckMetaCommand(const StringRef& reply) THROWS(GCodeException);		// Check whether the current command is a meta command, or we are skipping a block

	char GetCommandLetter() const noexcept;
	bool HasCommandNumber() const noexcept;
	int GetCommandNumber() const noexcept;
	int8_t GetCommandFraction() const noexcept;
	const char *GetCompleteParameters() noexcept;								// Get all of the line following the command. Currently called only for the Q0 command.
	int32_t GetLineNumber() const noexcept { return machineState->lineNumber; }
	GCodeResult GetLastResult() const noexcept { return lastResult; }
	void SetLastResult(GCodeResult r) noexcept { lastResult = r; }

	bool Seen(char c) noexcept __attribute__((hot));								// Is a character present?
	void MustSee(char c) THROWS(GCodeException);									// Test for character present, throw error if not

	float GetFValue() THROWS(GCodeException) __attribute__((hot));					// Get a float after a key letter
	float GetDistance() THROWS(GCodeException);										// Get a distance or coordinate and convert it from inches to mm if necessary
	int32_t GetIValue() THROWS(GCodeException) __attribute__((hot));				// Get an integer after a key letter
	int32_t GetLimitedIValue(char c, int32_t minValue, int32_t maxValue) THROWS(GCodeException)
		post(minValue <= result; result <= maxValue);								// Get an integer after a key letter
	uint32_t GetUIValue() THROWS(GCodeException);									// Get an unsigned integer value
	uint32_t GetLimitedUIValue(char c, uint32_t maxValuePlusOne) THROWS(GCodeException)
		post(result < maxValuePlusOne);												// Get an unsigned integer value, throw if >= limit
	void GetIPAddress(IPAddress& returnedIp) THROWS(GCodeException);				// Get an IP address quad after a key letter
	void GetMacAddress(MacAddress& mac) THROWS(GCodeException);						// Get a MAC address sextet after a key letter
	PwmFrequency GetPwmFrequency() THROWS(GCodeException);							// Get a PWM frequency
	float GetPwmValue() THROWS(GCodeException);										// Get a PWM value
	DriverId GetDriverId() THROWS(GCodeException);									// Get a driver ID
	void GetUnprecedentedString(const StringRef& str, bool allowEmpty = false) THROWS(GCodeException);	// Get a string with no preceding key letter
	void GetQuotedString(const StringRef& str) THROWS(GCodeException);				// Get and copy a quoted string
	void GetPossiblyQuotedString(const StringRef& str) THROWS(GCodeException);		// Get and copy a string which may or may not be quoted
	void GetReducedString(const StringRef& str) THROWS(GCodeException);				// Get and copy a quoted string, removing certain characters
	void GetFloatArray(float arr[], size_t& length, bool doPad) THROWS(GCodeException) __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length, bool doPad) THROWS(GCodeException);		// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) THROWS(GCodeException);	// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length) THROWS(GCodeException);	// Get a :-separated list of drivers after a key letter

	bool TryGetFValue(char c, float& val, bool& seen) THROWS(GCodeException);
	bool TryGetIValue(char c, int32_t& val, bool& seen) THROWS(GCodeException);
	bool TryGetLimitedIValue(char c, int32_t& val, bool& seen, int32_t minValue, int32_t maxValue) THROWS(GCodeException);
	bool TryGetUIValue(char c, uint32_t& val, bool& seen) THROWS(GCodeException);
	bool TryGetLimitedUIValue(char c, uint32_t& val, bool& seen, uint32_t maxValuePlusOne) THROWS(GCodeException);
	bool TryGetBValue(char c, bool& val, bool& seen) THROWS(GCodeException);
	bool TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad = false) THROWS(GCodeException);
	bool TryGetUIArray(char c, size_t numVals, uint32_t vals[], const StringRef& reply, bool& seen, bool doPad = false) THROWS(GCodeException);
	bool TryGetQuotedString(char c, const StringRef& str, bool& seen) THROWS(GCodeException);
	bool TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen) THROWS(GCodeException);

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
	unsigned int GetStackDepth() const noexcept;
	bool PushState(bool withinSameFile) noexcept;				// Push state returning true if successful (i.e. stack not overflowed)
	bool PopState(bool withinSameFile) noexcept;				// Pop state returning true if successful (i.e. no stack underrun)

	void AbortFile(bool abortAll, bool requestAbort = true) noexcept;
	bool IsDoingFile() const noexcept;							// Return true if this source is executing a file
	bool IsDoingLocalFile() const noexcept;						// Return true if this source is executing a file from the local SD card
	bool IsDoingFileMacro() const noexcept;						// Return true if this source is executing a file macro
	FilePosition GetFilePosition() const noexcept;				// Get the file position at the start of the current command

#if HAS_LINUX_INTERFACE
	bool IsBinary() const noexcept { return isBinaryBuffer; }	// Return true if the code is in binary format

	void SetPrintFinished() noexcept;							// Mark the print file as finished
	bool IsFileFinished() const noexcept;						// Return true if this source has finished execution of a file

	bool IsMacroRequested() const noexcept { return !requestedMacroFile.IsEmpty(); }			// Indicates if a macro file is being requested
	void RequestMacroFile(const char *filename, bool reportMissing, bool fromCode) noexcept;	// Request execution of a file macro
	const char *GetRequestedMacroFile(bool& reportMissing, bool &fromCode) const noexcept;		// Return requested macro file or nullptr if none

	bool IsAbortRequested() const noexcept;						// Is the cancellation of the current file requested?
	bool IsAbortAllRequested() const noexcept;					// Is the cancellation of all files being executed on this channel requested?
	void AcknowledgeAbort() noexcept;							// Indicates that the current macro file is being cancelled

	bool IsInvalidated() const noexcept { return invalidated; }	// Indicates if the channel is invalidated
	void Invalidate(bool i = true) noexcept { invalidated = i; }	// Invalidate this channel (or not)

	bool IsSendRequested() const noexcept { return sendToSbc; }	// Is this code supposed to be sent to the SBC
	void SendToSbc() noexcept { sendToSbc = true; }				// Send this code to the attached SBC
#endif

	GCodeState GetState() const noexcept;
	void SetState(GCodeState newState) noexcept;
	void SetState(GCodeState newState, const char *err) noexcept;
	void AdvanceState() noexcept;
	void MessageAcknowledged(bool cancelled) noexcept;

	GCodeChannel GetChannel() const noexcept { return codeChannel; }
	const char *GetIdentity() const noexcept { return codeChannel.ToString(); }
	bool CanQueueCodes() const noexcept;
	MessageType GetResponseMessageType() const noexcept;

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

	void MotionCommanded() noexcept { motionCommanded = true; }
	void MotionStopped() noexcept { motionCommanded = false; }
	bool WasMotionCommanded() const noexcept { return motionCommanded; }

protected:
	DECLARE_OBJECT_MODEL

private:

#if SUPPORT_OBJECT_MODEL
	const char *GetStateText() const noexcept;
#endif

	const GCodeChannel codeChannel;						// Channel number of this instance
	GCodeInput *normalInput;							// Our normal input stream, or nullptr if there isn't one

#if HAS_MASS_STORAGE
	FileGCodeInput *fileInput;							// Our file input stream for when we are reading from a print file or a macro file, may be shared with other GCodeBuffers
#endif

	const MessageType responseMessageType;				// The message type we use for responses to string codes coming from this channel

	GCodeResult lastResult;

#if HAS_LINUX_INTERFACE
	BinaryParser binaryParser;
#endif

	StringParser stringParser;

	GCodeBufferState bufferState;						// Idle, executing or paused
	GCodeMachineState *machineState;					// Machine state for this gcode source

	uint32_t whenTimerStarted;							// When we started waiting

#if HAS_LINUX_INTERFACE
	bool isBinaryBuffer;
#endif
	bool timerRunning;									// True if we are waiting
	bool motionCommanded;								// true if this GCode stream has commanded motion since it last waited for motion to stop

#if HAS_LINUX_INTERFACE
	alignas(4) char buffer[MaxCodeBufferSize];			// must be aligned because we do dword fetches from it
#else
	char buffer[GCODE_LENGTH];
#endif

#if HAS_LINUX_INTERFACE
	String<MaxFilenameLength> requestedMacroFile;
	uint8_t
		reportMissingMacro : 1,
		isMacroFromCode: 1,
		abortFile : 1,
		abortAllFiles : 1,
		invalidated : 1,
		sendToSbc : 1;
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

// Return true if this source is executing a file from the local SD card
inline bool GCodeBuffer::IsDoingLocalFile() const noexcept
{
#if HAS_LINUX_INTERFACE
	return !IsBinary() && IsDoingFile();
#else
	return IsDoingFile();
#endif
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

#endif

#endif /* SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H */
