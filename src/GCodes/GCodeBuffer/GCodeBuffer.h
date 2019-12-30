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

	GCodeBuffer(GCodeChannel channel, GCodeInput *normalIn, FileGCodeInput *fileIn, MessageType mt, Compatibility c = Compatibility::reprapFirmware);
	void Reset();																// Reset it to its state after start-up
	void Init();																// Set it up to parse another G-code
	void Diagnostics(MessageType mtype);										// Write some debug info

	bool IsBinary() const { return isBinaryBuffer; }							// Return true if the code is in binary format
	bool Put(char c) __attribute__((hot));										// Add a character to the end
	void PutAndDecode(const char *data, size_t len, bool isBinary);				// Add an entire G-Code, overwriting any existing content
	void PutAndDecode(const char *str);											// Add a null-terminated string, overwriting any existing content
	bool FileEnded();															// Called when we reach the end of the file we are reading from
	void DecodeCommand();														// Decode the command in the buffer when it is complete
	bool CheckMetaCommand() THROWS_PARSE_ERROR;									// Check whether the current command is a meta command, or we are skipping a block

	char GetCommandLetter() const;
	bool HasCommandNumber() const;
	int GetCommandNumber() const;
	int8_t GetCommandFraction() const;

	bool Seen(char c) noexcept __attribute__((hot));							// Is a character present?
	void MustSee(char c) THROWS_PARSE_ERROR;									// Test for character present, throw error if not

	float GetFValue() THROWS_PARSE_ERROR __attribute__((hot));					// Get a float after a key letter
	float GetDistance() THROWS_PARSE_ERROR;										// Get a distance or coordinate and convert it from inches to mm if necessary
	int32_t GetIValue() THROWS_PARSE_ERROR __attribute__((hot));				// Get an integer after a key letter
	uint32_t GetUIValue() THROWS_PARSE_ERROR;									// Get an unsigned integer value
	void GetIPAddress(IPAddress& returnedIp) THROWS_PARSE_ERROR;				// Get an IP address quad after a key letter
	void GetMacAddress(uint8_t mac[6]) THROWS_PARSE_ERROR;						// Get a MAC address sextet after a key letter
	PwmFrequency GetPwmFrequency() THROWS_PARSE_ERROR;							// Get a PWM frequency
	float GetPwmValue() THROWS_PARSE_ERROR;										// Get a PWM value
	DriverId GetDriverId() THROWS_PARSE_ERROR;									// Get a driver ID
	void GetUnprecedentedString(const StringRef& str, bool allowEmpty = false) THROWS_PARSE_ERROR;	// Get a string with no preceding key letter
	void GetQuotedString(const StringRef& str) THROWS_PARSE_ERROR;				// Get and copy a quoted string
	void GetPossiblyQuotedString(const StringRef& str) THROWS_PARSE_ERROR;		// Get and copy a string which may or may not be quoted
	void GetReducedString(const StringRef& str) THROWS_PARSE_ERROR;				// Get and copy a quoted string, removing certain characters
	void GetFloatArray(float arr[], size_t& length, bool doPad) THROWS_PARSE_ERROR __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length, bool doPad) THROWS_PARSE_ERROR;		// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) THROWS_PARSE_ERROR;	// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length) THROWS_PARSE_ERROR;	// Get a :-separated list of drivers after a key letter

	bool TryGetFValue(char c, float& val, bool& seen) THROWS_PARSE_ERROR;
	bool TryGetIValue(char c, int32_t& val, bool& seen) THROWS_PARSE_ERROR;
	bool TryGetUIValue(char c, uint32_t& val, bool& seen) THROWS_PARSE_ERROR;
	bool TryGetBValue(char c, bool& val, bool& seen) THROWS_PARSE_ERROR;
	bool TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad = false) THROWS_PARSE_ERROR;
	bool TryGetUIArray(char c, size_t numVals, uint32_t vals[], const StringRef& reply, bool& seen, bool doPad = false) THROWS_PARSE_ERROR;
	bool TryGetQuotedString(char c, const StringRef& str, bool& seen) THROWS_PARSE_ERROR;
	bool TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen) THROWS_PARSE_ERROR;

	bool IsIdle() const;
	bool IsCompletelyIdle() const;
	bool IsReady() const;								// Return true if a gcode is ready but hasn't been started yet
	bool IsExecuting() const;							// Return true if a gcode has been started and is not paused
	void SetFinished(bool f);							// Set the G Code executed (or not)
	void SetCommsProperties(uint32_t arg);

	GCodeMachineState& MachineState() const { return *machineState; }
	GCodeMachineState& OriginalMachineState() const;
	float ConvertDistance(float distance) const;
	float InverseConvertDistance(float distance) const;
	bool PushState(bool preserveLineNumber);			// Push state returning true if successful (i.e. stack not overflowed)
	bool PopState(bool preserveLineNumber);				// Pop state returning true if successful (i.e. no stack underrun)

	void AbortFile(bool abortAll, bool requestAbort = true);
	bool IsDoingFile() const;							// Return true if this source is executing a file
	bool IsDoingFileMacro() const;						// Return true if this source is executing a file macro
	FilePosition GetFilePosition() const;				// Get the file position at the start of the current command

#if HAS_LINUX_INTERFACE
	void SetPrintFinished();							// Mark the print file as finished
	bool IsFileFinished() const;						// Return true if this source has finished execution of a file

	bool IsMacroRequested() const { return !requestedMacroFile.IsEmpty(); }					// Indicates if a macro file is being requested
	void RequestMacroFile(const char *filename, bool reportMissing, bool fromCode);	// Request execution of a file macro
	const char *GetRequestedMacroFile(bool& reportMissing, bool &fromCode) const;		// Return requested macro file or nullptr if none

	bool IsAbortRequested() const;						// Is the cancellation of the current file requested?
	bool IsAbortAllRequested() const;					// Is the cancellation of all files being executed on this channel requested?
	void AcknowledgeAbort();							// Indicates that the current macro file is being cancelled

	void ReportStack() { reportStack = true; }			// Flags current stack details to be reported
	bool IsStackEventFlagged() const;					// Did the stack change?
	void AcknowledgeStackEvent();						// Indicates that the last stack event has been written

	bool IsInvalidated() const { return invalidated; }	// Indicates if the channel is invalidated
	void Invalidate(bool i = true) { invalidated = i; }	// Invalidate this channel (or not)
#endif

	GCodeState GetState() const;
	void SetState(GCodeState newState);
	void SetState(GCodeState newState, const char *err);
	void AdvanceState();
	void MessageAcknowledged(bool cancelled);

	GCodeChannel GetChannel() const { return codeChannel; }
	const char *GetIdentity() const { return gcodeChannelName[(size_t)codeChannel]; }
	bool CanQueueCodes() const;
	MessageType GetResponseMessageType() const;

	int GetToolNumberAdjust() const { return toolNumberAdjust; }
	void SetToolNumberAdjust(int arg) { toolNumberAdjust = arg; }

#if HAS_MASS_STORAGE
	bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32);	// open a file to write to
	bool IsWritingFile() const;							// Returns true if writing a file
	void WriteToFile();									// Write the current GCode to file

	bool IsWritingBinary() const;						// Returns true if writing binary
	void WriteBinaryToFile(char b);						// Write a byte to the file
	void FinishWritingBinary();
#endif

	const char* DataStart() const;						// Get the start of the current command
	size_t DataLength() const;							// Get the length of the current command

	void PrintCommand(const StringRef& s) const;
	void AppendFullCommand(const StringRef &s) const;

	bool IsTimerRunning() const { return timerRunning; }
	uint32_t WhenTimerStarted() const { return whenTimerStarted; }
	void StartTimer();
	void StopTimer() { timerRunning = false; }
	bool DoDwellTime(uint32_t dwellMillis);				// Execute a dwell returning true if it has finished

	void RestartFrom(FilePosition pos);

#if HAS_MASS_STORAGE
	FileGCodeInput *GetFileInput() const { return fileInput; }	//TEMPORARY!
#endif
	GCodeInput *GetNormalInput() const { return normalInput; }	//TEMPORARY!

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

inline bool GCodeBuffer::IsDoingFileMacro() const
{
	return machineState->doingFileMacro;
}

inline GCodeState GCodeBuffer::GetState() const
{
	return machineState->state;
}

inline void GCodeBuffer::SetState(GCodeState newState)
{
	machineState->state = newState;
}

inline void GCodeBuffer::SetState(GCodeState newState, const char *err)
{
	machineState->state = newState;
	machineState->errorMessage = err;
}

inline void GCodeBuffer::AdvanceState()
{
	machineState->state = static_cast<GCodeState>(static_cast<uint8_t>(machineState->state) + 1);
}

#endif /* SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H */
