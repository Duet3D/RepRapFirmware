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

#include <RepRapFirmware.h>
#include <GCodes/GCodeChannel.h>
#include <GCodes/GCodeMachineState.h>
#include <ObjectModel/ObjectModel.h>

#if HAS_SBC_INTERFACE
# include <SBC/SbcMessageFormats.h>
#endif

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

// Type of a status report
enum class StatusReportType : uint8_t
{
	none = 0,
	m105,
	m408,
	m409
};

// Class to hold an individual GCode and provide functions to allow it to be parsed
class GCodeBuffer INHERIT_OBJECT_MODEL
{
public:
#ifndef __ECV__		//temporary!
	friend class BinaryParser;
	friend class StringParser;
#endif

	GCodeBuffer(GCodeChannel::RawType channel, GCodeInput *_ecv_from normalIn, FileGCodeInput *fileIn, MessageType mt, Compatibility::RawType c = Compatibility::RepRapFirmware) noexcept;
	void Reset() noexcept;														// Reset it to its state after start-up
	void Init() noexcept;														// Set it up to parse another G-code
	void Diagnostics(MessageType mtype) noexcept;								// Write some debug info

	bool Put(char c) noexcept SPEED_CRITICAL;									// Add a character to the end
#if HAS_SBC_INTERFACE
	void PutBinary(const uint32_t *data, size_t len) noexcept;					// Add an entire binary G-Code, overwriting any existing content
#endif
	void PutAndDecode(const char *data, size_t len) noexcept;					// Add an entire G-Code, overwriting any existing content
	void PutAndDecode(const char *str) noexcept;								// Add a null-terminated string, overwriting any existing content
	void StartNewFile() noexcept;												// Called when we start a new file
	bool FileEnded() noexcept;													// Called when we reach the end of the file we are reading from
	void DecodeCommand() noexcept;												// Decode the command in the buffer when it is complete
	bool CheckMetaCommand(const StringRef& reply) THROWS(GCodeException);		// Check whether the current command is a meta command, or we are skipping a block

	char GetCommandLetter() const noexcept;
	bool HasCommandNumber() const noexcept;
	int GetCommandNumber() const noexcept;
	int8_t GetCommandFraction() const noexcept;										// Return the command fraction, or -1 if none given
	bool ContainsExpression() const noexcept;
	void GetCompleteParameters(const StringRef& str) THROWS(GCodeException);		// Get all of the line following the command. Currently called only for the Q0 command.
	int32_t GetLineNumber() const noexcept { return CurrentFileMachineState().lineNumber; }
	bool IsLastCommand() const noexcept;
	GCodeResult GetLastResult() const noexcept { return lastResult; }
	void SetLastResult(GCodeResult r) noexcept { lastResult = r; }
	ExpressionValue GetM291Result() const noexcept { return m291Result; }

	bool Seen(char c) noexcept SPEED_CRITICAL;										// Is a character present?
	void MustSee(char c) THROWS(GCodeException);									// Test for character present, throw error if not
	char MustSee(char c1, char c2) THROWS(GCodeException);							// Test for one of two characters present, throw error if not
	ParameterLettersBitmap AllParameters() const noexcept;							// Return a bitmap of all parameters in the command
	bool SeenAny(ParameterLettersBitmap bm) const noexcept							// Return true if any of the parameter letters in the bitmap were seen
		{ return AllParameters().Intersects(bm); }
	bool SeenAny(const char *s) const noexcept										// Return true if any of the parameter letters in the string were seen
		{ return SeenAny(ParameterLettersToBitmap(s)); }

	float GetFValue() THROWS(GCodeException) SPEED_CRITICAL;						// Get a float after a key letter
	float GetPositiveFValue() THROWS(GCodeException) SPEED_CRITICAL;				// Get a float after a key letter and check that it is greater than zero
	float GetNonNegativeFValue() THROWS(GCodeException) SPEED_CRITICAL;				// Get a float after a key letter and check that it is greater than or equal to zero
	float GetDistance() THROWS(GCodeException);										// Get a distance or coordinate and convert it from inches to mm if necessary
	float GetSpeed() THROWS(GCodeException);										// Get a speed in mm/min or inches/min and convert it to mm/step_clock
	float GetSpeedFromMm(bool useSeconds) THROWS(GCodeException);					// Get a speed in mm/min or optionally /sec and convert it to mm/step_clock
	float GetAcceleration() THROWS(GCodeException);									// Get an acceleration in mm/sec^2 or inches/sec^2 and convert it to mm/step_clock^2
	int32_t GetIValue() THROWS(GCodeException) SPEED_CRITICAL;						// Get an integer after a key letter
	int32_t GetLimitedIValue(char c, int32_t minValue, int32_t maxValue) THROWS(GCodeException)
		pre(minValue <= maxValue)
		post(minValue <= _ecv_result; _ecv_result <= maxValue);								// Get an integer after a key letter
	uint32_t GetUIValue() THROWS(GCodeException);									// Get an unsigned integer value
	uint32_t GetLimitedUIValue(char c, uint32_t minValue, uint32_t maxValuePlusOne) THROWS(GCodeException)		// Get an unsigned integer value, throw if outside limits
		pre(maxValuePlusOne > minValue)												// Get an unsigned integer value, throw if outside limits
		post(_ecv_result >= minValue; _ecv_result < maxValuePlusOne);
	uint32_t GetLimitedUIValue(char c, uint32_t maxValuePlusOne) THROWS(GCodeException)
		post(_ecv_result < maxValuePlusOne) { return GetLimitedUIValue(c, 0, maxValuePlusOne); }
	float GetLimitedFValue(char c, float minValue, float maxValue) THROWS(GCodeException)
		pre(minValue <= maxValue)
		post(minValue <= _ecv_result; _ecv_result <= maxValue);								// Get a float after a key letter
	void GetIPAddress(IPAddress& returnedIp) THROWS(GCodeException);				// Get an IP address quad after a key letter
	void GetMacAddress(MacAddress& mac) THROWS(GCodeException);						// Get a MAC address sextet after a key letter
	PwmFrequency GetPwmFrequency() THROWS(GCodeException);							// Get a PWM frequency
	float GetPwmValue() THROWS(GCodeException);										// Get a PWM value
	DriverId GetDriverId() THROWS(GCodeException);									// Get a driver ID
	void GetUnprecedentedString(const StringRef& str, bool allowEmpty = false) THROWS(GCodeException);	// Get a string with no preceding key letter
	void GetQuotedString(const StringRef& str, bool allowEmpty = false) THROWS(GCodeException);			// Get and copy a quoted string
	void GetPossiblyQuotedString(const StringRef& str, bool allowEmpty = false) THROWS(GCodeException);	// Get and copy a string which may or may not be quoted
	void GetReducedString(const StringRef& str) THROWS(GCodeException);				// Get and copy a quoted string, removing certain characters
	void GetFloatArray(float arr[], size_t& length, bool doPad) THROWS(GCodeException) SPEED_CRITICAL; // Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length, bool doPad) THROWS(GCodeException);		// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) THROWS(GCodeException);	// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length) THROWS(GCodeException);	// Get a :-separated list of drivers after a key letter
	ExpressionValue GetExpression() THROWS(GCodeException);							// Get a general expression after a key letter

	bool TryGetFValue(char c, float& val, bool& seen) THROWS(GCodeException);
	bool TryGetIValue(char c, int32_t& val, bool& seen) THROWS(GCodeException);
	bool TryGetLimitedIValue(char c, int32_t& val, bool& seen, int32_t minValue, int32_t maxValue) THROWS(GCodeException);
	bool TryGetUIValue(char c, uint32_t& val, bool& seen) THROWS(GCodeException);
	bool TryGetLimitedUIValue(char c, uint32_t& val, bool& seen, uint32_t maxValuePlusOne) THROWS(GCodeException);
	bool TryGetNonNegativeFValue(char c, float& val, bool& seen) THROWS(GCodeException);
	bool TryGetLimitedFValue(char c, float& val, bool& seen, float minValue, float maxValue) THROWS(GCodeException)
		pre(minValue <= maxValue);
	bool TryGetBValue(char c, bool& val, bool& seen) THROWS(GCodeException);
	void TryGetFloatArray(char c, size_t numVals, float vals[], bool& seen, bool doPad = false) THROWS(GCodeException);
	void TryGetUIArray(char c, size_t numVals, uint32_t vals[], bool& seen, bool doPad = false) THROWS(GCodeException);
	bool TryGetQuotedString(char c, const StringRef& str, bool& seen, bool allowEmpty = false) THROWS(GCodeException);
	bool TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen) THROWS(GCodeException);

	bool IsIdle() const noexcept;
	bool IsCompletelyIdle() const noexcept;
	bool IsReady() const noexcept;								// Return true if a gcode is ready but hasn't been started yet
	bool IsExecuting() const noexcept;							// Return true if a gcode has been started and is not paused
	void SetFinished(bool f) noexcept;							// Set the G Code executed (or not)

	size_t GetActiveQueueNumber() const noexcept				// Get the movement queue number that this buffer uses
	{
#if SUPPORT_ASYNC_MOVES
		return machineState->GetCommandedQueue();
#else
		return 0;
#endif
	}

#if SUPPORT_ASYNC_MOVES
	void SetActiveQueueNumber(MovementSystemNumber qn) noexcept { machineState->SetCommandedQueue(qn); }
	void ExecuteOnlyQueue(MovementSystemNumber qn) noexcept { machineState->ExecuteOnly(qn); }
	size_t GetOwnQueueNumber() const noexcept { return machineState->GetOwnQueue(); }
	void ExecuteAll() noexcept { machineState->ExecuteAll(); }
	bool Executing() const noexcept { return machineState->Executing(); }	// Return true if this GCodeBuffer for executing commands addressed to the current queue
	bool ExecutingAll() const noexcept { return machineState->ExecutingAll(); }
	size_t GetQueueNumberToLock() const noexcept { return machineState->GetQueueNumberToLock(); }
	void ForkFrom(const GCodeBuffer& other) noexcept;
#endif

	void SetCommsProperties(uint32_t arg) noexcept;

	GCodeMachineState& LatestMachineState() const noexcept { return *machineState; }
	GCodeMachineState& CurrentFileMachineState() const noexcept;
	GCodeMachineState& OriginalMachineState() const noexcept;
	GCodeMachineState::BlockState& GetBlockState() const noexcept { return CurrentFileMachineState().CurrentBlockState(); }
	uint16_t GetBlockIndent() const noexcept { return GetBlockState().GetIndent(); }
	bool AllStatesNormal() const noexcept;						// Return true if all GCode states on the stack are 'normal'

	void UseInches(bool inchesNotMm) noexcept { machineState->usingInches = inchesNotMm; }
	bool UsingInches() const noexcept { return machineState->usingInches; }
	float ConvertDistance(float distance) const noexcept;
	float InverseConvertDistance(float distance) const noexcept;
	float ConvertSpeed(float speed) const noexcept;
	float InverseConvertSpeed(float speed) const noexcept;
	const char *GetDistanceUnits() const noexcept;
	unsigned int GetStackDepth() const noexcept;
	bool PushState(bool withinSameFile) noexcept;							// Push state returning true if successful (i.e. stack not overflowed)
	bool PopState(bool withinSameFile) noexcept;							// Pop state returning true if successful (i.e. no stack underrun)

	void AbortFile(bool abortAll, bool requestAbort = true) noexcept;
	bool IsDoingFile() const noexcept;										// Return true if this source is executing a file
	bool IsDoingLocalFile() const noexcept;									// Return true if this source is executing a file from the local SD card
	bool IsDoingFileMacro() const noexcept;									// Return true if this source is executing a file macro
	FilePosition GetJobFilePosition() const noexcept;						// Get the file position at the start of the current command
	FilePosition GetPrintingFilePosition(bool allowNoFilePos) const noexcept;	// Get the file position in the printing file
	void SavePrintingFilePosition() noexcept;

	void WaitForAcknowledgement(uint32_t seq) noexcept;						// Flag that we are waiting for acknowledgement
	void ClosePrintFile() noexcept;											// Close the print file

	StatusReportType GetLastStatusReportType() const noexcept { return lastStatusReportType; }
	void RespondedToStatusRequest(StatusReportType t) noexcept { lastStatusReportType = t; }	// call this when a response to a status request is sent

#if HAS_SBC_INTERFACE
	bool IsBinary() const noexcept { return isBinaryBuffer; }				// Return true if the code is in binary format

	bool IsFileFinished() const noexcept;									// Return true if this source has finished execution of a file
	void SetFileFinished() noexcept;										// Mark the current file as finished
	void SetPrintFinished() noexcept;										// Mark the current print file as finished

	bool RequestMacroFile(const char *filename, bool fromCode) noexcept;	// Request execution of a file macro
	volatile bool IsWaitingForMacro() const noexcept { return isWaitingForMacro; }	// Indicates if the GB is waiting for a macro to be opened
	bool HasJustStartedMacro() const noexcept { return macroJustStarted; }	// Has this GB just started a new macro file?
	bool IsMacroRequestPending() const noexcept { return !requestedMacroFile.IsEmpty(); }		// Indicates if a macro file is being requested
	const char *GetRequestedMacroFile() const noexcept { return requestedMacroFile.c_str(); }	// Return requested macro file or nullptr if none
	bool IsMacroStartedByCode() const noexcept;								// Indicates if the last macro was requested from a code
	void MacroRequestSent() noexcept { requestedMacroFile.Clear(); }		// Called when a macro file request has been sent
	void ResolveMacroRequest(bool hadError, bool isEmpty) noexcept;			// Resolve the call waiting for a macro to be executed
	bool IsMacroEmpty() const noexcept { return macroFileEmpty; }			// Return true if the opened macro file is actually empty

	void MacroFileClosed() noexcept;										// Called to notify the SBC about the file being internally closed on success
	volatile bool IsMacroFileClosed() const noexcept { return macroFileClosed; }	// Indicates if a file has been closed internally in RRF
	void MacroFileClosedSent() noexcept { macroFileClosed = false; }		// Called when the SBC has been notified about the internally closed file

	bool IsAbortRequested() const noexcept { return abortFile; }			// Is the cancellation of the current file requested?
	bool IsAbortAllRequested() const noexcept { return abortAllFiles; }		// Is the cancellation of all files being executed on this channel requested?
	void FileAbortSent() noexcept { abortFile = abortAllFiles = false; }	// Called when the SBC has been notified about a file (or all files) being closed

	bool IsMessagePromptPending() const noexcept { return messagePromptPending; }	// Is the SBC supposed to be notified about a message waiting for acknowledgement?
	void MessagePromptSent() noexcept { messagePromptPending = false; }				// Called when the SBC has been notified about a message waiting for acknowledgement
	bool IsMessageAcknowledged() const noexcept { return messageAcknowledged; }		// Indicates if a message has been acknowledged
	void MessageAcknowledgementSent() noexcept { messageAcknowledged = false; }		// Called when the SBC has been notified about the message acknowledgement

	bool IsInvalidated() const noexcept { return invalidated; }		// Indicates if the channel is invalidated
	void Invalidate(bool i = true) noexcept { invalidated = i; }	// Invalidate this channel (or not)

	bool IsSendRequested() const noexcept { return sendToSbc; }	// Is this code supposed to be sent to the SBC
	void SendToSbc() noexcept { sendToSbc = true; }				// Send this code to the attached SBC
#endif

	GCodeState GetState() const noexcept;
	void SetState(GCodeState newState) noexcept;
	void SetState(GCodeState newState, uint16_t param) noexcept;
	void AdvanceState() noexcept;
	void MessageAcknowledged(bool cancelled, uint32_t seq, ExpressionValue rslt) noexcept;

	GCodeChannel GetChannel() const noexcept { return codeChannel; }
	bool IsFileChannel() const noexcept
	{
		return codeChannel == GCodeChannel::File
#if SUPPORT_ASYNC_MOVES
			|| codeChannel == GCodeChannel::File2
#endif
				;
	}
	const char *_ecv_array GetIdentity() const noexcept { return codeChannel.ToString(); }
	bool CanQueueCodes() const noexcept;
	MessageType GetResponseMessageType() const noexcept;

#if HAS_MASS_STORAGE
	bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) noexcept;
																// open a file to write to
	bool IsWritingFile() const noexcept;						// Returns true if writing a file
	void WriteToFile() noexcept;								// Write the current GCode to file

	bool IsWritingBinary() const noexcept;						// Returns true if writing binary
	bool WriteBinaryToFile(char b) noexcept;					// Write a byte to the file, returning true if the upload is now complete
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

	void ResetReportDueTimer() noexcept { whenReportDueTimerStarted = millis(); };
	bool IsReportDue() noexcept;

	bool IsWaitingForTemperatures() const noexcept;
	void CancelWaitForTemperatures() noexcept { cancelWait = true; }
	bool IsCancelWaitRequested() noexcept;

	void RestartFrom(FilePosition pos) noexcept;

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	FileGCodeInput *GetFileInput() const noexcept { return fileInput; }
#endif
	GCodeInput *_ecv_from GetNormalInput() const noexcept { return normalInput; }

	void MotionCommanded() noexcept { motionCommanded = true; }
	void MotionStopped() noexcept { motionCommanded = false; }
	bool WasMotionCommanded() const noexcept { return motionCommanded; }

	void AddParameters(VariableSet& vars, int codeRunning) THROWS(GCodeException);
	VariableSet& GetVariables() const noexcept;

	[[noreturn]] void ThrowGCodeException(const char *msg) const THROWS(GCodeException);
	[[noreturn]] void ThrowGCodeException(const char *msg, uint32_t param) const THROWS(GCodeException);

#if SUPPORT_COORDINATE_ROTATION
	bool DoingCoordinateRotation() const noexcept;
#endif

#if SUPPORT_ASYNC_MOVES
	bool IsLaterThan(const GCodeBuffer& other) const noexcept;
#endif

	Mutex mutex;

#if SUPPORT_ASYNC_MOVES
	enum class SyncState { running, syncing, synced } ;
	SyncState syncState = SyncState::running;
#endif

protected:
	DECLARE_OBJECT_MODEL

private:

#if SUPPORT_OBJECT_MODEL
	const char *GetStateText() const noexcept;
#endif

	FilePosition printFilePositionAtMacroStart;			// the saved file position when we started executing a macro
	GCodeInput *_ecv_from normalInput;					// Our normal input stream, or nullptr if there isn't one

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	FileGCodeInput *fileInput;							// Our file input stream for when we are reading from a print file or a macro file, may be shared with other GCodeBuffers
#endif

	const MessageType responseMessageType;				// The message type we use for responses to string codes coming from this channel

#if HAS_SBC_INTERFACE
	BinaryParser binaryParser;
#endif

	StringParser stringParser;

	GCodeMachineState *machineState;					// Machine state for this gcode source
	ExpressionValue m291Result;							// the value entered or choice selected in response to a M291 command

	uint32_t whenTimerStarted;							// When we started waiting
	uint32_t whenReportDueTimerStarted;					// When the report-due-timer has been started
	StatusReportType lastStatusReportType;				// the type of the last status report sent on this channel

	const GCodeChannel codeChannel;						// Channel number of this instance
	GCodeBufferState bufferState;						// Idle, executing or paused
	GCodeResult lastResult;

	bool timerRunning;									// true if we are waiting
	bool motionCommanded;								// true if this GCode stream has commanded motion since it last waited for motion to stop
	bool cancelWait;									// true to stop waiting for temperatures to be reached

	alignas(4) char buffer[MaxGCodeLength];				// must be aligned because in SBC binary mode we do dword fetches from it

#if HAS_SBC_INTERFACE
	static_assert(MaxGCodeLength >= MaxCodeBufferSize);	// make sure the GCodeBuffer is large enough to hold a command received from the SBC in binary

	// Accessed by both the Main and SBC tasks
	BinarySemaphore macroSemaphore;
	volatile bool isWaitingForMacro;	// Is this GB waiting in DoFileMacro?
	volatile bool macroFileClosed;		// Last macro file has been closed in RRF, tell the SBC

	// Accessed only when the GB mutex is acquired
	String<MaxFilenameLength> requestedMacroFile;
	bool isBinaryBuffer;
	uint8_t
		macroJustStarted : 1,		// Whether the GB has just started a macro file
		macroFileError : 1,			// Whether the macro file could be opened or if an error occurred
		macroFileEmpty : 1,			// Whether the macro file is actually empty
		abortFile : 1,				// Whether to abort the last file on the stack
		abortAllFiles : 1,			// Whether to abort all opened files
		sendToSbc : 1,				// Indicates if the GB string content is supposed to be sent to the SBC
		messagePromptPending : 1,	// Has the SBC been notified about a message waiting for acknowledgement?
		messageAcknowledged : 1;	// Last message has been acknowledged

	// Accessed only by the SBC task
	bool invalidated;				// Set to true if the GB content is not valid and about to be cleared
#endif
};

inline bool GCodeBuffer::IsDoingFileMacro() const noexcept
{
#if HAS_SBC_INTERFACE
	return machineState->doingFileMacro || IsMacroRequestPending() || macroFileClosed;
#else
	return machineState->doingFileMacro;
#endif
}

#if HAS_SBC_INTERFACE

inline bool GCodeBuffer::IsFileFinished() const noexcept
{
	return machineState->fileFinished;
}

inline bool GCodeBuffer::IsMacroStartedByCode() const noexcept
{
	return machineState->macroStartedByCode;
}

#endif

inline GCodeState GCodeBuffer::GetState() const noexcept
{
	return machineState->GetState();
}

inline void GCodeBuffer::SetState(GCodeState newState) noexcept
{
	machineState->SetState(newState);
}

inline void GCodeBuffer::SetState(GCodeState newState, uint16_t param) noexcept
{
	machineState->stateParameter = param;
	machineState->SetState(newState);
}

inline void GCodeBuffer::AdvanceState() noexcept
{
	machineState->AdvanceState();
}

// Return true if we can queue the current gcode command from this source. This is the case if a file is being executed.
// We can't queue it if it contains an expression, because the expression value may change or refer to 'iterations'.
inline bool GCodeBuffer::CanQueueCodes() const noexcept
{
	return machineState->DoingFile() && !ContainsExpression();
}

inline bool GCodeBuffer::IsDoingFile() const noexcept
{
#if HAS_SBC_INTERFACE
	return machineState->DoingFile() || IsMacroRequestPending();
#else
	return machineState->DoingFile();
#endif
}

inline bool GCodeBuffer::IsReady() const noexcept
{
	return bufferState == GCodeBufferState::ready;
}

inline bool GCodeBuffer::IsExecuting() const noexcept
{
	return bufferState == GCodeBufferState::executing;
}

// Return true if this source is executing a file from the local SD card
inline bool GCodeBuffer::IsDoingLocalFile() const noexcept
{
#if HAS_SBC_INTERFACE
	return !IsBinary() && IsDoingFile();
#else
	return IsDoingFile();
#endif
}

inline bool GCodeBuffer::IsCancelWaitRequested() noexcept
{
	const bool b = cancelWait;
	cancelWait = false;
	return b;
}

#endif /* SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H */
