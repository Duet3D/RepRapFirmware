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
#include "GCodes/GCodeMachineState.h"
#include "Linux/MessageFormats.h"
#include "MessageType.h"
#include "ObjectModel/ObjectModel.h"

// Class to hold an individual GCode and provide functions to allow it to be parsed
class GCodeBuffer
{
public:
	friend class BinaryParser;
	friend class StringParser;

	GCodeBuffer(const char *id, MessageType stringMt, MessageType binaryMt, bool usesCodeQueue);
	void Reset();											// Reset it to its state after start-up
	void Init();											// Set it up to parse another G-code
	void Diagnostics(MessageType mtype);					// Write some debug info

	bool IsBinary() const { return isBinaryBuffer; }		// Return true if the code is in binary format
	bool Put(char c) __attribute__((hot));					// Add a character to the end
	void Put(const char *data, size_t len, bool isBinary);	// Add an entire G-Code, overwriting any existing content
	void Put(const char *str);								// Add a null-terminated string, overwriting any existing content
	void FileEnded();										// Called when we reach the end of the file we are reading from

	char GetCommandLetter() const;
	bool HasCommandNumber() const;
	int GetCommandNumber() const;
	int8_t GetCommandFraction() const;

	bool Seen(char c) __attribute__((hot));					// Is a character present?
	float GetFValue() __attribute__((hot));					// Get a float after a key letter
	int32_t GetIValue() __attribute__((hot));				// Get an integer after a key letter
	uint32_t GetUIValue();									// Get an unsigned integer value
	bool GetIPAddress(IPAddress& returnedIp);				// Get an IP address quad after a key letter
	bool GetMacAddress(uint8_t mac[6]);						// Get a MAC address sextet after a key letter
	bool GetUnprecedentedString(const StringRef& str);		// Get a string with no preceding key letter
	bool GetQuotedString(const StringRef& str);				// Get and copy a quoted string
	bool GetPossiblyQuotedString(const StringRef& str);		// Get and copy a string which may or may not be quoted
	const void GetFloatArray(float arr[], size_t& length, bool doPad) __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	const void GetIntArray(int32_t arr[], size_t& length, bool doPad);			// Get a :-separated list of ints after a key letter
	const void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad);	// Get a :-separated list of unsigned ints after a key letter

	bool TryGetFValue(char c, float& val, bool& seen);
	bool TryGetIValue(char c, int32_t& val, bool& seen);
	bool TryGetUIValue(char c, uint32_t& val, bool& seen);
	bool TryGetBValue(char c, bool& val, bool& seen);
	bool TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad = false);
	bool TryGetUIArray(char c, size_t numVals, uint32_t vals[], const StringRef& reply, bool& seen, bool doPad = false);
	bool TryGetQuotedString(char c, const StringRef& str, bool& seen);
	bool TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen);

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
	bool PushState();									// Push state returning true if successful (i.e. stack not overflowed)
	bool PopState();									// Pop state returning true if successful (i.e. no stack underrun)

#if HAS_HIGH_SPEED_SD
	void AbortFile(FileGCodeInput* fileInput);			// Abort execution of any files or macros being executed
#elif HAS_LINUX_INTERFACE
	void AbortFile(bool requestAbort = true);
#endif

	bool IsDoingFile() const;							// Return true if this source is executing a file
	bool IsDoingFileMacro() const;						// Return true if this source is executing a file macro
	FilePosition GetFilePosition(size_t bytesCached) const;				// Get the file position at the start of the current command
#if HAS_LINUX_INTERFACE
	void SetPrintFinished();							// Mark the print file as finished
	bool IsFileFinished() const;						// Return true if this source has finished execution of a file

	bool IsMacroRequested() const { return !requestedMacroFile.IsEmpty(); }		// Indicate if a macro file is being requested
	void RequestMacroFile(const char *filename, bool reportMissing);			// Request execution of a file macro
	const char *GetRequestedMacroFile(bool& reportMissing) const;				// Return requested macro file or nullptr if none

	bool IsAbortRequested() const;						// Is the cancellation of the current file requested?
	void AcknowledgeAbort();							// Indicates that the current macro file is being cancelled

	void ReportStack() { reportStack = true; }			// Flags current stack details to be reported
	bool IsStackEventFlagged() const;					// Did the stack change?
	void AcknowledgeStackEvent();						// Indicates that the last stack event has been written
#endif

	GCodeState GetState() const;
	void SetState(GCodeState newState);
	void SetState(GCodeState newState, const char *err);
	void AdvanceState();
	void MessageAcknowledged(bool cancelled);

	const char *GetIdentity() const { return identity; }
	bool CanQueueCodes() const { return queueCodes; }
	MessageType GetResponseMessageType() const;

	int GetToolNumberAdjust() const { return toolNumberAdjust; }
	void SetToolNumberAdjust(int arg) { toolNumberAdjust = arg; }

	bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32);	// open a file to write to
	bool IsWritingFile() const;							// Returns true if writing a file
	void WriteToFile();									// Write the current GCode to file

	bool IsWritingBinary() const;						// Returns true if writing binary
	void WriteBinaryToFile(char b);						// Write a byte to the file
	void FinishWritingBinary();

	const char* DataStart() const;						// Get the start of the current command
	size_t DataLength() const;							// Get the length of the current command

	void PrintCommand(const StringRef& s) const;
	void AppendFullCommand(const StringRef &s) const;

	uint32_t whenTimerStarted;							// When we started waiting
	bool timerRunning;									// True if we are waiting

private:
	const char *identity;
	const MessageType responseMessageTypeString;		// The message type we use for responses to string codes coming from this channel
	const MessageType responseMessageTypeBinary;				// The message type we use for responses to binary codes coming from this channel
	const bool queueCodes;								// Can we queue certain G-codes from this source?

	int toolNumberAdjust;								// The adjustment to tool numbers in commands we receive

#if HAS_LINUX_INTERFACE
	char buffer[MaxCodeBufferSize];
#else
	char buffer[GCODE_LENGTH];
#endif

	bool isBinaryBuffer;
	BinaryParser binaryParser;
	StringParser stringParser;

	GCodeMachineState *machineState;					// Machine state for this gcode source

#if HAS_LINUX_INTERFACE
	String<MaxFilenameLength> requestedMacroFile;
	uint8_t
		reportMissingMacro : 1,
		abortFile : 1,
		reportStack : 1;
#endif
};

inline bool GCodeBuffer::IsDoingFile() const
{
#if HAS_HIGH_SPEED_SD
	return machineState->fileState.IsLive();
#elif HAS_LINUX_INTERFACE
	return machineState->fileId != 0;
#else
	return false;
#endif
}

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
