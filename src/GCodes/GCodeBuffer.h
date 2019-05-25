/*
 * GCodeBuffer.h
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

#ifndef GCODEBUFFER_H_
#define GCODEBUFFER_H_

#include "RepRapFirmware.h"
#include "GCodeMachineState.h"
#include "MessageType.h"
#include "ObjectModel/ObjectModel.h"

// Class to hold an individual GCode and provide functions to allow it to be parsed
class GCodeBuffer
{
public:
	GCodeBuffer(const char* id, MessageType mt, bool useCodeQueue);
	void Reset();										// Reset it to its state after start-up
	void Init(); 										// Set it up to parse another G-code
	void Diagnostics(MessageType mtype);				// Write some debug info
	bool Put(char c) __attribute__((hot));				// Add a character to the end
	void Put(const char *str, size_t len);				// Add an entire string, overwriting any existing content
	void Put(const char *str);							// Add a null-terminated string, overwriting any existing content
	void FileEnded();									// Called when we reach the end of the file we are reading from
	bool Seen(char c) __attribute__((hot));				// Is a character present?

	char GetCommandLetter() const { return commandLetter; }
	bool HasCommandNumber() const { return hasCommandNumber; }
	int GetCommandNumber() const { return commandNumber; }
	int8_t GetCommandFraction() const { return commandFraction; }

	float GetFValue() __attribute__((hot));				// Get a float after a key letter
	float GetDistance();								// Get a distance or coordinate and convert it from inches to mm if necessary
	int32_t GetIValue() __attribute__((hot));			// Get an integer after a key letter
	uint32_t GetUIValue();								// Get an unsigned integer value
	bool GetIPAddress(IPAddress& returnedIp);			// Get an IP address quad after a key letter
	bool GetMacAddress(uint8_t mac[6]);					// Get a MAC address sextet after a key letter
	bool GetUnprecedentedString(const StringRef& str);	// Get a string with no preceding key letter
	bool GetQuotedString(const StringRef& str);			// Get and copy a quoted string
	bool GetPossiblyQuotedString(const StringRef& str);	// Get and copy a string which may or may not be quoted
	void GetFloatArray(float arr[], size_t& length, bool doPad) __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length, bool doPad);		// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad);	// Get a :-separated list of unsigned ints after a key letter

	bool TryGetFValue(char c, float& val, bool& seen);
	bool TryGetIValue(char c, int32_t& val, bool& seen);
	bool TryGetUIValue(char c, uint32_t& val, bool& seen);
	bool TryGetBValue(char c, bool& val, bool& seen);
	bool TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad = false);
	bool TryGetUIArray(char c, size_t numVals, uint32_t vals[], const StringRef& reply, bool& seen, bool doPad = false);
	bool TryGetQuotedString(char c, const StringRef& str, bool& seen);
	bool TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen);

	const char* Buffer() const;
	bool IsIdle() const;
	bool IsCompletelyIdle() const;
	bool IsReady() const;								// Return true if a gcode is ready but hasn't been started yet
	bool IsExecuting() const;							// Return true if a gcode has been started and is not paused
	void SetFinished(bool f);							// Set the G Code executed (or not)
	int GetToolNumberAdjust() const { return toolNumberAdjust; }
	void SetToolNumberAdjust(int arg) { toolNumberAdjust = arg; }
	void SetCommsProperties(uint32_t arg) { checksumRequired = (arg & 1); }
	MessageType GetResponseMessageType() const { return responseMessageType; }

	GCodeMachineState& MachineState() const { return *machineState; }
	GCodeMachineState& OriginalMachineState() const;
	float ConvertDistance(float distance) const;
	float InverseConvertDistance(float distance) const;
	bool PushState();									// Push state returning true if successful (i.e. stack not overflowed)
	bool PopState();									// Pop state returning true if successful (i.e. no stack underrun)
	void AbortFile(FileGCodeInput* fileInput);			// Abort execution of any files or macros being executed

	bool IsDoingFileMacro() const;						// Return true if this source is executing a file macro
	GCodeState GetState() const;
	void SetState(GCodeState newState);
	void SetState(GCodeState newState, const char *err);
	void AdvanceState();
	const char *GetIdentity() const { return identity; }
	bool CanQueueCodes() const;
	void MessageAcknowledged(bool cancelled);
	FilePosition GetFilePosition(size_t bytesCached) const;	// Get the file position at the start of the current command

	bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32);	// open a file to write to
	bool IsWritingFile() const { return fileBeingWritten != nullptr; }		// returns true if writing a file
	void WriteToFile();														// write the current GCode to file

	bool IsWritingBinary() const { return IsWritingFile() && binaryWriting; }	// returns true if writing binary
	void WriteBinaryToFile(char b);											// write a byte to the file
	void FinishWritingBinary();

	size_t CommandLength() const { return commandEnd - commandStart; }		// get the length of the current command
	const char* CommandStart() const { return gcodeBuffer + commandStart; }	// get the start of the current command

	void PrintCommand(const StringRef& s) const;

	uint32_t whenTimerStarted;							// when we started waiting
	bool timerRunning;									// true if we are waiting

private:

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

	void AddToChecksum(char c);
	void StoreAndAddToChecksum(char c);
	bool LineFinished();								// Deal with receiving end-of-line and return true if we have a command
	void DecodeCommand();
	bool InternalGetQuotedString(const StringRef& str)
		pre (readPointer >= 0; gcodeBuffer[readPointer] == '"'; str.IsEmpty());
	bool InternalGetPossiblyQuotedString(const StringRef& str)
		pre (readPointer >= 0);
	float ReadFloatValue(const char *p, const char **endptr);
	uint32_t ReadUIValue(const char *p, const char **endptr);
	int32_t ReadIValue(const char *p, const char **endptr);

#if SUPPORT_OBJECT_MODEL
	bool GetStringExpression(const StringRef& str)
		pre (readPointer >= 0; gcodeBuffer[readPointer] == '['; str.IsEmpty());
	TypeCode EvaluateExpression(const char *p, const char **endptr, ExpressionValue& rslt)
		pre (readPointer >= 0; gcodeBuffer[readPointer] == '[');
#endif

	GCodeMachineState *machineState;					// Machine state for this gcode source
	const char* const identity;							// Where we are from (web, file, serial line etc)
	unsigned int commandStart;							// Index in the buffer of the command letter of this command
	unsigned int parameterStart;
	unsigned int commandEnd;							// Index in the buffer of one past the last character of this command
	unsigned int commandLength;							// Number of characters we read to build this command including the final \r or \n
	unsigned int gcodeLineEnd;							// Number of characters in the entire line of gcode
	int readPointer;									// Where in the buffer to read next
	GCodeBufferState bufferState;						// Idle, executing or paused

	FileStore *fileBeingWritten;						// If we are copying GCodes to a file, which file it is
	FilePosition writingFileSize;						// Size of the file being written, or zero if not known
	uint8_t eofStringCounter;							// Check the...

	int toolNumberAdjust;								// The adjustment to tool numbers in commands we receive
	const MessageType responseMessageType;				// The message type we use for responses to commands coming from this channel
	unsigned int lineNumber;
	unsigned int declaredChecksum;
	int commandNumber;
	uint32_t crc32;										// crc32 of the binary file

	uint8_t computedChecksum;
	bool hadLineNumber;
	bool hadChecksum;
	bool hasCommandNumber;
	char commandLetter;

	char gcodeBuffer[GCODE_LENGTH];						// The G Code
	bool checksumRequired;								// True if we only accept commands with a valid checksum
	int8_t commandFraction;

	bool queueCodes;									// Can we queue certain G-codes from this source?
	bool binaryWriting;									// Executing gcode or writing binary file?
};

inline const char* GCodeBuffer::Buffer() const
{
	return gcodeBuffer;
}

inline bool GCodeBuffer::IsIdle() const
{
	return bufferState != GCodeBufferState::ready && bufferState != GCodeBufferState::executing;
}

inline bool GCodeBuffer::IsCompletelyIdle() const
{
	return GetState() == GCodeState::normal && IsIdle();
}

inline bool GCodeBuffer::IsReady() const
{
	return bufferState == GCodeBufferState::ready;
}

inline bool GCodeBuffer::IsExecuting() const
{
	return bufferState == GCodeBufferState::executing;
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

#endif /* GCODEBUFFER_H_ */
