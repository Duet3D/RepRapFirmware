/*
 * StringGCodeBuffer.h
 *
 *  Created on: 30 Mar 2019
 *      Authors: David and Christian
 */

#ifndef SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H
#define SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H

#include "GCodeBuffer.h"

#include "RepRapFirmware.h"
#include "GCodes/GCodeMachineState.h"
#include "MessageType.h"
#include "ObjectModel/ObjectModel.h"

// Class to hold an individual GCode and provide functions to allow it to be parsed
class StringGCodeBuffer : public GCodeBuffer
{
public:
	StringGCodeBuffer(const char* id, MessageType mt, bool useCodeQueue);
	void Init() override; 								// Set it up to parse another G-code
	void Diagnostics(MessageType mtype) override;		// Write some debug info
	bool Put(char c) __attribute__((hot));				// Add a character to the end
	bool Put(const char *data, size_t len) override;	// Add an entire string, overwriting any existing content
	void Put(const char *data);							// Add a null-terminated string, overwriting any existing content
	void FileEnded() override;							// Called when we reach the end of the file we are reading from
	bool Seen(char c) override __attribute__((hot));	// Is a character present?

	char GetCommandLetter() const override { return commandLetter; }
	bool HasCommandNumber() const override { return hasCommandNumber; }
	int GetCommandNumber() const override { return commandNumber; }
	int8_t GetCommandFraction() const override { return commandFraction; }

	float GetFValue() override __attribute__((hot));			// Get a float after a key letter
	int32_t GetIValue() override __attribute__((hot));			// Get an integer after a key letter
	uint32_t GetUIValue() override;								// Get an unsigned integer value
	bool GetIPAddress(IPAddress& returnedIp) override;			// Get an IP address quad after a key letter
	bool GetMacAddress(uint8_t mac[6]) override;				// Get a MAC address sextet after a key letter
	bool GetUnprecedentedString(const StringRef& str) override;	// Get a string with no preceding key letter
	bool GetQuotedString(const StringRef& str) override;		// Get and copy a quoted string
	bool GetPossiblyQuotedString(const StringRef& str) override;	// Get and copy a string which may or may not be quoted
	const void GetFloatArray(float arr[], size_t& length, bool doPad) override __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	const void GetIntArray(int32_t arr[], size_t& length, bool doPad) override;			// Get a :-separated list of ints after a key letter
	const void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) override;	// Get a :-separated list of unsigned ints after a key letter

	bool IsIdle() const override;
	bool IsCompletelyIdle() const override;
	bool IsReady() const override;								// Return true if a gcode is ready but hasn't been started yet
	bool IsExecuting() const override;							// Return true if a gcode has been started and is not paused
	void SetFinished(bool f) override;							// Set the G Code executed (or not)
	void SetCommsProperties(uint32_t arg) override { checksumRequired = (arg & 1); }

	const char *GetIdentity() const override { return identity; }

	bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) override;	// open a file to write to
	bool IsWritingFile() const override { return fileBeingWritten != nullptr; }		// returns true if writing a file
	void WriteToFile() override;													// write the current GCode to file

	bool IsWritingBinary() const { return IsWritingFile() && binaryWriting; }		// returns true if writing binary
	void WriteBinaryToFile(char b);													// write a byte to the file
	void FinishWritingBinary();

	FilePosition GetFilePosition(size_t bytesCached) const override;	// Get the file position at the start of the current command

	const char* DataStart() const override { return gcodeBuffer + commandStart; }	// get the start of the current command
	size_t DataLength() const override { return commandEnd - commandStart; }		// get the length of the current command

	void PrintCommand(const StringRef& s) const override;
	void AppendFullCommand(const StringRef &s) const override;

private:

	const char* const identity;							// Where we are from (web, file, serial line etc)

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

	bool binaryWriting;									// Executing gcode or writing binary file?
};

inline bool StringGCodeBuffer::IsIdle() const
{
	return bufferState != GCodeBufferState::ready && bufferState != GCodeBufferState::executing;
}

inline bool StringGCodeBuffer::IsCompletelyIdle() const
{
	return GetState() == GCodeState::normal && IsIdle();
}

inline bool StringGCodeBuffer::IsReady() const
{
	return bufferState == GCodeBufferState::ready;
}

inline bool StringGCodeBuffer::IsExecuting() const
{
	return bufferState == GCodeBufferState::executing;
}

#endif /* SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H */
