/*
 * StringParser.h
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H
#define SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H

#include "RepRapFirmware.h"
#include "GCodes/GCodeInput.h"
#include "MessageType.h"
#include "ObjectModel/ObjectModel.h"

class GCodeBuffer;
class IPAddress;

class StringParser
{
public:
	StringParser(GCodeBuffer& gcodeBuffer);
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
	DriverId GetDriverId();								// Get a driver ID
	bool GetIPAddress(IPAddress& returnedIp);			// Get an IP address quad after a key letter
	bool GetMacAddress(uint8_t mac[6]);					// Get a MAC address sextet after a key letter
	bool GetUnprecedentedString(const StringRef& str);	// Get a string with no preceding key letter
	bool GetQuotedString(const StringRef& str);			// Get and copy a quoted string
	bool GetPossiblyQuotedString(const StringRef& str);	// Get and copy a string which may or may not be quoted
	bool GetReducedString(const StringRef& str);		// Get and copy a quoted string, removing certain characters
	void GetFloatArray(float arr[], size_t& length, bool doPad) __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length, bool doPad);		// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad);	// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length);	// Get a :-separated list of drivers after a key letter

	void SetFinished();									// Set the G Code finished
	void SetCommsProperties(uint32_t arg) { checksumRequired = (arg & 1); }

#if HAS_MASS_STORAGE
	bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32);	// Open a file to write to
	bool IsWritingFile() const { return fileBeingWritten != nullptr; }	// Returns true if writing a file
	void WriteToFile();													// Write the current GCode to file

	bool IsWritingBinary() const { return IsWritingFile() && binaryWriting; }	// Returns true if writing binary
	void WriteBinaryToFile(char b);												// Write a byte to the file
	void FinishWritingBinary();
#endif

	FilePosition GetFilePosition() const;				// Get the file position at the start of the current command

	const char* DataStart() const;						// Get the start of the current command
	size_t DataLength() const;							// Get the length of the current command

	void PrintCommand(const StringRef& s) const;
	void AppendFullCommand(const StringRef &s) const;

private:

	GCodeBuffer& gb;

	void AddToChecksum(char c);
	void StoreAndAddToChecksum(char c);
	bool LineFinished();								// Deal with receiving end-of-line and return true if we have a command
	void DecodeCommand();
	bool InternalGetQuotedString(const StringRef& str)
		pre (readPointer >= 0; gb.buffer[readPointer] == '"'; str.IsEmpty());
	bool InternalGetPossiblyQuotedString(const StringRef& str)
		pre (readPointer >= 0);
	float ReadFloatValue(const char *p, const char **endptr);
	uint32_t ReadUIValue(const char *p, const char **endptr);
	int32_t ReadIValue(const char *p, const char **endptr);
	DriverId ReadDriverIdValue(const char *p, const char **endptr);

	bool ProcessConditionalGCode(bool skippedIfFalse);	// Check for and process a conditional GCode language command returning true if we found one
	void CreateBlocks();								// Create new code blocks
	bool EndBlocks();									// End blocks returning true if nothing more to process on this line
	void ProcessIfCommand();
	void ProcessElseCommand(bool skippedIfFalse);
	void ProcessWhileCommand();
	void ProcessBreakCommand();
	void ProcessVarCommand();
	bool EvaluateCondition(const char* keyword);

#if SUPPORT_OBJECT_MODEL
	bool GetStringExpression(const StringRef& str)
		pre (readPointer >= 0; gb.buffer[readPointer] == '{'; str.IsEmpty());
	TypeCode EvaluateExpression(const char *p, const char **endptr, ExpressionValue& rslt)
		pre (readPointer >= 0; gb.buffer[readPointer] == '{');
#endif

	unsigned int commandStart;							// Index in the buffer of the command letter of this command
	unsigned int parameterStart;
	unsigned int commandEnd;							// Index in the buffer of one past the last character of this command
	unsigned int commandLength;							// Number of characters we read to build this command including the final \r or \n
	unsigned int gcodeLineEnd;							// Number of characters in the entire line of gcode
	int readPointer;									// Where in the buffer to read next, or -1

	FileStore *fileBeingWritten;						// If we are copying GCodes to a file, which file it is
	FilePosition writingFileSize;						// Size of the file being written, or zero if not known

	unsigned int receivedLineNumber;
	unsigned int declaredChecksum;
	int commandNumber;
	uint32_t crc32;										// crc32 of the binary file
	uint32_t whenTimerStarted;							// when we started waiting

	uint8_t eofStringCounter;							// Check the EOF

	uint16_t indentToSkipTo;
	static constexpr uint16_t NoIndentSkip = 0xFFFF;	// must be greater than any real indent

	uint8_t computedChecksum;
	bool hadLineNumber;
	bool hadChecksum;
	bool hasCommandNumber;
	char commandLetter;
	uint8_t commandIndent;								// Number of whitespace characters before the line number or the first command starts

	bool checksumRequired;								// True if we only accept commands with a valid checksum
	int8_t commandFraction;

	bool binaryWriting;									// Executing gcode or writing binary file?
};

#endif /* SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H */
