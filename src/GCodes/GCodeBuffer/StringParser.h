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
#include "GCodes/GCodeMachineState.h"
#include "MessageType.h"
#include "ObjectModel/ObjectModel.h"
#include "ParseException.h"

class GCodeBuffer;
class IPAddress;

class StringParser
{
public:
	StringParser(GCodeBuffer& gcodeBuffer) noexcept;
	void Init() noexcept; 								// Set it up to parse another G-code
	void Diagnostics(MessageType mtype) noexcept;		// Write some debug info
	bool Put(char c) __attribute__((hot));				// Add a character to the end
	void Put(const char *str, size_t len);				// Add an entire string, overwriting any existing content
	void Put(const char *str);							// Add a null-terminated string, overwriting any existing content
	void FileEnded();									// Called when we reach the end of the file we are reading from
	bool Seen(char c) noexcept __attribute__((hot));	// Is a character present?

	char GetCommandLetter() const noexcept { return commandLetter; }
	bool HasCommandNumber() const noexcept { return hasCommandNumber; }
	int GetCommandNumber() const noexcept { return commandNumber; }
	int8_t GetCommandFraction() const noexcept { return commandFraction; }

	float GetFValue() THROWS_PARSE_ERROR __attribute__((hot));				// Get a float after a key letter
	float GetDistance() THROWS_PARSE_ERROR;									// Get a distance or coordinate and convert it from inches to mm if necessary
	int32_t GetIValue() THROWS_PARSE_ERROR __attribute__((hot));			// Get an integer after a key letter
	uint32_t GetUIValue() THROWS_PARSE_ERROR;								// Get an unsigned integer value
	DriverId GetDriverId() THROWS_PARSE_ERROR;								// Get a driver ID
	void GetIPAddress(IPAddress& returnedIp) THROWS_PARSE_ERROR;			// Get an IP address quad after a key letter
	void GetMacAddress(uint8_t mac[6]) THROWS_PARSE_ERROR;					// Get a MAC address sextet after a key letter
	void GetUnprecedentedString(const StringRef& str, bool allowEmpty) THROWS_PARSE_ERROR;	// Get a string with no preceding key letter
	void GetQuotedString(const StringRef& str) THROWS_PARSE_ERROR;			// Get and copy a quoted string
	void GetPossiblyQuotedString(const StringRef& str) THROWS_PARSE_ERROR;	// Get and copy a string which may or may not be quoted
	void GetReducedString(const StringRef& str) THROWS_PARSE_ERROR;			// Get and copy a quoted string, removing certain characters
	void GetFloatArray(float arr[], size_t& length, bool doPad) THROWS_PARSE_ERROR __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length, bool doPad) THROWS_PARSE_ERROR;		// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) THROWS_PARSE_ERROR;	// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length) THROWS_PARSE_ERROR;	// Get a :-separated list of drivers after a key letter

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

	FilePosition GetFilePosition() const noexcept;				// Get the file position at the start of the current command

	const char* DataStart() const noexcept;						// Get the start of the current command
	size_t DataLength() const noexcept;							// Get the length of the current command

	void PrintCommand(const StringRef& s) const noexcept;
	void AppendFullCommand(const StringRef &s) const noexcept;

	ParseException ConstructParseException(const char *str) const;
	ParseException ConstructParseException(const char *str, const char *param) const;
	ParseException ConstructParseException(const char *str, uint32_t param) const;

private:
	GCodeBuffer& gb;

	void AddToChecksum(char c) noexcept;
	void StoreAndAddToChecksum(char c);
	bool LineFinished();								// Deal with receiving end-of-line and return true if we have a command
	void DecodeCommand();
	void InternalGetQuotedString(const StringRef& str) THROWS_PARSE_ERROR
		pre (readPointer >= 0; gb.buffer[readPointer] == '"'; str.IsEmpty());
	void InternalGetPossiblyQuotedString(const StringRef& str, bool allowEmpty) THROWS_PARSE_ERROR
		pre (readPointer >= 0);
	float ReadFloatValue() THROWS_PARSE_ERROR;
	uint32_t ReadUIValue() THROWS_PARSE_ERROR;
	int32_t ReadIValue() THROWS_PARSE_ERROR;
	DriverId ReadDriverIdValue() THROWS_PARSE_ERROR;
	void GetStringExpression(const StringRef& str) THROWS_PARSE_ERROR
		pre (readPointer >= 0; gb.buffer[readPointer] == '{'; str.IsEmpty());

	bool ProcessConditionalGCode(BlockType previousBlockType) THROWS_PARSE_ERROR;
														// Check for and process a conditional GCode language command returning true if we found one
	void CreateBlocks();								// Create new code blocks
	bool EndBlocks();									// End blocks returning true if nothing more to process on this line
	void ProcessIfCommand() THROWS_PARSE_ERROR;
	void ProcessElseCommand(BlockType previousBlockType);
	void ProcessElifCommand(BlockType previousBlockType) THROWS_PARSE_ERROR;
	void ProcessWhileCommand() THROWS_PARSE_ERROR;
	void ProcessBreakCommand();
	void ProcessVarCommand() THROWS_PARSE_ERROR;
	void ProcessSetCommand() THROWS_PARSE_ERROR;
	void ProcessAbortCommand();

	bool EvaluateCondition() THROWS_PARSE_ERROR;

	ExpressionValue ParseBracketedExpression(char closingBracket) THROWS_PARSE_ERROR
		pre (readPointer >= 0; gb.buffer[readPointer] == '{');
	ExpressionValue ParseExpression(uint8_t priority) THROWS_PARSE_ERROR
		pre (readPointer >= 0);
	ExpressionValue ParseNumber() THROWS_PARSE_ERROR
		pre(readPointer >= 0; isdigit(gb.buffer[readPointer]));
	ExpressionValue ParseIdentifier() THROWS_PARSE_ERROR
		pre(readPointer >= 0; isalpha(gb.buffer[readPointer]));

	void BalanceNumericTypes(ExpressionValue& val1, ExpressionValue& val2) THROWS_PARSE_ERROR;
	void BalanceTypes(ExpressionValue& val1, ExpressionValue& val2) THROWS_PARSE_ERROR;
	void ConvertToFloat(ExpressionValue& val) THROWS_PARSE_ERROR;
	void ConvertToBool(ExpressionValue& val) THROWS_PARSE_ERROR;
	void EnsureNumeric(ExpressionValue& val) THROWS_PARSE_ERROR;

	void SkipWhiteSpace();

	unsigned int commandStart;							// Index in the buffer of the command letter of this command
	unsigned int parameterStart;
	unsigned int commandEnd;							// Index in the buffer of one past the last character of this command
	unsigned int commandLength;							// Number of characters we read to build this command including the final \r or \n
	unsigned int braceCount;							// how many nested { } we are inside
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
