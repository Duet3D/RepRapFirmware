/*
 * StringParser.h
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H
#define SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H

#include <RepRapFirmware.h>
#include <GCodes/GCodeInput.h>
#include <GCodes/GCodeMachineState.h>
#include <ObjectModel/ObjectModel.h>
#include <GCodes/GCodeException.h>
#include <Networking/NetworkDefs.h>
#include <Storage/CRC16.h>

class GCodeBuffer;
class IPAddress;
class MacAddress;
class VariableSet;

class StringParser
{
public:
	StringParser(GCodeBuffer& gcodeBuffer) noexcept;
	void Init() noexcept; 													// Set it up to parse another G-code
	void Diagnostics(MessageType mtype) noexcept;							// Write some debug info
	bool Put(char c) noexcept SPEED_CRITICAL;				// Add a character to the end
	void PutCommand(const char *str) noexcept;								// Put a complete command but don't decode it
	void DecodeCommand() noexcept;											// Decode the next command in the line
	void PutAndDecode(const char *str, size_t len) noexcept;				// Add an entire string, overwriting any existing content
	void PutAndDecode(const char *str) noexcept;							// Add a null-terminated string, overwriting any existing content
	void StartNewFile() noexcept;											// Called when we start a new file
	bool FileEnded() noexcept;												// Called when we reach the end of the file we are reading from
	bool CheckMetaCommand(const StringRef& reply) THROWS(GCodeException);	// Check whether the current command is a meta command, or we are skipping block

	// The following may be called after calling DecodeCommand
	char GetCommandLetter() const noexcept { return commandLetter; }
	bool HasCommandNumber() const noexcept { return hasCommandNumber; }
	int GetCommandNumber() const noexcept { return commandNumber; }
	int8_t GetCommandFraction() const noexcept { return commandFraction; }
	bool IsLastCommand() const noexcept;
	bool ContainsExpression() const noexcept { return seenExpression; }

	bool Seen(char c) noexcept SPEED_CRITICAL;									// Is a character present?
	bool SeenAny(Bitmap<uint32_t> bm) const noexcept;							// Return true if any of the parameter letters in the bitmap were seen
	float GetFValue() THROWS(GCodeException) SPEED_CRITICAL;					// Get a float after a key letter
	float GetDistance() THROWS(GCodeException) SPEED_CRITICAL;					// Get a distance or coordinate and convert it from inches to mm if necessary
	int32_t GetIValue() THROWS(GCodeException) SPEED_CRITICAL;					// Get an integer after a key letter
	uint32_t GetUIValue() THROWS(GCodeException);								// Get an unsigned integer value
	DriverId GetDriverId() THROWS(GCodeException);								// Get a driver ID
	void GetIPAddress(IPAddress& returnedIp) THROWS(GCodeException);			// Get an IP address quad after a key letter
	void GetMacAddress(MacAddress& mac) THROWS(GCodeException);					// Get a MAC address sextet after a key letter
	void GetUnprecedentedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException);	// Get a string with no preceding key letter
	void GetCompleteParameters(const StringRef& str) const noexcept;							// Get the complete parameter string
	void GetQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException);			// Get and copy a quoted string
	void GetPossiblyQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException);	// Get and copy a string which may or may not be quoted
	void GetFloatArray(float arr[], size_t& length) THROWS(GCodeException) SPEED_CRITICAL; 		// Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length) THROWS(GCodeException);						// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length) THROWS(GCodeException);				// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length) THROWS(GCodeException);				// Get a :-separated list of drivers after a key letter

	void ResetIndentation() noexcept;										// Reset the indentation level to the last one
	void SetFinished() noexcept;											// Set the G Code finished
	void SetCommsProperties(uint32_t arg) noexcept { checksumRequired = (arg & 1); crcRequired = (arg & 4); }

#if HAS_MASS_STORAGE
	bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) noexcept;
																			// Open a file to write to
	bool IsWritingFile() const noexcept { return fileBeingWritten != nullptr; }	// Returns true if writing a file
	void WriteToFile() noexcept;											// Write the current GCode to file

	bool IsWritingBinary() const noexcept { return IsWritingFile() && binaryWriting; }	// Returns true if writing binary
	bool WriteBinaryToFile(char b) noexcept;								// Write a byte to the file
	void FinishWritingBinary() noexcept;
#endif

	FilePosition GetFilePosition() const noexcept;							// Get the file position at the start of the current command

	const char* DataStart() const noexcept;									// Get the start of the current command
	size_t DataLength() const noexcept;										// Get the length of the current command

	void PrintCommand(const StringRef& s) const noexcept;
	void AppendFullCommand(const StringRef &s) const noexcept;
	void AddParameters(VariableSet& vs, int codeRunning) noexcept;

	GCodeException ConstructParseException(const char *str) const noexcept;
	GCodeException ConstructParseException(const char *str, const char *param) const noexcept;
	GCodeException ConstructParseException(const char *str, uint32_t param) const noexcept;

private:
	GCodeBuffer& gb;

	void AddToChecksum(char c) noexcept;
	void StoreAndAddToChecksum(char c) noexcept;
	bool LineFinished() noexcept;											// Deal with receiving end-of-line and return true if we have a command
	void InternalGetQuotedString(const StringRef& str) THROWS(GCodeException)
		pre (readPointer >= 0; gb.buffer[readPointer] == '"'; str.IsEmpty());
	void InternalGetPossiblyQuotedString(const StringRef& str) THROWS(GCodeException)
		pre (readPointer >= 0);
	float ReadFloatValue() THROWS(GCodeException);
	uint32_t ReadUIValue() THROWS(GCodeException);
	int32_t ReadIValue() THROWS(GCodeException);
	DriverId ReadDriverIdValue() THROWS(GCodeException);
	void CheckArrayLength(size_t actualLength, size_t maxLength) THROWS(GCodeException);
	void CheckNumberFound(const char *endptr) THROWS(GCodeException);

	void CheckForMixedSpacesAndTabs() noexcept;
	bool ProcessConditionalGCode(const StringRef& reply, BlockType skippedBlockType, bool doingFile) THROWS(GCodeException);
																			// Check for and process a conditional GCode language command returning true if we found one
	void ProcessIfCommand() THROWS(GCodeException);
	void ProcessElseCommand(BlockType skippedBlockType) THROWS(GCodeException);
	void ProcessElifCommand(BlockType skippedBlockType) THROWS(GCodeException);
	void ProcessWhileCommand() THROWS(GCodeException);
	void ProcessBreakCommand() THROWS(GCodeException);
	void ProcessContinueCommand() THROWS(GCodeException);
	void ProcessVarOrGlobalCommand(bool isGlobal) THROWS(GCodeException);
	void ProcessSetCommand() THROWS(GCodeException);
	void ProcessAbortCommand(const StringRef& reply) noexcept;
	void ProcessEchoCommand(const StringRef& reply) THROWS(GCodeException);

	bool EvaluateCondition() THROWS(GCodeException);

	void SkipWhiteSpace() noexcept;
	void FindParameters() noexcept;

	unsigned int commandStart;							// Index in the buffer of the command letter of this command
	unsigned int parameterStart;
	unsigned int commandEnd;							// Index in the buffer of one past the last character of this command
	unsigned int commandLength;							// Number of characters we read to build this command including the final \r or \n
	unsigned int braceCount;							// how many nested { } we are inside
	unsigned int gcodeLineEnd;							// Number of characters in the entire line of gcode
	Bitmap<uint32_t> parametersPresent;					// which parameters are present in this command
	int readPointer;									// Where in the buffer to read next, or -1

	FileStore *fileBeingWritten;						// If we are copying GCodes to a file, which file it is
	FilePosition writingFileSize;						// Size of the file being written, or zero if not known

	unsigned int receivedLineNumber;
	unsigned int declaredChecksum;
	int commandNumber;
	uint32_t crc32;										// crc32 of the binary file
	uint32_t whenTimerStarted;							// when we started waiting

	uint16_t indentToSkipTo;
	static constexpr uint16_t NoIndentSkip = 0xFFFF;	// must be greater than any real indent

	CRC16 crc16;										// CRC of the characters received

	uint8_t computedChecksum;							// this is the computed checksum or CRC
	uint8_t checksumCharsReceived;						// the number of checksum characters received
	uint8_t eofStringCounter;							// Check the EOF

	bool hadLineNumber;
	bool hadChecksum;
	bool hasCommandNumber;
	char commandLetter;
	uint8_t commandIndent;								// Number of whitespace characters before the line number or the first command starts
	bool seenLeadingSpace;
	bool seenLeadingTab;
	bool seenMetaCommand;
	bool warnedAboutMixedSpacesAndTabs;
	bool overflowed;
	bool seenExpression;

	bool checksumRequired;								// True if we only accept commands with a valid checksum
	bool crcRequired;									// True if we only accept commands with a valid CRC, except for M409 commands
	int8_t commandFraction;

	bool binaryWriting;									// Executing gcode or writing binary file?
};

#endif /* SRC_GCODES_GCODEBUFFER_STRINGGCODEBUFFER_H */
