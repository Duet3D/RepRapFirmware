/*
 * BinaryGCodeBuffer.h
 *
 *  Created on: 30 Mar 2019
 *      Author: Christian
 */

#ifndef SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_
#define SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_

#include <RepRapFirmware.h>

#if HAS_SBC_INTERFACE

#include <SBC/SbcMessageFormats.h>
#include <GCodes/GCodeException.h>
#include <GCodes/GCodeMachineState.h>

class GCodeBuffer;
class IPAddress;
class MacAddress;

class BinaryParser
{
public:
	BinaryParser(GCodeBuffer& gcodeBuffer) noexcept;
	void Init() noexcept; 														// Set it up to parse another G-code
	void Put(const uint32_t *data, size_t len) noexcept;						// Add an entire binary code, overwriting any existing content
	void DecodeCommand() noexcept;												// Print the buffer content in debug mode and prepare for execution
	bool Seen(char c) noexcept SPEED_CRITICAL;									// Is a character present?
	bool SeenAny(Bitmap<uint32_t> bm) const noexcept;							// Return true if any of the parameter letters in the bitmap were seen

	char GetCommandLetter() const noexcept;
	bool HasCommandNumber() const noexcept;
	int GetCommandNumber() const noexcept;
	int8_t GetCommandFraction() const noexcept;
	bool ContainsExpression() const noexcept;

	float GetFValue() THROWS(GCodeException) SPEED_CRITICAL;					// Get a float after a key letter
	int32_t GetIValue() THROWS(GCodeException) SPEED_CRITICAL;					// Get an integer after a key letter
	uint32_t GetUIValue() THROWS(GCodeException);								// Get an unsigned integer value
	DriverId GetDriverId() THROWS(GCodeException);								// Get a driver ID
	void GetIPAddress(IPAddress& returnedIp) THROWS(GCodeException);			// Get an IP address quad after a key letter
	void GetMacAddress(MacAddress& mac) THROWS(GCodeException);					// Get a MAC address sextet after a key letter
	void GetUnprecedentedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException);	// Get a string with no preceding key letter
	void GetCompleteParameters(const StringRef& str) THROWS(GCodeException);					// Get the complete parameter string
	void GetQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException);			// Get and copy a quoted string
	void GetPossiblyQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException);	// Get and copy a string which may or may not be quoted
	void GetFloatArray(float arr[], size_t& length) THROWS(GCodeException) SPEED_CRITICAL;		// Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length) THROWS(GCodeException);						// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length) THROWS(GCodeException);				// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length) THROWS(GCodeException);				// Get a :-separated list of drivers after a key letter

	void SetFinished() noexcept;									// Set the G Code finished

	FilePosition GetFilePosition() const noexcept;					// Get the file position at the start of the current command

	const char* DataStart() const noexcept;							// Get the start of the current command
	size_t DataLength() const noexcept;								// Get the length of the current command

	void PrintCommand(const StringRef& s) const noexcept;
	void AppendFullCommand(const StringRef &s) const noexcept;
	void AddParameters(VariableSet& vs, int codeRunning) noexcept;

private:
	GCodeBuffer& gb;

	void CheckArrayLength(size_t maxLength) THROWS(GCodeException);
	void SetDriverIdFromBinary(DriverId& did, uint32_t val) THROWS(GCodeException);
	void SetDriverIdFromFloat(DriverId& did, float fval) THROWS(GCodeException);
	GCodeException ConstructParseException(const char *str) const noexcept;
	GCodeException ConstructParseException(const char *str, const char *param) const noexcept;
	GCodeException ConstructParseException(const char *str, uint32_t param) const noexcept;

	size_t AddPadding(size_t bytesRead) const noexcept { return (bytesRead + 3u) & (~3u); }
	template<typename T> void GetArray(T arr[], size_t& length) THROWS(GCodeException) SPEED_CRITICAL;
	void WriteParameters(const StringRef& s, bool quoteStrings) const noexcept;

	size_t bufferLength;
	const CodeHeader *header;

	int reducedBytesRead;
	const CodeParameter *seenParameter;
	const char *seenParameterValue;
};

// Get the complete parameter string. Used by the Q0 command to process comments.
inline void BinaryParser::GetCompleteParameters(const StringRef& str) THROWS(GCodeException)
{
	GetUnprecedentedString(str, true);
}

// Get a string, which must be enclosed in double quotes. DSF sorts out quotes, so we needn't worry about them.
inline void BinaryParser::GetQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
{
	GetPossiblyQuotedString(str, allowEmpty);
}

#endif

#endif /* SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_ */
