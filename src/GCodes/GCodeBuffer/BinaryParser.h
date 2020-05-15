/*
 * BinaryGCodeBuffer.h
 *
 *  Created on: 30 Mar 2019
 *      Author: Christian
 */

#ifndef SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_
#define SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_

#include <RepRapFirmware.h>

#if HAS_LINUX_INTERFACE

#include <Linux/LinuxMessageFormats.h>
#include <MessageType.h>
#include <GCodes/GCodeException.h>

class GCodeBuffer;
class IPAddress;
class MacAddress;

class BinaryParser
{
public:
	BinaryParser(GCodeBuffer& gcodeBuffer) noexcept;
	void Init() noexcept; 											// Set it up to parse another G-code
	void Put(const char *data, size_t len) noexcept;				// Add an entire string, overwriting any existing content
	bool Seen(char c) noexcept __attribute__((hot));				// Is a character present?

	char GetCommandLetter() const noexcept;
	bool HasCommandNumber() const noexcept;
	int GetCommandNumber() const noexcept;
	int8_t GetCommandFraction() const noexcept;

	float GetFValue() THROWS(GCodeException) __attribute__((hot));				// Get a float after a key letter
	int32_t GetIValue() THROWS(GCodeException) __attribute__((hot));			// Get an integer after a key letter
	uint32_t GetUIValue() THROWS(GCodeException);								// Get an unsigned integer value
	DriverId GetDriverId() THROWS(GCodeException);								// Get a driver ID
	void GetIPAddress(IPAddress& returnedIp) THROWS(GCodeException);			// Get an IP address quad after a key letter
	void GetMacAddress(MacAddress& mac) THROWS(GCodeException);					// Get a MAC address sextet after a key letter
	void GetUnprecedentedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException);	// Get a string with no preceding key letter
	void GetCompleteParameters(const StringRef& str) THROWS(GCodeException);	// Get the complete parameter string
	void GetQuotedString(const StringRef& str) THROWS(GCodeException);			// Get and copy a quoted string
	void GetPossiblyQuotedString(const StringRef& str, bool allowEmpty = false) THROWS(GCodeException);	// Get and copy a string which may or may not be quoted
	void GetReducedString(const StringRef& str) THROWS(GCodeException);			// Get and copy a quoted string, removing certain characters
	void GetFloatArray(float arr[], size_t& length, bool doPad) THROWS(GCodeException) __attribute__((hot)); // Get a colon-separated list of floats after a key letter
	void GetIntArray(int32_t arr[], size_t& length, bool doPad) THROWS(GCodeException);		// Get a :-separated list of ints after a key letter
	void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) THROWS(GCodeException);	// Get a :-separated list of unsigned ints after a key letter
	void GetDriverIdArray(DriverId arr[], size_t& length) THROWS(GCodeException);	// Get a :-separated list of drivers after a key letter

	void SetFinished() noexcept;									// Set the G Code finished

	FilePosition GetFilePosition() const noexcept;					// Get the file position at the start of the current command

	const char* DataStart() const noexcept;							// Get the start of the current command
	size_t DataLength() const noexcept;								// Get the length of the current command

	void PrintCommand(const StringRef& s) const noexcept;
	void AppendFullCommand(const StringRef &s) const noexcept;

private:
	GCodeBuffer& gb;

	void CheckArrayLength(size_t maxLength) THROWS(GCodeException);
	GCodeException ConstructParseException(const char *str) const noexcept;
	GCodeException ConstructParseException(const char *str, const char *param) const noexcept;
	GCodeException ConstructParseException(const char *str, uint32_t param) const noexcept;

	size_t AddPadding(size_t bytesRead) const noexcept { return (bytesRead + 3u) & (~3u); }
	template<typename T> void GetArray(T arr[], size_t& length, bool doPad) THROWS(GCodeException) __attribute__((hot));
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
inline void BinaryParser::GetQuotedString(const StringRef& str) THROWS(GCodeException)
{
	GetPossiblyQuotedString(str);
}

#endif

#endif /* SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_ */
