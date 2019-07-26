/*
 * BinaryGCodeBuffer.h
 *
 *  Created on: 30 Mar 2019
 *      Author: Christian
 */

#ifndef SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_
#define SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_

#include "Linux/MessageFormats.h"
#include "MessageType.h"
#include "RepRapFirmware.h"

class GCodeBuffer;
class IPAddress;

class BinaryParser
{
public:
	BinaryParser(GCodeBuffer& gcodeBuffer);
	void Init(); 								// Set it up to parse another G-code
	void Diagnostics(MessageType mtype);		// Write some debug info
	void Put(const char *data, size_t len);		// Add an entire string, overwriting any existing content
	bool Seen(char c) __attribute__((hot));		// Is a character present?

	char GetCommandLetter() const;
	bool HasCommandNumber() const;
	int GetCommandNumber() const;
	int8_t GetCommandFraction() const;

	float GetFValue() __attribute__((hot));				// Get a float after a key letter
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

	bool IsIdle() const { return isIdle; }
	bool IsCompletelyIdle() const { return isIdle && bufferLength == 0; }
	bool IsReady() const { return isIdle && bufferLength != 0; }		// Return true if a gcode is ready but hasn't been started yet
	bool IsExecuting() const { return !isIdle && bufferLength != 0; }	// Return true if a gcode has been started and is not paused
	void SetFinished(bool f);							// Set the G Code executed (or not)

	FilePosition GetFilePosition() const;				// Get the file position at the start of the current command

	const char* DataStart() const;						// Get the start of the current command
	size_t DataLength() const;							// Get the length of the current command

	void PrintCommand(const StringRef& s) const;
	void AppendFullCommand(const StringRef &s) const;

private:

	GCodeBuffer& gb;

	size_t AddPadding(size_t bytesRead) const;
	template<typename T> void GetArray(T arr[], size_t& length, bool doPad) __attribute__((hot));
	void WriteParameters(const StringRef& s, bool quoteStrings) const;

	size_t bufferLength;
	const CodeHeader *header;

	int reducedBytesRead;
	const CodeParameter *seenParameter;
	const char *seenParameterValue;

	bool isIdle;
};

#endif /* SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_ */
