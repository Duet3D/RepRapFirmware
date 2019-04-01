/*
 * BinaryGCodeBuffer.h
 *
 *  Created on: 30 Mar 2019
 *      Author: Christian
 */

#ifndef SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_
#define SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_

#include "GCodeBuffer.h"
#include "Linux/MessageFormats.h"

class BinaryGCodeBuffer: public GCodeBuffer
{
public:
	BinaryGCodeBuffer(CodeChannel c, MessageType mt, bool useCodeQueue);
	void Init() override; 								// Set it up to parse another G-code
	void Diagnostics(MessageType mtype) override;		// Write some debug info
	bool Put(const char *data, size_t len) override;	// Add an entire code packet, overwriting any existing content
	void FileEnded() override;							// Called when we reach the end of the file we are reading from
	bool Seen(char c) override __attribute__((hot));	// Is a character present?

	char GetCommandLetter() const override;
	bool HasCommandNumber() const override;
	int GetCommandNumber() const override;
	int8_t GetCommandFraction() const override;

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

	bool IsIdle() const override { return isIdle; }
	bool IsCompletelyIdle() const override { return isIdle; }
	bool IsReady() const override { return isIdle && bufferLength != 0; }		// Return true if a gcode is ready but hasn't been started yet
	bool IsExecuting() const override { return !isIdle && bufferLength != 0; }	// Return true if a gcode has been started and is not paused
	void SetFinished(bool f) override;							// Set the G Code executed (or not)

	const char *GetIdentity() const;

	FilePosition GetFilePosition(size_t bytesCached) const override;	// Get the file position at the start of the current command
	bool IsFileFinished() const { return isFileFinished; }				// Returns true if the file has been finished

	const char* DataStart() const override { return reinterpret_cast<const char *>(buffer); }	// get the start of the current command
	size_t DataLength() const override { return bufferLength; }		// get the length of the current command

	void PrintCommand(const StringRef& s) const override;
	void AppendFullCommand(const StringRef &s) const override;

	void SetRequestedMacroFile(const char *filename, bool reportMissing);
	const char *GetRequestedMacroFile(bool& reportMissing) const;

	uint32_t whenTimerStarted;							// when we started waiting
	bool timerRunning;									// true if we are waiting

private:

	size_t AddPadding(size_t bytesRead) const;
	template<typename T> const void GetArray(T arr[], size_t& length, bool doPad) __attribute__((hot));
	void WriteParameters(const StringRef& s, bool quoteStrings) const;

	CodeChannel channel;

	uint32_t buffer[MaxCodeBufferSize / 4];
	size_t bufferLength;

	const CodeHeader * const header = reinterpret_cast<CodeHeader*>(buffer);
	const CodeParameter *seenParameter;
	const char *seenParameterValue;

	bool isIdle, isFileFinished;

	String<MaxFilenameLength> requestedMacroFile;
	bool reportMissingMacro;
};

inline void BinaryGCodeBuffer::FileEnded()
{
	isFileFinished = true;
}

inline void BinaryGCodeBuffer::SetRequestedMacroFile(const char *filename, bool reportMissing)
{
	isFileFinished = false;
	requestedMacroFile.copy(filename);
	reportMissingMacro = reportMissing;
}

inline const char *BinaryGCodeBuffer::GetRequestedMacroFile(bool& reportMissing) const
{
	reportMissing = reportMissingMacro;
	return requestedMacroFile.IsEmpty() ? nullptr : requestedMacroFile.c_str();
}

#endif /* SRC_GCODES_GCODEBUFFER_BINARYGCODEBUFFER_H_ */
