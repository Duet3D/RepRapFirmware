/*
 * GCodeBuffer.h
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H
#define SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H

#include "RepRapFirmware.h"
#include "GCodes/GCodeMachineState.h"
#include "MessageType.h"
#include "ObjectModel/ObjectModel.h"

// Class to hold an individual GCode and provide functions to allow it to be parsed
class GCodeBuffer
{
public:
	GCodeBuffer(MessageType mt, bool useCodeQueue);
	void Reset();										// Reset it to its state after start-up
	virtual void Init();								// Set it up to parse another G-code
	virtual void Diagnostics(MessageType mtype) = 0;	// Write some debug info
	virtual bool Put(const char *data, size_t len) = 0;	// Add an entire chunk of data, overwriting any existing content
	virtual void FileEnded() = 0;						// Called when we reach the end of the file we are reading from
	virtual bool Seen(char c) __attribute__((hot)) = 0;	// Is a character present?

	virtual char GetCommandLetter() const = 0;
	virtual bool HasCommandNumber() const = 0;
	virtual int GetCommandNumber() const = 0;
	virtual int8_t GetCommandFraction() const = 0;

	virtual float GetFValue() __attribute__((hot)) = 0;				// Get a float after a key letter
	virtual int32_t GetIValue() __attribute__((hot)) = 0;			// Get an integer after a key letter
	virtual uint32_t GetUIValue() = 0;								// Get an unsigned integer value
	virtual bool GetIPAddress(IPAddress& returnedIp) = 0;			// Get an IP address quad after a key letter
	virtual bool GetMacAddress(uint8_t mac[6]) = 0;					// Get a MAC address sextet after a key letter
	virtual bool GetUnprecedentedString(const StringRef& str) = 0;	// Get a string with no preceding key letter
	virtual bool GetQuotedString(const StringRef& str) = 0;			// Get and copy a quoted string
	virtual bool GetPossiblyQuotedString(const StringRef& str) = 0;	// Get and copy a string which may or may not be quoted
	virtual const void GetFloatArray(float arr[], size_t& length, bool doPad) __attribute__((hot)) = 0; // Get a colon-separated list of floats after a key letter
	virtual const void GetIntArray(int32_t arr[], size_t& length, bool doPad) = 0;			// Get a :-separated list of ints after a key letter
	virtual const void GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) = 0;	// Get a :-separated list of unsigned ints after a key letter

	bool TryGetFValue(char c, float& val, bool& seen);
	bool TryGetIValue(char c, int32_t& val, bool& seen);
	bool TryGetUIValue(char c, uint32_t& val, bool& seen);
	bool TryGetBValue(char c, bool& val, bool& seen);
	bool TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad = false);
	bool TryGetUIArray(char c, size_t numVals, uint32_t vals[], const StringRef& reply, bool& seen, bool doPad = false);
	bool TryGetQuotedString(char c, const StringRef& str, bool& seen);
	bool TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen);

	virtual bool IsIdle() const = 0;
	virtual bool IsCompletelyIdle() const = 0;
	virtual bool IsReady() const = 0;								// Return true if a gcode is ready but hasn't been started yet
	virtual bool IsExecuting() const = 0;							// Return true if a gcode has been started and is not paused
	virtual void SetFinished(bool f) = 0;							// Set the G Code executed (or not)
	int GetToolNumberAdjust() const { return toolNumberAdjust; }
	void SetToolNumberAdjust(int arg) { toolNumberAdjust = arg; }
	virtual void SetCommsProperties(uint32_t arg) { }
	MessageType GetResponseMessageType() const { return responseMessageType; }

	GCodeMachineState& MachineState() const { return *machineState; }
	GCodeMachineState& OriginalMachineState() const;
	float ConvertDistance(float distance) const;
	float InverseConvertDistance(float distance) const;
	bool PushState();									// Push state returning true if successful (i.e. stack not overflowed)
	bool PopState();									// Pop state returning true if successful (i.e. no stack underrun)

	bool IsDoingFileMacro() const;						// Return true if this source is executing a file macro
	GCodeState GetState() const;
	void SetState(GCodeState newState);
	void SetState(GCodeState newState, const char *err);
	void AdvanceState();
	virtual const char *GetIdentity() const = 0;
	bool CanQueueCodes() const;
	void MessageAcknowledged(bool cancelled);

	virtual bool OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) { return false; };	// open a file to write to
	virtual bool IsWritingFile() const { return false; }		// returns true if writing a file
	virtual void WriteToFile() { }								// write the current GCode to file

	virtual bool IsWritingBinary() const { return false; }		// returns true if writing binary
	virtual void WriteBinaryToFile(char b) { }					// write a byte to the file
	virtual void FinishWritingBinary() { }

	virtual FilePosition GetFilePosition(size_t bytesCached) const = 0;	// Get the file position at the start of the current command
	virtual bool IsFileFinished() const;				// Returns true if the file has been finished

	virtual const char* DataStart() const = 0;			// Get the start of the current command
	virtual size_t DataLength() const = 0;				// Get the length of the current command

	virtual void PrintCommand(const StringRef& s) const = 0;
	virtual void AppendFullCommand(const StringRef &s) const = 0;

	uint32_t whenTimerStarted;							// when we started waiting
	bool timerRunning;									// true if we are waiting

private:
	const MessageType responseMessageType;				// The message type we use for responses to commands coming from this channel
	const bool queueCodes;								// Can we queue certain G-codes from this source?

	int toolNumberAdjust;								// The adjustment to tool numbers in commands we receive

protected:

	GCodeMachineState *machineState;					// Machine state for this gcode source
};

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

inline bool GCodeBuffer::IsFileFinished() const
{
	return machineState->fileState.IsLive();
}

#endif /* SRC_GCODES_GCODEBUFFER_GCODEBUFFER_H */
