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

// Class to hold an individual GCode and provide functions to allow it to be parsed
class GCodeBuffer
{
public:
	GCodeBuffer(const char* id, MessageType mt, bool useCodeQueue);
	void Reset();										// Reset it to its state after start-up
	void Init(); 										// Set it up to parse another G-code
	void Diagnostics(MessageType mtype);				// Write some debug info
	bool Put(char c);									// Add a character to the end
	bool Put(const char *str, size_t len);				// Add an entire string
	bool IsEmpty() const;								// Does this buffer contain any code?
	bool Seen(char c);									// Is a character present?

	char GetCommandLetter();							// Find the first G, M or T command
	float GetFValue();									// Get a float after a key letter
	int32_t GetIValue();								// Get an integer after a key letter
	bool GetIPAddress(uint8_t ip[4]);					// Get an IP address quad after a key letter
	bool GetIPAddress(uint32_t& ip);					// Get an IP address quad after a key letter
	const char* GetUnprecedentedString(bool optional = false);	// Get a string with no preceding key letter
	const char* GetString();							// Get a string after a key letter
	bool GetQuotedString(char *buf, size_t buflen);		// Get and copy a quoted string
	const void GetFloatArray(float a[], size_t& length, bool doPad); // Get a :-separated list of floats after a key letter
	const void GetLongArray(long l[], size_t& length);	// Get a :-separated list of longs after a key letter

	void TryGetFValue(char c, float& val, bool& seen);
	void TryGetIValue(char c, int32_t& val, bool& seen);
	bool TryGetFloatArray(char c, size_t numVals, float vals[], StringRef& reply, bool& seen);
	void TryGetQuotedString(char c, char *buf, size_t buflen, bool& seen);

	const char* Buffer() const;
	bool IsIdle() const;
	bool IsReady() const;								// Return true if a gcode is ready but hasn't been started yet
	bool IsExecuting() const;							// Return true if a gcode has been started and is not paused
	void SetFinished(bool f);							// Set the G Code executed (or not)
	const char* WritingFileDirectory() const;			// If we are writing the G Code to a file, where that file is
	void SetWritingFileDirectory(const char* wfd);		// Set the directory for the file to write the GCode in
	int GetToolNumberAdjust() const { return toolNumberAdjust; }
	void SetToolNumberAdjust(int arg) { toolNumberAdjust = arg; }
	void SetCommsProperties(uint32_t arg) { checksumRequired = (arg & 1); }
	bool StartingNewCode() const { return gcodePointer == 0; }
	MessageType GetResponseMessageType() const { return responseMessageType; }
	GCodeMachineState& MachineState() const { return *machineState; }
	GCodeMachineState& OriginalMachineState() const;
	bool PushState();									// Push state returning true if successful (i.e. stack not overflowed)
	bool PopState();									// Pop state returning true if successful (i.e. no stack underrun)
	bool IsDoingFileMacro() const;						// Return true if this source is executing a file macro
	GCodeState GetState() const;
	void SetState(GCodeState newState);
	void AdvanceState();
	const char *GetIdentity() const { return identity; }
	const bool CanQueueCodes() const { return queueCodes; }
	void MessageAcknowledged(bool cancelled);

	uint32_t whenTimerStarted;							// when we started waiting
	bool timerRunning;									// true if we are waiting

private:

	enum class GCodeBufferState
	{
		idle,			// we don't have a complete gcode ready
		ready,			// we have a complete gcode but haven't started executing it
		executing		// we have a complete gcode and have started executing it
	};

	int CheckSum() const;								// Compute the checksum (if any) at the end of the G Code

	GCodeMachineState *machineState;					// Machine state for this gcode source
	char gcodeBuffer[GCODE_LENGTH];						// The G Code
	const char* identity;								// Where we are from (web, file, serial line etc)
	int gcodePointer;									// Index in the buffer
	int readPointer;									// Where in the buffer to read next
	bool inQuotes;										// Are we inside double quotation marks?
	bool inComment;										// Are we after a ';' character?
	bool checksumRequired;								// True if we only accept commands with a valid checksum
	GCodeBufferState bufferState;						// Idle, executing or paused
	const char* writingFileDirectory;					// If the G Code is going into a file, where that is
	int toolNumberAdjust;								// The adjustment to tool numbers in commands we receive
	const MessageType responseMessageType;				// The message type we use for responses to commands coming from this channel
	bool queueCodes;									// Can we queue certain G-codes from this source?
};

inline const char* GCodeBuffer::Buffer() const
{
	return gcodeBuffer;
}

inline bool GCodeBuffer::IsIdle() const
{
	return bufferState == GCodeBufferState::idle;
}

inline bool GCodeBuffer::IsReady() const
{
	return bufferState == GCodeBufferState::ready;
}

inline bool GCodeBuffer::IsExecuting() const
{
	return bufferState == GCodeBufferState::executing;
}

inline void GCodeBuffer::SetFinished(bool f)
{
	bufferState = (f) ? GCodeBufferState::idle : GCodeBufferState::executing;
}

inline const char* GCodeBuffer::WritingFileDirectory() const
{
	return writingFileDirectory;
}

inline void GCodeBuffer::SetWritingFileDirectory(const char* wfd)
{
	writingFileDirectory = wfd;
}

inline GCodeState GCodeBuffer::GetState() const
{
	return machineState->state;
}

inline void GCodeBuffer::SetState(GCodeState newState)
{
	machineState->state = newState;
}

inline void GCodeBuffer::AdvanceState()
{
	machineState->state = static_cast<GCodeState>(static_cast<uint8_t>(machineState->state) + 1);
}

#endif /* GCODEBUFFER_H_ */
