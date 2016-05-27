/*
 * GCodeBuffer.h
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

#ifndef GCODEBUFFER_H_
#define GCODEBUFFER_H_

// Small class to hold an individual GCode and provide functions to allow it to be parsed
class GCodeBuffer
{
  public:
    GCodeBuffer(Platform* p, const char* id, MessageType mt);
    void Init(); 										// Set it up
	void Diagnostics(MessageType mtype);				// Write some debug info
    bool Put(char c);									// Add a character to the end
    bool Put(const char *str, size_t len);				// Add an entire string
    bool IsEmpty() const;								// Does this buffer contain any code?
    bool Seen(char c);									// Is a character present?
    float GetFValue();									// Get a float after a key letter
    int GetIValue();									// Get an integer after a key letter
    long GetLValue();									// Get a long integer after a key letter
    const char* GetUnprecedentedString(bool optional = false);	// Get a string with no preceding key letter
    const char* GetString();							// Get a string after a key letter
    const void GetFloatArray(float a[], size_t& length); // Get a :-separated list of floats after a key letter
    const void GetLongArray(long l[], size_t& length);	// Get a :-separated list of longs after a key letter
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
    bool IsPollRequest();
    MessageType GetResponseMessageType() const { return responseMessageType; }

    static bool IsPollCode(int code);

  private:

    enum class GCodeState
	{
    	idle,			// we don't have a complete gcode ready
		ready,			// we have a complete gcode but haven't started executing it
		executing		// we have a complete gcode and have started executing it
	};
    int CheckSum() const;								// Compute the checksum (if any) at the end of the G Code
    Platform* platform;									// Pointer to the RepRap's controlling class
    char gcodeBuffer[GCODE_LENGTH];						// The G Code
    const char* identity;								// Where we are from (web, file, serial line etc)
    int gcodePointer;									// Index in the buffer
    int readPointer;									// Where in the buffer to read next
    bool inComment;										// Are we after a ';' character?
    bool checksumRequired;								// True if we only accept commands with a valid checksum
    GCodeState state;									// Idle, executing or paused
    const char* writingFileDirectory;					// If the G Code is going into a file, where that is
    int toolNumberAdjust;								// The adjustment to tool numbers in commands we receive
    const MessageType responseMessageType;				// The message type we use for responses to commands coming from this channel
};

// Get an Int after a G Code letter
inline int GCodeBuffer::GetIValue()
{
	return static_cast<int>(GetLValue());
}

inline const char* GCodeBuffer::Buffer() const
{
	return gcodeBuffer;
}

inline bool GCodeBuffer::IsIdle() const
{
	return state == GCodeState::idle;
}

inline bool GCodeBuffer::IsReady() const
{
	return state == GCodeState::ready;
}

inline bool GCodeBuffer::IsExecuting() const
{
	return state == GCodeState::executing;
}

inline void GCodeBuffer::SetFinished(bool f)
{
	state = (f) ? GCodeState::idle : GCodeState::executing;
}

inline const char* GCodeBuffer::WritingFileDirectory() const
{
	return writingFileDirectory;
}

inline void GCodeBuffer::SetWritingFileDirectory(const char* wfd)
{
	writingFileDirectory = wfd;
}

#endif /* GCODEBUFFER_H_ */
