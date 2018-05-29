/*
 * GCodeInput.h
 *
 *  Created on: 16 Sep 2016
 *      Author: Christian
 */

#ifndef GCODEINPUT_H
#define GCODEINPUT_H

#include "RepRapFirmware.h"
#include "Storage/FileData.h"
#include "MessageType.h"
#include "RTOSIface.h"

const size_t GCodeInputBufferSize = 256;				// How many bytes can we cache per input source?
const size_t GCodeInputFileReadThreshold = 128;			// How many free bytes must be available before data is read from the SD card?


// This base class is intended to provide incoming G-codes for the GCodeBuffer class
class GCodeInput
{
public:
	virtual void Reset() = 0;							// Clean all the cached data from this input
	virtual bool FillBuffer(GCodeBuffer *gb);			// Fill a GCodeBuffer with the last available G-code
	virtual size_t BytesCached() const = 0;				// How many bytes have been cached?

protected:
	virtual char ReadByte() = 0;						// Get the next byte from the source
};

// This class wraps around an existing Stream device which lets us avoid double buffering.
// The only downside is that we cannot (yet) look through the hardware buffer and check for requested emergency stops.
// TODO: This will require some more work in the Arduino core.
class StreamGCodeInput : public GCodeInput
{
public:
	StreamGCodeInput(Stream &dev) : device(dev) { }

	void Reset() override;
	size_t BytesCached() const override;				// How many bytes have been cached?

protected:
	char ReadByte() override;

private:
	Stream &device;
};


// When characters from input sources are received, they should be checked consequently for M112 (Emergency Stop).
// This allows us to react faster to an incoming emergency stop since other codes may be blocking the associated
// GCodeBuffer instance.
enum class GCodeInputState
{
	idle,
	doingCode,
	doingMCode,
	doingMCode1,
	doingMCode11,
	doingMCode12,
	doingMCode112,
	doingMCode122
};

// This class allows caching of dynamic content (from web-based sources) and implements a simple ring buffer.
// In addition, incoming codes are checked for M112 (emergency stop) to execute perform emergency stops as quickly
// as possible. Comments can be optionally stripped from sources where comments are not needed (e.g. HTTP).
class RegularGCodeInput : public GCodeInput
{
public:
	RegularGCodeInput();

	void Reset() override;
	size_t BytesCached() const override;				// How many bytes have been cached?
	size_t BufferSpaceLeft() const;						// How much space do we have left?

protected:
	char ReadByte() override;

	GCodeInputState state;
	size_t writingPointer, readingPointer;
	char buffer[GCodeInputBufferSize];
};

enum class GCodeInputReadResult : uint8_t { haveData, noData, error };

// This class is an expansion of the RegularGCodeInput class to buffer G-codes and to rewind file positions when
// nested G-code files are started. However buffered codes are not explicitly checked for M112.
class FileGCodeInput : public RegularGCodeInput
{
public:

	FileGCodeInput() : RegularGCodeInput(), lastFile(nullptr) { }

	void Reset() override;								// This should be called when the associated file is being closed
	void Reset(const FileData &file);					// Should be called when a specific G-code or macro file is closed or re-opened outside the reading context

	GCodeInputReadResult ReadFromFile(FileData &file);	// Read another chunk of G-codes from the file and return true if more data is available

private:
	FileStore *lastFile;
};

// This class receives its data from the network task
class NetworkGCodeInput: public RegularGCodeInput
{
public:
	NetworkGCodeInput();

	bool FillBuffer(GCodeBuffer *gb) override;			// Fill a GCodeBuffer with the last available G-code
	void Put(MessageType mtype, const char *buf);		// Append a null-terminated string to the buffer

private:
	void Put(MessageType mtype, char c);				// Append a single character

	Mutex bufMutex;
};

#endif
