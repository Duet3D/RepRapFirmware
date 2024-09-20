/*
 * GCodeInput.h
 *
 *  Created on: 16 Sep 2016
 *      Author: Christian
 */

#ifndef GCODEINPUT_H
#define GCODEINPUT_H

#include <RepRapFirmware.h>
#include <Storage/FileData.h>
#include <RTOSIface/RTOSIface.h>

#include <Stream.h>

const size_t GCodeInputBufferSize = 256;						// How many bytes can we cache per input source? Make this a power of 2 for efficiency

// This base class provides incoming G-codes for the GCodeBuffer class
class GCodeInput
{
public:
	virtual void Reset() noexcept = 0;							// Clean all the cached data from this input
	virtual bool FillBuffer(GCodeBuffer *gb) noexcept = 0;		// Fill a GCodeBuffer with the last available G-code
	virtual size_t BytesCached() const noexcept = 0;			// How many bytes have been cached?
};

// This class provides a standard implementation of FillBuffer that calls ReadByte() to supply individual characters
class StandardGCodeInput : public GCodeInput
{
public:
	bool FillBuffer(GCodeBuffer *gb) noexcept override;			// Fill a GCodeBuffer with the last available G-code

protected:
	virtual char ReadByte() noexcept = 0;						// Get the next byte from the source
};

// This class wraps around an existing Stream device which lets us avoid double buffering.
class StreamGCodeInput : public StandardGCodeInput
{
public:
	explicit StreamGCodeInput(Stream &_ecv_from dev) noexcept : device(dev) { }

	void Reset() noexcept override;
	size_t BytesCached() const noexcept override;				// How many bytes have been cached?

protected:
	char ReadByte() noexcept override;

private:
	Stream &_ecv_from device;
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
class RegularGCodeInput : public StandardGCodeInput
{
public:
	RegularGCodeInput() noexcept;

	void Reset() noexcept override;
	size_t BytesCached() const noexcept override;				// How many bytes have been cached?
	size_t BufferSpaceLeft() const noexcept;					// How much space do we have left?

protected:
	char ReadByte() noexcept override;

	GCodeInputState state;
	size_t writingPointer, readingPointer;
	char buffer[GCodeInputBufferSize];
};

// Class to buffer input from streams that have very slow single-character interfaces, in particular the Microchip SAM4E/4S/E70 USB driver
class BufferedStreamGCodeInput : public RegularGCodeInput
{
public:
	explicit BufferedStreamGCodeInput(Stream &_ecv_from dev) noexcept : RegularGCodeInput(), device(dev) { }

	void Reset() noexcept override;
	bool FillBuffer(GCodeBuffer *gb) noexcept override;			// Fill a GCodeBuffer with the last available G-code

private:
	Stream &_ecv_from device;
};

enum class GCodeInputReadResult : uint8_t { haveData, noData, error };

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// This class is an expansion of the RegularGCodeInput class to buffer G-codes and to rewind file positions when
// nested G-code files are started. However buffered codes are not explicitly checked for M112.
class FileGCodeInput : public RegularGCodeInput
{
public:

	FileGCodeInput() noexcept : RegularGCodeInput() { }

	void Reset() noexcept override;								// Clears the buffer. Should be called when the associated file is being closed
	void Reset(const FileData &file) noexcept;					// Clears the buffer of a specific file. Should be called when it is closed or re-opened outside the reading context
	size_t FileBytesCached(const FileData &file) const noexcept;	// How many bytes have been cached for the given file?

	GCodeInputReadResult ReadFromFile(FileData &file) noexcept;	// Read another chunk of G-codes from the file and return true if more data is available

private:
	FileData lastFileRead;
};

#endif

// This class receives its data from the network task
class NetworkGCodeInput : public RegularGCodeInput
{
public:
	NetworkGCodeInput() noexcept;

	bool FillBuffer(GCodeBuffer *gb) noexcept override;			// Fill a GCodeBuffer with the last available G-code
	bool Put(MessageType mtype, const char *buf) noexcept;		// Append a null-terminated string to the buffer returning true if success

private:
	void Put(MessageType mtype, char c) noexcept;				// Append a single character. This does NOT lock the mutex!

	Mutex bufMutex;
};

#endif
