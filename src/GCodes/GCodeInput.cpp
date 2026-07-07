/*
 * GCodeInput.cpp
 *
 *  Created on: 16 Sep 2016
 *      Author: Christian
 */

#include "GCodeInput.h"

#include <Platform/RepRap.h>
#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"

#include <Devices.h>

const size_t GCodeInputFileReadThreshold = 128;		// How many free bytes must be available before data is read from the file
const size_t GCodeInputUSBReadThreshold = 128;		// How many free bytes must be available before we read more data from USB

// Read some input bytes into the GCode buffer. Return true if there is a line of GCode waiting to be processed.
// This needs to be efficient
bool StandardGCodeInput::FillBuffer(GCodeBuffer *gb) noexcept
{
	const size_t bytesToPass = BytesCached();

#if HAS_MASS_STORAGE
	// To save calling IsWritingBinary on every character when we are not uploading a file in binary mode, use a separate loop for uploading
	if (gb->IsWritingBinary())
	{
		for (size_t i = 0; i < bytesToPass; i++)
		{
			const char c = ReadByte();
			if (gb->WriteBinaryToFile(c))
			{
				return false;				// finished binary upload, so we may as well stop here
			}
		}
	}
	else
#endif
	{
		for (size_t i = 0; i < bytesToPass; i++)
		{
			const char c = ReadByte();
			if (gb->Put(c))					// process a character, returns true if a line of GCode is complete
			{
#if HAS_MASS_STORAGE
				if (gb->IsWritingFile())
				{
					gb->WriteToFile();
				}
				else
#endif
				{
					return true;			// a line of GCode is complete, so stop here
				}
			}
		}
	}

	return false;
}

// G-code input class for wrapping around Stream-based hardware ports

void StreamGCodeInput::Reset() noexcept
{
	while (device.available() > 0)
	{
		device.read();
	}
}

char StreamGCodeInput::ReadByte() noexcept
{
	return static_cast<char>(device.read());
}

size_t StreamGCodeInput::BytesCached() const noexcept
{
	return device.available();
}

// Dynamic G-code input class for caching codes from software-defined sources

RegularGCodeInput::RegularGCodeInput(MessageType mt) noexcept
	: mtype(mt), state(GCodeInputState::idle), writingFile(false), writingPointer(0), readingPointer(0)
{
}

void RegularGCodeInput::Reset() noexcept
{
	state = GCodeInputState::idle;
	writingPointer = readingPointer = 0;
}

char RegularGCodeInput::ReadByte() noexcept
{
	char c = buffer[readingPointer++];
	if (readingPointer == GCodeInputBufferSize)
	{
		readingPointer = 0;
	}
	return c;
}

size_t RegularGCodeInput::BytesCached() const noexcept
{
	return (writingPointer - readingPointer) % GCodeInputBufferSize;
}

size_t RegularGCodeInput::BufferSpaceLeft() const noexcept
{
	return (readingPointer - writingPointer - 1u) % GCodeInputBufferSize;
}

// Check a character for urgent commands: M112 (emergency stop), M122 (diagnostics) and M108 (cancel wait for temperature).
// Return true if an urgent command was handled and the buffer was reset
bool RegularGCodeInput::CheckForUrgentCommand(char c) noexcept
{
	if (writingFile)
	{
		return false;
	}

	switch (state)
	{
		case GCodeInputState::idle:
			if (c > ' ')
			{
				state = (c == 'M' || c == 'm') ? GCodeInputState::doingMCode : GCodeInputState::doingCode;
			}
			break;

		case GCodeInputState::doingCode:
			if (c == 0 || c == '\r' || c == '\n')
			{
				state = GCodeInputState::idle;
			}
			break;

		case GCodeInputState::doingMCode:
			state = (c == '1') ? GCodeInputState::doingMCode1 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode1:
			state = (c == '0') ? GCodeInputState::doingMCode10
				  : (c == '1') ? GCodeInputState::doingMCode11
				  : (c == '2') ? GCodeInputState::doingMCode12
				  : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode10:
			state = (c == '8') ? GCodeInputState::doingMCode108 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode11:
			state = (c == '2') ? GCodeInputState::doingMCode112 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode12:
			state = (c == '2') ? GCodeInputState::doingMCode122 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode108:
			if (c < ' ' || c == ';')
			{
				reprap.GetGCodes().CancelWaitForTemperatures(false);
				// Normally we would call Reset and then return here to prevent the command being executed twice.
				// However, the host may be expecting an empty response, which we can't always send from here.
				// So we allow the command to be queued and executed again, which is harmless.
			}
			state = GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode112:
			if (c <= ' ' || c == ';')
			{
				// Emergency stop requested - perform it now
				reprap.EmergencyStop();
				reprap.GetGCodes().Reset();

				// But don't run it twice
				Reset();
				return true;
			}
			state = GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode122:
			// Only execute M122 here if there is no parameter
			if (c < ' ' || c == ';')
			{
				// Diagnostics requested - defer to the main task to avoid race conditions if called from another task
				reprap.DeferredDiagnostics(mtype);

				// But don't report them twice
				Reset();
				return true;
			}
			state = GCodeInputState::doingCode;
			break;
	}
	return false;
}

// BufferedStreamGCodeInput methods

// How many bytes are ready to be passed to the GCodeBuffer. Any trailing characters that might still be
// completing an urgent command are held back, and how many that is follows directly from the scanner state
size_t BufferedStreamGCodeInput::BytesCached() const noexcept
{
	const size_t cached = RegularGCodeInput::BytesCached();
	if (writingFile)
	{
		return cached;					// raw file data is never an urgent command, so none of it is held back
	}

	// The scanner state encodes how far a potential M112/M122/M108 has been matched, which is exactly how many
	// trailing characters must stay hidden until we know whether they form an urgent command
	size_t held;
	switch (state)
	{
		case GCodeInputState::doingMCode:		held = 1; break;
		case GCodeInputState::doingMCode1:		held = 2; break;
		case GCodeInputState::doingMCode10:
		case GCodeInputState::doingMCode11:
		case GCodeInputState::doingMCode12:		held = 3; break;
		case GCodeInputState::doingMCode108:
		case GCodeInputState::doingMCode112:
		case GCodeInputState::doingMCode122:	held = 4; break;
		default:								held = 0; break;
	}
	return (cached > held) ? cached - held : 0;
}

// Read from the device into our ring buffer and scan for urgent commands (M112/M108/M122)
void BufferedStreamGCodeInput::Spin() noexcept
{
	if (!device.IsConnected())
	{
		// The USB host has disconnected. Drop any buffered data so that when the host reconnects, its first
		// command starts cleanly instead of being appended to a leftover fragment
		if (RegularGCodeInput::BytesCached() != 0)
		{
			Reset();
		}
		return;
	}

	if (device.available() != 0)
	{
		const size_t spaceLeft = BufferSpaceLeft();
		if (spaceLeft >= GCodeInputUSBReadThreshold)
		{
			const size_t maxToTransfer = min<size_t>(spaceLeft, GCodeInputBufferSize - writingPointer);
			const size_t endPointer = writingPointer + device.readBytes(buffer + writingPointer, maxToTransfer);
			while (writingPointer != endPointer)
			{
				const size_t pos = writingPointer;
				if (CheckForUrgentCommand(buffer[pos]))
				{
					// Urgent command handled and buffer was reset (writingPointer is now 0).
					// Copy any remaining bytes after the trigger into the buffer so that
					// following commands (e.g. M999 after M112) can still be processed
					const size_t remaining = endPointer - pos - 1;
					if (remaining != 0)
					{
						memmove(buffer, buffer + pos + 1, remaining);
						writingPointer = remaining;
					}
					return;
				}
				writingPointer++;
			}
			if (writingPointer == GCodeInputBufferSize)
			{
				writingPointer = 0;
			}
		}
	}
}

bool BufferedStreamGCodeInput::FillBuffer(GCodeBuffer *gb) noexcept
{
	return StandardGCodeInput::FillBuffer(gb);
}

// NetworkGCodeInput methods
void NetworkGCodeInput::Put(char c) noexcept
{
	if (BufferSpaceLeft() == 0)
	{
		// Don't let the buffer overflow if we run out of space
		return;
	}

	// Check for urgent commands (M112, M122, M108) while receiving new characters
	if (CheckForUrgentCommand(c))
	{
		return;
	}

	// Feed another character into the buffer
	buffer[writingPointer++] = c;
	if (writingPointer == GCodeInputBufferSize)
	{
		writingPointer = 0;
	}
}

bool NetworkGCodeInput::Put(const char *_ecv_array buf) noexcept
{
	const size_t len = strlen(buf) + 1;
	MutexLocker lock(bufMutex, 200);
	if (lock.IsAcquired())
	{
		// Only cache this if we have enough space left
		if (len <= BufferSpaceLeft())
		{
			for (size_t i = 0; i < len; i++)
			{
				Put(buf[i]);
			}
			return true;
		}
	}
	return false;
}

NetworkGCodeInput::NetworkGCodeInput(MessageType mt) noexcept : RegularGCodeInput(mt)
{
	bufMutex.Create("NetworkGCodeInput");
}

// Fill a GCodeBuffer with the last available G-code
bool NetworkGCodeInput::FillBuffer(GCodeBuffer *gb) noexcept /*override*/
{
	MutexLocker lock(bufMutex, 10);
	return lock.IsAcquired() && RegularGCodeInput::FillBuffer(gb);
}

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// File-based G-code input source

// Reset this input. Should be called when the associated file is being closed
void FileGCodeInput::Reset() noexcept
{
	lastFileRead.Close();
	RegularGCodeInput::Reset();
}

// Reset this input. Should be called when a specific G-code or macro file is closed outside of the reading context
void FileGCodeInput::Reset(const FileData &file) noexcept
{
	if (lastFileRead == file)
	{
		Reset();
	}
}

// How many bytes have been cached for the given file?
size_t FileGCodeInput::FileBytesCached(const FileData &file) const noexcept
{
	return (lastFileRead == file) ? RegularGCodeInput::BytesCached() : 0;
}

// Read another chunk of G-codes from the file and return true if more data is available
GCodeInputReadResult FileGCodeInput::ReadFromFile(FileData &file) noexcept
{
	size_t bytesCached = RegularGCodeInput::BytesCached();

	// Keep track of the last file we read from
	if (lastFileRead != file)
	{
		if (lastFileRead.IsLive() && bytesCached > 0)
		{
			// Rewind back to the right position so we can resume at the right position later.
			// This may be necessary when nested macros are executed.
			lastFileRead.Seek(lastFileRead.GetPosition() - bytesCached);
		}

		RegularGCodeInput::Reset();
		bytesCached = 0;

		lastFileRead.CopyFrom(file);
	}

	// Read more from the file
	if (bytesCached < GCodeInputFileReadThreshold)
	{
		// Reset the read+write pointers for better performance if possible
		if (readingPointer == writingPointer)
		{
			readingPointer = writingPointer = 0;
		}

		// The code here used to read into a local buffer in blocks that are multiples of 4 bytes.
		// However, unless we can use a buffer of at least 512 bytes then that is redundant,
		// because the data will be copied via the sector buffer in FatFS anyway. So we don't do that any more.
		const int bytesRead = file.Read(buffer + writingPointer, min<size_t>(BufferSpaceLeft(), GCodeInputBufferSize - writingPointer));
		if (bytesRead < 0)
		{
			return GCodeInputReadResult::error;
		}
		if (bytesRead > 0)
		{
			writingPointer = (writingPointer + (size_t)bytesRead) % GCodeInputBufferSize;
			return GCodeInputReadResult::haveData;
		}
	}

	return (bytesCached > 0) ? GCodeInputReadResult::haveData : GCodeInputReadResult::noData;
}

#endif

// End
