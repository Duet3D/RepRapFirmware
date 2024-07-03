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

RegularGCodeInput::RegularGCodeInput() noexcept
	: state(GCodeInputState::idle), writingPointer(0), readingPointer(0)
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

// BufferedStreamGCodeInput methods
void BufferedStreamGCodeInput::Reset() noexcept
{
	RegularGCodeInput::Reset();
	while (device.available() > 0)
	{
		device.read();
	}
}

bool BufferedStreamGCodeInput::FillBuffer(GCodeBuffer *gb) noexcept
{
	const size_t spaceLeft = BufferSpaceLeft();
	if (spaceLeft >= GCodeInputUSBReadThreshold)
	{
		const size_t maxToTransfer = (readingPointer > writingPointer || writingPointer == 0) ? spaceLeft : GCodeInputBufferSize - writingPointer;
		writingPointer = (writingPointer + device.readBytes(buffer + writingPointer, maxToTransfer)) % GCodeInputBufferSize;
	}
	return StandardGCodeInput::FillBuffer(gb);
}

// NetworkGCodeInput methods
void NetworkGCodeInput::Put(MessageType mtype, char c) noexcept
{
	if (BufferSpaceLeft() == 0)
	{
		// Don't let the buffer overflow if we run out of space
		return;
	}

	// Check for M112 (emergency stop) and for M122 (diagnostics) while receiving new characters
	switch (state)
	{
		case GCodeInputState::idle:
			if (c <= ' ')
			{
				// ignore whitespaces at the beginning
				return;
			}

			state = (c == 'M' || c == 'm') ? GCodeInputState::doingMCode : GCodeInputState::doingCode;
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
			state = (c == '1') ? GCodeInputState::doingMCode11: (c == '2') ? GCodeInputState::doingMCode12 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode11:
			state = (c == '2') ? GCodeInputState::doingMCode112 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode12:
			state = (c == '2') ? GCodeInputState::doingMCode122 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode112:
			if (c <= ' ' || c == ';')
			{
				// Emergency stop requested - perform it now
				reprap.EmergencyStop();
				reprap.GetGCodes().Reset();

				// But don't run it twice
				Reset();
				return;
			}
			state = GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode122:
			// Only execute M122 here if there is no parameter
			if (c < ' ' || c == ';')
			{
				// Diagnostics requested - report them now
				// Don't use the Network task itself to send them because this may result in deadlock if there is insufficient buffer space,
				// because the Network task itself is used to send the output to the clients and free the buffers.
				// Ask the main task to do it instead.
				reprap.DeferredDiagnostics(mtype);

				// But don't report them twice
				Reset();
				return;
			}
			state = GCodeInputState::doingCode;
			break;
	}

	// Feed another character into the buffer
	buffer[writingPointer++] = c;
	if (writingPointer == GCodeInputBufferSize)
	{
		writingPointer = 0;
	}
}

bool NetworkGCodeInput::Put(MessageType mtype, const char *buf) noexcept
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
				Put(mtype, buf[i]);
			}
			return true;
		}
	}
	return false;
}

NetworkGCodeInput::NetworkGCodeInput() noexcept : RegularGCodeInput()
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
