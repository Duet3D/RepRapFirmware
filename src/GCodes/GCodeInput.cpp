/*
 * GCodeInput.cpp
 *
 *  Created on: 16 Sep 2016
 *      Author: Christian
 */

#include "GCodeInput.h"

#include "RepRap.h"
#include "GCodes.h"
#include "GCodeBuffer.h"

bool GCodeInput::FillBuffer(GCodeBuffer *gb)
{
	const size_t bytesToPass = min<size_t>(BytesCached(), GCODE_LENGTH);
	for (size_t i = 0; i < bytesToPass; i++)
	{
		const char c = ReadByte();

		if (gb->IsWritingBinary())
		{
			// HTML uploads are handled by the GCodes class
			reprap.GetGCodes().WriteHTMLToFile(*gb, c);
		}
		else if (gb->Put(c))
		{
			// Check if we can finish a file upload
			if (gb->WritingFileDirectory() != nullptr)
			{
				reprap.GetGCodes().WriteGCodeToFile(*gb);
				gb->SetFinished(true);
			}

			// Code is complete, stop here
			return true;
		}
	}

	return false;
}

// G-code input class for wrapping around Stream-based hardware ports

void StreamGCodeInput::Reset()
{
	while (device.available() > 0)
	{
		device.read();
	}
}

char StreamGCodeInput::ReadByte()
{
	return static_cast<char>(device.read());
}

size_t StreamGCodeInput::BytesCached() const
{
	return device.available();
}

// Dynamic G-code input class for caching codes from software-defined sources

RegularGCodeInput::RegularGCodeInput()
	: state(GCodeInputState::idle), writingPointer(0), readingPointer(0)
{
}

void RegularGCodeInput::Reset()
{
	state = GCodeInputState::idle;
	writingPointer = readingPointer = 0;
}

char RegularGCodeInput::ReadByte()
{
	char c = buffer[readingPointer++];
	if (readingPointer == GCodeInputBufferSize)
	{
		readingPointer = 0;
	}
	return c;
}


size_t RegularGCodeInput::BytesCached() const
{
	if (writingPointer >= readingPointer)
	{
		return writingPointer - readingPointer;
	}
	return GCodeInputBufferSize - readingPointer + writingPointer;
}

size_t RegularGCodeInput::BufferSpaceLeft() const
{
	return (readingPointer - writingPointer - 1u) % GCodeInputBufferSize;
}

void NetworkGCodeInput::Put(MessageType mtype, char c)
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

			state = (c == 'M') ? GCodeInputState::doingMCode : GCodeInputState::doingCode;
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

void NetworkGCodeInput::Put(MessageType mtype, const char *buf)
{
	const size_t len = strlen(buf) + 1;
	MutexLocker lock(bufMutex, 200);
	if (lock)
	{
		// Only cache this if we have enough space left
		if (len <= BufferSpaceLeft())
		{
			for (size_t i = 0; i < len; i++)
			{
				Put(mtype, buf[i]);
			}
		}
	}
}

NetworkGCodeInput::NetworkGCodeInput() : RegularGCodeInput()
{
	bufMutex.Create();
}

// Fill a GCodeBuffer with the last available G-code
bool NetworkGCodeInput::FillBuffer(GCodeBuffer *gb) /*override*/
{
	MutexLocker lock(bufMutex, 10);
	return lock && RegularGCodeInput::FillBuffer(gb);
}

// File-based G-code input source

// Reset this input. Should be called when the associated file is being closed
void FileGCodeInput::Reset()
{
	lastFile = nullptr;
	RegularGCodeInput::Reset();
}

// Reset this input. Should be called when a specific G-code or macro file is closed outside of the reading context
void FileGCodeInput::Reset(const FileData &file)
{
	if (file.f == lastFile)
	{
		Reset();
	}
}

// Read another chunk of G-codes from the file and return true if more data is available
GCodeInputReadResult FileGCodeInput::ReadFromFile(FileData &file)
{
	const size_t bytesCached = BytesCached();

	// Keep track of the last file we read from
	if (lastFile != nullptr && lastFile != file.f)
	{
		if (bytesCached > 0)
		{
			// Rewind back to the right position so we can resume at the right position later.
			// This may be necessary when nested macros are executed.
			lastFile->Seek(lastFile->Position() - bytesCached);
		}

		RegularGCodeInput::Reset();
	}
	lastFile = file.f;

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

// End
