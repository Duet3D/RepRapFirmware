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
	: state(GCodeInputState::idle), buffer(reinterpret_cast<char * const>(buf32)), writingPointer(0), readingPointer(0)
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

void RegularGCodeInput::Put(MessageType mtype, const char c)
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
				// Only send the report to the appropriate channel, because if we send it as a generic message instead then it gets truncated.
				reprap.Diagnostics(mtype);

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

void RegularGCodeInput::Put(MessageType mtype, const char *buf)
{
	Put(mtype, buf, strlen(buf) + 1);
}

void RegularGCodeInput::Put(MessageType mtype, const char *buf, size_t len)
{
	if (len > BufferSpaceLeft())
	{
		// Don't cache this if we don't have enough space left
		return;
	}

	for (size_t i = 0; i < len; i++)
	{
		Put(mtype, buf[i]);
	}
}

size_t RegularGCodeInput::BufferSpaceLeft() const
{
	return (readingPointer - writingPointer - 1u) % GCodeInputBufferSize;
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
bool FileGCodeInput::ReadFromFile(FileData &file)
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

		// Read blocks with sizes multiple of 4 for HSMCI efficiency
		uint32_t readBuffer32[(GCodeInputBufferSize + 3) / 4];
		char * const readBuffer = reinterpret_cast<char * const>(readBuffer32);

		int bytesRead = file.Read(readBuffer, BufferSpaceLeft() & (~3));
		if (bytesRead > 0)
		{
			int remaining = GCodeInputBufferSize - writingPointer;
			if (bytesRead <= remaining)
			{
				memcpy(buffer + writingPointer, readBuffer, bytesRead);
			}
			else
			{
				memcpy(buffer + writingPointer, readBuffer, remaining);
				memcpy(buffer, readBuffer + remaining, bytesRead - remaining);
			}
			writingPointer = (writingPointer + bytesRead) % GCodeInputBufferSize;

			return true;
		}
	}

	return bytesCached > 0;
}

// End
