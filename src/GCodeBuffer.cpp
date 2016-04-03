/*
 * GCodeBuffer.cpp
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

//*************************************************************************************

#include "RepRapFirmware.h"

// This class stores a single G Code and provides functions to allow it to be parsed

GCodeBuffer::GCodeBuffer(Platform* p, const char* id)
	: platform(p), identity(id), checksumRequired(false), writingFileDirectory(nullptr), toolNumberAdjust(0)
{
	Init();
}

void GCodeBuffer::Init()
{
	gcodePointer = 0;
	readPointer = -1;
	inComment = false;
	state = GCodeState::idle;
}

int GCodeBuffer::CheckSum() const
{
	uint8_t cs = 0;
	for (size_t i = 0; gcodeBuffer[i] != '*' && gcodeBuffer[i] != 0; i++)
	{
		cs = cs ^ (uint8_t)gcodeBuffer[i];
	}
	return (int)cs;
}

// Add a byte to the code being assembled.  If false is returned, the code is
// not yet complete.  If true, it is complete and ready to be acted upon.
bool GCodeBuffer::Put(char c)
{
	if (c == '\r')
	{
		// Ignore carriage return, it messes up filenames sometimes if it appears in macro files etc.
		// Alternatively, we could handle it in the same way as linefeed, and add an optimisation to ignore blank lines.
		return false;
	}

	if (c == ';')
	{
		inComment = true;
	}
	else if (c == '\n' || c == 0)
	{
		gcodeBuffer[gcodePointer] = 0;
		if (reprap.Debug(moduleGcodes) && gcodeBuffer[0] != 0 && !writingFileDirectory) // Don't bother with blank/comment lines
		{
			platform->MessageF(HOST_MESSAGE, "%s%s\n", identity, gcodeBuffer);
		}

		// Deal with line numbers and checksums
		if (Seen('*'))
		{
			const int csSent = GetIValue();
			const int csHere = CheckSum();
			if (csSent != csHere)
			{
				if (Seen('N'))
				{
					snprintf(gcodeBuffer, GCODE_LENGTH, "M998 P%d", GetIValue());
				}
				Init();
				return true;
			}

			// Strip out the line number and checksum
			gcodePointer = 0;
			while (gcodeBuffer[gcodePointer] != ' ' && gcodeBuffer[gcodePointer] != 0)
			{
				gcodePointer++;
			}

			// Anything there?
			if (gcodeBuffer[gcodePointer] == 0)
			{
				// No...
				gcodeBuffer[0] = 0;
				Init();
				return false;
			}

			// Yes...
			gcodePointer++;
			int gp2 = 0;
			while (gcodeBuffer[gcodePointer] != '*' && gcodeBuffer[gcodePointer] != 0)
			{
				gcodeBuffer[gp2] = gcodeBuffer[gcodePointer++];
				gp2++;
			}
			gcodeBuffer[gp2] = 0;
		}
		else if (checksumRequired || IsEmpty())
		{
			// Checksum not found or buffer empty - cannot do anything
			gcodeBuffer[0] = 0;
			Init();
			return false;
		}
		Init();
		state = GCodeState::executing;
		return true;
	}
	else if (!inComment || writingFileDirectory)
	{
		gcodeBuffer[gcodePointer++] = c;
		if (gcodePointer >= (int)GCODE_LENGTH)
		{
			platform->Message(GENERIC_MESSAGE, "Error: G-Code buffer length overflow.\n");
			gcodePointer = 0;
			gcodeBuffer[0] = 0;
		}
	}

	return false;
}

bool GCodeBuffer::Put(const char *str, size_t len)
{
	for(size_t i=0; i<=len; i++)
	{
		if (Put(str[i]))
		{
			return true;
		}
	}

	return false;
}

// Does this buffer contain any code?

bool GCodeBuffer::IsEmpty() const
{
	const char *buf = gcodeBuffer;
	while (*buf != 0 && strchr(" \t\n\r", *buf) != nullptr)
	{
		buf++;
	}
	return *buf == 0;
}

// Is 'c' in the G Code string?
// Leave the pointer there for a subsequent read.

bool GCodeBuffer::Seen(char c)
{
	readPointer = 0;
	for (;;)
	{
		char b = gcodeBuffer[readPointer];
		if (b == 0 || b == ';') break;
		if (b == c) return true;
		++readPointer;
	}
	readPointer = -1;
	return false;
}

// Get a float after a G Code letter found by a call to Seen()

float GCodeBuffer::GetFValue()
{
	if (readPointer < 0)
	{
		platform->Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode float before a search.\n");
		readPointer = -1;
		return 0.0;
	}
	float result = (float) strtod(&gcodeBuffer[readPointer + 1], 0);
	readPointer = -1;
	return result;
}

// Get a :-separated list of floats after a key letter

const void GCodeBuffer::GetFloatArray(float a[], size_t& returnedLength)
{
	if(readPointer < 0)
	{
		platform->Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode float array before a search.\n");
		readPointer = -1;
		returnedLength = 0;
		return;
	}

	size_t length = 0;
	bool inList = true;
	while(inList)
	{
		if(length >= returnedLength) // Array limit has been set in here
		{
			platform->MessageF(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode float array that is too long: %s\n", gcodeBuffer);
			readPointer = -1;
			returnedLength = 0;
			return;
		}
		a[length] = (float)strtod(&gcodeBuffer[readPointer + 1], 0);
		length++;
		readPointer++;
		while(gcodeBuffer[readPointer] && (gcodeBuffer[readPointer] != ' ') && (gcodeBuffer[readPointer] != LIST_SEPARATOR))
		{
			readPointer++;
		}
		if(gcodeBuffer[readPointer] != LIST_SEPARATOR)
		{
			inList = false;
		}
	}

	// Special case if there is one entry and returnedLength requests several.
	// Fill the array with the first entry.

	if (length == 1 && returnedLength > 1)
	{
		for(size_t i = 1; i < returnedLength; i++)
		{
			a[i] = a[0];
		}
	}
	else
	{
		returnedLength = length;
	}

	readPointer = -1;
}

// Get a :-separated list of longs after a key letter

const void GCodeBuffer::GetLongArray(long l[], size_t& returnedLength)
{
	if(readPointer < 0)
	{
		platform->Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode long array before a search.\n");
		readPointer = -1;
		return;
	}

	size_t length = 0;
	bool inList = true;
	while(inList)
	{
		if(length >= returnedLength) // Array limit has been set in here
		{
			platform->MessageF(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode long array that is too long: %s\n", gcodeBuffer);
			readPointer = -1;
			returnedLength = 0;
			return;
		}
		l[length] = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
		length++;
		readPointer++;
		while(gcodeBuffer[readPointer] && (gcodeBuffer[readPointer] != ' ') && (gcodeBuffer[readPointer] != LIST_SEPARATOR))
		{
			readPointer++;
		}
		if(gcodeBuffer[readPointer] != LIST_SEPARATOR)
		{
			inList = false;
		}
	}
	returnedLength = length;
	readPointer = -1;
}

// Get a string after a G Code letter found by a call to Seen().
// It will be the whole of the rest of the GCode string, so strings
// should always be the last parameter.

const char* GCodeBuffer::GetString()
{
	if (readPointer < 0)
	{
		platform->Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode string before a search.\n");
		readPointer = -1;
		return "";
	}
	const char* result = &gcodeBuffer[readPointer + 1];
	readPointer = -1;
	return result;
}

// This returns a pointer to the end of the buffer where a
// string starts.  It assumes that an M or G search has
// been done followed by a GetIValue(), so readPointer will
// be -1.  It absorbs "M/Gnnn " (including the space) from the
// start and returns a pointer to the next location.

// This is provided for legacy use, in particular in the M23
// command that sets the name of a file to be printed.  In
// preference use GetString() which requires the string to have
// been preceded by a tag letter.

const char* GCodeBuffer::GetUnprecedentedString(bool optional)
{
	readPointer = 0;
	while (gcodeBuffer[readPointer] != 0 && gcodeBuffer[readPointer] != ' ')
	{
		readPointer++;
	}

	if (gcodeBuffer[readPointer] == 0)
	{
		readPointer = -1;
		if (optional)
		{
			return nullptr;
		}
		platform->Message(GENERIC_MESSAGE, "Error: GCodes: String expected but not seen.\n");
		return gcodeBuffer; // Good idea?
	}

	const char* result = &gcodeBuffer[readPointer + 1];
	readPointer = -1;
	return result;
}

// Get an long after a G Code letter

long GCodeBuffer::GetLValue()
{
	if (readPointer < 0)
	{
		platform->Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode int before a search.\n");
		readPointer = -1;
		return 0;
	}
	long result = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
	readPointer = -1;
	return result;
}

// Return true if this buffer contains a poll request or empty request that can be executed while macros etc. from another source are being completed
bool GCodeBuffer::IsPollRequest()
{
	if (state == GCodeState::executing)
	{	if (IsEmpty())
		{
			return true;
		}
		if (Seen('M'))
		{
			return IsPollCode(GetIValue());
		}
	}
	return false;
}

// Return true if the specified M code can be executed concurrently with other requests.
// These should be codes that do not modify the state (we make an exception for M111) and are always executed in a single call to function HandleMCode.
/*static*/ bool GCodeBuffer::IsPollCode(int code)
{
	return code == 27 || code == 105 || code == 111 || code == 114 || code == 119 || code == 122 || code == 408 || code == 573;
}

// End
