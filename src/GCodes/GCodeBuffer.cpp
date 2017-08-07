/*
 * GCodeBuffer.cpp
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

//*************************************************************************************

#include "GCodeBuffer.h"
#include "Platform.h"
#include "RepRap.h"

// Create a default GCodeBuffer
GCodeBuffer::GCodeBuffer(const char* id, MessageType mt, bool usesCodeQueue)
	: machineState(new GCodeMachineState()), identity(id), checksumRequired(false), writingFileDirectory(nullptr),
	  toolNumberAdjust(0), responseMessageType(mt), queueCodes(usesCodeQueue), binaryWriting(false)
{
	Init();
}

void GCodeBuffer::Reset()
{
	while (PopState()) { }
	Init();
}

void GCodeBuffer::Init()
{
	gcodePointer = 0;
	commandLength = 0;
	readPointer = -1;
	inQuotes = inComment = timerRunning = false;
	bufferState = GCodeBufferState::idle;
}

void GCodeBuffer::Diagnostics(MessageType mtype)
{
	switch (bufferState)
	{
	case GCodeBufferState::idle:
		scratchString.printf("%s is idle", identity);
		break;

	case GCodeBufferState::ready:
		scratchString.printf("%s is ready with \"%s\"", identity, Buffer());
		break;

	case GCodeBufferState::executing:
		scratchString.printf("%s is doing \"%s\"", identity, Buffer());
	}

	scratchString.cat(" in state(s)");
	const GCodeMachineState *ms = machineState;
	do
	{
		scratchString.catf(" %d", ms->state);
		ms = ms->previous;
	}
	while (ms != nullptr);
	scratchString.cat('\n');
	reprap.GetPlatform().Message(mtype, scratchString.Pointer());
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
	if (c != 0)
	{
		++commandLength;
	}

	if (!inQuotes && ((c == ';') || (gcodePointer == 0 && c == '(')))
	{
		inComment = true;
	}
	else if (c == '\n' || c == '\r' || c == 0)
	{
		gcodeBuffer[gcodePointer] = 0;
		if (reprap.Debug(moduleGcodes) && gcodeBuffer[0] != 0 && !writingFileDirectory) // Don't bother with blank/comment lines
		{
			reprap.GetPlatform().MessageF(DEBUG_MESSAGE, "%s: %s\n", identity, gcodeBuffer);
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
					snprintf(gcodeBuffer, GCODE_LENGTH, "M998 P%ld", GetIValue());
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
		else if ((checksumRequired && machineState->previous == nullptr) || IsEmpty())
		{
			// Checksum not found or buffer empty - cannot do anything
			gcodeBuffer[0] = 0;
			Init();
			return false;
		}
		readPointer = -1;;
		bufferState = GCodeBufferState::ready;
		return true;
	}
	else if (!inComment || writingFileDirectory)
	{
		if (gcodePointer >= (int)GCODE_LENGTH)
		{
			reprap.GetPlatform().MessageF(GENERIC_MESSAGE, "Error: G-Code buffer '$s' length overflow\n", identity);
			Init();
		}
		else
		{
			gcodeBuffer[gcodePointer++] = c;
			if (c == '"' && !inComment)
			{
				inQuotes = !inQuotes;
			}
		}
	}

	return false;
}

bool GCodeBuffer::Put(const char *str, size_t len)
{
	for(size_t i = 0; i <= len; i++)
	{
		if (Put(str[i]))
		{
			return true;
		}
	}

	return false;
}

void GCodeBuffer::SetFinished(bool f)
{
	if (f)
	{	bufferState = GCodeBufferState::idle;
		Init();
	}
	else
	{
		bufferState = GCodeBufferState::executing;
	}
}

// Get the file position at the start of the current command
FilePosition GCodeBuffer::GetFilePosition(size_t bytesCached) const
{
	if (machineState->fileState.IsLive())
	{
		return machineState->fileState.GetPosition() - bytesCached - commandLength;
	}
	return noFilePosition;
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
	bool inQuotes = false;
	for (;;)
	{
		const char b = gcodeBuffer[readPointer];
		if (b == 0)
		{
			break;
		}
		if (b == '"')
		{
			inQuotes = !inQuotes;
		}
		else if (!inQuotes)
		{
			if (b == ';')
			{
				break;
			}
			if (toupper(b) == c)
			{
				return true;
			}
		}
		++readPointer;
	}
	readPointer = -1;
	return false;
}

// Return the first G, M or T command letter. Needed so that we don't pick up a spurious command letter form inside a string parameter.
char GCodeBuffer::GetCommandLetter()
{
	readPointer = 0;
	for (;;)
	{
		char b = gcodeBuffer[readPointer];
		if (b == 0 || b == ';') break;
		b = (char)toupper(b);
		if (b == 'G' || b == 'M' || b == 'T')
		{
			return b;
		}
		++readPointer;
	}
	readPointer = -1;
	return 0;
}

// Get a float after a G Code letter found by a call to Seen()
float GCodeBuffer::GetFValue()
{
	if (readPointer < 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode float before a search.\n");
		return 0.0;
	}
	float result = (float) strtod(&gcodeBuffer[readPointer + 1], 0);
	readPointer = -1;
	return result;
}

// Get a :-separated list of floats after a key letter
const void GCodeBuffer::GetFloatArray(float a[], size_t& returnedLength, bool doPad)
{
	if (readPointer < 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode float array before a search.\n");
		returnedLength = 0;
		return;
	}

	size_t length = 0;
	bool inList = true;
	while(inList)
	{
		if (length >= returnedLength)		// array limit has been set in here
		{
			reprap.GetPlatform().MessageF(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode float array that is too long: %s\n", gcodeBuffer);
			readPointer = -1;
			returnedLength = 0;
			return;
		}
		a[length] = (float)strtod(&gcodeBuffer[readPointer + 1], 0);
		length++;
		do
		{
			readPointer++;
		} while(gcodeBuffer[readPointer] && (gcodeBuffer[readPointer] != ' ') && (gcodeBuffer[readPointer] != LIST_SEPARATOR));
		if (gcodeBuffer[readPointer] != LIST_SEPARATOR)
		{
			inList = false;
		}
	}

	// Special case if there is one entry and returnedLength requests several.
	// Fill the array with the first entry.
	if (doPad && length == 1 && returnedLength > 1)
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
	if (readPointer < 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode long array before a search.\n");
		return;
	}

	size_t length = 0;
	bool inList = true;
	while(inList)
	{
		if (length >= returnedLength) // Array limit has been set in here
		{
			reprap.GetPlatform().MessageF(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode long array that is too long: %s\n", gcodeBuffer);
			readPointer = -1;
			returnedLength = 0;
			return;
		}
		l[length] = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
		length++;
		do
		{
			readPointer++;
		} while(gcodeBuffer[readPointer] != 0 && (gcodeBuffer[readPointer] != ' ') && (gcodeBuffer[readPointer] != LIST_SEPARATOR));
		if (gcodeBuffer[readPointer] != LIST_SEPARATOR)
		{
			inList = false;
		}
	}
	returnedLength = length;
	readPointer = -1;
}

// Get a string after a G Code letter found by a call to Seen().
// It will be the whole of the rest of the GCode string, so strings should always be the last parameter.
// Use the other overload of GetString to get strings that may not be the last parameter, or may be quoted.
const char* GCodeBuffer::GetString()
{
	if (readPointer < 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode string before a search.\n");
		return "";
	}
	const char* const result = &gcodeBuffer[readPointer + 1];
	readPointer = -1;
	return result;
}

// Get and copy a quoted string
bool GCodeBuffer::GetQuotedString(char *buf, size_t buflen)
{
	if (readPointer < 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode string before a search.\n");
		return false;
	}
	++readPointer;				// skip the character that introduced the string
	char c = gcodeBuffer[readPointer++];
	size_t i = 0;
	if (c == '"')
	{
		// Quoted string
		for (;;)
		{
			c = gcodeBuffer[readPointer++];
			if (c < ' ')
			{
				return false;
			}
			if (c == '"')
			{
				if (gcodeBuffer[readPointer++] != '"')
				{
					if (i < buflen)
					{
						buf[i] = 0;
						return true;
					}
					return false;
				}
			}
			if (i < buflen)
			{
				buf[i] = c;
			}
			++i;
		}
	}
	return false;
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

// If no string was provided, it produces an error message if the string was not optional, and returns nullptr.
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
		if (!optional)
		{
			reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: String expected but not seen.\n");
		}
		return nullptr;
	}

	const char* const result = &gcodeBuffer[readPointer + 1];
	readPointer = -1;
	return result;
}

// Get an int32 after a G Code letter
int32_t GCodeBuffer::GetIValue()
{
	if (readPointer < 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode int before a search.\n");
		readPointer = -1;
		return 0;
	}
	const int32_t result = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
	readPointer = -1;
	return result;
}

// Get an uint32 after a G Code letter
uint32_t GCodeBuffer::GetUIValue()
{
	if (readPointer < 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode int before a search.\n");
		readPointer = -1;
		return 0;
	}
	const uint32_t result = strtoul(&gcodeBuffer[readPointer + 1], 0, 0);
	readPointer = -1;
	return result;
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
void GCodeBuffer::TryGetFValue(char c, float& val, bool& seen)
{
	if (Seen(c))
	{
		val = GetFValue();
		seen = true;
	}
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
void GCodeBuffer::TryGetIValue(char c, int32_t& val, bool& seen)
{
	if (Seen(c))
	{
		val = GetIValue();
		seen = true;
	}
}

// Try to get a float array exactly 'numVals' long after parameter letter 'c'.
// If the wrong number of values is provided, generate an error message and return true.
// Else set 'seen' if we saw the letter and value, and return false.
bool GCodeBuffer::TryGetFloatArray(char c, size_t numVals, float vals[], StringRef& reply, bool& seen, bool doPad)
{
	if (Seen(c))
	{
		size_t count = numVals;
		GetFloatArray(vals, count, doPad);
		if (count == numVals)
		{
			seen = true;
		}
		else
		{
			reply.printf("Wrong number of values after '\''%c'\'', expected %d", c, numVals);
			return true;
		}
	}
	return false;
}

// Try to get a quoted string after parameter letter.
// If we found it then set 'seen' true and return true, else leave 'seen' alone and return false
bool GCodeBuffer::TryGetQuotedString(char c, char *buf, size_t buflen, bool& seen)
{
	if (Seen(c) && GetQuotedString(buf, buflen))
	{
		seen = true;
		return true;
	}
	return false;
}

// Get an IP address quad after a key letter
bool GCodeBuffer::GetIPAddress(uint8_t ip[4])
{
	if (readPointer < 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: GCodes: Attempt to read a GCode string before a search.\n");
		return false;
	}
	const char* p = &gcodeBuffer[readPointer + 1];
	unsigned int n = 0;
	for (;;)
	{
		char *pp;
		const unsigned long v = strtoul(p, &pp, 10);
		if (pp == p || v > 255)
		{
			readPointer = -1;
			return false;
		}
		ip[n] = (uint8_t)v;
		++n;
		p = pp;
		if (*p != '.')
		{
			break;
		}
		if (n == 4)
		{
			readPointer = -1;
			return false;
		}
		++p;
	}
	readPointer = -1;
	return n == 4;
}

// Get an IP address quad after a key letter
bool GCodeBuffer::GetIPAddress(uint32_t& ip)
{
	uint8_t ipa[4];
	const bool ok = GetIPAddress(ipa);
	if (ok)
	{
		ip = (uint32_t)ipa[0] | ((uint32_t)ipa[1] << 8) | ((uint32_t)ipa[2] << 16) | ((uint32_t)ipa[3] << 24);
	}
	return ok;
}

// Get the original machine state before we pushed anything
GCodeMachineState& GCodeBuffer::OriginalMachineState() const
{
	GCodeMachineState *ms = machineState;
	while (ms->previous != nullptr)
	{
		ms = ms->previous;
	}
	return *ms;
}

// Push state returning true if successful (i.e. stack not overflowed)
bool GCodeBuffer::PushState()
{
	// Check the current stack depth
	unsigned int depth = 0;
	for (const GCodeMachineState *m1 = machineState; m1 != nullptr; m1 = m1->previous)
	{
		++depth;
	}
	if (depth >= MaxStackDepth + 1)				// the +1 is to allow for the initial state record
	{
		return false;
	}

	GCodeMachineState * const ms = GCodeMachineState::Allocate();
	ms->previous = machineState;
	ms->feedrate = machineState->feedrate;
	ms->fileState.CopyFrom(machineState->fileState);
	ms->lockedResources = machineState->lockedResources;
	ms->drivesRelative = machineState->drivesRelative;
	ms->axesRelative = machineState->axesRelative;
	ms->doingFileMacro = machineState->doingFileMacro;
	ms->waitWhileCooling = machineState->waitWhileCooling;
	ms->runningM502 = machineState->runningM502;
	ms->volumetricExtrusion = false;
	ms->messageAcknowledged = false;
	ms->waitingForAcknowledgement = false;
	machineState = ms;
	return true;
}

// Pop state returning true if successful (i.e. no stack underrun)
bool GCodeBuffer::PopState()
{
	GCodeMachineState * const ms = machineState;
	if (ms->previous == nullptr)
	{
		ms->messageAcknowledged = false;			// avoid getting stuck in a loop trying to pop
		ms->waitingForAcknowledgement = false;
		return false;
	}

	machineState = ms->previous;
	GCodeMachineState::Release(ms);
	return true;
}

// Return true if this source is executing a file macro
bool GCodeBuffer::IsDoingFileMacro() const
{
	return machineState->doingFileMacro;
}

// Tell this input source that any message it sent and is waiting on has been acknowledged
// Allow for the possibility that the source may have started running a macro since it started waiting
void GCodeBuffer::MessageAcknowledged(bool cancelled)
{
	for (GCodeMachineState *ms = machineState; ms != nullptr; ms = ms->previous)
	{
		if (ms->waitingForAcknowledgement)
		{
			ms->waitingForAcknowledgement = false;
			ms->messageAcknowledged = true;
			ms->messageCancelled = cancelled;
		}
	}
}

// End
