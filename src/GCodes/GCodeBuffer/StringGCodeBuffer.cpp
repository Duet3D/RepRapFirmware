/*
 * GCodeBuffer.cpp
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

//*************************************************************************************

#include "StringGCodeBuffer.h"
#include "GCodes/GCodes.h"
#include "Platform.h"
#include "RepRap.h"
#include "General/IP4String.h"

static constexpr char eofString[] = EOF_STRING;		// What's at the end of an HTML file?

StringGCodeBuffer::StringGCodeBuffer(const char* id, MessageType mt, bool usesCodeQueue)
	: GCodeBuffer(mt, usesCodeQueue), identity(id), fileBeingWritten(nullptr), writingFileSize(0),
	  eofStringCounter(0), hasCommandNumber(false), commandLetter('Q'), checksumRequired(false),
	  binaryWriting(false)
{
	// Init is called by base class
	Init();
}

void StringGCodeBuffer::Init()
{
	GCodeBuffer::Init();
	gcodeLineEnd = 0;
	commandLength = 0;
	readPointer = -1;
	hadLineNumber = hadChecksum = false;
	computedChecksum = 0;
	bufferState = GCodeBufferState::parseNotStarted;
}

void StringGCodeBuffer::Diagnostics(MessageType mtype)
{
	String<ScratchStringLength> scratchString;
	switch (bufferState)
	{
	case GCodeBufferState::parseNotStarted:
		scratchString.printf("%s is idle", GetIdentity());
		break;

	case GCodeBufferState::ready:
		scratchString.printf("%s is ready with \"%s\"", GetIdentity(), gcodeBuffer);
		break;

	case GCodeBufferState::executing:
		scratchString.printf("%s is doing \"%s\"", GetIdentity(), gcodeBuffer);
		break;

	default:
		scratchString.printf("%s is assembling a command", GetIdentity());
	}

	scratchString.cat(" in state(s)");
	const GCodeMachineState *ms = machineState;
	do
	{
		scratchString.catf(" %d", (int)ms->state);
		ms = ms->previous;
	}
	while (ms != nullptr);
	scratchString.cat('\n');
	reprap.GetPlatform().Message(mtype, scratchString.c_str());
}

inline void StringGCodeBuffer::AddToChecksum(char c)
{
	computedChecksum ^= (uint8_t)c;
}

inline void StringGCodeBuffer::StoreAndAddToChecksum(char c)
{
	computedChecksum ^= (uint8_t)c;
	if (gcodeLineEnd < ARRAY_SIZE(gcodeBuffer))
	{
		gcodeBuffer[gcodeLineEnd++] = c;
	}
}

// Add a byte to the code being assembled.  If false is returned, the code is
// not yet complete.  If true, it is complete and ready to be acted upon.
bool StringGCodeBuffer::Put(char c)
{
	if (c != 0)
	{
		++commandLength;
	}

	if (c == 0 || c == '\n' || c == '\r')
	{
		return LineFinished();
	}

	if (c == 0x7F && bufferState != GCodeBufferState::discarding)
	{
		// The UART receiver stores 0x7F in the buffer if an overrun or framing errors occurs. So discard the command and resync on the next newline.
		gcodeLineEnd = 0;
		bufferState = GCodeBufferState::discarding;
	}

	// Process the incoming character in a state machine
	bool again;
	do
	{
		again = false;
		switch (bufferState)
		{
		case GCodeBufferState::parseNotStarted:				// we haven't started parsing yet
			switch (c)
			{
			case 'N':
			case 'n':
				hadLineNumber = true;
				AddToChecksum(c);
				bufferState = GCodeBufferState::parsingLineNumber;
				lineNumber = 0;
				break;

			case ' ':
			case '\t':
				AddToChecksum(c);
				break;

			default:
				bufferState = GCodeBufferState::parsingGCode;
				commandStart = 0;
				again = true;
				break;
			}
			break;

		case GCodeBufferState::parsingLineNumber:			// we saw N at the start and we are parsing the line number
			if (isDigit(c))
			{
				AddToChecksum(c);
				lineNumber = (10 * lineNumber) + (c - '0');
				break;
			}
			else
			{
				bufferState = GCodeBufferState::parsingWhitespace;
				again = true;
			}
			break;

		case GCodeBufferState::parsingWhitespace:
			switch (c)
			{
			case ' ':
			case '\t':
				AddToChecksum(c);
				break;

			default:
				bufferState = GCodeBufferState::parsingGCode;
				commandStart = 0;
				again = true;
				break;
			}
			break;

		case GCodeBufferState::parsingGCode:				// parsing GCode words
			switch (c)
			{
			case '*':
				declaredChecksum = 0;
				hadChecksum = true;
				bufferState = GCodeBufferState::parsingChecksum;
				break;

			case ';':
				bufferState = GCodeBufferState::discarding;
				break;

			case '(':
				AddToChecksum(c);
				bufferState = GCodeBufferState::parsingBracketedComment;
				break;

			case '"':
				StoreAndAddToChecksum(c);
				bufferState = GCodeBufferState::parsingQuotedString;
				break;

			default:
				StoreAndAddToChecksum(c);
			}
			break;

		case GCodeBufferState::parsingBracketedComment:		// inside a (...) comment
			AddToChecksum(c);
			if (c == ')')
			{
				bufferState = GCodeBufferState::parsingGCode;
			}
			break;

		case GCodeBufferState::parsingQuotedString:			// inside a double-quoted string
			StoreAndAddToChecksum(c);
			if (c == '"')
			{
				bufferState = GCodeBufferState::parsingGCode;
			}
			break;

		case GCodeBufferState::parsingChecksum:				// parsing the checksum after '*'
			if (isDigit(c))
			{
				declaredChecksum = (10 * declaredChecksum) + (c - '0');
			}
			else
			{
				bufferState = GCodeBufferState::discarding;
				again = true;
			}
			break;

		case GCodeBufferState::discarding:					// discarding characters after the checksum or an end-of-line comment
		default:
			// throw the character away
			break;
		}
	} while (again);

	return false;
}

// This is called when we are fed a null, CR or LF character.
// Return true if there is a completed command ready to be executed.
bool StringGCodeBuffer::LineFinished()
{
	if (gcodeLineEnd == 0)
	{
		// Empty line
		Init();
		return false;
	}

	if (gcodeLineEnd == ARRAY_SIZE(gcodeBuffer))
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "G-Code buffer '%s' length overflow\n", GetIdentity());
		Init();
		return false;
	}

	gcodeBuffer[gcodeLineEnd] = 0;
	const bool badChecksum = (hadChecksum && computedChecksum != declaredChecksum);
	const bool missingChecksum = (checksumRequired && !hadChecksum && machineState->previous == nullptr);
	if (reprap.Debug(moduleGcodes) && fileBeingWritten == nullptr)
	{
		reprap.GetPlatform().MessageF(DebugMessage, "%s%s: %s\n", GetIdentity(), ((badChecksum) ? "(bad-csum)" : (missingChecksum) ? "(no-csum)" : ""), gcodeBuffer);
	}

	if (badChecksum)
	{
		if (hadLineNumber)
		{
			SafeSnprintf(gcodeBuffer, ARRAY_SIZE(gcodeBuffer), "M998 P%u", lineNumber);	// request resend
		}
		else
		{
			Init();
			return false;
		}
	}
	else if (missingChecksum)
	{
		// Checksum required but none was provided
		Init();
		return false;
	}

	commandStart = 0;
	DecodeCommand();
	return true;
}

// Decode this command command and find the start of the next one on the same line.
// On entry, 'commandStart' has already been set to the address the start of where the command should be.
// On return, the state must be set to 'ready' to indicate that a command is available and we should stop adding characters.
void StringGCodeBuffer::DecodeCommand()
{
	// Check for a valid command letter at the start
	const char cl = toupper(gcodeBuffer[commandStart]);
	commandFraction = -1;
	if (cl == 'G' || cl == 'M' || cl == 'T')
	{
		commandLetter = cl;
		hasCommandNumber = false;
		commandNumber = -1;
		parameterStart = commandStart + 1;
		const bool negative = (gcodeBuffer[parameterStart] == '-');
		if (negative)
		{
			++parameterStart;
		}
		if (isdigit(gcodeBuffer[parameterStart]))
		{
			hasCommandNumber = true;
			// Read the number after the command letter
			commandNumber = 0;
			do
			{
				commandNumber = (10 * commandNumber) + (gcodeBuffer[parameterStart] - '0');
				++parameterStart;
			}
			while (isdigit(gcodeBuffer[parameterStart]));
			if (negative)
			{
				commandNumber = -commandNumber;
			}

			// Read the fractional digit, if any
			if (gcodeBuffer[parameterStart] == '.')
			{
				++parameterStart;
				if (isdigit(gcodeBuffer[parameterStart]))
				{
					commandFraction = gcodeBuffer[parameterStart] - '0';
					++parameterStart;
				}
			}
		}

		// Find where the end of the command is. We assume that a G or M preceded by a space and not inside quotes is the start of a new command.
		bool inQuotes = false;
		bool primed = false;
		for (commandEnd = parameterStart; commandEnd < gcodeLineEnd; ++commandEnd)
		{
			const char c = gcodeBuffer[commandEnd];
			char c2;
			if (c == '"')
			{
				inQuotes = !inQuotes;
				primed = false;
			}
			else if (!inQuotes)
			{
				if (primed && ((c2 = toupper(c)) == 'G' || c2 == 'M'))
				{
					break;
				}
				primed = (c == ' ' || c == '\t');
			}
		}
	}
	else if (   hasCommandNumber
			 && commandLetter == 'G'
			 && commandNumber <= 3
			 && (   strchr(reprap.GetGCodes().GetAxisLetters(), cl) != nullptr
				 || ((cl == 'I' || cl == 'J') && commandNumber >= 2)
				)
			 && reprap.GetGCodes().GetMachineType() == MachineType::cnc
			)
	{
		// Fanuc-style GCode, repeat the existing G0/G1/G2/G3 command with the new parameters
		parameterStart = commandStart;
		commandEnd = gcodeLineEnd;
	}
	else
	{
		// Bad command
		commandLetter = cl;
		hasCommandNumber = false;
		commandNumber = -1;
		commandFraction = -1;
		parameterStart = commandStart;
		commandEnd = gcodeLineEnd;
	}

	bufferState = GCodeBufferState::ready;
}

// Add an entire string, overwriting any existing content and adding '\n' at the end if necessary to make it a complete line
bool StringGCodeBuffer::Put(const char *data, size_t len)
{
	Init();

	for (size_t i = 0; i < len; i++)
	{
		const char c = data[i];
		if (IsWritingBinary())
		{
			// HTML uploads are handled by the GCodes class
			WriteBinaryToFile(c);
		}
		else if (Put(c))
		{
			if (IsWritingFile())
			{
				WriteToFile();
			}

			// Code is complete or has been written to file, stop here
			return true;
		}
	}

	return Put('\n');		// in case there wasn't one at the end of the string
}

void StringGCodeBuffer::Put(const char *data)
{
	(void)Put(data, strlen(data));
}

void StringGCodeBuffer::SetFinished(bool f)
{
	if (f)
	{
		if (commandEnd < gcodeLineEnd)
		{
			// There is another command in the same line of gcode
			commandStart = commandEnd;
			DecodeCommand();
		}
		else
		{
			machineState->g53Active = false;		// G53 does not persist beyond the current line
			Init();
		}
	}
	else
	{
		bufferState = GCodeBufferState::executing;
	}
}

// Get the file position at the start of the current command
FilePosition StringGCodeBuffer::GetFilePosition(size_t bytesCached) const
{
	if (machineState->fileState.IsLive())
	{
		return machineState->fileState.GetPosition() - bytesCached - commandLength + commandStart;
	}
	return noFilePosition;
}

// Is 'c' in the G Code string? 'c' must be uppercase.
// Leave the pointer there for a subsequent read.
bool StringGCodeBuffer::Seen(char c)
{
	bool inQuotes = false;
	unsigned int inBrackets = 0;
	for (readPointer = parameterStart; (unsigned int)readPointer < commandEnd; ++readPointer)
	{
		const char b = gcodeBuffer[readPointer];
		if (b == '"')
		{
			inQuotes = !inQuotes;
		}
		else if (!inQuotes)
		{
			if (inBrackets == 0 && toupper(b) == c && (c != 'E' || (unsigned int)readPointer == parameterStart || !isdigit(gcodeBuffer[readPointer - 1])))
			{
				return true;
			}
			if (b == '[')
			{
				++inBrackets;
			}
			else if (b == ']' && inBrackets != 0)
			{
				--inBrackets;
			}
		}
	}
	readPointer = -1;
	return false;
}

// Get a float after a G Code letter found by a call to Seen()
float StringGCodeBuffer::GetFValue()
{
	if (readPointer >= 0)
	{
		const float result = ReadFloatValue(&gcodeBuffer[readPointer + 1], nullptr);
		readPointer = -1;
		return result;
	}

	INTERNAL_ERROR;
	return 0.0;
}

// Get a colon-separated list of floats after a key letter
// If doPad is true then we allow just one element to be given, in which case we fill all elements with that value
const void StringGCodeBuffer::GetFloatArray(float arr[], size_t& returnedLength, bool doPad)
{
	if (readPointer >= 0)
	{
		size_t length = 0;
		const char *p = gcodeBuffer + readPointer + 1;
		for (;;)
		{
			if (length >= returnedLength)		// array limit has been set in here
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "GCodes: Attempt to read a GCode float array that is too long: %s\n", gcodeBuffer);
				readPointer = -1;
				returnedLength = 0;
				return;
			}
			const char *q;
			arr[length] = ReadFloatValue(p, &q);
			length++;
			if (*q != LIST_SEPARATOR)
			{
				break;
			}
			p = q + 1;
		}

		// Special case if there is one entry and returnedLength requests several. Fill the array with the first entry.
		if (doPad && length == 1 && returnedLength > 1)
		{
			for (size_t i = 1; i < returnedLength; i++)
			{
				arr[i] = arr[0];
			}
		}
		else
		{
			returnedLength = length;
		}

		readPointer = -1;
	}
	else
	{
		INTERNAL_ERROR;
		returnedLength = 0;
	}
}

// Get a :-separated list of ints after a key letter
const void StringGCodeBuffer::GetIntArray(int32_t arr[], size_t& returnedLength, bool doPad)
{
	if (readPointer >= 0)
	{
		size_t length = 0;
		const char *p = gcodeBuffer + readPointer + 1;
		for (;;)
		{
			if (length >= returnedLength) // Array limit has been set in here
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "GCodes: Attempt to read a GCode int array that is too long: %s\n", gcodeBuffer);
				readPointer = -1;
				returnedLength = 0;
				return;
			}
			const char *q;
			arr[length] = ReadIValue(p, &q);
			length++;
			if (*q != LIST_SEPARATOR)
			{
				break;
			}
			p = q + 1;
		}

		// Special case if there is one entry and returnedLength requests several. Fill the array with the first entry.
		if (doPad && length == 1 && returnedLength > 1)
		{
			for (size_t i = 1; i < returnedLength; i++)
			{
				arr[i] = arr[0];
			}
		}
		else
		{
			returnedLength = length;
		}
		readPointer = -1;
	}
	else
	{
		INTERNAL_ERROR;
		returnedLength = 0;
	}
}

// Get a :-separated list of unsigned ints after a key letter
const void StringGCodeBuffer::GetUnsignedArray(uint32_t arr[], size_t& returnedLength, bool doPad)
{
	if (readPointer >= 0)
	{
		size_t length = 0;
		const char *p = gcodeBuffer + readPointer + 1;
		for (;;)
		{
			if (length >= returnedLength) // Array limit has been set in here
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "GCodes: Attempt to read a GCode unsigned array that is too long: %s\n", gcodeBuffer);
				readPointer = -1;
				returnedLength = 0;
				return;
			}
			const char *q;
			arr[length] = ReadUIValue(p, &q);
			length++;
			if (*q != LIST_SEPARATOR)
			{
				break;
			}
			p = q + 1;
		}

		// Special case if there is one entry and returnedLength requests several. Fill the array with the first entry.
		if (doPad && length == 1 && returnedLength > 1)
		{
			for (size_t i = 1; i < returnedLength; i++)
			{
				arr[i] = arr[0];
			}
		}
		else
		{
			returnedLength = length;
		}

		readPointer = -1;
	}
	else
	{
		INTERNAL_ERROR;
		returnedLength = 0;
	}
}

// Get and copy a quoted string returning true if successful
bool StringGCodeBuffer::GetQuotedString(const StringRef& str)
{
	str.Clear();
	if (readPointer >= 0)
	{
		++readPointer;				// skip the character that introduced the string
		switch (gcodeBuffer[readPointer])
		{
		case '"':
			return InternalGetQuotedString(str);

#if SUPPORT_OBJECT_MODEL
		case '[':
			return GetStringExpression(str);
#endif

		default:
			return false;
		}
	}

	INTERNAL_ERROR;
	return false;
}

// Given that the current character is double-quote, fetch the quoted string
bool StringGCodeBuffer::InternalGetQuotedString(const StringRef& str)
{
	str.Clear();
	++readPointer;
	for (;;)
	{
		char c = gcodeBuffer[readPointer++];
		if (c < ' ')
		{
			return false;
		}
		if (c == '"')
		{
			if (gcodeBuffer[readPointer++] != '"')
			{
				return true;
			}
		}
		else if (c == '\'')
		{
			if (isalpha(gcodeBuffer[readPointer]))
			{
				// Single quote before an alphabetic character forces that character to lower case
				c = tolower(gcodeBuffer[readPointer++]);
			}
			else if (gcodeBuffer[readPointer] == c)
			{
				// Two single quotes are used to represent one
				++readPointer;
			}
		}
		str.cat(c);
	}
	return false;
}

// Get and copy a string which may or may not be quoted. If it is not quoted, it ends at the first space or control character.
// Return true if successful.
bool StringGCodeBuffer::GetPossiblyQuotedString(const StringRef& str)
{
	if (readPointer >= 0)
	{
		++readPointer;
		return InternalGetPossiblyQuotedString(str);
	}

	INTERNAL_ERROR;
	return false;
}

// Get and copy a string which may or may not be quoted, starting at readPointer. Return true if successful.
bool StringGCodeBuffer::InternalGetPossiblyQuotedString(const StringRef& str)
{
	str.Clear();
	if (gcodeBuffer[readPointer] == '"')
	{
		return InternalGetQuotedString(str);
	}

#if SUPPORT_OBJECT_MODEL
	if (gcodeBuffer[readPointer] == '[')
	{
		return GetStringExpression(str);
	}
#endif

	commandEnd = gcodeLineEnd;				// the string is the remainder of the line of gcode
	for (;;)
	{
		const char c = gcodeBuffer[readPointer++];
		if (c < ' ')
		{
			break;
		}
		str.cat(c);
	}
	str.StripTrailingSpaces();
	return !str.IsEmpty();
}

// This returns a string comprising the rest of the line, excluding any comment
// It is provided for legacy use, in particular in the M23
// command that sets the name of a file to be printed.  In
// preference use GetString() which requires the string to have
// been preceded by a tag letter.
bool StringGCodeBuffer::GetUnprecedentedString(const StringRef& str)
{
	readPointer = parameterStart;
	char c;
	while ((unsigned int)readPointer < commandEnd && ((c = gcodeBuffer[readPointer]) == ' ' || c == '\t'))
	{
		++readPointer;	// skip leading spaces
	}
	return InternalGetPossiblyQuotedString(str);
}

// Get an int32 after a G Code letter
int32_t StringGCodeBuffer::GetIValue()
{
	if (readPointer >= 0)
	{
		const int32_t result = ReadIValue(&gcodeBuffer[readPointer + 1], nullptr);
		readPointer = -1;
		return result;
	}

	INTERNAL_ERROR;
	return 0;
}

// Get an uint32 after a G Code letter
uint32_t StringGCodeBuffer::GetUIValue()
{
	if (readPointer >= 0)
	{
		const uint32_t result = ReadUIValue(&gcodeBuffer[readPointer + 1], nullptr);
		readPointer = -1;
		return result;
	}

	INTERNAL_ERROR;
	return 0;
}

// Get an IP address quad after a key letter
bool StringGCodeBuffer::GetIPAddress(IPAddress& returnedIp)
{
	if (readPointer < 0)
	{
		INTERNAL_ERROR;
		return false;
	}

	const char* p = &gcodeBuffer[readPointer + 1];
	uint8_t ip[4];
	unsigned int n = 0;
	for (;;)
	{
		const char *pp;
		const unsigned long v = SafeStrtoul(p, &pp);
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
	if (n == 4)
	{
		returnedIp.SetV4(ip);
		return true;
	}
	returnedIp.SetNull();
	return false;
}

// Get a MAX address sextet after a key letter
bool StringGCodeBuffer::GetMacAddress(uint8_t mac[6])
{
	if (readPointer < 0)
	{
		INTERNAL_ERROR;
		return false;
	}

	const char* p = gcodeBuffer + readPointer + 1;
	unsigned int n = 0;
	for (;;)
	{
		const char *pp;
		const unsigned long v = SafeStrtoul(p, &pp, 16);
		if (pp == p || v > 255)
		{
			readPointer = -1;
			return false;
		}
		mac[n] = (uint8_t)v;
		++n;
		p = pp;
		if (*p != ':')
		{
			break;
		}
		if (n == 6)
		{
			readPointer = -1;
			return false;
		}
		++p;
	}
	readPointer = -1;
	return n == 6;
}

// Write the command to a string
void StringGCodeBuffer::PrintCommand(const StringRef& s) const
{
	s.printf("%c%d", commandLetter, commandNumber);
	if (commandFraction >= 0)
	{
		s.catf(".%d", commandFraction);
	}
}

// Append the full command content to a string
void StringGCodeBuffer::AppendFullCommand(const StringRef &s) const
{
	s.cat(gcodeBuffer);
}

// Open a file to write to
bool StringGCodeBuffer::OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32)
{
	fileBeingWritten = reprap.GetPlatform().OpenFile(directory, fileName, OpenMode::write);
	eofStringCounter = 0;
	writingFileSize = size;
	if (fileBeingWritten == nullptr)
	{
		return false;
	}

	crc32 = fileCRC32;
	binaryWriting = binaryWrite;
	return true;
}

// Write the current GCode to file
void StringGCodeBuffer::WriteToFile()
{
	if (GetCommandLetter() == 'M')
	{
		if (GetCommandNumber() == 29)						// end of file?
		{
			fileBeingWritten->Close();
			fileBeingWritten = nullptr;
			SetFinished(true);
			const char* const r = (reprap.GetPlatform().EmulatingMarlin()) ? "Done saving file." : "";
			reprap.GetGCodes().HandleReply(*this, GCodeResult::ok, r);
			return;
		}
	}
	else if (GetCommandLetter() == 'G' && GetCommandNumber() == 998)						// resend request?
	{
		if (Seen('P'))
		{
			SetFinished(true);
			String<ShortScratchStringLength> scratchString;
			scratchString.printf("%" PRIi32 "\n", GetIValue());
			reprap.GetGCodes().HandleReply(*this, GCodeResult::ok, scratchString.c_str());
			return;
		}
	}

	fileBeingWritten->Write(gcodeBuffer);
	fileBeingWritten->Write('\n');
	SetFinished(true);
}

void StringGCodeBuffer::WriteBinaryToFile(char b)
{
	if (b == eofString[eofStringCounter] && writingFileSize == 0)
	{
		eofStringCounter++;
		if (eofStringCounter < ARRAY_SIZE(eofString) - 1)
		{
			return;					// not reached end of input yet
		}
	}
	else
	{
		if (eofStringCounter != 0)
		{
			for (uint8_t i = 0; i < eofStringCounter; i++)
			{
				fileBeingWritten->Write(eofString[i]);
			}
			eofStringCounter = 0;
		}
		fileBeingWritten->Write(b);		// writing one character at a time isn't very efficient, but uploading HTML files via USB is rarely done these days
		if (writingFileSize == 0 || fileBeingWritten->Length() < writingFileSize)
		{
			return;					// not reached end of input yet
		}
	}

	FinishWritingBinary();
}

void StringGCodeBuffer::FinishWritingBinary()
{
	// If we get here then we have come to the end of the data
	fileBeingWritten->Close();
	const bool crcOk = (crc32 == fileBeingWritten->GetCRC32() || crc32 == 0);
	fileBeingWritten = nullptr;
	binaryWriting = false;
	if (crcOk)
	{
		const char* const r = (reprap.GetPlatform().EmulatingMarlin()) ? "Done saving file." : "";
		reprap.GetGCodes().HandleReply(*this, GCodeResult::ok, r);
	}
	else
	{
		reprap.GetGCodes().HandleReply(*this, GCodeResult::error, "CRC32 checksum doesn't match");
	}
}

// This is called when we reach the end of the file we are reading from
void StringGCodeBuffer::FileEnded()
{
	if (IsWritingBinary())
	{
		// We are in the middle of writing a binary file but the input stream has ended
		FinishWritingBinary();
	}
	else
	{
		if (gcodeLineEnd != 0)				// if there is something in the buffer
		{
			Put('\n');						// append a newline in case the file didn't end with one
		}
		if (IsWritingFile())
		{
			bool gotM29 = false;
			if (IsReady())					// if we have a complete command
			{
				gotM29 = (GetCommandLetter() == 'M' && GetCommandNumber() == 29);
				if (!gotM29)				// if it wasn't M29, write it to file
				{
					fileBeingWritten->Write(gcodeBuffer);
					fileBeingWritten->Write('\n');
				}
			}

			// Close the file whether or not we saw M29
			fileBeingWritten->Close();
			fileBeingWritten = nullptr;
			SetFinished(true);
			const char* const r = (reprap.GetPlatform().EmulatingMarlin()) ? "Done saving file." : "";
			reprap.GetGCodes().HandleReply(*this, GCodeResult::ok, r);
		}
	}
}

// Functions to read values from lines of GCode, allowing for expressions and variable substitution
float StringGCodeBuffer::ReadFloatValue(const char *p, const char **endptr)
{
#if SUPPORT_OBJECT_MODEL
	if (*p == '[')
	{
		ExpressionValue val;
		switch (EvaluateExpression(p, endptr, val))
		{
		case TYPE_OF(float):
			return val.fVal;

		case TYPE_OF(int32_t):
			return (float)val.iVal;

		case TYPE_OF(uint32_t):
			return (float)val.uVal;

		default:
			//TODO report error
			return 1.0;
		}
	}
#endif

	return SafeStrtof(p, endptr);
}

uint32_t StringGCodeBuffer::ReadUIValue(const char *p, const char **endptr)
{
#if SUPPORT_OBJECT_MODEL
	if (*p == '[')
	{
		ExpressionValue val;
		switch (EvaluateExpression(p, endptr, val))
		{
		case TYPE_OF(uint32_t):
			return val.uVal;

		case TYPE_OF(int32_t):
			if (val.iVal >= 0)
			{
				return (uint32_t)val.iVal;
			}
			//TODO report error
			return 0;

		default:
			//TODO report error
			return 0;
		}
	}
#endif

	int base = 10;
	size_t skipTrailingQuote = 0;

	// Allow "0xNNNN" or "xNNNN" where NNNN are hex digits
	if (*p == '"')
	{
		++p;
		skipTrailingQuote = 1;
		switch (*p)
		{
		case 'x':
		case 'X':
			base = 16;
			++p;
			break;

		case '0':
			if (*(p + 1) == 'x' || *(p + 1) == 'X')
			{
				base = 16;
				p += 2;
			}
			break;

		default:
			break;
		}
	}
	const uint32_t result = SafeStrtoul(p, endptr, base);
	endptr += skipTrailingQuote;
	return result;
}

int32_t StringGCodeBuffer::ReadIValue(const char *p, const char **endptr)
{
#if SUPPORT_OBJECT_MODEL
	if (*p == '[')
	{
		ExpressionValue val;
		switch (EvaluateExpression(p, endptr, val))
		{
		case TYPE_OF(int32_t):
			return val.iVal;

		case TYPE_OF(uint32_t):
			return (int32_t)val.uVal;

		default:
			//TODO report error
			return 0;
		}
	}
#endif

	return SafeStrtol(p, endptr);
}

#if SUPPORT_OBJECT_MODEL

// Get a string expression. The current character is '['.
bool StringGCodeBuffer::GetStringExpression(const StringRef& str)
{
	ExpressionValue val;
	switch (EvaluateExpression(gcodeBuffer + readPointer, nullptr, val))
	{
	case TYPE_OF(const char*):
		str.copy(val.sVal);
		break;

	case TYPE_OF(float):
		str.printf("%.1f", (double)val.fVal);
		break;

	case TYPE_OF(Float2):
		str.printf("%.2f", (double)val.fVal);
		break;

	case TYPE_OF(Float3):
		str.printf("%.3f", (double)val.fVal);
		break;

	case TYPE_OF(uint32_t):
		str.printf("%" PRIu32, val.uVal);			// convert unsigned integer to string
		break;

	case TYPE_OF(int32_t):
		str.printf("%" PRIi32, val.uVal);			// convert signed integer to string
		break;

	case TYPE_OF(bool):
		str.copy((val.bVal) ? "true" : "false");	// convert bool to string
		break;

	case TYPE_OF(IPAddress):
		str.copy(IP4String(val.uVal).c_str());
		break;

	default:
		//TODO report error
		return false;
	}

	return true;
}

// Evaluate an expression. the current character is '['.
TypeCode StringGCodeBuffer::EvaluateExpression(const char *p, const char **endptr, ExpressionValue& rslt)
{
	++p;						// skip the '['
	// For now the only form of expression we handle is [variable-name]
	if (isalpha(*p))			// if it's a variable name
	{
		const char * const start = p;
		unsigned int numBrackets = 0;
		while (isalpha(*p) || isdigit(*p) || *p == '_' || *p == '.' || *p == '[' || (*p == ']' && numBrackets != 0))
		{
			if (*p == '[')
			{
				++numBrackets;
			}
			else if (*p == ']')
			{
				-- numBrackets;
			}
			++p;
		}
		String<MaxVariableNameLength> varName;
		if (varName.copy(start, p - start))
		{
			// variable name is too long
			//TODO error handling
			return NoType;
		}
		//TODO consider supporting standard CNC functions here
		const TypeCode tc = reprap.GetObjectValue(rslt, varName.c_str());
		if (tc != NoType && (tc & IsArray) == 0 && *p == ']')
		{
			if (endptr != nullptr)
			{
				*endptr = p + 1;
			}
			return tc;
		}
	}
	return NoType;
}

#endif

// End
