/*
 * StringParser.cpp
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

//*************************************************************************************

#include "StringParser.h"
#include "GCodeBuffer.h"

#include "GCodes/GCodes.h"
#include "Platform.h"
#include "RepRap.h"
#include <General/IP4String.h>
#include <General/StringBuffer.h>
#include <Networking/NetworkDefs.h>

// Replace the default definition of THROW_INTERNAL_ERROR by one that gives line information
#undef THROW_INTERNAL_ERROR
#define THROW_INTERNAL_ERROR	throw ConstructParseException("internal error at file " __FILE__ "(%d)", __LINE__)

#if HAS_MASS_STORAGE
static constexpr char eofString[] = EOF_STRING;		// What's at the end of an HTML file?
#endif

StringParser::StringParser(GCodeBuffer& gcodeBuffer) noexcept
	: gb(gcodeBuffer), fileBeingWritten(nullptr), writingFileSize(0), eofStringCounter(0), indentToSkipTo(NoIndentSkip),
	  hasCommandNumber(false), commandLetter('Q'), checksumRequired(false), binaryWriting(false)
{
	StartNewFile();
	Init();
}

void StringParser::Init() noexcept
{
	gcodeLineEnd = 0;
	commandLength = 0;
	readPointer = -1;
	hadLineNumber = hadChecksum = overflowed = false;
	computedChecksum = 0;
	gb.bufferState = GCodeBufferState::parseNotStarted;
	commandIndent = 0;
	if (!seenMetaCommand)
	{
		seenLeadingSpace = seenLeadingTab = false;
	}
}

inline void StringParser::AddToChecksum(char c) noexcept
{
	computedChecksum ^= (uint8_t)c;
}

inline void StringParser::StoreAndAddToChecksum(char c) noexcept
{
	computedChecksum ^= (uint8_t)c;
	if (gcodeLineEnd + 1 < ARRAY_SIZE(gb.buffer))			// if there is space for this characate and a trailing null
	{
		gb.buffer[gcodeLineEnd++] = c;
	}
	else
	{
		overflowed = true;
	}
}

// Add a byte to the code being assembled.  If false is returned, the code is
// not yet complete.  If true, it is complete and ready to be acted upon and 'indent'
// is the number of leading white space characters..
bool StringParser::Put(char c) noexcept
{
	if (c != 0)
	{
		++commandLength;
	}

	if (c == 0 || c == '\n' || c == '\r')
	{
		return LineFinished();
	}

	if (c == 0x7F && gb.bufferState != GCodeBufferState::discarding)
	{
		// The UART receiver stores 0x7F in the buffer if an overrun or framing errors occurs. So discard the command and resync on the next newline.
		gcodeLineEnd = 0;
		gb.bufferState = GCodeBufferState::discarding;
	}

	// Process the incoming character in a state machine
	bool again;
	do
	{
		again = false;
		switch (gb.bufferState)
		{
		case GCodeBufferState::parseNotStarted:				// we haven't started parsing yet
			braceCount = 0;
			switch (c)
			{
			case 'N':
			case 'n':
				hadLineNumber = true;
				AddToChecksum(c);
				gb.bufferState = GCodeBufferState::parsingLineNumber;
				receivedLineNumber = 0;
				break;

			case ' ':
				AddToChecksum(c);
				++commandIndent;
				seenLeadingSpace = true;
				break;

			case '\t':
				AddToChecksum(c);
				commandIndent = (commandIndent + 4) & ~3;	// move on at least 1 to next multiple of 4
				seenLeadingTab = true;
				break;

			default:
				gb.bufferState = GCodeBufferState::parsingGCode;
				commandStart = 0;
				again = true;
				break;
			}
			break;

		case GCodeBufferState::parsingLineNumber:			// we saw N at the start and we are parsing the line number
			if (isDigit(c))
			{
				AddToChecksum(c);
				receivedLineNumber = (10 * receivedLineNumber) + (c - '0');
				break;
			}
			else
			{
				gb.bufferState = GCodeBufferState::parsingWhitespace;
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
				gb.bufferState = GCodeBufferState::parsingGCode;
				commandStart = 0;
				again = true;
				break;
			}
			break;

		case GCodeBufferState::parsingGCode:				// parsing GCode words
			switch (c)
			{
			case '*':
				if (braceCount == 0)
				{
					declaredChecksum = 0;
					hadChecksum = true;
					gb.bufferState = GCodeBufferState::parsingChecksum;
				}
				else
				{
					StoreAndAddToChecksum(c);
				}
				break;

			case ';':
				gb.bufferState = GCodeBufferState::discarding;
				break;

			case '(':
				if (braceCount == 0)
				{
					AddToChecksum(c);
					gb.bufferState = GCodeBufferState::parsingBracketedComment;
				}
				else
				{
					StoreAndAddToChecksum(c);
				}
				break;

			case '"':
				StoreAndAddToChecksum(c);
				gb.bufferState = GCodeBufferState::parsingQuotedString;
				break;

			case '{':
				++braceCount;
				StoreAndAddToChecksum(c);
				break;

			case '}':
				if (braceCount != 0)
				{
					--braceCount;
				}
				StoreAndAddToChecksum(c);
				break;

			default:
				StoreAndAddToChecksum(c);
			}
			break;

		case GCodeBufferState::parsingBracketedComment:		// inside a (...) comment
			AddToChecksum(c);
			if (c == ')')
			{
				gb.bufferState = GCodeBufferState::parsingGCode;
			}
			break;

		case GCodeBufferState::parsingQuotedString:			// inside a double-quoted string
			StoreAndAddToChecksum(c);
			if (c == '"')
			{
				gb.bufferState = GCodeBufferState::parsingGCode;
			}
			break;

		case GCodeBufferState::parsingChecksum:				// parsing the checksum after '*'
			if (isDigit(c))
			{
				declaredChecksum = (10 * declaredChecksum) + (c - '0');
			}
			else
			{
				gb.bufferState = GCodeBufferState::discarding;
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
bool StringParser::LineFinished()
{
	if (hadLineNumber)
	{
		gb.machineState->lineNumber = receivedLineNumber;
	}
	else
	{
		++gb.machineState->lineNumber;
	}

	if (gcodeLineEnd == 0)
	{
		// Empty line
		Init();
		return false;
	}

	gb.buffer[gcodeLineEnd] = 0;
	const bool badChecksum = (hadChecksum && computedChecksum != declaredChecksum);
	const bool missingChecksum = (checksumRequired && !hadChecksum && gb.machineState->previous == nullptr);
	if (reprap.Debug(moduleGcodes) && fileBeingWritten == nullptr)
	{
		reprap.GetPlatform().MessageF(DebugMessage, "%s%s: %s\n", gb.GetIdentity(), ((badChecksum) ? "(bad-csum)" : (missingChecksum) ? "(no-csum)" : ""), gb.buffer);
	}

	commandStart = 0;
	return true;
}

// Check whether the current command is a meta command, or we are skipping commands in a block
// Return true if the current line no longer needs to be processed
bool StringParser::CheckMetaCommand(const StringRef& reply)
{
	if (overflowed)
	{
		throw GCodeException(gb.MachineState().lineNumber, ARRAY_SIZE(gb.buffer) - 1, "GCode command too long");
	}

	const bool doingFile = gb.IsDoingFile();
	BlockType skippedBlockType = BlockType::plain;
	if (doingFile)
	{
		// Deal with warning about mixed spaces and tabs
		if (commandIndent == 0)
		{
			seenLeadingSpace = seenLeadingTab = false;						// it's OK if the previous block used only space and the following one uses only tab, or v.v.
		}
		else
		{
			CheckForMixedSpacesAndTabs();
		}

		// Deal with skipping blocks
		if (indentToSkipTo != NoIndentSkip)
		{
			if (indentToSkipTo < commandIndent)
			{
				Init();
				return true;												// continue skipping this block
			}
			else
			{
				// Finished skipping the nested block
				if (indentToSkipTo == commandIndent)
				{
					skippedBlockType = gb.machineState->CurrentBlockState().GetType();	// save the type of the block we skipped in case the command is 'else' or 'elif'
					gb.machineState->CurrentBlockState().SetPlainBlock();	// we've ended the loop or if-block
				}
				indentToSkipTo = NoIndentSkip;								// no longer skipping
			}
		}

		while (commandIndent < gb.machineState->CurrentBlockIndent())
		{
			gb.machineState->EndBlock();
			if (gb.machineState->CurrentBlockState().GetType() == BlockType::loop)
			{
				// Go back to the start of the loop and re-evaluate the while-part
				gb.machineState->lineNumber = gb.machineState->CurrentBlockState().GetLineNumber();
				gb.RestartFrom(gb.machineState->CurrentBlockState().GetFilePosition());
				Init();
				return true;
			}
		}

		if (commandIndent > gb.machineState->CurrentBlockIndent())
		{
			// Indentation has increased so start new block(s)
			if (!gb.machineState->CreateBlock(commandIndent))
			{
				throw ConstructParseException("blocks nested too deeply");
			}
		}
	}

	const bool b = ProcessConditionalGCode(reply, skippedBlockType, doingFile);	// this may throw a ParseException
	if (b)
	{
		seenMetaCommand = true;
		if (doingFile)
		{
			CheckForMixedSpacesAndTabs();
		}
		Init();
	}

	return b;
}

void StringParser::CheckForMixedSpacesAndTabs() noexcept
{
	if (seenMetaCommand && !warnedAboutMixedSpacesAndTabs && seenLeadingSpace && seenLeadingTab)
	{
		reprap.GetPlatform().MessageF(AddWarning(gb.GetResponseMessageType()),
								"both space and tab characters used to indent blocks by line %" PRIu32, gb.MachineState().lineNumber);
		warnedAboutMixedSpacesAndTabs = true;
	}
}

// Check for and process a conditional GCode language command returning true if we found one, false if it's a regular line of GCode that we need to process
// If we just finished skipping an if- or elif-block when the condition was false then 'skippedBlockType' is the type of that block, else it is BlockType::plain
bool StringParser::ProcessConditionalGCode(const StringRef& reply, BlockType skippedBlockType, bool doingFile)
{
	// First count the number of lowercase characters.
	unsigned int i = 0;
	while (gb.buffer[i] >= 'a' && gb.buffer[i] <= 'z')
	{
		++i;
		if (i == 9)
		{
			break;				// all command words are less than 9 characters long
		}
	}

	if (i >= 2 && i < 9 && (gb.buffer[i] == 0 || gb.buffer[i] == ' ' || gb.buffer[i] == '\t' || gb.buffer[i] == '{'))		// if the command word is properly terminated
	{
		readPointer = i;
		const char * const command = gb.buffer;
		switch (i)
		{
		case 2:
			if (doingFile && StringStartsWith(command, "if"))
			{
				ProcessIfCommand();
				return true;
			}
			break;

		case 3:
			if (doingFile)
			{
				if (StringStartsWith(command, "var"))
				{
					ProcessVarCommand();
					return true;
				}
				if (StringStartsWith(command, "set"))
				{
					ProcessSetCommand();
					return true;
				}
			}
			break;

		case 4:
			if (doingFile)
			{
				if (StringStartsWith(command, "else"))
				{
					ProcessElseCommand(skippedBlockType);
					return true;
				}
				if (StringStartsWith(command, "elif"))
				{
					ProcessElifCommand(skippedBlockType);
					return true;
				}
			}
			if (StringStartsWith(command, "echo"))
			{
				ProcessEchoCommand(reply);
				return true;
			}
			break;

		case 5:
			if (doingFile)
			{
				if (StringStartsWith(command, "while"))
				{
					ProcessWhileCommand();
					return true;
				}
				if (StringStartsWith(command, "break"))
				{
					ProcessBreakCommand();
					return true;
				}
				if (StringStartsWith(command, "abort"))
				{
					ProcessAbortCommand(reply);
					return true;
				}
			}
			break;

		case 8:
			if (doingFile && StringStartsWith(command, "continue"))
			{
				ProcessContinueCommand();
				return true;
			}
			break;

		default:
			break;
		}
	}

	readPointer = -1;
	return false;
}

void StringParser::ProcessIfCommand()
{
	if (EvaluateCondition())
	{
		gb.machineState->CurrentBlockState().SetIfTrueBlock();
	}
	else
	{
		gb.machineState->CurrentBlockState().SetIfFalseNoneTrueBlock();
		indentToSkipTo = gb.machineState->CurrentBlockIndent();			// skip forwards to the end of the block
	}
}

void StringParser::ProcessElseCommand(BlockType skippedBlockType)
{
	if (skippedBlockType == BlockType::ifFalseNoneTrue)
	{
		gb.machineState->CurrentBlockState().SetPlainBlock();			// execute the else-block, treating it like a plain block
	}
	else if (skippedBlockType == BlockType::ifFalseHadTrue || gb.machineState->CurrentBlockState().GetType() == BlockType::ifTrue)
	{
		indentToSkipTo = gb.machineState->CurrentBlockIndent();			// skip forwards to the end of the if-block
		gb.machineState->CurrentBlockState().SetPlainBlock();			// so that we get an error if there is another 'else' part
	}
	else
	{
		throw ConstructParseException("'else' did not follow 'if'");
	}
}

void StringParser::ProcessElifCommand(BlockType skippedBlockType)
{
	if (skippedBlockType == BlockType::ifFalseNoneTrue)
	{
		if (EvaluateCondition())
		{
			gb.machineState->CurrentBlockState().SetIfTrueBlock();
		}
		else
		{
			indentToSkipTo = gb.machineState->CurrentBlockIndent();		// skip forwards to the end of the elif-block
			gb.machineState->CurrentBlockState().SetIfFalseNoneTrueBlock();
		}
	}
	else if (skippedBlockType == BlockType::ifFalseHadTrue || gb.machineState->CurrentBlockState().GetType() == BlockType::ifTrue)
	{
		indentToSkipTo = gb.machineState->CurrentBlockIndent();			// skip forwards to the end of the if-block
		gb.machineState->CurrentBlockState().SetIfFalseHadTrueBlock();
	}
	else
	{
		throw ConstructParseException("'elif' did not follow 'if");
	}
}

void StringParser::ProcessWhileCommand()
{
	// Set the current block as a loop block first so that we may use 'iterations' in the condition
	if (gb.machineState->CurrentBlockState().GetType() == BlockType::loop)
	{
		gb.machineState->CurrentBlockState().IncrementIterations();		// starting another iteration
	}
	else
	{
		gb.machineState->CurrentBlockState().SetLoopBlock(GetFilePosition(), gb.machineState->lineNumber);
	}

	if (!EvaluateCondition())
	{
		gb.machineState->CurrentBlockState().SetPlainBlock();
		indentToSkipTo = gb.machineState->CurrentBlockIndent();			// skip forwards to the end of the block
	}
}

void StringParser::ProcessBreakCommand()
{
	do
	{
		if (gb.machineState->CurrentBlockIndent() == 0)
		{
			throw ConstructParseException("'break' was not inside a loop");
		}
		gb.machineState->EndBlock();
	} while (gb.machineState->CurrentBlockState().GetType() != BlockType::loop);
	gb.machineState->CurrentBlockState().SetPlainBlock();
	indentToSkipTo = gb.machineState->CurrentBlockIndent();				// skip forwards to the end of the loop
}

void StringParser::ProcessContinueCommand()
{
	do
	{
		if (gb.machineState->CurrentBlockIndent() == 0)
		{
			throw ConstructParseException("'continue' was not inside a loop");
		}
		gb.machineState->EndBlock();
	} while (gb.machineState->CurrentBlockState().GetType() != BlockType::loop);

	// Go back to the start of the loop and re-evaluate the while-part
	gb.machineState->lineNumber = gb.machineState->CurrentBlockState().GetLineNumber();
	gb.RestartFrom(gb.machineState->CurrentBlockState().GetFilePosition());
}

void StringParser::ProcessVarCommand()
{
#if 0
	SkipWhiteSpace();
	String<MaxVariableNameLength> varName;
	ParseIdentifier(varName.GetRef());
	qq;
#endif
	throw ConstructParseException("'var' not implemented");
}

void StringParser::ProcessSetCommand()
{
	throw ConstructParseException("'set' not implemented");
}

void StringParser::ProcessAbortCommand(const StringRef& reply) noexcept
{
	SkipWhiteSpace();
	if (gb.buffer[readPointer] != 0)
	{
		// If we fail to parse the expression, we want to abort anyway
		try
		{
			char stringBuffer[StringBufferLength];
			StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
			const ExpressionValue val = ParseExpression(bufRef, 0, true);
			AppendAsString(val, reply);
		}
		catch (const GCodeException& e)
		{
			e.GetMessage(reply, gb);
			reply.Insert(0, "invalid expression after 'abort': ");
		}
	}
	else
	{
		reply.copy("'abort' command executed");
	}

	reprap.GetGCodes().AbortPrint(gb);
}

void StringParser::ProcessEchoCommand(const StringRef& reply)
{
	while (true)
	{
		SkipWhiteSpace();
		if (gb.buffer[readPointer] == 0)
		{
			return;
		}
		char stringBuffer[StringBufferLength];
		StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
		const ExpressionValue val = ParseExpression(bufRef, 0, true);
		if (!reply.IsEmpty())
		{
			reply.cat(' ');
		}
		AppendAsString(val, reply);
		SkipWhiteSpace();
		if (gb.buffer[readPointer] == ',')
		{
			++readPointer;
		}
		else if (gb.buffer[readPointer] != 0)
		{
			throw ConstructParseException("expected ','");
		}
	}
}

// Evaluate the condition that should follow 'if' or 'while'
bool StringParser::EvaluateCondition()
{
	char stringBuffer[StringBufferLength];
	StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
	ExpressionValue val = ParseExpression(bufRef, 0, true);
	SkipWhiteSpace();
	if (gb.buffer[readPointer] != 0)
	{
		throw ConstructParseException("unexpected characters following condition");
	}
	ConvertToBool(val, true);
	return val.bVal;
}

// Decode this command and find the start of the next one on the same line.
// On entry, 'commandStart' has already been set to the address the start of where the command should be
// and 'commandIndent' is the number of leading whitespace characters at the start of the current line.
// On return, the state must be set to 'ready' to indicate that a command is available and we should stop adding characters.
void StringParser::DecodeCommand() noexcept
{
	// Check for a valid command letter at the start
	const char cl = toupper(gb.buffer[commandStart]);
	commandFraction = -1;
	if (cl == 'G' || cl == 'M' || cl == 'T')
	{
		commandLetter = cl;
		hasCommandNumber = false;
		commandNumber = -1;
		parameterStart = commandStart + 1;
		const bool negative = (gb.buffer[parameterStart] == '-');
		if (negative)
		{
			++parameterStart;
		}
		if (isdigit(gb.buffer[parameterStart]))
		{
			hasCommandNumber = true;
			// Read the number after the command letter
			commandNumber = 0;
			do
			{
				commandNumber = (10 * commandNumber) + (gb.buffer[parameterStart] - '0');
				++parameterStart;
			}
			while (isdigit(gb.buffer[parameterStart]));
			if (negative)
			{
				commandNumber = -commandNumber;
			}

			// Read the fractional digit, if any
			if (gb.buffer[parameterStart] == '.')
			{
				++parameterStart;
				if (isdigit(gb.buffer[parameterStart]))
				{
					commandFraction = gb.buffer[parameterStart] - '0';
					++parameterStart;
				}
			}
		}

		// Find where the end of the command is. We assume that a G or M preceded by a space and not inside quotes or { } is the start of a new command.
		bool inQuotes = false;
		unsigned int braceCount = 0;
		bool primed = false;
		for (commandEnd = parameterStart; commandEnd < gcodeLineEnd; ++commandEnd)
		{
			const char c = gb.buffer[commandEnd];
			if (c == '"')
			{
				inQuotes = !inQuotes;
				primed = false;
			}
			else if (!inQuotes)
			{
				char c2;
				if (c == '{')
				{
					++braceCount;
					primed = false;
				}
				else if (c == '}')
				{
					if (braceCount != 0)
					{
						--braceCount;
					}
					primed = false;
				}
				else if (primed && ((c2 = toupper(c)) == 'G' || c2 == 'M'))
				{
					break;
				}
				else if (braceCount == 0)
				{
					primed = (c == ' ' || c == '\t');
				}
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
			 && !isalpha(gb.buffer[commandStart + 1])			// make sure it isn't an if-command or other meta command
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

	gb.bufferState = GCodeBufferState::ready;
}

// Add an entire string, overwriting any existing content and adding '\n' at the end if necessary to make it a complete line
void StringParser::PutAndDecode(const char *str, size_t len) noexcept
{
	Init();
	for (size_t i = 0; i < len; i++)
	{
		if (Put(str[i]))	// if the line is complete
		{
			DecodeCommand();
			return;
		}
	}

	(void)Put('\n');		// because there wasn't one at the end of the string
	DecodeCommand();
}

void StringParser::PutAndDecode(const char *str) noexcept
{
	PutAndDecode(str, strlen(str));
}

void StringParser::SetFinished() noexcept
{
	if (commandEnd < gcodeLineEnd)
	{
		// There is another command in the same line of gcode
		commandStart = commandEnd;
		DecodeCommand();
	}
	else
	{
		gb.machineState->g53Active = false;		// G53 does not persist beyond the current line
		Init();
	}
}

// Get the file position at the start of the current command
FilePosition StringParser::GetFilePosition() const noexcept
{
#if HAS_MASS_STORAGE
	if (gb.machineState->DoingFile()
# if HAS_LINUX_INTERFACE
		&& !reprap.UsingLinuxInterface()
# endif
	   )
	{
		return gb.machineState->fileState.GetPosition() - gb.fileInput->BytesCached() - commandLength + commandStart;
	}
#endif
	return noFilePosition;
}

const char* StringParser::DataStart() const noexcept
{
	return gb.buffer + commandStart;
}

size_t StringParser::DataLength() const noexcept
{
	return commandEnd - commandStart;
}

// Is 'c' in the G Code string? 'c' must be uppercase.
// Leave the pointer one after it for a subsequent read.
bool StringParser::Seen(char c) noexcept
{
	bool inQuotes = false;
	unsigned int inBrackets = 0;
	for (readPointer = parameterStart; (unsigned int)readPointer < commandEnd; ++readPointer)
	{
		const char b = gb.buffer[readPointer];
		if (b == '"')
		{
			inQuotes = !inQuotes;
		}
		else if (!inQuotes)
		{
			if (inBrackets == 0 && toupper(b) == c && (c != 'E' || (unsigned int)readPointer == parameterStart || !isdigit(gb.buffer[readPointer - 1])))
			{
				++readPointer;
				return true;
			}
			if (b == '{')
			{
				++inBrackets;
			}
			else if (b == '}' && inBrackets != 0)
			{
				--inBrackets;
			}
		}
	}
	readPointer = -1;
	return false;
}

// Get a float after a G Code letter found by a call to Seen()
float StringParser::GetFValue()
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	const float result = ReadFloatValue();
	readPointer = -1;
	return result;
}

// Get a colon-separated list of floats after a key letter
// If doPad is true then we allow just one element to be given, in which case we fill all elements with that value
void StringParser::GetFloatArray(float arr[], size_t& returnedLength, bool doPad)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	size_t length = 0;
	for (;;)
	{
		if (length >= returnedLength)		// array limit has been set in here
		{
			throw ConstructParseException("array too long, max length = %u", (uint32_t)returnedLength);
		}
		arr[length++] = ReadFloatValue();
		if (gb.buffer[readPointer] != LIST_SEPARATOR)
		{
			break;
		}
		++readPointer;
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

// Get a :-separated list of ints after a key letter
void StringParser::GetIntArray(int32_t arr[], size_t& returnedLength, bool doPad)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	size_t length = 0;
	for (;;)
	{
		if (length >= returnedLength) // Array limit has been set in here
		{
			throw ConstructParseException("array too long, max length = %u", (uint32_t)returnedLength);
		}
		arr[length] = ReadIValue();
		length++;
		if (gb.buffer[readPointer] != LIST_SEPARATOR)
		{
			break;
		}
		++readPointer;
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

// Get a :-separated list of unsigned ints after a key letter
void StringParser::GetUnsignedArray(uint32_t arr[], size_t& returnedLength, bool doPad)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	size_t length = 0;
	for (;;)
	{
		if (length >= returnedLength) // Array limit has been set in here
		{
			throw ConstructParseException("array too long, max length = %u", (uint32_t)returnedLength);
		}
		arr[length] = ReadUIValue();
		length++;
		if (gb.buffer[readPointer] != LIST_SEPARATOR)
		{
			break;
		}
		++readPointer;
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

// Get a :-separated list of drivers after a key letter
void StringParser::GetDriverIdArray(DriverId arr[], size_t& returnedLength)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	size_t length = 0;
	for (;;)
	{
		if (length >= returnedLength) // Array limit has been set in here
		{
			throw ConstructParseException("array too long, max length = %u", (uint32_t)returnedLength);
		}
		arr[length] = ReadDriverIdValue();
		length++;
		if (gb.buffer[readPointer] != LIST_SEPARATOR)
		{
			break;
		}
		++readPointer;
	}

	returnedLength = length;
	readPointer = -1;
}

// Get and copy a quoted string returning true if successful
void StringParser::GetQuotedString(const StringRef& str, bool allowEmpty)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	str.Clear();
	switch (gb.buffer[readPointer])
	{
	case '"':
		InternalGetQuotedString(str);
		break;

	case '{':
		++readPointer;									// skip the '{'
		{
			char stringBuffer[StringBufferLength];
			StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
			AppendAsString(ParseBracketedExpression(bufRef, '}', true), str);
		}
		break;

	default:
		throw ConstructParseException("expected string expression");
	}

	if (!allowEmpty && str.IsEmpty())
	{
		throw ConstructParseException("non-empty string expected");
	}
}

// Given that the current character is double-quote, fetch the quoted string
void StringParser::InternalGetQuotedString(const StringRef& str)
{
	str.Clear();
	++readPointer;
	for (;;)
	{
		char c = gb.buffer[readPointer++];
		if (c < ' ')
		{
			throw ConstructParseException("control character in string");
		}
		if (c == '"')
		{
			if (gb.buffer[readPointer] != c)
			{
				return;
			}
			++readPointer;
		}
		else if (c == '\'')
		{
			if (isalpha(gb.buffer[readPointer]))
			{
				// Single quote before an alphabetic character forces that character to lower case
				c = tolower(gb.buffer[readPointer++]);
			}
			else if (gb.buffer[readPointer] == c)
			{
				// Two backslashes are used to represent one
				++readPointer;
			}
		}
		if (str.cat(c))
		{
			throw ConstructParseException("string too long");
		}
	}
}

// Get and copy a string which may or may not be quoted. If it is not quoted, it ends at the first space or control character.
void StringParser::GetPossiblyQuotedString(const StringRef& str, bool allowEmpty)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	InternalGetPossiblyQuotedString(str);
	if (!allowEmpty && str.IsEmpty())
	{
		throw ConstructParseException("non-empty string expected");
	}
}

// Get and copy a string which may or may not be quoted, starting at readPointer. Return true if successful.
void StringParser::InternalGetPossiblyQuotedString(const StringRef& str)
{
	str.Clear();
	if (gb.buffer[readPointer] == '"')
	{
		InternalGetQuotedString(str);
	}
	else if (gb.buffer[readPointer] == '{')
	{
		++readPointer;									// skip the '{'
		char stringBuffer[StringBufferLength];
		StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
		AppendAsString(ParseBracketedExpression(bufRef, '}', true), str);
	}
	else
	{
		commandEnd = gcodeLineEnd;				// the string is the remainder of the line of gcode
		for (;;)
		{
			const char c = gb.buffer[readPointer++];
			if (c < ' ')
			{
				break;
			}
			str.cat(c);
		}
		str.StripTrailingSpaces();
	}
}

void StringParser::GetReducedString(const StringRef& str)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	// Reduced strings must start with a double-quote
	if (gb.buffer[readPointer] != '"')
	{
		throw ConstructParseException("string expected");
	}

	++readPointer;
	str.Clear();
	for (;;)
	{
		const char c = gb.buffer[readPointer++];
		switch(c)
		{
		case '"':
			if (gb.buffer[readPointer++] != '"')
			{
				if (str.IsEmpty())
				{
					throw ConstructParseException("non-empty string expected");
				}
				return;
			}
			str.cat(c);
			break;

		case '_':
		case '-':
		case ' ':
			break;

		default:
			if (c < ' ')
			{
				throw ConstructParseException("control character in string");
			}
			str.cat(tolower(c));
			break;
		}
	}
}

// This returns a string comprising the rest of the line, excluding any comment
// It is provided for legacy use, in particular in the M23
// command that sets the name of a file to be printed.  In
// preference use GetString() which requires the string to have
// been preceded by a tag letter.
void StringParser::GetUnprecedentedString(const StringRef& str, bool allowEmpty)
{
	readPointer = parameterStart;
	char c;
	while ((unsigned int)readPointer < commandEnd && ((c = gb.buffer[readPointer]) == ' ' || c == '\t'))
	{
		++readPointer;	// skip leading spaces
	}

	InternalGetPossiblyQuotedString(str);
	if (!allowEmpty && str.IsEmpty())
	{
		throw ConstructParseException("non-empty string expected");
	}
}

// Get an int32 after a G Code letter
int32_t StringParser::GetIValue()
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	const int32_t result = ReadIValue();
	readPointer = -1;
	return result;
}

// Get an uint32 after a G Code letter
uint32_t StringParser::GetUIValue()
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	const uint32_t result = ReadUIValue();
	readPointer = -1;
	return result;
}

// Get a driver ID
DriverId StringParser::GetDriverId()
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	DriverId result = ReadDriverIdValue();
	readPointer = -1;
	return result;
}

// Get an IP address quad after a key letter
void StringParser::GetIPAddress(IPAddress& returnedIp)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	const char* p = gb.buffer + readPointer;
	uint8_t ip[4];
	unsigned int n = 0;
	for (;;)
	{
		const char *pp;
		const unsigned long v = SafeStrtoul(p, &pp);
		if (pp == p || v > 255)
		{
			readPointer = -1;
			throw ConstructParseException("invalid IP address");
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
			throw ConstructParseException("invalid IP address");
		}
		++p;
	}
	readPointer = -1;
	if (n != 4)
	{
		throw ConstructParseException("invalid IP address");
	}
	returnedIp.SetV4(ip);
}

// Get a MAC address sextet after a key letter
void StringParser::GetMacAddress(MacAddress& mac)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	const char* p = gb.buffer + readPointer;
	unsigned int n = 0;
	for (;;)
	{
		const char *pp;
		const unsigned long v = SafeStrtoul(p, &pp, 16);
		if (pp == p || v > 255)
		{
			readPointer = -1;
			throw ConstructParseException("invalid MAC address");
		}
		mac.bytes[n] = (uint8_t)v;
		++n;
		p = pp;
		if (*p != ':')
		{
			break;
		}
		if (n == 6)
		{
			readPointer = -1;
			throw ConstructParseException("invalid MAC address");
		}
		++p;
	}
	readPointer = -1;
	if (n != 6)
	{
		throw ConstructParseException("invalid MAC address");
	}
}

// Write the command to a string
void StringParser::PrintCommand(const StringRef& s) const noexcept
{
	s.printf("%c%d", commandLetter, commandNumber);
	if (commandFraction >= 0)
	{
		s.catf(".%d", commandFraction);
	}
}

// Append the full command content to a string
void StringParser::AppendFullCommand(const StringRef &s) const noexcept
{
	s.cat(gb.buffer);
}

#if HAS_MASS_STORAGE

// Open a file to write to
bool StringParser::OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) noexcept
{
	fileBeingWritten = reprap.GetPlatform().OpenFile(directory, fileName, OpenMode::writeWithCrc);
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

// Write the current line of GCode to file
void StringParser::WriteToFile() noexcept
{
	DecodeCommand();
	if (GetCommandLetter() == 'M')
	{
		if (GetCommandNumber() == 29)						// end of file?
		{
			fileBeingWritten->Close();
			fileBeingWritten = nullptr;
			Init();
			const char* const r = (gb.MachineState().compatibility == Compatibility::marlin) ? "Done saving file." : "";
			reprap.GetGCodes().HandleReply(gb, GCodeResult::ok, r);
			return;
		}
	}
	else if (GetCommandLetter() == 'G' && GetCommandNumber() == 998)						// resend request?
	{
		if (Seen('P'))
		{
			Init();
			String<StringLength20> scratchString;
			scratchString.printf("%" PRIi32 "\n", GetIValue());
			reprap.GetGCodes().HandleReply(gb, GCodeResult::ok, scratchString.c_str());
			return;
		}
	}

	fileBeingWritten->Write(gb.buffer);
	fileBeingWritten->Write('\n');
	Init();
}

void StringParser::WriteBinaryToFile(char b) noexcept
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

void StringParser::FinishWritingBinary() noexcept
{
	// If we get here then we have come to the end of the data
	fileBeingWritten->Close();
	const bool crcOk = (crc32 == fileBeingWritten->GetCRC32() || crc32 == 0);
	fileBeingWritten = nullptr;
	binaryWriting = false;
	if (crcOk)
	{
		const char* const r = (gb.MachineState().compatibility == Compatibility::marlin) ? "Done saving file." : "";
		reprap.GetGCodes().HandleReply(gb, GCodeResult::ok, r);
	}
	else
	{
		reprap.GetGCodes().HandleReply(gb, GCodeResult::error, "CRC32 checksum doesn't match");
	}
}

// Called when we start a new file
void StringParser::StartNewFile() noexcept
{
	seenLeadingSpace = seenLeadingTab = seenMetaCommand = warnedAboutMixedSpacesAndTabs = false;
}

// This is called when we reach the end of the file we are reading from. Return true if there is a line waiting to be processed.
bool StringParser::FileEnded() noexcept
{
	if (IsWritingBinary())
	{
		// We are in the middle of writing a binary file but the input stream has ended
		FinishWritingBinary();
		Init();
		return false;
	}

	bool commandCompleted = false;
	if (gcodeLineEnd != 0)				// if there is something in the buffer
	{
		Put('\n');						// append a newline in case the file didn't end with one
		commandCompleted = true;
	}

	if (IsWritingFile())
	{
		if (commandCompleted)
		{
			DecodeCommand();
			if (gb.IsReady())				// if we have a complete command
			{
				const bool gotM29 = (GetCommandLetter() == 'M' && GetCommandNumber() == 29);
				if (!gotM29)				// if it wasn't M29, write it to file
				{
					fileBeingWritten->Write(gb.buffer);
					fileBeingWritten->Write('\n');
				}
			}
		}

		// Close the file whether or not we saw M29
		fileBeingWritten->Close();
		fileBeingWritten = nullptr;
		SetFinished();
		const char* const r = (gb.MachineState().compatibility == Compatibility::marlin) ? "Done saving file." : "";
		reprap.GetGCodes().HandleReply(gb, GCodeResult::ok, r);
		return false;
	}

	return commandCompleted;
}

#endif

// Functions to read values from lines of GCode, allowing for expressions and variable substitution
float StringParser::ReadFloatValue()
{
	if (gb.buffer[readPointer] == '{')
	{
		++readPointer;
		char stringBuffer[StringBufferLength];
		StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
		const ExpressionValue val = ParseBracketedExpression(bufRef, '}', true);
		switch (val.type)
		{
		case TYPE_OF(float):
			return val.fVal;

		case TYPE_OF(int32_t):
			return (float)val.iVal;

		case TYPE_OF(uint32_t):
			return (float)val.uVal;

		default:
			throw ConstructParseException("expected float value");
		}
	}

	const char *endptr;
	const float rslt = SafeStrtof(gb.buffer + readPointer, &endptr);
	readPointer = endptr - gb.buffer;
	return rslt;
}

uint32_t StringParser::ReadUIValue()
{
	if (gb.buffer[readPointer] == '{')
	{
		++readPointer;
		char stringBuffer[StringBufferLength];
		StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
		const ExpressionValue val = ParseBracketedExpression(bufRef, '}', true);
		switch (val.type)
		{
		case TYPE_OF(uint32_t):
			return val.uVal;

		case TYPE_OF(int32_t):
			if (val.iVal >= 0)
			{
				return (uint32_t)val.iVal;
			}
			throw ConstructParseException("value must be non-negative");

		default:
			throw ConstructParseException("expected non-negative integer value");
		}
	}

	int base = 10;
	size_t skipTrailingQuote = 0;

	// Allow "0xNNNN" or "xNNNN" where NNNN are hex digits
	if (gb.buffer[readPointer] == '"')
	{
		++readPointer;
		skipTrailingQuote = 1;
		switch (gb.buffer[readPointer])
		{
		case 'x':
		case 'X':
			base = 16;
			++readPointer;
			break;

		case '0':
			if (gb.buffer[readPointer + 1] == 'x' || gb.buffer[readPointer + 1] == 'X')
			{
				base = 16;
				readPointer += 2;
			}
			break;

		default:
			break;
		}
	}

	const char *endptr;
	const uint32_t rslt = SafeStrtoul(gb.buffer + readPointer, &endptr, base);
	readPointer = endptr - gb.buffer + skipTrailingQuote;
	return rslt;
}

int32_t StringParser::ReadIValue()
{
	if (gb.buffer[readPointer] == '{')
	{
		++readPointer;
		char stringBuffer[StringBufferLength];
		StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
		const ExpressionValue val = ParseBracketedExpression(bufRef, '}', true);
		switch (val.type)
		{
		case TYPE_OF(int32_t):
			return val.iVal;

		case TYPE_OF(uint32_t):
			return (int32_t)val.uVal;

		default:
			throw ConstructParseException("expected integer value");
		}
	}

	const char *endptr;
	const int32_t rslt = SafeStrtol(gb.buffer + readPointer, &endptr);
	readPointer = endptr - gb.buffer;
	return rslt;
}

DriverId StringParser::ReadDriverIdValue()
{
	DriverId result;
	const uint32_t v1 = ReadUIValue();
#if SUPPORT_CAN_EXPANSION
	if (gb.buffer[readPointer] == '.')
	{
		++readPointer;
		const uint32_t v2 = ReadUIValue();
		result.localDriver = v2;
		result.boardAddress = v1;
	}
	else
	{
		result.localDriver = v1;
		result.boardAddress = 0;
	}
#else
	result.localDriver = v1;
#endif
	return result;
}

// Get a string expression and append it to the string
void StringParser::AppendAsString(ExpressionValue val, const StringRef& str)
{
	switch (val.type)
	{
	case TYPE_OF(char):
		str.cat(val.cVal);
		break;

	case TYPE_OF(const char*):
		str.cat(val.sVal);
		break;

	case TYPE_OF(float):
		str.catf(val.GetFloatFormatString(), (double)val.fVal);
		break;

	case TYPE_OF(uint32_t):
		str.catf("%" PRIu32, val.uVal);			// convert unsigned integer to string
		break;

	case TYPE_OF(int32_t):
		str.catf("%" PRIi32, val.uVal);			// convert signed integer to string
		break;

	case TYPE_OF(bool):
		str.cat((val.bVal) ? "true" : "false");	// convert bool to string
		break;

	case TYPE_OF(IPAddress):
		str.cat(IP4String(val.uVal).c_str());
		break;

	case TYPE_OF(DateTime):
		{
			const time_t time = val.Get56BitValue();
			tm timeInfo;
			gmtime_r(&time, &timeInfo);
			str.catf("%04u-%02u-%02u %02u:%02u:%02u",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		}
		break;

	case TYPE_OF(DriverId):
#if SUPPORT_CAN_EXPANSION
		str.catf("%u.%u", (unsigned int)(val.uVal >> 8), (unsigned int)(val.uVal & 0xFF));
#else
		str.catf("%u", (unsigned int)val.uVal);
#endif
		break;

	case TYPE_OF(MacAddress):
		str.catf("%02x:%02x:%02x:%02x:%02x:%02x",
					(unsigned int)(val.uVal & 0xFF), (unsigned int)((val.uVal >> 8) & 0xFF), (unsigned int)((val.uVal >> 16) & 0xFF), (unsigned int)((val.uVal >> 24) & 0xFF),
					(unsigned int)(val.param & 0xFF), (unsigned int)((val.param >> 8) & 0xFF));
		break;

#if SUPPORT_CAN_EXPANSION
	case TYPE_OF(CanExpansionBoardDetails):
		val.ExtractRequestedPart(str);
		break;
#endif

	default:
		throw ConstructParseException("string value expected");
	}
}

// Evaluate a bracketed expression
ExpressionValue StringParser::ParseBracketedExpression(StringBuffer& stringBuffer, char closingBracket, bool evaluate)
{
	auto rslt = ParseExpression(stringBuffer, 0, evaluate);
	if (gb.buffer[readPointer] != closingBracket)
	{
		throw ConstructParseException("expected '%c'", (uint32_t)closingBracket);
	}
	++readPointer;
	return rslt;
}

// Evaluate an expression, stopping before any binary operators with priority 'priority' or lower
ExpressionValue StringParser::ParseExpression(StringBuffer& stringBuffer, uint8_t priority, bool evaluate)
{
	// Lists of binary operators and their priorities
	static constexpr const char *operators = "?^&|!=<>+-*/";				// for multi-character operators <= and >= and != this is the first character
	static constexpr uint8_t priorities[] = { 1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6 };
	constexpr uint8_t UnaryPriority = 10;									// must be higher than any binary operator priority
	static_assert(ARRAY_SIZE(priorities) == strlen(operators));

	// Start by looking for a unary operator or opening bracket
	SkipWhiteSpace();
	const char c = gb.buffer[readPointer];
	ExpressionValue val;
	switch (c)
	{
	case '"':
		InternalGetQuotedString(stringBuffer.GetRef());
		val.Set(GetAndFix(stringBuffer));
		break;

	case '-':
		++readPointer;
		val = ParseExpression(stringBuffer, UnaryPriority, evaluate);
		switch (val.type)
		{
		case TYPE_OF(int32_t):
			val.iVal = -val.iVal;		//TODO overflow check
			break;

		case TYPE_OF(float):
			val.fVal = -val.fVal;
			break;

		default:
			throw ConstructParseException("expected numeric value after '-'");
		}
		break;

	case '+':
		++readPointer;
		val = ParseExpression(stringBuffer, UnaryPriority, evaluate);
		switch (val.type)
		{
		case TYPE_OF(uint32_t):
			// Convert enumeration to integer
			val.iVal = (int32_t)val.uVal;
			val.type = TYPE_OF(int32_t);
			break;

		case TYPE_OF(int32_t):
		case TYPE_OF(float):
			break;

		default:
			throw ConstructParseException("expected numeric or enumeration value after '+'");
		}
		break;

	case '#':
		++readPointer;
		SkipWhiteSpace();
		if (isalpha(gb.buffer[readPointer]))
		{
			// Probably applying # to an object model array, so optimise by asking the OM for just the length
			val = ParseIdentifierExpression(stringBuffer, true, evaluate);
		}
		else
		{
			val = ParseExpression(stringBuffer, UnaryPriority, evaluate);
			if (val.type == TYPE_OF(const char*))
			{
				const char* s = val.sVal;
				val.Set((int32_t)strlen(s));
				stringBuffer.FinishedUsing(s);
				val.type = TYPE_OF(int32_t);
			}
			else
			{
				throw ConstructParseException("expected object model value or string after '#");
			}
		}
		break;

	case '{':
		++readPointer;
		val = ParseBracketedExpression(stringBuffer, '}', evaluate);
		break;

	case '(':
		++readPointer;
		val = ParseBracketedExpression(stringBuffer, ')', evaluate);
		break;

	case '!':
		++readPointer;
		val = ParseExpression(stringBuffer, UnaryPriority, evaluate);
		ConvertToBool(val, evaluate);
		val.bVal = !val.bVal;
		break;

	default:
		if (isdigit(c))						// looks like a number
		{
			val = ParseNumber();
		}
		else if (isalpha(c))				// looks like a variable name
		{
			val = ParseIdentifierExpression(stringBuffer, false, evaluate);
		}
		else
		{
			throw ConstructParseException("expected an expression");
		}
		break;
	}

	// See if it is followed by a binary operator
	do
	{
		SkipWhiteSpace();
		char opChar = gb.buffer[readPointer];
		if (opChar == 0)	// don't pass null to strchr
		{
			return val;
		}

		const char * const p = strchr(operators, opChar);
		if (p == nullptr)
		{
			return val;
		}
		const size_t index = p - operators;
		const uint8_t opPrio = priorities[index];
		if (opPrio <= priority)
		{
			return val;
		}

		++readPointer;						// skip the [first] operator character

		// Handle >= and <= and !=
		bool invert = false;
		if (opChar == '!')
		{
			if (gb.buffer[readPointer] != '=')
			{
				throw ConstructParseException("expected '='");
			}
			invert = true;
			++readPointer;
			opChar = '=';
		}
		else if ((opChar == '>' || opChar == '<') && gb.buffer[readPointer] == '=')
		{
			invert = true;
			++readPointer;
			opChar ^= ('>' ^ '<');			// change < to > or vice versa
		}

		// Allow == && || as alternatives to = & |
		if ((opChar == '=' || opChar == '&' || opChar == '|') && gb.buffer[readPointer] == opChar)
		{
			++readPointer;
		}

		// Handle operators that do not always evaluate their second operand
		switch (opChar)
		{
		case '&':
			ConvertToBool(val, evaluate);
			{
				ExpressionValue val2 = ParseExpression(stringBuffer, opPrio, evaluate && val.bVal);		// get the next operand
				if (val.bVal)
				{
					ConvertToBool(val2, evaluate);
					val.bVal = val.bVal && val2.bVal;
				}
			}
			break;

		case '|':
			ConvertToBool(val, evaluate);
			{
				ExpressionValue val2 = ParseExpression(stringBuffer, opPrio, evaluate && !val.bVal);		// get the next operand
				if (!val.bVal)
				{
					ConvertToBool(val2, evaluate);
					val.bVal = val.bVal || val2.bVal;
				}
			}
			break;

		case '?':
			ConvertToBool(val, evaluate);
			{
				ExpressionValue val2 = ParseExpression(stringBuffer, opPrio, evaluate && val.bVal);		// get the second operand
				if (gb.buffer[readPointer] != ':')
				{
					throw ConstructParseException("expected ':'");
				}
				++readPointer;
				ExpressionValue val3 = ParseExpression(stringBuffer, opPrio - 1, evaluate && !val.bVal);	// get the third operand, which may be a further conditional expression
				return (val.bVal) ? val2 : val3;
			}

		default:
			// Handle binary operators that always evaluate both operands
			{
				ExpressionValue val2 = ParseExpression(stringBuffer, opPrio, evaluate);	// get the next operand
				switch(opChar)
				{
				case '+':
					BalanceNumericTypes(val, val2, evaluate);
					if (val.type == TYPE_OF(float))
					{
						val.fVal += val2.fVal;
						val.param = max(val.param, val2.param);
					}
					else
					{
						val.iVal += val2.iVal;
					}
					break;

				case '-':
					BalanceNumericTypes(val, val2, evaluate);
					if (val.type == TYPE_OF(float))
					{
						val.fVal -= val2.fVal;
						val.param = max(val.param, val2.param);
					}
					else
					{
						val.iVal -= val2.iVal;
					}
					break;

				case '*':
					BalanceNumericTypes(val, val2, evaluate);
					if (val.type == TYPE_OF(float))
					{
						val.fVal *= val2.fVal;
						val.param = max(val.param, val2.param);
					}
					else
					{
						val.iVal *= val2.iVal;
					}
					break;

				case '/':
					ConvertToFloat(val, evaluate);
					ConvertToFloat(val2, evaluate);
					val.fVal /= val2.fVal;
					val.param = 0;
					break;

				case '>':
					BalanceTypes(val, val2, evaluate);
					switch (val.type)
					{
					case TYPE_OF(int32_t):
						val.bVal = (val.iVal > val2.iVal);
						break;

					case TYPE_OF(float_t):
						val.bVal = (val.fVal > val2.fVal);
						break;

					case TYPE_OF(bool):
						val.bVal = (val.bVal && !val2.bVal);
						break;

					default:
						throw ConstructParseException("expected numeric or Boolean operands to comparison operator");
					}
					val.type = TYPE_OF(bool);
					if (invert)
					{
						val.bVal = !val.bVal;
					}
					break;

				case '<':
					BalanceTypes(val, val2, evaluate);
					switch (val.type)
					{
					case TYPE_OF(int32_t):
						val.bVal = (val.iVal < val2.iVal);
						break;

					case TYPE_OF(float_t):
						val.bVal = (val.fVal < val2.fVal);
						break;

					case TYPE_OF(bool):
						val.bVal = (!val.bVal && val2.bVal);
						break;

					default:
						throw ConstructParseException("expected numeric or Boolean operands to comparison operator");
					}
					val.type = TYPE_OF(bool);
					if (invert)
					{
						val.bVal = !val.bVal;
					}
					break;

				case '=':
					BalanceTypes(val, val2, evaluate);
					switch (val.type)
					{
					case TYPE_OF(const ObjectModel*):
						{
							const bool v1Null = (val.omVal == nullptr),
										v2Null = (val2.omVal == nullptr);
							val.bVal = (v1Null) ? v2Null
										: (v2Null) ? false
											: throw ConstructParseException("cannot compare objects");
						}
						break;

					case TYPE_OF(int32_t):
						val.bVal = (val.iVal == val2.iVal);
						break;

					case TYPE_OF(uint32_t):
						val.bVal = (val.uVal == val2.uVal);
						break;

					case TYPE_OF(float_t):
						val.bVal = (val.fVal == val2.fVal);
						break;

					case TYPE_OF(bool):
						val.bVal = (val.bVal == val2.bVal);
						break;

					case TYPE_OF(const char*):
						val.bVal = (strcmp(val.sVal, val2.sVal) == 0);
						break;

					default:
						throw ConstructParseException("unexpected operand type to equality operator");
					}
					val.type = TYPE_OF(bool);
					if (invert)
					{
						val.bVal = !val.bVal;
					}
					break;

				case '^':
					ConvertToString(val, evaluate, stringBuffer);
					ConvertToString(val2, evaluate, stringBuffer);
					// We could skip evaluation if evaluate is false, but there is no real need to
					if (stringBuffer.Concat(val.sVal, val2.sVal))
					{
						throw ConstructParseException("too many strings");
					}
					val.sVal = GetAndFix(stringBuffer);
					break;
				}
			}
		}
	} while (true);
}

void StringParser::BalanceNumericTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate)
{
	if (val1.type == TYPE_OF(float))
	{
		ConvertToFloat(val2, evaluate);
	}
	else if (val2.type == TYPE_OF(float))
	{
		ConvertToFloat(val1, evaluate);
	}
	else if (val1.type != TYPE_OF(int32_t) || val2.type != TYPE_OF(int32_t))
	{
		if (evaluate)
		{
			throw ConstructParseException("expected numeric operands");
		}
		val1.Set((int32_t)0);
		val2.Set((int32_t)0);
	}
}

void StringParser::BalanceTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate)
{
	if (val1.type == TYPE_OF(float))
	{
		ConvertToFloat(val2, evaluate);
	}
	else if (val2.type == TYPE_OF(float))
	{
		ConvertToFloat(val1, evaluate);
	}
	else if (val1.type != val2.type)
	{
		if (evaluate)
		{
			throw ConstructParseException("cannot convert operands to same type");
		}
		val1.Set((int32_t)0);
		val2.Set((int32_t)0);
	}
}

void StringParser::EnsureNumeric(ExpressionValue& val, bool evaluate)
{
	switch (val.type)
	{
	case TYPE_OF(uint32_t):
		val.type = TYPE_OF(int32_t);
		val.iVal = val.uVal;
		break;

	case TYPE_OF(int32_t):
	case TYPE_OF(float):
		break;

	default:
		if (evaluate)
		{
			throw ConstructParseException("expected numeric operand");
		}
		val.Set((int32_t)0);
	}
}

void StringParser::ConvertToFloat(ExpressionValue& val, bool evaluate)
{
	switch (val.type)
	{
	case TYPE_OF(int32_t):
		val.fVal = (float)val.iVal;
		val.type = TYPE_OF(float);
		val.param = 1;
		break;

	case TYPE_OF(float):
		break;

	default:
		if (evaluate)
		{
			throw ConstructParseException("expected numeric operand");
		}
		val.Set(0.0f);
	}
}

void StringParser::ConvertToBool(ExpressionValue& val, bool evaluate)
{
	if (val.type != TYPE_OF(bool))
	{
		if (evaluate)
		{
			throw ConstructParseException("expected Boolean operand");
		}
		val.Set(false);
	}
}

void StringParser::ConvertToString(ExpressionValue& val, bool evaluate, StringBuffer& stringBuffer)
{
	if (val.type != TYPE_OF(const char*))
	{
		if (evaluate)
		{
			stringBuffer.ClearLatest();
			AppendAsString(val, stringBuffer.GetRef());
			val.Set(GetAndFix(stringBuffer));
		}
		else
		{
			val.Set("");
		}
	}
}

// Get a C-style pointer to the latest string in the buffer, and start a new one
const char *StringParser::GetAndFix(StringBuffer& stringBuffer)
{
	const char *const rslt = stringBuffer.LatestCStr();
	if (stringBuffer.Fix())
	{
		throw ConstructParseException("too many strings");
	}
	return rslt;
}

void StringParser::SkipWhiteSpace() noexcept
{
	while (gb.buffer[readPointer] == ' ' || gb.buffer[readPointer] == '\t')
	{
		++readPointer;
	}
}

// Parse a number. the initial character of the string is a decimal digit.
ExpressionValue StringParser::ParseNumber()
{
	// 2. Read digits before decimal point, E or e
	unsigned long valueBeforePoint = 0;
	char c;
	while (isdigit((c = gb.buffer[readPointer])))
	{
		const unsigned int digit = c - '0';
		if (valueBeforePoint > ULONG_MAX/10 || (valueBeforePoint *= 10, valueBeforePoint > ULONG_MAX - digit))
		{
			throw ConstructParseException("too many digits");
		}
		valueBeforePoint += digit;
		++readPointer;
	}

	// 3. Check for decimal point before E or e
	unsigned long valueAfterPoint = 0;
	long digitsAfterPoint = 0;
	bool isFloat = (c == '.');
	if (isFloat)
	{
		++readPointer;

		// 3b. Read the digits (if any) after the decimal point
		while (isdigit((c = gb.buffer[readPointer])))
		{
			const unsigned int digit = c - '0';
			if (valueAfterPoint > LONG_MAX/10 || (valueAfterPoint *= 10, valueAfterPoint > LONG_MAX - digit))
			{
				throw ConstructParseException("too many decimal digits");
			}
			valueAfterPoint += digit;
			++digitsAfterPoint;
			++readPointer;
		}
	}

	// 5. Check for exponent part
	long exponent = 0;
	if (toupper(c) == 'E')
	{
		isFloat = true;
		++readPointer;
		c = gb.buffer[readPointer];

		// 5a. Check for signed exponent
		const bool expNegative = (c == '-');
		if (expNegative || c == '+')
		{
			++readPointer;
		}

		// 5b. Read exponent digits
		while (isdigit((c = gb.buffer[readPointer])))
		{
			exponent = (10 * exponent) + (c - '0');	// could overflow, but anyone using such large numbers is being very silly
			++readPointer;
		}

		if (expNegative)
		{
			exponent = -exponent;
		}
	}

	// 6. Compute the composite value
	ExpressionValue retvalue;

	if (isFloat)
	{
		retvalue.type = TYPE_OF(float);
		retvalue.param = constrain<long>(digitsAfterPoint, 1, MaxFloatDigitsDisplayedAfterPoint);
		if (valueAfterPoint != 0)
		{
			if (valueBeforePoint == 0)
			{
				retvalue.fVal = (float)((double)valueAfterPoint * pow(10, exponent - digitsAfterPoint));
			}
			else
			{
				retvalue.fVal = (float)(((double)valueAfterPoint/pow(10, digitsAfterPoint) + valueBeforePoint) * pow(10, exponent));
			}
		}
		else
		{
			retvalue.fVal = (float)(valueBeforePoint * pow(10, exponent));
		}
	}
	else
	{
		retvalue.type = TYPE_OF(int32_t);
		retvalue.iVal = (int32_t)valueBeforePoint;
	}

	return retvalue;
}

// Parse an identifier expression
ExpressionValue StringParser::ParseIdentifierExpression(StringBuffer& stringBuffer, bool applyLengthOperator, bool evaluate)
{
	if (!isalpha(gb.buffer[readPointer]))
	{
		throw ConstructParseException("expected an identifier");
	}

	String<MaxVariableNameLength> id;
	ObjectExplorationContext context("v", applyLengthOperator, 99, gb.machineState->lineNumber, readPointer);

	// Loop parsing identifiers and index expressions
	// When we come across an index expression, evaluate it, add it to the context, and place a marker in the identifier string.
	char c;
	while (isalpha((c = gb.buffer[readPointer])) || isdigit(c) || c == '_' || c == '.' || c == '[')
	{
		++readPointer;
		if (c == '[')
		{
			char stringBuffer[StringLength20];			// we shouldn't need any strings while evaluate an index, so use a small buffer
			StringBuffer bufRef(stringBuffer, ARRAY_SIZE(stringBuffer));
			const ExpressionValue index = ParseExpression(bufRef, 0, evaluate);
			if (gb.buffer[readPointer] != ']')
			{
				throw ConstructParseException("expected ']'");
			}
			if (index.type != TYPE_OF(int32_t))
			{
				throw ConstructParseException("expected integer expression");
			}
			++readPointer;								// skip the ']'
			context.ProvideIndex(index.iVal);
			c = '^';									// add the marker
		}
		if (id.cat(c))
		{
			throw ConstructParseException("variable name too long");;
		}
	}

	// Check for the names of constants
	if (id.Equals("true"))
	{
		return ExpressionValue(true);
	}

	if (id.Equals("false"))
	{
		return ExpressionValue(false);
	}

	if (id.Equals("null"))
	{
		return ExpressionValue((const ObjectModel *)nullptr);
	}

	if (id.Equals("pi"))
	{
		return ExpressionValue(Pi);
	}

	if (id.Equals("iterations"))
	{
		const int32_t v = gb.MachineState().GetIterations();
		if (v < 0)
		{
			throw ConstructParseException("'iterations' used when not inside a loop");
		}
		return ExpressionValue(v);
	}

	if (id.Equals("result"))
	{
		int32_t rslt;
		switch (gb.GetLastResult())
		{
		case GCodeResult::ok:
			rslt = 0;
			break;

		case GCodeResult::warning:
		case GCodeResult::warningNotSupported:
			rslt = 1;
			break;

		default:
			rslt = 2;
			break;
		}
		return ExpressionValue(rslt);
	}

	if (id.Equals("line"))
	{
		return ExpressionValue((int32_t)gb.MachineState().lineNumber);
	}

	// Check whether it is a function call
	SkipWhiteSpace();
	if (gb.buffer[readPointer] == '(')
	{
		// It's a function call
		++readPointer;
		ExpressionValue rslt = ParseExpression(stringBuffer, 0, evaluate);
		//TODO use a binary search to do function lookup
		if (id.Equals("abs"))
		{
			switch (rslt.type)
			{
			case TYPE_OF(int32_t):
				rslt.iVal = labs(rslt.iVal);
				break;

			case TYPE_OF(float):
				rslt.fVal = fabsf(rslt.fVal);
				break;

			default:
				if (evaluate)
				{
					throw ConstructParseException("expected numeric operand");
				}
				rslt.Set((int32_t)0);
			}
		}
		else if (id.Equals("sin"))
		{
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = sinf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
		}
		else if (id.Equals("cos"))
		{
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = cosf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
		}
		else if (id.Equals("tan"))
		{
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = tanf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
		}
		else if (id.Equals("asin"))
		{
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = asinf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
		}
		else if (id.Equals("acos"))
		{
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = acosf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
		}
		else if (id.Equals("atan"))
		{
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = atanf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
		}
		else if (id.Equals("atan2"))
		{
			ConvertToFloat(rslt, evaluate);
			SkipWhiteSpace();
			if (gb.buffer[readPointer] != ',')
			{
				throw ConstructParseException("expected ','");
			}
			++readPointer;
			SkipWhiteSpace();
			ExpressionValue nextOperand = ParseExpression(stringBuffer, 0, evaluate);
			ConvertToFloat(nextOperand, evaluate);
			rslt.fVal = atan2f(rslt.fVal, nextOperand.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
		}
		else if (id.Equals("sqrt"))
		{
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = sqrtf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
		}
		else if (id.Equals("isnan"))
		{
			ConvertToFloat(rslt, evaluate);
			rslt.type = TYPE_OF(bool);
			rslt.bVal = (isnan(rslt.fVal) != 0);
		}
		else if (id.Equals("floor"))
		{
			ConvertToFloat(rslt, evaluate);
			const float f = floorf(rslt.fVal);
			if (f <= (float)std::numeric_limits<int32_t>::max() && f >= (float)std::numeric_limits<int32_t>::min())
			{
				rslt.type = TYPE_OF(int32_t);
				rslt.iVal = (int32_t)f;
			}
			else
			{
				rslt.fVal = f;
			}
		}
		else if (id.Equals("mod"))
		{
			SkipWhiteSpace();
			if (gb.buffer[readPointer] != ',')
			{
				throw ConstructParseException("expected ','");
			}
			++readPointer;
			SkipWhiteSpace();
			ExpressionValue nextOperand = ParseExpression(stringBuffer, 0, evaluate);
			BalanceNumericTypes(rslt, nextOperand, evaluate);
			if (rslt.type == TYPE_OF(float))
			{
				rslt.fVal = fmod(rslt.fVal, nextOperand.fVal);
			}
			else if (nextOperand.iVal == 0)
			{
				rslt.iVal = 0;
			}
			else
			{
				rslt.iVal %= nextOperand.iVal;
			}
		}
		else if (id.Equals("max"))
		{
			for (;;)
			{
				SkipWhiteSpace();
				if (gb.buffer[readPointer] != ',')
				{
					break;
				}
				++readPointer;
				SkipWhiteSpace();
				ExpressionValue nextOperand = ParseExpression(stringBuffer, 0, evaluate);
				BalanceNumericTypes(rslt, nextOperand, evaluate);
				if (rslt.type == TYPE_OF(float))
				{
					rslt.fVal = max<float>(rslt.fVal, nextOperand.fVal);
					rslt.param = max(rslt.param, nextOperand.param);
				}
				else
				{
					rslt.iVal = max<int32_t>(rslt.iVal, nextOperand.iVal);
				}
			}
		}
		else if (id.Equals("min"))
		{
			for (;;)
			{
				SkipWhiteSpace();
				if (gb.buffer[readPointer] != ',')
				{
					break;
				}
				++readPointer;
				SkipWhiteSpace();
				ExpressionValue nextOperand = ParseExpression(stringBuffer, 0, evaluate);
				BalanceNumericTypes(rslt, nextOperand, evaluate);
				if (rslt.type == TYPE_OF(float))
				{
					rslt.fVal = min<float>(rslt.fVal, nextOperand.fVal);
					rslt.param = max(rslt.param, nextOperand.param);
				}
				else
				{
					rslt.iVal = min<int32_t>(rslt.iVal, nextOperand.iVal);
				}
			}
		}
		else
		{
			throw ConstructParseException("unknown function");
		}
		SkipWhiteSpace();
		if (gb.buffer[readPointer] != ')')
		{
			throw ConstructParseException("expected ')'");
		}
		++readPointer;
		return rslt;
	}

	return reprap.GetObjectValue(context, id.c_str());
}

GCodeException StringParser::ConstructParseException(const char *str) const
{
	return GCodeException(gb.machineState->lineNumber, readPointer + commandIndent, str);
}

GCodeException StringParser::ConstructParseException(const char *str, const char *param) const
{
	return GCodeException(gb.machineState->lineNumber, readPointer + commandIndent, str, param);
}

GCodeException StringParser::ConstructParseException(const char *str, uint32_t param) const
{
	return GCodeException(gb.machineState->lineNumber, readPointer + commandIndent, str, param);
}

// End
