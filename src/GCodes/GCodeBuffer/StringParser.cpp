/*
 * StringParser.cpp
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

//*************************************************************************************

#include "StringParser.h"
#include "GCodeBuffer.h"
#include "ExpressionParser.h"

#include <GCodes/GCodes.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Networking/NetworkDefs.h>

// Replace the default definition of THROW_INTERNAL_ERROR by one that gives line information
#undef THROW_INTERNAL_ERROR
#define THROW_INTERNAL_ERROR	ThrowGCodeException("internal error at file " __FILE__ "(%d)", __LINE__)

#if HAS_MASS_STORAGE
static constexpr char eofString[] = EOF_STRING;		// What's at the end of an HTML file?
#endif

StringParser::StringParser(GCodeBuffer& gcodeBuffer) noexcept
	: gb(gcodeBuffer), fileBeingWritten(nullptr), writingFileSize(0), indentToSkipTo(NoIndentSkip), eofStringCounter(0),
	  hasCommandNumber(false), commandLetter('Q'), checksumRequired(false), crcRequired(false), binaryWriting(false)
{
	StartNewFile();
	Init();
}

void StringParser::Init() noexcept
{
	gcodeLineEnd = 0;
	commandStart = commandLength = 0;								// set both to zero so that calls to GetFilePosition don't return negative values
	readPointer = -1;
	hadLineNumber = hadChecksum = overflowed = seenExpression = false;
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
	// As computing the CRC takes several cycles, we only do it if we had a line number
	if (hadLineNumber)
	{
		computedChecksum ^= (uint8_t)c;
		crc16.Update(c);
	}
}

inline void StringParser::StoreAndAddToChecksum(char c) noexcept
{
	AddToChecksum(c);
	if (gcodeLineEnd + 1 < ARRAY_SIZE(gb.buffer))					// if there is space for this character and a trailing null
	{
		gb.buffer[gcodeLineEnd++] = c;
	}
	else if (gb.bufferState != GCodeBufferState::parsingComment)	// we don't care if comment lines overflow
	{
		overflowed = true;
	}
}

// Add a byte to the code being assembled.  If false is returned, the code is not yet complete.
// If true, it is complete and ready to be acted upon and 'indent' is the number of leading white space characters.
bool StringParser::Put(char c) noexcept
{
	if (c != 0)
	{
		++commandLength;
	}

	// We now discard CR if we are reading from file. It makes line number counting easier and it's unlikely that a pre-OSX Mac will be used with a Duet.
	// When not reading from file we still accept CR as a line terminator, for compatibility with Putty and some other terminal emulators.
	if (c == '\r' && gb.IsDoingFile())
	{
		return false;
	}

	if (c == 0 || c == '\n' || c == '\r')
	{
		return LineFinished();
	}

	if (c == (char)0x7Fu && gb.bufferState != GCodeBufferState::discarding)
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
				crc16.Reset(0);
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
				receivedLineNumber = (10 * receivedLineNumber) + (unsigned int)(c - '0');
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
				if (hadLineNumber && braceCount == 0)
				{
					declaredChecksum = 0;
					checksumCharsReceived = 0;
					hadChecksum = true;
					gb.bufferState = GCodeBufferState::parsingChecksum;
				}
				else
				{
					StoreAndAddToChecksum(c);
				}
				break;

			case ';':
				if (commandIndent == 0 && gcodeLineEnd == 0)
				{
					StoreAndAddToChecksum(c);
					gb.bufferState = GCodeBufferState::parsingComment;
				}
				else
				{
					gb.bufferState = GCodeBufferState::discarding;
				}
				break;

			case '(':
				if (braceCount == 0 && reprap.GetGCodes().GetMachineType() == MachineType::cnc)
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
				seenExpression = true;
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
				break;
			}
			break;

		case GCodeBufferState::parsingComment:
			// We are parsing a whole-line comment, which may possibly be of interest
			// For now we assume that a '*' character is not the start of a checksum
			StoreAndAddToChecksum(c);
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
				declaredChecksum = (10 * declaredChecksum) + (unsigned int)(c - '0');
				++checksumCharsReceived;
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
bool StringParser::LineFinished() noexcept
{
	if (hadLineNumber)
	{
		gb.CurrentFileMachineState().lineNumber = receivedLineNumber;
	}
	else
	{
		++gb.CurrentFileMachineState().lineNumber;
	}
#if SUPPORT_ASYNC_MOVES
	gb.CurrentFileMachineState().fpos = GetFilePosition();
#endif

	if (gcodeLineEnd == 0)
	{
		// Empty line
		Init();
		return false;
	}

	gb.buffer[gcodeLineEnd] = 0;

	if (gb.bufferState != GCodeBufferState::parsingComment)			// we don't checksum or echo comment lines, but we still need to process them
	{
		bool badChecksum, missingChecksum;
		if (hadChecksum)
		{
			missingChecksum = false;
			switch (checksumCharsReceived)
			{
			case 1:
			case 2:
			case 3:
				// If a CRC is required then the only command we allow without a CRC is M409
				badChecksum = (crcRequired || computedChecksum != declaredChecksum);
				break;

			case 5:
				badChecksum = (crc16.Get() != declaredChecksum);
				break;

			default:
				badChecksum = true;
				break;
			}
		}
		else
		{
			badChecksum = false;
			missingChecksum = ((checksumRequired || crcRequired) && gb.LatestMachineState().GetPrevious() == nullptr);
		}

		if (reprap.GetDebugFlags(Module::Gcodes).IsBitSet(gb.GetChannel().ToBaseType()) && fileBeingWritten == nullptr)
		{
			debugPrintf("%s%s: %s\n", gb.GetChannel().ToString(), ((badChecksum) ? "(bad-csum)" : (missingChecksum) ? "(no-csum)" : ""), gb.buffer);
		}

		if (badChecksum || missingChecksum)
		{
			Init();
			return false;
		}
	}

	commandStart = 0;
	return true;
}

// Check whether the current command is a meta command, or we are skipping commands in a block
// Return true if the current line no longer needs to be processed
bool StringParser::CheckMetaCommand(const StringRef& reply) THROWS(GCodeException)
{
	if (overflowed)
	{
		throw GCodeException(&gb, ARRAY_SIZE(gb.buffer) + commandIndent - 1, "GCode command too long");
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
					skippedBlockType = gb.GetBlockState().GetType();		// save the type of the block we skipped in case the command is 'else' or 'elif'
					gb.GetBlockState().SetPlainBlock();						// we've ended the loop or if-block
				}
				indentToSkipTo = NoIndentSkip;								// no longer skipping
			}
		}

		while (commandIndent < gb.GetBlockIndent())
		{
			gb.CurrentFileMachineState().EndBlock();
			if (gb.GetBlockState().GetType() == BlockType::loop)
			{
				// Go back to the start of the loop and re-evaluate the while-part
				gb.CurrentFileMachineState().lineNumber = gb.GetBlockState().GetLineNumber();
#if SUPPORT_ASYNC_MOVES
				gb.CurrentFileMachineState().fpos = gb.GetBlockState().GetFilePosition();
#endif
				gb.RestartFrom(gb.GetBlockState().GetFilePosition());
				Init();
				return true;
			}
		}

		if (commandIndent > gb.GetBlockIndent())
		{
			// Indentation has increased so start new block(s)
			gb.CurrentFileMachineState().CreateBlock(commandIndent);
		}
	}

	// Save and clear the command letter in case ProcessConditionalGCode throws an exception, so that the error message won't include the previous command
	const char savedCommandLetter = commandLetter;
	commandLetter = 'E';						// we use this to flag a meta command, see GCodeException::GetMessage
	const bool b = ProcessConditionalGCode(reply, skippedBlockType, doingFile);	// this may throw a GCodeException
	if (b)
	{
		seenMetaCommand = true;
		commandEnd = gcodeLineEnd;				// there are no more commands on this line
		if (doingFile)
		{
			CheckForMixedSpacesAndTabs();
		}
		Init();
	}
	else
	{
		commandLetter = savedCommandLetter;		// restore this so that we can handle Fanuc-style GCode
	}

	return b;
}

void StringParser::CheckForMixedSpacesAndTabs() noexcept
{
	if (seenMetaCommand && !warnedAboutMixedSpacesAndTabs && seenLeadingSpace && seenLeadingTab)
	{
		reprap.GetPlatform().MessageF(AddWarning(gb.GetResponseMessageType()),
								"both space and tab characters used to indent blocks at/before line %" PRIu32 "\n", gb.GetLineNumber());
		warnedAboutMixedSpacesAndTabs = true;
	}
}

// Check for and process a conditional GCode language command returning true if we found one, false if it's a regular line of GCode that we need to process
// If we just finished skipping an if- or elif-block when the condition was false then 'skippedBlockType' is the type of that block, else it is BlockType::plain
bool StringParser::ProcessConditionalGCode(const StringRef& reply, BlockType skippedBlockType, bool doingFile) THROWS(GCodeException)
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

	if (i >= 2 && i < 9
		&& (gb.buffer[i] == 0 || gb.buffer[i] == ' ' || gb.buffer[i] == '\t' || gb.buffer[i] == '{' || gb.buffer[i] == '"' || gb.buffer[i] == '(')	// if the command word is properly terminated
	   )
	{
		readPointer = i;
		const char *_ecv_array const command = gb.buffer;
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
			if (StringStartsWith(command, "var"))
			{
				ProcessVarOrGlobalCommand(false);
				return true;
			}
			if (StringStartsWith(command, "set"))
			{
				ProcessSetCommand();
				return true;
			}
			break;

		case 4:
			if (StringStartsWith(command, "echo"))
			{
				ProcessEchoCommand(reply);
				return true;
			}
			if (StringStartsWith(command, "skip"))
			{
				return true;
			}
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

		case 6:
			if (StringStartsWith(command, "global"))
			{
				ProcessVarOrGlobalCommand(true);
				return true;
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

void StringParser::ProcessIfCommand() THROWS(GCodeException)
{
	if (EvaluateCondition())
	{
		gb.GetBlockState().SetIfTrueBlock();
	}
	else
	{
		gb.GetBlockState().SetIfFalseNoneTrueBlock();
		indentToSkipTo = gb.GetBlockIndent();			// skip forwards to the end of the block
	}
}

void StringParser::ProcessElseCommand(BlockType skippedBlockType) THROWS(GCodeException)
{
	// Users may be tempted to put a command after 'else' so flag an error if that happens
	SkipWhiteSpace();
	if (gb.buffer[readPointer] != 0)
	{
		throw ConstructParseException("unexpected characters after 'else'");
	}

	if (skippedBlockType == BlockType::ifFalseNoneTrue)
	{
		gb.GetBlockState().SetPlainBlock();				// execute the else-block, treating it like a plain block
	}
	else if (skippedBlockType == BlockType::ifFalseHadTrue || gb.GetBlockState().GetType() == BlockType::ifTrue)
	{
		indentToSkipTo = gb.GetBlockIndent();			// skip forwards to the end of the if-block
		gb.GetBlockState().SetPlainBlock();				// so that we get an error if there is another 'else' part
	}
	else
	{
		throw ConstructParseException("'else' did not follow 'if'");
	}
}

void StringParser::ProcessElifCommand(BlockType skippedBlockType) THROWS(GCodeException)
{
	if (skippedBlockType == BlockType::ifFalseNoneTrue)
	{
		if (EvaluateCondition())
		{
			gb.GetBlockState().SetIfTrueBlock();
		}
		else
		{
			indentToSkipTo = gb.GetBlockIndent();		// skip forwards to the end of the elif-block
			gb.GetBlockState().SetIfFalseNoneTrueBlock();
		}
	}
	else if (skippedBlockType == BlockType::ifFalseHadTrue || gb.GetBlockState().GetType() == BlockType::ifTrue)
	{
		indentToSkipTo = gb.GetBlockIndent();			// skip forwards to the end of the if-block
		gb.GetBlockState().SetIfFalseHadTrueBlock();
	}
	else
	{
		throw ConstructParseException("'elif' did not follow 'if");
	}
}

void StringParser::ProcessWhileCommand() THROWS(GCodeException)
{
	// Set the current block as a loop block first so that we may use 'iterations' in the condition
	if (gb.GetBlockState().GetType() == BlockType::loop)
	{
		gb.GetBlockState().IncrementIterations();		// starting another iteration
	}
	else
	{
		gb.GetBlockState().SetLoopBlock(GetFilePosition(), gb.GetLineNumber() - 1);
	}

	if (!EvaluateCondition())
	{
		gb.GetBlockState().SetPlainBlock();
		indentToSkipTo = gb.GetBlockIndent();			// skip forwards to the end of the block
	}
}

void StringParser::ProcessBreakCommand() THROWS(GCodeException)
{
	do
	{
		if (gb.GetBlockIndent() == 0)
		{
			throw ConstructParseException("'break' was not inside a loop");
		}
		gb.CurrentFileMachineState().EndBlock();
	} while (gb.GetBlockState().GetType() != BlockType::loop);
	gb.GetBlockState().SetPlainBlock();
	indentToSkipTo = gb.GetBlockIndent();				// skip forwards to the end of the loop
}

void StringParser::ProcessContinueCommand() THROWS(GCodeException)
{
	do
	{
		if (gb.GetBlockIndent() == 0)
		{
			throw ConstructParseException("'continue' was not inside a loop");
		}
		gb.CurrentFileMachineState().EndBlock();
	} while (gb.GetBlockState().GetType() != BlockType::loop);

	// Go back to the start of the loop and re-evaluate the while-part
	gb.CurrentFileMachineState().lineNumber = gb.GetBlockState().GetLineNumber();
#if SUPPORT_ASYNC_MOVES
	gb.CurrentFileMachineState().fpos = gb.GetBlockState().GetFilePosition();
#endif
	gb.RestartFrom(gb.GetBlockState().GetFilePosition());
}

void StringParser::ProcessVarOrGlobalCommand(bool isGlobal) THROWS(GCodeException)
{
#if SUPPORT_ASYNC_MOVES
	if (isGlobal && !gb.Executing()) return;
#endif

	SkipWhiteSpace();

	// Get the identifier
	char c = gb.buffer[readPointer];
	if (!isAlpha(c))
	{
		throw ConstructParseException("expected a new variable name");
	}
	String<MaxVariableNameLength> varName;
	do
	{
		varName.cat(c);
		++readPointer;
		c = gb.buffer[readPointer];
	} while (isAlnum(c) || c == '_' );

	// Expect '='
	SkipWhiteSpace();
	if (gb.buffer[readPointer] != '=')
	{
		throw ConstructParseException("expected '='");
	}
	++readPointer;

	// Check whether the identifier already exists
	WriteLockedPointer<VariableSet> vset = (isGlobal)
											? reprap.GetGlobalVariablesForWriting()
												: WriteLockedPointer<VariableSet>(nullptr, &gb.GetVariables());
	Variable *_ecv_null const v = vset->Lookup(varName.c_str(), false);
	if (v != nullptr)
	{
		// For now we don't allow an existing variable to be reassigned using a 'var' or 'global' statement. We may need to allow it for 'global' statements.
		throw ConstructParseException("variable '%s' already exists", varName.c_str());
	}

	SkipWhiteSpace();
	ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
	ExpressionValue ev = parser.Parse();
	vset->InsertNew(varName.c_str(), ev, (isGlobal) ? 0 : (int)gb.CurrentFileMachineState().GetBlockNesting());
	if (isGlobal)
	{
		reprap.GlobalUpdated();
	}
}

void StringParser::ProcessSetCommand() THROWS(GCodeException)
{
	SkipWhiteSpace();
	const bool isGlobal = StringStartsWith(gb.buffer + readPointer, "global.");

#if SUPPORT_ASYNC_MOVES
	if (isGlobal && !gb.Executing()) return;
#endif

	// Skip the "var." or "global." prefix and get access to the appropriate variable set
	WriteLockedPointer<VariableSet> vset = (isGlobal)
											? (readPointer += (int)strlen("global."), reprap.GetGlobalVariablesForWriting())
											: (StringStartsWith(gb.buffer + readPointer, "var."))
											  	? (readPointer += (int)strlen("var."), WriteLockedPointer<VariableSet>(nullptr, &gb.GetVariables()))
												: throw ConstructParseException("expected a global or local variable");

	// Get the identifier
	char c = gb.buffer[readPointer];
	if (!isAlpha(c))
	{
		throw ConstructParseException("expected a new variable name");
	}

	String<MaxVariableNameLength> varName;
	do
	{
		varName.cat(c);
		++readPointer;
		c = gb.buffer[readPointer];
	} while (isAlnum(c) || c == '_' );

	// Look up the identifier
	Variable *_ecv_null const var = vset->Lookup(varName.c_str(), false);
	if (var == nullptr)
	{
		throw ConstructParseException("unknown variable '%s'", varName.c_str());
	}

	SkipWhiteSpace();

	// Check for index expressions after the identifier
	uint32_t indices[MaxExpressionArrayIndices];
	size_t numIndices = 0;
	for (numIndices = 0; ; ++numIndices)
	{
		SkipWhiteSpace();
		if (gb.buffer[readPointer] != '[')
		{
			break;
		}
		if (numIndices == MaxExpressionArrayIndices)
		{
			throw ConstructParseException("Too many array indices");
		}

		++readPointer;
		ExpressionParser indexParser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		const uint32_t indexExpr = indexParser.ParseUnsigned();
		readPointer = indexParser.GetEndptr() - gb.buffer;
		if (gb.buffer[readPointer] != ']')
		{
			throw ConstructParseException("expected ']'");
		}
		indices[numIndices] = indexExpr;
		++readPointer;
	}

	// Expect '='
	if (gb.buffer[readPointer] != '=')
	{
		throw ConstructParseException("expected '='");
	}
	++readPointer;

	ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
	ExpressionValue ev = parser.Parse();

	if (numIndices == 0)
	{
		var->Assign(ev);
	}
	else
	{
		var->AssignIndexed(ev, numIndices, indices);
	}

	if (isGlobal)
	{
		reprap.GlobalUpdated();
	}
}

void StringParser::ProcessAbortCommand(const StringRef& reply) noexcept
{
#if SUPPORT_ASYNC_MOVES
	if (!gb.Executing()) return;
#endif

	SkipWhiteSpace();
	if (gb.buffer[readPointer] != 0)
	{
		// If we fail to parse the expression, we want to abort anyway
		try
		{
			ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
			const ExpressionValue val = parser.Parse();
			readPointer = parser.GetEndptr() - gb.buffer;
			val.AppendAsString(reply);
		}
		catch (const GCodeException& e)
		{
			e.GetMessage(reply, &gb);
			reply.Insert(0, "invalid expression after 'abort': ");
		}
	}
	else
	{
		reply.copy("'abort' command executed");
	}

	reprap.GetGCodes().AbortPrint(gb);
}

void StringParser::ProcessEchoCommand(const StringRef& reply) THROWS(GCodeException)
{
#if SUPPORT_ASYNC_MOVES
	if (!gb.Executing()) return;
#endif

	SkipWhiteSpace();

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	FileData outputFile;
	bool appendNewline = true;
#endif

	if (gb.buffer[readPointer] == '>')
	{
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
		// Redirect the line to file
		++readPointer;
		OpenMode openMode;
		if (gb.buffer[readPointer] == '>')
		{
			openMode = OpenMode::append;
			++readPointer;
			if (gb.buffer[readPointer] == '>')
			{
				appendNewline = false;
				++readPointer;
			}
		}
		else
		{
			openMode = OpenMode::write;
		}
		String<MaxFilenameLength> filename;
		GetQuotedString(filename.GetRef(), false);
		FileStore *_ecv_null const f = reprap.GetPlatform().OpenSysFile(filename.c_str(), openMode);
		if (f == nullptr)
		{
			throw GCodeException(&gb, readPointer + (int)commandIndent, "Failed to create or open file");
		}
		outputFile.Set(f);
#else
		throw GCodeException(&gb, readPointer + commandIndent, "Can't write to this file system");
#endif
	}

	while (true)
	{
		SkipWhiteSpace();
		if (gb.buffer[readPointer] == 0)
		{
			break;
		}
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		const ExpressionValue val = parser.Parse();
		readPointer = parser.GetEndptr() - gb.buffer;
		if (!reply.IsEmpty())
		{
			reply.cat(' ');
		}
		val.AppendAsString(reply);
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

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	if (outputFile.IsLive())
	{
		if (appendNewline)
		{
			reply.cat('\n');
		}
		const bool ok = outputFile.Write(reply.c_str());
		outputFile.Close();
		reply.Clear();
		if (!ok)
		{
			throw GCodeException(&gb, -1, "Failed to write to redirect file");
		}
	}
#endif
}

// Evaluate the condition that should follow 'if' or 'while'
bool StringParser::EvaluateCondition() THROWS(GCodeException)
{
	ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
	const bool b = parser.ParseBoolean();
	parser.CheckForExtraCharacters();
	return b;
}

// Decode this command and find the start of the next one on the same line.
// On entry, 'commandStart' has already been set to the address the start of where the command should be
// and 'commandIndent' is the number of leading whitespace characters at the start of the current line.
// On return, the state must be set to 'ready' to indicate that a command is available and we should stop adding characters.
void StringParser::DecodeCommand() noexcept
{
	// Check for a valid command letter at the start
	char cl = gb.buffer[commandStart];
	if (cl == '\'')									// check for a lowercase axis letter in Fanuc mode
	{
		++commandStart;
		cl = (char)tolower(gb.buffer[commandStart]);
	}
	else
	{
		cl = (char)toupper(cl);
	}
	commandFraction = -1;
	if (cl == 'G' || cl == 'M' || cl == 'T')
	{
		commandLetter = cl;
		hasCommandNumber = false;
		commandNumber = -1;
		if (cl == 'T' && gb.buffer[commandStart + 1] == '{')
		{
			// It's a T command with an expression for the tool number. This will be treated as if it's "T T{...}.
			parameterStart = commandStart; 			// so that 'Seen('T')' will return true
		}
		else
		{
			parameterStart = commandStart + 1;
			const bool negative = (gb.buffer[parameterStart] == '-');
			if (negative)
			{
				++parameterStart;
			}
			if (isDigit(gb.buffer[parameterStart]))
			{
				hasCommandNumber = true;
				// Read the number after the command letter
				commandNumber = 0;
				do
				{
					commandNumber = (10 * commandNumber) + (gb.buffer[parameterStart] - '0');
					++parameterStart;
				}
				while (isDigit(gb.buffer[parameterStart]));
				if (negative)
				{
					commandNumber = -commandNumber;
				}

				// Read the fractional digit, if any
				if (gb.buffer[parameterStart] == '.')
				{
					++parameterStart;
					if (isDigit(gb.buffer[parameterStart]))
					{
						commandFraction = gb.buffer[parameterStart] - '0';
						++parameterStart;
					}
				}
			}
		}

		// Skip any whitespace after the command letter/number. This speeds up searching for parameters and is assumed by GetUnprecedentedString.
		while (parameterStart < gcodeLineEnd && (gb.buffer[parameterStart] == ' ' || gb.buffer[parameterStart] == '\t'))
		{
			++parameterStart;
		}

		FindParameters();
	}
	else if (cl == ';')
	{
		// It's a whole line comment without indentation. Turn it into an internal Q0 command.
		overflowed = false;															// we can get very long comment lines and we don't mind if they are truncated
		commandLetter = 'Q';
		commandNumber = 0;
		hasCommandNumber = true;
		parameterStart = 1;															// there is a single unquoted string parameter, which is the remainder of the line
		commandEnd = gcodeLineEnd;
	}
	else if (   hasCommandNumber
			 && commandLetter == 'G'
			 && commandNumber <= 3
			 && strchr(reprap.GetGCodes().GetAxisLetters(), cl) != nullptr			// this assumes that the first letter will always be an axis coordinate
			 && (   reprap.GetGCodes().GetMachineType() == MachineType::cnc			// Fanuc style CNC
				 || reprap.GetGCodes().GetMachineType() == MachineType::laser		// LaserWeb style
				)
			 && !isAlpha(gb.buffer[commandStart + 1])								// make sure it isn't an if-command or other meta command
			)
	{
		// Fanuc or LaserWeb-style GCode, repeat the existing G0/G1/G2/G3 command with the new parameters
		parameterStart = commandStart;
		FindParameters();
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

// Find where the end of the command is. We assume that a G or M not inside quotes or { } and not preceded by ' is the start of a new command.
// This isn't true if the command has an unquoted string argument, but we deal with that later.
void StringParser::FindParameters() noexcept
{
	bool inQuotes = false;
	bool escaped = false;
	unsigned int localBraceCount = 0;
	parametersPresent.Clear();
	for (commandEnd = parameterStart; commandEnd < gcodeLineEnd; ++commandEnd)
	{
		const char c = gb.buffer[commandEnd];
		if (c == '\'')
		{
			escaped = !inQuotes;
		}
		else
		{
			if (c == '"')
			{
				inQuotes = !inQuotes;
			}
			else if (!inQuotes)
			{
				if (c == '{')
				{
					++localBraceCount;
				}
				else if (localBraceCount != 0)
				{
					if (c == '}')
					{
						--localBraceCount;
					}
				}
				else
				{
					const char c2 = (char)toupper(c);
					if (escaped)
					{
						if (c2 >= 'A' && c2 <= (char)toupper(HighestAxisLetter) && (c2 != 'E' || commandEnd == parameterStart || !isDigit(gb.buffer[commandEnd - 1])))
						{
							parametersPresent.SetBit(c2 - ('A' - 26));
						}
					}
					else if (c2 == 'G' || c2 == 'M')
					{
						break;
					}
					else if (c2 >= 'A' && c2 <= 'Z' && (c2 != 'E' || commandEnd == parameterStart || !isDigit(gb.buffer[commandEnd - 1])))
					{
						parametersPresent.SetBit(c2 - 'A');
					}
				}
			}
			escaped = false;
		}
	}
}

// Add an entire string, overwriting any existing content and adding '\n' at the end if necessary to make it a complete line
void StringParser::PutAndDecode(const char *_ecv_array str, size_t len) noexcept
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

void StringParser::PutAndDecode(const char *_ecv_array str) noexcept
{
	PutAndDecode(str, strlen(str));
}

// Put a complete command but don't decode it
void StringParser::PutCommand(const char *_ecv_array str) noexcept
{
	char c;
	do
	{
		c = *str++;
		Put(c);
	} while (c != 0);
}

void StringParser::ResetIndentationAfterPop() noexcept
{
	if (indentToSkipTo != NoIndentSkip)
	{
		indentToSkipTo = (gb.GetBlockIndent() > 0) ? gb.GetBlockIndent() : NoIndentSkip;
	}
}

// Flag this command as completed
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
		gb.LatestMachineState().g53Active = false;		// G53 does not persist beyond the current line
		Init();
	}
}

// Get the file position at the start of the current command.
// This is called to get the file position within the current job file so that we can save the position in the DDAs to facilitate pause and resume.
// It is also called to get the position within a macro file when executing a while loop in that file.
FilePosition StringParser::GetFilePosition() const noexcept
{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	const GCodeMachineState *ms = gb.machineState;
	while (ms->waitingForAcknowledgement && ms->GetPrevious() != nullptr)
	{
		ms = ms->GetPrevious();
	}

	if (ms->DoingFile()
# if HAS_SBC_INTERFACE
		&& !reprap.UsingSbcInterface()
# endif
	   )
	{
		const FileData &file = ms->fileState;
		return file.GetPosition() - gb.fileInput->FileBytesCached(file) - commandLength + commandStart;
	}
#endif
	return noFilePosition;
}

const char *_ecv_array StringParser::DataStart() const noexcept
{
	return gb.buffer + commandStart;
}

size_t StringParser::DataLength() const noexcept
{
	return commandEnd - commandStart;
}

// Return true if the command being processed is the last one in this line of GCode
bool StringParser::IsLastCommand() const noexcept
{
	return commandEnd >= gcodeLineEnd;			// using >= here also covers the case where the buffer is empty and gcodeLineEnd has been set to zero
}

// Is 'c' in the G Code string? 'c' must be in A..Z or a..HighestAxisLetter
// Leave the pointer one after it for a subsequent read.
bool StringParser::Seen(char c) noexcept
{
	const bool wantLowerCase = (c >= 'a');
	unsigned int bit;
	if (wantLowerCase)
	{
		bit = c - ('a' - 26);
		c = (char)toupper(c);
	}
	else
	{
		bit = c - 'A';
	}
	if (bit >= ParameterLettersBitmap::MaxBits() || !parametersPresent.IsBitSet(bit))
	{
		return false;
	}

	bool inQuotes = false;
	bool escaped = false;
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
			if (b == '\'' && !escaped)
			{
				escaped = true;
			}
			else
			{
				if (   inBrackets == 0
					&& (char)toupper(b) == c
					&& escaped == wantLowerCase
					&& (c != 'E' || (unsigned int)readPointer == parameterStart || !isDigit(gb.buffer[readPointer - 1]))
				   )
				{
					++readPointer;
					characterSeen = c;
					return true;
				}
				escaped = false;
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
	}
	readPointer = -1;
	return false;
}

// Get a float after a G Code letter found by a call to Seen()
float StringParser::GetFValue() THROWS(GCodeException)
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
void StringParser::GetFloatArray(float arr[], size_t& returnedLength) THROWS(GCodeException)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		parser.ParseFloatArray(arr, returnedLength);
	}
	else
	{
		size_t length = 0;
		for (;;)
		{
			CheckArrayLength(length, returnedLength);
			arr[length++] = ReadFloatValue();
			if (gb.buffer[readPointer] != LIST_SEPARATOR)
			{
				break;
			}
			++readPointer;
		}

		returnedLength = length;
	}

	readPointer = -1;
}

// Get a :-separated list of ints after a key letter
void StringParser::GetIntArray(int32_t arr[], size_t& returnedLength) THROWS(GCodeException)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		parser.ParseIntArray(arr, returnedLength);
	}
	else
	{
		size_t length = 0;
		for (;;)
		{
			CheckArrayLength(length, returnedLength);
			arr[length] = ReadIValue();
			length++;
			if (gb.buffer[readPointer] != LIST_SEPARATOR)
			{
				break;
			}
			++readPointer;
		}

		returnedLength = length;
	}

	readPointer = -1;
}

// Get a :-separated list of unsigned ints after a key letter
void StringParser::GetUnsignedArray(uint32_t arr[], size_t& returnedLength) THROWS(GCodeException)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		parser.ParseUnsignedArray(arr, returnedLength);
	}
	else
	{
		size_t length = 0;
		for (;;)
		{
			CheckArrayLength(length, returnedLength);
			arr[length] = ReadUIValue();
			length++;
			if (gb.buffer[readPointer] != LIST_SEPARATOR)
			{
				break;
			}
			++readPointer;
		}

		returnedLength = length;
	}

	readPointer = -1;
}

// Get a :-separated list of drivers after a key letter
void StringParser::GetDriverIdArray(DriverId arr[], size_t& returnedLength) THROWS(GCodeException)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		parser.ParseDriverIdArray(arr, returnedLength);
	}
	else
	{
		size_t length = 0;
		for (;;)
		{
			CheckArrayLength(length, returnedLength);
			arr[length] = ReadDriverIdValue();
			length++;
			if (gb.buffer[readPointer] != LIST_SEPARATOR)
			{
				break;
			}
			++readPointer;
		}

		returnedLength = length;
	}

	readPointer = -1;
}

// Get a :-separated list of strings after a key letter
ExpressionValue StringParser::GetExpression() THROWS(GCodeException)
{
	if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		return parser.Parse();
	}
	throw ConstructParseException("expected an expression inside { }");
}

void StringParser::CheckArrayLength(size_t actualLength, size_t maxLength) THROWS(GCodeException)
{
	if (actualLength >= maxLength)
	{
		throw ConstructParseException("array too long for parameter '%c'", (uint32_t)characterSeen);
	}
}

// Get and copy a quoted string returning true if successful, leaving the read pointer just after the string
void StringParser::GetQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
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
		{
			ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
			const ExpressionValue val = parser.Parse();
			readPointer = parser.GetEndptr() - gb.buffer;
			val.AppendAsString(str);
		}
		break;

	default:
		throw ConstructParseException("expected a string expression");
	}

	if (!allowEmpty && str.IsEmpty())
	{
		throw ConstructParseException("expected a non-empty string");
	}
}

// Given that the current character is double-quote, fetch the quoted string, leaving the read pointer just after the string
void StringParser::InternalGetQuotedString(const StringRef& str) THROWS(GCodeException)
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
			if (isAlpha(gb.buffer[readPointer]))
			{
				// Single quote before an alphabetic character forces that character to lower case
				c = (char)tolower(gb.buffer[readPointer++]);
			}
			else if (gb.buffer[readPointer] == c)
			{
				// Two single quote characters are used to represent one
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
void StringParser::GetPossiblyQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	InternalGetPossiblyQuotedString(str);
	if (!allowEmpty && str.IsEmpty())
	{
		throw ConstructParseException("expected a non-empty string");
	}
}

// Get and copy a string which may or may not be quoted, starting at readPointer. Return true if successful.
void StringParser::InternalGetPossiblyQuotedString(const StringRef& str) THROWS(GCodeException)
{
	str.Clear();
	if (gb.buffer[readPointer] == '"')
	{
		InternalGetQuotedString(str);
	}
	else if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		const ExpressionValue val = parser.Parse();
		readPointer = parser.GetEndptr() - gb.buffer;
		val.AppendAsString(str);
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

// This returns a string comprising the rest of the line, excluding any comment
// It is provided for legacy use, in particular in the M23 and similar commands that set the name of a file to be printed.
// Leading spaces and tabs have already been skipped in DecodeCommand.
void StringParser::GetUnprecedentedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
{
	readPointer = parameterStart;
	InternalGetPossiblyQuotedString(str);
	if (!allowEmpty && str.IsEmpty())
	{
		throw ConstructParseException("expected a non-empty string");
	}
}

// Get the complete parameter string
void StringParser::GetCompleteParameters(const StringRef& str) const noexcept
{
	str.copy(gb.buffer + parameterStart);
}

// Get an int32 after a G Code letter
int32_t StringParser::GetIValue() THROWS(GCodeException)
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
uint32_t StringParser::GetUIValue() THROWS(GCodeException)
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
DriverId StringParser::GetDriverId() THROWS(GCodeException)
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
void StringParser::GetIPAddress(IPAddress& returnedIp) THROWS(GCodeException)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	const char *_ecv_array p = gb.buffer + readPointer;
	uint8_t ip[4];
	unsigned int n = 0;
	for (;;)
	{
		const char *_ecv_array pp;
		const uint32_t v = StrToU32(p, &pp);
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
void StringParser::GetMacAddress(MacAddress& mac) THROWS(GCodeException)
{
	if (readPointer <= 0)
	{
		THROW_INTERNAL_ERROR;
	}

	const char *_ecv_array p = gb.buffer + readPointer;
	unsigned int n = 0;
	for (;;)
	{
		const char *_ecv_array pp;
		const unsigned long v = StrHexToU32(p, &pp);
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
// This is used when logging and also called when we report a "Bad command" error, so make sure we display any control characters.
void StringParser::AppendFullCommand(const StringRef &s) const noexcept
{
	// Don't include any trailing tabs or spaces (e.g. before an end-of-line comment)
	size_t cmdEnd = commandEnd;
	{
		char c;
		while (cmdEnd != 0 && ((c = gb.buffer[cmdEnd - 1]) == ' ' || c == '\t'))
		{
			--cmdEnd;
		}
	}

	for (size_t i = commandStart; i < cmdEnd; ++i)
	{
		const char c = gb.buffer[i];
		if (c < (char)0x20u)
		{
			s.catf("[0x%02x]", (unsigned int)c);
		}
		else
		{
			s.cat(c);
		}
	}
}

// Called when we start a new file
void StringParser::StartNewFile() noexcept
{
	seenLeadingSpace = seenLeadingTab = seenMetaCommand = warnedAboutMixedSpacesAndTabs = false;
}

#if HAS_MASS_STORAGE

// Open a file to write to
bool StringParser::OpenFileToWrite(const char *_ecv_array directory, const char *_ecv_array fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) noexcept
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
			const char *_ecv_array const r = (gb.LatestMachineState().compatibility == Compatibility::Marlin) ? "Done saving file." : "";
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

	size_t indent = commandIndent;
	while (indent != 0)
	{
		fileBeingWritten->Write(' ');
		--indent;
	}
	fileBeingWritten->Write(gb.buffer);
	fileBeingWritten->Write('\n');
	Init();
}

// Write a character to file, returning true if we finished doing the binary upload
bool StringParser::WriteBinaryToFile(char b) noexcept
{
	if (b == eofString[eofStringCounter] && writingFileSize == 0)
	{
		eofStringCounter++;
		if (eofStringCounter < ARRAY_SIZE(eofString) - 1)
		{
			return false;					// not reached end of input yet
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
			return false;					// not reached end of input yet
		}
	}

	FinishWritingBinary();
	return true;
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
		const char *_ecv_array const r = (gb.LatestMachineState().compatibility == Compatibility::Marlin) ? "Done saving file." : "";
		reprap.GetGCodes().HandleReply(gb, GCodeResult::ok, r);
	}
	else
	{
		reprap.GetGCodes().HandleReply(gb, GCodeResult::error, "CRC32 checksum doesn't match");
	}
}

#endif

// This is called when we reach the end of the file we are reading from. Return true if there is a line waiting to be processed.
bool StringParser::FileEnded() noexcept
{
#if HAS_MASS_STORAGE
	if (IsWritingBinary())
	{
		// We are in the middle of writing a binary file but the input stream has ended
		FinishWritingBinary();
		Init();
		return false;
	}
#endif

	bool commandCompleted = false;
	if (gcodeLineEnd != 0)				// if there is something in the buffer
	{
		Put('\n');						// append a newline in case the file didn't end with one
		commandCompleted = true;
	}

#if HAS_MASS_STORAGE
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
		const char *_ecv_array const r = (gb.LatestMachineState().compatibility == Compatibility::Marlin) ? "Done saving file." : "";
		reprap.GetGCodes().HandleReply(gb, GCodeResult::ok, r);
		return false;
	}
#endif

	if (!commandCompleted && gb.CurrentFileMachineState().GetIterations() >= 0)
	{
		// We reached the end of the file while inside a loop. Insert a dummy 'skip' command to allow the loop processing to complete.
		Init();
		PutCommand("skip");
		commandCompleted = true;
	}
	return commandCompleted;
}

// Check that a number was found. If it was, advance readPointer past it. Otherwise throw an exception.
void StringParser::CheckNumberFound(const char *_ecv_array endptr) THROWS(GCodeException)
{
	if (endptr == gb.buffer + readPointer)
	{
		throw ConstructParseException("expected number after '%c'", (uint32_t)gb.buffer[readPointer - 1]);
	}
	readPointer = endptr - gb.buffer;
}

// Functions to read values from lines of GCode, allowing for expressions and variable substitution
float StringParser::ReadFloatValue() THROWS(GCodeException)
{
	if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		const float val = parser.ParseFloat();
		readPointer = parser.GetEndptr() - gb.buffer;
		return val;
	}

	const char *_ecv_array endptr;
	const float rslt = SafeStrtof(gb.buffer + readPointer, &endptr);
	CheckNumberFound(endptr);
	return rslt;
}

uint32_t StringParser::ReadUIValue() THROWS(GCodeException)
{
	if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		const uint32_t val = parser.ParseUnsigned();
		readPointer = parser.GetEndptr() - gb.buffer;
		return val;
	}

	// Allow "0xNNNN" or "xNNNN" where NNNN are hex digits. We could stop supporting this because we already support {0xNNNN}.
	const char *_ecv_array endptr;
	const uint32_t rslt = StrToU32(gb.buffer + readPointer, &endptr);
	CheckNumberFound(endptr);
	return rslt;
}

int32_t StringParser::ReadIValue() THROWS(GCodeException)
{
	if (gb.buffer[readPointer] == '{')
	{
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		const int32_t val = parser.ParseInteger();
		readPointer = parser.GetEndptr() - gb.buffer;
		return val;
	}

	const char *_ecv_array endptr;
	const int32_t rslt = StrToI32(gb.buffer + readPointer, &endptr);
	CheckNumberFound(endptr);
	return rslt;
}

DriverId StringParser::ReadDriverIdValue() THROWS(GCodeException)
{
	DriverId result;
	if (gb.buffer[readPointer] == '{')
	{
		// Allow a floating point expression to be converted to a driver ID
		// We assume that a driver ID only ever has a single fractional digit. This means that e.g. 3.10 will be treated the same as 3.1.
		ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + ARRAY_SIZE(gb.buffer), (int)commandIndent + readPointer);
		const float val = 10.0 * parser.ParseFloat();
		readPointer = parser.GetEndptr() - gb.buffer;
		const int32_t ival = lrintf(val);
#if SUPPORT_CAN_EXPANSION
		if (ival >= 0 && fabsf(val - (float)ival) <= 0.002)
		{
			result.boardAddress = ival/10;
			result.localDriver = ival % 10;
		}
#else
		if (ival >= 0 && ival < 10 && fabsf(val - (float)ival) <= 0.002)
		{
			result.localDriver = ival % 10;
		}
#endif
		else
		{
			throw ConstructParseException("Invalid driver ID expression");
		}
	}
	else
	{
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
			result.boardAddress = CanInterface::GetCanAddress();
		}
#else
		// We now allow driver names of the form "0.x" on boards without CAN expansion
		if (gb.buffer[readPointer] == '.')
		{
			if (v1 != 0)
			{
				throw ConstructParseException("Board address of driver must be 0");
			}
			++readPointer;
			result.localDriver = ReadUIValue();
		}
		else
		{
			result.localDriver = v1;
		}
#endif
	}
	return result;
}

void StringParser::SkipWhiteSpace() noexcept
{
	while (gb.buffer[readPointer] == ' ' || gb.buffer[readPointer] == '\t')
	{
		++readPointer;
	}
}

void StringParser::AddParameters(VariableSet& vs, int codeRunning) THROWS(GCodeException)
{
	parametersPresent.IterateWithExceptions([this, &vs, codeRunning](unsigned int bit, unsigned int count)
								{
									const char letter = BitNumberToParameterLetter(bit);
									if ((letter != 'P' || codeRunning != 98) && Seen(letter))
									{
										const char c = gb.buffer[readPointer];
										if (!isDigit(c) && c != '"' && c != '{' && c != '.' && c != '-' && c != '+')
										{
											throw ConstructParseException("invalid value for parameter '%c'", (uint32_t)c);
										}
										ExpressionParser parser(&gb, gb.buffer + readPointer, gb.buffer + commandEnd);
										ExpressionValue ev = parser.Parse();
										char paramName[2] = { letter, 0 };
										vs.InsertNewParameter(paramName, ev);
									}
								}
							  );
}

GCodeException StringParser::ConstructParseException(const char *_ecv_array str) const noexcept
{
	return GCodeException(&gb, GetColumn(), str);
}

GCodeException StringParser::ConstructParseException(const char *_ecv_array str, const char *_ecv_array param) const noexcept
{
	return GCodeException(&gb, GetColumn(), str, param);
}

GCodeException StringParser::ConstructParseException(const char *_ecv_array str, uint32_t param) const noexcept
{
	return GCodeException(&gb, GetColumn(), str, param);
}

// Get the current column if we can, else return -1
int StringParser::GetColumn() const noexcept
{
	return (readPointer < 0) ? -1 : readPointer + (int)commandIndent;
}

// End
