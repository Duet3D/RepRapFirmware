/*
 * GCodeBuffer.cpp
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

//*************************************************************************************

#include "GCodeBuffer.h"
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
# include <GCodes/GCodeInput.h>
#endif
#if HAS_SBC_INTERFACE
# include <SBC/SbcInterface.h>
#endif
#include "BinaryParser.h"
#include "StringParser.h"
#include <GCodes/GCodeException.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Movement/StepTimer.h>

// Macros to reduce the amount of explicit conditional compilation in this file
#if HAS_SBC_INTERFACE

# define PARSER_OPERATION(_x)	((isBinaryBuffer) ? (binaryParser._x) : (stringParser._x))
# define IS_BINARY_OR(_x)		((isBinaryBuffer) || (_x))
# define NOT_BINARY_AND(_x)		((!isBinaryBuffer) && (_x))
# define IF_NOT_BINARY(_x)		{ if (!isBinaryBuffer) { _x; } }

#else

# define PARSER_OPERATION(_x)	(stringParser._x)
# define IS_BINARY_OR(_x)		(_x)
# define NOT_BINARY_AND(_x)		(_x)
# define IF_NOT_BINARY(_x)		{ _x; }

#endif

#if SUPPORT_OBJECT_MODEL
// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(GCodeBuffer, __VA_ARGS__)

constexpr ObjectModelTableEntry GCodeBuffer::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. inputs[] root
	{ "axesRelative",		OBJECT_MODEL_FUNC((bool)self->machineState->axesRelative),							ObjectModelEntryFlags::none },
	{ "compatibility",		OBJECT_MODEL_FUNC(self->machineState->compatibility.ToString()),					ObjectModelEntryFlags::none },
	{ "distanceUnit",		OBJECT_MODEL_FUNC(self->GetDistanceUnits()),										ObjectModelEntryFlags::none },
	{ "drivesRelative",		OBJECT_MODEL_FUNC((bool)self->machineState->drivesRelative),						ObjectModelEntryFlags::none },
	{ "feedRate",			OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerSec(self->machineState->feedRate), 1),	ObjectModelEntryFlags::live },
	{ "inMacro",			OBJECT_MODEL_FUNC((bool)self->machineState->doingFileMacro),						ObjectModelEntryFlags::live },
	{ "lineNumber",			OBJECT_MODEL_FUNC((int32_t)self->GetLineNumber()),									ObjectModelEntryFlags::live },
	{ "macroRestartable",	OBJECT_MODEL_FUNC((bool)self->machineState->macroRestartable),						ObjectModelEntryFlags::none },
	{ "name",				OBJECT_MODEL_FUNC(self->codeChannel.ToString()),									ObjectModelEntryFlags::none },
	{ "stackDepth",			OBJECT_MODEL_FUNC((int32_t)self->GetStackDepth()),									ObjectModelEntryFlags::none },
	{ "state",				OBJECT_MODEL_FUNC(self->GetStateText()),											ObjectModelEntryFlags::live },
	{ "volumetric",			OBJECT_MODEL_FUNC((bool)self->machineState->volumetricExtrusion),					ObjectModelEntryFlags::none },
};

constexpr uint8_t GCodeBuffer::objectModelTableDescriptor[] = { 1, 12 };

DEFINE_GET_OBJECT_MODEL_TABLE(GCodeBuffer)

const char *GCodeBuffer::GetStateText() const noexcept
{
	if (machineState->waitingForAcknowledgement)
	{
		return "awaitingAcknowledgement";
	}

	switch (bufferState)
	{
	case GCodeBufferState::parseNotStarted:		return "idle";
	case GCodeBufferState::ready:				return "executing";
	case GCodeBufferState::executing:			return "waiting";
	default:									return "reading";
	}
}

#endif

// Create a default GCodeBuffer
GCodeBuffer::GCodeBuffer(GCodeChannel::RawType channel, GCodeInput *normalIn, FileGCodeInput *fileIn, MessageType mt, Compatibility::RawType c) noexcept
	: codeChannel(channel), normalInput(normalIn),
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	  fileInput(fileIn),
#endif
	  responseMessageType(mt), lastResult(GCodeResult::ok),
#if HAS_SBC_INTERFACE
	  binaryParser(*this),
#endif
	  stringParser(*this),
	  machineState(new GCodeMachineState()), whenReportDueTimerStarted(millis()),
#if HAS_SBC_INTERFACE
	  isBinaryBuffer(false),
#endif
	  timerRunning(false), motionCommanded(false)
#if HAS_SBC_INTERFACE
	  , isWaitingForMacro(false), invalidated(false)
#endif
{
	mutex.Create(((GCodeChannel)channel).ToString());
	machineState->compatibility = c;
	Reset();
}

// Reset it to its state after start-up
void GCodeBuffer::Reset() noexcept
{
#if HAS_SBC_INTERFACE
	if (isWaitingForMacro)
	{
		ResolveMacroRequest(true, false);
	}
#endif

	while (PopState()) { }

#if HAS_SBC_INTERFACE
	isBinaryBuffer = false;
	requestedMacroFile.Clear();
	isWaitingForMacro = macroFileClosed = false;
	macroJustStarted = macroFileError = macroFileEmpty = abortFile = abortAllFiles = sendToSbc = messagePromptPending = messageAcknowledged = false;
	machineState->lastCodeFromSbc = machineState->macroStartedByCode = false;
#endif
	Init();
}

// Set it up to parse another G-code
void GCodeBuffer::Init() noexcept
{
#if HAS_SBC_INTERFACE
	sendToSbc = false;
	binaryParser.Init();
#endif
	stringParser.Init();
	timerRunning = false;
}

void GCodeBuffer::StartTimer() noexcept
{
	whenTimerStarted = millis();
	timerRunning = true;
}

// Delay executing this GCodeBuffer for the specified time. Return true when the timer has expired.
bool GCodeBuffer::DoDwellTime(uint32_t dwellMillis) noexcept
{
	const uint32_t now = millis();

	// Are we already in the dwell?
	if (timerRunning)
	{
		if (now - whenTimerStarted >= dwellMillis)
		{
			timerRunning = false;
			return true;
		}
		return false;
	}

	// New dwell - set it up
	StartTimer();
	return false;
}

// Delay executing this GCodeBuffer for the specified time. Return true when the timer has expired.
bool GCodeBuffer::IsReportDue() noexcept
{
	const uint32_t now = millis();

	// Are we due?
	if (now - whenReportDueTimerStarted >= reportDueInterval)
	{
		ResetReportDueTimer();
		return true;
	}
	return false;
}

// Write some debug info
void GCodeBuffer::Diagnostics(MessageType mtype) noexcept
{
	String<StringLength256> scratchString;
	scratchString.copy(codeChannel.ToString());
#if HAS_SBC_INTERFACE
	scratchString.cat(IsBinary() ? "* " : " ");
#else
	scratchString.cat(" ");
#endif
	switch (bufferState)
	{
	case GCodeBufferState::parseNotStarted:
		scratchString.cat("is idle");
		break;

	case GCodeBufferState::ready:
		scratchString.cat("is ready with \"");
		AppendFullCommand(scratchString.GetRef());
		scratchString.cat('"');
		break;

	case GCodeBufferState::executing:
		scratchString.cat("is doing \"");
		AppendFullCommand(scratchString.GetRef());
		scratchString.cat('"');
		break;

	default:
		scratchString.cat("is assembling a command");
	}

	scratchString.cat(" in state(s)");
	const GCodeMachineState *ms = machineState;
	do
	{
		scratchString.catf(" %d", (int)ms->GetState());
		ms = ms->GetPrevious();
	} while (ms != nullptr);
	if (IsDoingFileMacro())
	{
		scratchString.cat(", running macro");
	}
	scratchString.cat('\n');
	reprap.GetPlatform().Message(mtype, scratchString.c_str());
}

// Add a character to the end
bool GCodeBuffer::Put(char c) noexcept
{
#if HAS_SBC_INTERFACE
	machineState->lastCodeFromSbc = false;
	isBinaryBuffer = false;
#endif
	return stringParser.Put(c);
}

// Decode the command in the buffer when it is complete
void GCodeBuffer::DecodeCommand() noexcept
{
	PARSER_OPERATION(DecodeCommand());
}

// Check whether the current command is a meta command, or we are skipping a block. Return true if we are and the current line no longer needs to be processed.
bool GCodeBuffer::CheckMetaCommand(const StringRef& reply)
{
	return NOT_BINARY_AND(stringParser.CheckMetaCommand(reply));
}

#if HAS_SBC_INTERFACE

// Add an entire binary G-Code, overwriting any existing content
// CAUTION! This may be called with the task scheduler suspended, so don't do anything that might block or take more than a few microseconds to execute
void GCodeBuffer::PutBinary(const uint32_t *data, size_t len) noexcept
{
	machineState->lastCodeFromSbc = true;
	isBinaryBuffer = true;
	macroJustStarted = false;
	binaryParser.Put(data, len);
}

#endif

// Add an entire G-Code, overwriting any existing content
void GCodeBuffer::PutAndDecode(const char *str, size_t len) noexcept
{
#if HAS_SBC_INTERFACE
	machineState->lastCodeFromSbc = false;
	isBinaryBuffer = false;
#endif
	stringParser.PutAndDecode(str, len);
}

// Add a null-terminated string, overwriting any existing content
void GCodeBuffer::PutAndDecode(const char *str) noexcept
{
#if HAS_SBC_INTERFACE
	machineState->lastCodeFromSbc = false;
	isBinaryBuffer = false;
#endif
	stringParser.PutAndDecode(str);
}

void GCodeBuffer::StartNewFile() noexcept
{
#if HAS_SBC_INTERFACE
	machineState->SetFileExecuting();
#endif
	machineState->lineNumber = 0;						// reset line numbering when M32 is run
	IF_NOT_BINARY(stringParser.StartNewFile());
}

// Called when we reach the end of the file we are reading from. Return true if there is a line waiting to be processed.
bool GCodeBuffer::FileEnded() noexcept
{
	return NOT_BINARY_AND(stringParser.FileEnded());
}

char GCodeBuffer::GetCommandLetter() const noexcept
{
	return PARSER_OPERATION(GetCommandLetter());
}

bool GCodeBuffer::HasCommandNumber() const noexcept
{
	return PARSER_OPERATION(HasCommandNumber());
}

int GCodeBuffer::GetCommandNumber() const noexcept
{
	return PARSER_OPERATION(GetCommandNumber());
}

void GCodeBuffer::GetCompleteParameters(const StringRef& str) THROWS(GCodeException)
{
	PARSER_OPERATION(GetCompleteParameters(str));
}

int8_t GCodeBuffer::GetCommandFraction() const noexcept
{
	return PARSER_OPERATION(GetCommandFraction());
}

// Return true if the command we have just completed was the last command in the line of GCode.
// If the command was or called a macro then there will be no command in the buffer, so we must return true for this case also.
bool GCodeBuffer::IsLastCommand() const noexcept
{
	return 	IS_BINARY_OR((bufferState != GCodeBufferState::ready && bufferState != GCodeBufferState::executing) || stringParser.IsLastCommand());
}

bool GCodeBuffer::ContainsExpression() const noexcept
{
	return PARSER_OPERATION(ContainsExpression());
}

// Is a character present?
bool GCodeBuffer::Seen(char c) noexcept
{
	return PARSER_OPERATION(Seen(c));
}

// Return true if any of the parameter letters in the bitmap were seen
bool GCodeBuffer::SeenAny(Bitmap<uint32_t> bm) const noexcept
{
	return PARSER_OPERATION(SeenAny(bm));
}

// Test for character present, throw error if not
void GCodeBuffer::MustSee(char c) THROWS(GCodeException)
{
	if (!Seen(c))
	{
		throw GCodeException(GetLineNumber(), -1, "missing parameter '%c'", (uint32_t)c);
	}
}

// Test for one of two characters present, throw error if not saying that the first one is missing
char GCodeBuffer::MustSee(char c1, char c2) THROWS(GCodeException)
{
	if (Seen(c1)) { return c1; }
	if (Seen(c2)) { return c2; }
	throw GCodeException(GetLineNumber(), -1, "missing parameter '%c'", (uint32_t)c1);
}

// Get a float after a key letter
float GCodeBuffer::GetFValue() THROWS(GCodeException)
{
	return PARSER_OPERATION(GetFValue());
}

float GCodeBuffer::GetLimitedFValue(char c, float minValue, float maxValue) THROWS(GCodeException)
{
	MustSee(c);
	const float ret = GetFValue();
	if (ret < minValue)
	{
		throw GCodeException(GetLineNumber(), -1, "parameter '%c' too low", (uint32_t)c);
	}
	if (ret > maxValue)
	{
		throw GCodeException(GetLineNumber(), -1, "parameter '%c' too high", (uint32_t)c);
	}
	return ret;
}

// Get a distance or coordinate and convert it from inches to mm if necessary
float GCodeBuffer::GetDistance() THROWS(GCodeException)
{
	return ConvertDistance(GetFValue());
}

// Get a speed in mm/min or inches/min and convert it to mm/step_clock
float GCodeBuffer::GetSpeed() THROWS(GCodeException)
{
	return ConvertSpeed(GetFValue());
}

// Get a speed in mm/min mm/sec and convert it to mm/step_clock
float GCodeBuffer::GetSpeedFromMm(bool useSeconds) THROWS(GCodeException)
{
	return ConvertSpeedFromMm(GetFValue(), useSeconds);
}

// Get an acceleration in mm/sec^2 and convert it to mm/step_clock^2
float GCodeBuffer::GetAcceleration() THROWS(GCodeException)
{
	return ConvertAcceleration(GetFValue());
}

// Get an integer after a key letter
int32_t GCodeBuffer::GetIValue() THROWS(GCodeException)
{
	return PARSER_OPERATION(GetIValue());
}

// Get an integer with limit checking
int32_t GCodeBuffer::GetLimitedIValue(char c, int32_t minValue, int32_t maxValue) THROWS(GCodeException)
{
	MustSee(c);
	const int32_t ret = GetIValue();
	if (ret < minValue)
	{
		throw GCodeException(GetLineNumber(), -1, "parameter '%c' too low", (uint32_t)c);
	}
	if (ret > maxValue)
	{
		throw GCodeException(GetLineNumber(), -1, "parameter '%c' too high", (uint32_t)c);
	}
	return ret;
}

// Get an unsigned integer value
uint32_t GCodeBuffer::GetUIValue() THROWS(GCodeException)
{
	return PARSER_OPERATION(GetUIValue());
}

// Get an unsigned integer value, throw if >= limit
uint32_t GCodeBuffer::GetLimitedUIValue(char c, uint32_t minValue, uint32_t maxValuePlusOne) THROWS(GCodeException)
{
	MustSee(c);
	const uint32_t ret = GetUIValue();
	if (ret < minValue)
	{
		throw GCodeException(GetLineNumber(), -1, "parameter '%c' too low", (uint32_t)c);
	}
	if (ret >= maxValuePlusOne)
	{
		throw GCodeException(GetLineNumber(), -1, "parameter '%c' too high", (uint32_t)c);
	}
	return ret;
}

// Get an IP address quad after a key letter
void GCodeBuffer::GetIPAddress(IPAddress& returnedIp) THROWS(GCodeException)
{
	PARSER_OPERATION(GetIPAddress(returnedIp));
}

// Get a MAC address sextet after a key letter
void GCodeBuffer::GetMacAddress(MacAddress& mac) THROWS(GCodeException)
{
	PARSER_OPERATION(GetMacAddress(mac));
}

// Get a string with no preceding key letter
void GCodeBuffer::GetUnprecedentedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
{
	PARSER_OPERATION(GetUnprecedentedString(str, allowEmpty));
}

// Get and copy a quoted string
void GCodeBuffer::GetQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
{
	PARSER_OPERATION(GetQuotedString(str, allowEmpty));
}

// Get and copy a string which may or may not be quoted
void GCodeBuffer::GetPossiblyQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
{
	PARSER_OPERATION(GetPossiblyQuotedString(str, allowEmpty));
}

void GCodeBuffer::GetReducedString(const StringRef& str) THROWS(GCodeException)
{
	// In order to handle string expressions here we first get a quoted string, then we reduce it
	PARSER_OPERATION(GetQuotedString(str, false));
	char *q = str.Pointer();
	const char *p = q;
	while (*p != 0)
	{
		const char c = *p++;
		if (c != '-' && c != '_' && c != ' ')
		{
			*q++ = tolower(c);
		}
	}
	*q = 0;
}

// Get a colon-separated list of floats after a key letter
void GCodeBuffer::GetFloatArray(float arr[], size_t& length, bool doPad) THROWS(GCodeException)
{
	const size_t maxLength = length;
	PARSER_OPERATION(GetFloatArray(arr, length));
	// If there is one entry and doPad is true, fill the rest of the array with the first entry.
	if (doPad && length == 1)
	{
		while (length < maxLength)
		{
			arr[length++] = arr[0];
		}
	}
}

// Get a :-separated list of ints after a key letter
void GCodeBuffer::GetIntArray(int32_t arr[], size_t& length, bool doPad) THROWS(GCodeException)
{
	const size_t maxLength = length;
	PARSER_OPERATION(GetIntArray(arr, length));
	// If there is one entry and doPad is true, fill the rest of the array with the first entry.
	if (doPad && length == 1)
	{
		while (length < maxLength)
		{
			arr[length++] = arr[0];
		}
	}
}

// Get a :-separated list of unsigned ints after a key letter
void GCodeBuffer::GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad) THROWS(GCodeException)
{
	const size_t maxLength = length;
	PARSER_OPERATION(GetUnsignedArray(arr, length));
	// If there is one entry and doPad is true, fill the rest of the array with the first entry.
	if (doPad && length == 1)
	{
		while (length < maxLength)
		{
			arr[length++] = arr[0];
		}
	}
}

// Get a :-separated list of drivers after a key letter
void GCodeBuffer::GetDriverIdArray(DriverId arr[], size_t& length)
{
	PARSER_OPERATION(GetDriverIdArray(arr, length));
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
bool GCodeBuffer::TryGetFValue(char c, float& val, bool& seen) THROWS(GCodeException)
{
	const bool ret = Seen(c);
	if (ret)
	{
		val = GetFValue();
		seen = true;
	}
	return ret;
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
bool GCodeBuffer::TryGetIValue(char c, int32_t& val, bool& seen) THROWS(GCodeException)
{
	const bool ret = Seen(c);
	if (ret)
	{
		val = GetIValue();
		seen = true;
	}
	return ret;
}

// Try to get a signed integer value, throw if outside limits
bool GCodeBuffer::TryGetLimitedIValue(char c, int32_t& val, bool& seen, int32_t minValue, int32_t maxValue) THROWS(GCodeException)
{
	const bool b = TryGetIValue(c, val, seen);
	if (b)
	{
		if (val < minValue)
		{
			throw GCodeException(GetLineNumber(), -1, "parameter '%c' too low", (uint32_t)c);
		}
		if (val > maxValue)
		{
			throw GCodeException(GetLineNumber(), -1, "parameter '%c' too high", (uint32_t)c);
		}
	}
	return b;
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
bool GCodeBuffer::TryGetUIValue(char c, uint32_t& val, bool& seen) THROWS(GCodeException)
{
	const bool ret = Seen(c);
	if (ret)
	{
		val = GetUIValue();
		seen = true;
	}
	return ret;
}

// Try to get an unsigned integer value, throw if >= limit
bool GCodeBuffer::TryGetLimitedUIValue(char c, uint32_t& val, bool& seen, uint32_t maxValuePlusOne) THROWS(GCodeException)
{
	const bool b = TryGetUIValue(c, val, seen);
	if (b && val >= maxValuePlusOne)
	{
		throw GCodeException(GetLineNumber(), -1, "parameter '%c' too high", (uint32_t)c);
	}
	return b;
}

// If the specified parameter character is found, fetch 'value' as a Boolean and set 'seen'. Otherwise leave val and seen alone.
bool GCodeBuffer::TryGetBValue(char c, bool& val, bool& seen) THROWS(GCodeException)
{
	const bool ret = Seen(c);
	if (ret)
	{
		val = GetIValue() > 0;
		seen = true;
	}
	return ret;
}

// Try to get an int array exactly 'numVals' long after parameter letter 'c'.
// If the wrong number of values is provided, generate an error message and return true.
// Else set 'seen' if we saw the letter and value, and return false.
bool GCodeBuffer::TryGetUIArray(char c, size_t numVals, uint32_t vals[], const StringRef& reply, bool& seen, bool doPad) THROWS(GCodeException)
{
	if (Seen(c))
	{
		size_t count = numVals;
		GetUnsignedArray(vals, count, doPad);
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

// Try to get a float array exactly 'numVals' long after parameter letter 'c'.
// If the wrong number of values is provided, generate an error message and return true.
// Else set 'seen' if we saw the letter and value, and return false.
bool GCodeBuffer::TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad) THROWS(GCodeException)
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
bool GCodeBuffer::TryGetQuotedString(char c, const StringRef& str, bool& seen, bool allowEmpty) THROWS(GCodeException)
{
	if (Seen(c))
	{
		seen = true;
		GetQuotedString(str, allowEmpty);
		return true;
	}
	return false;
}

// Try to get a non-empty string, which may be quoted, after parameter letter.
// If we found it then set 'seen' true and return true, else leave 'seen' alone and return false
bool GCodeBuffer::TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen) THROWS(GCodeException)
{
	if (Seen(c))
	{
		seen = true;
		GetPossiblyQuotedString(str);
		return true;
	}
	return false;
}

// Get a PWM frequency
PwmFrequency GCodeBuffer::GetPwmFrequency() THROWS(GCodeException)
{
	return (PwmFrequency)constrain<uint32_t>(GetUIValue(), 1, 65535);
}

// Get a PWM value. If may be in the old style 0..255 in which case convert it to be in the range 0.0..1.0
float GCodeBuffer::GetPwmValue() THROWS(GCodeException)
{
	float v = GetFValue();
	if (v > 1.0)
	{
		v = v/255.0;
	}
	return constrain<float>(v, 0.0, 1.0);
}

// Get a driver ID
DriverId GCodeBuffer::GetDriverId() THROWS(GCodeException)
{
	return PARSER_OPERATION(GetDriverId());
}

bool GCodeBuffer::IsIdle() const noexcept
{
	return bufferState != GCodeBufferState::ready && bufferState != GCodeBufferState::executing;
}

bool GCodeBuffer::IsCompletelyIdle() const noexcept
{
	return GetState() == GCodeState::normal && IsIdle();
}

void GCodeBuffer::SetFinished(bool f) noexcept
{
	if (f)
	{
#if HAS_SBC_INTERFACE
		sendToSbc = false;
#endif
		LatestMachineState().firstCommandAfterRestart = false;
		PARSER_OPERATION(SetFinished());
	}
	else
	{
		bufferState = GCodeBufferState::executing;
	}
}

void GCodeBuffer::SetCommsProperties(uint32_t arg) noexcept
{
	IF_NOT_BINARY(stringParser.SetCommsProperties(arg));
}

// Get the original machine state before we pushed anything
GCodeMachineState& GCodeBuffer::OriginalMachineState() const noexcept
{
	GCodeMachineState *ms = machineState;
	while (ms->GetPrevious() != nullptr)
	{
		ms = ms->GetPrevious();
	}
	return *ms;
}

GCodeMachineState& GCodeBuffer::CurrentFileMachineState() const noexcept
{
	GCodeMachineState *ms = machineState;
	while (ms->localPush && ms->GetPrevious() != nullptr)
	{
		ms = ms->GetPrevious();
	}
	return *ms;
}

// Convert from inches to mm if necessary
float GCodeBuffer::ConvertDistance(float distance) const noexcept
{
	return (UsingInches()) ? distance * InchToMm : distance;
}

// Convert from mm to inches if necessary
float GCodeBuffer::InverseConvertDistance(float distance) const noexcept
{
	return (UsingInches()) ? distance/InchToMm : distance;
}

// Convert speed from mm/min or inches/min to mm per step clock
float GCodeBuffer::ConvertSpeed(float speed) const noexcept
{
	return speed * ((UsingInches()) ? InchToMm/(StepClockRate * iMinutesToSeconds) : 1.0/(StepClockRate * iMinutesToSeconds));
}

// Convert speed to mm/min or inches/min
float GCodeBuffer::InverseConvertSpeed(float speed) const noexcept
{
	return speed * ((UsingInches()) ? (StepClockRate * iMinutesToSeconds)/InchToMm : (float)(StepClockRate * iMinutesToSeconds));
}

const char *GCodeBuffer::GetDistanceUnits() const noexcept
{
	return (UsingInches()) ? "in" : "mm";
}

// Return the  current stack depth
unsigned int GCodeBuffer::GetStackDepth() const noexcept
{
	unsigned int depth = 0;
	for (const GCodeMachineState *m1 = machineState; m1->GetPrevious() != nullptr; m1 = m1->GetPrevious())
	{
		++depth;
	}
	return depth;
}

// Push state returning true if successful (i.e. stack not overflowed)
bool GCodeBuffer::PushState(bool withinSameFile) noexcept
{
	// Check the current stack depth
	if (GetStackDepth() >= MaxStackDepth)
	{
		return false;
	}

	machineState = new GCodeMachineState(*machineState, withinSameFile);
	reprap.InputsUpdated();
	return true;
}

// Pop state returning true if successful (i.e. no stack underrun)
bool GCodeBuffer::PopState() noexcept
{
	GCodeMachineState * const ms = machineState;
	if (ms->GetPrevious() == nullptr)
	{
		ms->messageAcknowledged = false;			// avoid getting stuck in a loop trying to pop
		ms->waitingForAcknowledgement = false;
		return false;
	}

	machineState = ms->Pop();						// get the previous state and copy down any error message
	delete ms;

	reprap.InputsUpdated();
	return true;
}

// Abort execution of any files or macros being executed
// We now avoid popping the state if we were not executing from a file, so that if DWC or PanelDue is used to jog the axes before they are homed, we don't report stack underflow.
void GCodeBuffer::AbortFile(bool abortAll, bool requestAbort) noexcept
{
	if (machineState->DoingFile())
	{
		do
		{
			if (machineState->DoingFile())
			{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
# if HAS_SBC_INTERFACE
				if (!reprap.UsingSbcInterface())
# endif
				{
					fileInput->Reset(machineState->fileState);
				}
#endif
				machineState->CloseFile();
			}
		} while (PopState() && (abortAll || !machineState->DoingFile()));

#if HAS_SBC_INTERFACE
		abortFile = requestAbort;
		abortAllFiles = requestAbort && abortAll;
	}
	else if (!requestAbort)
	{
		abortFile = abortAllFiles = false;
#endif
	}
}

#if HAS_SBC_INTERFACE

void GCodeBuffer::SetFileFinished() noexcept
{
	FileId macroFileId = NoFileId, printFileId = OriginalMachineState().fileId;
	for (GCodeMachineState *ms = machineState; ms != nullptr; ms = ms->GetPrevious())
	{
		if (macroFileId == NoFileId && ms->fileId != NoFileId && ms->fileId != printFileId && !ms->fileFinished)
		{
			// Get the next macro file being executed
			macroFileId = ms->fileId;
		}

		if (macroFileId != NoFileId)
		{
			if (ms->fileId == macroFileId)
			{
				// Flag it (and following machine states) as finished
				ms->fileFinished = true;
			}
			else
			{
				break;
			}
		}
	}

	if (macroFileId != NoFileId)
	{
		reprap.GetSbcInterface().EventOccurred();
	}
}

void GCodeBuffer::SetPrintFinished() noexcept
{
	FileId printFileId = OriginalMachineState().fileId;
	if (printFileId != NoFileId)
	{
		for (GCodeMachineState *ms = machineState; ms != nullptr; ms = ms->GetPrevious())
		{
			if (ms->fileId == printFileId)
			{
				// Mark machine states executing the print file as finished
				ms->fileFinished = true;
			}
		}
		reprap.GetSbcInterface().EventOccurred();
	}
}

void GCodeBuffer::ClosePrintFile() noexcept
{
	FileId printFileId = OriginalMachineState().fileId;
	if (printFileId != NoFileId)
	{
		for (GCodeMachineState *ms = machineState; ms != nullptr; ms = ms->GetPrevious())
		{
			if (ms->fileId == printFileId)
			{
				ms->fileId = NoFileId;
			}
		}
	}
}

// This is only called when using the SBC interface and returns if the macro file could be opened
bool GCodeBuffer::RequestMacroFile(const char *filename, bool fromCode) noexcept
{
	if (!reprap.GetSbcInterface().IsConnected())
	{
		// Don't wait for a macro file if no SBC is connected
		return false;
	}

	// Request the macro file from the SBC
	macroJustStarted = macroFileError = macroFileEmpty = false;
	machineState->macroStartedByCode = fromCode;
	requestedMacroFile.copy(filename);

	// There is no need to block the main task if daemon.g is requested.
	// If it doesn't exist, DSF will simply close the virtual file again
	if (GetChannel() != GCodeChannel::Daemon || machineState->doingFileMacro)
	{
		// Wait for a response (but not forever)
		isWaitingForMacro = true;
		reprap.GetSbcInterface().EventOccurred(true);
		if (!macroSemaphore.Take(SpiMaxRequestTime))
		{
			isWaitingForMacro = false;
			reprap.GetPlatform().MessageF(ErrorMessage, "Timeout while waiting for macro file %s (channel %s)\n", filename, GetChannel().ToString());
			return false;
		}
	}

	// When we get here we expect the SBC interface to have set the variables above for us
	if (!macroFileError)
	{
		macroJustStarted = true;
		return true;
	}
	return false;
}

void GCodeBuffer::ResolveMacroRequest(bool hadError, bool isEmpty) noexcept
{
	macroFileError = hadError;
	macroFileEmpty = !hadError && isEmpty;
	isWaitingForMacro = false;
	macroSemaphore.Give();
}

void GCodeBuffer::MacroFileClosed() noexcept
{
	machineState->CloseFile();
	macroJustStarted = false;
	macroFileClosed = true;
	reprap.GetSbcInterface().EventOccurred();
}

#endif

// Tell this input source that any message it sent and is waiting on has been acknowledged
// Allow for the possibility that the source may have started running a macro since it started waiting
void GCodeBuffer::MessageAcknowledged(bool cancelled) noexcept
{
	for (GCodeMachineState *ms = machineState; ms != nullptr; ms = ms->GetPrevious())
	{
		if (ms->waitingForAcknowledgement)
		{
			ms->waitingForAcknowledgement = false;
			ms->messageAcknowledged = true;
			ms->messageCancelled = cancelled;
#if HAS_SBC_INTERFACE
			messageAcknowledged = !cancelled || !ms->DoingFile();
			reprap.GetSbcInterface().EventOccurred();
#endif
		}
	}
}

MessageType GCodeBuffer::GetResponseMessageType() const noexcept
{
#if HAS_SBC_INTERFACE
	if (machineState->lastCodeFromSbc)
	{
		return (MessageType)((1u << codeChannel.ToBaseType()) | BinaryCodeReplyFlag);
	}
#endif
	return responseMessageType;
}

FilePosition GCodeBuffer::GetFilePosition() const noexcept
{
	return PARSER_OPERATION(GetFilePosition());
}

void GCodeBuffer::WaitForAcknowledgement() noexcept
{
	machineState->WaitForAcknowledgement();
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		messagePromptPending = true;
	}
#endif
}

#if HAS_MASS_STORAGE

bool GCodeBuffer::OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) noexcept
{
	return NOT_BINARY_AND(stringParser.OpenFileToWrite(directory, fileName, size, binaryWrite, fileCRC32));
}

bool GCodeBuffer::IsWritingFile() const noexcept
{
	return NOT_BINARY_AND(stringParser.IsWritingFile());
}

void GCodeBuffer::WriteToFile() noexcept
{
	IF_NOT_BINARY(stringParser.WriteToFile());
}

bool GCodeBuffer::IsWritingBinary() const noexcept
{
	return NOT_BINARY_AND(stringParser.IsWritingBinary());
}

bool GCodeBuffer::WriteBinaryToFile(char b) noexcept
{
	return IS_BINARY_OR(stringParser.WriteBinaryToFile(b));
}

void GCodeBuffer::FinishWritingBinary() noexcept
{
	IF_NOT_BINARY(stringParser.FinishWritingBinary());
}

#endif

void GCodeBuffer::RestartFrom(FilePosition pos) noexcept
{
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	fileInput->Reset(machineState->fileState);		// clear the buffered data
	machineState->fileState.Seek(pos);				// replay the abandoned instructions when we resume
#endif
	Init();											// clear the next move
}

const char* GCodeBuffer::DataStart() const noexcept
{
	return PARSER_OPERATION(DataStart());
}

// Return the length of the command.
// WARNING! This may return the wrong value if the command has an unquoted string parameter and GetUnprecedentedString or GetPossiblyQuotedString hasn't been called yet.
size_t GCodeBuffer::DataLength() const noexcept
{
	return PARSER_OPERATION(DataLength());
}

void GCodeBuffer::PrintCommand(const StringRef& s) const noexcept
{
	PARSER_OPERATION(PrintCommand(s));
}

void GCodeBuffer::AppendFullCommand(const StringRef &s) const noexcept
{
	PARSER_OPERATION(AppendFullCommand(s));
}

void GCodeBuffer::AddParameters(VariableSet& vars, int codeRunning) noexcept
{
	PARSER_OPERATION(AddParameters(vars, codeRunning));
}

VariableSet& GCodeBuffer::GetVariables() const noexcept
{
	GCodeMachineState *mc = machineState;
	while (mc->localPush && mc->GetPrevious() != nullptr)
	{
		mc = mc->GetPrevious();
	}
	return mc->variables;
}

#if SUPPORT_COORDINATE_ROTATION

bool GCodeBuffer::DoingCoordinateRotation() const noexcept
{
	return !LatestMachineState().g53Active && !LatestMachineState().runningSystemMacro && LatestMachineState().selectedPlane == 0;
}

#endif

// End
