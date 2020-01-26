/*
 * GCodeBuffer.cpp
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

//*************************************************************************************

#include "GCodeBuffer.h"
#if HAS_MASS_STORAGE
# include <GCodes/GCodeInput.h>
#endif
#include "BinaryParser.h"
#include "StringParser.h"
#include <GCodes/GCodeException.h>
#include <RepRap.h>
#include <Platform.h>

// Create a default GCodeBuffer
GCodeBuffer::GCodeBuffer(GCodeChannel channel, GCodeInput *normalIn, FileGCodeInput *fileIn, MessageType mt, Compatibility c) noexcept
	: codeChannel(channel), normalInput(normalIn),
#if HAS_MASS_STORAGE
	  fileInput(fileIn),
#endif
	  responseMessageType(mt), toolNumberAdjust(0), lastResult(GCodeResult::ok), isBinaryBuffer(false), binaryParser(*this), stringParser(*this),
	  machineState(new GCodeMachineState())
{
	machineState->compatibility = c;
	Reset();
}

// Reset it to its state after start-up
void GCodeBuffer::Reset() noexcept
{
	while (PopState(false)) { }
#if HAS_LINUX_INTERFACE
	requestedMacroFile.Clear();
	reportMissingMacro = isMacroFromCode = abortFile = abortAllFiles = reportStack = false;
#endif
	isBinaryBuffer = false;
	Init();
}

// Set it up to parse another G-code
void GCodeBuffer::Init() noexcept
{
	binaryParser.Init();
	stringParser.Init();
	timerRunning = false;
}

void GCodeBuffer::StartTimer() noexcept
{
	whenTimerStarted = millis();
	timerRunning = true;
}

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

// Write some debug info
void GCodeBuffer::Diagnostics(MessageType mtype) noexcept
{
	String<StringLength256> scratchString;
	scratchString.copy(GetIdentity());
	scratchString.cat(IsBinary() ? "* " : " ");
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
		scratchString.catf(" %d", (int)ms->state);
		ms = ms->previous;
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
	isBinaryBuffer = false;
	return stringParser.Put(c);
}

// Decode the command in the buffer when it is complete
void GCodeBuffer::DecodeCommand() noexcept
{
	if (!isBinaryBuffer)
	{
		stringParser.DecodeCommand();
	}
}

// Check whether the current command is a meta command, or we are skipping a block. Return true if we are and the current line no longer needs to be processed.
bool GCodeBuffer::CheckMetaCommand(const StringRef& reply)
{
	return !isBinaryBuffer && stringParser.CheckMetaCommand(reply);
}

// Add an entire G-Code, overwriting any existing content
void GCodeBuffer::PutAndDecode(const char *str, size_t len, bool isBinary) noexcept
{
	isBinaryBuffer = isBinary;
	if (isBinary)
	{
		binaryParser.Put(str, len);
	}
	else
	{
		stringParser.PutAndDecode(str, len);
	}
}

// Add a null-terminated string, overwriting any existing content
void GCodeBuffer::PutAndDecode(const char *str) noexcept
{
	isBinaryBuffer = false;
	stringParser.PutAndDecode(str);
}

void GCodeBuffer::StartNewFile() noexcept
{
	if (!isBinaryBuffer)
	{
		stringParser.StartNewFile();
	}
}

// Called when we reach the end of the file we are reading from. Return true if there is a line waiting to be processed.
bool GCodeBuffer::FileEnded() noexcept
{
	return (!isBinaryBuffer) && stringParser.FileEnded();
}

char GCodeBuffer::GetCommandLetter() const noexcept
{
	return isBinaryBuffer ? binaryParser.GetCommandLetter() : stringParser.GetCommandLetter();
}

bool GCodeBuffer::HasCommandNumber() const noexcept
{
	return isBinaryBuffer ? binaryParser.HasCommandNumber() : stringParser.HasCommandNumber();
}

int GCodeBuffer::GetCommandNumber() const noexcept
{
	return isBinaryBuffer ? binaryParser.GetCommandNumber() : stringParser.GetCommandNumber();
}

int8_t GCodeBuffer::GetCommandFraction() const noexcept
{
	return isBinaryBuffer ? binaryParser.GetCommandFraction() : stringParser.GetCommandFraction();
}

// Is a character present?
bool GCodeBuffer::Seen(char c) noexcept
{
	return isBinaryBuffer ? binaryParser.Seen(c) : stringParser.Seen(c);
}

// Test for character present, throw error if not
void GCodeBuffer::MustSee(char c)
{
	if (!Seen(c))
	{
		throw GCodeException(machineState->lineNumber, -1, "missing parameter '%c'", (uint32_t)c);
	}
}

// Get a float after a key letter
float GCodeBuffer::GetFValue()
{
	return isBinaryBuffer ? binaryParser.GetFValue() : stringParser.GetFValue();
}

// Get a distance or coordinate and convert it from inches to mm if necessary
float GCodeBuffer::GetDistance()
{
	return ConvertDistance(GetFValue());
}

// Get an integer after a key letter
int32_t GCodeBuffer::GetIValue()
{
	return isBinaryBuffer ? binaryParser.GetIValue() : stringParser.GetIValue();
}

// Get an unsigned integer value
uint32_t GCodeBuffer::GetUIValue()
{
	return isBinaryBuffer ? binaryParser.GetUIValue() : stringParser.GetUIValue();
}

// Get an IP address quad after a key letter
void GCodeBuffer::GetIPAddress(IPAddress& returnedIp)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetIPAddress(returnedIp);
	}
	else
	{
		stringParser.GetIPAddress(returnedIp);
	}
}

// Get a MAC address sextet after a key letter
void GCodeBuffer::GetMacAddress(MacAddress& mac)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetMacAddress(mac);
	}
	else
	{
		stringParser.GetMacAddress(mac);
	}
}

// Get a string with no preceding key letter
void GCodeBuffer::GetUnprecedentedString(const StringRef& str, bool allowEmpty)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetUnprecedentedString(str, allowEmpty);
	}
	else
	{
		stringParser.GetUnprecedentedString(str, allowEmpty);
	}
}

// Get and copy a quoted string
void GCodeBuffer::GetQuotedString(const StringRef& str)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetQuotedString(str);
	}
	else
	{
		stringParser.GetQuotedString(str);
	}
}

// Get and copy a string which may or may not be quoted
void GCodeBuffer::GetPossiblyQuotedString(const StringRef& str)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetPossiblyQuotedString(str);
	}
	else
	{
		stringParser.GetPossiblyQuotedString(str);
	}
}

void GCodeBuffer::GetReducedString(const StringRef& str)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetReducedString(str);
	}
	else
	{
		stringParser.GetReducedString(str);
	}
}

// Get a colon-separated list of floats after a key letter
void GCodeBuffer::GetFloatArray(float arr[], size_t& length, bool doPad)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetFloatArray(arr, length, doPad);
	}
	else
	{
		stringParser.GetFloatArray(arr, length, doPad);
	}
}

// Get a :-separated list of ints after a key letter
void GCodeBuffer::GetIntArray(int32_t arr[], size_t& length, bool doPad)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetIntArray(arr, length, doPad);
	}
	else
	{
		stringParser.GetIntArray(arr, length, doPad);
	}
}

// Get a :-separated list of unsigned ints after a key letter
void GCodeBuffer::GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetUnsignedArray(arr, length, doPad);
	}
	else
	{
		stringParser.GetUnsignedArray(arr, length, doPad);
	}
}

// Get a :-separated list of drivers after a key letter
void GCodeBuffer::GetDriverIdArray(DriverId arr[], size_t& length)
{
	if (isBinaryBuffer)
	{
		binaryParser.GetDriverIdArray(arr, length);
	}
	else
	{
		stringParser.GetDriverIdArray(arr, length);
	}
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
bool GCodeBuffer::TryGetFValue(char c, float& val, bool& seen)
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
bool GCodeBuffer::TryGetIValue(char c, int32_t& val, bool& seen)
{
	const bool ret = Seen(c);
	if (ret)
	{
		val = GetIValue();
		seen = true;
	}
	return ret;
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
bool GCodeBuffer::TryGetUIValue(char c, uint32_t& val, bool& seen)
{
	const bool ret = Seen(c);
	if (ret)
	{
		val = GetUIValue();
		seen = true;
	}
	return ret;
}

// If the specified parameter character is found, fetch 'value' as a Boolean and set 'seen'. Otherwise leave val and seen alone.
bool GCodeBuffer::TryGetBValue(char c, bool& val, bool& seen)
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
bool GCodeBuffer::TryGetUIArray(char c, size_t numVals, uint32_t vals[], const StringRef& reply, bool& seen, bool doPad)
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
bool GCodeBuffer::TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad)
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
bool GCodeBuffer::TryGetQuotedString(char c, const StringRef& str, bool& seen)
{
	if (Seen(c))
	{
		seen = true;
		GetQuotedString(str);
		return true;
	}
	return false;
}

// Try to get a string, which may be quoted, after parameter letter.
// If we found it then set 'seen' true and return true, else leave 'seen' alone and return false
bool GCodeBuffer::TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen)
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
PwmFrequency GCodeBuffer::GetPwmFrequency()
{
	return (PwmFrequency)constrain<uint32_t>(GetUIValue(), 1, 65535);
}

// Get a PWM value. If may be in the old style 0..255 in which case convert it to be in the range 0.0..1.0
float GCodeBuffer::GetPwmValue()
{
	float v = GetFValue();
	if (v > 1.0)
	{
		v = v/255.0;
	}
	return constrain<float>(v, 0.0, 1.0);
}

// Get a driver ID
DriverId GCodeBuffer::GetDriverId()
{
	return isBinaryBuffer ? binaryParser.GetDriverId() : stringParser.GetDriverId();
}

bool GCodeBuffer::IsIdle() const noexcept
{
	return bufferState != GCodeBufferState::ready && bufferState != GCodeBufferState::executing;
}

bool GCodeBuffer::IsCompletelyIdle() const noexcept
{
	return GetState() == GCodeState::normal && IsIdle();
}

bool GCodeBuffer::IsReady() const noexcept
{
	return bufferState == GCodeBufferState::ready;
}

bool GCodeBuffer::IsExecuting() const noexcept
{
	return bufferState == GCodeBufferState::executing;
}

void GCodeBuffer::SetFinished(bool f) noexcept
{
	if (f)
	{
		if (isBinaryBuffer)
		{
			binaryParser.SetFinished();
		}
		else
		{
			stringParser.SetFinished();
		}
	}
	else
	{
		bufferState = GCodeBufferState::executing;
	}
}

void GCodeBuffer::SetCommsProperties(uint32_t arg) noexcept
{
	if (!isBinaryBuffer)
	{
		stringParser.SetCommsProperties(arg);
	}
}

// Get the original machine state before we pushed anything
GCodeMachineState& GCodeBuffer::OriginalMachineState() const noexcept
{
	GCodeMachineState *ms = machineState;
	while (ms->previous != nullptr)
	{
		ms = ms->previous;
	}
	return *ms;
}

// Convert from inches to mm if necessary
float GCodeBuffer::ConvertDistance(float distance) const noexcept
{
	return (machineState->usingInches) ? distance * InchToMm : distance;
}

// Convert from mm to inches if necessary
float GCodeBuffer::InverseConvertDistance(float distance) const noexcept
{
	return (machineState->usingInches) ? distance/InchToMm : distance;
}

// Push state returning true if successful (i.e. stack not overflowed)
bool GCodeBuffer::PushState(bool withinSameFile) noexcept
{
	// Check the current stack depth
	unsigned int depth = 0;
	for (const GCodeMachineState *m1 = machineState; m1 != nullptr; m1 = m1->previous)
	{
		++depth;
	}
	if (depth >= MaxStackDepth + 1)					// the +1 is to allow for the initial state record
	{
		return false;
	}

	machineState = new GCodeMachineState(*machineState, withinSameFile);
	return true;
}

// Pop state returning true if successful (i.e. no stack underrun)
bool GCodeBuffer::PopState(bool withinSameFile) noexcept
{
	GCodeMachineState * const ms = machineState;
	if (ms->previous == nullptr)
	{
		ms->messageAcknowledged = false;			// avoid getting stuck in a loop trying to pop
		ms->waitingForAcknowledgement = false;
		return false;
	}

	machineState = ms->previous;
	if (withinSameFile)
	{
		machineState->lineNumber = ms->lineNumber;
	}
	delete ms;

#if HAS_LINUX_INTERFACE
	reportStack = true;
#endif
	return true;
}


// Abort execution of any files or macros being executed, returning true if any files were closed
// We now avoid popping the state if we were not executing from a file, so that if DWC or PanelDue is used to jog the axes before they are homed, we don't report stack underflow.
void GCodeBuffer::AbortFile(bool abortAll, bool requestAbort) noexcept
{
	if (machineState->DoingFile())
	{
		do
		{
			if (machineState->DoingFile())
			{
#if HAS_MASS_STORAGE
				fileInput->Reset(machineState->fileState);
#endif
				machineState->CloseFile();
			}
		} while (PopState(false) && (!machineState->DoingFile() || abortAll));
	}

#if HAS_LINUX_INTERFACE
	abortFile = requestAbort;
	abortAllFiles = requestAbort && abortAll;
#endif
}

#if HAS_LINUX_INTERFACE

bool GCodeBuffer::IsFileFinished() const noexcept
{
	return machineState->isFileFinished;
}

void GCodeBuffer::SetPrintFinished() noexcept
{
	const uint32_t fileId = OriginalMachineState().fileId;
	for (GCodeMachineState *ms = machineState; ms != nullptr; ms = ms->previous)
	{
		if (ms->fileId == fileId)
		{
			ms->SetFileFinished(false);
		}
	}
}

// This is only called when using the Linux interface. 'filename' is sometimes null.
void GCodeBuffer::RequestMacroFile(const char *filename, bool reportMissing, bool fromCode) noexcept
{
	machineState->SetFileExecuting();
	if (filename == nullptr)
	{
		requestedMacroFile.Clear();
	}
	else
	{
		requestedMacroFile.copy(filename);
	}
	reportMissingMacro = reportMissing;
	isMacroFromCode = fromCode;
	abortFile = abortAllFiles = false;
	isBinaryBuffer = true;
}

const char *GCodeBuffer::GetRequestedMacroFile(bool& reportMissing, bool& fromCode) const noexcept
{
	reportMissing = reportMissingMacro;
	fromCode = isMacroFromCode;
	return requestedMacroFile.IsEmpty() ? nullptr : requestedMacroFile.c_str();
}

#endif

// Tell this input source that any message it sent and is waiting on has been acknowledged
// Allow for the possibility that the source may have started running a macro since it started waiting
void GCodeBuffer::MessageAcknowledged(bool cancelled) noexcept
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

MessageType GCodeBuffer::GetResponseMessageType() const noexcept
{
	if (isBinaryBuffer)
	{
		return (MessageType)((1 << (size_t)codeChannel) | BinaryCodeReplyFlag);
	}
	return responseMessageType;
}

FilePosition GCodeBuffer::GetFilePosition() const noexcept
{
	return isBinaryBuffer ? binaryParser.GetFilePosition() : stringParser.GetFilePosition();
}

#if HAS_MASS_STORAGE

bool GCodeBuffer::OpenFileToWrite(const char* directory, const char* fileName, const FilePosition size, const bool binaryWrite, const uint32_t fileCRC32) noexcept
{
	if (isBinaryBuffer)
	{
		return false;
	}
	return stringParser.OpenFileToWrite(directory, fileName, size, binaryWrite, fileCRC32);
}

bool GCodeBuffer::IsWritingFile() const noexcept
{
	return !isBinaryBuffer && stringParser.IsWritingFile();
}

void GCodeBuffer::WriteToFile() noexcept
{
	if (!isBinaryBuffer)
	{
		stringParser.WriteToFile();
	}
}

bool GCodeBuffer::IsWritingBinary() const noexcept
{
	return !isBinaryBuffer && stringParser.IsWritingBinary();
}

void GCodeBuffer::WriteBinaryToFile(char b) noexcept
{
	if (!isBinaryBuffer)
	{
		stringParser.WriteBinaryToFile(b);
	}
}

void GCodeBuffer::FinishWritingBinary() noexcept
{
	if (!isBinaryBuffer)
	{
		stringParser.FinishWritingBinary();
	}
}

#endif

void GCodeBuffer::RestartFrom(FilePosition pos) noexcept
{
#if HAS_MASS_STORAGE
	fileInput->Reset(machineState->fileState);		// clear the buffered data
	machineState->fileState.Seek(pos);				// replay the abandoned instructions when we resume
#endif
	Init();											// clear the next move
}

const char* GCodeBuffer::DataStart() const noexcept
{
	return isBinaryBuffer ? binaryParser.DataStart() : stringParser.DataStart();
}

size_t GCodeBuffer::DataLength() const noexcept
{
	return isBinaryBuffer ? binaryParser.DataLength() : stringParser.DataLength();
}

void GCodeBuffer::PrintCommand(const StringRef& s) const noexcept
{
	if (isBinaryBuffer)
	{
		binaryParser.PrintCommand(s);
	}
	else
	{
		stringParser.PrintCommand(s);
	}
}

void GCodeBuffer::AppendFullCommand(const StringRef &s) const noexcept
{
	if (isBinaryBuffer)
	{
		binaryParser.AppendFullCommand(s);
	}
	else
	{
		stringParser.AppendFullCommand(s);
	}
}

// End
