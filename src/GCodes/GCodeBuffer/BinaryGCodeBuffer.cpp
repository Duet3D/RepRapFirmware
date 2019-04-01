/*
 * BinaryGCodeBuffer.cpp
 *
 *  Created on: 30 Mar 2019
 *      Author: Christian
 */

#include "BinaryGCodeBuffer.h"
#include "Platform.h"
#include "RepRap.h"

BinaryGCodeBuffer::BinaryGCodeBuffer(CodeChannel c, MessageType mt, bool usesCodeQueue)
	: GCodeBuffer(mt, usesCodeQueue), channel(c)
{
	// Init is called by the base class
}

void BinaryGCodeBuffer::Init()
{
	bufferLength = 0;
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	isIdle = isFileFinished = true;
}

void BinaryGCodeBuffer::Diagnostics(MessageType mtype)
{
	String<ScratchStringLength> scratchString;
	if (IsIdle())
	{
		scratchString.printf("%s is idle", GetIdentity());
	}
	else if (IsExecuting())
	{
		scratchString.printf("%s is doing \"", GetIdentity());
		AppendFullCommand(scratchString.GetRef());
		scratchString.cat('"');
	}
	else
	{
		scratchString.printf("%s is ready with \"", GetIdentity());
		AppendFullCommand(scratchString.GetRef());
		scratchString.cat('"');
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

bool BinaryGCodeBuffer::Put(const char *data, size_t len)
{
	memcpy(buffer, data, len);
	bufferLength = len;
	return true;
}

bool BinaryGCodeBuffer::Seen(char c)
{
	if (bufferLength != 0)
	{
		const char *parameterStart = reinterpret_cast<const char*>(buffer) + sizeof(CodeHeader);
		seenParameter = nullptr;
		seenParameterValue = parameterStart + header->numParameters * sizeof(CodeParameter);

		for (size_t i = 0; i < header->numParameters; i++)
		{
			const CodeParameter *param = reinterpret_cast<const CodeParameter*>(parameterStart + i * sizeof(CodeParameter));
			if (param->letter == c)
			{
				seenParameter = param;
				return true;
			}

			if (param->type == DataType::IntArray ||
				param->type == DataType::UIntArray ||
				param->type == DataType::FloatArray)
			{
				seenParameterValue += param->intValue * sizeof(uint32_t);
			}
			else if (param->type == DataType::String ||
					 param->type == DataType::Expression)
			{
				seenParameterValue += AddPadding(param->intValue);
			}
		}
	}
	return false;
}

char BinaryGCodeBuffer::GetCommandLetter() const
{
	return (bufferLength != 0) ? header->letter : 'Q';
}

bool BinaryGCodeBuffer::HasCommandNumber() const
{
	return (bufferLength != 0 && (header->flags & CodeFlags::NoMajorCommandNumber) == 0);
}

int BinaryGCodeBuffer::GetCommandNumber() const
{
	return HasCommandNumber() ? header->majorCode : -1;
}

int8_t BinaryGCodeBuffer::GetCommandFraction() const
{
	return (bufferLength != 0 && (header->flags & CodeFlags::NoMinorCommandNumber) == 0) ? header->minorCode : -1;
}

float BinaryGCodeBuffer::GetFValue()
{
	if (seenParameter != nullptr)
	{
		float value = seenParameter->floatValue;
		seenParameter = nullptr;
		seenParameterValue = nullptr;
		return value;
	}

	INTERNAL_ERROR;
	return 0.0f;
}

int32_t BinaryGCodeBuffer::GetIValue()
{
	if (seenParameter != nullptr)
	{
		int32_t value = seenParameter->intValue;
		seenParameter = nullptr;
		seenParameterValue = nullptr;
		return value;
	}

	INTERNAL_ERROR;
	return 0;
}

uint32_t BinaryGCodeBuffer::GetUIValue()
{
	if (seenParameter != nullptr)
	{
		uint32_t value = seenParameter->uintValue;
		seenParameter = nullptr;
		seenParameterValue = nullptr;
		return value;
	}

	INTERNAL_ERROR;
	return 0;
}

bool BinaryGCodeBuffer::GetIPAddress(IPAddress& returnedIp)
{
	if (seenParameter == nullptr)
	{
		INTERNAL_ERROR;
		return false;
	}

	if (seenParameter->type != DataType::String)
	{
		seenParameter = nullptr;
		seenParameterValue = nullptr;
		return false;
	}

	const char* p = seenParameterValue;
	uint8_t ip[4];
	unsigned int n = 0;
	for (;;)
	{
		const char *pp;
		const unsigned long v = SafeStrtoul(p, &pp);
		if (pp == p || pp > seenParameterValue + seenParameter->intValue || v > 255)
		{
			seenParameter = nullptr;
			seenParameterValue = nullptr;
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
			seenParameter = nullptr;
			seenParameterValue = nullptr;
			return false;
		}
		++p;
	}
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	if (n == 4)
	{
		returnedIp.SetV4(ip);
		return true;
	}
	returnedIp.SetNull();
	return false;
}

bool BinaryGCodeBuffer::GetMacAddress(uint8_t mac[6])
{
	if (seenParameter == nullptr)
	{
		INTERNAL_ERROR;
		return false;
	}

	if (seenParameter->type != DataType::String)
	{
		seenParameter = nullptr;
		seenParameterValue = nullptr;
		return false;
	}

	const char* p = seenParameterValue;
	unsigned int n = 0;
	for (;;)
	{
		const char *pp;
		const unsigned long v = SafeStrtoul(p, &pp, 16);
		if (pp == p || pp > seenParameterValue + seenParameter->intValue || v > 255)
		{
			seenParameter = nullptr;
			seenParameterValue = nullptr;
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
			seenParameter = nullptr;
			seenParameterValue = nullptr;
			return false;
		}
		++p;
	}
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	return n == 6;
}

bool BinaryGCodeBuffer::GetUnprecedentedString(const StringRef& str)
{
	str.Clear();
	WriteParameters(str, false);
	return !str.IsEmpty();
}

bool BinaryGCodeBuffer::GetQuotedString(const StringRef& str)
{
	return GetPossiblyQuotedString(str);
}

bool BinaryGCodeBuffer::GetPossiblyQuotedString(const StringRef& str)
{
	if (seenParameter != nullptr && (seenParameter->type == DataType::String || seenParameter->type == DataType::Expression))
	{
		str.copy(seenParameterValue, seenParameter->intValue);
	}
	else
	{
		str.Clear();
	}
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	return !str.IsEmpty();
}


const void BinaryGCodeBuffer::GetFloatArray(float arr[], size_t& length, bool doPad)
{
	GetArray(arr, length, doPad);
}

const void BinaryGCodeBuffer::GetIntArray(int32_t arr[], size_t& length, bool doPad)
{
	GetArray(arr, length, doPad);
}

const void BinaryGCodeBuffer::GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad)
{
	GetArray(arr, length, doPad);
}

void BinaryGCodeBuffer::SetFinished(bool f)
{
	isIdle = f;
	if (f)
	{
		machineState->g53Active = false;		// G53 does not persist beyond the current line
		Init();
	}
}

const char *BinaryGCodeBuffer::GetIdentity() const
{
	// I would have put this into MessageFormats.cpp but then G++ complains about an unresolved reference
	const char * const codeChannelName[] =
	{
		"file",
		"http",
		"telnet",
		"spi"
	};
	static_assert(ARRAY_SIZE(codeChannelName) == (size_t)CodeChannel::SPI + 1, "ID relations do not match");

	return codeChannelName[(uint8_t)channel];
}

FilePosition BinaryGCodeBuffer::GetFilePosition(size_t bytesCached) const
{
	return (bufferLength != 0) ? header->filePosition : noFilePosition;
}

void BinaryGCodeBuffer::PrintCommand(const StringRef& s) const
{
	if (bufferLength != 0 && (header->flags & CodeFlags::NoMajorCommandNumber) == 0)
	{
		s.printf("%c%" PRId32, header->letter, header->majorCode);
		if ((header->flags & CodeFlags::NoMinorCommandNumber) == 0)
		{
			s.catf("%" PRId32, header->minorCode);
		}
	}
	else
	{
		s.Clear();
	}
}

void BinaryGCodeBuffer::AppendFullCommand(const StringRef &s) const
{
	if (bufferLength != 0)
	{
		if ((header->flags & CodeFlags::NoMajorCommandNumber) == 0)
		{
			s.catf("%c%" PRId32, header->letter, header->majorCode);
			if ((header->flags & CodeFlags::NoMinorCommandNumber) == 0)
			{
				s.catf("%" PRId32, header->minorCode);
			}
		}

		if (header->numParameters != 0)
		{
			s.cat(' ');
		}
		WriteParameters(s, true);
	}
}

size_t BinaryGCodeBuffer::AddPadding(size_t bytesRead) const
{
    size_t padding = 4 - bytesRead % 4;
    return (padding != 4) ? bytesRead + padding : bytesRead;
}

template<typename T> const void BinaryGCodeBuffer::GetArray(T arr[], size_t& length, bool doPad)
{
	if (seenParameter == nullptr)
	{
		INTERNAL_ERROR;
		return;
	}

	int lastIndex = -1;
	switch (seenParameter->type)
	{
	case DataType::Int:
		arr[0] = seenParameter->intValue;
		lastIndex = 0;
		break;
	case DataType::UInt:
		arr[0] = seenParameter->uintValue;
		lastIndex = 0;
		break;
	case DataType::Float:
		arr[0] = seenParameter->floatValue;
		lastIndex = 0;
		break;
	case DataType::IntArray:
		for (int i = 0; i < seenParameter->intValue; i++)
		{
			arr[i] = reinterpret_cast<const int32_t*>(seenParameterValue)[i];
		}
		lastIndex = seenParameter->intValue - 1;
		break;
	case DataType::UIntArray:
		for (int i = 0; i < seenParameter->intValue; i++)
		{
			arr[i] = reinterpret_cast<const uint32_t*>(seenParameterValue)[i];
		}
		lastIndex = seenParameter->intValue - 1;
		break;
	case DataType::FloatArray:
		for (int i = 0; i < seenParameter->intValue; i++)
		{
			arr[i] = reinterpret_cast<const float*>(seenParameterValue)[i];
		}
		lastIndex = seenParameter->intValue - 1;
		break;
	case DataType::String:
	case DataType::Expression:
		length = 0;
		return;
	}

	if (doPad && lastIndex >= 0)
	{
		for (size_t i = 1; i < length; i++)
		{
			arr[i] = arr[lastIndex];
		}
	}
}

void BinaryGCodeBuffer::WriteParameters(const StringRef& s, bool quoteStrings) const
{
	if (bufferLength != 0)
	{
		const char *parameterStart = reinterpret_cast<const char *>(buffer) + sizeof(CodeHeader);
		const char *val = parameterStart + header->numParameters * sizeof(CodeParameter);
		for (int i = 0; i < header->numParameters; i++)
		{
			if (i != 0)
			{
				s.cat(' ');
			}

			const CodeParameter *param = reinterpret_cast<const CodeParameter*>(parameterStart + i * sizeof(CodeParameter));
			switch (param->type)
			{
			case DataType::Int:
				s.catf("%c%" PRId32, param->letter, param->intValue);
				break;
			case DataType::UInt:
				s.catf("%c%" PRIu32, param->letter, param->uintValue);
				break;
			case DataType::Float:
				s.catf("%c%f", param->letter, (double)param->floatValue);
				break;
			case DataType::IntArray:
				s.cat(param->letter);
				for (int k = 0; k < param->intValue; k++)
				{
					if (k != 0)
					{
						s.cat(':');
					}
					s.catf("%" PRIu32, *reinterpret_cast<const int32_t*>(val));
					val += sizeof(int32_t);
				}
				break;
			case DataType::UIntArray:
				s.cat(param->letter);
				for (int k = 0; k < param->intValue; k++)
				{
					if (k != 0)
					{
						s.cat(':');
					}
					s.catf("%lu", *reinterpret_cast<const uint32_t*>(val));
					val += sizeof(uint32_t);
				}
				break;
			case DataType::FloatArray:
				s.cat(param->letter);
				for (int k = 0; k < param->intValue; k++)
				{
					if (k != 0)
					{
						s.cat(':');
					}
					s.catf("%f", (double)*reinterpret_cast<const float*>(val));
					val += sizeof(float);
				}
				break;
			case DataType::String:
			case DataType::Expression:
				char string[param->intValue + 1];
				memcpy(string, val, param->intValue);
				string[param->intValue] = 0;
				val += AddPadding(param->intValue);

				s.cat(param->letter);
				if (quoteStrings)
				{
					s.cat('"');
				}
				s.cat(string);
				if (quoteStrings)
				{
					s.cat('"');
				}
				break;
			}
		}
	}
}

