/*
 * BinaryParser.cpp
 *
 *  Created on: 30 Mar 2019
 *      Author: Christian
 */

#include "BinaryParser.h"

#if HAS_LINUX_INTERFACE

#include "GCodeBuffer.h"
#include "ExpressionParser.h"
#include <Platform.h>
#include <RepRap.h>
#include <Networking/NetworkDefs.h>

BinaryParser::BinaryParser(GCodeBuffer& gcodeBuffer) noexcept : gb(gcodeBuffer)
{
	header = reinterpret_cast<const CodeHeader *>(gcodeBuffer.buffer);
}

void BinaryParser::Init() noexcept
{
	gb.bufferState = GCodeBufferState::parseNotStarted;
	seenParameter = nullptr;
	seenParameterValue = nullptr;
}

void BinaryParser::Put(const char *data, size_t len) noexcept
{
	memcpy(gb.buffer, data, len);
	bufferLength = len;
	gb.bufferState = GCodeBufferState::ready;
	gb.machineState->g53Active = (header->flags & CodeFlags::EnforceAbsolutePosition) != 0;
	gb.machineState->lineNumber = header->lineNumber;

	if (reprap.Debug(moduleGcodes))
	{
		String<MaxCodeBufferSize> buf;
		AppendFullCommand(buf.GetRef());
		reprap.GetPlatform().MessageF(DebugMessage, "%s: %s\n", gb.GetIdentity(), buf.c_str());
	}
}

bool BinaryParser::Seen(char c) noexcept
{
	if (bufferLength != 0 && header->numParameters != 0)
	{
		const char *parameterStart = reinterpret_cast<const char*>(gb.buffer) + sizeof(CodeHeader);
		reducedBytesRead = 0;
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
				param->type == DataType::FloatArray ||
				param->type == DataType::DriverIdArray)
			{
				seenParameterValue += param->intValue * sizeof(uint32_t);
			}
			else if (param->type == DataType::String || param->type == DataType::Expression)
			{
				seenParameterValue += AddPadding(param->intValue);
			}
		}
	}
	return false;
}

char BinaryParser::GetCommandLetter() const noexcept
{
	return (bufferLength != 0) ? header->letter : 'Q';
}

bool BinaryParser::HasCommandNumber() const noexcept
{
	return (bufferLength != 0 && (header->flags & CodeFlags::HasMajorCommandNumber) != 0);
}

int BinaryParser::GetCommandNumber() const noexcept
{
	return HasCommandNumber() ? header->majorCode : -1;
}

int8_t BinaryParser::GetCommandFraction() const noexcept
{
	return (bufferLength != 0 && (header->flags & CodeFlags::HasMinorCommandNumber) != 0) ? header->minorCode : -1;
}

float BinaryParser::GetFValue()
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	float value;
	switch (seenParameter->type)
	{
	case DataType::Float:
		value = seenParameter->floatValue;
		break;
	case DataType::Int:
		value = seenParameter->intValue;
		break;
	case DataType::UInt:
		value = seenParameter->uintValue;
		break;
	case DataType::Expression:
		{
			ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
			value = parser.ParseFloat();
			parser.CheckForExtraCharacters();
		}
		break;
	default:
		value = 0.0;
		break;
	}
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	return value;
}

int32_t BinaryParser::GetIValue()
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	int32_t value;
	switch (seenParameter->type)
	{
	case DataType::Float:
		value = seenParameter->floatValue;
		break;
	case DataType::Int:
		value = seenParameter->intValue;
		break;
	case DataType::UInt:
		value = seenParameter->uintValue;
		break;
	case DataType::Expression:
		{
			ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
			value = parser.ParseInteger();
			parser.CheckForExtraCharacters();
		}
		break;
	default:
		value = 0;
		break;
	}
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	return value;
}

uint32_t BinaryParser::GetUIValue()
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	uint32_t value;
	switch (seenParameter->type)
	{
	case DataType::Float:
		value = (uint32_t)seenParameter->floatValue;
		break;
	case DataType::Int:
		value = (uint32_t)seenParameter->intValue;
		break;
	case DataType::UInt:
		value = seenParameter->uintValue;
		break;
	case DataType::Expression:
		{
			ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
			value = parser.ParseUnsigned();
			parser.CheckForExtraCharacters();
		}
		break;
	default:
		value = 0;
		break;
	}
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	return value;
}

// Get a driver ID
DriverId BinaryParser::GetDriverId()
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	DriverId value;
	switch (seenParameter->type)
	{
	case DataType::Int:
	case DataType::UInt:
	case DataType::DriverId:
		value.SetFromBinary(seenParameter->uintValue);
		break;

	default:
		value.Clear();
		break;
	}
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	return value;
}

void BinaryParser::GetIPAddress(IPAddress& returnedIp)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	switch (seenParameter->type)
	{
	case DataType::String:
		{
			const char* p = seenParameterValue;
			uint8_t ip[4];
			unsigned int n = 0;
			for (;;)
			{
				const char *pp;
				const unsigned long v = SafeStrtoul(p, &pp);
				if (pp == p || pp > seenParameterValue + seenParameter->intValue || v > 255)
				{
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
					throw ConstructParseException("invalid IP address");
				}
				++p;
			}

			if (n != 4)
			{
				throw ConstructParseException("invalid IP address");
			}
			returnedIp.SetV4(ip);
		}
		break;

	case DataType::Expression:
		//TODO not handled yet
	default:
		throw ConstructParseException("IP address string expected");
	}

	seenParameter = nullptr;
	seenParameterValue = nullptr;
}

void BinaryParser::GetMacAddress(MacAddress& mac)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	switch (seenParameter->type)
	{
	case DataType::String:
		{
			const char* p = seenParameterValue;
			unsigned int n = 0;
			for (;;)
			{
				const char *pp;
				const unsigned long v = SafeStrtoul(p, &pp, 16);
				if (pp == p || pp > seenParameterValue + seenParameter->intValue || v > 255)
				{
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
					throw ConstructParseException("invalid MAC address");
				}
				++p;
			}

			if (n != 6)
			{
				throw ConstructParseException("invalid MAC address");
			}
		}
		break;

	case DataType::Expression:
		//TODO not handled yet
	default:
		throw ConstructParseException("MAC address string expected");
	}

	seenParameter = nullptr;
	seenParameterValue = nullptr;
}

void BinaryParser::GetUnprecedentedString(const StringRef& str, bool allowEmpty)
{
	//TODO DSF should know which commands take a string parameter without a preceding parameter letter,
	// so it should send the argument as a string or as an expression with a dummy parameter letter.
	str.Clear();
	WriteParameters(str, false);
	if (!allowEmpty && str.IsEmpty())
	{
		throw ConstructParseException("non-empty string expected");
	}
}

// Get the complete parameter string
const char *BinaryParser::GetCompleteParameters() const noexcept
{
	return "";			// If we decide to support Q codes from DSF then change this to copy the single parameter to gb.buffer and return a pointer to it
}

void BinaryParser::GetQuotedString(const StringRef& str)
{
	GetPossiblyQuotedString(str);
}

void BinaryParser::GetPossiblyQuotedString(const StringRef& str)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	switch (seenParameter->type)
	{
	case DataType::String:
		str.copy(seenParameterValue, seenParameter->intValue);
		break;

	case DataType::Expression:
		{
			ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
			const ExpressionValue val = parser.Parse();
			parser.CheckForExtraCharacters();
			val.AppendAsString(str);
		}
		break;

	default:
		str.Clear();
		break;
	}

	seenParameter = nullptr;
	seenParameterValue = nullptr;
	if (str.IsEmpty())
	{
		throw ConstructParseException("non-empty string expected");
	}
}

void BinaryParser::GetReducedString(const StringRef& str)
{
	str.Clear();
	if (seenParameterValue != nullptr && seenParameter->type == DataType::String)
	{
		while (reducedBytesRead < seenParameter->intValue)
		{
			const char c = seenParameterValue[reducedBytesRead++];
			switch(c)
			{
			case '_':
			case '-':
			case ' ':
				break;

			default:
				if (c < ' ')
				{
					seenParameter = nullptr;
					seenParameterValue = nullptr;
					throw ConstructParseException("control character in string");
				}
				str.cat(tolower(c));
				break;
			}
		}
	}

	seenParameter = nullptr;
	seenParameterValue = nullptr;
	if (str.IsEmpty())
	{
		throw ConstructParseException("non-empty string expected");
	}
}

void BinaryParser::GetFloatArray(float arr[], size_t& length, bool doPad)
{
	GetArray(arr, length, doPad);
}

void BinaryParser::GetIntArray(int32_t arr[], size_t& length, bool doPad)
{
	GetArray(arr, length, doPad);
}

void BinaryParser::GetUnsignedArray(uint32_t arr[], size_t& length, bool doPad)
{
	GetArray(arr, length, doPad);
}

// Get a :-separated list of drivers after a key letter
void BinaryParser::GetDriverIdArray(DriverId arr[], size_t& length)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	switch (seenParameter->type)
	{
	case DataType::Int:
	case DataType::UInt:
	case DataType::DriverId:
		arr[0].SetFromBinary(seenParameter->uintValue);
		length = 1;
		break;

	case DataType::IntArray:
	case DataType::UIntArray:
	case DataType::DriverIdArray:
		for (int i = 0; i < seenParameter->intValue; i++)
		{
			arr[i].SetFromBinary(reinterpret_cast<const uint32_t*>(seenParameterValue)[i]);
		}
		length = seenParameter->intValue;
		break;

	default:
		length = 0;
		return;
	}
}

void BinaryParser::SetFinished() noexcept
{
	gb.machineState->g53Active = false;		// G53 does not persist beyond the current command
	Init();
}

FilePosition BinaryParser::GetFilePosition() const noexcept
{
	return ((header->flags & CodeFlags::HasFilePosition) != 0) ? header->filePosition : noFilePosition;
}

const char* BinaryParser::DataStart() const noexcept
{
	return gb.buffer;
}

size_t BinaryParser::DataLength() const noexcept
{
	return bufferLength;
}

void BinaryParser::PrintCommand(const StringRef& s) const noexcept
{
	if (bufferLength != 0 && (header->flags & CodeFlags::HasMajorCommandNumber) != 0)
	{
		s.printf("%c%" PRId32, header->letter, header->majorCode);
		if ((header->flags & CodeFlags::HasMinorCommandNumber) != 0)
		{
			s.catf(".%" PRId32, header->minorCode);
		}
	}
	else
	{
		s.Clear();
	}
}

void BinaryParser::AppendFullCommand(const StringRef &s) const noexcept
{
	if (bufferLength != 0)
	{
		if ((header->flags & CodeFlags::HasMajorCommandNumber) != 0)
		{
			s.catf("%c%" PRId32, header->letter, header->majorCode);
			if ((header->flags & CodeFlags::HasMinorCommandNumber) != 0)
			{
				s.catf(".%" PRId32, header->minorCode);
			}
		}

		if (header->numParameters != 0)
		{
			s.cat(' ');
		}
		WriteParameters(s, true);
	}
}

//TODO need a way to pass arrays in which one or more elements is an expression from DSF to RRF
template<typename T> void BinaryParser::GetArray(T arr[], size_t& length, bool doPad)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	int lastIndex = -1;
	switch (seenParameter->type)
	{
	case DataType::Int:
		arr[0] = seenParameter->intValue;
		lastIndex = 0;
		break;
	case DataType::UInt:
	case DataType::DriverId:
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
	case DataType::DriverIdArray:
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
	default:
		length = 0;
		return;
	}

	if (doPad && lastIndex >= 0)
	{
		for (size_t i = lastIndex + 1; i < length; i++)
		{
			arr[i] = arr[lastIndex];
		}
	}
	else
	{
		length = lastIndex + 1;
	}
}

void BinaryParser::WriteParameters(const StringRef& s, bool quoteStrings) const noexcept
{
	if (bufferLength != 0)
	{
		const char *parameterStart = reinterpret_cast<const char *>(gb.buffer) + sizeof(CodeHeader);
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
			{
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
			case DataType::DriverId:
				s.catf("%c%d.%d", param->letter, (int)(param->uintValue >> 16), (int)(param->uintValue & 0xFFFF));
				break;
			case DataType::DriverIdArray:
				s.cat(param->letter);
				for (int k = 0; k < param->intValue; k++)
				{
					if (k != 0)
					{
						s.cat(':');
					}
					const uint32_t driver = *reinterpret_cast<const uint32_t*>(val);
					s.catf("%d.%d", (int)(driver >> 16), (int)(driver & 0xFFFF));
					val += sizeof(uint32_t);
				}
				break;
			case DataType::Bool:
				s.catf("%c%c", param->letter, (param->intValue != 0) ? '1' : '0');
				break;
			case DataType::BoolArray:
				s.cat(param->letter);
				for (int k = 0; k < param->intValue; k++)
				{
					if (k != 0)
					{
						s.cat(':');
					}
					s.cat(((const uint8_t*)val != 0) ? '1' : '0');
					val += sizeof(uint8_t);
				}
				break;
			}
		}
	}
}

GCodeException BinaryParser::ConstructParseException(const char *str) const noexcept
{
	return GCodeException(header->lineNumber, -1, str);
}

GCodeException BinaryParser::ConstructParseException(const char *str, const char *param) const noexcept
{
	return GCodeException(header->lineNumber, -1, str, param);
}

GCodeException BinaryParser::ConstructParseException(const char *str, uint32_t param) const noexcept
{
	return GCodeException(header->lineNumber, -1, str, param);
}

#endif

// End
