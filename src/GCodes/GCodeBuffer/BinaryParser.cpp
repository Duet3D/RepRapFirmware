/*
 * BinaryParser.cpp
 *
 *  Created on: 30 Mar 2019
 *      Author: Christian
 */

#include "BinaryParser.h"

#if HAS_SBC_INTERFACE

#include "GCodeBuffer.h"
#include "ExpressionParser.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <ObjectModel/Variable.h>
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

// Add an entire binary G-Code, overwriting any existing content
// CAUTION! This may be called with the task scheduler suspended, so don't do anything that might block or take more than a few microseconds to execute
void BinaryParser::Put(const uint32_t *data, size_t len) noexcept
{
	memcpyu32(reinterpret_cast<uint32_t *>(gb.buffer), data, len);
	bufferLength = len * sizeof(uint32_t);
	gb.bufferState = GCodeBufferState::parsingGCode;
	gb.LatestMachineState().g53Active = (header->flags & CodeFlags::EnforceAbsolutePosition) != 0;
	gb.CurrentFileMachineState().lineNumber = header->lineNumber;
}

void BinaryParser::DecodeCommand() noexcept
{
	if (gb.bufferState == GCodeBufferState::parsingGCode)
	{
		if (reprap.GetDebugFlags(moduleGcodes).IsBitSet(gb.GetChannel().ToBaseType()))
		{
			String<MaxCodeBufferSize> buf;
			AppendFullCommand(buf.GetRef());
			debugPrintf("%s: %s\n", gb.GetIdentity(), buf.c_str());
		}
		gb.bufferState = GCodeBufferState::executing;
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

			// Skip to the next parameter
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

// Return true if any of the parameter letters in the bitmap were seen
bool BinaryParser::SeenAny(Bitmap<uint32_t> bm) const noexcept
{
	if (bufferLength != 0 && header->numParameters != 0)
	{
		const char *parameterStart = reinterpret_cast<const char*>(gb.buffer) + sizeof(CodeHeader);
		for (size_t i = 0; i < header->numParameters; i++)
		{
			const CodeParameter *param = reinterpret_cast<const CodeParameter*>(parameterStart + i * sizeof(CodeParameter));
			const char paramLetter = param->letter;
			if (paramLetter >= 'A' && paramLetter <= 'Z' && bm.IsBitSet(paramLetter - 'A'))
			{
				return true;
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

bool BinaryParser::ContainsExpression() const noexcept
{
	if (bufferLength != 0 && header->numParameters != 0)
	{
		const char *parameterStart = reinterpret_cast<const char*>(gb.buffer) + sizeof(CodeHeader);
		for (size_t i = 0; i < header->numParameters; i++)
		{
			const CodeParameter *param = reinterpret_cast<const CodeParameter*>(parameterStart + i * sizeof(CodeParameter));
			if (param->type == DataType::Expression)
			{
				return true;
			}
		}
	}
	return false;
}

float BinaryParser::GetFValue() THROWS(GCodeException)
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

int32_t BinaryParser::GetIValue() THROWS(GCodeException)
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

uint32_t BinaryParser::GetUIValue() THROWS(GCodeException)
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

void BinaryParser::SetDriverIdFromBinary(DriverId& did, uint32_t val) THROWS(GCodeException)
{
#if SUPPORT_CAN_EXPANSION
	did.SetFromBinary(val);
#else
	if (did.SetFromBinary(val))
	{
		throw ConstructParseException("Board address of driver must be 0");
	}
#endif
}

void BinaryParser::SetDriverIdFromFloat(DriverId& did, float fval) THROWS(GCodeException)
{
	fval *= 10.0;
	const int32_t ival = lrintf(fval);
#if SUPPORT_CAN_EXPANSION
	if (ival >= 0 && fabsf(fval - (float)ival) <= 0.002)
	{
		did.boardAddress = ival/10;
		did.localDriver = ival % 10;
	}
#else
	if (ival >= 0 && ival < 10 && fabsf(fval - (float)ival) <= 0.002)
	{
		did.localDriver = ival % 10;
	}
#endif
	else
	{
		throw ConstructParseException("Invalid driver ID expression");
	}
}

// Get a driver ID
DriverId BinaryParser::GetDriverId() THROWS(GCodeException)
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
	case DataType::DriverId_dt:
		SetDriverIdFromBinary(value, seenParameter->uintValue);
		break;

	case DataType::Float:
		SetDriverIdFromFloat(value, seenParameter->floatValue);
		break;

	case DataType::Expression:
		{
			ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
			const float fval = parser.ParseFloat();
			parser.CheckForExtraCharacters();
			SetDriverIdFromFloat(value, fval);
		}
		break;

	default:
		break;
	}
	seenParameter = nullptr;
	seenParameterValue = nullptr;
	return value;
}

void BinaryParser::GetIPAddress(IPAddress& returnedIp) THROWS(GCodeException)
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
				const uint32_t v = StrToU32(p, &pp);
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

void BinaryParser::GetMacAddress(MacAddress& mac) THROWS(GCodeException)
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
				const uint32_t v = StrHexToU32(p, &pp);
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

void BinaryParser::GetUnprecedentedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
{
	if (Seen('@') || Seen('\0'))		// DCS 2.1.3 and earlier use '\0', later DCS versions use '@'
	{
		GetPossiblyQuotedString(str, allowEmpty);
	}
	else if (!allowEmpty)
	{
		throw ConstructParseException("non-empty string expected");
	}
}

void BinaryParser::GetPossiblyQuotedString(const StringRef& str, bool allowEmpty) THROWS(GCodeException)
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
	if (!allowEmpty && str.IsEmpty())
	{
		throw ConstructParseException("non-empty string expected");
	}
}

void BinaryParser::GetFloatArray(float arr[], size_t& length) THROWS(GCodeException)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	if (seenParameter->type == DataType::Expression)
	{
		ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
		parser.ParseFloatArray(arr, length);
	}
	else
	{
		GetArray(arr, length);
	}
}

void BinaryParser::GetIntArray(int32_t arr[], size_t& length) THROWS(GCodeException)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	if (seenParameter->type == DataType::Expression)
	{
		ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
		parser.ParseIntArray(arr, length);
	}
	else
	{
		GetArray(arr, length);
	}
}

void BinaryParser::GetUnsignedArray(uint32_t arr[], size_t& length) THROWS(GCodeException)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	if (seenParameter->type == DataType::Expression)
	{
		ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
		parser.ParseUnsignedArray(arr, length);
	}
	else
	{
		GetArray(arr, length);
	}
}

// Get a :-separated list of drivers after a key letter
void BinaryParser::GetDriverIdArray(DriverId arr[], size_t& length) THROWS(GCodeException)
{
	if (seenParameter == nullptr)
	{
		THROW_INTERNAL_ERROR;
	}

	switch (seenParameter->type)
	{
	case DataType::Int:
	case DataType::UInt:
	case DataType::DriverId_dt:
		SetDriverIdFromBinary(arr[0], seenParameter->uintValue);
		length = 1;
		break;

	case DataType::IntArray:
	case DataType::UIntArray:
	case DataType::DriverIdArray:
		CheckArrayLength(length);
		for (int i = 0; i < seenParameter->intValue; i++)
		{
			SetDriverIdFromBinary(arr[i], reinterpret_cast<const uint32_t*>(seenParameterValue)[i]);
		}
		length = seenParameter->intValue;
		break;

	case DataType::Expression:
		{
			ExpressionParser parser(gb, seenParameterValue, seenParameterValue + seenParameter->intValue, -1);
			parser.ParseDriverIdArray(arr, length);
			parser.CheckForExtraCharacters();
		}
		break;

	default:
		length = 0;
		return;
	}
}

void BinaryParser::SetFinished() noexcept
{
	gb.LatestMachineState().g53Active = false;		// G53 does not persist beyond the current command
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

template<typename T> void BinaryParser::GetArray(T arr[], size_t& length) THROWS(GCodeException)
{
	int lastIndex = -1;
	switch (seenParameter->type)
	{
	case DataType::Int:
		arr[0] = (T)seenParameter->intValue;
		lastIndex = 0;
		break;

	case DataType::UInt:
	case DataType::DriverId_dt:
		arr[0] = (T)seenParameter->uintValue;
		lastIndex = 0;
		break;

	case DataType::Float:
		arr[0] = (T)seenParameter->floatValue;
		lastIndex = 0;
		break;

	case DataType::IntArray:
		CheckArrayLength(length);
		for (int i = 0; i < seenParameter->intValue; i++)
		{
			arr[i] = (T)reinterpret_cast<const int32_t*>(seenParameterValue)[i];
		}
		lastIndex = seenParameter->intValue - 1;
		break;

	case DataType::DriverIdArray:
	case DataType::UIntArray:
		CheckArrayLength(length);
		for (int i = 0; i < seenParameter->intValue; i++)
		{
			arr[i] = (T)reinterpret_cast<const uint32_t*>(seenParameterValue)[i];
		}
		lastIndex = seenParameter->intValue - 1;
		break;

	case DataType::FloatArray:
		CheckArrayLength(length);
		for (int i = 0; i < seenParameter->intValue; i++)
		{
			arr[i] = (T)reinterpret_cast<const float*>(seenParameterValue)[i];
		}
		lastIndex = seenParameter->intValue - 1;
		break;

	default:
		length = 0;
		return;
	}

	length = lastIndex + 1;
}

void BinaryParser::CheckArrayLength(size_t maxLength) THROWS(GCodeException)
{
	if ((unsigned int)seenParameter->intValue > maxLength)
	{
		throw ConstructParseException("array too long, max length = %u", (uint32_t)maxLength);
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
			case DataType::DateTime:
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
			case DataType::DriverId_dt:
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
			case DataType::ULong:
			{
				uint64_t ulVal;
				memcpy(reinterpret_cast<char *>(&ulVal), val, sizeof(uint64_t));
				s.catf("%" PRIu64, ulVal);
				break;
			}
			case DataType::Null:
				s.cat("null");
				break;
			}
		}
	}
}

void BinaryParser::AddParameters(VariableSet& vs, int codeRunning) noexcept
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
			if (param->letter != 'P' || codeRunning != 98)
			{
				ExpressionValue ev;
				switch (param->type)
				{
				case DataType::String:
					{
						StringHandle sh(seenParameterValue, param->intValue);
						ev.Set(sh);
					}
					break;

				case DataType::Expression:
					try
					{
						ExpressionParser parser(gb, seenParameterValue, seenParameterValue + param->intValue, -1);
						ev = parser.Parse();
					}
					catch (const GCodeException&) { }			// TODO error handling
					break;

				case DataType::Float:
					ev.SetFloat(param->floatValue);
					break;

				case DataType::Int:
					ev.SetSigned(param->intValue);
					break;

				case DataType::UInt:
					ev.SetSigned((int32_t)param->uintValue);
					break;

				default:
					break;
				}

				if (ev.GetType() != TypeCode::None)
				{
					char paramName[2] = { param->letter, 0 };
					vs.InsertNewParameter(paramName, ev);
				}
			}

			// Skip to the next parameter
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
