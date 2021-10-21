/*
 * CanMessageGenericConstructor.cpp
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#include "CanMessageGenericConstructor.h"
#include "Hardware/IoPorts.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMessageBuffer.h"
#include "CanInterface.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#define STRINGIZE(_v) #_v

CanMessageGenericConstructor::CanMessageGenericConstructor(const ParamDescriptor *p_param) noexcept
	: paramTable(p_param), dataLen(0)
{
	msg.paramMap = 0;
}

// Append a value to the data, throwing if it wouldn't fit
void CanMessageGenericConstructor::StoreValue(const void *vp, size_t sz) THROWS(GCodeException)
{
	if (dataLen + sz > sizeof(msg.data))
	{
		throw ConstructParseException("CAN message too long");
	}
	memcpy(msg.data + dataLen, vp, sz);
	dataLen += sz;
}

// Insert a value in the data, throwing if it wouldn't fit
void CanMessageGenericConstructor::InsertValue(const void *vp, size_t sz, size_t pos) THROWS(GCodeException)
{
	if (dataLen + sz > sizeof(msg.data))
	{
		throw ConstructParseException("CAN message too long");
	}
	memmove(msg.data + pos + sz, msg.data + pos, dataLen - pos);
	memcpy(msg.data + pos, vp, sz);
	dataLen += sz;
}

// Populate the CAN message from a GCode message returning true if successful. Throws if an error occurs.
void CanMessageGenericConstructor::PopulateFromCommand(GCodeBuffer& gb) THROWS(GCodeException)
{
	uint32_t paramBit = 1;
	for (const ParamDescriptor *d = paramTable; d->letter != 0; ++d)
	{
		if (d->letter >= 'A' && d->letter <= 'Z' && gb.Seen(d->letter))
		{
			switch (d->type)
			{
			case ParamDescriptor::uint32:
				StoreValue(gb.GetUIValue());
				break;

			case ParamDescriptor::int32:
				StoreValue(gb.GetIValue());
				break;

			case ParamDescriptor::uint16:
				StoreValue((uint16_t)min<uint32_t>(gb.GetUIValue(), std::numeric_limits<uint16_t>::max()));
				break;

			case ParamDescriptor::int16:
				StoreValue((int16_t)constrain<int32_t>(gb.GetIValue(), std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::max()));
				break;

			case ParamDescriptor::uint8:
				StoreValue((uint8_t)min<uint32_t>(gb.GetUIValue(), std::numeric_limits<uint8_t>::max()));
				break;

			case ParamDescriptor::int8:
				StoreValue((int8_t)constrain<int32_t>(gb.GetIValue(), std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max()));
				break;

			case ParamDescriptor::localDriver:
				{
					const DriverId id = gb.GetDriverId();
					StoreValue(id.localDriver);
				}
				break;

			case ParamDescriptor::float_p:
				StoreValue(gb.GetFValue());
				break;

			case ParamDescriptor::char_p:
				{
					String<StringLength20> str;
					gb.GetQuotedString(str.GetRef());
					if (str.strlen() != 1)
					{
						throw ConstructParseException("expected single-character quoted string after '%c'", (uint32_t)d->letter);
					}
					StoreValue(str[0]);
				}
				break;

			case ParamDescriptor::string:
				{
					String<StringLength50> str;
					gb.GetQuotedString(str.GetRef());
					StoreValue(str.c_str(), str.strlen() + 1);
				}
				break;

			case ParamDescriptor::reducedString:
				{
					String<StringLength50> str;
					gb.GetReducedString(str.GetRef());
					// We don't want port names sent to expansion boards to include the board number, so remove the board number.
					// We also use the reducedString type for sensor names, but they should't start with digits followed by '.'.
					(void)IoPort::RemoveBoardAddress(str.GetRef());
					StoreValue(str.c_str(), str.strlen() + 1);
				}
				break;

			case ParamDescriptor::uint32_array:
			case ParamDescriptor::uint16_array:
			case ParamDescriptor::uint8_array:
				{
					uint32_t arr[59];				// max size is 59 * uint8_t + 1 length byte + parameters present bitmap = 64
					size_t siz = min<size_t>(ARRAY_SIZE(arr), d->maxArrayLength);
					gb.GetUnsignedArray(arr, siz, false);
					StoreValue((uint8_t)siz);
					for (size_t i = 0; i < siz; ++i)
					{
						StoreValue(&arr[i], d->ItemSize());
					}
				}
				break;

			case ParamDescriptor::float_array:
				{
					float arr[14];
					size_t siz = min<size_t>(ARRAY_SIZE(arr), d->maxArrayLength);
					gb.GetFloatArray(arr, siz, false);
					StoreValue((uint8_t)siz);
					for (size_t i = 0; i < siz; ++i)
					{
						StoreValue(arr[i]);
					}
				}
				break;

			default:
				throw ConstructParseException("internal error at " __FILE__ "(" STRINGIZE(#__LINE__) ")");
			}
			msg.paramMap |= paramBit;
		}
		paramBit <<= 1;
	}
}

// Return the correct position in the data to insert a parameter. If successful, add the bit to the parameter map and pass back the expect5ed parameter type; else throw.
unsigned int CanMessageGenericConstructor::FindInsertPoint(char c, ParamDescriptor::ParamType& t, size_t &sz) THROWS(GCodeException)
{
	unsigned int pos = 0;
	uint32_t paramBit = 1;
	for (const ParamDescriptor *d = paramTable; d->letter != 0; ++d)
	{
		const bool present = (msg.paramMap & paramBit) != 0;
		if (d->letter == c)
		{
			if (present)
			{
				throw ConstructParseException("duplicate parameter");
			}
			msg.paramMap |= paramBit;
			t = d->type;
			sz = d->ItemSize();
			return pos;
		}

		if (present)
		{
			// This parameter is present, so skip it
			const size_t size = d->ItemSize();
			if (size != 0)
			{
				pos += size;
			}
			else
			{
				// The only item with size 0 is string, so skip up to and including the null terminator
				do
				{
				} while (msg.data[pos++] != 0);
			}
		}
		paramBit <<= 1;
	}
	throw ConstructParseException("wrong parameter letter");
}

//TODO factor out the common code in the following several routines
void CanMessageGenericConstructor::AddU64Param(char c, uint64_t v) THROWS(GCodeException)
{
	ParamDescriptor::ParamType t;
	size_t sz;
	const unsigned int pos = FindInsertPoint(c, t, sz);
	if (t != ParamDescriptor::uint64)
	{
		throw ConstructParseException("u64val wrong parameter type");
	}
	InsertValue(&v, sz, pos);
}

void CanMessageGenericConstructor::AddUParam(char c, uint32_t v) THROWS(GCodeException)
{
	ParamDescriptor::ParamType t;
	size_t sz;
	const unsigned int pos = FindInsertPoint(c, t, sz);
	switch (t)
	{
	case ParamDescriptor::uint32:
		break;

	case ParamDescriptor::uint16:
	case ParamDescriptor::pwmFreq:
		if (v >= (1u << 16))
		{
			throw ConstructParseException("uval too large");
		}
		break;

	case ParamDescriptor::uint8:
		if (v >= (1u << 8))
		{
			throw ConstructParseException("uval too large");
		}
		break;

	default:
		throw ConstructParseException("uval wrong parameter type");
	}

	InsertValue(&v, sz, pos);
}

void CanMessageGenericConstructor::AddIParam(char c, int32_t v) THROWS(GCodeException)
{
	ParamDescriptor::ParamType t;
	size_t sz;
	const unsigned int pos = FindInsertPoint(c, t, sz);
	switch (t)
	{
	case ParamDescriptor::int32:
		break;

	case ParamDescriptor::int16:
		if (v >= (int32_t)(1u << 15) || v < -(int32_t)(1u << 15))
		{
			throw ConstructParseException("ival too large");
		}
		break;

	case ParamDescriptor::uint8:
		if (v >= (int32_t)(1u << 7) || v < -(int32_t)(1u << 7))
		{
			throw ConstructParseException("ival too large");
		}
		break;

	default:
		throw ConstructParseException("ival wrong parameter type");
	}

	InsertValue(&v, sz, pos);
}

void CanMessageGenericConstructor::AddFParam(char c, float v) THROWS(GCodeException)
{
	ParamDescriptor::ParamType t;
	size_t sz;
	const unsigned int pos = FindInsertPoint(c, t, sz);
	if (t != ParamDescriptor::float_p)
	{
		throw ConstructParseException("fval wrong parameter type");
	}
	InsertValue(&v, sz, pos);
}

void CanMessageGenericConstructor::AddCharParam(char c, char v) THROWS(GCodeException)
{
	ParamDescriptor::ParamType t;
	size_t sz;
	const unsigned int pos = FindInsertPoint(c, t, sz);
	if (t != ParamDescriptor::char_p)
	{
		throw ConstructParseException("cval wrong parameter type");
	}
	InsertValue(&v, sz, pos);
}

void CanMessageGenericConstructor::AddStringParam(char c, const char *v) THROWS(GCodeException)
{
	ParamDescriptor::ParamType t;
	size_t sz;
	const unsigned int pos = FindInsertPoint(c, t, sz);
	switch (t)
	{
	case ParamDescriptor::string:
	case ParamDescriptor::reducedString:			//TODO currently we don't reduce the string, but it should already be reduced
		InsertValue(v, strlen(v) + 1, pos);
		break;

	default:
		throw ConstructParseException("sval wrong parameter type");
	}
}

void CanMessageGenericConstructor::AddDriverIdParam(char c, DriverId did) THROWS(GCodeException)
{
	ParamDescriptor::ParamType t;
	size_t sz;
	const unsigned int pos = FindInsertPoint(c, t, sz);
	if (t != ParamDescriptor::localDriver)
	{
		throw ConstructParseException("didval wrong parameter type");
	}

	InsertValue(&did.localDriver, sz, pos);
}

GCodeResult CanMessageGenericConstructor::SendAndGetResponse(CanMessageType msgType, CanAddress dest, const StringRef& reply) const noexcept
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("no CAN buffer available");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(dest, buf);
	const size_t actualMessageLength = CanMessageGeneric::GetActualDataLength(dataLen);
	CanMessageGeneric *m2 = buf->SetupGenericRequestMessage(rid, CanInterface::GetCanAddress(), dest, msgType, actualMessageLength);
	memcpy(m2, &msg, actualMessageLength);
	m2->requestId = rid;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

#endif

// End
