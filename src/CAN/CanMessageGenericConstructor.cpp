/*
 * CanMessageGenericConstructo.cpp
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#include "CanMessageGenericConstructor.h"
#include "CanMessageBuffer.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#define STRINGIZE(_v) #_v

CanMessageGenericConstructor::CanMessageGenericConstructor(const ParamDescriptor *p_param)
	: paramTable(p_param), dataLen(0), err(nullptr)
{
	msg.paramMap = 0;
}

// Append a value to the data, returning true if it wouldn't fit
bool CanMessageGenericConstructor::StoreValue(const void *vp, size_t sz)
{
	if (dataLen + sz < sizeof(msg.data))
	{
		memcpy(msg.data + dataLen, vp, sz);
		dataLen += sz;
		return false;
	}
	return true;
}

// Populate the CAN message from a GCode message returning true if successful. If an error occurs, write the message to 'reply' and return false.
bool CanMessageGenericConstructor::PopulateFromCommand(GCodeBuffer& gb, const StringRef& reply)
{
	uint32_t paramBit = 1;
	for (const ParamDescriptor *d = paramTable; d->letter != 0; ++d)
	{
		if (gb.Seen(d->letter))
		{
			bool overflowed;
			switch (d->type)
			{
			case ParamDescriptor::uint32:
				{
					const uint32_t val = gb.GetUIValue();
					overflowed = StoreValue(val);
				}
				break;

			case ParamDescriptor::int32:
				{
					const int32_t val = gb.GetIValue();
					overflowed = StoreValue(val);
				}
				break;

			case ParamDescriptor::uint16:
				{
					const uint16_t val = (uint16_t)gb.GetUIValue();
					overflowed = StoreValue(val);
				}
				break;

			case ParamDescriptor::int16:
				{
					const int16_t val = (int16_t)gb.GetIValue();
					overflowed = StoreValue(val);
				}
				break;

			case ParamDescriptor::uint8:
				{
					const uint8_t val = (uint8_t)gb.GetUIValue();
					overflowed = StoreValue(val);
				}
				break;

			case ParamDescriptor::int8:
				{
					const int8_t val = (int8_t)gb.GetIValue();
					overflowed = StoreValue(val);
				}
				break;

			case ParamDescriptor::float_p:
				{
					const float val = gb.GetFValue();
					overflowed = StoreValue(val);
				}
				break;

			case ParamDescriptor::char_p:
				{
					String<StringLength20> str;
					if (!gb.GetQuotedString(str.GetRef()))
					{
						reply.printf("expected quoted string after '%c'", d->letter);
						return false;
					}
					if (str.strlen() != 1)
					{
						reply.printf("expected single-character quoted string after '%c'", d->letter);
						return false;
					}
					const char c = str[0];
					overflowed = StoreValue(c);
				}
				break;

			case ParamDescriptor::string:
				{
					String<StringLength20> str;
					if (!gb.GetQuotedString(str.GetRef()))
					{
						reply.printf("expected quoted string after '%c'", d->letter);
						return false;
					}
					overflowed = StoreValue(str.c_str(), str.strlen() + 1);
				}
				break;

			case ParamDescriptor::reducedString:
				{
					String<StringLength20> str;
					if (!gb.GetReducedString(str.GetRef()))
					{
						reply.printf("expected quoted string after '%c'", d->letter);
						return false;
					}
					overflowed = StoreValue(str.c_str(), str.strlen() + 1);
				}
				break;

			default:
				reply.copy("internal error at " __FILE__ "(" STRINGIZE(#__LINE__) ")");
				return false;
			}
			if (overflowed)
			{
				reply.copy("CAN message too long");
				return false;
			}
			msg.paramMap |= paramBit;
		}
		paramBit <<= 1;
	}
	return true;
}

// Look for the specified parameter. If found, return its type and set the index of the corresponding data.
void CanMessageGenericConstructor::AddParam(char c, const char *v, ParamDescriptor::ParamType expectedType, size_t length)
{
	unsigned int pos = 0;
	uint32_t paramBit = 1;
	for (const ParamDescriptor *d = paramTable; d->letter != 0; ++d)
	{
		const bool present = (msg.paramMap & paramBit) != 0;
		if (d->letter == c)
		{
			if (d->type != expectedType)
			{
				err = "wrong parameter type";
				return;
			}
			if (present)
			{
				err = "duplicate parameter";
				return;
			}
			if (dataLen + length > sizeof(msg.data))
			{
				err = "overflow";
				return;
			}
			memmove(msg.data + pos + length, msg.data + pos, dataLen - pos);
			memcpy(msg.data + pos, v, length);
			msg.paramMap |= paramBit;
			return;
		}
		if (present)
		{
			// This parameter is present, so skip it
			unsigned int size = d->ItemSize();
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
	err = "wrong parameter letter";
}

GCodeResult CanMessageGenericConstructor::SendAndGetResponse(CanMessageType msgType, uint16_t dest, const StringRef& reply)
{
	CanMessageBuffer * buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("no CAN buffer available");
		return GCodeResult::error;
	}

	CanMessageGeneric *m2 = buf->SetupGenericMessage(msgType, dest, dataLen);
	memcpy(m2, &msg, dataLen + sizeof(msg.paramMap));
	//TODO
	m2->DebugPrint(paramTable);
	CanMessageBuffer::Free(buf);
	reply.copy("CAN message sending not implemented");
	return GCodeResult::error;
}

// End
