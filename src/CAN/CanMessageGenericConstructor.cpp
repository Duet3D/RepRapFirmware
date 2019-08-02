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
#include "CanSender.h"
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
	if (dataLen + sz <= sizeof(msg.data))
	{
		memcpy(msg.data + dataLen, vp, sz);
		dataLen += sz;
		return false;
	}
	return true;
}

// Insert a value in the data, returning true if it wouldn't fit
bool CanMessageGenericConstructor::InsertValue(const void *vp, size_t sz, size_t pos)
{
	if (dataLen + sz <= sizeof(msg.data))
	{
		memmove(msg.data + pos + sz, msg.data + pos, dataLen - pos);
		memcpy(msg.data + pos, vp, sz);
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
					// We don't want port names sent to expansion boards to include the board number, so remove the board number.
					// We also use the reducedString type for sensor names, but they should't start with digits followed by '.'.
					(void)IoPort::RemoveBoardAddress(str.GetRef());
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

void CanMessageGenericConstructor::AddUParam(char c, uint32_t v)
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
				err = "duplicate parameter";
				return;
			}
			else
			{
				switch (d->type)
				{
				case ParamDescriptor::uint32:
					break;

				case ParamDescriptor::uint16:
					if (v >= (1u << 16))
					{
						err = "uval too large";
					}
					break;

				case ParamDescriptor::uint8:
					if (v >= (1u << 8))
					{
						err = "uval too large";
					}
					break;

				default:
					err = "uval wrong parameter type";
					return;
				}

				if (InsertValue(&v, d->ItemSize(), pos))
				{
					err = "overflow";
				}
				else
				{
					msg.paramMap |= paramBit;
				}
			}
			return;
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
	err = "wrong parameter letter";
}

void CanMessageGenericConstructor::AddIParam(char c, int32_t v)
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
				err = "duplicate parameter";
			}
			else
			{
				switch (d->type)
				{
				case ParamDescriptor::int32:
					break;

				case ParamDescriptor::int16:
					if (v >= (int32_t)(1u << 15) || v < -(int32_t)(1u << 15))
					{
						err = "ival too large";
					}
					break;

				case ParamDescriptor::uint8:
					if (v >= (int32_t)(1u << 7) || v < -(int32_t)(1u << 7))
					{
						err = "ival too large";
					}
					break;

				default:
					err = "ival wrong parameter type";
					return;
				}

				if (InsertValue(&v, d->ItemSize(), pos))
				{
					err = "overflow";
				}
				else
				{
					msg.paramMap |= paramBit;
				}
			}
			return;
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
	err = "wrong parameter letter";
}

void CanMessageGenericConstructor::AddFParam(char c, float v)
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
				err = "duplicate parameter";
			}
			else if (d->type != ParamDescriptor::float_p)
			{
				err = "fval wrong parameter type";
			}
			else if (InsertValue(&v, d->ItemSize(), pos))
			{
				err = "overflow";
			}
			else
			{
				msg.paramMap |= paramBit;
			}
			return;
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
	err = "wrong parameter letter";
}

void CanMessageGenericConstructor::AddCharParam(char c, char v)
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
				err = "duplicate parameter";
			}
			else if (d->type != ParamDescriptor::char_p)
			{
				err = "cval wrong parameter type";
			}
			else if (InsertValue(&v, d->ItemSize(), pos))
			{
				err = "overflow";
			}
			else
			{
				msg.paramMap |= paramBit;
			}
			return;
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
	err = "wrong parameter letter";
}

void CanMessageGenericConstructor::AddStringParam(char c, const char *v)
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
				err = "duplicate parameter";
			}
			else if (d->type != ParamDescriptor::string)
			{
				err = "sval wrong parameter type";
			}
			else if (InsertValue(&v, strlen(v) + 1, pos))
			{
				err = "overflow";
			}
			else
			{
				msg.paramMap |= paramBit;
			}
			return;
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
	err = "wrong parameter letter";
}

GCodeResult CanMessageGenericConstructor::SendAndGetResponse(CanMessageType msgType, CanAddress dest, const StringRef& reply)
{
	CanMessageBuffer * buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("no CAN buffer available");
		return GCodeResult::error;
	}

	CanMessageGeneric *m2 = buf->SetupGenericMessage(msgType, dest, dataLen + sizeof(msg.paramMap));
	memcpy(m2, &msg, dataLen + sizeof(msg.paramMap));
	m2->DebugPrint(paramTable);		//DEBUG
	CanSender::Send(buf);

	//TODO wait for reply
	reply.copy("CAN message reception not implemented");
	return GCodeResult::ok;
}

#endif

// End
