/*
 * UniqueId.cpp
 *
 *  Created on: 4 Oct 2021
 *      Author: David
 */

#include "UniqueId.h"

// Append the unique ID in character form to an output buffer
void UniqueId::AppendCharsToBuffer(OutputBuffer *buf) const noexcept
{
	AppendCharsTo([buf](char c)-> void { buf->cat(c);});
}

// Generate a MAC address from this unique ID. Caller should check IsValid() first.
void UniqueId::GenerateMacAddress(MacAddress& addr) const noexcept
{
	// The unique ID is 128 bits long whereas the whole MAC address is only 48 bits, so we can't guarantee that each Duet will get a unique MAC address this way.
	// There are 16 ID bytes not including the checksum. It appears that the last bytes are more likely to change, so they must be included.
	memset(addr.bytes, 0, sizeof(addr.bytes));
	addr.bytes[0] = 0xBE;					// use a fixed first byte with the locally-administered bit set
	const uint8_t * const idBytes = reinterpret_cast<const uint8_t *>(data);
	for (size_t i = 0; i < 16; ++i)
	{
		addr.bytes[(i % 5) + 1] ^= idBytes[i];
	}
}

// End
