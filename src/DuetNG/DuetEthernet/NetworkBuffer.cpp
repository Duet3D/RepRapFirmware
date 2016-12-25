/*
 * NetworkBuffer.cpp
 *
 *  Created on: 24 Dec 2016
 *      Author: David
 */

#include "NetworkBuffer.h"
#include <cstring>

NetworkBuffer *NetworkBuffer::freelist = nullptr;

NetworkBuffer::NetworkBuffer(NetworkBuffer *n) : next(n), dataLength(0), readPointer(0)
{
}

// Release this buffer and return the next one in the chain
NetworkBuffer *NetworkBuffer::Release()
{
	NetworkBuffer *ret = next;
	next = freelist;
	freelist = this;
	return ret;
}

// Read 1 character, returning true if successful, false if no data left
bool NetworkBuffer::ReadChar(char& b)
{
	if (readPointer < dataLength)
	{
		b = data[readPointer++];
		return true;
	}

	b = 0;
	return false;
}

// Read some data, but not more than the amount in the first buffer
size_t NetworkBuffer::ReadBuffer(uint8_t *buffer, size_t maxLen)
{
	if (maxLen > Remaining())
	{
		maxLen = Remaining();
	}
	memcpy((void*)buffer, data + readPointer, maxLen);
	readPointer += maxLen;
	return maxLen;
}

// Return the amount of data available, including continuation buffers
size_t NetworkBuffer::TotalRemaining() const
{
	const NetworkBuffer *b = this;
	size_t ret = 0;
	while (b != nullptr)
	{
		ret += b->Remaining();
	}
	return ret;
}

/*static*/ NetworkBuffer *NetworkBuffer::Allocate()
{
	NetworkBuffer *ret = freelist;
	if (ret != nullptr)
	{
		freelist = ret->next;
		ret->next = nullptr;
	}
	return ret;
}

/*static*/ void NetworkBuffer::AllocateBuffers(unsigned int number)
{
	while (number != 0)
	{
		freelist = new NetworkBuffer(freelist);
		--number;
	}
}

// End
