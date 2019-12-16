/*
 * NetworkBuffer.cpp
 *
 *  Created on: 24 Dec 2016
 *      Author: David
 */

#include "NetworkBuffer.h"
#include "Storage/FileStore.h"

NetworkBuffer *NetworkBuffer::freelist = nullptr;

NetworkBuffer::NetworkBuffer(NetworkBuffer *n) noexcept : next(n), dataLength(0), readPointer(0)
{
}

// Release this buffer and return the next one in the chain
NetworkBuffer *NetworkBuffer::Release() noexcept
{
	NetworkBuffer *ret = next;
	next = freelist;
	freelist = this;
	return ret;
}

// Read 1 character, returning true if successful, false if no data left
bool NetworkBuffer::ReadChar(char& b) noexcept
{
	if (readPointer < dataLength)
	{
		b = Data()[readPointer++];
		return true;
	}

	b = 0;
	return false;
}

// Return the amount of data available, including continuation buffers
size_t NetworkBuffer::TotalRemaining() const noexcept
{
	const NetworkBuffer *b = this;
	size_t ret = 0;
	while (b != nullptr)
	{
		ret += b->Remaining();
		b = b->next;
	}
	return ret;
}

// Append some data, returning the amount appended
size_t NetworkBuffer::AppendData(const uint8_t *source, size_t length) noexcept
{
	if (length > SpaceLeft())
	{
		length = SpaceLeft();
	}
	memcpy(Data() + dataLength, source, length);
	dataLength += length;
	return length;
}

#if HAS_MASS_STORAGE

// Read into the buffer from a file returning the number of bytes read
int NetworkBuffer::ReadFromFile(FileStore *f) noexcept
{
	const int ret = f->Read(reinterpret_cast<char*>(data32), bufferSize);
	dataLength = (ret > 0) ? (size_t)ret : 0;
	readPointer = 0;
	return ret;
}

#endif

// Clear this buffer and release any successors
void NetworkBuffer::Empty() noexcept
{
	readPointer = dataLength = 0;
	while (next != nullptr)
	{
		next = next->Release();
	}
}

/*static*/ void NetworkBuffer::AppendToList(NetworkBuffer **list, NetworkBuffer *b) noexcept
{
	b->next = nullptr;
	while (*list != nullptr)
	{
		list = &((*list)->next);
	}
	*list = b;
}

// Find the last buffer in a list
/*static*/ NetworkBuffer *NetworkBuffer::FindLast(NetworkBuffer *list) noexcept
{
	if (list != nullptr)
	{
		while (list->next != nullptr)
		{
			list = list->next;
		}
	}
	return list;
}

/*static*/ NetworkBuffer *NetworkBuffer::Allocate() noexcept
{
	NetworkBuffer *ret = freelist;
	if (ret != nullptr)
	{
		freelist = ret->next;
		ret->next = nullptr;
		ret->dataLength = ret->readPointer = 0;
	}
	return ret;
}

/*static*/ void NetworkBuffer::AllocateBuffers(unsigned int number) noexcept
{
	while (number != 0)
	{
		freelist = new NetworkBuffer(freelist);
		--number;
	}
}

// Count how many buffers there are in a chain
/*static*/ unsigned int NetworkBuffer::Count(NetworkBuffer*& ptr) noexcept
{
	unsigned int ret = 0;
	for (NetworkBuffer *n = ptr; n != nullptr; n = n->next)
	{
		++ret;
	}
	return ret;
}

// End
