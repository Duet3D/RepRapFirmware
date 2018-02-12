/*
 * NetworkBuffer.h
 *
 *  Created on: 24 Dec 2016
 *      Author: David
 */

#ifndef SRC_NETWORKING_NETWORKBUFFER_H_
#define SRC_NETWORKING_NETWORKBUFFER_H_

#include "RepRapFirmware.h"
#include "NetworkDefs.h"

class WiFiSocket;
class W5500Socket;

// Network buffer class. These buffers are 2K long so that they can accept as much data as the W5500 can provide in one go.
class NetworkBuffer
{
public:
	friend class WiFiSocket;
	friend class W5500Socket;

	// Release this buffer and return the next one in the chain
	NetworkBuffer *Release();

	// Read 1 character, returning true of successful, false if no data left
	bool ReadChar(char& b);

	const uint8_t* UnreadData() const { return Data() + readPointer; }

	// Return the amount of data available, not including continuation buffers
	size_t Remaining() const { return dataLength - readPointer; }

	// Return the amount of data available, including continuation buffers
	size_t TotalRemaining() const;

	// Return true if there no data left to read
	bool IsEmpty() const { return readPointer == dataLength; }

	// Mark some data as taken
	void Taken(size_t amount) { readPointer += amount; }

	// Return the length available for writing
	size_t SpaceLeft() const { return bufferSize - dataLength; }

	// Return a pointer to the space available for writing
	uint8_t* UnwrittenData() { return Data() + dataLength; }

	// Append some data, returning the amount appended
	size_t AppendData(const uint8_t *source, size_t length);

	// Read into the buffer from a file
	int ReadFromFile(FileStore *f);

	// Clear this buffer and release any successors
	void Empty();

	// Append a buffer to a list
	static void AppendToList(NetworkBuffer **list, NetworkBuffer *b);

	// Find the last buffer in a list
	static NetworkBuffer *FindLast(NetworkBuffer *list);

	// Allocate a buffer
	static NetworkBuffer *Allocate();

	// Alocate buffers and put them in the freelist
	static void AllocateBuffers(unsigned int number);

	// Count how many buffers there are in a chain
	static unsigned int Count(NetworkBuffer*& ptr);

	static const size_t bufferSize =
#ifdef USE_3K_BUFFERS
									 3 * 1024;
#else
									 2 * 1024;
#endif

private:
	NetworkBuffer(NetworkBuffer *n);
	uint8_t *Data() { return reinterpret_cast<uint8_t*>(data32); }
	const uint8_t *Data() const { return reinterpret_cast<const uint8_t*>(data32); }

	NetworkBuffer *next;
	size_t dataLength;
	size_t readPointer;
	// When doing unaligned transfers on the WiFi interface, up to 3 extra bytes may be returned, hence the +1 in the following
	uint32_t data32[bufferSize/sizeof(uint32_t) + 1];		// 32-bit aligned buffer so we can do direct DMA
	static NetworkBuffer *freelist;
};

#endif /* SRC_NETWORKING_NETWORKBUFFER_H_ */
