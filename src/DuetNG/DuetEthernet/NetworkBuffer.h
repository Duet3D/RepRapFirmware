/*
 * NetworkBuffer.h
 *
 *  Created on: 24 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_NETWORKBUFFER_H_
#define SRC_DUETNG_DUETETHERNET_NETWORKBUFFER_H_

#include "RepRapFirmware.h"
#include "NetworkDefs.h"

// Network buffer class. These buffers are 2K long so that they can accept as much data as the W5500 can provide in one go.
class NetworkBuffer
{
public:
	friend class Socket;

	// Release this buffer and return the next one in the chain
	NetworkBuffer *Release();

	// Read 1 character, returning true of successful, false if no data left
	bool ReadChar(char& b);

	// Read some data
	const uint8_t* TakeData(size_t &len);

	// Return the amount of data available, not including continuation buffers
	size_t Remaining() const { return dataLength - readPointer; }

	// Return the amount of data available, including continuation buffers
	size_t TotalRemaining() const;

	// Return true if there no data left to read
	bool IsEmpty() const { return readPointer == dataLength; }

	// Return the length available for writing
	size_t SpaceLeft() const { return bufferSize - dataLength; }

	// Append some data, returning the amount appended
	size_t AppendData(const uint8_t *source, size_t length);

	// Read into the buffer from a file
	int ReadFromFile(FileStore *f);

	// Clear this buffer and release any successors
	void Empty();

	// Reset the data pointer to the start of the buffer
	void ResetPointer() { readPointer = 0; }

	// Append a buffer to a list
	static void AppendToList(NetworkBuffer **list, NetworkBuffer *b);

	// Allocate a buffer
	static NetworkBuffer *Allocate();

	// Alocate buffers and put them in the freelist
	static void AllocateBuffers(unsigned int number);

	static const size_t bufferSize =
#ifdef USE_3K_BUFFERS
									 3 * 1024;
#else
									 2 * 1024;
#endif

private:
	NetworkBuffer(NetworkBuffer *n);
	uint8_t *Data() { return reinterpret_cast<uint8_t*>(data32); }

	NetworkBuffer *next;
	size_t dataLength;
	size_t readPointer;
	uint32_t data32[bufferSize/sizeof(uint32_t)];			// 32-bit aligned buffer so we can do direct DMA

	static NetworkBuffer *freelist;
};

#endif /* SRC_DUETNG_DUETETHERNET_NETWORKBUFFER_H_ */
