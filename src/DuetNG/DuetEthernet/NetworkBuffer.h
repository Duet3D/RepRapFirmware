/*
 * NetworkBuffer.h
 *
 *  Created on: 24 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_NETWORKBUFFER_H_
#define SRC_DUETNG_DUETETHERNET_NETWORKBUFFER_H_

#include <cstdint>
#include <cstddef>

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
	size_t ReadBuffer(uint8_t *buffer, size_t maxLen);

	// Return the amount of data available, not including continuation buffers
	size_t Remaining() const { return dataLength - readPointer; }

	// Return the amount of data available, including continuation buffers
	size_t TotalRemaining() const;

	bool IsEmpty() const { return readPointer == dataLength; }

	static NetworkBuffer *Allocate();

	static void AllocateBuffers(unsigned int number);

	static const size_t bufferSize = 2048;

private:
	NetworkBuffer(NetworkBuffer *n);

	NetworkBuffer *next;
	size_t dataLength;
	size_t readPointer;
	uint8_t data[bufferSize];

	static NetworkBuffer *freelist;
};

#endif /* SRC_DUETNG_DUETETHERNET_NETWORKBUFFER_H_ */
