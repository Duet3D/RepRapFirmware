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
class RTOSPlusTCPEthernetSocket;

// Network buffer class. These buffers are 2K long so that they can accept as much data as the W5500 can provide in one go.
class NetworkBuffer final
{
public:
	friend class WiFiSocket;
	friend class W5500Socket;
	friend class RTOSPlusTCPEthernetSocket;

	// Release this buffer and return the next one in the chain
	NetworkBuffer *_ecv_null Release() noexcept;

	// Read 1 character, returning true of successful, false if no data left
	bool ReadChar(char& b) noexcept;

	const uint8_t *_ecv_array UnreadData() const noexcept { return data + readPointer; }

	// Return the amount of data available, not including continuation buffers
	size_t Remaining() const noexcept { return dataLength - readPointer; }

	// Return the amount of data available, including continuation buffers
	size_t TotalRemaining() const noexcept;

	// Return true if there no data left to read
	bool IsEmpty() const noexcept { return readPointer == dataLength; }

	// Mark some data as taken
	void Taken(size_t amount) noexcept { readPointer += amount; }

	// Return the length available for writing
	size_t SpaceLeft() const noexcept { return bufferSize - dataLength; }

	// Return a pointer to the space available for writing
	uint8_t *_ecv_array UnwrittenData() noexcept { return data + dataLength; }

	// Append some data, returning the amount appended
	size_t AppendData(const uint8_t *_ecv_array source, size_t length) noexcept;

#if HAS_MASS_STORAGE
	// Read into the buffer from a file
	int ReadFromFile(FileStore *f) noexcept;
#endif

	// Clear this buffer and release any successors
	void Empty() noexcept;

	// Append a buffer to a list
	static void AppendToList(NetworkBuffer *_ecv_null &r_list, NetworkBuffer *b) noexcept;

	// Find the last buffer in a list
	static NetworkBuffer *_ecv_null FindLast(NetworkBuffer *_ecv_null list) noexcept;

	// Allocate a buffer
	static NetworkBuffer *_ecv_null Allocate() noexcept;

	// Allocate buffers and put them in the freelist
	static void AllocateBuffers(unsigned int number) noexcept;

	// Count how many buffers there are in a chain
	static unsigned int Count(NetworkBuffer *_ecv_null & ptr) noexcept;

	static const size_t bufferSize = 2 * 1024;

private:
	explicit NetworkBuffer(NetworkBuffer *_ecv_null n) noexcept;

	// Direct access to data for friend classes
	uint8_t *_ecv_array Data() noexcept { return data; }
	const uint8_t *_ecv_array Data() const noexcept { return data; }

	NetworkBuffer *_ecv_null next;
	size_t dataLength;
	size_t readPointer;
	// When doing unaligned transfers on the WiFi interface, up to 3 extra bytes may be returned, hence the +4 in the following
	alignas(4) uint8_t data[bufferSize + 4];		// 32-bit aligned buffer so we can do direct DMA
	static NetworkBuffer *_ecv_null freelist;
};

#endif /* SRC_NETWORKING_NETWORKBUFFER_H_ */
