/*
 * FileWriteBuffer.h
 *
 *  Created on: 19 May 2017
 *      Author: Christian
 */

#ifndef SRC_STORAGE_FILEWRITEBUFFER_H_
#define SRC_STORAGE_FILEWRITEBUFFER_H_

#include "RepRapFirmware.h"

#if SAM4E || SAM4S || SAME70
const size_t NumFileWriteBuffers = 2;					// Number of write buffers
const size_t FileWriteBufLen = 8192;					// Size of each write buffer
#elif __LPC17xx__
const size_t NumFileWriteBuffers = 1;
const size_t FileWriteBufLen = 2*256; //4096; save some memory on LPC for networking
#else
const size_t NumFileWriteBuffers = 1;
const size_t FileWriteBufLen = 4096;
#endif

// Class to cache data that is about to be written to the SD card. This is NOT a ring buffer,
// instead it just provides simple interfaces to cache a certain amount of data so that fewer
// f_write() calls are needed. This effectively improves upload speeds.
class FileWriteBuffer
{
public:
	FileWriteBuffer(FileWriteBuffer *n) : next(n), index(0) { }

	FileWriteBuffer *Next() const { return next; }
	void SetNext(FileWriteBuffer *n) { next = n; }

	char *Data() { return reinterpret_cast<char *>(data32); }
	const char *Data() const { return reinterpret_cast<const char *>(data32); }
	const size_t BytesStored() const { return index; }
	const size_t BytesLeft() const { return FileWriteBufLen - index; }

	size_t Store(const char *data, size_t length);			// Stores some data and returns how much could be stored
	void DataTaken() { index = 0; }							// Called to indicate that the buffer has been written to the SD card
	void DataStored(size_t numBytes) { index += numBytes; }	// Called when more data has been stored directly in the buffer

private:
	FileWriteBuffer *next;

	size_t index;
	uint32_t data32[FileWriteBufLen / sizeof(uint32_t)];	// 32-bit aligned buffer for better HSMCI performance
};

inline size_t FileWriteBuffer::Store(const char *data, size_t length)
{
	size_t bytesToStore = min<size_t>(BytesLeft(), length);
	memcpy(Data() + index, data, bytesToStore);
	index += bytesToStore;
	return bytesToStore;
}

#endif
