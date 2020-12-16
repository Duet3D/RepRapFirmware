/*
 * FileWriteBuffer.h
 *
 *  Created on: 19 May 2017
 *      Author: Christian
 */

#ifndef SRC_STORAGE_FILEWRITEBUFFER_H_
#define SRC_STORAGE_FILEWRITEBUFFER_H_

#include "RepRapFirmware.h"

#if SAM4E || SAM4S || SAME70 || SAME5x
const size_t NumFileWriteBuffers = 2;					// Number of write buffers
const size_t FileWriteBufLen = 8192;					// Size of each write buffer
#elif defined(__LPC17xx__)
# if HAS_WIFI_NETWORKING
const size_t NumFileWriteBuffers = 1;
const size_t FileWriteBufLen = 1024;
# else
const size_t NumFileWriteBuffers = 1;
const size_t FileWriteBufLen = 512;
# endif
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
#if SAME70
	FileWriteBuffer(FileWriteBuffer *n, char *storage) noexcept : next(n), index(0), buf(storage) { }
#else
	FileWriteBuffer(FileWriteBuffer *n) noexcept : next(n), index(0) { }
#endif

	FileWriteBuffer *Next() const noexcept { return next; }
	void SetNext(FileWriteBuffer *n) noexcept { next = n; }

	char *Data() noexcept { return buf; }
	const char *Data() const noexcept { return buf; }
	const size_t BytesStored() const noexcept { return index; }
	const size_t BytesLeft() const noexcept { return FileWriteBufLen - index; }

	size_t Store(const char *data, size_t length) noexcept;				// Stores some data and returns how much could be stored
	void DataTaken() noexcept { index = 0; }							// Called to indicate that the buffer has been written to the SD card
	void DataStored(size_t numBytes) noexcept { index += numBytes; }	// Called when more data has been stored directly in the buffer

private:
	FileWriteBuffer *next;

	size_t index;
#if SAME70
	char *buf;
#else
	alignas(4) char buf[FileWriteBufLen];								// 32-bit aligned buffer for better HSMCI performance
#endif
};

inline size_t FileWriteBuffer::Store(const char *data, size_t length) noexcept
{
	size_t bytesToStore = min<size_t>(BytesLeft(), length);
	memcpy(buf + index, data, bytesToStore);
	index += bytesToStore;
	return bytesToStore;
}

#endif
