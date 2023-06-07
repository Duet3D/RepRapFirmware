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
constexpr size_t NumFileWriteBuffers = 2;					// Number of write buffers
constexpr size_t FileWriteBufLen = 8192;					// Size of each write buffer
constexpr size_t SbcFileWriteBufLen = 4192;					// Available size of each write buffer in SBC mode
#else
# error Unsupported processor
#endif
static_assert(FileWriteBufLen >= SbcFileWriteBufLen, "File write buffer must be at least as big as the configured SBC threshold");

// Class to cache data that is about to be written to the SD card. This is NOT a ring buffer,
// instead it just provides simple interfaces to cache a certain amount of data so that fewer
// f_write() calls are needed. This effectively improves upload speeds.
class FileWriteBuffer
{
public:
#if SAME70
	FileWriteBuffer(FileWriteBuffer *n, char *storage) noexcept : next(n), index(0), buf(storage) { }
#else
	explicit FileWriteBuffer(FileWriteBuffer *n) noexcept : next(n), index(0) { }
#endif
	static void UsingSbcMode() { fileWriteBufLen = SbcFileWriteBufLen; }	// only called by RepRap on startup

	FileWriteBuffer *Next() const noexcept { return next; }
	void SetNext(FileWriteBuffer *n) noexcept { next = n; }

	char *_ecv_array Data() noexcept { return buf; }
	const char *_ecv_array Data() const noexcept { return buf; }
	const size_t BytesStored() const noexcept { return index; }
	const size_t BytesLeft() const noexcept { return fileWriteBufLen - index; }

	size_t Store(const char *data, size_t length) noexcept;				// Stores some data and returns how much could be stored
	void DataTaken() noexcept { index = 0; }							// Called to indicate that the buffer has been written to the SD card
	void DataStored(size_t numBytes) noexcept { index += numBytes; }	// Called when more data has been stored directly in the buffer

private:
	static size_t fileWriteBufLen;
	FileWriteBuffer *next;

	size_t index;
#if SAME70
	char *_ecv_array buf;
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
