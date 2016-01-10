/*
 * OutputBuffer.h
 *
 *  Created on: 10 Jan 2016
 *      Author: David
 */

#ifndef OUTPUTBUFFER_H_
#define OUTPUTBUFFER_H_

#include "Arduino.h"
#include "Configuration.h"
#include "StringRef.h"

// This class is used to hold data for sending (either for Serial or Network destinations)
class OutputBuffer
{
public:
	OutputBuffer(OutputBuffer *n) : next(n) { }

	OutputBuffer *Next() const { return next; }
	void Append(OutputBuffer *other);
	size_t References() const { return references; }
	void IncreaseReferences(size_t refs);

	const char *Data() const { return data; }
	uint16_t DataLength() const { return dataLength; }		// How many bytes have been written to this instance?
	uint32_t Length() const;								// How many bytes have been written to the whole chain?

	char& operator[](size_t index);
	char operator[](size_t index) const;
	const char *Read(uint16_t len);
	uint16_t BytesLeft() const { return bytesLeft; }		// How many bytes have not been sent yet?

	int printf(const char *fmt, ...);
	int vprintf(const char *fmt, va_list vargs);
	int catf(const char *fmt, ...);

	size_t copy(const char c);
	size_t copy(const char *src);
	size_t copy(const char *src, size_t len);

	size_t cat(const char c);
	size_t cat(const char *src);
	size_t cat(const char *src, size_t len);
	size_t cat(StringRef &str);

	size_t EncodeString(const char *src, uint16_t srcLength, bool allowControlChars, bool encapsulateString = true);
	size_t EncodeReply(OutputBuffer *src, bool allowControlChars);

	// Initialise the output buffers manager
	static void Init();

	// Allocate an unused OutputBuffer instance. Returns true on success or false if no instance could be allocated.
	// Setting isAppending to true will guarantee that one OutputBuffer will remain available for single allocation.
	static bool Allocate(OutputBuffer *&buf, bool isAppending = false);

	// Get the number of bytes left for allocation. If writingBuffer is not NULL, this returns the number of free bytes for
	// continuous writes, i.e. for writes that need to allocate an extra OutputBuffer instance to finish the message.
	static size_t GetBytesLeft(const OutputBuffer *writingBuffer);

	// Replace an existing OutputBuffer with another one.
	static void Replace(OutputBuffer *&destination, OutputBuffer *source);

	// Truncate an OutputBuffer instance to free up more memory. Returns the number of released bytes.
	static size_t Truncate(OutputBuffer *buffer, size_t bytesNeeded);

	// Release one OutputBuffer instance. Returns the next item from the chain or nullptr if this was the last instance.
	static OutputBuffer *Release(OutputBuffer *buf);

	// Release all OutputBuffer objects in a chain
	static void ReleaseAll(OutputBuffer *buf);

	static void Diagnostics();

private:
	OutputBuffer *next;

	char data[OUTPUT_BUFFER_SIZE];
	uint16_t dataLength, bytesLeft;

	size_t references;

	static OutputBuffer * volatile freeOutputBuffers;		// Messages may also be sent by ISRs,
	static volatile size_t usedOutputBuffers;				// so make these volatile.
	static volatile size_t maxUsedOutputBuffers;
};

#endif /* OUTPUTBUFFER_H_ */
