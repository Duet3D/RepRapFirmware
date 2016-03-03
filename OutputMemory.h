/*
 * OutputMemory.h
 *
 *  Created on: 10 Jan 2016
 *      Authors: David and Christian
 */

#ifndef OUTPUTMEMORY_H_
#define OUTPUTMEMORY_H_

#include "Arduino.h"
#include "Configuration.h"
#include "StringRef.h"

const size_t OUTPUT_STACK_DEPTH = 4;	// Number of OutputBuffer chains that can be pushed onto one stack instance

class OutputStack;

// This class is used to hold data for sending (either for Serial or Network destinations)
class OutputBuffer
{
	public:
		friend class OutputStack;

		OutputBuffer(OutputBuffer *n) : next(n) { }

		void Append(OutputBuffer *other);
		OutputBuffer *Next() const { return next; }
		bool IsReferenced() const { return isReferenced; }
		void IncreaseReferences(size_t refs);

		const char *Data() const { return data; }
		size_t DataLength() const { return dataLength; }	// How many bytes have been written to this instance?
		size_t Length() const;								// How many bytes have been written to the whole chain?

		char& operator[](size_t index);
		char operator[](size_t index) const;
		const char *Read(size_t len);
		size_t BytesLeft() const { return dataLength - bytesRead; }	// How many bytes have not been sent yet?

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

		size_t EncodeString(const char *src, size_t srcLength, bool allowControlChars, bool encapsulateString = true);
		size_t EncodeReply(OutputBuffer *src, bool allowControlChars);

		uint32_t GetAge() const;

		// Initialise the output buffers manager
		static void Init();

		// Allocate an unused OutputBuffer instance. Returns true on success or false if no instance could be allocated.
		// Setting isAppending to true will guarantee that one OutputBuffer will remain available for single allocation.
		static bool Allocate(OutputBuffer *&buf, bool isAppending = false);

		// Get the number of bytes left for allocation. If writingBuffer is not NULL, this returns the number of free bytes for
		// continuous writes, i.e. for writes that need to allocate an extra OutputBuffer instance to finish the message.
		static size_t GetBytesLeft(const OutputBuffer *writingBuffer);

		// Truncate an OutputBuffer instance to free up more memory. Returns the number of released bytes.
		static size_t Truncate(OutputBuffer *buffer, size_t bytesNeeded);

		// Release one OutputBuffer instance. Returns the next item from the chain or nullptr if this was the last instance.
		static OutputBuffer *Release(OutputBuffer *buf);

		// Release all OutputBuffer objects in a chain
		static void ReleaseAll(OutputBuffer *buf);

		static void Diagnostics();

	private:

		OutputBuffer *next;
		OutputBuffer *last;

		uint32_t whenQueued;

		char data[OUTPUT_BUFFER_SIZE];
		size_t dataLength, bytesRead;

		bool isReferenced;
		size_t references;

		static OutputBuffer * volatile freeOutputBuffers;		// Messages may also be sent by ISRs,
		static volatile size_t usedOutputBuffers;				// so make these volatile.
		static volatile size_t maxUsedOutputBuffers;
};

inline uint32_t OutputBuffer::GetAge() const
{
	return millis() - whenQueued;
}

// This class is used to manage references to OutputBuffer chains for all output destinations
class OutputStack
{
	public:
		OutputStack() : count(0) { }

		// Is there anything on this stack?
		bool IsEmpty() const { return count == 0; }

		// Clear the reference list
		void Clear() { count = 0; }

		// Push an OutputBuffer chain
		void Push(OutputBuffer *buffer);

		// Pop an OutputBuffer chain or return NULL if none is available
		OutputBuffer *Pop();

		// Returns the first item from the stack or NULL if none is available
		OutputBuffer *GetFirstItem() const;

		// Set the first item of the stack. If it's NULL, then the first item will be removed
		void SetFirstItem(OutputBuffer *buffer);

		// Returns the last item from the stack or NULL if none is available
		OutputBuffer *GetLastItem() const;

		// Get the total length of all queued buffers
		size_t DataLength() const;

		// Append another OutputStack to this instance. If no more space is available,
		// all OutputBuffers that can't be added are automatically released
		void Append(OutputStack *stack);

		// Increase the number of references for each OutputBuffer on the stack
		void IncreaseReferences(size_t num);

		// Release all buffers and clean up
		void ReleaseAll();

	private:
		volatile size_t count;
		OutputBuffer * volatile items[OUTPUT_STACK_DEPTH];
};

#endif /* OUTPUTMEMORY_H_ */

// vim: ts=4:sw=4
