/*
 * OutputMemory.h
 *
 *  Created on: 10 Jan 2016
 *      Authors: David and Christian
 */

#ifndef OUTPUTMEMORY_H_
#define OUTPUTMEMORY_H_

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "Storage/FileData.h"

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
		bool HadOverflow() const { return hadOverflow; }
		void IncreaseReferences(size_t refs);

		const char *Data() const { return data; }
		const char *UnreadData() const { return data + bytesRead; }
		size_t DataLength() const { return dataLength; }	// How many bytes have been written to this instance?
		size_t Length() const;								// How many bytes have been written to the whole chain?

		char& operator[](size_t index);
		char operator[](size_t index) const;
		const char *Read(size_t len);
		void Taken(size_t len) { bytesRead += len; }
		size_t BytesLeft() const { return dataLength - bytesRead; }	// How many bytes have not been sent yet?

		size_t printf(const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
		size_t vprintf(const char *fmt, va_list vargs);
		size_t catf(const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));

		size_t copy(const char c);
		size_t copy(const char *src);
		size_t copy(const char *src, size_t len);

		size_t cat(const char c);
		size_t cat(const char *src);
		size_t cat(const char *src, size_t len);
		size_t cat(StringRef &str);

		size_t EncodeString(const char *src, bool allowControlChars, bool prependAsterisk = false);

		template<size_t Len> size_t EncodeString(const String<Len>& str, bool allowControlChars, bool prependAsterisk = false)
		{
			return EncodeString(str.c_str(), allowControlChars, prependAsterisk);
		}

		size_t EncodeReply(OutputBuffer *src);

		uint32_t GetAge() const;

		// Write the buffer to file returning true if successful
		bool WriteToFile(FileData& f) const;

		// Initialise the output buffers manager
		static void Init();

		// Allocate an unused OutputBuffer instance. Returns true on success or false if no instance could be allocated.
		static bool Allocate(OutputBuffer *&buf);

		// Get the number of bytes left for allocation. If writingBuffer is not NULL, this returns the number of free bytes for
		// continuous writes, i.e. for writes that need to allocate an extra OutputBuffer instance to finish the message.
		static size_t GetBytesLeft(const OutputBuffer *writingBuffer);

		// Truncate an OutputBuffer instance to free up more memory. Returns the number of released bytes.
		static size_t Truncate(OutputBuffer *buffer, size_t bytesNeeded);

		// Release one OutputBuffer instance. Returns the next item from the chain or nullptr if this was the last instance.
		static OutputBuffer *Release(OutputBuffer *buf);

		// Release all OutputBuffer objects in a chain
		static void ReleaseAll(OutputBuffer * volatile &buf);

		static void Diagnostics(MessageType mtype);

		static unsigned int GetFreeBuffers() { return OUTPUT_BUFFER_COUNT - usedOutputBuffers; }

	private:
		size_t EncodeChar(char c);

		OutputBuffer *next;
		OutputBuffer *last;

		uint32_t whenQueued;

		char data[OUTPUT_BUFFER_SIZE];
		size_t dataLength, bytesRead;

		bool isReferenced;
		bool hadOverflow;
		volatile size_t references;

		static OutputBuffer * volatile freeOutputBuffers;		// Messages may be sent by multiple tasks
		static volatile size_t usedOutputBuffers;				// so make these volatile.
		static volatile size_t maxUsedOutputBuffers;
};

inline uint32_t OutputBuffer::GetAge() const
{
	return millis() - whenQueued;
}

// This class is used to manage references to OutputBuffer chains for all output destinations.
// Note that OutputStack objects should normally be declared volatile.
class OutputStack
{
	public:
		OutputStack() : count(0) { }

		// Is there anything on this stack?
		bool IsEmpty() const volatile { return count == 0; }

		// Clear the reference list
		void Clear() volatile { count = 0; }

		// Push an OutputBuffer chain. Return true if successful, else release the buffer and return false.
		bool Push(OutputBuffer *buffer) volatile;

		// Pop an OutputBuffer chain or return NULL if none is available
		OutputBuffer *Pop() volatile;

		// Returns the first item from the stack or NULL if none is available
		OutputBuffer *GetFirstItem() const volatile;

		// Release the first item at the top of the stack
		void ReleaseFirstItem() volatile;

		// Apply a timeout to the first item at the top of the stack
		bool ApplyTimeout(uint32_t ticks) volatile;

		// Returns the last item from the stack or NULL if none is available
		OutputBuffer *GetLastItem() const volatile;

		// Get the total length of all queued buffers
		size_t DataLength() const volatile;

		// Append another OutputStack to this instance. If no more space is available,
		// all OutputBuffers that can't be added are automatically released
		void Append(volatile OutputStack& stack) volatile;

		// Increase the number of references for each OutputBuffer on the stack
		void IncreaseReferences(size_t num) volatile;

		// Release all buffers and clean up
		void ReleaseAll() volatile;

	private:
		size_t count;
		OutputBuffer * items[OUTPUT_STACK_DEPTH];
};

#endif /* OUTPUTMEMORY_H_ */
