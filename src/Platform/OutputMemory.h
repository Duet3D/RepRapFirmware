/*
 * OutputMemory.h
 *
 *  Created on: 10 Jan 2016
 *      Authors: David and Christian
 */

#ifndef OUTPUTMEMORY_H_
#define OUTPUTMEMORY_H_

#include <RepRapFirmware.h>
#include <Storage/FileData.h>

#if HAS_SBC_INTERFACE
const size_t OUTPUT_STACK_DEPTH = 64;	// Number of OutputBuffer chains that can be pushed onto one stack instance
#else
const size_t OUTPUT_STACK_DEPTH = 4;	// Number of OutputBuffer chains that can be pushed onto one stack instance
#endif

// This class is used to hold data for sending (either for Serial or Network destinations)
class OutputBuffer
{
public:
	explicit OutputBuffer(OutputBuffer *_ecv_null n) noexcept : next(n) { }
	OutputBuffer(const OutputBuffer&) = delete;

	void Append(OutputBuffer *_ecv_null other) noexcept;
	OutputBuffer *null Next() const noexcept { return next; }
	bool IsReferenced() const noexcept { return isReferenced; }
	bool HadOverflow() const noexcept { return hadOverflow; }
	void IncreaseReferences(size_t refs) noexcept;

	const char *_ecv_array Data() const noexcept { return data; }
	const char *_ecv_array UnreadData() const noexcept { return data + bytesRead; }
	size_t DataLength() const noexcept { return dataLength; }	// How many bytes have been written to this instance?
	size_t Length() const noexcept;								// How many bytes have been written to the whole chain?

	char operator[](size_t index) const noexcept;
	const char *_ecv_array Read(size_t len) noexcept;
	void Taken(size_t len) noexcept { bytesRead += len; }
	size_t BytesLeft() const noexcept { return dataLength - bytesRead; }	// How many bytes have not been sent yet?

	uint32_t WhenQueued() const noexcept { return whenQueued; }
	void UpdateWhenQueued() noexcept;

	size_t vprintf(const char *_ecv_array fmt, va_list vargs) noexcept;
	size_t printf(const char *_ecv_array fmt, ...) noexcept __attribute__ ((format (printf, 2, 3)));
	size_t vcatf(const char *_ecv_array fmt, va_list vargs) noexcept;
	size_t catf(const char *_ecv_array fmt, ...) noexcept __attribute__ ((format (printf, 2, 3)));
	size_t lcatf(const char *_ecv_array fmt, ...) noexcept __attribute__ ((format (printf, 2, 3)));

	size_t copy(const char c) noexcept;
	size_t copy(const char *_ecv_array src) noexcept;
	size_t copy(const char *_ecv_array src, size_t len) noexcept;

	size_t cat(const char c) noexcept;
	size_t cat(const char *_ecv_array src) noexcept;
	size_t lcat(const char *_ecv_array src) noexcept;
	size_t cat(const char *_ecv_array src, size_t len) noexcept;
	size_t lcat(const char *_ecv_array src, size_t len) noexcept;
	size_t cat(StringRef &str) noexcept;

	size_t EncodeChar(char c) noexcept;
	size_t EncodeReply(OutputBuffer *_ecv_null src) noexcept;

	uint32_t GetAge() const noexcept;

#if HAS_MASS_STORAGE
	// Write the buffer to file returning true if successful
	bool WriteToFile(FileData& f) const noexcept;
#endif
	// Initialise the output buffers manager
	static void Init() noexcept;

	// Allocate an unused OutputBuffer instance. Returns true on success or false if no instance could be allocated.
	static bool Allocate(OutputBuffer *_ecv_null &buf) noexcept;

	// Get the number of bytes left for allocation. If writingBuffer is not NULL, this returns the number of free bytes for
	// continuous writes, i.e. for writes that need to allocate an extra OutputBuffer instance to finish the message.
	static size_t GetBytesLeft(const OutputBuffer *writingBuffer) noexcept;

	// Truncate an OutputBuffer instance to free up more memory. Returns the number of released bytes.
	static size_t Truncate(OutputBuffer *_ecv_null buffer, size_t bytesNeeded) noexcept;

	// Release one OutputBuffer instance. Returns the next item from the chain or nullptr if this was the last instance.
	__attribute__((warn_unused_result)) static OutputBuffer *Release(OutputBuffer *buf) noexcept;

	// Release all OutputBuffer objects in a chain
	static void ReleaseAll(OutputBuffer *_ecv_null volatile &buf) noexcept;

	static void Diagnostics(MessageType mtype) noexcept;

	static unsigned int GetFreeBuffers() noexcept { return OUTPUT_BUFFER_COUNT - usedOutputBuffers; }

private:
	void Clear() noexcept;

	OutputBuffer *_ecv_null next;
	OutputBuffer *last;

	uint32_t whenQueued;									// milliseconds timer when this buffer was filled in

	char data[OUTPUT_BUFFER_SIZE];
	size_t dataLength, bytesRead;

	bool isReferenced;
	bool hadOverflow;
	volatile size_t references;

	static OutputBuffer *_ecv_null volatile freeOutputBuffers;		// Messages may be sent by multiple tasks
	static volatile size_t usedOutputBuffers;				// so make these volatile.
	static volatile size_t maxUsedOutputBuffers;
};

inline uint32_t OutputBuffer::GetAge() const noexcept
{
	return millis() - whenQueued;
}

// This class is used to manage references to OutputBuffer chains for all output destinations.
// Note that OutputStack objects should normally be declared volatile.
class OutputStack
{
public:
	OutputStack() noexcept : count(0) { }
	OutputStack(const OutputStack&) = delete;

	// Is there anything on this stack?
	bool IsEmpty() const volatile noexcept { return count == 0; }

	// Clear the reference list
	void Clear() volatile noexcept { count = 0; }

	// Push an OutputBuffer chain. Return true if successful, else release the buffer and return false.
	bool Push(OutputBuffer *_ecv_null buffer, MessageType type = NoDestinationMessage) volatile noexcept;

	// Pop an OutputBuffer chain or return NULL if none is available
	OutputBuffer *Pop() volatile noexcept;

	// Returns the first item from the stack or NULL if none is available
	OutputBuffer *GetFirstItem() const volatile noexcept;

	// Returns the first item's type from the stack or NoDestinationMessage if none is available
	MessageType GetFirstItemType() const volatile noexcept;

#if HAS_SBC_INTERFACE
	// Set the first item of the stack. If it's NULL, then the first item will be removed
	void SetFirstItem(OutputBuffer *buffer) volatile noexcept;
#endif
	// Release the first item at the top of the stack
	void ReleaseFirstItem() volatile noexcept;

	// Apply a timeout to the first item at the top of the stack
	bool ApplyTimeout(uint32_t ticks) volatile noexcept;

	// Returns the last item from the stack or NULL if none is available
	OutputBuffer *_ecv_null GetLastItem() const volatile noexcept;

	// Returns the type of the last item from the stack or NoDestinationMessage if none is available
	MessageType GetLastItemType() const volatile noexcept;

	// Get the total length of all queued buffers
	size_t DataLength() const volatile noexcept;

	// Append another OutputStack to this instance. If no more space is available,
	// all OutputBuffers that can't be added are automatically released
	void Append(volatile OutputStack& stack) volatile noexcept;

	// Increase the number of references for each OutputBuffer on the stack
	void IncreaseReferences(size_t num) volatile noexcept;

	// Release all buffers and clean up
	void ReleaseAll() volatile noexcept;

private:
	size_t count;
	OutputBuffer *_ecv_null items[OUTPUT_STACK_DEPTH];
	MessageType types[OUTPUT_STACK_DEPTH];
};

#endif /* OUTPUTMEMORY_H_ */
