/*
 * OutputMemory.cpp
 *
 *  Created on: 10 Jan 2016
 *      Authors: David and Christian
 */

#include "OutputMemory.h"
#include "Platform.h"
#include "RepRap.h"
#include <cstdarg>

/*static*/ OutputBuffer * volatile OutputBuffer::freeOutputBuffers = nullptr;		// Messages may also be sent by ISRs,
/*static*/ volatile size_t OutputBuffer::usedOutputBuffers = 0;						// so make these volatile.
/*static*/ volatile size_t OutputBuffer::maxUsedOutputBuffers = 0;

//*************************************************************************************************
// OutputBuffer class implementation

void OutputBuffer::Append(OutputBuffer *other) noexcept
{
	if (other != nullptr)
	{
		last->next = other;
		last = other->last;
		if (other->hadOverflow)
		{
			hadOverflow = true;
		}

		for (OutputBuffer *item = Next(); item != other; item = item->Next())
		{
			item->last = last;
		}
	}
}

void OutputBuffer::IncreaseReferences(size_t refs) noexcept
{
	if (refs > 0)
	{
		TaskCriticalSectionLocker lock;

		for(OutputBuffer *item = this; item != nullptr; item = item->Next())
		{
			item->references += refs;
			item->isReferenced = true;
		}
	}
}

size_t OutputBuffer::Length() const noexcept
{
	size_t totalLength = 0;
	for (const OutputBuffer *current = this; current != nullptr; current = current->Next())
	{
		totalLength += current->DataLength();
	}
	return totalLength;
}

char &OutputBuffer::operator[](size_t index) noexcept
{
	// Get the right buffer to access
	OutputBuffer *itemToIndex = this;
	while (index >= itemToIndex->DataLength())
	{
		index -= itemToIndex->DataLength();
		itemToIndex = itemToIndex->Next();
	}

	// Return the char reference
	return itemToIndex->data[index];
}

char OutputBuffer::operator[](size_t index) const noexcept
{
	// Get the right buffer to access
	const OutputBuffer *itemToIndex = this;
	while (index >= itemToIndex->DataLength())
	{
		index -= itemToIndex->DataLength();
		itemToIndex = itemToIndex->Next();
	}

	// Return the char reference
	return itemToIndex->data[index];
}

const char *OutputBuffer::Read(size_t len) noexcept
{
	size_t offset = bytesRead;
	bytesRead += len;
	return data + offset;
}

// Empty this buffer
void OutputBuffer::Clear() noexcept
{
	if (next != nullptr)
	{
		ReleaseAll(next);
		last = this;
	}
	dataLength = 0;
}

void OutputBuffer::UpdateWhenQueued() noexcept
{
	whenQueued = millis();
}

size_t OutputBuffer::vprintf(const char *fmt, va_list vargs) noexcept
{
	Clear();
	return vcatf(fmt, vargs);
}

size_t OutputBuffer::printf(const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	size_t ret = vprintf(fmt, vargs);
	va_end(vargs);
	return ret;
}

size_t OutputBuffer::vcatf(const char *fmt, va_list vargs) noexcept
{
	return vuprintf([this](char c) noexcept -> bool
					{
						return c != 0 && cat(c) != 0;
					},
					fmt, vargs);
}

size_t OutputBuffer::catf(const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	size_t ret = vcatf(fmt, vargs);
	va_end(vargs);
	return ret;
}

size_t OutputBuffer::lcatf(const char *fmt, ...) noexcept
{
	size_t extra = 0;
	if (Length() != 0 && operator[](Length() - 1) != '\n')
	{
		extra = cat('\n');
	}

	va_list vargs;
	va_start(vargs, fmt);
	const size_t ret = vcatf(fmt, vargs);
	va_end(vargs);
	return ret + extra;
}

size_t OutputBuffer::copy(const char c) noexcept
{
	Clear();
	data[0] = c;
	dataLength = 1;
	return 1;
}

size_t OutputBuffer::copy(const char *src) noexcept
{
	return copy(src, strlen(src));
}

size_t OutputBuffer::copy(const char *src, size_t len) noexcept
{
	Clear();
	return cat(src, len);
}

size_t OutputBuffer::cat(const char c) noexcept
{
	// See if we can append a char
	if (last->dataLength == OUTPUT_BUFFER_SIZE)
	{
		// No - allocate a new item and copy the data
		OutputBuffer *nextBuffer;
		if (!Allocate(nextBuffer))
		{
			// We cannot store any more data
			hadOverflow = true;
			return 0;
		}
		nextBuffer->references = references;
		nextBuffer->copy(c);

		// Link the new item to this list
		last->next = nextBuffer;
		for (OutputBuffer *item = this; item != nextBuffer; item = item->Next())
		{
			item->last = nextBuffer;
		}
	}
	else
	{
		// Yes - we have enough space left
		last->data[last->dataLength++] = c;
	}
	return 1;
}

size_t OutputBuffer::cat(const char *src) noexcept
{
	return cat(src, strlen(src));
}

size_t OutputBuffer::lcat(const char *src) noexcept
{
	return lcat(src, strlen(src));
}

size_t OutputBuffer::cat(const char *src, size_t len) noexcept
{
	size_t copied = 0;
	while (copied < len)
	{
		if (last->dataLength == OUTPUT_BUFFER_SIZE)
		{
			// The last buffer is full
			OutputBuffer *nextBuffer;
			if (!Allocate(nextBuffer))
			{
				// We cannot store any more data, stop here
				hadOverflow = true;
				break;
			}
			nextBuffer->references = references;
			last->next = nextBuffer;
			last = nextBuffer->last;
			for (OutputBuffer *item = Next(); item != nextBuffer; item = item->Next())
			{
				item->last = last;
			}
		}
		const size_t copyLength = min<size_t>(len - copied, OUTPUT_BUFFER_SIZE - last->dataLength);
		memcpy(last->data + last->dataLength, src + copied, copyLength);
		last->dataLength += copyLength;
		copied += copyLength;
	}
	return copied;
}

size_t OutputBuffer::lcat(const char *src, size_t len) noexcept
{
	if (Length() != 0)
	{
		cat('\n');
	}
	return cat(src, len);
}

size_t OutputBuffer::cat(StringRef &str) noexcept
{
	return cat(str.c_str(), str.strlen());
}

// Encode a character in JSON format, and append it to the buffer and return the number of bytes written
size_t OutputBuffer::EncodeChar(char c) noexcept
{
	char esc;
	switch (c)
	{
	case '\r':
		esc = 'r';
		break;
	case '\n':
		esc = 'n';
		break;
	case '\t':
		esc = 't';
		break;
	case '"':
	case '\\':
#if 1
	// Escaping '/' is optional in JSON, although doing so so confuses PanelDue (fixed in PanelDue firmware version 1.15 and later). As it's optional, we don't do it.
#else
	case '/':
#endif
		esc = c;
		break;
	default:
		esc = 0;
		break;
	}

	if (esc != 0)
	{
		cat('\\');
		cat(esc);
		return 2;
	}

	cat(c);
	return 1;
}

size_t OutputBuffer::EncodeReply(OutputBuffer *src) noexcept
{
	size_t bytesWritten = cat('"');

	while (src != nullptr)
	{
		for (size_t index = 0; index < src->DataLength(); ++index)
		{
			bytesWritten += EncodeChar(src->Data()[index]);
		}
		src = Release(src);
	}

	bytesWritten += cat('"');
	return bytesWritten;
}

#if HAS_MASS_STORAGE

// Write all the data to file, but don't release the buffers
// Returns true if successful
bool OutputBuffer::WriteToFile(FileData& f) const noexcept
{
	bool endedInNewline = false;
	const OutputBuffer *current = this;
	do
	{
		if (current->dataLength != 0)
		{
			if (!f.Write(current->data, current->dataLength))
			{
				return false;
			}
			endedInNewline = current->data[current->dataLength - 1] == '\n';
		}
		current = current->Next();
	} while (current != nullptr);

	if (!endedInNewline)
	{
		return f.Write('\n');
	}
	return true;
}

#endif

// Initialise the output buffers manager
/*static*/ void OutputBuffer::Init() noexcept
{
	freeOutputBuffers = nullptr;
	for (size_t i = 0; i < OUTPUT_BUFFER_COUNT; i++)
	{
		freeOutputBuffers = new OutputBuffer(freeOutputBuffers);
	}
}

// Allocates an output buffer instance which can be used for (large) string outputs. This must be thread safe. Not safe to call from interrupts!
/*static*/ bool OutputBuffer::Allocate(OutputBuffer *&buf) noexcept
{
	{
		TaskCriticalSectionLocker lock;

		buf = freeOutputBuffers;
		if (buf != nullptr)
		{
			freeOutputBuffers = buf->next;
			usedOutputBuffers++;
			if (usedOutputBuffers > maxUsedOutputBuffers)
			{
				maxUsedOutputBuffers = usedOutputBuffers;
			}

			// Initialise the buffer before we release the lock in case another task uses it immediately
			buf->next = nullptr;
			buf->last = buf;
			buf->dataLength = buf->bytesRead = 0;
			buf->references = 1;					// assume it's only used once by default
			buf->isReferenced = false;
			buf->hadOverflow = false;
			buf->UpdateWhenQueued();				// use the time of allocation as the default when-used time

			return true;
		}
	}

	reprap.GetPlatform().LogError(ErrorCode::OutputStarvation);
	return false;
}

// Get the number of bytes left for continuous writing
/*static*/ size_t OutputBuffer::GetBytesLeft(const OutputBuffer *writingBuffer) noexcept
{
	const size_t freeBuffers = OUTPUT_BUFFER_COUNT - usedOutputBuffers;
	const size_t bytesLeft = OUTPUT_BUFFER_SIZE - writingBuffer->last->DataLength();

	if (freeBuffers < RESERVED_OUTPUT_BUFFERS)
	{
		// Keep some space left to encapsulate the responses (e.g. via an HTTP header)
		return bytesLeft;
	}

	return bytesLeft + (freeBuffers - RESERVED_OUTPUT_BUFFERS) * OUTPUT_BUFFER_SIZE;
}

// Truncate an output buffer to free up more memory. Returns the number of released bytes.
// This never releases the first buffer in the chain, so call it with a large value of bytesNeeded to release all buffers except the first.
/*static */ size_t OutputBuffer::Truncate(OutputBuffer *buffer, size_t bytesNeeded) noexcept
{
	// Can we free up space from this chain? Don't break it up if it's referenced anywhere else
	if (buffer == nullptr || buffer->Next() == nullptr || buffer->IsReferenced())
	{
		// No
		return 0;
	}

	// Yes - free up the last entries
	size_t releasedBytes = 0;
	OutputBuffer *previousItem;
	do {
		// Get two the last entries from the chain
		previousItem = buffer;
		OutputBuffer *lastItem = previousItem->Next();
		while (lastItem->Next() != nullptr)
		{
			previousItem = lastItem;
			lastItem = lastItem->Next();
		}

		// Unlink and free the last entry
		ReleaseAll(previousItem->next);
		releasedBytes += OUTPUT_BUFFER_SIZE;
	} while (previousItem != buffer && releasedBytes < bytesNeeded);

	// Update all the references to the last item
	for (OutputBuffer *item = buffer; item != nullptr; item = item->Next())
	{
		item->last = previousItem;
	}
	return releasedBytes;
}

// Releases an output buffer instance and returns the next entry from the chain
/*static */ OutputBuffer *OutputBuffer::Release(OutputBuffer *buf) noexcept
{
	TaskCriticalSectionLocker lock;
	OutputBuffer * const nextBuffer = buf->next;

	// If this one is reused by another piece of code, don't free it up
	if (buf->references > 1)
	{
		buf->references--;
		buf->bytesRead = 0;
	}
	else
	{
		// Otherwise prepend it to the list of free output buffers again
		buf->next = freeOutputBuffers;
		freeOutputBuffers = buf;
		usedOutputBuffers--;
	}
	return nextBuffer;
}

/*static */ void OutputBuffer::ReleaseAll(OutputBuffer * volatile &buf) noexcept
{
	while (buf != nullptr)
	{
		buf = Release(buf);
	}
}

/*static*/ void OutputBuffer::Diagnostics(MessageType mtype) noexcept
{
	reprap.GetPlatform().MessageF(mtype, "Used output buffers: %d of %d (%d max)\n",
			usedOutputBuffers, OUTPUT_BUFFER_COUNT, maxUsedOutputBuffers);
}

//*************************************************************************************************
// OutputStack class implementation

// Push an OutputBuffer chain. Return true if successful, else release the buffer and return false.
bool OutputStack::Push(OutputBuffer *buffer, MessageType type) volatile noexcept
{
	{
		TaskCriticalSectionLocker lock;

		if (count < OUTPUT_STACK_DEPTH)
		{
			if (buffer != nullptr)
			{
				buffer->UpdateWhenQueued();
			}
			items[count] = buffer;
			types[count] = type;
			count++;
			return true;
		}
	}
	OutputBuffer::ReleaseAll(buffer);
	reprap.GetPlatform().LogError(ErrorCode::OutputStackOverflow);
	return false;
}

// Pop an OutputBuffer chain or return nullptr if none is available
OutputBuffer *OutputStack::Pop() volatile noexcept
{
	TaskCriticalSectionLocker lock;

	if (count == 0)
	{
		return nullptr;
	}

	OutputBuffer *item = items[0];
	for (size_t i = 1; i < count; i++)
	{
		items[i - 1] = items[i];
		types[i - 1] = types[i];
	}
	count--;

	return item;
}

// Returns the first item from the stack or nullptr if none is available
OutputBuffer *OutputStack::GetFirstItem() const volatile noexcept
{
	return (count == 0) ? nullptr : items[0];
}

// Returns the first item's type from the stack or NoDestinationMessage if none is available
MessageType OutputStack::GetFirstItemType() const volatile noexcept
{
	return (count == 0) ? MessageType::NoDestinationMessage : types[0];
}

#if HAS_SBC_INTERFACE

// Update the first item of the stack
void OutputStack::SetFirstItem(OutputBuffer *buffer) volatile noexcept
{
	if (count != 0)
	{
		if (buffer == nullptr)
		{
			(void)Pop();
		}
		else
		{
			items[0] = buffer;
			buffer->UpdateWhenQueued();
		}
	}
}

#endif

// Release the first item at the top of the stack
void OutputStack::ReleaseFirstItem() volatile noexcept
{
	if (count != 0)
	{
		OutputBuffer * const buf = items[0];					// capture volatile variable
		if (buf != nullptr)
		{
			items[0] = OutputBuffer::Release(buf);
		}
		if (items[0] == nullptr)
		{
			(void)Pop();
		}
	}
}

// Release the first item on the top of the stack if it is too old. Return true if the item was timed out or was null.
bool OutputStack::ApplyTimeout(uint32_t ticks) volatile noexcept
{
	bool ret = false;
	if (count != 0)
	{
		OutputBuffer * buf = items[0];							// capture volatile variable
		while (buf != nullptr && millis() - buf->WhenQueued() >= ticks)
		{
			items[0] = buf = OutputBuffer::Release(buf);
			ret = true;
		}
		if (items[0] == nullptr)
		{
			(void)Pop();
			ret = true;
		}
	}
	return ret;
}

// Returns the last item from the stack or nullptr if none is available
OutputBuffer *OutputStack::GetLastItem() const volatile noexcept
{
	return (count == 0) ? nullptr : items[count - 1];
}

// Returns the type of the last item from the stack or NoDestinationMessage if none is available
MessageType OutputStack::GetLastItemType() const volatile noexcept
{
	return (count == 0) ? MessageType::NoDestinationMessage : types[count - 1];
}

// Get the total length of all queued buffers
size_t OutputStack::DataLength() const volatile noexcept
{
	size_t totalLength = 0;

	TaskCriticalSectionLocker lock;
	for (size_t i = 0; i < count; i++)
	{
		if (items[i] != nullptr)
		{
			totalLength += items[i]->Length();
		}
	}

	return totalLength;
}

// Append another OutputStack to this instance. If no more space is available,
// all OutputBuffers that can't be added are automatically released
void OutputStack::Append(volatile OutputStack& stack) volatile noexcept
{
	for (size_t i = 0; i < stack.count; i++)
	{
		if (count < OUTPUT_STACK_DEPTH)
		{
			items[count] = stack.items[i];
			types[count] = stack.types[i];
			count++;
		}
		else
		{
			reprap.GetPlatform().LogError(ErrorCode::OutputStackOverflow);
			OutputBuffer::ReleaseAll(stack.items[i]);
		}
	}
}

// Increase the number of references for each OutputBuffer on the stack
void OutputStack::IncreaseReferences(size_t num) volatile noexcept
{
	TaskCriticalSectionLocker lock;
	for (size_t i = 0; i < count; i++)
	{
		if (items[i] != nullptr)
		{
			items[i]->IncreaseReferences(num);
		}
	}
}

// Release all buffers and clean up
void OutputStack::ReleaseAll() volatile noexcept
{
	for (size_t i = 0; i < count; i++)
	{
		OutputBuffer::ReleaseAll(items[i]);
	}
	count = 0;
}

// End
