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

void OutputBuffer::Append(OutputBuffer *other)
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

void OutputBuffer::IncreaseReferences(size_t refs)
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

size_t OutputBuffer::Length() const
{
	size_t totalLength = 0;
	for (const OutputBuffer *current = this; current != nullptr; current = current->Next())
	{
		totalLength += current->DataLength();
	}
	return totalLength;
}

char &OutputBuffer::operator[](size_t index)
{
	// Get the right buffer to access
	OutputBuffer *itemToIndex = this;
	while (index > itemToIndex->DataLength())
	{
		index -= itemToIndex->DataLength();
		itemToIndex = itemToIndex->Next();
	}

	// Return the char reference
	return itemToIndex->data[index];
}

char OutputBuffer::operator[](size_t index) const
{
	// Get the right buffer to access
	const OutputBuffer *itemToIndex = this;
	while (index > itemToIndex->DataLength())
	{
		index -= itemToIndex->DataLength();
		itemToIndex = itemToIndex->Next();
	}

	// Return the char reference
	return itemToIndex->data[index];
}

const char *OutputBuffer::Read(size_t len)
{
	size_t offset = bytesRead;
	bytesRead += len;
	return data + offset;
}

size_t OutputBuffer::printf(const char *fmt, ...)
{
	char formatBuffer[FormatStringLength];
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);

	return copy(formatBuffer);
}

size_t OutputBuffer::vprintf(const char *fmt, va_list vargs)
{
	char formatBuffer[FormatStringLength];
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);

	return cat(formatBuffer);
}

size_t OutputBuffer::catf(const char *fmt, ...)
{
	char formatBuffer[FormatStringLength];
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);

	formatBuffer[ARRAY_UPB(formatBuffer)] = 0;
	return cat(formatBuffer);
}

size_t OutputBuffer::copy(const char c)
{
	// Unlink existing entries before starting the copy process
	if (next != nullptr)
	{
		ReleaseAll(next);
		last = this;
	}

	// Set the data
	data[0] = c;
	dataLength = 1;
	return 1;
}

size_t OutputBuffer::copy(const char *src)
{
	return copy(src, strlen(src));
}

size_t OutputBuffer::copy(const char *src, size_t len)
{
	// Unlink existing entries before starting the copy process
	if (next != nullptr)
	{
		ReleaseAll(next);
		last = this;
	}

	dataLength = 0;
	return cat(src, len);
}

size_t OutputBuffer::cat(const char c)
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

size_t OutputBuffer::cat(const char *src)
{
	return cat(src, strlen(src));
}

size_t OutputBuffer::cat(const char *src, size_t len)
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

size_t OutputBuffer::cat(StringRef &str)
{
	return cat(str.c_str(), str.strlen());
}

// Encode a character in JSON format, and append it to the buffer and return the number of bytes written
size_t OutputBuffer::EncodeChar(char c)
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

// Encode a string in JSON format and append it to the buffer and return the number of bytes written
size_t OutputBuffer::EncodeString(const char *src, bool allowControlChars, bool prependAsterisk)
{
	size_t bytesWritten = cat('"');
	if (prependAsterisk)
	{
		bytesWritten += cat('*');
	}

	if (src != nullptr)
	{
		char c;
		while ((c = *src++) != 0 && (c >= ' ' || allowControlChars))
		{
			bytesWritten += EncodeChar(c);
		}
	}

	bytesWritten += cat('"');
	return bytesWritten;
}

size_t OutputBuffer::EncodeReply(OutputBuffer *src)
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

// Write all the data to file, but don't release the buffers
// Returns true if successful
bool OutputBuffer::WriteToFile(FileData& f) const
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

// Initialise the output buffers manager
/*static*/ void OutputBuffer::Init()
{
	freeOutputBuffers = nullptr;
	for (size_t i = 0; i < OUTPUT_BUFFER_COUNT; i++)
	{
		freeOutputBuffers = new OutputBuffer(freeOutputBuffers);
	}
}

// Allocates an output buffer instance which can be used for (large) string outputs. This must be thread safe. Not safe to call from interrupts!
/*static*/ bool OutputBuffer::Allocate(OutputBuffer *&buf)
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
			buf->whenQueued = millis();				// use the time of allocation as the default when-used time

			return true;
		}
	}

	reprap.GetPlatform().LogError(ErrorCode::OutputStarvation);
	return false;
}

// Get the number of bytes left for continuous writing
/*static*/ size_t OutputBuffer::GetBytesLeft(const OutputBuffer *writingBuffer)
{
	const size_t freeOutputBuffers = OUTPUT_BUFFER_COUNT - usedOutputBuffers;
	const size_t bytesLeft = OUTPUT_BUFFER_SIZE - writingBuffer->last->DataLength();

	if (freeOutputBuffers < RESERVED_OUTPUT_BUFFERS)
	{
		// Keep some space left to encapsulate the responses (e.g. via an HTTP header)
		return bytesLeft;
	}

	return bytesLeft + (freeOutputBuffers - RESERVED_OUTPUT_BUFFERS) * OUTPUT_BUFFER_SIZE;
}

// Truncate an output buffer to free up more memory. Returns the number of released bytes.
// This never releases the first buffer in the chain, so call it with a large value of bytesNeeded to release all buffers except the first.
/*static */ size_t OutputBuffer::Truncate(OutputBuffer *buffer, size_t bytesNeeded)
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
		previousItem->next = nullptr;
		Release(lastItem);
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
/*static */ OutputBuffer *OutputBuffer::Release(OutputBuffer *buf)
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

/*static */ void OutputBuffer::ReleaseAll(OutputBuffer * volatile &buf)
{
	while (buf != nullptr)
	{
		buf = Release(buf);
	}
}

/*static*/ void OutputBuffer::Diagnostics(MessageType mtype)
{
	reprap.GetPlatform().MessageF(mtype, "Used output buffers: %d of %d (%d max)\n",
			usedOutputBuffers, OUTPUT_BUFFER_COUNT, maxUsedOutputBuffers);
}

//*************************************************************************************************
// OutputStack class implementation

// Push an OutputBuffer chain. Return true if successful, else release the buffer and return false.
bool OutputStack::Push(OutputBuffer *buffer) volatile
{
	if (buffer != nullptr)
	{
		{
			TaskCriticalSectionLocker lock;

			if (count < OUTPUT_STACK_DEPTH)
			{
				buffer->whenQueued = millis();
				items[count++] = buffer;
				return true;
			}
		}
		OutputBuffer::ReleaseAll(buffer);
		reprap.GetPlatform().LogError(ErrorCode::OutputStackOverflow);
	}
	return false;
}

// Pop an OutputBuffer chain or return nullptr if none is available
OutputBuffer *OutputStack::Pop() volatile
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
	}
	count--;

	return item;
}

// Returns the first item from the stack or nullptr if none is available
OutputBuffer *OutputStack::GetFirstItem() const volatile
{
	return (count == 0) ? nullptr : items[0];
}

// Release the first item at the top of the stack
void OutputStack::ReleaseFirstItem() volatile
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
bool OutputStack::ApplyTimeout(uint32_t ticks) volatile
{
	bool ret = false;
	if (count != 0)
	{
		OutputBuffer * buf = items[0];							// capture volatile variable
		while (buf != nullptr && millis() - buf->whenQueued >= ticks)
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
OutputBuffer *OutputStack::GetLastItem() const volatile
{
	return (count == 0) ? nullptr : items[count - 1];
}

// Get the total length of all queued buffers
size_t OutputStack::DataLength() const volatile
{
	size_t totalLength = 0;

	TaskCriticalSectionLocker lock;
	for (size_t i = 0; i < count; i++)
	{
		totalLength += items[i]->Length();
	}

	return totalLength;
}

// Append another OutputStack to this instance. If no more space is available,
// all OutputBuffers that can't be added are automatically released
void OutputStack::Append(volatile OutputStack& stack) volatile
{
	for(size_t i = 0; i < stack.count; i++)
	{
		if (count < OUTPUT_STACK_DEPTH)
		{
			items[count++] = stack.items[i];
		}
		else
		{
			reprap.GetPlatform().LogError(ErrorCode::OutputStackOverflow);
			OutputBuffer::ReleaseAll(stack.items[i]);
		}
	}
}

// Increase the number of references for each OutputBuffer on the stack
void OutputStack::IncreaseReferences(size_t num) volatile
{
	TaskCriticalSectionLocker lock;
	for(size_t i = 0; i < count; i++)
	{
		items[i]->IncreaseReferences(num);
	}
}

// Release all buffers and clean up
void OutputStack::ReleaseAll() volatile
{
	for(size_t i = 0; i < count; i++)
	{
		OutputBuffer::ReleaseAll(items[i]);
	}
	count = 0;
}

// End
