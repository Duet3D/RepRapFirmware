/*
 * OutputMemory.cpp
 *
 *  Created on: 10 Jan 2016
 *      Authors: David and Christian
 */

#include "OutputMemory.h"
#include "RepRapFirmware.h"
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
		for(OutputBuffer *item = Next(); item != other; item = item->Next())
		{
			item->last = last;
		}
	}
}

void OutputBuffer::IncreaseReferences(size_t refs)
{
	if (refs > 0)
	{
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
	for(const OutputBuffer *current = this; current != nullptr; current = current->Next())
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

int OutputBuffer::printf(const char *fmt, ...)
{
	char formatBuffer[FORMAT_STRING_LENGTH];
	va_list vargs;
	va_start(vargs, fmt);
	int ret = vsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);

	copy(formatBuffer);
	return ret;
}

int OutputBuffer::vprintf(const char *fmt, va_list vargs)
{
	char formatBuffer[FORMAT_STRING_LENGTH];
	int res = vsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);

	cat(formatBuffer);
	return res;
}

int OutputBuffer::catf(const char *fmt, ...)
{
	char formatBuffer[FORMAT_STRING_LENGTH];
	va_list vargs;
	va_start(vargs, fmt);
	int ret = vsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);

	cat(formatBuffer);
	return ret;
}

size_t OutputBuffer::copy(const char c)
{
	// Unlink existing entries before starting the copy process
	if (next != nullptr)
	{
		ReleaseAll(next);
		next = nullptr;
		last = this;
	}

	// Set the date
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
		next = nullptr;
		last = this;
	}

	// Does the whole string fit into this instance?
	if (len > OUTPUT_BUFFER_SIZE)
	{
		// No - copy what we can't write into a new chain
		OutputBuffer *currentBuffer;
		size_t bytesCopied = OUTPUT_BUFFER_SIZE;
		do {
			if (!Allocate(currentBuffer, true))
			{
				// We cannot store the whole string, stop here
				break;
			}
			currentBuffer->references = references;

			// Fill up the next instance
			const size_t copyLength = min<size_t>(OUTPUT_BUFFER_SIZE, len - bytesCopied);
			memcpy(currentBuffer->data, src + bytesCopied, copyLength);
			currentBuffer->dataLength = copyLength;
			bytesCopied += copyLength;

			// Link it to the chain
			if (next == nullptr)
			{
				next = last = currentBuffer;
			}
			else
			{
				last->next = currentBuffer;
				last = currentBuffer;
			}
		} while (bytesCopied < len);

		// Update references to the last entry for all items
		for(OutputBuffer *item = Next(); item != last; item = item->Next())
		{
			item->last = last;
		}

		// Then copy the rest into this instance
		memcpy(data, src, OUTPUT_BUFFER_SIZE);
		dataLength = OUTPUT_BUFFER_SIZE;
		return bytesCopied;
	}

	// Yes, the whole string fits into this instance. No need to allocate a new item
	memcpy(data, src, len);
	dataLength = len;
	return len;
}

size_t OutputBuffer::cat(const char c)
{
	// See if we can append a char
	if (last->dataLength == OUTPUT_BUFFER_SIZE)
	{
		// No - allocate a new item and copy the data
		OutputBuffer *nextBuffer;
		if (!Allocate(nextBuffer, true))
		{
			// We cannot store any more data. Should never happen
			return 0;
		}
		nextBuffer->references = references;
		nextBuffer->copy(c);

		// Link the new item to this list
		last->next = nextBuffer;
		for(OutputBuffer *item = this; item != nextBuffer; item = item->Next())
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
	// Copy what we can into the last buffer
	size_t copyLength = min<size_t>(len, OUTPUT_BUFFER_SIZE - last->dataLength);
	memcpy(last->data + last->dataLength, src, copyLength);
	last->dataLength += copyLength;

	// Is there any more data left?
	if (len > copyLength)
	{
		// Yes - copy what we couldn't write into a new chain
		OutputBuffer *nextBuffer;
		if (!Allocate(nextBuffer, true))
		{
			// We cannot store any more data, stop here
			return copyLength;
		}
		nextBuffer->references = references;
		const size_t bytesCopied = copyLength + nextBuffer->copy(src + copyLength, len - copyLength);

		// Done - now append the new entry to the chain
		last->next = nextBuffer;
		last = nextBuffer->last;
		for(OutputBuffer *item = Next(); item != nextBuffer; item = item->Next())
		{
			item->last = last;
		}
		return bytesCopied;
	}

	// No more data had to be written, we could store everything
	return len;
}

size_t OutputBuffer::cat(StringRef &str)
{
	return cat(str.Pointer(), str.Length());
}

// Encode a string in JSON format and append it to a string buffer and return the number of bytes written
size_t OutputBuffer::EncodeString(const char *src, size_t srcLength, bool allowControlChars, bool encapsulateString)
{
	size_t bytesWritten = 0;
	if (encapsulateString)
	{
		bytesWritten += cat('"');
	}

	size_t srcPointer = 1;
	char c = *src++;
	while (srcPointer <= srcLength && c != 0 && (c >= ' ' || allowControlChars))
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
				esc = '"';
				break;
			case '\\':
				esc = '\\';
				break;
			default:
				esc = 0;
				break;
		}

		if (esc != 0)
		{
			bytesWritten += cat('\\');
			bytesWritten += cat(esc);
		}
		else
		{
			bytesWritten += cat(c);
		}

		c = *src++;
		srcPointer++;
	}

	if (encapsulateString)
	{
		bytesWritten += cat('"');
	}
	return bytesWritten;
}

size_t OutputBuffer::EncodeReply(OutputBuffer *src, bool allowControlChars)
{
	size_t bytesWritten = cat('"');

	while (src != nullptr)
	{
		bytesWritten += EncodeString(src->Data(), src->DataLength(), allowControlChars, false);
		src = Release(src);
	}

	bytesWritten += cat('"');
	return bytesWritten;
}

// Initialise the output buffers manager
/*static*/ void OutputBuffer::Init()
{
	freeOutputBuffers = nullptr;
	for(size_t i = 0; i < OUTPUT_BUFFER_COUNT; i++)
	{
		freeOutputBuffers = new OutputBuffer(freeOutputBuffers);
	}
}

// Allocates an output buffer instance which can be used for (large) string outputs
/*static*/ bool OutputBuffer::Allocate(OutputBuffer *&buf, bool isAppending)
{
	const irqflags_t flags = cpu_irq_save();

	if (freeOutputBuffers == nullptr)
	{
		reprap.GetPlatform()->RecordError(ErrorCode::OutputStarvation);
		cpu_irq_restore(flags);

		buf = nullptr;
		return false;
	}
	else if (isAppending)
	{
		// It's a good idea to leave at least one OutputBuffer available if we're
		// writing a large chunk of data...
		if (freeOutputBuffers->next == nullptr)
		{
			cpu_irq_restore(flags);

			buf = nullptr;
			return false;
		}
	}

	buf = freeOutputBuffers;
	freeOutputBuffers = buf->next;

	usedOutputBuffers++;
	if (usedOutputBuffers > maxUsedOutputBuffers)
	{
		maxUsedOutputBuffers = usedOutputBuffers;
	}

	buf->next = nullptr;
	buf->last = buf;
	buf->dataLength = buf->bytesRead = 0;
	buf->references = 1; // Assume it's only used once by default
	buf->isReferenced = false;

	cpu_irq_restore(flags);
	return true;
}

// Get the number of bytes left for continuous writing
/*static*/ size_t OutputBuffer::GetBytesLeft(const OutputBuffer *writingBuffer)
{
	// If writingBuffer is NULL, just return how much space there is left for continuous writing
	if (writingBuffer == nullptr)
	{
		if (usedOutputBuffers == OUTPUT_BUFFER_COUNT)
		{
			// No more instances can be allocated
			return 0;
		}

		return (OUTPUT_BUFFER_COUNT - usedOutputBuffers - 1) * OUTPUT_BUFFER_SIZE;
	}

	// Do we have any more buffers left for writing?
	if (usedOutputBuffers == OUTPUT_BUFFER_COUNT)
	{
		// No - refer to this one only
		return OUTPUT_BUFFER_SIZE - writingBuffer->last->DataLength();
	}

	// Yes - we know how many buffers are in use, so there is no need to work through the free list
	return (OUTPUT_BUFFER_SIZE - writingBuffer->last->DataLength() + (OUTPUT_BUFFER_COUNT - usedOutputBuffers - 1) * OUTPUT_BUFFER_SIZE);
}


// Truncate an output buffer to free up more memory. Returns the number of released bytes.
/*static */ size_t OutputBuffer::Truncate(OutputBuffer *buffer, size_t bytesNeeded )
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
	for(OutputBuffer *item = buffer; item != nullptr; item = item->Next())
	{
		item->last = previousItem;
	}
	return releasedBytes;
}

// Releases an output buffer instance and returns the next entry from the chain
/*static */ OutputBuffer *OutputBuffer::Release(OutputBuffer *buf)
{
	const irqflags_t flags = cpu_irq_save();
	OutputBuffer *nextBuffer = buf->next;

	// If this one is reused by another piece of code, don't free it up
	if (buf->references > 1)
	{
		buf->references--;
		buf->bytesRead = 0;
		cpu_irq_restore(flags);
		return nextBuffer;
	}

	// Otherwise prepend it to the list of free output buffers again
	buf->next = freeOutputBuffers;
	freeOutputBuffers = buf;
	usedOutputBuffers--;

	cpu_irq_restore(flags);
	return nextBuffer;
}

/*static */ void OutputBuffer::ReleaseAll(OutputBuffer *buf)
{
	while (buf != nullptr)
	{
		buf = Release(buf);
	}
}

/*static*/ void OutputBuffer::Diagnostics()
{
	reprap.GetPlatform()->MessageF(GENERIC_MESSAGE, "Used output buffers: %d of %d (%d max)\n",
			usedOutputBuffers, OUTPUT_BUFFER_COUNT, maxUsedOutputBuffers);
}

//*************************************************************************************************
// OutputStack class implementation

// Push an OutputBuffer chain to the stack
void OutputStack::Push(OutputBuffer *buffer)
{
	if (count == OUTPUT_STACK_DEPTH)
	{
		OutputBuffer::ReleaseAll(buffer);
		reprap.GetPlatform()->RecordError(ErrorCode::OutputStackOverflow);
		return;
	}

	if (buffer != nullptr)
	{
		buffer->whenQueued = millis();
		const irqflags_t flags = cpu_irq_save();
		items[count++] = buffer;
		cpu_irq_restore(flags);
	}
}

// Pop an OutputBuffer chain or return NULL if none is available
OutputBuffer *OutputStack::Pop()
{
	if (count == 0)
	{
		return nullptr;
	}

	const irqflags_t flags = cpu_irq_save();
	OutputBuffer *item = items[0];
	for(size_t i = 1; i < count; i++)
	{
		items[i - 1] = items[i];
	}
	count--;
	cpu_irq_restore(flags);

	return item;
}

// Returns the first item from the stack or NULL if none is available
OutputBuffer *OutputStack::GetFirstItem() const
{
	if (count == 0)
	{
		return nullptr;
	}
	return items[0];
}

// Set the first item of the stack. If it's NULL, then the first item will be removed
void OutputStack::SetFirstItem(OutputBuffer *buffer)
{
	const irqflags_t flags = cpu_irq_save();
	if (buffer == nullptr)
	{
		// If buffer is NULL, then the first item is removed from the stack
		for(size_t i = 1; i < count; i++)
		{
			items[i - 1] = items[i];
		}
		count--;
	}
	else
	{
		// Else only the first item is updated
		items[0] = buffer;
		buffer->whenQueued = millis();
	}
	cpu_irq_restore(flags);
}

// Returns the last item from the stack or NULL if none is available
OutputBuffer *OutputStack::GetLastItem() const
{
	if (count == 0)
	{
		return nullptr;
	}
	return items[count - 1];
}

// Get the total length of all queued buffers
size_t OutputStack::DataLength() const
{
	size_t totalLength = 0;

	const irqflags_t flags = cpu_irq_save();
	for(size_t i = 0; i < count; i++)
	{
		totalLength += items[i]->Length();
	}
	cpu_irq_restore(flags);

	return totalLength;
}

// Append another OutputStack to this instance. If no more space is available,
// all OutputBuffers that can't be added are automatically released
void OutputStack::Append(OutputStack *stack)
{
	for(size_t i = 0; i < stack->count; i++)
	{
		if (count < OUTPUT_STACK_DEPTH)
		{
			items[count++] = stack->items[i];
		}
		else
		{
			reprap.GetPlatform()->RecordError(ErrorCode::OutputStackOverflow);
			OutputBuffer::ReleaseAll(stack->items[i]);
		}
	}
}

// Increase the number of references for each OutputBuffer on the stack
void OutputStack::IncreaseReferences(size_t num)
{
	const irqflags_t flags = cpu_irq_save();
	for(size_t i = 0; i < count; i++)
	{
		items[i]->IncreaseReferences(num);
	}
	cpu_irq_restore(flags);
}

// Release all buffers and clean up
void OutputStack::ReleaseAll()
{
	for(size_t i = 0; i < count; i++)
	{
		OutputBuffer::ReleaseAll(items[i]);
	}
	count = 0;
}

// vim: ts=4:sw=4
