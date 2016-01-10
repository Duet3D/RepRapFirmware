/*
 * OuputBuffer.cpp
 *
 *  Created on: 10 Jan 2016
 *      Author: David
 */

#include "OutputBuffer.h"
#include "RepRapFirmware.h"
#include <cstdarg>

/*static*/ OutputBuffer * volatile OutputBuffer::freeOutputBuffers = nullptr;		// Messages may also be sent by ISRs,
/*static*/ volatile size_t OutputBuffer::usedOutputBuffers = 0;				// so make these volatile.
/*static*/ volatile size_t OutputBuffer::maxUsedOutputBuffers = 0;

//*************************************************************************************************
// OutputBuffer class implementation

void OutputBuffer::Append(OutputBuffer *other)
{
	if (other != nullptr)
	{
		OutputBuffer *lastBuffer = this;
		while (lastBuffer->next != nullptr)
		{
			lastBuffer = lastBuffer->next;
		}
		lastBuffer->next = other;
	}
}

void OutputBuffer::IncreaseReferences(size_t refs)
{
	references += refs;
	if (next != nullptr)
	{
		next->IncreaseReferences(refs);
	}
}

uint32_t OutputBuffer::Length() const
{
	uint32_t totalLength = 0;
	const OutputBuffer *current = this;
	do {
		totalLength += current->DataLength();
		current = current->next;
	} while (current != nullptr);
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

const char *OutputBuffer::Read(uint16_t len)
{
	size_t offset = dataLength - bytesLeft;
	bytesLeft -= len;
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
	data[0] = c;
	dataLength = bytesLeft = 1;
	return 1;
}

size_t OutputBuffer::copy(const char *src)
{
	return copy(src, strlen(src));
}

size_t OutputBuffer::copy(const char *src, size_t len)
{
	// Unlink other entries before starting the copy process
	OutputBuffer *nextBuffer = next;
	while (nextBuffer != nullptr)
	{
		nextBuffer = Release(nextBuffer);
	}

	// Does the whole string fit into this instance?
	if (len > OUTPUT_BUFFER_SIZE)
	{
		// No - copy what we can't write into a new chain
		OutputBuffer *currentBuffer, *lastBuffer = nullptr;
		size_t bytesCopied = OUTPUT_BUFFER_SIZE;
		do {
			if (!Allocate(currentBuffer, true))
			{
				// We cannot store the whole string. Should never happen
				break;
			}
			currentBuffer->references = references;

			const size_t copyLength = min<size_t>(OUTPUT_BUFFER_SIZE, len - bytesCopied);
			memcpy(currentBuffer->data, src + bytesCopied, copyLength);
			currentBuffer->dataLength = currentBuffer->bytesLeft = copyLength;
			bytesCopied += copyLength;

			if (next == nullptr)
			{
				next = lastBuffer = currentBuffer;
			}
			else
			{
				lastBuffer->next = currentBuffer;
				lastBuffer = currentBuffer;
			}
		} while (bytesCopied < len);

		// Then copy the rest into this instance
		memcpy(data, src, OUTPUT_BUFFER_SIZE);
		dataLength = bytesLeft = OUTPUT_BUFFER_SIZE;
		next = nextBuffer;
		return bytesCopied;
	}

	// Yes - no need to use a new item
	memcpy(data, src, len);
	dataLength = bytesLeft = len;
	return len;
}

size_t OutputBuffer::cat(const char c)
{
	// Get the last entry from the chain
	OutputBuffer *lastBuffer = this;
	while (lastBuffer->next != nullptr)
	{
		lastBuffer = lastBuffer->next;
	}

	// See if we can append a char
	if (lastBuffer->dataLength == OUTPUT_BUFFER_SIZE)
	{
		// No - allocate a new item and link it
		OutputBuffer *nextBuffer;
		if (!Allocate(nextBuffer, true))
		{
			// We cannot store any more data. Should never happen
			return 0;
		}
		nextBuffer->references = references;
		nextBuffer->copy(c);

		lastBuffer->next = nextBuffer;
	}
	else
	{
		// Yes - we have enough space left
		lastBuffer->data[lastBuffer->dataLength++] = c;
		lastBuffer->bytesLeft++;
	}
	return 1;
}

size_t OutputBuffer::cat(const char *src)
{
	return cat(src, strlen(src));
}

size_t OutputBuffer::cat(const char *src, size_t len)
{
	// Get the last entry from the chain
	OutputBuffer *lastBuffer = this;
	while (lastBuffer->next != nullptr)
	{
		lastBuffer = lastBuffer->next;
	}

	// Do we need to use an extra buffer?
	if (lastBuffer->dataLength + len > OUTPUT_BUFFER_SIZE)
	{
		size_t copyLength = OUTPUT_BUFFER_SIZE - lastBuffer->dataLength;
		size_t bytesCopied = copyLength;
		bytesCopied = copyLength;

		// Yes - copy what we can't write into a new chain
		OutputBuffer *nextBuffer;
		if (!Allocate(nextBuffer, true))
		{
			// We cannot store any more data. Should never happen
			return 0;
		}
		nextBuffer->references = references;
		bytesCopied += nextBuffer->copy(src + copyLength, len - copyLength);
		lastBuffer->next = nextBuffer;

		// Then copy the rest into this one
		memcpy(lastBuffer->data + lastBuffer->dataLength, src, copyLength);
		lastBuffer->dataLength += copyLength;
		lastBuffer->bytesLeft += copyLength;
		return bytesCopied;
	}

	// No - reuse this one instead
	memcpy(lastBuffer->data + lastBuffer->dataLength, src, len);
	lastBuffer->dataLength += len;
	lastBuffer->bytesLeft += len;
	return len;
}

size_t OutputBuffer::cat(StringRef &str)
{
	return cat(str.Pointer(), str.Length());
}

// Encode a string in JSON format and append it to a string buffer and return the number of bytes written
size_t OutputBuffer::EncodeString(const char *src, uint16_t srcLength, bool allowControlChars, bool encapsulateString)
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
	buf->dataLength = buf->bytesLeft = 0;
	buf->references = 1; // Assume it's only used once by default

	cpu_irq_restore(flags);

	return true;
}

// Get the number of bytes left for continuous writing
/*static*/ size_t OutputBuffer::GetBytesLeft(const OutputBuffer *writingBuffer)
{
	// If writingBuffer is NULL, just return how much space there is left for everything
	if (writingBuffer == nullptr)
	{
		return (OUTPUT_BUFFER_COUNT - usedOutputBuffers) * OUTPUT_BUFFER_SIZE;
	}

	// Otherwise get the last entry from the chain
	const OutputBuffer *lastBuffer = writingBuffer;
	while (lastBuffer->Next() != nullptr)
	{
		lastBuffer = lastBuffer->Next();
	}

	// Do we have any more buffers left for writing?
	if (usedOutputBuffers >= OUTPUT_BUFFER_COUNT)
	{
		// No - refer to this one only
		return OUTPUT_BUFFER_SIZE - lastBuffer->DataLength();
	}

	// Yes - we know how many buffers are in use, so there is no need to work through the free list
	return (OUTPUT_BUFFER_SIZE - lastBuffer->DataLength() + (OUTPUT_BUFFER_COUNT - usedOutputBuffers - 1) * OUTPUT_BUFFER_SIZE);
}


// Truncate an output buffer to free up more memory. Returns the number of released bytes.
/*static */ size_t OutputBuffer::Truncate(OutputBuffer *buffer, size_t bytesNeeded)
{
	// Can we free up space from this entry?
	if (buffer == nullptr || buffer->Next() == nullptr)
	{
		// No
		return 0;
	}

	// Yes - free up the last entry (entries) from this chain
	size_t releasedBytes = OUTPUT_BUFFER_SIZE;
	OutputBuffer *previousItem;
	do {
		// Get the last entry from the chain
		previousItem = buffer;
		OutputBuffer *lastItem = previousItem->Next();
		while (lastItem->Next() != nullptr)
		{
			previousItem = lastItem;
			lastItem = lastItem->Next();
		}

		// Unlink and free it
		previousItem->next = nullptr;
		Release(lastItem);
		releasedBytes += OUTPUT_BUFFER_SIZE;
	} while (previousItem != buffer && releasedBytes < bytesNeeded);

	return releasedBytes;
}

/*static*/ void OutputBuffer::Replace(OutputBuffer *&destination, OutputBuffer *source)
{
	OutputBuffer *temp = destination;
	while (temp != nullptr)
	{
		temp = Release(temp);
	}

	destination = source;
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
		buf->bytesLeft = buf->dataLength;
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

// End





