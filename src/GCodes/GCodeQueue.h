/*
 * GCodeQueue.h
 *
 *  Created on: 22 Jun 2016
 *      Author: Christian
 */
#ifndef GCODEQUEUE_H
#define GCODEQUEUE_H

#include "RepRapFirmware.h"
#include "GCodeInput.h"

class QueuedCode;

const size_t BufferSizePerQueueItem = ShortGCodeLength;

class GCodeQueue : public GCodeInput
{
public:
	GCodeQueue() noexcept;

	void Reset() noexcept override;										// Clean all the cached data from this input
	bool FillBuffer(GCodeBuffer *gb) noexcept override;					// If there is another move to execute at this time, fill a buffer
	size_t BytesCached() const noexcept override;						// How many bytes have been cached?

	bool QueueCode(const GCodeBuffer &gb) noexcept;						// Queue a G-code
	void PurgeEntries() noexcept;										// Remove stored codes when a print is being paused
	void Clear() noexcept;												// Clean up all the stored codes
	bool IsIdle() const noexcept;										// Return true if there is nothing to do

	void Diagnostics(const StringRef& reply) noexcept;

	static bool ShouldQueueG10(GCodeBuffer &gb, ParameterLettersBitmap allAxisLetters) noexcept;					// Return true if this code should be queued
	static bool ShouldQueueMCode(GCodeBuffer &gb) THROWS(GCodeException);	// Return true if this code should be queued

private:
	QueuedCode *_ecv_null freeItems;
	QueuedCode *_ecv_null queuedItems;
};

class QueuedCode
{
public:
	friend class GCodeQueue;

	explicit QueuedCode(QueuedCode *_ecv_null n) noexcept : next(n), dataLength(0) { }
	QueuedCode *_ecv_null Next() const noexcept { return next; }

private:
	QueuedCode *_ecv_null next;

#if HAS_SBC_INTERFACE
	bool isBinary;
	alignas(4) char data[BufferSizePerQueueItem];
#else
	char data[BufferSizePerQueueItem];
#endif
	size_t dataLength;

	uint32_t executeAtMove;

	void AssignFrom(const GCodeBuffer &gb) noexcept;
	void AssignTo(GCodeBuffer *gb) noexcept;
};

#endif
