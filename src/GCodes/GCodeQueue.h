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

	void Diagnostics(MessageType mtype, unsigned int queueNumber) noexcept;

	static bool ShouldQueueG10(GCodeBuffer &gb) noexcept;					// Return true if this code should be queued
	static bool ShouldQueueMCode(GCodeBuffer &gb) THROWS(GCodeException);	// Return true if this code should be queued

private:
	QueuedCode *freeItems;
	QueuedCode *queuedItems;
};

class QueuedCode
{
public:
	friend class GCodeQueue;

	QueuedCode(QueuedCode *n) noexcept : next(n), dataLength(0) { }
	QueuedCode *Next() const noexcept { return next; }

private:
	QueuedCode *next;

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
