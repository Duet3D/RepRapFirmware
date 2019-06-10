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

class GCodeQueue : public GCodeInput
{
public:
	GCodeQueue();

	void Reset() override;										// Clean all the cached data from this input
	bool FillBuffer(GCodeBuffer *gb) override;					// If there is another move to execute at this time, fill a buffer
	size_t BytesCached() const override;						// How many bytes have been cached?

	bool QueueCode(GCodeBuffer &gb);							// Queue a G-code
	void PurgeEntries();										// Remove stored codes when a print is being paused
	void Clear();												// Clean up all the stored codes
	bool IsIdle() const;										// Return true if there is nothing to do

	void Diagnostics(MessageType mtype);

	static bool ShouldQueueCode(GCodeBuffer &gb);				// Return true if this code should be queued

private:
	QueuedCode *freeItems;
	QueuedCode *queuedItems;
};

class QueuedCode
{
public:
	friend class GCodeQueue;

	QueuedCode(QueuedCode *n) : next(n) { }
	QueuedCode *Next() const { return next; }

private:
	QueuedCode *next;

	char code[SHORT_GCODE_LENGTH];
	uint32_t executeAtMove;
	int toolNumberAdjust;

	void AssignFrom(GCodeBuffer &gb);
	void AssignTo(GCodeBuffer *gb);
};

#endif
