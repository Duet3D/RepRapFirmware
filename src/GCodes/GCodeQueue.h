/*
 * GCodeQueue.h
 *
 *  Created on: 22 Jun 2016
 *      Author: Christian
 */
#ifndef GCODEQUEUE_H
#define GCODEQUEUE_H

#include "RepRapFirmware.h"
#include "GCodeBuffer/GCodeBuffer.h"

class QueuedCode;

const size_t BufferSizePerQueueItem = SHORT_GCODE_LENGTH;

class GCodeQueue
{
public:
	GCodeQueue();

	static bool ShouldQueueCode(GCodeBuffer &gb);				// Return true if this code should be queued
	bool QueueCode(GCodeBuffer &gb);							// Queue a G-code and return true if it could be stored
	bool FillBuffer(GCodeBuffer *gb);							// If there is another move to execute at this time, fill a buffer
	void PurgeEntries();										// Remove stored codes when a print is being paused
	void Clear();												// Clean up all the stored codes
	bool IsIdle() const;										// Return true if there is nothing to do

	void Diagnostics(MessageType mtype);

private:
	QueuedCode *freeItems;
	QueuedCode *queuedItems;
};

class QueuedCode
{
public:
	friend class GCodeQueue;

	QueuedCode(QueuedCode *n) : next(n), dataLength(0) { }
	QueuedCode *Next() const { return next; }

private:
	QueuedCode *next;

	bool isBinary;
	char data[BufferSizePerQueueItem];
	size_t dataLength;

	uint32_t executeAtMove;
	int toolNumberAdjust;

	void AssignFrom(GCodeBuffer &gb);
	void AssignTo(GCodeBuffer *gb);
};

#endif
