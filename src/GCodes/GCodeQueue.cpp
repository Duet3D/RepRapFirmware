/*
 * GCodeQueue.cpp
 *
 *  Created on: 22 Jun 2016
 *      Author: Christian
 */

#include "GCodeQueue.h"

#include <Platform/RepRap.h>
#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"
#include <Movement/Move.h>

// GCodeQueue class

GCodeQueue::GCodeQueue() noexcept : freeItems(nullptr), queuedItems(nullptr)
{
	for (size_t i = 0; i < maxQueuedCodes; i++)
	{
		freeItems = new QueuedCode(freeItems);
	}
}

// Return true if the GCode in the GCodeBuffer should be queued
// Caller has already checked that the command does not contain an expression and involves modifying tool temperatures or spindle speed
/*static*/ bool GCodeQueue::ShouldQueueG10(GCodeBuffer &gb) noexcept
{
	return reprap.GetMove().GetScheduledMoves() != reprap.GetMove().GetCompletedMoves()
			&& gb.DataLength() <= BufferSizePerQueueItem;						// only queue it if it is short enough to fit in a queue item
}

// Return true if the MCode in the GCodeBuffer should be queued. Caller has already checked that the command does not contain an expression.
/*static*/ bool GCodeQueue::ShouldQueueMCode(GCodeBuffer &gb) THROWS(GCodeException)
{
	// Don't queue anything if no moves are being performed
	if (reprap.GetMove().GetScheduledMoves() != reprap.GetMove().GetCompletedMoves())
	{
		bool shouldQueue;
		switch (gb.GetCommandNumber())
		{
		case 3:		// spindle or laser control
		case 5:		// spindle or laser control
			// On laser devices we use these codes to set the default laser power for the next G1 command
			shouldQueue = reprap.GetGCodes().GetMachineType() != MachineType::laser;
			break;

		case 4:		// spindle control
		case 42:	// set IO pin
		case 106:	// fan control
		case 107:	// fan off
		case 104:	// set temperatures and return immediately
		case 140:	// set bed temperature and return immediately
		case 141:	// set chamber temperature and return immediately
		case 144:	// bed standby
		case 280:	// set servo
		case 300:	// beep
		case 568:	// spindle or temperature control
			shouldQueue = true;
			break;

		case 117:	// display message
			{
				// We need to call GetUnprecedentedString to ensure that if the string argument is not quoted, gb.DataLength() will return the correct value.
				// We need to pass the correct length string buffer here because GetUnprecedentedString will throw if the string is too long for the buffer.
				String<M117StringLength> dummy;
				gb.GetUnprecedentedString(dummy.GetRef());
			}
			shouldQueue = true;
			break;

#if SUPPORT_LED_STRIPS
		case 150:	// set LED colours
			shouldQueue = !reprap.GetPlatform().GetLedStripManager().MustStopMovement(gb);		// if it is going to call LockMovementAndWaitForStandstill then we mustn't queue it
			break;
#endif
		// A note about M291:
		// - We cannot queue M291 messages that are blocking, i.e. with S2 or S3 parameter
		// - If we queue non-blocking M291 messages then it can happen that if a non-blocking M291 is used and a little later a blocking M291 is used,
		//   then the blocking one gets displayed while the non-blocking one is still in the queue. Then the non-blocking one overwrites it, and the
		//   blocking one can no longer be acknowledged except by sending M292 manually.
		// - Therefore we no longer queue any M291 commands.
		case 291:
		default:
			return false;
		}

		if (shouldQueue && gb.DataLength() <= BufferSizePerQueueItem)	// only queue it if it is short enough to fit in a queue item
		{
			return true;
		}
	}

	return false;
}

// Try to queue the command in the passed GCodeBuffer.
// If successful, return true to indicate it has been queued.
// If the queue is full, return false. Caller will wait for space to become available.
bool GCodeQueue::QueueCode(const GCodeBuffer &gb) noexcept
{
	// Can we queue this code somewhere?
	if (freeItems == nullptr)
	{
		return false;
	}

	// Unlink a free element and assign gb's code to it
	QueuedCode * const code = _ecv_not_null(freeItems);
	freeItems = code->next;
	code->AssignFrom(gb);
	code->executeAtMove = reprap.GetMove().GetScheduledMoves();
	code->next = nullptr;

	// Append it to the list of queued codes
	if (queuedItems == nullptr)
	{
		queuedItems = code;
	}
	else
	{
		QueuedCode * last = _ecv_not_null(queuedItems);
		while (last->Next() != nullptr)
		{
			last = _ecv_not_null(last->Next());
		}
		last->next = code;
	}

	return true;
}

bool GCodeQueue::FillBuffer(GCodeBuffer *gb) noexcept
{
	// Can this buffer be filled?
	if (queuedItems == nullptr || queuedItems->executeAtMove > reprap.GetMove().GetCompletedMoves())
	{
		// No - stop here
		return false;
	}

	// Yes - load it into the passed GCodeBuffer instance
	QueuedCode * code = _ecv_not_null(queuedItems);
	code->AssignTo(gb);

	// Release this item again
	queuedItems = queuedItems->next;
	code->next = freeItems;
	freeItems = code;
	return true;
}

// These inherited virtual functions need to be defined but are not called
void GCodeQueue::Reset() noexcept
{
	Clear();
}

size_t GCodeQueue::BytesCached() const noexcept
{
	return 0;
}

// Return true if there is nothing to do
bool GCodeQueue::IsIdle() const noexcept
{
	return queuedItems == nullptr || queuedItems->executeAtMove > reprap.GetMove().GetCompletedMoves();
}

// Because some moves may end before the print is actually paused, we need a method to
// remove all the entries that will not be executed after the print has finally paused
void GCodeQueue::PurgeEntries() noexcept
{
	QueuedCode *_ecv_null item = queuedItems, *_ecv_null lastItem = nullptr;
	while (item != nullptr)
	{
		if (item->executeAtMove > reprap.GetMove().GetScheduledMoves())
		{
			// Release this item
			QueuedCode *_ecv_null nextItem = item->Next();
			item->next = freeItems;
			freeItems = item;

			// Unlink it from the list
			if (lastItem == nullptr)
			{
				queuedItems = nextItem;
			}
			else
			{
				lastItem->next = nextItem;
			}
			item = nextItem;
		}
		else
		{
			lastItem = item;
			item = item->Next();
		}
	}
}

void GCodeQueue::Clear() noexcept
{
	while (queuedItems != nullptr)
	{
		QueuedCode * const item = _ecv_not_null(queuedItems);
		queuedItems = item->Next();
		item->next = freeItems;
		freeItems = item;
	}
}

void GCodeQueue::Diagnostics(MessageType mtype, unsigned int queueNumber) noexcept
{
	if (queuedItems == nullptr)
	{
		reprap.GetPlatform().MessageF(mtype, "Code queue %u is empty\n", queueNumber);
	}
	else
	{
		const QueuedCode *_ecv_null item = queuedItems;
		do
		{
#if HAS_SBC_INTERFACE
			// The following may output binary gibberish if this code is stored in binary.
			// We could restore this message by using GCodeBuffer::AppendFullCommand but there is probably no need to
			if (!reprap.UsingSbcInterface())
#endif
			{
				reprap.GetPlatform().MessageF(mtype, "Queue %u has '%.*s' for move %" PRIu32 "\n", queueNumber, item->dataLength, item->data, item->executeAtMove);
			}
		} while ((item = item->Next()) != nullptr);
	}
}

// QueuedCode class

void QueuedCode::AssignFrom(const GCodeBuffer &gb) noexcept
{
#if HAS_SBC_INTERFACE
	isBinary = gb.IsBinary();
#endif
	dataLength = min<size_t>(gb.DataLength(), sizeof(data));
	memcpy(data, gb.DataStart(), dataLength);
}

void QueuedCode::AssignTo(GCodeBuffer *gb) noexcept
{
#if HAS_SBC_INTERFACE
	if (isBinary)
	{
		// Note that the data has to remain on a 4-byte boundary for this to work
		gb->PutBinary(reinterpret_cast<const uint32_t *>(data), dataLength / sizeof(uint32_t));
	}
	else
#endif
	{
		gb->PutAndDecode(data, dataLength);
	}
}

// End

