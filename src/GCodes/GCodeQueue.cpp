/*
 * GCodeQueue.cpp
 *
 *  Created on: 22 Jun 2016
 *      Author: Christian
 */

#include "GCodeQueue.h"

#include "RepRap.h"
#include "GCodes.h"
#include "Movement/Move.h"

// GCodeQueue class

GCodeQueue::GCodeQueue() : freeItems(nullptr), queuedItems(nullptr)
{
	for (size_t i = 0; i < maxQueuedCodes; i++)
	{
		freeItems = new QueuedCode(freeItems);
	}
}

// If moves are scheduled and the command in the passed GCodeBuffer can be queued, try to queue it.
// If successful, return true to indicate it has been queued and the caller should not execute it.
// If it is not a command that should be queued, return false.
// If the queue is full, free up the oldest queued entry by copying its command to our own gcode buffer
// so that we have room to queue the original command.
bool GCodeQueue::QueueCode(GCodeBuffer &gb, uint32_t segmentsLeft)
{
	// Don't queue anything if no moves are being performed
	const uint32_t scheduledMoves = reprap.GetMove().GetScheduledMoves() + segmentsLeft;
	if (scheduledMoves == reprap.GetMove().GetCompletedMoves())
	{
		return false;
	}

#if SUPPORT_ROLAND
	// Don't queue codes if the Roland module is active
	if (reprap.GetRoland()->Active())
	{
		return false;
	}
#endif

	// Check for G-Codes that can be queued
	bool queueCode = false;
	switch (gb.GetCommandLetter())
	{
	case 'G':
		{
			const int code = gb.GetIValue();

			// Set active/standby temperatures
			queueCode = (code == 10 && gb.Seen('P'));
		}
		break;

	case 'M':
		{
			const int code = gb.GetIValue();
			switch(code)
			{
			case 3:		// spindle control
			case 4:
			case 5:
			case 42:	// set IO pin
			case 106:	// fan control
			case 107:	// fan off
			case 104:	// set temperatures and return immediately
			case 140:	// set bed temperature and return immediately
			case 141:	// set chamber temperature and return immediately
			case 144:	// bed standby
			case 117:	// display message
			case 280:	// set servo
			case 300:	// beep
			case 420:	// set RGB colour
				queueCode = true;
				break;

			default:
				break;
			}
		}
		break;

	default:
		break;
	}

	// Does it make sense to queue this code?
	if (queueCode)
	{
		char codeToRun[GCODE_LENGTH];
		size_t codeToRunLength;

		// Can we queue this code somewhere?
		if (freeItems == nullptr)
		{
			// No - we've run out of free items. Run the first outstanding code
			queueCode = false;
			codeToRunLength = strlen(queuedItems->code) + 1;
			SafeStrncpy(codeToRun, queuedItems->code, ARRAY_SIZE(codeToRun));

			// Release the first queued item so that it can be reused later
			QueuedCode * const item = queuedItems;
			queuedItems = item->next;
			item->next = nullptr;
			freeItems = item;
		}

		// Unlink a free element and assign gb's code to it
		QueuedCode * const code = freeItems;
		freeItems = code->next;
		code->AssignFrom(gb);
		code->executeAtMove = scheduledMoves;
		code->next = nullptr;

		// Append it to the list of queued codes
		if (queuedItems == nullptr)
		{
			queuedItems = code;
		}
		else
		{
			QueuedCode *last = queuedItems;
			while (last->Next() != nullptr)
			{
				last = last->Next();
			}
			last->next = code;
		}

		// Overwrite the passed gb's content if we could not store its original code
		if (!queueCode)
		{
			if (reprap.Debug(moduleGcodes))
			{
				reprap.GetPlatform().Message(DebugMessage, "(swap) ");
			}
			if (!gb.Put(codeToRun, codeToRunLength))
			{
				gb.Put('\n');
			}
		}
	}

	return queueCode;
}

bool GCodeQueue::FillBuffer(GCodeBuffer *gb)
{
	// Can this buffer be filled?
	if (queuedItems == nullptr || queuedItems->executeAtMove > reprap.GetMove().GetCompletedMoves())
	{
		// No - stop here
		return false;
	}

	// Yes - load it into the passed GCodeBuffer instance
	QueuedCode *code = queuedItems;
	code->AssignTo(gb);

	// Release this item again
	queuedItems = queuedItems->next;
	code->next = freeItems;
	freeItems = code;
	return true;
}

// Return true if there is nothing to do
bool GCodeQueue::IsIdle() const
{
	return queuedItems == nullptr || queuedItems->executeAtMove > reprap.GetMove().GetCompletedMoves();
}

// Because some moves may end before the print is actually paused, we need a method to
// remove all the entries that will not be executed after the print has finally paused
void GCodeQueue::PurgeEntries()
{
	QueuedCode *item = queuedItems, *lastItem = nullptr;
	while (item != nullptr)
	{
		if (item->executeAtMove > reprap.GetMove().GetScheduledMoves())
		{
			// Release this item
			QueuedCode *nextItem = item->Next();
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

void GCodeQueue::Clear()
{
	while (queuedItems != nullptr)
	{
		QueuedCode * const item = queuedItems;
		queuedItems = item->Next();
		item->next = freeItems;
		freeItems = item;
	}
}

void GCodeQueue::Diagnostics(MessageType mtype)
{
	reprap.GetPlatform().MessageF(mtype, "Code queue is %s\n", (queuedItems == nullptr) ? "empty." : "not empty:");
	if (queuedItems != nullptr)
	{
		const QueuedCode *item = queuedItems;
		size_t queueLength = 0;
		do
		{
			queueLength++;
			reprap.GetPlatform().MessageF(mtype, "Queued '%s' for move %" PRIu32 "\n", item->code, item->executeAtMove);
		} while ((item = item->Next()) != nullptr);
		reprap.GetPlatform().MessageF(mtype, "%d of %d codes have been queued.\n", queueLength, maxQueuedCodes);
	}
}

// QueuedCode class

void QueuedCode::AssignFrom(GCodeBuffer &gb)
{
	toolNumberAdjust = gb.GetToolNumberAdjust();
	SafeStrncpy(code, gb.Buffer(), ARRAY_SIZE(code));
}

void QueuedCode::AssignTo(GCodeBuffer *gb)
{
	gb->SetToolNumberAdjust(toolNumberAdjust);
	if (!gb->Put(code, strlen(code) + 1))
	{
		gb->Put('\n');
	}
}

// End

