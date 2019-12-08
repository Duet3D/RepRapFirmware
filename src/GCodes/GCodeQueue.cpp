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

// Return true if the move in the GCodeBuffer should be queued
/*static*/ bool GCodeQueue::ShouldQueueCode(GCodeBuffer &gb)
{
#if SUPPORT_ROLAND
	// Don't queue codes if the Roland module is active
	if (reprap.GetRoland()->Active())
	{
		return false;
	}
#endif

	// Don't queue anything if no moves are being performed
	const uint32_t scheduledMoves = reprap.GetMove().GetScheduledMoves();
	if (scheduledMoves != reprap.GetMove().GetCompletedMoves())
	{
		switch (gb.GetCommandLetter())
		{
		case 'G':
			{
				const int code = gb.GetCommandNumber();
				return code == 10 && gb.Seen('P') && !gb.Seen('L') && (gb.Seen('R') || gb.Seen('S'));			// set active/standby temperatures
			}

		case 'M':
			{
				switch (gb.GetCommandNumber())
				{
				case 3:		// spindle or laser control
				case 5:		// spindle or laser control
					// On laser devices we use these codes to set the default laser power for the next G1 command
					return reprap.GetGCodes().GetMachineType() != MachineType::laser;

				case 4:		// spindle control
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
					return true;

				case 291:
					{
						bool seen = false;
						int32_t sParam = 1;
						gb.TryGetIValue('S', sParam, seen);
						return sParam < 2;					// queue non-blocking messages only
					}

				default:
					break;
				}
			}
			break;

		default:
			break;
		}
	}

	return false;
}

// Try to queue the command in the passed GCodeBuffer.
// If successful, return true to indicate it has been queued.
// If the queue is full or the command is too long to be queued, return false.
bool GCodeQueue::QueueCode(GCodeBuffer &gb)
{
	// Can we queue this code somewhere?
	if (freeItems == nullptr || gb.CommandLength() > SHORT_GCODE_LENGTH - 1)
	{
		return false;
	}

	// Unlink a free element and assign gb's code to it
	QueuedCode * const code = freeItems;
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
		QueuedCode *last = queuedItems;
		while (last->Next() != nullptr)
		{
			last = last->Next();
		}
		last->next = code;
	}

	return true;
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
	const size_t length = min<size_t>(gb.CommandLength(), ARRAY_SIZE(code) - 1);
	memcpy(code, gb.CommandStart(), length);
	code[length] = 0;
}

void QueuedCode::AssignTo(GCodeBuffer *gb)
{
	gb->SetToolNumberAdjust(toolNumberAdjust);
	gb->Put(code);
}

// End

