/*
 * MoveSegment.cpp
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 */

#include "MoveSegment.h"

// Static members

MoveSegment *MoveSegment::freeList = nullptr;
unsigned int MoveSegment::numCreated = 0;

// Allocate a MoveSegment, from the freelist if possible, else create a new one
MoveSegment *MoveSegment::Allocate(MoveSegment *p_next) noexcept
{
	const irqflags_t iflags = IrqSave();
	MoveSegment * ms = freeList;
	if (ms != nullptr)
	{
		freeList = ms->next;
		IrqRestore(iflags);
		ms->next = p_next;
	}
	else
	{
		++numCreated;
		IrqRestore(iflags);
		ms = new MoveSegment(p_next);
	}
	return ms;
}

// Release a MoveSegment
void MoveSegment::ReleaseAll(MoveSegment *item) noexcept
{
	while (item != nullptr)
	{
		MoveSegment *itemToRelease = item;
		item = item->next;
		Release(itemToRelease);
	}
}

void MoveSegment::DebugPrint(char ch) const noexcept
{
	debugPrintf("%c s=%" PRIu32 " t=%.1f d=%.2f u=%.4e a=%.4e f=%02" PRIx32 "\n", ch, (uint32_t)startTime, (double)duration, (double)distance, (double)u, (double)a, flags.all);
}

/*static*/ void MoveSegment::DebugPrintList(char ch, const MoveSegment *segs) noexcept
{
	if (segs == nullptr)
	{
		debugPrintf("%c null\n", ch);
	}
	else
	{
		while (segs != nullptr)
		{
			segs->DebugPrint(ch);
			segs = segs->GetNext();
		}
	}
}

// End
