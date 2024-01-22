/*
 * MoveSegment.cpp
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 */

#include "MoveSegment.h"

// Static members

MoveSegment *MoveSegment::freeList = nullptr;
DeltaMoveSegment *MoveSegment::deltaFreeList = nullptr;
unsigned int MoveSegment::numCreated = 0;

// Allocate a MoveSegment, from the freelist if possible, else create a new one. Not thread-safe. Clears the flags.
MoveSegment *MoveSegment::Allocate(MoveSegment *p_next) noexcept
{
	MoveSegment * ms = freeList;
	if (ms != nullptr)
	{
		freeList = ms->next;
		ms->next = p_next;
	}
	else
	{
		ms = new MoveSegment(p_next);
		++numCreated;
	}
	return ms;
}

void MoveSegment::AddToTail(MoveSegment *tail) noexcept
{
	MoveSegment *seg = this;
	while (seg->GetNext() != nullptr)
	{
		seg = seg->GetNext();
	}
	seg->SetNext(tail);
}

void MoveSegment::DebugPrint(char ch) const noexcept
{
	debugPrintf("%c t=%" PRIu32 " u=%.4e a=%.4e", ch, (uint32_t)duration, (double)u, (double)a);
	if (IsDelta())
	{
		((const DeltaMoveSegment*)this)->DebugPrintDelta();
	}
	debugPrintf("\n");
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

// Allocate a MoveSegment, from the freelist if possible, else create a new one. Not thread-safe. Sets the delta flag, clears the other flags.
DeltaMoveSegment *DeltaMoveSegment::Allocate(MoveSegment *p_next) noexcept
{
	DeltaMoveSegment * ms = deltaFreeList;
	if (ms != nullptr)
	{
		deltaFreeList = (DeltaMoveSegment*)ms->next;
		ms->next = p_next;
		ms->isDelta = 1;
	}
	else
	{
		ms = new DeltaMoveSegment(p_next);
		++numCreated;
	}
	return ms;
}

// Print the extra bits in a delta move segment
void DeltaMoveSegment::DebugPrintDelta() const noexcept
{
	debugPrintf(" dA=%.4e dB=%.4e dC=%.4e dD=%.4e dE=%.4e", (double)deltaA, (double)deltaB, (double)deltaC, (double)deltaD, (double)deltaE);
}

// End
