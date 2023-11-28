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
		freeList = next;
		ms->next = p_next;
	}
	else
	{
		ms = new MoveSegment(p_next);
		++numCreated;
	}
	return ms;
}

// Release a single MoveSegment. Not thread-safe.
inline void MoveSegment::Release(MoveSegment *item) noexcept
{
	if (item->IsDelta())
	{
		item->next = deltaFreeList;
		deltaFreeList = (DeltaMoveSegment*)item;
	}
	else
	{
		item->next = freeList;
		freeList = item;
	}
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
	debugPrintf("%c d=%.4e t=%.1f ", ch, (double)segLength, (double)segTime);
	if (IsLinear())
	{
		debugPrintf("c=%.4e", (double)c);
	}
	else
	{
		debugPrintf("b=%.4e c=%.4e a=%.4e", (double)b, (double)c, (double)acceleration);
	}
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

// Allocate a MoveSegment, from the freelist if possible, else create a new one. Not thread-safe. Clears the flags.
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
	debugPrintf(" dv=[%.3f %.3f %.3f] minusAaPlusBbTimesS=%.4e dSquaredMinusAsquaredMinusBsquared=%.4e",
				(double)dv[0], (double)dv[1], (double)dv[2], (double)fMinusAaPlusBbTimesS, (double)fDSquaredMinusAsquaredMinusBsquaredTimesSsquared);
	//TODO
}

// End
