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
	debugPrintf("%c n=%" PRIi32 " t=%" PRIu32 " s0=%.4e u=%.4e a=%.4e", ch, steps, (uint32_t)duration, (double)s0, (double)u, (double)a);
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
