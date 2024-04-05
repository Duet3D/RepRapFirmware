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

void MoveSegment::InitialAllocate(unsigned int num) noexcept
{
	while (num > numCreated)
	{
		freeList = new MoveSegment(freeList);
		++numCreated;
	}
}

// Allocate a MoveSegment, from the freelist if possible, else create a new one. Not thread-safe. Clears the flags.
MoveSegment *MoveSegment::Allocate(MoveSegment *next) noexcept
{
	MoveSegment * ms = freeList;
	if (ms != nullptr)
	{
		freeList = ms->GetNext();
		ms->nextAndFlags = reinterpret_cast<uint32_t>(next);
	}
	else
	{
		ms = new MoveSegment(next);
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
	debugPrintf("%c d=%.4e t=%.1f b=%+.4e c=%+.4e a=%+.4e\n", ch, (double)segLength, (double)segTime, (double)b, (double)c, (double)acceleration);
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

// Check that the calculated distance of each segment agrees with its length. If it doesn't then report it and return true.
/*static*/ bool MoveSegment::DebugCheckSegments(const MoveSegment *segs) noexcept
{
	unsigned int n = 0;
	bool foundBadSegment = false;
	while (segs != nullptr)
	{
		const float diff = segs->GetCalculatedDistance() - segs->GetSegmentLength();
		if (fabsf(diff) * 1000 > segs->GetSegmentLength())
		{
			debugPrintf("Seg length diff %.2e at %u\n", (double)diff, n);
			MoveSegment::DebugPrintList('S', segs);
			foundBadSegment = true;
		}
		++n;
		segs = segs->GetNext();
	}
	return foundBadSegment;
}

// End
