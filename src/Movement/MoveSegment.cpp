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

// Return the first deceleration segment in the move segment chain, or nullptr if there isn't one
// Called when preparing extruders, which have at most one deceleration segment,
const MoveSegment *MoveSegment::GetFirstDecelSegment() const noexcept
{
	const MoveSegment *seg = this;
	do
	{
		if (!seg->IsLinear() && !seg->IsAccelerating())
		{
			break;
		}
		seg = seg->GetNext();
	} while (seg != nullptr);
	return seg;

}

void MoveSegment::DebugPrint(char ch) const noexcept
{
	debugPrintf("%c d=%7.3f t=%7" PRIu32 " ", ch, (double)segmentLength, (uint32_t)segTime);
	if (IsLinear())
	{
		debugPrintf("c=%.3e\n", (double)c);
	}
	else
	{
		debugPrintf("b=%.3e c=%.3e\n", (double)b, (double)c);
	}
}

// End
