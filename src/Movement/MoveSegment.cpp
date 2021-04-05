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

void MoveSegment::DebugPrint() const noexcept
{
	debugPrintf("f=%12g t=%6" PRIu32 " ", (double)endDistanceFraction, (uint32_t)segTime);
	if (IsLinear())
	{
		debugPrintf("dDivU=%14g\n", (double)linear.dDivU);
	}
	else
	{
		debugPrintf("uDivA=%14g twoDDivA=%14g a=%14g\n", (double)quadratic.uDivA, (double)quadratic.twoDDivA, (double)quadratic.acceleration);
	}
}

// End
