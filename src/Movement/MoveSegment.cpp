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

// Allocate a MoveSegment, from the freelist if possible, else create a new one. Not thread-safe.
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

// End
