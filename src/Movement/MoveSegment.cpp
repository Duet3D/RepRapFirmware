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

// Allocate a MoveSegment, from the freelist if possible, else create a new one
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

uint32_t MoveSegment::CalcForwardStepTime(float moveFraction) const noexcept
{
	const float ret = pC + ((!IsLinear()) ? fastSqrtf(pA + pB * moveFraction) : pB * moveFraction);
	return (uint32_t)ret;
}

uint32_t MoveSegment::CalcReverseStepTime(float moveFraction) const noexcept
{
	const float ret = pC - ((!IsLinear()) ? fastSqrtf(pA + pB * moveFraction) : pB * moveFraction);
	return (uint32_t)ret;
}

// End
