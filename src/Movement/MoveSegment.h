/*
 * MoveSegment.h
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 *
 * This class holds the parameters of a segment of a move.
 */

#ifndef SRC_MOVEMENT_MOVESEGMENT_H_
#define SRC_MOVEMENT_MOVESEGMENT_H_

#include <RepRapFirmware.h>
#include <Tasks.h>

class MoveSegment
{
public:
	void* operator new(size_t count) { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	MoveSegment(MoveSegment *p_next) noexcept;

	MoveSegment *GetNext() const noexcept;
	bool IsLinear() const noexcept;
	bool IsLast() const noexcept;

	void SetNext(MoveSegment *p_next) noexcept;
	void SetLinear(float uB, float uC) noexcept;
	void SetNonLinear(float uA, float uB, float uC) noexcept;
	void SetLast() noexcept;

	uint32_t CalcForwardStepTime(float moveFraction) const noexcept;
	uint32_t CalcReverseStepTime(float moveFraction) const noexcept;

	// Allocate a MoveSegment, clearing the Linear and Last flags
	static MoveSegment *Allocate(MoveSegment *next) noexcept;

	// Release a MoveSegment
	static void Release(MoveSegment *item) noexcept;

	static void InitialAllocate(unsigned int num) noexcept;
	static unsigned int NumCreated() noexcept { return numCreated; }

private:
	static MoveSegment *freeList;
	static unsigned int numCreated;

	static_assert(sizeof(MoveSegment*) == sizeof(uint32_t));

	// The 'next' field is a MoveSegment pointer with two flag bits in the bottom two bits
	uint32_t nextAndFlags;
	float pA, pB, pC;
	uint32_t segTime;
};

// Create a new one, leaving the flags clear
inline MoveSegment::MoveSegment(MoveSegment *p_next) noexcept
	: nextAndFlags(reinterpret_cast<uint32_t>(p_next))				// this also clears the flags
{
	// pA, pB and pC are not initialised
}

inline MoveSegment *MoveSegment::GetNext() const noexcept
{
	return reinterpret_cast<MoveSegment*>(nextAndFlags & (~3u));
}

void MoveSegment::SetNext(MoveSegment *p_next) noexcept
{
	nextAndFlags = (nextAndFlags & 3) | reinterpret_cast<uint32_t>(p_next);
}

inline bool MoveSegment::IsLinear() const noexcept
{
	return nextAndFlags & 1u;
}

inline bool MoveSegment::IsLast() const noexcept
{
	return nextAndFlags & 2u;
}

inline void MoveSegment::SetLinear(float uB, float uC) noexcept
{
	pB = uB;
	pC = uC;
	nextAndFlags |= 1u;
}

// Set up an accelerating or decelerating move. We assume that the 'linear' flag is already clear.
inline void MoveSegment::SetNonLinear(float uA, float uB, float uC) noexcept
{
	pA = uA;
	pB = uB;
	pC = uC;
}

inline void MoveSegment::SetLast() noexcept
{
	nextAndFlags |= 2u;
}

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
