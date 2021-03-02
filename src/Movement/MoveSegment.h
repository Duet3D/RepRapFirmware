/*
 * MoveSegment.h
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 *
 * This class holds the parameters of a segment of a move.
 *
 * Accelerating moves obey t^2/2 + (u/a)t - (s/a) = 0 where t = time, a = acceleration, u = initial speed, s = distance moved
 * from which t = sqrt(A + Bs) + C
 * where A = (u/a)^2, B = 2/a, C = -u/a
 * We can interpret A/B as the distance at which the move reversed, C is the time at which it reversed (in the past)
 *
 * Decelerating moves obey t^2/2 - (u/d)t + (s/d) = 0 where t = time, d = deceleration, u = initial speed, s = distance moved
 * from which t = sqrt(A + Bs) + C
 * where A = (u/a)^2, B = -2/d, C = u/d
 * We can interpret -(A/B) as the distance at which the move will reverse, C is the time at which it will reverse (in the future)
 *
 * When a decelerating move reverses, s gets replaced by sr - (s - sr) = 2sr - s where sr is the distance at reverse
 * from which t = sqrt(A + B(2sr - s)) + C = sqrt(A + 2Bsr - Bs)
 * but A = -Bsr, therefore t = sqrt(-A - Bs) + C
 * so we can just change the sign of A and B and use the same parameters for the reverse phase
 *
 * Linear moves obey ut = s where t = time, u = speed, s = distance moved
 * from which t = Bs + C
 * where B = 1/u, C is the time at which the segment started
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

	float GetA() const noexcept { return pA; }
	float GetB() const noexcept { return pB; }
	float GetC() const noexcept { return pC; }
	float GetDistanceLimit() const noexcept { return distanceLimit; }
	MoveSegment *GetNext() const noexcept;
	bool IsLinear() const noexcept;
	bool IsLast() const noexcept;

	void SetNext(MoveSegment *p_next) noexcept;
	void SetLinear(float pDistanceLimit, float uB, float uC) noexcept;
	void SetNonLinear(float pDistanceLimit, float uA, float uB, float uC) noexcept;
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
	float distanceLimit;
};

// Create a new one, leaving the flags clear
inline MoveSegment::MoveSegment(MoveSegment *p_next) noexcept
	: nextAndFlags(reinterpret_cast<uint32_t>(p_next))				// this also clears the flags
{
	// clocks, distanceLimit, pA, pB and pC are not initialised
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

inline void MoveSegment::SetLinear(float pDistanceLimit, float uB, float uC) noexcept
{
	distanceLimit = pDistanceLimit;
	pB = uB;
	pC = uC;
	nextAndFlags |= 1u;
}

// Set up an accelerating or decelerating move. We assume that the 'linear' flag is already clear.
inline void MoveSegment::SetNonLinear(float pDistanceLimit, float uA, float uB, float uC) noexcept
{
	distanceLimit = pDistanceLimit;
	pA = uA;
	pB = uB;
	pC = uC;
}

inline void MoveSegment::SetLast() noexcept
{
	nextAndFlags |= 2u;
}

inline uint32_t MoveSegment::CalcForwardStepTime(float moveFraction) const noexcept
{
	const float ret = pC + ((!IsLinear()) ? fastSqrtf(pA + pB * moveFraction) : pB * moveFraction);
	return (uint32_t)ret;
}

inline uint32_t MoveSegment::CalcReverseStepTime(float moveFraction) const noexcept
{
	const float ret = pC + fastSqrtf((-pB) * moveFraction - pA);
	return (uint32_t)ret;
}

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
