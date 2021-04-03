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
#include <Platform/Tasks.h>

class MoveSegment
{
public:
	void* operator new(size_t count) { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	MoveSegment(MoveSegment *p_next) noexcept;

	float GetDistanceLimit() const noexcept { return endDistanceFraction; }
	float GetSegmentTime() const noexcept { return segTime; }
	float GetDDivU() const noexcept { return linear.dDivU; }
	float GetUDivA() const noexcept { return quadratic.uDivA; }
	float GetTwoDDivA() const noexcept { return quadratic.twoDDivA; }
	float GetAcceleration() const noexcept { return quadratic.acceleration; }

	MoveSegment *GetNext() const noexcept;
	bool IsLinear() const noexcept;
	bool IsLast() const noexcept;

	void SetNext(MoveSegment *p_next) noexcept;
	void SetLinear(float pDistanceLimit, float p_segTime, float p_dDivU) noexcept;
	void SetNonLinear(float pDistanceLimit, float p_segTime, float p_uDivA, float p_twoDDivA, float a) noexcept;

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
	uint32_t nextAndFlags;			// pointer to the next segment, plus flag bits
	float endDistanceFraction;		// the fraction of the move that has been completed at the end of this segment
	float segTime;					// the time in step clocks at which this move ends
	union
	{
		struct						// parameters for quadratic (accelerating/decelerating) moves
		{
			float uDivA;			// initial speed divided by acceleration
			float twoDDivA;			// twice the movement distance divided by the acceleration
			float acceleration;		// the acceleration of this move
		} quadratic;
		struct						// parameters for linear moves
		{
			uint32_t dDivU;			// the movement distance divided by the speed
		} linear;
	};
};

// Create a new one, leaving the flags clear
inline MoveSegment::MoveSegment(MoveSegment *p_next) noexcept
	: nextAndFlags(reinterpret_cast<uint32_t>(p_next))				// this also clears the flags
{
	// remaining fields are not initialised
}

inline MoveSegment *MoveSegment::GetNext() const noexcept
{
	return reinterpret_cast<MoveSegment*>(nextAndFlags & (~3u));
}

inline void MoveSegment::SetNext(MoveSegment *p_next) noexcept
{
	nextAndFlags = (nextAndFlags & 3u) | reinterpret_cast<uint32_t>(p_next);
}

inline bool MoveSegment::IsLinear() const noexcept
{
	return nextAndFlags & 1u;
}

inline bool MoveSegment::IsLast() const noexcept
{
	return (nextAndFlags & ~3u) == 0;
}

inline void MoveSegment::SetLinear(float pDistanceLimit, float p_segTime, float p_dDivU) noexcept
{
	endDistanceFraction = pDistanceLimit;
	segTime = p_segTime;
	linear.dDivU = p_dDivU;
	nextAndFlags |= 1u;
}

// Set up an accelerating or decelerating move. We assume that the 'linear' flag is already clear.
inline void MoveSegment::SetNonLinear(float pDistanceLimit, float p_segTime, float p_uDivA, float p_twoDDivA, float a) noexcept
{
	endDistanceFraction = pDistanceLimit;
	segTime = p_segTime;
	quadratic.uDivA = p_uDivA;
	quadratic.twoDDivA = p_twoDDivA;
	quadratic.acceleration = a;
}

// Release a single MoveSegment. Not thread-safe.
inline void MoveSegment::Release(MoveSegment *item) noexcept
{
	item->nextAndFlags = reinterpret_cast<uint32_t>(freeList);
	freeList = item;
}

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
