/*
 * MoveSegment.h
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 *
 * This class holds the parameters of a segment of a move. The idea is that when preparing the move, we pre-calculate as much as possible so that the step time calculation in the step ISR is fast.
 * In particular, we wish to avoid division as much as possible when generating steps.
 *
 * To save memory, we wish to share move segments between motors as far as possible. We can do this if all axes use the same input shaping, at the expense of an additional multiplication in the step ISR.
 * The move segments for axes refer to the move as a whole e.g. the composite linear axis movement. For each axis, when starting the move segment we scale the C factor by 1.0/(f*m)
 * where f is the move fraction and m s the steps/mm. We can do this using a multiplication if we precalculate 1.0/(f*m) when preparing the move and store it in the DM.
 *
 * To handle extruders with pressure advance, we have a few options:
 * 1. Use separate MoveSegments for the extruders. Each extruder will need its own MoveSegment chain, except that multiple extruders using the same pressure advance constant could share a chain if we
 *    scale by 1.0/(f*m) again. We don't need to apply input shaping to extruders.
 * 2. Use the axis move segments. When starting a new segment, each extruder will need to calculate and store that modified A B C coefficients to take account of PA. When input shaping is used,
 *    shaping will be applied to extruders (so there will be more MoveSegments to process). This would save memory by using fewer MoveSegments, but require more computation.
 * 3. Use a common MoveSegment chain without input shaping for all extruders (which could be shared with the axes when the move does not use input shaping), and adjust the A B C coefficients to account for PA.
 *
 * Let:
 *   s = distance travelled by this axis or extruder since the start of the entire move
 *   s0 = distance travelled by this axis or extruder at the start of this segment
 *   a = acceleration of the move as a whole taking multiple axes into account
 *   d = deceleration of the move as a whole taking multiple axes into account (so d is positive)
 *   u = initial speed of the move segment as a whole
 *   f = the fraction of the move that this axis/extruder uses
 *   m = the steps/mm for this axis or extruder
 *   n = number of steps taken since start of move
 * The basic motion equation for an acceleration segment is:
 *   s = s0 + u*f*t + 0.5*a*f*t^2
 * Solving for t:
 *   t = -(u/a) + sqrt((u/a)^2 + 2*(s-s0)/(a*f))
 * If we take s0 = S0 * f:
 *   t = -(u/a) + sqrt((u/a)^2 + 2*s/(a*f) - 2*S0/a)
 * Substituting s = n/m:
 *   t = -(u/a) + sqrt((u/a)^2 + 2*n/(f*m*a) - 2*S0/a)
 * For a deceleration segment, just set a = -d:
 *   t = (u/d) + sqrt((u/d)^2 - 2*n/(f*m*d) + 2*S0/d)
 * For a linear segment:
 *   s = s0 + u*f*t
 * from which:
 *   t = (s - s0)/(u*f)
 * If we take s0 = S0 * f:
 *   t = s/(u*f) - S0/u
 * Substituting s = n/m:
 *   t = n/(u*f*m) - S0/u
 *
 * Now add pressure advance with constant k seconds.
 * For the acceleration segment, u is replaced by u+(k*a):
 *   t = -(u/a) - k + sqrt((u/a + k)^2 + 2*n/(f*m*a) - 2*S0/a)
 * For the deceleration segment, u is replaced by u-(k*d). Additionally, S0 is replaced by S0 + EAD where EAD stands for extra acceleration distance, EAD = (vaccel-uaccel)*k = a*T*k where T was the acceleration time:
 *   t = (u/d) - k + sqrt((u/d - k)^2 - 2*n/(f*m*d) + 2*(S0 + EAD)/d)
 * For the linear segment, S0 must include EAD again:
 *   t = n/(u*f*m) - (S0 + EAD)/u
 * Now assume that there is also a fractional step p brought forward, where 0 <= p < 1.0. This must be subtracted from s0. Equivalently, add p/f to S0.
 * Acceleration segment:
 *   t = -(u/a) - k + sqrt((u/a + k)^2 + 2*n/(f*m*a) - 2*(S0 + p/f)/a)
 * Deceleration segment:
 *   t = (u/d) - k + sqrt((u/d - k)^2 - 2*n/(f*m*d) + 2*(S0 + p/f + EAD)/d)
 * Linear segment:
 *   t = n/(u*f*m) - (S0 + p/f + EAD)/u
 *
 * We can summarise like this. For an acceleration or deceleration segment:
 *   t = B + sqrt(A + C*n/(f*m))
 * where for an acceleration segment:
 *   B = -u/a + k
 *   A = B^2 - 2*(S0 + p/f)/a
 *   C = 2/a
 * and for a deceleration segment:
 *   B = u/d - k
 *   A = B^2 + 2*(S0 + p/f + EAD)/d
 *   C = -2/d
 * For a linear segment:
 *   t = B + C*n/(f*m)
 * where:
 *   B = -(S0 + p/f + EAD)/u
 *   C = 1/u
 *
 * The move segment stores a flag indicating whether the segment is accel/decel or linear, and the coefficients A B and C.
 * It also stores the distance limit for the segment, i.e. S + EAD at the end of the segment. From this we can work out the step limit N = (S + EAD)/(f*m).
 *
 * For accelerating moves we can interpret (-A/C) as the (negative) distance at which the move reversed, and B is the time at which it reversed (negative so in the past)
 * For decelerating moves we can interpret (A/C) as the distance at which the move will reverse, B is the time at which it will reverse (positive so in the future)
 *
 * When starting a new axis segment we calculate actual coefficients A' B' C' as follows:
 *   A' = A (only used for acceleration/deceleration segments)
 *   B' = B
 *   C' = C * 1/(f*m)
 *
 * If we share common move segments between all extruders and account for PA and fractional steps when starting them, we will need to modify the coefficients as follows.
 * For acceleration segments:
 *   A' = A - 2*k*u/a - (p/f)/a
 * or:
 *   A' = B'^2 - (p/f)/a
 *   B' = B - k
 *   C' = C * 1/(f*m)
 * For deceleration segments:
 *   A' = A - 2*k*u/d + k^2 + 2*(p/f + EAD)/d
 * alternatively, if instead of storing A we store Amod where Amod = 2*S0/d (so that A = B'2 + Amod):
 *   A' = B'^2 + Amod  + 2*(p/f + EAD)/d
 *   B' = B - k
 *   C' = C * 1/(f*m)
 * For linear segments:
 *   B' = B - (p/f + EAD)
 *   C' = C * 1/(f*m)
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

	MoveSegment *GetNext() const noexcept;
	bool IsLinear() const noexcept;
	bool IsReverse() const noexcept;
	bool IsLast() const noexcept;

	void SetNext(MoveSegment *p_next) noexcept;
	void SetLinear(float pDistanceLimit, float p_segTime, float p_dDivU) noexcept;
	void SetNonLinear(float pDistanceLimit, float p_segTime, float p_uDivA, float p_twoDDivA) noexcept;
	void SetReverse() noexcept;

	void AddToTail(MoveSegment *tail) noexcept;

	void DebugPrint() const noexcept;

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
			float uDivA;			// initial speed divided by acceleration (number of clocks since or until zero speed)
			float twoDDivA;			// Reciprocal of the acceleration, scaled by twice the distance of the complete move
		} quadratic;
		struct						// parameters for linear moves
		{
			uint32_t dDivU;			// the reciprocal of the speed, scaled by the distance of the complete move
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

inline bool MoveSegment::IsReverse() const noexcept
{
	return nextAndFlags & 2u;
}

inline void MoveSegment::SetReverse() noexcept
{
	nextAndFlags |= 2u;
}

inline bool MoveSegment::IsLast() const noexcept
{
	return GetNext() == nullptr;
}

inline void MoveSegment::SetLinear(float pDistanceLimit, float p_segTime, float p_dDivU) noexcept
{
	endDistanceFraction = pDistanceLimit;
	segTime = p_segTime;
	linear.dDivU = p_dDivU;
	nextAndFlags |= 1u;
}

// Set up an accelerating or decelerating move. We assume that the 'linear' flag is already clear.
inline void MoveSegment::SetNonLinear(float pDistanceLimit, float p_segTime, float p_uDivA, float p_twoDDivA) noexcept
{
	endDistanceFraction = pDistanceLimit;
	segTime = p_segTime;
	quadratic.uDivA = p_uDivA;
	quadratic.twoDDivA = p_twoDDivA;
}

// Release a single MoveSegment. Not thread-safe.
inline void MoveSegment::Release(MoveSegment *item) noexcept
{
	item->nextAndFlags = reinterpret_cast<uint32_t>(freeList);
	freeList = item;
}

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
