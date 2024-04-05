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
 *    scale by 1.0/(f*m) again. We don't need to apply input shaping to extruders because we assume that extruders don't react quickly enough. to acceleration changes lasting up to ~30ms.
 * 2. Use the axis move segments. When starting a new segment, each extruder will need to calculate and store that modified A B C coefficients to take account of PA. When input shaping is used,
 *    shaping will be applied to extruders (so there will be more MoveSegments to process). This would save memory by using fewer MoveSegments, but require more computation.
 * 3. Use a common MoveSegment chain without input shaping for all extruders (which could be shared with the axes when the move does not use input shaping), and adjust the A B C coefficients to account for PA.
 *
 * To handle delta movement, we convert the step count to distance along the overall linear trajectory (accounting for the possible reversal), then convert that distance to time using the axis MoveSegments.
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
 *   t = time since start of move
 *   ts = segment start time
 * The basic motion equation for an acceleration segment is:
 *   s = s0 + u*f*(t - ts) + 0.5*a*f*(t - ts)t^2
 * Solving for t:
 *   t = ts - u/a + sqrt((-u/a)^2 + 2*(s-s0)/af
 * If we take s0 = S0 * f:
 *   t = ts - u/a + sqrt((-u/a)^2 + 2*s/af - 2*S0/a)
 * Substituting s = n/m:
 *   t = ts - u/a + sqrt((-u/a)^2 + 2*n/amf - 2*S0/a)
 * For a deceleration segment, just set a = -d:
 *   t = ts + u/d + sqrt((u/d)^2 - 2*n/dmf + 2*S0/d)
 * For a linear segment:
 *   s = s0 + u*f*(t - ts)
 * from which:
 *   t = ts + (s - s0)/uf
 * If we take s0 = S0 * f:
 *   t = ts - S0/u + s/uf
 * Substituting s = n/m:
 *   t = n/(u*f*m) + ts - S0/u
 *
 * Now add pressure advance with constant k seconds.
 * For the acceleration segment, u is replaced by u+(k*a):
 *   t = ts - u/a - k + sqrt((-u/a - k)^2 + 2*n/amf - 2*S0/a)
 * For the deceleration segment, u is replaced by u-(k*d). Additionally, S0 is replaced by S0 + EAD where EAD stands for
 * extra acceleration distance, EAD = (vaccel-uaccel)*k = a*T*k where T was the acceleration time:
 *   t = ts + u/d - k + sqrt((u/d - k)^2 - 2*n/dmf) + 2*(S0 + EAD)/d)
 * For the linear segment, S0 must include EAD again:
 *   t = ts - (S0 + EAD)/u + n/umf
 *
 * Now assume that there is also an overdue fractional step p brought forward, where -1.0 < p < 1.0. This must be added to s0. Equivalently, add p/mf to S0.
 * Acceleration segment:
 *   t = ts - u/a - k + sqrt((-u/a - k)^2 + 2*n/amf - 2*(S0 + p/mf)/a)
 * Deceleration segment:
 *   t = ts + u/d - k + sqrt((u/d - k)^2 - 2*n/dmf + 2*(S0 + p/mf + EAD)/d)
 * Linear segment:
 *   t = ts - (S0 + p/mf + EAD)/u + n/umf
 *
 * We can summarise like this. For an acceleration or deceleration segment:
 *   t = ts + B + sqrt(A + C*n/mf)
 * where for an acceleration segment:
 *   B = -u/a - k
 *   A = B^2 - 2*(S0 + p/mf)/a = B^2 - C * (S0 + p/mf)
 *   C = 2/a
 * and for a deceleration segment:
 *   B = u/d - k
 *   A = B^2 + 2*(S0 + EAD + p/mf)/d = B^2 - C * (S0 + p/mf)
 *   C = -2/d
 *
 * For a linear segment:
 *   t = ts + B + C*n/mf
 * where:
 *   B = -(S0 + EAD + p/mf)/u
 *   C = 1/u
 *
 * For accelerating moves we can interpret (-A/C) as the (negative) distance at which the move reversed, and B is the time at which it reversed (negative so in the past)
 * For decelerating moves we can interpret (A/C) as the distance at which the move will reverse, B is the time at which it will reverse (positive so in the future)
 *
 * The move segment stores a flag indicating whether the segment is accel/decel or linear, the length of this segment D = (S - S0) + EAD, and the time T for this segment in step clocks.
 * For an accel/decel segment it also stores the coefficients B and C. For a linear segment, just C.
 * From the distance limit we can work out the step limit N = D/(f*m).
 * From the segment times we can accumulate the start time ts of each segment.
 *
 * When starting a new axis segment we calculate actual coefficients A' B' C' as follows (Dprev is the distance limit of the previous segment, i.e. total length of previous segments):
 * For accel/decel segments:
 *   A' = B^2 - C*Dprev
 *   B' = B + ts
 *   C' = C * 1/mf
 * (for axis segments we could instead include ts in B, but we can't do that for common extruder segments, see later)
 * For linear segments:
 *   B' = Dprev*C + ts
 *   C' = C * 1/mf
 *
 * Assuming we share common segments between all extruders and account for PA and fractional steps when starting them, then when starting a segment we need to calculate the coefficients as follows:
 * For accel/decel segments:
 *   A' = (B - k)^2 + C*(Dprev + p/mf)
 *   B' = (B - k) + ts
 *   C' = C * 1/mf
 * For linear segments:
 *   B' = -(Dprev + p/mf)*C + ts
 *   C' = C * 1/mf
 *
 * When preparing a move we need to:
 *   Set up the axis and extruder move segments
 *   For each linear axis and extruder, calculate and store 1/(f*m) and store in the DM
 *   For each delta axis, also calculate and store in the DM additional parameters needed for delta movement
 *   If we use common extruder segments, for each extruder also calculate and store in the DM the following: 2*(p/f)/a, p/f + EAD, 2*(p/f + EAD)/d
 *   If we use common extruder segments, either make sure we have fast access to k, or for each extruder store k
 *
 * When starting a new segment we need to:
 *   Calculate the segment start time ts (by accumulating the durations of previous segments)
 *   Compute A' B' C' for this segment and motor
 *
 * If the MCU supports hardware floating point then it is more efficient to use FP arithmetic because of the speed of FP divide and sqrt operations.
 * If it doesn't then for accel/decel segments we probably need to compute sqrt(A + C*n/(f*m)) using 64-bit maths before we take the sqrt:
 *   A is in step_clocks^2 so it can be stored directly in 64 bits
 *   B is in step_clocks so it can be stored directly in 32 bits
 *   C for accel/decel is in step clocks^2/mm so it can probably be stored directly in 64 bits (check - does this work for very low f ?)
 *   C for linear motion is in step clocks/mm so can be stored directly in 32 bits
 *   1/(f*m) is in mm/step. We will have to store it multiplied by e.g. 2^24 as 32 bits, and after multiplying it by C to get a 64-bit result, shift it right to divide by 2^24.
 */

#ifndef SRC_MOVEMENT_MOVESEGMENT_H_
#define SRC_MOVEMENT_MOVESEGMENT_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>

class MoveSegment
{
public:
	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	MoveSegment(MoveSegment *p_next) noexcept;

	bool IsLinear() const noexcept;
	bool IsAccelerating() const noexcept pre(!IsLinear());
	bool IsLast() const noexcept;

	// Get the segment length in mm
	float GetSegmentLength() const noexcept { return segLength; }

	// Get the segment duration in step clocks
	float GetSegmentTime() const noexcept { return segTime; }

	// Get the segment speed change in mm/step_clock. Only for accelerating or decelerating moves.
	float GetNonlinearSpeedChange() const noexcept pre(!IsLinear()) { return acceleration * segTime; }

	// Get the start speed for an accelerating or decelerating move
	float GetNonlinearStartSpeed(float pressureAdvanceK) const noexcept pre(!IsLinear()) { return (pressureAdvanceK - b) * acceleration; }

	// Get the end speed for an accelerating or decelerating move
	float GetNonlinearEndSpeed(float pressureAdvanceK) const noexcept pre(!IsLinear()) { return (pressureAdvanceK - b + segTime) * acceleration; }

#if 1
	// Functions used to verify segments
	// Get the start speed for any move assuming no pressure advance
	float GetStartSpeed() const noexcept { return (IsLinear()) ? 1.0/c : -b * acceleration; }

	// Get the calculated distance for any move assuming no pressure advance
	float GetCalculatedDistance() const noexcept { return (GetStartSpeed() + 0.5 * acceleration * segTime) * segTime; }
#endif

	// Calculate the move A coefficient in step_clocks^2 for an accelerating or decelerating move
	float CalcNonlinearA(float startDistance) const noexcept pre(!IsLinear());
	float CalcNonlinearA(float startDistance, float pressureAdvanceK) const noexcept pre(!IsLinear());

	// Calculate the move B coefficient in step_clocks for an accelerating or decelerating move
	float CalcNonlinearB(float startTime) const noexcept pre(!IsLinear());
	float CalcNonlinearB(float startTime, float pressureAdvanceK) const noexcept pre(!IsLinear());

	// Calculate the move B coefficient in step_clocks for a steady speed move
	float CalcLinearB(float startDistance, float startTime) const noexcept pre(IsLinear());

	// Calculate the move C coefficient in step_clocks/step for a linear move, or step_clocks^2/step for an accelerating or decelerating move
	float CalcCFromMmPerStep(float mmPerStep) const noexcept;

	// Calculate the move C coefficient in step_clocks/step for a linear move, or step_clocks^2/step for an accelerating or decelerating move
	float CalcCFromStepsPerMm(float stepsPerMm) const noexcept;

	// For a decelerating move, calculate the distance before the move reverses
	float GetDistanceToReverse(float startSpeed) const noexcept;

	void SetLinear(float pSegmentLength, float p_segTime, float p_c) noexcept post(IsLinear());
	void SetNonLinear(float pSegmentLength, float p_segTime, float p_b, float p_c, float p_acceleration) noexcept post(!IsLinear());

	MoveSegment *GetNext() const noexcept;
	void SetNext(MoveSegment *p_next) noexcept;
	void AddToTail(MoveSegment *tail) noexcept;
	void DebugPrint(char ch) const noexcept;
	static void DebugPrintList(char ch, const MoveSegment *segs) noexcept;
	static bool DebugCheckSegments(const MoveSegment *segs) noexcept;

	// Allocate a MoveSegment, clearing the flags
	static MoveSegment *Allocate(MoveSegment *next) noexcept;

	// Release a MoveSegment
	static void Release(MoveSegment *item) noexcept;

	static void InitialAllocate(unsigned int num) noexcept;
	static unsigned int NumCreated() noexcept { return numCreated; }

private:
	// We can store up to 2 flag bits in the link field, because the next move segment in the list will be 4-byte aligned
	static constexpr uint32_t LinearFlag = 0x01;			// set if this segment is linear, clear if it is accelerating or decelerating
	static constexpr uint32_t SpareFlag = 0x02;				// unused flag bit
	static constexpr uint32_t AllFlags = 0x03;

	static MoveSegment *freeList;
	static unsigned int numCreated;

	static_assert(sizeof(MoveSegment*) == sizeof(uint32_t));

	// The 'nextAndFlags' field is a MoveSegment pointer with two flag bits in the bottom two bits
	uint32_t nextAndFlags;									// pointer to the next segment, plus flag bits
	float segLength;										// the length of this segment before applying the movement fraction
	float segTime;											// the time in step clocks at which this move ends
	float c;												// the c move parameter, units are step_clocks/mm for linear moves, units are steps_clocks^2/mm for accelerating or decelerating moves
	float b;												// the b move parameter, equal to -(u/a), units are step_clocks, not used by linear move segments
	float acceleration;										// the acceleration during this segment, not used by linear move segments
};

// Create a new one, leaving the flags clear
inline MoveSegment::MoveSegment(MoveSegment *p_next) noexcept
	: nextAndFlags(reinterpret_cast<uint32_t>(p_next))				// this also clears the flags
{
	// remaining fields are not initialised
}

inline MoveSegment *MoveSegment::GetNext() const noexcept
{
	return reinterpret_cast<MoveSegment*>(nextAndFlags & (~AllFlags));
}

inline void MoveSegment::SetNext(MoveSegment *p_next) noexcept
{
	nextAndFlags = (nextAndFlags & AllFlags) | reinterpret_cast<uint32_t>(p_next);
}

inline bool MoveSegment::IsLinear() const noexcept
{
	return nextAndFlags & LinearFlag;
}

inline bool MoveSegment::IsLast() const noexcept
{
	return GetNext() == nullptr;
}

inline float MoveSegment::CalcNonlinearA(float startDistance) const noexcept
{
	return fsquare(b) - startDistance * c;
}

inline float MoveSegment::CalcNonlinearA(float startDistance, float pressureAdvanceK) const noexcept
{
	return fsquare(b - pressureAdvanceK) - startDistance * c;
}

inline float MoveSegment::CalcNonlinearB(float startTime) const noexcept
{
	return b + startTime;
}

inline float MoveSegment::CalcNonlinearB(float startTime, float pressureAdvanceK) const noexcept
{
	return (b - pressureAdvanceK) + startTime;
}

inline float MoveSegment::CalcLinearB(float startDistance, float startTime) const noexcept
{
	return startTime - (startDistance * c);
}

inline float MoveSegment::CalcCFromMmPerStep(float mmPerStep) const noexcept
{
	return c * mmPerStep;
}

// Calculate the move C coefficient in step_clocks/step for a linear move, or step_clocks^2/step for an accelerating or decelerating move
inline float MoveSegment::CalcCFromStepsPerMm(float stepsPerMm) const noexcept
{
	return c/stepsPerMm;
}

// For a decelerating move with positive start speed, calculate the distance from the start of the segment before the move reverses
// From (v^2-u^2) = 2as, if v=0 then s=-u^2/2a = u^2/2d
// But c = -2/d, so d = -2/c, so s = u^2/(-4/c) = 0.25 * u^2 * c.
inline float MoveSegment::GetDistanceToReverse(float startSpeed) const noexcept
{
	return fsquare(startSpeed) * c * (-0.25);
}

inline void MoveSegment::SetLinear(float pSegmentLength, float p_segTime, float p_c) noexcept
{
	segLength = pSegmentLength;
	segTime = p_segTime;
	b = 0.0;
	c = p_c;
	acceleration = 0.0;
	nextAndFlags |= LinearFlag;
}

// Set up an accelerating or decelerating move. We assume that the 'linear' flag is already clear.
inline void MoveSegment::SetNonLinear(float pSegmentLength, float p_segTime, float p_b, float p_c, float p_acceleration) noexcept
{
	segLength = pSegmentLength;
	segTime = p_segTime;
	b = p_b;
	c = p_c;
	acceleration = p_acceleration;
}

// Given that this is an accelerating or decelerating move, return true if it is accelerating
inline bool MoveSegment::IsAccelerating() const noexcept
{
	return c > 0.0;
}

// Release a single MoveSegment. Not thread-safe.
inline void MoveSegment::Release(MoveSegment *item) noexcept
{
	item->nextAndFlags = reinterpret_cast<uint32_t>(freeList);
	freeList = item;
}

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
