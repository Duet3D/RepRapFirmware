/*
 * MoveSegment.h
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 *
 * This class holds the parameters of a segment of a move with constant acceleration.
 * In order to handle input shaping we need to superimpose segments. This means we have to store the basic parameters.
 * The distance travelled when acceleration is a and initial speed is u is:
 *
 *		s = u*t + 0.5*a*t^2
 *
 * After n steps we want to achieve this distance plus any outstanding movement when the move started. So if q is the mm per step then:
 *
 * 		n*q = s0 + u*t + 0.5*a*t^2
 *
 * The segment parameters are therefore s0, u and a. We also store the start time t0 and the segment duration td.
 * We can superimpose two segments that start at the same times t0 by adding the s0, u and a parameters.
 * If the segments start and/or end at different times then we must split one or both into two or three segments so that we can superimpose segments with the same times.
 */

#ifndef SRC_MOVEMENT_MOVESEGMENT_H_
#define SRC_MOVEMENT_MOVESEGMENT_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>
#include <new>		// for align_val_t

#define SEGMENT_DEBUG	(0)
#define CHECK_SEGMENTS	(0)

// This bit field is used in multiple contexts so that we can copy them efficiently from one context to another Not all flags are used in all contexts.
union MovementFlags
{
	uint32_t all;												// this is to provide a means to clear all the flags in one go
	struct
	{
		uint32_t nonPrintingMove : 1,							// true if the move that generated this segment does not have both forwards extrusion and associated axis movement; used for filament monitoring
				 checkEndstops : 1,								// true if we need to check endstops or Z probe while executing this segment
				 noShaping : 1,									// true if input shaping should be disabled for this move
				 executing : 1;									// normally clear, set in a MoveSegment when the move starts to be executed
	};

	constexpr void Clear() noexcept { all = 0; }

	constexpr void Init() noexcept { all = 0; nonPrintingMove = true; }

	// This operator sets checkingEndstops if either of the segments to be combined checks endstops, and sets nonPrintingMove if either of them is a non printing move
	MovementFlags operator|(const MovementFlags other) const noexcept
	{
		MovementFlags ret;
		ret.all = all | other.all;
		return ret;
	}

	MovementFlags& operator|=(const MovementFlags other) noexcept
	{
		all |= other.all;
		return *this;
	}
};

// This class stores the characteristics of a segment of a move with constant acceleration.
// The characteristics stored are the start time in step clocks, the duration in step clocks, the distance moved in steps, the acceleration, and some flags.
// We no longer store the initial speed because it can be calculated from the duration, distance and acceleration.
class MoveSegment
{
public:
	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	// Read the values of the flag bits
	bool IsLinear() const noexcept { return a == 0; }		//TODO: should we ignore very small accelerations, to avoid rounding error in the calculation?
	MovementFlags GetFlags() const noexcept { return flags; }

#if 0 //SUPPORT_REMOTE_COMMANDS
	bool IsRemote() const noexcept { return isRemote; }
#endif

	// Given that this is not a constant-speed segment, test whether it is accelerating or decelerating
	bool IsAccelerating() const noexcept { return a > (motioncalc_t)0.0; }

	// Get the segment start time in step clocks
	uint32_t GetStartTime() const noexcept { return startTime; }

	// Get the segment duration in step clocks
	uint32_t GetDuration() const noexcept { return duration; }

	// Get the initial speed
	motioncalc_t CalcU() const noexcept { return distance/(motioncalc_t)duration - 0.5 * a * (motioncalc_t)duration; }

	// Get the reciprocal of the initial speed assuming this move has no acceleration
	motioncalc_t CalcLinearRecipU() const noexcept pre(a == 0.0) { return (motioncalc_t)duration/distance; }

	// Get the acceleration
	motioncalc_t GetA() const noexcept { return a; }

	// Get the length
	motioncalc_t GetLength() const noexcept { return distance; }

	// Set the parameters of this segment
	void SetParameters(uint32_t p_startTime, uint32_t p_duration, motioncalc_t p_distance, motioncalc_t p_a, MovementFlags p_flags) noexcept;

	// Split this segment in two, returning a pointer to the second part
	MoveSegment *Split(uint32_t firstDuration) noexcept pre(firstDuration < duration);

	// Merge the parameters for another segment with the same start time and duration into this one
	void Merge(motioncalc_t p_distance, motioncalc_t p_a, MovementFlags p_flags) noexcept;

	// Normalise this segment by removing very small accelerations that cause problems, update t0, return true if it is linear
	bool NormaliseAndCheckLinear(motioncalc_t distanceCarriedForwards, motioncalc_t& t0) noexcept;

	// Set the 'executing' bit in the flags
	void SetExecuting() noexcept { flags.executing = true; }

	// Get the next segment in this list
	MoveSegment *GetNext() const noexcept;

	// Set the next segment in this list
	void SetNext(MoveSegment *p_next) noexcept;

	// Print this segment to the debug channel
	void DebugPrint() const noexcept;

	// Print list of segments
	static void DebugPrintList(const MoveSegment *segs) noexcept;

	// Allocate a MoveSegment, clearing the flags
	static MoveSegment *Allocate(MoveSegment *p_next) noexcept;

	// Release a MoveSegment
	static void Release(MoveSegment *item) noexcept;

	// Release all MoveSegments in a chain
	static void ReleaseAll(MoveSegment *item) noexcept;

	// Return the number of MoveSegment objects that have been created
	static unsigned int NumCreated() noexcept { return numCreated; }

	static constexpr int32_t MinDuration = 10;				// the minimum duration in movement clock ticks that we consider sensible

protected:
	static MoveSegment *freeList;							// list of recycled segment objects
	static unsigned int numCreated;							// total number of segment objects created

	MoveSegment *next;										// pointer to the next segment
	MovementFlags flags;
	uint32_t startTime;										// when this segment should start, in movement clock ticks
	uint32_t duration;										// the duration of this segment in movement ticks
	motioncalc_t distance;									// the number of steps moved
	motioncalc_t a;											// the acceleration during this segment in steps per movement tick squared

private:
	MoveSegment(MoveSegment *p_next) noexcept;
};

// Create a new one, leaving the flags clear
inline MoveSegment::MoveSegment(MoveSegment *p_next) noexcept
	: next(p_next)
{
	// remaining fields are not initialised
}

// Normalise this segment by removing very small accelerations that cause problems, update t0, return true if it is linear
// Called only from DriveMovement::NewSegment. Speed critical, hence inline and the rather unusual behaviour.
// Returns:
//  true if the segment is constant speed, with t0 = time from start of segment at which the distance would be/will be/would have been zero
//  false if the segment has acceleration or deceleration, with t0 = time from start of segment at which the speed would have been/will be/would be zero
inline bool MoveSegment::NormaliseAndCheckLinear(motioncalc_t distanceCarriedForwards, motioncalc_t& t0) noexcept
{
	if (a != (motioncalc_t)0.0)
	{
		// The move has acceleration or deceleration, but it may be small enough to cause problems with the calculations.
		// The reason is that the step time is calculated as:
		//   time_from_segment_start = t0 +/- sqrt(q - p*n)
		// where q equals t0^2 or something very close to it. This gives rise to two issues:
		// 1. The maximum value that can be represented by a float is a little more than 3.4e38, so t0 values greater than about 1e19 cause trouble when we square them to calculate q.
		// 2. Rounding error may cause large errors in the step time, when t0 can't represented to within a small number of step clocks
		// Issue #2 causes problems when abs(t0) exceeds about 2^24 because then the number of step clocks can't be represented exactly.
		// Here are two possible ways round this:
		// 1. When t0 gets large we could use the Maclaurin expansion of sqrt(q - p*n) to give:
		//    time_from_segment_start ~= p*n/(2 * sqrt(q + p*n))
		// This is accurate to within about 1 clock on the last step N when (p*N)^4 < 8*(q + p*N)^3
		// so approximately when (p*N)^4 < 8*q^3, or very roughly when p*N << q
		// However, using the Maclaurin expansion requires an extra division in each step calculation, which we would prefer to avoid.
		// 2. We can convert the segment to a constant-speed segment, on the assumption that the speed won't change much during it. This is what we currently do.
		const motioncalc_t provisionalT0 = (motioncalc_t)0.5 * (motioncalc_t)duration - distance/(a * (motioncalc_t)duration);
		if (likely(fabsm(provisionalT0) <= 4 * (motioncalc_t)16777216.0))
		{
			t0 = provisionalT0;
			return false;
		}

		// The acceleration/deceleration is small enough to cause calculation problems, so change it to a linear move
		a = (motioncalc_t)0.0;
	}

	// The move is constant speed
	t0 = -distanceCarriedForwards * (motioncalc_t)duration/distance;
	return true;
}

// Release a MoveSegment.  Not thread-safe.
inline void MoveSegment::Release(MoveSegment *item) noexcept
{
	const irqflags_t iflags = IrqSave();
	item->next = freeList;
	freeList = item;
	IrqRestore(iflags);
}

inline MoveSegment *MoveSegment::GetNext() const noexcept
{
	return next;
}

inline void MoveSegment::SetNext(MoveSegment *p_next) noexcept
{
	next = p_next;
}

// Set the parameters of this segment
inline void MoveSegment::SetParameters(uint32_t p_startTime, uint32_t p_duration, motioncalc_t p_distance, motioncalc_t p_a, MovementFlags p_flags) noexcept
{
	startTime = p_startTime;
	duration = p_duration;
	distance = p_distance;
	a = p_a;
	flags = p_flags;
}

// Split this segment in two, returning a pointer to the new second part
inline MoveSegment *MoveSegment::Split(uint32_t firstDuration) noexcept
{
	MoveSegment *const secondSeg = Allocate(next);
	const motioncalc_t firstDistance = (CalcU() + (motioncalc_t)0.5 * a * (motioncalc_t)firstDuration) * (motioncalc_t)firstDuration;
	secondSeg->SetParameters(startTime + firstDuration, duration - firstDuration, distance - firstDistance, a, flags);
#if SEGMENT_DEBUG
	debugPrintf("split at %" PRIu32 ", fd=%.2f, sd=%.2f\n", firstDuration, (double)firstDistance, (double)(distance - firstDistance));
#endif
	duration = firstDuration;
	distance = firstDistance;
	next = secondSeg;
	return secondSeg;
}

// Merge the parameters for another segment with the same start time and duration into this one
// s = u*t * 0.5*a*t^2 therefore s1+s2 = (u1+u2)*t + 0.5*(a1+a2)*t^2
inline void MoveSegment::Merge(motioncalc_t p_distance, motioncalc_t p_a, MovementFlags p_flags) noexcept
{
#if SEGMENT_DEBUG
	debugPrintf("merge d=%.2f a=%.4e into ", (double)p_distance, (double)p_a);
	DebugPrint();
#endif
	distance += p_distance;
	a += p_a;
	flags |= p_flags;
}

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
