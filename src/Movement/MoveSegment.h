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

class MoveSegment
{
public:
	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	// Read the values of the flag bits
	bool IsLinear() const noexcept { return a == 0; }		//TODO: should we ignore very small accelerations, to avoid rounding error in the calculation?
	bool IsPrintingMove() const noexcept { return isPrintingMove; }

#if SUPPORT_REMOTE_COMMANDS
	bool IsRemote() const noexcept { return isRemote; }
#endif

	// Given that this is not a constant-speed segment, test whether it is accelerating or decelerating
	bool IsAccelerating() const noexcept { return a > 0.0; }

	// Get the segment duration in step clocks
	float GetStartTime() const noexcept { return startTime; }

	// Get the segment duration in step clocks
	float GetDuration() const noexcept { return duration; }

	// Get the initial speed
	float GetU() const noexcept { return u; }

	// Get the acceleration
	float GetA() const noexcept { return a; }

	// Get the actual initial speed when pressure advance is being applied
	float GetStartSpeed(float pressureAdvanceK) const noexcept { return u + a * pressureAdvanceK; }

	// Get the actual ending speed when pressure advance is being applied
	float GetEndSpeed(float pressureAdvanceK) const noexcept { return u + a * (pressureAdvanceK + duration); }

	// Get the length
	float GetLength() const noexcept { return  (u + 0.5 * a * duration) * duration; }

	// For a decelerating move, calculate the distance before the move reverses
	float GetDistanceToReverse() const noexcept;

	MoveSegment *GetNext() const noexcept;
	void SetNext(MoveSegment *p_next) noexcept;
	void AddToTail(MoveSegment *tail) noexcept;
	void DebugPrint(char ch) const noexcept;
	static void DebugPrintList(char ch, const MoveSegment *segs) noexcept;

	// Allocate a MoveSegment, clearing the flags
	static MoveSegment *Allocate(MoveSegment *p_next) noexcept;

	// Release a MoveSegment or a DeltaMoveSegment
	static void Release(MoveSegment *item) noexcept;

	static unsigned int NumCreated() noexcept { return numCreated; }

protected:
	static MoveSegment *freeList;
	static unsigned int numCreated;

	MoveSegment *next;										// pointer to the next segment
	uint32_t initialDirection : 1,							// set if the initial direction is forwards
			 isPrintingMove : 1								// for extruder segments, indicates whether this is a printing move (i.e. forwards and with associated axis movement)
#if SUPPORT_REMOTE_COMMANDS
		   , isRemote : 1									// set if we are in expansion board mode and this segment came from a move commanded by the main board
#endif
			 ;
	uint32_t startTime;										// when this segment should start, in step clock ticks
	float duration;											// the duration in ticks of this segment
	float u;												// the initial speed in steps per tick
	float a;												// the acceleration during this segment in steps per tick squared

private:
	MoveSegment(MoveSegment *p_next) noexcept;
};

// Create a new one, leaving the flags clear
inline MoveSegment::MoveSegment(MoveSegment *p_next) noexcept
	: next(p_next)
#if SUPPORT_REMOTE_COMMANDS
	   , isRemote(0)
#endif
{
	// remaining fields are not initialised
}

// Release a MoveSegment.  Not thread-safe.
inline void MoveSegment::Release(MoveSegment *item) noexcept
{
	item->next = freeList;
	freeList = item;
}

inline MoveSegment *MoveSegment::GetNext() const noexcept
{
	return next;
}

inline void MoveSegment::SetNext(MoveSegment *p_next) noexcept
{
	next = p_next;
}

// For a decelerating move with positive start speed, calculate the distance before the move reverses
inline float MoveSegment::GetDistanceToReverse() const noexcept
{
	return fsquare(u)/-(2 * a);
}

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
