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
 * For delta movement we need to store additional parameters such as the XYZ part of the direction vector and the starting distance to the tower whose motor the segments is for.
 * We also store a flag to say whether pressure advance should be applied.
 */

#ifndef SRC_MOVEMENT_MOVESEGMENT_H_
#define SRC_MOVEMENT_MOVESEGMENT_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>
#include <new>		// for align_val_t

// We have two types of them: this one, and a larger one for delta movement.
class DeltaMoveSegment;

class MoveSegment
{
public:
	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	MoveSegment(MoveSegment *p_next) noexcept;

	// Read the values of the flag bits
	bool IsLinear() const noexcept { return a == 0; }		//TODO: should we ignore very small accelerations, to avoid rounding error in the calculation?
	bool IsDelta() const noexcept;
	bool UsePressureAdvance() const noexcept;
	bool IsRemote() const noexcept;

	// Given that this is not a constant-speed segment, test whether it is accelerating or decelerating
	bool IsAccelerating() const noexcept { return a > 0.0; }

	// Get the number of steps in this segment
	int32_t GetSteps() const noexcept { return steps; }

	// Get the segment duration in step clocks
	float GetStartTime() const noexcept { return startTime; }

	// Get the segment duration in step clocks
	float GetDuration() const noexcept { return duration; }

	// Get the segment speed change in mm/step_clock
	float GetSpeedChange() const noexcept { return a * duration; }

	// Get the distance carries forward from the previous segment
	float GetS0() const noexcept { return s0; }

	// Get the initial speed
	float GetU() const noexcept { return u; }

	// Get the acceleration
	float GetA() const noexcept { return a; }

	// Get the actual initial speed when pressure advance is being applied
	float GetStartSpeed(float pressureAdvanceK) const noexcept { return u + a * pressureAdvanceK; }

	// Get the actual ending speed when pressure advance is being applied
	float GetEndSpeed(float pressureAdvanceK) const noexcept { return u + a * (pressureAdvanceK + duration); }

	// For a decelerating move, calculate the distance before the move reverses
	float GetDistanceToReverse() const noexcept;

	void SetPressureAdvance(bool p_usePressureAdvance) noexcept { usePressureAdvance = p_usePressureAdvance; }

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
	static DeltaMoveSegment *deltaFreeList;
	static unsigned int numCreated;

	MoveSegment *next;										// pointer to the next segment
	uint32_t isDelta : 1,									// set if this is a delta segment
			 usePressureAdvance : 1,						// set if we should apply pressure advance (not applicable to delta segments)
			 initialDirection : 1							// set if the initial direction is forwards
#if SUPPORT_REMOTE_COMMANDS
		   , isRemote : 1									// set if we are in expansion board mode and this segment came from a move commanded by the main board
#endif
			 ;
	uint32_t startTime;										// when this segment should start
	int32_t steps;											// the number of microsteps in this segment
	float duration;											// the duration in step clocks of this segment
	float s0;												// the movement carried forward from the previous segment
	float u;												// the initial speed
	float a;												// the acceleration during this segment
};

// Create a new one, leaving the flags clear
inline MoveSegment::MoveSegment(MoveSegment *p_next) noexcept
	: next(p_next), isDelta(0), usePressureAdvance(0)
#if SUPPORT_REMOTE_COMMANDS
	   , isRemote(0)
#endif
{
	// remaining fields are not initialised
}

inline MoveSegment *MoveSegment::GetNext() const noexcept
{
	return next;
}

inline void MoveSegment::SetNext(MoveSegment *p_next) noexcept
{
	next = p_next;
}

inline bool MoveSegment::IsDelta() const noexcept
{
	return isDelta;
}

inline bool MoveSegment::UsePressureAdvance() const noexcept
{
	return usePressureAdvance;
}

inline bool MoveSegment::IsRemote() const noexcept
{
	return isRemote;
}

// For a decelerating move with positive start speed, calculate the distance before the move reverses
inline float MoveSegment::GetDistanceToReverse() const noexcept
{
	return fsquare(u)/-(2 * a);
}

class alignas(8) DeltaMoveSegment : public MoveSegment
{
public:
	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	DeltaMoveSegment(MoveSegment *p_next) noexcept;

	const float *GetDv() const noexcept { return dv; }
	float GetfTwoA() const noexcept { return fTwoA; }
	float GetfTwoB() const noexcept { return fTwoB; }
	float GetH0MinusZ0() const noexcept { return h0MinusZ0; }
	float GetfMinusAaPlusBbTimesS() const noexcept { return fMinusAaPlusBbTimesS; }
	float GetfDSquaredMinusAsquaredMinusBsquaredTimesSsquared() const noexcept { return fDSquaredMinusAsquaredMinusBsquaredTimesSsquared; }

	void SetDeltaParameters(const float *p_dv, float p_fTwoS, float p_fTwoB, float p_m0MinusZ0, float p_fMinusAaPlusBbTimesS, float p_fDSquaredMinusAsquaredMinusBsquaredTimesSsquared) noexcept;

	// Allocate a MoveSegment, clearing the flags
	static DeltaMoveSegment *Allocate(MoveSegment *p_next) noexcept;

	// Print the extra bits in a delta move segment
	void DebugPrintDelta() const noexcept;

private:
	float fTwoA, fTwoB;
	float h0MinusZ0;
	float fMinusAaPlusBbTimesS;
	float fDSquaredMinusAsquaredMinusBsquaredTimesSsquared;
	float dv[3];			// the XYZ movement fractions
	//TODO
};

// Create a new one, leaving the flags clear
inline DeltaMoveSegment::DeltaMoveSegment(MoveSegment *p_next) noexcept : MoveSegment(p_next)
{
	isDelta = 1;
	// remaining fields are not initialised
}

// Set the parameters that are specific to delta movement
inline void DeltaMoveSegment::SetDeltaParameters(const float *p_dv, float p_fTwoA, float p_fTwoB, float p_h0MinusZ0, float p_fMinusAaPlusBbTimesS, float p_fDSquaredMinusAsquaredMinusBsquaredTimesSsquared) noexcept
{
	dv[0]= p_dv[0];
	dv[1] = p_dv[1];
	dv[2] = p_dv[2];
	fTwoA = p_fTwoA;
	fTwoB = p_fTwoB;
	h0MinusZ0 = p_h0MinusZ0;
	fMinusAaPlusBbTimesS = p_fMinusAaPlusBbTimesS;
	fDSquaredMinusAsquaredMinusBsquaredTimesSsquared = p_fDSquaredMinusAsquaredMinusBsquaredTimesSsquared;
}

// Release a single MoveSegment. Not thread-safe.
inline void MoveSegment::Release(MoveSegment *item) noexcept
{
	if (item->IsDelta())
	{
		item->next = deltaFreeList;
		deltaFreeList = (DeltaMoveSegment*)item;
	}
	else
	{
		item->next = freeList;
		freeList = item;
	}
}

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
