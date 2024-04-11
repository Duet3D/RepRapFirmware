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
	float duration;											// the duration in step clocks of this segment
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

// For delta movement the carriage height h0+dh is given by:
//  (h0 + dh - z)^2 = L^2 - x^2 - y^2
// where L is the rod length, z0 is the  and x,y,z are the effector X and Y distances from the tower and the Z height
// Substituting x = x0 + fx*ds, y = y0 + fx*dy, z=z0 + fz*ds:
//  (h0 + dh - z0 - fz*ds)^2 = L^2 - (x0 + fx*ds)^2 - (y0 + fy*ds)^2
// Expanding:
//  h0^2 + dh^2 + z0^2 + (fz*ds)^2 + 2*h0*dh - 2*h0*z0 - 2*h0*fz*ds - 2*dh*z0 - 2*dh*fz*ds + 2*z0*fz*ds = L^2 - x0^2 - (fx*ds)^2 - 2*x0*fx*ds - y0^2 - (fy*ds)^2 - 2*y0*fy*ds
// Subtract the following:
//  (h0 - z0)^2 = L^2 + x0^2 + y0^2
// to give:
//  dh^2 + (fz*ds)^2 + 2*h0*dh - 2*h0*fz*ds - 2*dh*z0 - 2*dh*fz*ds + 2*z0*fz*ds = -(fx*ds)^2 - 2*x0*fx*ds - (fy*ds)^2 - 2*y0*fy*ds
// Rearrange:
//  ds^2*(fz^2 + fx^2 + fy^2) + ds*2*(-h0*fz - dh*fz + z0*fz + x0*fx + y0*fy) + 2*dh*(h0 - z0) - dh^2 = 0
// Substituting fx^2 + fy^2 + fz^2 = 1:
//  ds^2 + ds*2*(-h0*fz - dh*fz + z0*fz + x0*fx + y0*fy) + dh*(2*h0 - 2*z0 - dh) = 0
// Solving for ds, this means that the linear distance moved as a function of required carriage height is given by:
//	ds = D * dh + E +/- sqrt(A * dh^2 + B * dh + C)
// where:
ok//	A = -2*(fx^2+fy^2)
//	B = A * h0 - 2*(fy*y0 + fx*x0)*fz
//	C = (fx*x0 + fy*y0 - fz*h0)^2
ok//	D = fz
ok//	E = fz*h0 - fy*y0 - fx*x0 - fz*z0
// where:
//  dh is the increase in carriage height
//  fx, fy and fz are the fraction of the distance moved that are in the x, y and z directions (i.e. [fx,fy,fz] is the direction vector)
//  h0 is the initial carriage height
//  x0, y0 are the initial X and Y offsets from the tower position

class alignas(8) DeltaMoveSegment : public MoveSegment
{
public:
	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	DeltaMoveSegment(MoveSegment *p_next) noexcept;

	float GetDeltaA() const noexcept { return deltaA; }
	float GetDeltaB() const noexcept { return deltaB; }
	float GetDeltaC() const noexcept { return deltaC; }
	float GetDeltaD() const noexcept { return deltaD; }
	float GetDeltaE() const noexcept { return deltaE; }

	void SetDeltaParameters(const float *dv, float initialXoffsetFromTower, float initialYoffsetFromTower, float initialH) noexcept;

	// Allocate a MoveSegment, clearing the flags
	static DeltaMoveSegment *Allocate(MoveSegment *p_next) noexcept;

	// Print the extra bits in a delta move segment
	void DebugPrintDelta() const noexcept;

private:
	float deltaA, deltaB, deltaC, deltaD, deltaE;
};

// Create a new one, leaving the flags clear
inline DeltaMoveSegment::DeltaMoveSegment(MoveSegment *p_next) noexcept : MoveSegment(p_next)
{
	isDelta = 1;
	// remaining fields are not initialised
}

// Set the parameters that are specific to delta movement. Vector dv must be normalised so that the sum of squares of the XYZ components is one.
inline void DeltaMoveSegment::SetDeltaParameters(const float *dv, float initialXoffsetFromTower, float initialYoffsetFromTower, float initialH) noexcept
{
	deltaA = -2 * (fsquare(dv[0]) + fsquare(dv[1]));
	deltaB = deltaA * initialH - 2 * (dv[0] * initialXoffsetFromTower + dv[1] * initialYoffsetFromTower) * dv[2];
	deltaC = fsquare(dv[0] * initialXoffsetFromTower + dv[1] * initialYoffsetFromTower + dv[2] * initialH);
	deltaD = dv[2];
	deltaE = dv[2] * initialH - 2 * (dv[0] * initialXoffsetFromTower + dv[1] * initialYoffsetFromTower);
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
