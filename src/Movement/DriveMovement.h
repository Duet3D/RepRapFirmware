/*
 * DriveMovement.h
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#ifndef DRIVEMOVEMENT_H_
#define DRIVEMOVEMENT_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>
#include "MoveSegment.h"

class LinearDeltaKinematics;
class PrepParams;

enum class DMState : uint8_t
{
	idle = 0,

	// All states from stepError1 to just less than firstMotionState must be error states, see function DDA::HasStepEreor
	stepError1,
	stepError2,
	stepError3,

	extruderPendingPreparation,						// an extruder that couldn't be fully prepared yet

	// All higher values are various states of motion
	firstMotionState,
	cartAccel = firstMotionState,					// linear accelerating motion
	cartLinear,										// linear steady speed
	cartDecelNoReverse,
	cartDecelForwardsReversing,						// linear decelerating motion, expect reversal
	cartDecelReverse,								// linear decelerating motion, reversed

#if SUPPORT_LINEAR_DELTA
	deltaNormal,									// moving forwards without reversing in this segment, or in reverse
	deltaForwardsReversing,							// moving forwards to start with, reversing before the end of this segment
#endif
};

// This class describes a single movement of one drive
class DriveMovement
{
public:
	friend class DDA;

	DriveMovement(DriveMovement *next) noexcept;

	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	bool CalcNextStepTime(const DDA &dda) noexcept SPEED_CRITICAL;
	bool PrepareCartesianAxis(const DDA& dda) noexcept SPEED_CRITICAL;
#if SUPPORT_LINEAR_DELTA
	bool PrepareDeltaAxis(const DDA& dda, const PrepParams& params) noexcept SPEED_CRITICAL;
#endif
	void PrepareExtruder(const DDA& dda, float signedEffStepsPerMm) noexcept SPEED_CRITICAL;
	bool LatePrepareExtruder(const DDA& dda) noexcept SPEED_CRITICAL;

	void DebugPrint() const noexcept;
	int32_t GetNetStepsTaken() const noexcept;

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t microstepShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

	static void InitialAllocate(unsigned int num) noexcept;
	static unsigned int NumCreated() noexcept { return numCreated; }
	static DriveMovement *Allocate(size_t p_drive) noexcept;
	static void Release(DriveMovement *item) noexcept;
	static int32_t GetAndClearMaxStepsLate() noexcept;
	static int32_t GetAndClearMinStepInterval() noexcept;
	static unsigned int GetAndClearBadSegmentCalcs() noexcept;

private:
	bool CalcNextStepTimeFull(const DDA &dda) noexcept SPEED_CRITICAL;
	bool NewCartesianSegment() noexcept SPEED_CRITICAL;
	bool NewExtruderSegment() noexcept SPEED_CRITICAL;
#if SUPPORT_LINEAR_DELTA
	bool NewDeltaSegment(const DDA& dda) noexcept SPEED_CRITICAL;
#endif

	void CheckDirection(bool reversed) noexcept;

	static DriveMovement *freeList;
	static unsigned int numCreated;
	static int32_t maxStepsLate;
	static unsigned int badSegmentCalcs;
	static int32_t minStepInterval;

	// Parameters common to Cartesian, delta and extruder moves

	DriveMovement *nextDM;								// link to next DM that needs a step
	const MoveSegment *currentSegment;

	DMState state;										// whether this is active or not
	uint8_t drive;										// the drive that this DM controls
	uint8_t direction : 1,								// true=forwards, false=backwards
			directionChanged : 1,						// set by CalcNextStepTime if the direction is changed
			directionReversed : 1,						// true if we have reversed the requested motion direction because of pressure advance
			isDelta : 1,								// true if this motor is executing a delta tower move
			isExtruder : 1,								// true if this DM is for an extruder (only matters if !isDelta)
					: 1,								// padding to make the next field last
			stepsTakenThisSegment : 2;					// how many steps we have taken this phase, counts from 0 to 2. Last field in the byte so that we can increment it efficiently.
	uint8_t stepsTillRecalc;							// how soon we need to recalculate

	int32_t totalSteps;									// total number of steps for this move, always positive, but zero for extruders

	// These values change as the step is executed, except for reverseStartStep
	int32_t nextStep;									// number of steps already done. For extruders this gets reset to the net steps already done at the start of each segment, so it can go negative.
	int32_t segmentStepLimit;							// the first step number of the next phase, or the reverse start step if smaller
	int32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	uint32_t nextStepTime;								// how many clocks after the start of this move the next step is due
	uint32_t stepInterval;								// how many clocks between steps

	float distanceSoFar;								// the accumulated distance at the end of the current move segment
	float timeSoFar;									// the accumulated taken for this current DDA at the end of the current move segment
	float pA, pB, pC;									// the move parameters for the current move segment. pA is not used when performing a move at constant speed.

	// Parameters unique to a style of move (Cartesian, delta or extruder). Currently, extruders and Cartesian moves use the same parameters.
	union
	{
#if SUPPORT_LINEAR_DELTA
		struct DeltaParameters							// Parameters for delta movement
		{
			// The following don't depend on how the move is executed, so they could be set up in Init() if we use fixed acceleration/deceleration
			float fTwoA;
			float fTwoB;
			float h0MinusZ0;							// the height subtended by the rod at the start of the move
			float fDSquaredMinusAsquaredMinusBsquaredTimesSsquared;
			float fHmz0s;								// the starting height less the starting Z height, multiplied by the Z movement fraction (can go negative)
			float fMinusAaPlusBbTimesS;
			float reverseStartDistance;					// the overall move distance at which movement reversal occurs
		} delta;
#endif

		struct CartesianParameters						// Parameters for Cartesian and extruder movement, including extruder pressure advance
		{
			float pressureAdvanceK;						// how much pressure advance is applied to this move
			float effectiveStepsPerMm;					// the steps/mm multiplied by the movement fraction
			float effectiveMmPerStep;					// reciprocal of [the steps/mm multiplied by the movement fraction]
		} cart;
	} mp;
};

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1 and state == DMState::idle.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTime(const DDA &dda) noexcept
{
	++nextStep;
	if (nextStep <= totalSteps || isExtruder)
	{
		if (stepsTillRecalc != 0)
		{
			--stepsTillRecalc;				// we are doing double/quad/octal stepping
			nextStepTime += stepInterval;
#ifdef DUET3_MB6HC							// we need to increase the minimum step pulse length to be long enough for the TMC5160
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
#endif
			return true;
		}
		if (CalcNextStepTimeFull(dda))
		{
			return true;
		}
	}

	if (state >= DMState::firstMotionState)	// don't change the state if there was a step error
	{
		state = DMState::idle;
	}

#ifdef DUET3_MB6HC							// we need to increase the minimum step pulse length to be long enough for the TMC5160
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
			asm volatile("nop");
#endif
	return false;
}

// Return the number of net steps already taken for the move in the forwards direction.
// We have already taken nextSteps - 1 steps
inline int32_t DriveMovement::GetNetStepsTaken() const noexcept
{
	int32_t netStepsTaken;
	if (directionReversed)															// if started reverse phase
	{
		netStepsTaken = nextStep - (2 * reverseStartStep) + 1;						// allowing for direction having changed
	}
	else
	{
		netStepsTaken = nextStep - 1;
	}
	return (direction) ? netStepsTaken : -netStepsTaken;
}

// This is inlined because it is only called from one place
inline void DriveMovement::Release(DriveMovement *item) noexcept
{
	item->nextDM = freeList;
	freeList = item;
}

inline void DriveMovement::CheckDirection(bool reversed) noexcept
{
	if (reversed != directionReversed)
	{
		directionReversed = !directionReversed;										// this can be done by an xor so hopefully more efficient than assignment
		direction = !direction;
		directionChanged = true;
	}
}

inline int32_t DriveMovement::GetAndClearMaxStepsLate() noexcept
{
	const int32_t ret = maxStepsLate;
	maxStepsLate = 0;
	return ret;
}

inline int32_t DriveMovement::GetAndClearMinStepInterval() noexcept
{
	const int32_t ret = minStepInterval;
	minStepInterval = 0;
	return ret;

}

inline unsigned int DriveMovement::GetAndClearBadSegmentCalcs() noexcept
{
	const unsigned int ret = badSegmentCalcs;
	badSegmentCalcs = 0;
	return ret;
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder
inline uint32_t DriveMovement::GetStepInterval(uint32_t microstepShift) const noexcept
{
	return ((nextStep >> microstepShift) != 0)										// if at least 1 full step done
				? stepInterval << microstepShift									// return the interval between steps converted to full steps
					: 0;
}

#endif

#endif /* DRIVEMOVEMENT_H_ */
