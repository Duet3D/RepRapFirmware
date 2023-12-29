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

class PrepParams;

enum class DMState : uint8_t
{
	idle = 0,
	stepError1,
	stepError2,
	stepError3,

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
	friend class Move;

	DriveMovement() noexcept;

	bool CalcNextStepTime() noexcept SPEED_CRITICAL;
	bool PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept SPEED_CRITICAL;
#if SUPPORT_LINEAR_DELTA
	bool PrepareDeltaAxis(const DDA& dda, const PrepParams& params) noexcept SPEED_CRITICAL;
#endif
	bool PrepareExtruder(const DDA& dda, const PrepParams& params, float signedEffStepsPerMm) noexcept SPEED_CRITICAL;

	void DebugPrint() const noexcept;
	int32_t GetCurrentMotorPosition() const noexcept;
	bool StopDriver(int32_t& netStepsTaken) noexcept;					// if the driver is moving, stop it, update the position and pass back the net steps taken
#if SUPPORT_REMOTE_COMMANDS
	void StopDriverFromRemote() noexcept;
#endif
	bool GetNetStepsTaken(int32_t& netStepsTaken) const noexcept;		// like stopDriver but doesn't stop the driver
	void SetMotorPosition(int32_t pos) noexcept;
	void AdjustMotorPosition(int32_t adjustment) noexcept;
	bool MotionPending() const noexcept { return segments != nullptr; }
	bool IsPrintingExtruderMovement() const noexcept;					// returns true if this is an extruder executing a printing move
	void AddSegment(uint32_t startTime, uint32_t duration, float distance, float u, float a, bool usePressureAdvance) noexcept;

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t microstepShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

	static int32_t GetAndClearMaxStepsLate() noexcept;

private:
	bool CalcNextStepTimeFull() noexcept SPEED_CRITICAL;
	MoveSegment *NewCartesianSegment() noexcept SPEED_CRITICAL;
	MoveSegment *NewExtruderSegment() noexcept SPEED_CRITICAL;
#if SUPPORT_LINEAR_DELTA
	MoveSegment *NewDeltaSegment() noexcept SPEED_CRITICAL;
#endif

	void CheckDirection(bool reversed) noexcept;
	void ReleaseSegments() noexcept;					// release the list of segments and set it to nullptr

	static int32_t maxStepsLate;

	// Parameters common to Cartesian, delta and extruder moves

	DriveMovement *nextDM;								// link to next DM that needs a step
	MoveSegment *segments = nullptr;					// pointer to the segment list for this driver

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
	int32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement

	// These values change as the step is executed
	int32_t nextStep;									// number of steps already done. For extruders this gets reset to the net steps already done at the start of each segment, so it can go negative.
	int32_t segmentStepLimit;							// the first step number of the next phase, or the reverse start step if smaller
	uint32_t nextStepTime;								// how many clocks after the start of this move the next step is due
	uint32_t stepInterval;								// how many clocks between steps
	float pA, pB, pC;									// the movement parameters of the current segment

	int32_t positionAtMoveStart = 0;					// the microstep position of the motor. If the motor is moving then this is the position at the start of the move.
	float movementAccumulator;							// the accumulated movement since GetAccumulatedMovement was last called. Only used for extruders.

	// Parameters unique to a style of move (Cartesian, delta or extruder). Currently, extruders and Cartesian moves use the same parameters.
	union
	{
#if SUPPORT_LINEAR_DELTA
		/* The equation for distance travelled along the planned straight path in terms of carriage height is:
				s=(sqrt(
						-(dy^2+dx^2)*h^2

						+2*(dy^2+dx^2)*z0*h
						-2*dy*dz*(y0-yt)*h
						-2*dx*dz*(x0-xt)*h

						+2*dx*dz*(x0-xt)*z0
						+2*dy*dz*(y0-yt)*z0
						+2*dx*dy*(x0-xt)*(y0-yt)

						-(dy^2+dx^2)*z0^2
						-(dz^2+dx^2)*(y0-yt)^2
						-(dz^2+dy^2)*(x0-xt)^2
						+L^2*(dz^2+dy^2+dx^2)
					   )
				   -dy*(y0-yt)
				   -dx*(x0-xt)
				   +dz*h
				   -dz*z0
			  	  )/(dz^2+dy^2+dx^2)

			which is of the form:
		  	  	 s = (sqrt(A*h^2 + B*h + C) + D*h + E)/F
			which can be normalised by incorporating F into the other constants.

			We can also express it in terms of (h-z0):
				s=(sqrt(
					  	-(dy^2+dx^2)*(h-z0)^2

						-2*dy*dz*(y0-yt)*(h-z0)
						-2*dx*dz*(x0-xt)*(h-z0)

						+2*dx*dy*(x0-xt)*(y0-yt)

						-(dz^2+dx^2)*(y0-yt)^2
						-(dz^2+dy^2)*(x0-xt)^2
						+L^2*(dz^2+dy^2+dx^2)
					   )
				   -dy*(y0-yt)
				   -dx*(x0-xt)
				   +dz*(h-z0)
			  	  )/(dz^2+dy^2+dx^2)
		*/
		struct DeltaParameters							// Parameters for delta movement
		{
			float fTwoA;								// 2*(x0-xt)
			float fTwoB;								// 2*(y0-yt)
			float h0MinusZ0;							// h0-z0
			float fDSquaredMinusAsquaredMinusBsquaredTimesSsquared;
			float fHmz0s;								// the starting height less the starting Z height, multiplied by the Z movement fraction (can go negative)
			float fMinusAaPlusBbTimesS;					// -((x0-xt)*dx + (y0-yt)*dy) * steps_per_mm
			float reverseStartDistance;					// the overall move distance at which movement reversal occurs
		} delta;
#endif

		struct CartesianParameters						// Parameters for Cartesian and extruder movement, including extruder pressure advance
		{
			float pressureAdvanceK;						// how much pressure advance is applied to this move
			float effectiveStepsPerMm;					// the steps/mm multiplied by the movement fraction
			float effectiveMmPerStep;					// reciprocal of steps/mm
		} cart;
	} mp;
};

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1 and state == DMState::idle.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTime() noexcept
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
		if (CalcNextStepTimeFull())
		{
			return true;
		}
	}

	state = DMState::idle;
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
inline bool DriveMovement::GetNetStepsTaken(int32_t& netStepsTaken) const noexcept
{
	if (segments == nullptr)
	{
		return false;
	}

	if (directionReversed)															// if started reverse phase
	{
		netStepsTaken = nextStep - (2 * reverseStartStep) + 1;						// allowing for direction having changed
	}
	else
	{
		netStepsTaken = nextStep - 1;
	}
	if (direction)
	{
		netStepsTaken = -netStepsTaken;
	}
	return true;
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

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder, or zero if no motion in progress
inline uint32_t DriveMovement::GetStepInterval(uint32_t microstepShift) const noexcept
{
	return (segments == nullptr || (nextStep >> microstepShift) == 0) ? 0
			: stepInterval << microstepShift;									// return the interval between steps converted to full steps
}

#endif

#endif /* DRIVEMOVEMENT_H_ */
