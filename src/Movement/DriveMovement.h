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
#include "ExtruderShaper.h"
#include "StepTimer.h"

#if SUPPORT_PHASE_STEPPING
#include <Movement/PhaseStep.h>
#endif

#define STEPS_DEBUG					(1)

class PrepParams;

enum class DMState : uint8_t
{
	idle = 0,
	stepError,

	// All higher values are various states of motion and require interrupts to be generated
	firstMotionState,
	starting = firstMotionState,					// interrupt scheduled for when the move should start
	ending,											// interrupt scheduled for when the move should end
	cartAccel,										// linear accelerating motion
	cartLinear,										// linear steady speed
	cartDecelNoReverse,
	cartDecelForwardsReversing,						// linear decelerating motion, expect reversal
	cartDecelReverse,								// linear decelerating motion, reversed
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	phaseStepping,
#endif
};


// This class describes a single movement of one drive
class DriveMovement
{
public:
	friend class Move;

	DriveMovement() noexcept { }
	void Init(size_t drv) noexcept;

	bool CalcNextStepTime(uint32_t now) noexcept SPEED_CRITICAL;

	void DebugPrint() const noexcept;
	bool StopLogicalDrive(int32_t& netStepsTaken, uint32_t when) noexcept;	// if the driver is moving, stop it, update the position and pass back the net steps taken
#if SUPPORT_REMOTE_COMMANDS
	void StopDriverFromRemote() noexcept;
#endif
	int32_t GetNetStepsTakenThisSegment() const noexcept;				// return the number of steps taken in the current segment
	int32_t GetNetStepsTakenThisMove(uint32_t when) const noexcept;		// return the number of steps taken in the current move, only valid for isolated moves
	void SetMotorPosition(int32_t pos) noexcept;
	bool MotionPending() const noexcept { return segments != nullptr; }
	bool IsPrintingExtruderMovement() const noexcept;					// returns true if this is an extruder executing a printing move
	bool CheckingEndstops() const noexcept;								// returns true when executing a move that checks endstops or Z probe

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t microstepShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

	float GetCurrentPosition(uint32_t when) const noexcept;
#if SUPPORT_PHASE_STEPPING
	bool SetStepMode(StepMode mode) noexcept;
	StepMode GetStepMode() const noexcept { return stepMode; }
	bool IsPhaseStepEnabled() const noexcept { return stepMode == StepMode::phase; }
	bool UpdateCurrentMotion(uint32_t when, MotionParameters& mParams) noexcept;
#endif

	void ClearMovementPending() noexcept;

	bool HasError() const noexcept { return state != DMState::idle && state < DMState::firstMotionState; }

	static void DiagnosticHeader(const StringRef& reply) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;

	static int32_t GetAndClearMaxStepsLate() noexcept;
	static int32_t GetAndClearMinStepInterval() noexcept;

private:
	bool CalcNextStepTimeFull(uint32_t now) noexcept SPEED_CRITICAL;
#if SUPPORT_CAN_EXPANSION
	void TakeStepsAndCalcStepTimeRarely(uint32_t clocksNow) noexcept SPEED_CRITICAL;
#endif
	MoveSegment *_ecv_null NewSegment(uint32_t now) noexcept SPEED_CRITICAL;
	bool ScheduleFirstSegment() noexcept;

	void RetireSegment(MoveSegment *oldSegment) noexcept;							// retire the current segment but keep it available temporarily for debugging
	bool LogStepError(uint8_t type, float info, const MoveSegment *seg) noexcept;	// record a step error

	motioncalc_t GetStepsTakenThisSegment(uint32_t when) const noexcept;

#if SUPPORT_S_CURVE
	void UpdateSpeedAndAccelerationChange(motioncalc_t newSpeed, motioncalc_t speedChange, motioncalc_t newAcc, motioncalc_t accChange) noexcept;
	void MovementStopped() noexcept;
	void PrintRetiredSegment() const noexcept;
#endif

#if CHECK_SEGMENTS
	void CheckSegment(unsigned int line, MoveSegment *seg) noexcept;
#endif

	static int32_t maxStepsLate;
	static int32_t minStepInterval;

	// Parameters common to Cartesian, delta and extruder moves

	DriveMovement *_ecv_null nextDM ;							// link to next DM that needs a step
	MoveSegment *volatile _ecv_null segments = nullptr;			// pointer to the segment list for this driver
	MoveSegment *volatile _ecv_null retiredSegment = nullptr;	// the most recent segment we retired

	ExtruderShaper extruderShaper;						// pressure advance control

	DMState state;										// whether this is active or not
	uint8_t drive;										// the drive that this DM controls
	bool direction;										// true=forwards, false=backwards
	bool directionChanged;								// set by CalcNextStepTime if the direction is changed
	uint8_t stepsTakenThisSegment;						// how many steps we have taken this phase, counts from 0 to 2. Last field in the byte so that we can increment it efficiently.
	uint8_t stepsTillRecalc;							// how soon we need to recalculate. Use the top 2 bits of the byte so that we can increment it efficiently.

	int32_t netStepsThisSegment;						// the (signed) net number of steps in the current segment
	int32_t segmentStepLimit;							// the first step number of the next phase, or the reverse start step if smaller
	int32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	motioncalc_t q, t0, p;								// the movement parameters of the current segment. Only set when not phase stepping
	motioncalc_t u;										// the initial speed of the current segment. Only set when phase stepping, or when 3rd order motion is supported.
#if SUPPORT_PHASE_STEPPING
	motioncalc_t phaseStepsTakenSinceMoveStart;			// how many steps we took in previous segments of the current isolated move
#endif
	MovementFlags segmentFlags;							// whether this segment checks endstops etc.
	motioncalc_t distanceCarriedForwards;				// the residual distance in microsteps (less than one) that was pending at the end of the previous segment

	int32_t currentMotorPosition;						// the current motor position in microsteps
	int32_t positionAtSegmentStart;						// the value of currentMotorPosition at the start of the current segment
	int32_t positionAtMoveStart;						// the position at the start of the current move, if it is an isolated move
#if STEPS_DEBUG
	motioncalc_t positionRequested;						// accumulated position changes requested by moves executed - caution, the step ISR modifies this!
#endif

	// These values change as the segment is executed
	int32_t nextStep;									// number of steps already done. For extruders this gets reset to the net steps already done at the start of each segment, so it can go negative.
	uint32_t nextStepTime;								// when the next step is due
	uint32_t stepInterval;								// how many clocks between steps

	uint32_t driversNormallyUsed;						// the local drivers that this axis or extruder uses
	uint32_t driversCurrentlyUsed;						// the bitmap of local drivers for this axis or extruder that we should step when the next step interrupt is due
	uint32_t driverEndstopsTriggeredAtStart;			// which drivers have endstops that are triggered at the start of the move

	std::atomic<int32_t> movementAccumulator;			// the accumulated movement in microsteps since GetAccumulatedMovement was last called. Only used for extruders.
	uint32_t extruderPrintingSince;						// the millis ticks when this extruder started doing printing moves

#if SUPPORT_S_CURVE
	motioncalc_t finalSpeed, finalAcc;					// the final speed and acceleration of the current segment
	motioncalc_t peakDeltaV, peakDeltaA;				// For debugging: the maximum instantaneous speed change and acceleration change recorded
#endif

#if SUPPORT_PHASE_STEPPING
	PhaseStep phaseStepControl;
	StepMode stepMode;
#endif

	bool isExtruder;									// true if this is an extruder, false it it is an axis
	bool extruderPrinting;								// true if this is an extruder and the most recent segment started was a printing move
};

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1 and state == DMState::idle.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTime(uint32_t now) noexcept
{
	// We have just taken a step, so update the current motor position
	const int32_t adjustment = (int32_t)(direction << 1) - 1;	// to avoid a conditional jump, calculate +1 or -1 according to direction
	currentMotorPosition += adjustment;					// adjust the current position

	++nextStep;
	if (stepsTillRecalc != 0)
	{
		--stepsTillRecalc;								// we are doing double/quad/octal stepping
		nextStepTime += stepInterval;
#ifdef DUET3_MB6HC										// we need to increase the minimum step pulse length to be long enough for the TMC5160
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
#endif
		return true;
	}
	return CalcNextStepTimeFull(now);
}

// Return the number of net steps already taken for the current segment in the forwards direction. Used for filament monitoring.
// Caller must disable interrupts before calling this
inline int32_t DriveMovement::GetNetStepsTakenThisSegment() const noexcept
{
#if SUPPORT_PHASE_STEPPING
	if (phaseStepControl.IsEnabled())
	{
		return std::lrint(GetStepsTakenThisSegment(StepTimer::GetMovementTimerTicks()));
	}
#endif
	return currentMotorPosition - positionAtSegmentStart;
}

// Return the number of net steps already taken for the current move in the forwards direction. Used for moves that are stopped by endstops or a Z probe.
// Only valid for isolated moves. Caller must disable interrupts before calling this.
inline int32_t DriveMovement::GetNetStepsTakenThisMove(uint32_t when) const noexcept
{
	return (int32_t)GetCurrentPosition(when) - positionAtMoveStart;
}

// Return true if this is an extruder executing a printing move
// Call must disable interrupts before calling this
inline bool DriveMovement::IsPrintingExtruderMovement() const noexcept
{
	return !segmentFlags.nonPrintingMove;
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

// Clear any pending movement. This is called for extruders, mostly as an aid to debugging.
// Don't clear the extrusion pending if movement is in progress because this may lead to distanceCarriedForwards becoming out of range, resulting in step errors.
inline void DriveMovement::ClearMovementPending() noexcept
{
	AtomicCriticalSectionLocker lock;
	if (state == DMState::idle)
	{
		distanceCarriedForwards = 0.0;
	}
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder, or zero if no motion in progress
inline uint32_t DriveMovement::GetStepInterval(uint32_t microstepShift) const noexcept
{
	return (segments == nullptr || ((uint32_t)nextStep >> microstepShift) == 0) ? 0
			: stepInterval << microstepShift;									// return the interval between steps converted to full steps
}

#endif

/**
 * @brief Get the current position relative to the start of this segment. Units are microsteps and step clocks.
 * @param when step clock time at which to evaluate the motion. Because the function only reads the first segment this should be the current time.
 * @return position of the dm in microsteps
 */
inline float DriveMovement::GetCurrentPosition(uint32_t when) const noexcept
{
	AtomicCriticalSectionLocker lock;										// we don't want 'segments' changing while we do this

	const MoveSegment* const seg = segments;
	if (seg != nullptr)
	{
		int32_t timeSinceStart = (int32_t)(when - seg->GetStartTime());
		if (timeSinceStart >= 0)
		{
			if ((uint32_t)timeSinceStart >= seg->GetDuration())				// if segment should have finished by now
			{
				// We can't get the next segment because that needs `NewSegment()` to be called
				timeSinceStart = seg->GetDuration();
			}

			return (float)((u + 0.5 * seg->GetA() * timeSinceStart) * timeSinceStart
							  + (motioncalc_t)positionAtSegmentStart + distanceCarriedForwards
						  );
		}

		// If we get here then we have been asked for the position before the current segment started
		return (float)((motioncalc_t)positionAtSegmentStart + distanceCarriedForwards);
	}

	// If we get here then no movement is taking place
	return (float)((motioncalc_t)currentMotorPosition + distanceCarriedForwards);
}

#if SUPPORT_PHASE_STEPPING

/**
 * @brief Get the current position relative to the start of this segment, speed and acceleration. Units are microsteps
 * and step clocks.
 * @param when step clock time at which to evaluate the motion.
 * @param mParams [out] structure to receive the position, speed and acceleration
 * @return true if this drive is moving
 *
 * @note Segments are advanced as necessary if we are phase stepping.
 */
inline bool DriveMovement::UpdateCurrentMotion(uint32_t when, MotionParameters& mParams) noexcept
{
	bool hasMotion = false;
	AtomicCriticalSectionLocker lock;									// we don't want 'segments' changing while we do this

	if (state == DMState::phaseStepping)
	{
		MoveSegment *_ecv_null seg = segments;
		while (seg != nullptr)
		{
			int32_t timeSinceStart = (int32_t)(when - seg->GetStartTime());
			if (timeSinceStart < 0)
			{
				break;													// segment isn't due to start yet
			}
			if ((uint32_t)timeSinceStart >= seg->GetDuration())			// if segment should have finished by now
			{
				if (phaseStepControl.IsEnabled())
				{
					currentMotorPosition = positionAtSegmentStart + netStepsThisSegment;
					phaseStepsTakenSinceMoveStart += seg->GetLength();
					movementAccumulator += netStepsThisSegment;			// update the amount of extrusion

					motioncalc_t provisionalDistanceCarriedForwards = distanceCarriedForwards + seg->GetLength() - (motioncalc_t)netStepsThisSegment;
					if (seg->GetNext() == nullptr && !seg->GetFlags().isExtruder)
					{
						// This is an axis and there are no further segments, so we may need to round the current position to the nearest microstep
						if (std::fabs(provisionalDistanceCarriedForwards) < (motioncalc_t)0.05)
						{
							provisionalDistanceCarriedForwards = (motioncalc_t)0.0;						// just remove the rounding error
						}
						else if (provisionalDistanceCarriedForwards > (motioncalc_t)0.95)
						{
							++currentMotorPosition;														// round up to next microstep
							provisionalDistanceCarriedForwards = (motioncalc_t)0.0;
						}
						else if (provisionalDistanceCarriedForwards < -(motioncalc_t)0.95)
						{
							--currentMotorPosition;														// round down to next position
							provisionalDistanceCarriedForwards = (motioncalc_t)0.0;
						}
					}
					distanceCarriedForwards = provisionalDistanceCarriedForwards;

					MoveSegment *oldSeg = _ecv_not_null(seg);
					segments = oldSeg->GetNext();
					RetireSegment(oldSeg);
					seg = NewSegment(when);
					hasMotion = true;
					continue;
				}
				timeSinceStart = seg->GetDuration();
			}

#if SUPPORT_S_CURVE
			const float rawPosition = (float)((u + ((motioncalc_t)0.5 * seg->GetA() + OneSixth * seg->GetJ() * timeSinceStart) * timeSinceStart) * timeSinceStart
										+ (motioncalc_t)positionAtSegmentStart + distanceCarriedForwards);
#else
			const float rawPosition = (float)((u + (motioncalc_t)0.5 * seg->GetA() * timeSinceStart) * timeSinceStart
										+ (motioncalc_t)positionAtSegmentStart + distanceCarriedForwards);
#endif
			currentMotorPosition = (int32_t)rawPosition;												// store the approximate position for OM updates
			mParams.position = rawPosition;
#if SUPPORT_S_CURVE
			mParams.speed = (float)(u + (seg->GetA() + (motioncalc_t)0.5 * seg->GetJ() * timeSinceStart) * timeSinceStart);
			mParams.acceleration = (float)(seg->GetA() + seg->GetJ() * timeSinceStart);
#else
			mParams.speed = (float)(u + seg->GetA() * timeSinceStart);
			mParams.acceleration = (float)seg->GetA();
#endif
			return true;
		}
	}

	// If we get here then no movement is taking place
	mParams.position = (float)((motioncalc_t)currentMotorPosition + distanceCarriedForwards);
	mParams.speed = mParams.acceleration = 0.0;
	return hasMotion;
}

#endif	// SUPPORT_PHASE_STEPPING

#endif /* DRIVEMOVEMENT_H_ */
