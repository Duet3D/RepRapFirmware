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
	bool StopLogicalDrive(int32_t& netStepsTaken) noexcept;					// if the driver is moving, stop it, update the position and pass back the net steps taken
#if SUPPORT_REMOTE_COMMANDS
	void StopDriverFromRemote() noexcept;
#endif
	int32_t GetNetStepsTakenThisSegment() const noexcept;				// return the number of steps taken in the current segment
	int32_t GetNetStepsTakenThisMove() const noexcept;					// return the number of steps taken in the current move, only valid for isolated moves
	void SetMotorPosition(int32_t pos) noexcept;
	bool MotionPending() const noexcept { return segments != nullptr; }
	bool IsPrintingExtruderMovement() const noexcept;					// returns true if this is an extruder executing a printing move
	bool CheckingEndstops() const noexcept;								// returns true when executing a move that checks endstops or Z probe

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t microstepShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

#if SUPPORT_PHASE_STEPPING
	bool SetStepMode(StepMode mode) noexcept;
	StepMode GetStepMode() const noexcept { return stepMode; }
	bool IsPhaseStepEnabled() const noexcept { return stepMode == StepMode::phase; }
	// Get the current position relative to the start of this move, speed and acceleration. Units are microsteps and step clocks.
	// Return true if this drive is moving. Segments are advanced as necessary.
	bool GetCurrentMotion(uint32_t when, MotionParameters& mParams) noexcept;
#endif

	void ClearMovementPending() noexcept;

	bool HasError() const noexcept { return state != DMState::idle && state < DMState::firstMotionState; }

	static int32_t GetAndClearMaxStepsLate() noexcept;
	static int32_t GetAndClearMinStepInterval() noexcept;

private:
	bool CalcNextStepTimeFull(uint32_t now) noexcept SPEED_CRITICAL;
#if SUPPORT_CAN_EXPANSION
	void TakeStepsAndCalcStepTimeRarely(uint32_t clocksNow) noexcept SPEED_CRITICAL;
#endif
	MoveSegment *_ecv_null NewSegment(uint32_t now) noexcept SPEED_CRITICAL;
	bool ScheduleFirstSegment() noexcept;

	void ReleaseSegments() noexcept;					// release the list of segments and set it to nullptr
	bool LogStepError(uint8_t type, float info, const MoveSegment *seg) noexcept;	// record a step error

#if SUPPORT_PHASE_STEPPING
	motioncalc_t GetPhaseStepsTakenThisSegment() const noexcept;
#endif

#if CHECK_SEGMENTS
	void CheckSegment(unsigned int line, MoveSegment *seg) noexcept;
#endif

	static int32_t maxStepsLate;
	static int32_t minStepInterval;

	// Parameters common to Cartesian, delta and extruder moves

	DriveMovement *_ecv_null nextDM ;					// link to next DM that needs a step
	MoveSegment *volatile _ecv_null segments;			// pointer to the segment list for this driver

	ExtruderShaper extruderShaper;						// pressure advance control

	DMState state;										// whether this is active or not
	uint8_t drive;										// the drive that this DM controls
	uint8_t direction : 1,								// true=forwards, false=backwards
			directionChanged : 1,						// set by CalcNextStepTime if the direction is changed
							: 1,						// unused
			stepErrorType : 3,							// records what type of step error we had
			stepsTakenThisSegment : 2;					// how many steps we have taken this phase, counts from 0 to 2. Last field in the byte so that we can increment it efficiently.
	uint8_t stepsTillRecalc;							// how soon we need to recalculate. Use the top 2 bits of the byte so that we can increment it efficiently.

	int32_t netStepsThisSegment;						// the (signed) net number of steps in the current segment
	int32_t segmentStepLimit;							// the first step number of the next phase, or the reverse start step if smaller
	int32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	motioncalc_t q, t0, p;								// the movement parameters of the current segment. Only set when not phase stepping
#if SUPPORT_PHASE_STEPPING
	motioncalc_t u;										// the initial speed of this segment. Only set when in phase stepping
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

	bool extruderPrinting;								// true if this is an extruder and the most recent segment started was a printing move

#if SUPPORT_PHASE_STEPPING
	PhaseStep phaseStepControl;
	StepMode stepMode;
#endif
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
		return lrintf(GetPhaseStepsTakenThisSegment());
	}
#endif
	return currentMotorPosition - positionAtSegmentStart;
}

// Return the number of net steps already taken for the current move in the forwards direction. Used for moves that are stopped by endstops or a Z probe.
// Only valid for isolated moves. Caller must disable interrupts before calling this.
inline int32_t DriveMovement::GetNetStepsTakenThisMove() const noexcept
{
#if SUPPORT_PHASE_STEPPING
	if (phaseStepControl.IsEnabled())
	{
		return (int32_t)(GetPhaseStepsTakenThisSegment() + phaseStepsTakenSinceMoveStart);
	}
#endif
	return currentMotorPosition - positionAtMoveStart;
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

#if SUPPORT_PHASE_STEPPING

// Get the current position relative to the start of this segment, speed and acceleration. Units are microsteps and step clocks.
// Return true if this drive is moving. Segments are advanced as necessary if we are in closed loop mode.
// Inlined because it is only called from one place
inline bool DriveMovement::GetCurrentMotion(uint32_t when, MotionParameters& mParams) noexcept
{
	bool hasMotion = false;
	AtomicCriticalSectionLocker lock;									// we don't want 'segments' changing while we do this

	if (state == DMState::phaseStepping)
	{
		MoveSegment *seg = segments;
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
					distanceCarriedForwards += seg->GetLength() - (motioncalc_t)netStepsThisSegment;
					phaseStepsTakenSinceMoveStart += seg->GetLength();
					movementAccumulator += netStepsThisSegment;		// update the amount of extrusion
					MoveSegment *oldSeg = seg;
					segments = oldSeg->GetNext();
					MoveSegment::Release(oldSeg);
					seg = NewSegment(when);
					hasMotion = true;
					continue;
				}
				timeSinceStart = seg->GetDuration();
			}

#if SUPPORT_S_CURVE
			mParams.position = (float)((u + (0.5 * seg->GetA() + OneSixth * seg->GetJ() * timeSinceStart) * timeSinceStart) * timeSinceStart + (motioncalc_t)positionAtSegmentStart + distanceCarriedForwards);
#else
			mParams.position = (float)((u + seg->GetA() * timeSinceStart * 0.5) * timeSinceStart + (motioncalc_t)positionAtSegmentStart + distanceCarriedForwards);
#endif
			currentMotorPosition = (int32_t)mParams.position;			// store the approximate position for OM updates
			mParams.speed = (float)(u + seg->GetA() * timeSinceStart);
			mParams.acceleration = (float)seg->GetA();
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
