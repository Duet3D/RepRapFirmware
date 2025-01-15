/*
 * DriveMovement.cpp
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#include "DriveMovement.h"
#include "MoveTiming.h"
#include "DDA.h"
#include "Move.h"
#include "MoveDebugFlags.h"
#include "StepTimer.h"
#include <Math/Isqrt.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

#if SUPPORT_TMC51xx
# include "Movement/StepperDrivers/TMC51xx.h"
#endif

// Static members

int32_t DriveMovement::maxStepsLate = 0;
int32_t DriveMovement::minStepInterval = 0;

// Non static members

void DriveMovement::Init(size_t drv) noexcept
{
	drive = (uint8_t)drv;
	state = DMState::idle;
	stepErrorType = 0;
	distanceCarriedForwards = 0.0;
	currentMotorPosition = positionAtSegmentStart = 0;
	movementAccumulator = 0;
	extruderPrinting = false;
#if STEPS_DEBUG
	positionRequested = 0;
#endif
	driversNormallyUsed = driversCurrentlyUsed = driverEndstopsTriggeredAtStart = 0;
	nextDM = nullptr;
	segments = nullptr;
	isExtruder = false;
	segmentFlags.Init();

#if SUPPORT_PHASE_STEPPING
	stepMode = StepMode::stepDir;
#endif
}

void DriveMovement::DebugPrint() const noexcept
{
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
		debugPrintf("DM%c state=%u err=%u dir=%c next=%" PRIi32 " rev=%" PRIi32 " ssl=%" PRIi32 " sns=%" PRIi32 " interval=%" PRIu32 " q=%.4g t0=%.4g p=%.4g dcf=%.2f\n",
						c, (unsigned int)state, (unsigned int)stepErrorType, (direction) ? 'F' : 'B',
							nextStep, reverseStartStep, segmentStepLimit, netStepsThisSegment, stepInterval,
								(double)q, (double)t0, (double)p, (double)distanceCarriedForwards);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Set the position of a motor. Only call this when the motor is not moving.
void DriveMovement::SetMotorPosition(int32_t pos) noexcept
{
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintTransforms))
	{
		debugPrintf("Changing drive %u pos from %" PRIi32 " to %" PRIi32 "\n", drive, currentMotorPosition, pos);
	}
	currentMotorPosition = positionAtSegmentStart = pos;
#if STEPS_DEBUG
	positionRequested = (float)pos;
#endif
	ClearMovementPending();
	movementAccumulator.store(0);
	extruderPrinting = false;
}

// Set up to schedule the first segment, returning true if an interrupt for this DM is needed
bool DriveMovement::ScheduleFirstSegment() noexcept
{
	directionChanged = true;						// force the direction to be set
	const uint32_t now = StepTimer::GetMovementTimerTicks();
	if (NewSegment(now) != nullptr)
	{
		if (state == DMState::starting)
		{
			return true;
		}
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		if (state == DMState::phaseStepping)
		{
			return false;
		}
#endif
		return CalcNextStepTimeFull(now);
	}
	return false;
}

// This is called when we need to examine the segment list and prepare the head segment (if there is one) for execution.
// If there is no segment to execute, set our state to 'idle' and return nullptr.
// If there is a segment to execute but it isn't due to start for a while, set our state to 'starting', set nextStepTime to when the move is due to start or shortly before,
// set driversCurrentlyUsed to 0 to suppress the step pulse, and return the segment.
// If there is a segment ready to execute and it has steps, set up our movement parameters, copy the flags over, set the 'executing' flag in the segment, and return the segment.
// If there is a segment ready to execute but it involves zero steps, skip and free it and start again.
MoveSegment *_ecv_null DriveMovement::NewSegment(uint32_t now) noexcept
{
	positionAtSegmentStart = currentMotorPosition;

	while (true)
	{
		MoveSegment *_ecv_null seg = segments;				// capture volatile variable
		if (seg == nullptr)
		{
			segmentFlags.Init();
			state = DMState::idle;					// if we have been round this loop already then we will have changed the state, so reset it to idle
			return nullptr;
		}

		segmentFlags = seg->GetFlags();				// assume we are going to execute this segment, or at least generate an interrupt when it is due to begin

		if ((int32_t)(seg->GetStartTime() - now) > (int32_t)MoveTiming::MaximumMoveStartAdvanceClocks)
		{
			state = DMState::starting;				// the segment is not due to start for a while. To allow it to be changed meanwhile, generate an interrupt when it is due to start.
			driversCurrentlyUsed = 0;				// don't generate a step on that interrupt
			driverEndstopsTriggeredAtStart = 0;		// reset since we will be setting this in DDA::Prepare()
			nextStepTime = seg->GetStartTime();		// this is when we want the interrupt
			return seg;
		}

		seg->SetExecuting();

		// Calculate the movement parameters
		netStepsThisSegment = (int32_t)(seg->GetLength() + distanceCarriedForwards);

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		if (IsPhaseStepEnabled())
		{
			u = seg->CalcU();
			state = DMState::phaseStepping;
			return seg;
		}
#endif

		bool newDirection;
		int32_t multiplier;
		motioncalc_t rawP;

		if (seg->NormaliseAndCheckLinear(distanceCarriedForwards, t0))
		{
			// Segment is linear
			rawP = seg->CalcLinearRecipU();
			newDirection = !std::signbit(seg->GetLength());
			multiplier = 2 * (int32_t)newDirection - 1;			// +1 or -1
			reverseStartStep = segmentStepLimit = 1 + netStepsThisSegment * multiplier;
			q = 0.0;											// to make the debug output consistent
			state = DMState::cartLinear;
		}
		else
		{
			// Segment has acceleration or deceleration
			// n = distanceCarriedForwards + u * t + 0.5 * a * t^2
			// Therefore 0.5 * t^2 + u * t/a + (distanceCarriedForwards - n)/a = 0
			// Therefore t = -u/a +/- sqrt((u/a)^2 - 2 * (distanceCarriedForwards - n)/a)
			// Calculate the t0, p and q coefficients for an accelerating or decelerating move such that t = t0 + sqrt(p*n + q) and set up the initial direction
			newDirection = !std::signbit(seg->GetA());			// assume accelerating motion
			multiplier = 2 * (int32_t)newDirection - 1;			// +1 or -1
			if (t0 <= (motioncalc_t)0.0)
			{
				// The direction reversal is in the past so the initial direction is the direction of the acceleration
				segmentStepLimit = reverseStartStep = 1 + netStepsThisSegment * multiplier;
				state = DMState::cartAccel;
			}
			else
			{
				// The initial direction is opposite to the acceleration
				newDirection = !newDirection;
				multiplier = -multiplier;
				const int32_t netStepsInInitialDirection = netStepsThisSegment * multiplier;

				if (t0 < (motioncalc_t)seg->GetDuration())
				{
					// Reversal is potentially in this segment, but it may be before the first step, or may be beyond the last step we are going to take
					// It can also happen that the target end speed is zero but due to FP rounding error, distanceToReverse was just below netStepsInInitialDirection and got rounded down
					// Note, t0 = -u/a therefore u = a*t0 therefore u*t0^2 + 0.5*a*t0^2 = -a*t0^2 + 0.5*a*t0^2 = -0.5*a*t0^2
					const motioncalc_t rawDistanceToReverse = (motioncalc_t)-0.5 * seg->GetA() * msquare(t0) + distanceCarriedForwards;
					const motioncalc_t distanceToReverse = rawDistanceToReverse * multiplier;
					const int32_t stepsBeforeReverse = (int32_t)(distanceToReverse - (motioncalc_t)0.2);			// don't step and immediately step back again
					// Note, stepsBeforeReverse may be negative at this point
					if (stepsBeforeReverse <= netStepsInInitialDirection && netStepsInInitialDirection >= 0)
					{
						segmentStepLimit = reverseStartStep = 1 + netStepsInInitialDirection;
						state = DMState::cartDecelNoReverse;
					}
					else if (stepsBeforeReverse <= 0)
					{
						// Reversal happens immediately
						newDirection = !newDirection;
						multiplier = -multiplier;
						segmentStepLimit = reverseStartStep = 1 - netStepsInInitialDirection;
						state = DMState::cartAccel;
					}
					else
					{
						reverseStartStep = stepsBeforeReverse + 1;
						segmentStepLimit = 2 * reverseStartStep - netStepsInInitialDirection - 1;
						state = DMState::cartDecelForwardsReversing;
					}
				}
				else
				{
					// Reversal doesn't occur until after the end of this segment
					segmentStepLimit = reverseStartStep = netStepsInInitialDirection + 1;
					state = DMState::cartDecelNoReverse;
				}
			}
			rawP = (motioncalc_t)2.0/seg->GetA();
			q = msquare(t0) - rawP * distanceCarriedForwards;
#if 0
			if (std::isinf(q))
			{
				debugPrintf("t0=%.1f mult=%.1f dcf=%.3e a=%.4e\n", (double)t0, (double)multiplier, (double)distanceCarriedForwards, (double)seg->GetA());
			}
#endif
		}

		p = rawP * multiplier;

		nextStep = 1;
		if (nextStep < segmentStepLimit)
		{
			if (newDirection != direction)
			{
				directionChanged = true;
				direction = newDirection;
			}

			// Unless we're possibly in the middle of a homing move, re-enable all drivers for this axis
			if (!segmentFlags.checkEndstops)
			{
				driversCurrentlyUsed = driversNormallyUsed;
			}

			if (isExtruder)
			{
				if (segmentFlags.nonPrintingMove)
				{
					extruderPrinting = false;
				}
				else if (!extruderPrinting)
				{
					extruderPrintingSince = millis();
					extruderPrinting = true;
				}
			}

#if 0	//DEBUG
			debugPrintf("New cart seg: state %u q=%.4e t0=%.4e p=%.4e ns=%" PRIi32 " ssl=%" PRIi32 "\n",
							(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit);
#endif
			return seg;
		}

#if 0
		if (netStepsThisSegment != 0)
		{
			debugPrintf("Calc error! dcf=%.2f seg: ", (double)distanceCarriedForwards);
			seg->DebugPrint();
		}
#endif

#if 0
		debugPrintf("skipping seg: state %u q=%.4e t0=%.4e p=%.4e ns=%" PRIi32 " ssl=%" PRIi32 "\n",
						(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit);
		seg->DebugPrint();
#endif
		motioncalc_t newDcf = distanceCarriedForwards + seg->GetLength();
		if (fabsm(newDcf) > (motioncalc_t)1.0)
		{
			if (reprap.Debug(Module::Move))
			{
				debugPrintf("newDcf=%.3e\n", (double)newDcf);
			}
			LogStepError(7, (float)newDcf);
			newDcf = constrain<motioncalc_t>(newDcf, -1.0, 1.0);	// to prevent the next segment erroring out
		}
		distanceCarriedForwards = newDcf;
		MoveSegment *oldSeg = seg;
		segments = seg = seg->GetNext();						// skip this segment
		MoveSegment::Release(oldSeg);
	}
}

// Version of fastSqrt that allows for slightly negative operands caused by rounding error
static inline motioncalc_t fastLimSqrtm(motioncalc_t f) noexcept
{
#if USE_DOUBLE_MOTIONCALC
	return (f >= (motioncalc_t)0.0) ? fastSqrtd(f) : (motioncalc_t)0.0;
#else
	return (f > 0.0) ? fastSqrtf(f) : 0.0;
#endif
}

// Tell the Move class that we had a step error. This always returns false so that CalcNextStepTimeFull can tail-chain to it.
bool DriveMovement::LogStepError(uint8_t type, float extra) noexcept
{
	state = DMState::stepError;
	stepErrorType = type;
	if (reprap.Debug(Module::Move))
	{
		debugPrintf("Step err %u on ", type);
		DebugPrint();
		MoveSegment::DebugPrintList(segments);
	}
	reprap.GetMove().LogStepError(type, drive, extra);
	return false;
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// We have already incremented nextStep and checked that it does not exceed totalSteps, so at least one more step is due
// Return true if all OK and there are more steps to do.
// If no more segments to execute, return false with state = DMState::idle.
// If a step error occurs, call LogStepError and return false with state set to the error state.
bool DriveMovement::CalcNextStepTimeFull(uint32_t now) noexcept
pre(stepsTillRecalc == 0; segments != nullptr)
{
	MoveSegment *currentSegment = segments;							// capture volatile variable
	uint32_t shiftFactor = 0;										// assume single stepping
	{
		int32_t stepsToLimit = segmentStepLimit - nextStep;
		// If there are no more steps left in this segment, skip to the next segment and use single stepping
		if (stepsToLimit <= 0)
		{
			distanceCarriedForwards += currentSegment->GetLength() - (motioncalc_t)netStepsThisSegment;
			if (distanceCarriedForwards > (motioncalc_t)1.0 || distanceCarriedForwards < (motioncalc_t)-1.0)
			{
				return LogStepError(5, distanceCarriedForwards);
			}
			if (currentMotorPosition - positionAtSegmentStart != netStepsThisSegment)
			{
				return LogStepError(6, 0.0);
			}
			if (isExtruder)
			{
				movementAccumulator += netStepsThisSegment;			// update the amount of extrusion
			}
			segments = currentSegment->GetNext();
			const uint32_t prevEndTime = currentSegment->GetStartTime() + currentSegment->GetDuration();
			MoveSegment::Release(currentSegment);
			currentSegment = NewSegment(now);
			if (currentSegment == nullptr)
			{
				return false;										// the call to NewSegment has already set the state to idle
			}

			if (state == DMState::starting)
			{
				return true;										// the call to NewSegment has already set up the interrupt time
			}

			if (unlikely((int32_t)(currentSegment->GetStartTime() - prevEndTime) < -10))
			{
				return LogStepError(1, (float)(int32_t)(currentSegment->GetStartTime() - prevEndTime));
			}

			// Leave shiftFactor set to 0 so that we compute a single step time, because the interval will have changed
			stepsTakenThisSegment = 1;								// this will be the first step in this segment
		}
		else if (stepsTakenThisSegment < 2)
		{
			// Reasons why we always use single stepping until we are on the third step in a segment:
			// 1. On the very first step of a move we don't know what the step interval is, so we must use single stepping for the first step.
			// 2. For extruders the step interval calculated for the very first step may be very small because of overdue extrusion,
			//    so we don't have a reliable step interval until we have calculated 2 steps.
			// 3. When starting a subsequent segment there may be a discontinuity due to rounding error,
			//    so the step interval calculated after the first step in a subsequent phase is not reliable.
			++stepsTakenThisSegment;
		}
		else
		{
			if (reverseStartStep < segmentStepLimit && nextStep <= reverseStartStep)
			{
				stepsToLimit = reverseStartStep - nextStep;
			}

			if (stepsToLimit > 1 && stepInterval < MoveTiming::MinCalcInterval)
			{
				if (stepInterval < MoveTiming::MinCalcInterval/4 && stepsToLimit > 8)
				{
					shiftFactor = 3;							// octal stepping
				}
				else if (stepInterval < MoveTiming::MinCalcInterval/2 && stepsToLimit > 4)
				{
					shiftFactor = 2;							// quad stepping
				}
				else if (stepsToLimit > 2)
				{
					shiftFactor = 1;							// double stepping
				}
			}
		}
	}

	stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate

	motioncalc_t nextCalcStepTime;

	// Work out the time of the step
	switch (state)
	{
	case DMState::cartLinear:									// linear steady speed
		nextCalcStepTime = (motioncalc_t)(nextStep + (int32_t)stepsTillRecalc) * p;
		break;

	case DMState::cartAccel:									// Cartesian accelerating
		nextCalcStepTime = fastLimSqrtm(q + p * (motioncalc_t)(nextStep + (int32_t)stepsTillRecalc));
		break;

	case DMState::cartDecelForwardsReversing:
		if (nextStep + (int32_t)stepsTillRecalc < reverseStartStep)
		{
			nextCalcStepTime = -fastLimSqrtm(q + p * (motioncalc_t)(nextStep + (int32_t)stepsTillRecalc));
			break;
		}

		direction = !direction;
		directionChanged = true;
		state = DMState::cartDecelReverse;
		// no break
	case DMState::cartDecelReverse:								// Cartesian decelerating, reverse motion. Convert the steps to int32_t because the net steps may be negative.
		{
			const int32_t netSteps = 2 * reverseStartStep - nextStep - 1;
			nextCalcStepTime = fastLimSqrtm(q + p * (motioncalc_t)(netSteps - (int32_t)stepsTillRecalc));
		}
		break;

	case DMState::cartDecelNoReverse:							// Cartesian decelerating with no reversal
		nextCalcStepTime = -fastLimSqrtm(q + p * (motioncalc_t)(nextStep + (int32_t)stepsTillRecalc));
		break;

	default:
#if SEGMENT_DEBUG
		debugPrintf("DMstate %u, quitting\n", (unsigned int)state);
#endif
		return LogStepError(4, 0.0);
	}

	nextCalcStepTime += t0;

	if (unlikely(std::isnan(nextCalcStepTime) || nextCalcStepTime < (motioncalc_t)0.0))
	{
		if (reprap.Debug(Module::Move))
		{
			debugPrintf("nextCalcStepTime=%.3e\n", (double)nextCalcStepTime);
		}
		return LogStepError(2, (float)nextCalcStepTime);
	}

	uint32_t iNextCalcStepTime = (uint32_t)nextCalcStepTime;

	if (iNextCalcStepTime > currentSegment->GetDuration())
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// 2023-12-06: we now allow any step to be late but we record the maximum number.
		// 2024-040-5: we now allow steps to be late on any segment, not just the last one, because a segment may be 0 or 1 step long and on deltas the last 2 steps may be calculated late.
		iNextCalcStepTime = currentSegment->GetDuration();
		const int32_t nextCalcStep = nextStep + (int32_t)stepsTillRecalc;
		const int32_t stepsLate = segmentStepLimit - nextCalcStep;
		if (stepsLate > maxStepsLate) { maxStepsLate = stepsLate; }
	}

	iNextCalcStepTime += currentSegment->GetStartTime();
	if (nextStep == 1)
	{
		nextStepTime = iNextCalcStepTime;
	}
	else
	{
		// When crossing between movement phases with high microstepping, due to rounding errors the next step may appear to be due before the last one
		const int32_t interval = (int32_t)(iNextCalcStepTime - nextStepTime);
		if (interval > 0)
		{
			stepInterval = (uint32_t)interval >> shiftFactor;		// calculate the time per step, ready for next time
		}
		else
		{
			if (interval < minStepInterval) { minStepInterval = interval; }
			stepInterval = 0;
		}

#if 0	//DEBUG
		if (isExtruder && stepInterval < 20 /*&& nextStep + stepsTillRecalc + 1 < totalSteps*/)
		{
			state = DMState::stepError1;
			return LogStepError();
		}
#endif

		nextStepTime = iNextCalcStepTime - (stepsTillRecalc * stepInterval);
	}

	return true;
}

#if SUPPORT_CAN_EXPANSION

// This is called when the 'drive' (i.e. axis or extruder) concerned has no local drivers and we are not checking endstops or Z probe.
// Instead of generating an interrupt for each step of the remote drive, generate interrupts only occasionally and at the end of each segment, to keep the axis position fairly up to date.
// We must not call NewSegment significantly in advance of when the segment is due to start, to allow for segments being modified as new ones are added.
// To make sure this is the case we schedule an interrupt at the end of each segment, so that a segment cannot be started before the previous one has completed.
void DriveMovement::TakeStepsAndCalcStepTimeRarely(uint32_t clocksNow) noexcept
{
	MoveSegment *currentSegment = segments;				// capture volatile variable
	if (state == DMState::ending)
	{
		currentMotorPosition = positionAtSegmentStart + netStepsThisSegment;
		distanceCarriedForwards += currentSegment->GetLength() - (motioncalc_t)netStepsThisSegment;
		segments = currentSegment->GetNext();
		MoveSegment::Release(currentSegment);
		currentSegment = NewSegment(clocksNow);
		if (currentSegment == nullptr || state == DMState::starting)
		{
			return;
		}
	}

	// We may be invoked slightly before the move started, so allow for that
	const uint32_t timeFromStart = (uint32_t)max<int32_t>((int32_t)(clocksNow - currentSegment->GetStartTime()), 0);
	currentMotorPosition = positionAtSegmentStart + lrintf((currentSegment->CalcU() + ((motioncalc_t)0.5 * currentSegment->GetA() * (motioncalc_t)timeFromStart)) * (motioncalc_t)timeFromStart + distanceCarriedForwards);
	uint32_t targetTime;
	if (currentSegment->GetDuration() <= timeFromStart + MoveTiming::MaxRemoteDriverPositionUpdateInterval)
	{
		// Generate an interrupt at the end of this segment
		state = DMState::ending;								// this is just a flag to say we need a new segment next time
		targetTime = currentSegment->GetDuration();
	}
	else
	{
		targetTime = timeFromStart + MoveTiming::NominalRemoteDriverPositionUpdateInterval;
	}
	nextStepTime = targetTime + currentSegment->GetStartTime();
}

#endif

// If the logical drive is moving, stop it and update the position.
// Return true if the drive was moving.
bool DriveMovement::StopLogicalDrive(int32_t& netStepsTaken) noexcept
{
	AtomicCriticalSectionLocker lock;

	if (state != DMState::idle)
	{
		state = DMState::idle;
		reprap.GetMove().DeactivateDM(this);
		netStepsTaken = GetNetStepsTakenThisMove();
		MoveSegment *seg = nullptr;
		std::swap(seg, const_cast<MoveSegment*&>(segments));
		MoveSegment::ReleaseAll(seg);
		//TODO do we need to update the endpoint anywhere? Or will the caller do this?
#if STEPS_DEBUG
		positionRequested = currentMotorPosition;
#endif
		return true;
	}

	netStepsTaken = 0;
	return false;
}

#if CHECK_SEGMENTS

void DriveMovement::CheckSegment(unsigned int line, MoveSegment *seg) noexcept
{
	if (seg != nullptr)
	{
		if (   (int32_t)seg->GetDuration() <= 0
			|| (seg->GetNext() != nullptr && (int32_t)(seg->GetNext()->GetStartTime() - (seg->GetStartTime() + seg->GetDuration())) < 0)
		   )
		{
			debugPrintf("bad seg at %u: ", line);
			MoveSegment::DebugPrintList(seg);
		}
	}
}

#endif

#if SUPPORT_REMOTE_COMMANDS

void DriveMovement::StopDriverFromRemote() noexcept
{
	int32_t dummy;
	(void)StopLogicalDrive(dummy);
}

#endif

#if SUPPORT_PHASE_STEPPING

bool DriveMovement::SetStepMode(StepMode mode) noexcept
{
	switch (mode)
	{
	case StepMode::stepDir:
		phaseStepControl.SetEnabled(false);
		break;
	case StepMode::phase:
		phaseStepControl.SetEnabled(true);
		break;
	default:
		return false;
	}
	stepMode = mode;
	return true;
}

motioncalc_t DriveMovement::GetPhaseStepsTakenThisSegment() const noexcept
{
	const MoveSegment *const seg = segments;
	if (seg == nullptr)
	{
		return 0;
	}
	int32_t timeSinceStart = (int32_t)(StepTimer::GetMovementTimerTicks() - seg->GetStartTime());
	if (timeSinceStart < 0)
	{
		return 0;
	}

	if ((uint32_t)timeSinceStart >= seg->GetDuration())
	{
		timeSinceStart = seg->GetDuration();
	}

	return (u + seg->GetA() * timeSinceStart * 0.5) * timeSinceStart;
}

#endif

// End
