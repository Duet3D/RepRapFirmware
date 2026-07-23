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

constexpr uint32_t MinimumExecutingSegmentDuration = 20;

// Static members

int32_t DriveMovement::maxStepsLate = 0;
int32_t DriveMovement::minStepInterval = 0;

// Non static members

void DriveMovement::Init(size_t drv) noexcept
{
	drive = (uint8_t)drv;
	state = DMState::idle;
	distanceCarriedForwards = 0.0;
	currentMotorPosition = positionAtSegmentStart = 0;
	movementAccumulator = 0;
	extruderPrinting = isExtruder = false;
#if STEPS_DEBUG
	positionRequested = 0;
#endif
	driversNormallyUsed = driversCurrentlyUsed = driverEndstopsTriggeredAtStart = 0;
	nextDM = nullptr;
	segments = nullptr;
	segmentFlags.Init();

#if SUPPORT_PHASE_STEPPING
	stepMode = StepMode::stepDir;
#endif
	u = (motioncalc_t)0.0;
#if SUPPORT_S_CURVE
	peakDeltaV = peakDeltaA = (motioncalc_t)0.0;
	finalSpeed = finalAcc = (motioncalc_t)0.0;
#endif
}

void DriveMovement::DebugPrint() const noexcept
{
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
		debugPrintf("DM%c state=%u dir=%c next=%" PRIi32 " rev=%" PRIi32 " ssl=%" PRIi32 " sns=%" PRIi32 " interval=%" PRIu32 " q=%.4g t0=%.4g p=%.4g dcf=%.2f\n",
						c, (unsigned int)state, (direction) ? 'F' : 'B',
							nextStep, reverseStartStep, segmentStepLimit, netStepsThisSegment, stepInterval,
								(double)q, (double)t0, (double)p, (double)distanceCarriedForwards);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

void DriveMovement::DiagnosticHeader(const StringRef& reply) noexcept
{
	reply.lcat(
#if 0
				"Drive req/act/dcf/state/segs?"
#else
				"Drive req/act/dcf"
#endif
# if SUPPORT_S_CURVE
				" deltaV/deltaA"
# endif
				":");
}

// Report diagnostic information for this logical drive
void DriveMovement::Diagnostics(const StringRef& reply) noexcept
{
# if 0	// DEBUG
	reply.lcatf("%2u: %.2f/%" PRIi32 "/%.2f/%u/%u", drive, (double)positionRequested, currentMotorPosition, (double)distanceCarriedForwards, (unsigned int)state, segments != nullptr);
# else
	reply.lcatf("%2u: %.2f/%" PRIi32 "/%.2f", drive, (double)positionRequested, currentMotorPosition, (double)distanceCarriedForwards);
# endif
#if SUPPORT_S_CURVE
	reply.catf(" %.2f/%.2f",
				(double)InverseConvertSpeedToMmPerSec((float)peakDeltaV/reprap.GetMove().DriveStepsPerMm(drive)),
				(double)InverseConvertAcceleration((float)peakDeltaA/reprap.GetMove().DriveStepsPerMm(drive))
			  );
	peakDeltaV = peakDeltaA = 0.0;
#endif
}

// Retire the current segment but keep it available temporarily for debugging
void DriveMovement::RetireSegment(MoveSegment *oldSegment) noexcept
{
	if (retiredSegment != nullptr)
	{
		MoveSegment::Release(retiredSegment);
	}
	retiredSegment = oldSegment;
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

#if SUPPORT_S_CURVE

# define DEBUG_DISCONTINUITIES	(1)			// set nonzero to generate debug messages if extruder 0 has discontinuous speed or acceleration

# if DEBUG_DISCONTINUITIES
constexpr motioncalc_t MaxSpeedChange = ConvertSpeedFromMmPerSec(0.1 * 420);		// 420 is extruder steps/mm in test config
constexpr motioncalc_t MaxAccChange = ConvertAcceleration(5.0 * 420);				// 420 is extruder steps/mm in test config
# endif

void DriveMovement::PrintRetiredSegment() const noexcept
{
	if (retiredSegment != nullptr)
	{
		retiredSegment->DebugPrint();
	}
	else
	{
		debugPrintf("null\n");
	}
}

// Update the stored speed, final acceleration, speed change and acceleration change values
inline void DriveMovement::UpdateSpeedAndAccelerationChange(motioncalc_t newSpeed, motioncalc_t speedChange, motioncalc_t newAcc, motioncalc_t accChange) noexcept
{
	u = newSpeed;
	peakDeltaV = max<motioncalc_t>(peakDeltaV, std::fabs(newSpeed - finalSpeed));
# if DEBUG_DISCONTINUITIES
	if (isExtruder && std::fabs(newSpeed - finalSpeed) > MaxSpeedChange && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
	{
		debugPrintf("Extruder speed change %.1f to %.1f, retired: ", (double)InverseConvertSpeedToMmPerSec((float)finalSpeed), (double)InverseConvertSpeedToMmPerSec((float)newSpeed));
		PrintRetiredSegment();
		MoveSegment::DebugPrintList(segments);
	}
# endif
	finalSpeed = newSpeed + speedChange;

	peakDeltaA = max<motioncalc_t>(peakDeltaA, std::fabs(newAcc - finalAcc));
# if DEBUG_DISCONTINUITIES
	if (isExtruder && std::fabs(newAcc - finalAcc) > MaxAccChange && !extruderShaper.IsActive() && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
	{
		debugPrintf("Extruder acc change %.1f to %.1f, retired: ", (double)InverseConvertAcceleration((float)finalAcc), (double)InverseConvertAcceleration((float)newAcc));
		PrintRetiredSegment();
		MoveSegment::DebugPrintList(segments);
	}
# endif
	finalAcc = newAcc + accChange;
}

// Update the stored speed, final acceleration, speed change and acceleration change values when movement has stopped
inline void DriveMovement::MovementStopped() noexcept
{
	u = (motioncalc_t)0.0;
	peakDeltaV = max<motioncalc_t>(peakDeltaV, std::fabs(finalSpeed));
# if DEBUG_DISCONTINUITIES
	if (isExtruder && std::fabs(finalSpeed) > MaxSpeedChange && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
	{
		debugPrintf("Extruder speed change %.1f to standstill, retired: ", (double)InverseConvertSpeedToMmPerSec((float)finalSpeed));
		PrintRetiredSegment();
	}
# endif
	finalSpeed = (motioncalc_t)0.0;

	peakDeltaA = max<motioncalc_t>(peakDeltaA, std::fabs(finalAcc));
# if DEBUG_DISCONTINUITIES
	if (isExtruder && std::fabs(finalAcc) > MaxAccChange && !extruderShaper.IsActive() && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintBadMoves))
	{
		debugPrintf("Extruder acc change %.1f to standstill, retired: ", (double)InverseConvertAcceleration((float)finalAcc));
		PrintRetiredSegment();
	}
# endif
	finalAcc = (motioncalc_t)0.0;
}

#endif

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
		MoveSegment *_ecv_null seg = segments;					// capture volatile variable
		if (seg == nullptr)
		{
			segmentFlags.Init();
			state = DMState::idle;								// if we have been round this loop already then we will have changed the state, so reset it to idle
#if SUPPORT_S_CURVE
			MovementStopped();
#endif
			return nullptr;
		}

		segmentFlags = seg->GetFlags();							// assume we are going to execute this segment, or at least generate an interrupt when it is due to begin

		if ((int32_t)(seg->GetStartTime() - now) > (int32_t)MoveTiming::MaximumMoveStartAdvanceClocks)
		{
			state = DMState::starting;							// the segment is not due to start for a while. To allow it to be changed meanwhile, generate an interrupt when it is due to start.
			driversCurrentlyUsed = 0;							// don't generate a step on that interrupt
			driverEndstopsTriggeredAtStart = 0;					// reset since we will be setting this in DDA::Prepare()
			nextStepTime = seg->GetStartTime();					// this is when we want the interrupt
#if SUPPORT_S_CURVE
			MovementStopped();									// say we have stopped. NewSegment will be called again for this segment when the state changes from 'starting' to something else.
#endif
			return seg;
		}

		// If this segment is very short, merge it into the next one. This improves efficiency and avoids reporting speed/acceleration discontinuities caused by rounding error.
		// Typically we merge a very short segment into a much longer segment, which works well.
		MoveSegment *_ecv_null nextSeg;
		while (   seg->GetDuration() < MinimumExecutingSegmentDuration
			   && (nextSeg = seg->GetNext()) != nullptr
			   && nextSeg->GetFlags().SameStaticFlags(segmentFlags)
			   && nextSeg->GetStartTime() == seg->GetStartTime() + seg->GetDuration()
			  )
		{
			// We can and should merge this segment into the next one. When the segment is executed, the initial speed will be adjusted to match them.
			nextSeg->CombinePrevious(seg);
			segments = nextSeg;
			MoveSegment::Release(seg);							// release the segment, don't retire it
			seg = nextSeg;
		}

		seg->SetExecuting();
#if SUPPORT_S_CURVE
		UpdateSpeedAndAccelerationChange(seg->CalcU(), seg->GetSpeedChange(), seg->GetA(), seg->GetAccChange());
#else
		u = seg->CalcU(); // used for GetCurrentPosition()
#endif

		// Calculate the movement parameters
		netStepsThisSegment = (int32_t)(seg->GetLength() + distanceCarriedForwards);

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		if (IsPhaseStepEnabled())
		{
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

			if (segmentFlags.isExtruder)
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
		// nextStep is not less than segmentStepLimit, so we are not going to execute this segment
		motioncalc_t newDcf = distanceCarriedForwards + seg->GetLength();
		if (std::fabs(newDcf) > (motioncalc_t)1.0)
		{
			(void)LogStepError(7, (float)newDcf, seg);
			newDcf = constrain<motioncalc_t>(newDcf, -1.0, 1.0);	// to prevent the next segment erroring out
		}
		distanceCarriedForwards = newDcf;
		MoveSegment *oldSeg = seg;
		segments = seg = seg->GetNext();							// skip this segment
		RetireSegment(oldSeg);
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

// Notify a step error. This always returns false so that CalcNextStepTimeFull can tail-chain to it.
bool DriveMovement::LogStepError(uint8_t type, float info, const MoveSegment *seg) noexcept
{
	const StringRef& dbgRef = Platform::genericDebugBuffer.GetRef();
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive]
															   : (char)('0' + LogicalDriveToExtruder(drive));
	dbgRef.printf("Code %u move error: dm=%u %c, info=%.3g, seg: ", type, drive, c, (double)info);
	if (seg != nullptr)
	{
		seg->AppendDetails(dbgRef);
	}
	dbgRef.cat('\n');
	Platform::shouldTurnOffHeaters = true;
	Platform::hasGenericDebug = true;
	reprap.GetMove().StepErrorHalt();
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
		if (stepsToLimit == 1 && currentSegment->GetNext() == nullptr && !currentSegment->GetFlags().isExtruder && reverseStartStep != nextStep)
		{
			// It's an axis and we are soon to stop movement, so we should end on an exact microstep.
			// Check whether taking the last step would end up going a little too far or not quite far enough
			const motioncalc_t provisionalDistanceCarriedForwards = distanceCarriedForwards + currentSegment->GetLength() - (motioncalc_t)netStepsThisSegment;
			if (std::fabs(provisionalDistanceCarriedForwards) < (motioncalc_t)0.05)
			{
				currentSegment->AdjustLength(-provisionalDistanceCarriedForwards);				// just correct the segment length
			}
			else if (provisionalDistanceCarriedForwards > (motioncalc_t)0.95)
			{
				currentSegment->AdjustLength((motioncalc_t)1.0 - provisionalDistanceCarriedForwards);	// adjust the segment length slightly
				if (direction)
				{
					// Take 1 more step
					++netStepsThisSegment;														// add 1 step to it
					const int32_t oldSsl = segmentStepLimit;
					segmentStepLimit = oldSsl + 1;												// increase the number of steps due
					if (reverseStartStep == oldSsl) { reverseStartStep = oldSsl + 1; }			// if we didn't reverse already, make sure we don't reverse when we take the extra step
					++stepsToLimit;																// we can take 1 more step
				}
				else
				{
					// Take 1 less step
					--segmentStepLimit;
					--netStepsThisSegment;
					stepsToLimit = 0;
				}
			}
			else if (provisionalDistanceCarriedForwards < -(motioncalc_t)0.95)
			{
				currentSegment->AdjustLength(-(motioncalc_t)1.0 - provisionalDistanceCarriedForwards);	// adjust the segment length slightly
				if (direction)
				{
					// Take 1 less step
					--segmentStepLimit;
					--netStepsThisSegment;
					stepsToLimit = 0;
				}
				else
				{
					--netStepsThisSegment;														// add 1 step to it in the backwards direction
					const int32_t oldSsl = segmentStepLimit;
					segmentStepLimit = oldSsl + 1;												// increase the number of steps due
					if (reverseStartStep == oldSsl) { reverseStartStep = oldSsl + 1; }			// if we didn't reverse already, make sure we don't reverse now
					++stepsToLimit;
				}
			}
		}

		// If there are no more steps left in this segment, skip to the next segment and use single stepping
		if (stepsToLimit <= 0)
		{
			distanceCarriedForwards += currentSegment->GetLength() - (motioncalc_t)netStepsThisSegment;
			if (std::fabs(distanceCarriedForwards) > (motioncalc_t)1.0)
			{
				return LogStepError(5, (float)distanceCarriedForwards, currentSegment);
			}
			if (currentMotorPosition - positionAtSegmentStart != netStepsThisSegment)
			{
				return LogStepError(6, (float)(currentMotorPosition - positionAtSegmentStart - netStepsThisSegment), currentSegment);
			}

			movementAccumulator += netStepsThisSegment;				// update the amount of extrusion
			segments = currentSegment->GetNext();
			const uint32_t prevEndTime = currentSegment->GetStartTime() + currentSegment->GetDuration();
			RetireSegment(currentSegment);
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
				return LogStepError(1, (float)(int32_t)(currentSegment->GetStartTime() - prevEndTime), currentSegment);
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
		[[fallthrough]];
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
		return LogStepError(4, (float)state, currentSegment);
	}

	nextCalcStepTime += t0;
	uint32_t iNextCalcStepTime;
	// Check that the next step time is reasonable
	if (unlikely(std::isnan(nextCalcStepTime)))
	{
		return LogStepError(2, (float)nextCalcStepTime, currentSegment);
	}

	if (unlikely(nextCalcStepTime < (motioncalc_t)0.0))
	{
		// If we are carrying almost a whole step forward to this segment so that the first step is due almost immediately,
		// then due to floating point rounding error we can get a slightly negative value here for nextCalcStepTime.
		// The only value we have had reported so far is -0.00195 but we now allow up to two step clocks of error.
		if (nextCalcStepTime < -(motioncalc_t)2.0)
		{
			return LogStepError(2, (float)nextCalcStepTime, currentSegment);
		}
		iNextCalcStepTime = 0;
	}
	else
	{
		iNextCalcStepTime = (uint32_t)nextCalcStepTime;
	}

	if (iNextCalcStepTime > currentSegment->GetDuration())
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// 2023-12-06: we now allow any step to be late but we record the maximum number.
		// 2024-04-05: we now allow steps to be late on any segment, not just the last one, because a segment may be 0 or 1 step long and on deltas the last 2 steps may be calculated late.
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
	currentMotorPosition = positionAtSegmentStart + std::lrint((currentSegment->CalcU() + ((motioncalc_t)0.5 * currentSegment->GetA() * (motioncalc_t)timeFromStart)) * (motioncalc_t)timeFromStart + distanceCarriedForwards);
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
bool DriveMovement::StopLogicalDrive(int32_t& netStepsTaken, uint32_t when) noexcept
{
	AtomicCriticalSectionLocker lock;

	if (state != DMState::idle)
	{
		state = DMState::idle;
		reprap.GetMove().DeactivateDM(this);
		netStepsTaken = GetNetStepsTakenThisMove(when);
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
	(void)StopLogicalDrive(dummy, StepTimer::GetMovementTimerTicks());
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

// Return the number of steps taken since the start of this segment. This works when doing normal stepping or phase stepping.
// It works for CAN-connected drives only if we are generating and retiring segments for them, which we do for moves that have endstops of probes enabled.
motioncalc_t DriveMovement::GetStepsTakenThisSegment(uint32_t when) const noexcept
{
	const MoveSegment *const seg = segments;
	if (seg == nullptr)
	{
		return 0;
	}
	int32_t timeSinceStart = (int32_t)(when - seg->GetStartTime());
	if (timeSinceStart < 0)
	{
		return 0;
	}

	if ((uint32_t)timeSinceStart >= seg->GetDuration())
	{
		timeSinceStart = seg->GetDuration();
	}

	return (u + seg->GetA() * timeSinceStart * (motioncalc_t)0.5) * timeSinceStart;
}

#endif

// End
