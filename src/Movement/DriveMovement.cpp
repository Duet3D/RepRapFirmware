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
#if STEPS_DEBUG
	positionRequested = 0;
#endif
	driversNormallyUsed = driversCurrentlyUsed = 0;
	nextDM = nullptr;
	segments = nullptr;
	homingDda = nullptr;
	isExtruder = false;
	segmentFlags.Init();
}

void DriveMovement::DebugPrint() const noexcept
{
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
		debugPrintf("DM%c state=%u err=%u dir=%c next=%" PRIi32 " rev=%" PRIi32 " ssl=%" PRIi32 " sns=%" PRIi32 " interval=%" PRIu32 " q=%.4e t0=%.4e p=%.4e dcf=%.2f\n",
						c, (unsigned int)state, (unsigned int)stepErrorType, (direction) ? 'F' : 'B',
							nextStep, reverseStartStep, segmentStepLimit, netStepsThisSegment, stepInterval,
								(double)q, (double)t0, (double)p, (double)distanceCarriedForwards);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

void DriveMovement::SetMotorPosition(int32_t pos) noexcept
{
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintTransforms))
	{
		debugPrintf("Changing drive %u pos from %" PRIi32 " to %" PRIi32 "\n", drive, currentMotorPosition, pos);
	}
	currentMotorPosition = pos;
#if STEPS_DEBUG
	positionRequested = (float)pos;
#endif
	ClearMovementPending();
}

void DriveMovement::AdjustMotorPosition(int32_t adjustment) noexcept
{
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintTransforms))
	{
		debugPrintf("Adjusting drive %u pos from %" PRIi32 " to %" PRIi32 "\n", drive, currentMotorPosition, currentMotorPosition + adjustment);
	}
	currentMotorPosition += adjustment;
#if STEPS_DEBUG
	positionRequested = (float)currentMotorPosition;
#endif
}

// Add a segment into the list. If the list is not empty then the new segment may overlap segments already in the list but will never start earlier than the first existing one.
// The units of the input parameters are steps for distance and step clocks for time.
void DriveMovement::AddSegment(uint32_t startTime, uint32_t duration, motioncalc_t distance, motioncalc_t u, motioncalc_t a, MovementFlags moveFlags) noexcept
{
	// Adjust the initial speed and distance to account for pressure advance
	if (isExtruder && !moveFlags.nonPrintingMove)
	{
		const motioncalc_t extraSpeed = a * (motioncalc_t)extruderShaper.GetKclocks();
		u += extraSpeed;
		distance += extraSpeed * (motioncalc_t)duration;
	}

#if !SEGMENT_DEBUG
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Segments))
#endif
	{
		debugPrintf("Add seg: dr=%u st=%" PRIu32 " t=%" PRIu32 " dist=%.2f u=%.3e a=%.3e f=%02" PRIx32 "\n", drive, startTime, duration, (double)distance, (double)u, (double)a, moveFlags.all);
	}

	MoveSegment *prev = nullptr;

	// Shut out the step interrupt and task switching while we mess with the segments
	// TODO probably only need to shut out task switching here, or shut out the step interrupt but leave the UART interrupt enabled
	const uint32_t oldPrio = ChangeBasePriority(NvicPriorityStep);		// shut out the step interrupt

	MoveSegment *seg = segments;

	if (seg != nullptr)
	{
		// Check that the segment we are adding does not start before the currently-executing segment ends, because we can't modify an executing segment
		int32_t offset = (int32_t)(startTime - seg->GetStartTime());
		if (seg->GetFlags().executing)
		{
			 const int32_t timeInHand = offset - (int32_t)seg->GetDuration();
			 if (timeInHand < 0)
			 {
				 const uint32_t now = StepTimer::GetTimerTicks();
				 LogStepError(3);
				 RestoreBasePriority(oldPrio);
				 if (reprap.Debug(Module::Move))
				 {
					 debugPrintf("was executing, overlap %" PRIi32 " while trying to add s=%" PRIu32 " t=%" PRIu32 " d=%.2f u=%.4e a=%.4e f=%02" PRIx32 " at time %" PRIu32 "\n",
						 	 	 	 -timeInHand, startTime, duration, (double)distance, (double)u, (double)a, moveFlags.all, now);
				 }
				 return;
			 }
		}

		// Find the earliest existing segment that the new one will come before (i.e. new one starts before existing one) or will overlap (i.e. the new one starts before the existing segment ends)
		while (true)
		{
			if (offset <= 0)														// if the new segment starts before the existing one starts, or at the same time
			{
				if (offset > -MoveSegment::MinDuration && duration >= 10 * MoveSegment::MinDuration)	// if it starts only slightly earlier and we can reasonably shorten it
				{
					startTime = seg->GetStartTime();								// then just delay and shorten the new segment slightly, to avoid creating a tiny segment
					const motioncalc_t durationIncrease = (motioncalc_t)offset;					// get the (negative) increase in segment duration
					const motioncalc_t oldDuration = (motioncalc_t)duration;
#if SEGMENT_DEBUG
					debugPrintf("Adjusting(1) t=%" PRIu32 " u=%.4e a=%.4e", duration, (double)u, (double)a);
#endif
					duration += offset;
					u = (u * oldDuration - a * durationIncrease * (oldDuration + (motioncalc_t)0.5 * durationIncrease))/(motioncalc_t)duration;
#if SEGMENT_DEBUG
					debugPrintf(" to t=%" PRIu32 " u=%.4e a=%.4e\n", duration, (double)u, (double)a);
#endif
					offset = 0;
				}
				else if (offset + (int32_t)duration <= MoveSegment::MinDuration)	// if the new segment starts earlier than the existing one and ends before or only slight later than the existing one starts
				{
					if (offset + (int32_t)duration > 0 && duration >= 10 * MoveSegment::MinDuration)	// if the new segment does overlap the existing one a little and we can reasonably shorten it
					{
						// Shorten the new segment slightly so than it fits before the existing segment
						const int32_t durationIncrease = -(offset + (int32_t)duration);	// get the (negative) increase in segment duration
						const motioncalc_t oldDuration = (motioncalc_t)duration;
#if SEGMENT_DEBUG
						debugPrintf("Adjusting(2) t=%" PRIu32 " u=%.4e a=%.4e", duration, (double)u, (double)a);
#endif
						duration = -offset;
						u = (u * oldDuration - a * durationIncrease * (oldDuration + (motioncalc_t)0.5 * durationIncrease))/(motioncalc_t)duration;
#if SEGMENT_DEBUG
						debugPrintf(" to t=%" PRIu32 " u=%.4e a=%.4e\n", duration, (double)u, (double)a);
#endif
					}
					break;															// now insert the new segment before the existing one
				}
				else																// new segment overlaps the existing one significantly
				{
					// Insert part of the new segment before the existing one, then merge the rest
					const uint32_t firstDuration = -offset;
					const motioncalc_t firstDistance = (u + (motioncalc_t)0.5 * a * (motioncalc_t)firstDuration) * (motioncalc_t)firstDuration;
					seg = MoveSegment::Allocate(seg);
					seg->SetParameters(startTime, firstDuration, firstDistance, u, a, moveFlags);
					if (prev == nullptr)
					{
						segments = seg;
					}
					else
					{
						prev->SetNext(seg);
					}
					duration -= firstDuration;
					startTime += firstDuration;
					distance -= firstDistance;
					u += a * (motioncalc_t)firstDuration;
					prev = seg;
					seg = seg->GetNext();
					offset = 0;
				}
			}

			// If we get here then the new segment starts later or at the same time as the existing one
			if (offset + MoveSegment::MinDuration < (int32_t)seg->GetDuration())	// if the segment we are adding starts significantly before the existing one ends
			{
				if (offset < MoveSegment::MinDuration && duration >= 10 * MoveSegment::MinDuration)	// if it starts only slightly before the existing segment ends and we can reasonably shorten it
				{
					startTime = seg->GetStartTime();								// postpone and shorten it a little
					const int32_t durationIncrease = -offset;						// get the (negative) increase in segment duration
					const motioncalc_t oldDuration = (motioncalc_t)duration;
#if SEGMENT_DEBUG
					debugPrintf("Adjusting(3) t=%" PRIu32 " u=%.4e a=%.4e", duration, (double)u, (double)a);
#endif
					duration -= offset;
					u = (u * oldDuration - a * durationIncrease * (oldDuration + (motioncalc_t)0.5 * durationIncrease))/(motioncalc_t)duration;
#if SEGMENT_DEBUG
					debugPrintf(" to t=%" PRIu32 " u=%.4e a=%.4e\n", duration, (double)u, (double)a);
#endif
				}
				else																// else split the existing segment
				{
					seg = seg->Split((uint32_t)offset);
					// 'prev' is now wrong but we're not about to insert anything before 'seg'
				}

				// The segment we wish to add now starts at the same time as 'seg' but it may end earlier or later than the one at 'seg' does.
				const int32_t timeDifference = (int32_t)(duration - seg->GetDuration());
				if (timeDifference > MoveSegment::MinDuration)
				{
					// The existing segment is shorter in time than the new one, so add the new segment in two or more parts
					const motioncalc_t firstDistance = (u + (motioncalc_t)0.5 * a * (motioncalc_t)seg->GetDuration()) * (motioncalc_t)seg->GetDuration();	// distance moved by the first part of the new segment
#if SEGMENT_DEBUG
					debugPrintf("merge1: ");
#endif
					seg->Merge(firstDistance, u, a, moveFlags);
					distance -= firstDistance;
					startTime += seg->GetDuration();
					u += a * (motioncalc_t)seg->GetDuration();
					duration = (uint32_t)timeDifference;
				}
				else
				{
					// New segment ends earlier or at the same time as the old one
					if (timeDifference > -MoveSegment::MinDuration && duration >= 10 * MoveSegment::MinDuration)
					{
						// Make the new segment duration fit the existing one by adjusting the initial speed slightly
#if SEGMENT_DEBUG
						debugPrintf("Adjusting(4) t=%" PRIu32 " u=%.4e a=%.4e", duration, (double)u, (double)a);
#endif
						u = ((u * (motioncalc_t)duration) - (a * (motioncalc_t)timeDifference * ((motioncalc_t)duration + (motioncalc_t)0.5 * (motioncalc_t)timeDifference)))/(motioncalc_t)seg->GetDuration();
						duration = seg->GetDuration();
#if SEGMENT_DEBUG
						debugPrintf(" to t=%" PRIu32 " u=%.4e a=%.4e\n", duration, (double)u, (double)a);
#endif
					}
					else
					{
						// Split the existing segment in two
						seg->Split(duration);
					}

					// The new segment and the existing one now have the same start time and duration, so merge them
#if SEGMENT_DEBUG
					debugPrintf("merge2: ");
#endif
					seg->Merge(distance, u, a, moveFlags);
#if SEGMENT_DEBUG
					MoveSegment::DebugPrintList(segments);
#endif
					RestoreBasePriority(oldPrio);
					return;
				}
			}

			prev = seg;
			seg = seg->GetNext();
			if (seg == nullptr) break;
			offset = (int32_t)(startTime - seg->GetStartTime());
		}
	}

	// The new segment (or what's left of it) needs to be added before 'seg' which may be null
	seg = MoveSegment::Allocate(seg);
	seg->SetParameters(startTime, duration, distance, u, a, moveFlags);
	if (prev == nullptr)
	{
		segments = seg;
	}
	else
	{
		prev->SetNext(seg);
	}
#if SEGMENT_DEBUG
	MoveSegment::DebugPrintList(segments);
#endif
	RestoreBasePriority(oldPrio);
}

// Set up to schedule the first segment, returning true if an interrupt for tihs DM is needed
bool DriveMovement::ScheduleFirstSegment() noexcept
{
	const uint32_t now = StepTimer::GetTimerTicks();
	if (NewSegment(now) != nullptr)
	{
		if (state == DMState::starting)
		{
			return true;
		}
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
MoveSegment *DriveMovement::NewSegment(uint32_t now) noexcept
{
	positionAtSegmentStart = currentMotorPosition;

	while (true)
	{
		MoveSegment *seg = segments;				// capture volatile variable
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
			nextStepTime = seg->GetStartTime();		// this is when we want the interrupt
			return seg;
		}

		seg->SetExecuting();

		// Calculate the movement parameters
		bool newDirection;
		netStepsThisSegment = (int32_t)(seg->GetLength() + distanceCarriedForwards);
		if (seg->NormaliseAndCheckLinear(distanceCarriedForwards, t0))
		{
			if (seg->GetU() < (motioncalc_t)0.0)
			{
				newDirection = false;
				p = (motioncalc_t)-1.0/seg->GetU();
				segmentStepLimit = 1 - netStepsThisSegment;
			}
			else
			{
				newDirection = true;
				p = (motioncalc_t)1.0/seg->GetU();
				segmentStepLimit = netStepsThisSegment + 1;
			}
			reverseStartStep = segmentStepLimit;
			q = 0.0;								// to make the debug output consistent
			state = DMState::cartLinear;
		}
		else
		{
			// n = distanceCarriedForwards + u * t + 0.5 * a * t^2
			// Therefore 0.5 * t^2 + u * t/a + (distanceCarriedForwards - n)/a = 0
			// Therefore t = -u/a +/- sqrt((u/a)^2 - 2 * (distanceCarriedForwards - n)/a)
			// Calculate the t0, p and q coefficients for an accelerating or decelerating move such that t = t0 + sqrt(p*n + q) and set up the initial direction
			newDirection = (seg->GetU() == (motioncalc_t)0.0) ? (seg->GetA() > (motioncalc_t)0.0) : (seg->GetU() > (motioncalc_t)0.0);
			motioncalc_t multiplier = (newDirection) ? (motioncalc_t)1.0 : (motioncalc_t)-1.0;
			const int32_t netStepsInInitialDirection = (newDirection) ? netStepsThisSegment : -netStepsThisSegment;

			if (t0 <= (motioncalc_t)0.0)
			{
				// The direction reversal is in the past
				segmentStepLimit = reverseStartStep = netStepsInInitialDirection + 1;
				state = DMState::cartAccel;
			}
			else if (t0 < (motioncalc_t)segments->GetDuration())
			{
				// Reversal is potentially in this segment, but it may be before the first step, or may be beyond the last step we are going to take
				// It can happen that the target end speed is zero but due to FP rounding error, distanceToReverse was just below netStepsInInitialDirection and got rounded down
				// To avoid this and other issues, don't do a reversing movement unless there is more than 1 reverse step.
				const motioncalc_t distanceToReverse = (segments->GetDistanceToReverse() + distanceCarriedForwards) * multiplier;
				const int32_t netStepsBeforeReverse = (int32_t)distanceToReverse;
				if (netStepsBeforeReverse <= netStepsInInitialDirection + 1)
				{
					segmentStepLimit = reverseStartStep = netStepsInInitialDirection + 1;
					state = DMState::cartDecelNoReverse;
				}
				else if (netStepsBeforeReverse == 0)
				{
					// Reversal happens immediately
					newDirection = !newDirection;
					multiplier = -multiplier;
					segmentStepLimit = reverseStartStep = 1 - netStepsInInitialDirection;
					state = DMState::cartAccel;
				}
				else
				{
					reverseStartStep = netStepsBeforeReverse + 1;
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
			const motioncalc_t rawP = (motioncalc_t)2.0/seg->GetA();
			p = rawP * multiplier;
			q = msquare(t0) - rawP * distanceCarriedForwards;
#if 0
			if (std::isinf(q))
			{
				debugPrintf("t0=%.1f mult=%.1f dcf=%.3e a=%.4e\n", (double)t0, (double)multiplier, (double)distanceCarriedForwards, (double)seg->GetA());
			}
#endif
		}

		nextStep = 1;
		if (nextStep < segmentStepLimit)
		{
			if (newDirection != direction)
			{
				direction = newDirection;
				directionChanged = true;
			}

			driversCurrentlyUsed = driversNormallyUsed;

#if 0	//DEBUG
			debugPrintf("New cart seg: state %u q=%.4e t0=%.4e p=%.4e ns=%" PRIi32 " ssl=%" PRIi32 "\n",
							(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit);
#endif
			return seg;
		}

#if 0
		debugPrintf("skipping seg: state %u q=%.4e t0=%.4e p=%.4e ns=%" PRIi32 " ssl=%" PRIi32 "\n",
						(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit);
		seg->DebugPrint('k');
#endif
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
bool DriveMovement::LogStepError(uint8_t type) noexcept
{
	state = DMState::stepError;
	stepErrorType = type;
	if (reprap.Debug(Module::Move))
	{
		debugPrintf("Step err %u on ", type);
		DebugPrint();
		MoveSegment::DebugPrintList(segments);
	}
	reprap.GetMove().LogStepError();
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
				return LogStepError(5);
			}
			if (currentMotorPosition - positionAtSegmentStart != netStepsThisSegment)
			{
				return LogStepError(6);
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

			if (unlikely((int32_t)(currentSegment->GetStartTime() <-prevEndTime) < -2))
			{
#if SEGMENT_DEBUG
				debugPrintf("step err1, %" PRIu32 ", %" PRIu32 "\n", currentSegment->GetStartTime(), prevEndTime);
				DebugPrint();
#endif
				return LogStepError(1);
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
		return LogStepError(4);
	}

	nextCalcStepTime += t0;

	if (unlikely(std::isnan(nextCalcStepTime) || nextCalcStepTime < (motioncalc_t)0.0))
	{
		if (reprap.Debug(Module::Move))
		{
			debugPrintf("nextCalcStepTime=%.3e\n", (double)nextCalcStepTime);
		}
		return LogStepError(2);
	}

	uint32_t iNextCalcStepTime = (uint32_t)nextCalcStepTime;

	if (iNextCalcStepTime > segments->GetDuration())
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// 2023-12-06: we now allow any step to be late but we record the maximum number.
		// 2024-040-5: we now allow steps to be late on any segment, not just the last one, because a segment may be 0 or 1 step long and on deltas the last 2 steps may be calculated late.
		iNextCalcStepTime = segments->GetDuration();
		const int32_t nextCalcStep = nextStep + (int32_t)stepsTillRecalc;
		const int32_t stepsLate = segmentStepLimit - nextCalcStep;
		if (stepsLate > maxStepsLate) { maxStepsLate = stepsLate; }
	}

	iNextCalcStepTime += segments->GetStartTime();
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

// This is called when the 'drive' (i.e. axis) concerned has no local drivers and we are not checking endstops or Z probe.
// Instead of generating an interrupt for each step of the remote drive, generate interrupts only occasionally and at the end of each segment, to keep the axis position fairly up to date.
// We must not call NewSegment significantly in advance of when the segment is due to start, to allow for segments being modified as new ones are added.
// To make sure this is the case we schedule an interrupt at the end of each segment, so that a segment cannot be started before the previous one has completed.
void DriveMovement::TakeStepsAndCalcStepTimeRarely(uint32_t clocksNow) noexcept
{
	MoveSegment *const currentSegment = segments;				// capture volatile variable
	if (state == DMState::ending)
	{
		currentMotorPosition = positionAtSegmentStart + netStepsThisSegment;
		distanceCarriedForwards += currentSegment->GetLength() - (motioncalc_t)netStepsThisSegment;
		segments = currentSegment->GetNext();
		MoveSegment::Release(currentSegment);
		if (NewSegment(clocksNow) == nullptr || state == DMState::starting)
		{
			return;
		}
	}

	const int32_t timeFromStart = (int32_t)(clocksNow - currentSegment->GetStartTime());
	currentMotorPosition = positionAtSegmentStart + lrintf((currentSegment->GetU() + ((motioncalc_t)0.5 * currentSegment->GetA() * (motioncalc_t)timeFromStart)) * (motioncalc_t)timeFromStart + distanceCarriedForwards);
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

// If the driver is moving, stop it, update the position and pass back the net steps taken in the executing segment.
// Return true if the drive was moving.
bool DriveMovement::StopDriver(int32_t& netStepsTaken) noexcept
{
	AtomicCriticalSectionLocker lock;

	if (state != DMState::idle)
	{
		state = DMState::idle;
		reprap.GetMove().DeactivateDM(this);
		netStepsTaken = GetNetStepsTaken();
		MoveSegment *seg = nullptr;
		std::swap(seg, const_cast<MoveSegment*&>(segments));
		MoveSegment::ReleaseAll(seg);
		if (homingDda != nullptr)
		{
			homingDda->SetDriveCoordinate(currentMotorPosition, drive);
		}
#if STEPS_DEBUG
		positionRequested = currentMotorPosition;
#endif
		return true;
	}

	netStepsTaken = 0;
	return false;
}

#if SUPPORT_REMOTE_COMMANDS

void DriveMovement::StopDriverFromRemote() noexcept
{
	int32_t dummy;
	(void)StopDriver(dummy);
}

#endif

// End
