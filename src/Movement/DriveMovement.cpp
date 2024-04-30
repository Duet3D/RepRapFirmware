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
	distanceCarriedForwards = 0.0;
	currentMotorPosition = 0;
	nextDM = nullptr;
	segments = nullptr;
	isExtruder = false;
	segmentFlags.Init();
	homingDda = nullptr;
}

void DriveMovement::DebugPrint() const noexcept
{
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
		const char *const errText = (state == DMState::stepError1) ? " ERR1:"
									: (state == DMState::stepError2) ? " ERR2:"
										: (state == DMState::stepError3) ? " ERR3:"
											: ":";
		debugPrintf("DM%c%s state=%u dir=%c next=%" PRIi32 " rev=%" PRIi32 " interval=%" PRIu32 " ssl=%" PRIi32 " q=%.4e t0=%.4e p=%.4e dcf=%.2f\n",
						c, errText, (unsigned int)state, (direction) ? 'F' : 'B', nextStep, reverseStartStep, stepInterval, segmentStepLimit,
							(double)q, (double)t0, (double)p, (double)distanceCarriedForwards);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Add a segment into the list. If the list is not empty then the new segment may overlap segments already in the list but will never start earlier than the first existing one.
// The units of the input parameters are steps for distance and step clocks for time.
void DriveMovement::AddSegment(uint32_t startTime, uint32_t duration, float distance, float u, float a, MovementFlags moveFlags) noexcept
{
	// Adjust the initial speed and distance to account for pressure advance
	if (isExtruder && !moveFlags.nonPrintingMove)
	{
		const float extraSpeed = a * extruderShaper.GetKclocks();
		u += extraSpeed;
		distance += extraSpeed * (float)duration;
	}

	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Segments))
	{
		debugPrintf("Adding seg: st=%" PRIu32 " t=%" PRIu32 " dist=%.2f u=%.3e a=%.3e\n", startTime, duration, (double)distance, (double)u, (double)a);
	}

	MoveSegment *prev = nullptr;

	// Shut out the step interrupt and task switching while we mess with the segments
	// TODO probably only need to shut out task switching here, or shut out the step interrupt but leave the UART interrupt enabled
	AtomicCriticalSectionLocker lock;

	// Find the earliest existing segment that the new one will overlap
	MoveSegment *seg = segments;
	int32_t offset;
	while (seg != nullptr)
	{
		offset = (int32_t)(startTime - seg->GetStartTime());
		if (   offset <= 0												// new segment starts before the old one does
			|| (float)(offset + MoveSegment::MinDuration) < (uint32_t)seg->GetDuration()
		   )
		{
			// The segment we wish to add starts before the one at 'seg' ends. It should not start before the one at 'seg' starts, but if it does then just postpone it.
			// If it starts significantly later, split the existing segment
			if (offset < MoveSegment::MinDuration)
			{
				startTime = seg->GetStartTime();
			}
			else
			{
				// Split the existing segment
				prev = seg;
				seg = seg->Split((uint32_t)offset);
			}

			// The segment we wish to add now starts at the same time as 'seg' but it may end earlier or later than the one at 'seg' does.
			const int32_t timeDifference = (int32_t)(duration - (uint32_t)seg->GetDuration());
			if (timeDifference > MoveSegment::MinDuration)
			{
				// The existing segment is shorter in time than the new one, so add the new segment in two or more parts
				const float firstDistance = (u + 0.5 * a * seg->GetDuration()) * seg->GetDuration();	// distance moved by the first part of the new segment
#if SEGMENT_DEBUG
				debugPrintf("merge1: ");
#endif
				seg->Merge(firstDistance, u, a, moveFlags);
				distance -= firstDistance;
				startTime += seg->GetDuration();
				u += a * seg->GetDuration();
				duration = (uint32_t)timeDifference;
				prev = seg;
				seg = seg->GetNext();
				continue;
			}

			// New segment ends earlier or at the same time as the old one
			if (timeDifference < -MoveSegment::MinDuration)
			{
				// Split the existing segment in two
				seg->Split(duration);
			}
			else
			{
				// Make the new segment duration fit the existing one by adjusting the initial speed slightly
#if SEGMENT_DEBUG
				debugPrintf("Adjusting t=%" PRIu32 " u=%.4e a=%.4e", duration, (double)u, (double)a);
#endif
				u = ((u * duration) - (a * (float)timeDifference * (duration + 0.5 * (float)timeDifference)))/seg->GetDuration();
				duration = seg->GetDuration();
#if SEGMENT_DEBUG
				debugPrintf(" to t=%" PRIu32 " u=%.4e a=%.4e\n", duration, (double)u, (double)a);
#endif
			}

			// The new segment and the existing one now have the same start time and duration, so merge them
#if SEGMENT_DEBUG
			debugPrintf("merge2: ");
#endif
			seg->Merge(distance, u, a, moveFlags);
#if SEGMENT_DEBUG
			MoveSegment::DebugPrintList('m', segments);
#endif
			return;
		}

		prev = seg;
		seg = seg->GetNext();
	}

	// The new segment (or what's left of it) needs to be added at the end
	seg = MoveSegment::Allocate(nullptr);
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
	MoveSegment::DebugPrintList('r', segments);
#endif
}

// Set up to schedule the first segment, returning true if there is a segment to be processed
bool DriveMovement::ScheduleFirstSegment() noexcept
{
	if (NewSegment() != nullptr)
	{
		return CalcNextStepTimeFull();
	}
	return false;
}

// This is called when 'segments' has just been changed to a new segment. Return the new segment to execute, or nullptr.
MoveSegment *DriveMovement::NewSegment() noexcept
{
	while (true)
	{
		MoveSegment *seg = segments;			// capture volatile variable
		if (seg == nullptr)
		{
			segmentFlags.Init();
			return nullptr;
		}

		// Calculate the movement parameters
		bool newDirection;
		netStepsThisSegment = (int32_t)(seg->GetLength() + distanceCarriedForwards);
		if (seg->NormaliseAndCheckLinear(distanceCarriedForwards, t0))
		{
			// n = distanceCarriedForwards + u * t
			// Therefore t = -distanceCarriedForwards/u + n/u = t0 + n/u
			if (seg->GetU() < 0)
			{
				newDirection = false;
				p = -1.0/seg->GetU();
				segmentStepLimit = 1 - netStepsThisSegment;
			}
			else
			{
				newDirection = true;
				p = 1.0/seg->GetU();
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
			newDirection = (seg->GetU() == 0.0) ? (seg->GetA() > 0.0) : (seg->GetU() > 0.0);
			float multiplier = (newDirection) ? 1.0 : -1.0;
			int32_t stepsInInitialDirection = (newDirection) ? netStepsThisSegment : -netStepsThisSegment;

			if (t0 <= 0.0)
			{
				// The direction reversal is in the past
				segmentStepLimit = reverseStartStep = stepsInInitialDirection + 1;
				state = DMState::cartAccel;
			}
			else if (t0 < segments->GetDuration())
			{
				// Reversal is in this segment, but it may be the first step, or may be beyond the last step we are going to take
				float distanceToReverse = (segments->GetDistanceToReverse() + distanceCarriedForwards) * multiplier;
				const int32_t netStepsBeforeReverse = (int32_t)distanceToReverse;
				if (netStepsBeforeReverse == 0)
				{
					// Reversal happens immediately
					newDirection = !newDirection;
					multiplier = -multiplier;
					segmentStepLimit = reverseStartStep = 1 - stepsInInitialDirection;
					state = DMState::cartAccel;
				}
				else if (netStepsBeforeReverse >= stepsInInitialDirection)
				{
					segmentStepLimit = reverseStartStep = stepsInInitialDirection + 1;
					state = DMState::cartDecelNoReverse;
				}
				else
				{
					reverseStartStep = netStepsBeforeReverse + 1;
					segmentStepLimit = 2 * reverseStartStep - stepsInInitialDirection - 1;
					state = DMState::cartDecelForwardsReversing;
				}
			}
			else
			{
				// Reversal doesn't occur until after the end of this segment
				segmentStepLimit = reverseStartStep = stepsInInitialDirection + 1;
				state = DMState::cartDecelNoReverse;
			}
			p = (2.0 * multiplier)/seg->GetA();
			q = fsquare(t0) - 2.0 * multiplier * distanceCarriedForwards/seg->GetA();
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

			segmentFlags = seg->GetFlags();

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

// Version of fastSqrtf that allows for slightly negative operands caused by rounding error
static inline float fastLimSqrtf(float f) noexcept
{
	return (f > 0.0) ? fastSqrtf(f) : 0.0;
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// We have already incremented nextStep and checked that it does not exceed totalSteps, so at least one more step is due
// Return true if all OK, false if no more segments to execute
bool DriveMovement::CalcNextStepTimeFull() noexcept
pre(nextStep <= totalSteps; stepsTillRecalc == 0)
{
	MoveSegment *currentSegment = segments;
	uint32_t shiftFactor = 0;										// assume single stepping
	{
		int32_t stepsToLimit = segmentStepLimit - nextStep;
		// If there are no more steps left in this segment, skip to the next segment and use single stepping
		if (stepsToLimit == 0)
		{
			distanceCarriedForwards += currentSegment->GetLength() - netStepsThisSegment;
			segments = currentSegment->GetNext();
			MoveSegment::Release(currentSegment);
			currentSegment = NewSegment();
			if (currentSegment == nullptr)
			{
				state = DMState::idle;
				return false;
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

	float nextCalcStepTime;

	// Work out the time of the step
	switch (state)
	{
	case DMState::cartLinear:									// linear steady speed
		nextCalcStepTime = (float)(nextStep + (int32_t)stepsTillRecalc) * p;
		break;

	case DMState::cartAccel:									// Cartesian accelerating
		nextCalcStepTime = fastLimSqrtf(q + p * (float)(nextStep + (int32_t)stepsTillRecalc));
		break;

	case DMState::cartDecelForwardsReversing:
		if (nextStep + (int32_t)stepsTillRecalc < reverseStartStep)
		{
			nextCalcStepTime = -fastLimSqrtf(q + p * (float)(nextStep + (int32_t)stepsTillRecalc));
			break;
		}

		CheckDirection(true);
		state = DMState::cartDecelReverse;
		// no break
	case DMState::cartDecelReverse:								// Cartesian decelerating, reverse motion. Convert the steps to int32_t because the net steps may be negative.
		{
			const int32_t netSteps = 2 * reverseStartStep - nextStep - 1;
			nextCalcStepTime = fastLimSqrtf(q + p * (float)(netSteps - (int32_t)stepsTillRecalc));
		}
		break;

	case DMState::cartDecelNoReverse:							// Cartesian decelerating with no reversal
		nextCalcStepTime = -fastLimSqrtf(q + p * (float)(nextStep + (int32_t)stepsTillRecalc));
		break;

	default:
#if 0
		debugPrintf("DMstate %u, quitting\n", (unsigned int)state);
#endif
		return false;
	}

	nextCalcStepTime += t0;

	if (std::isnan(nextCalcStepTime) || nextCalcStepTime < 0.0)
	{
#if 0	//DEBUG
		debugPrintf("step err3, %.2f\n", (double)nextCalcStepTime);
		DebugPrint();
#endif
		state = DMState::stepError3;
		return false;
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
			stepInterval = (uint32_t)interval >> shiftFactor;				// calculate the time per step, ready for next time
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
			return false;
		}
#endif

		nextStepTime = iNextCalcStepTime - (stepsTillRecalc * stepInterval);
	}

	return true;
}

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
		currentMotorPosition += netStepsTaken;
		MoveSegment *seg = nullptr;
		std::swap(seg, const_cast<MoveSegment*&>(segments));
		MoveSegment::ReleaseAll(seg);
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
