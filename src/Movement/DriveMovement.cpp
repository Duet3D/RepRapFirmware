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
#include "StepTimer.h"
#include <Math/Isqrt.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

// Static members

int32_t DriveMovement::maxStepsLate = 0;

// Constructors
DriveMovement::DriveMovement() noexcept
{
}

// Non static members

void DriveMovement::DebugPrint() const noexcept
{
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
		const char *const errText = (state == DMState::stepError1) ? " ERR1:"
									: (state == DMState::stepError2) ? " ERR2:"
										: (state == DMState::stepError3) ? " ERR3:"
											: ":";
		debugPrintf("DM%c%s dir=%c next=%" PRIi32 " rev=%" PRIi32 " interval=%" PRIu32 " ssl=%" PRIi32 " A=%.4e B=%.4e C=%.4e\n",
						c, errText, (direction) ? 'F' : 'B', nextStep, reverseStartStep, stepInterval, segmentStepLimit,
							(double)q, (double)t0, (double)p);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Set the steps/mm for this driver
void DriveMovement::SetStepsPerMm(float p_stepsPerMm) noexcept
{
	stepsPerMm = p_stepsPerMm;
}

// This is called when segments has just been changed to a new segment. Return the new segment to execute, or nullptr.
MoveSegment *DriveMovement::NewCartesianSegment() noexcept
{
	while (true)
	{
		if (segments == nullptr)
		{
			return nullptr;
		}

		// Calculate the movement parameters
		bool newDirection;
		if (segments->IsLinear())
		{
			// n * mmPerStep = distanceCarriedForwards + u * t
			// Therefore t = -distanceCarriedForwards/u + n * mmPerStep/u
			// Calculate the t0 and p coefficients such that t = t0 + p*n
			t0 = -distanceCarriedForwards/segments->GetU() + segments->GetStartTime();
			p = 1.0/segments->GetU();
			q = 0.0;								// to make the debug output consistent
			const float segmentDistance = distanceCarriedForwards + segments->GetU() * segments->GetDuration();
			netStepsThisSegment = (int32_t)segmentDistance;
			if (netStepsThisSegment < 0)
			{
				newDirection = false;
				reverseStartStep = segmentStepLimit = 1 - netStepsThisSegment;
			}
			else
			{
				newDirection = true;
				reverseStartStep = segmentStepLimit = netStepsThisSegment + 1;
			}
			state = DMState::cartLinear;
		}
		else
		{
			// n * mmPerStep = distanceCarriedForwards + u * t + 0.5 * a * t^2
			// Therefore 0.5 * t^2 + u * t/a + (distanceCarriedForwards - mmPerStep * n)/a = 0
			// Therefore t = -u/a +/- sqrt((u/a)^2 - 2 * (distanceCarriedForwards - mmPerStep * n)/a)
			// Calculate the t0, p and q coefficients for an accelerating or decelerating move such that t = t0 + sqrt(p*n + q)
			const float uDivA = segments->GetU()/segments->GetA();
			t0 = segments->GetStartTime() - uDivA;
			p = 2.0/segments->GetA();
			q = fsquare(uDivA) - 2 * distanceCarriedForwards/segments->GetA();

			const float segmentDistance = distanceCarriedForwards + segments->GetLength();
			netStepsThisSegment = (int32_t)segmentDistance;
			if (uDivA >= 0.0)
			{
				// Any reversal is in the past
				if (netStepsThisSegment >= 0)
				{
					newDirection = true;
					segmentStepLimit = reverseStartStep = netStepsThisSegment + 1;
					state = DMState::cartAccel;
				}
				else
				{
					newDirection = false;
					segmentStepLimit = reverseStartStep = 1 - netStepsThisSegment;
					state = DMState::cartDecelNoReverse;
				}
			}
			else if (-uDivA < segments->GetDuration())
			{
				// Reversal is in this segment, but it may be the first step, or may be beyond the last step we are going to take
				const float distanceToReverse = segments->GetDistanceToReverse() + distanceCarriedForwards;
				const int32_t netStepsBeforeReverse = (int32_t)distanceToReverse;
				if (netStepsBeforeReverse <= 0)
				{
					newDirection = false;
					segmentStepLimit = reverseStartStep = netStepsThisSegment + 1;
					state = DMState::cartDecelNoReverse;
				}
				else if (netStepsBeforeReverse <= netStepsThisSegment)
				{
					newDirection = true;
					segmentStepLimit = reverseStartStep = netStepsThisSegment + 1;
					state = DMState::cartDecelNoReverse;
				}
				else
				{
					newDirection = true;
					reverseStartStep = netStepsBeforeReverse + 1;
					segmentStepLimit = 2 * reverseStartStep - netStepsThisSegment - 1;
					state = DMState::cartDecelForwardsReversing;
				}
			}
			else
			{
				// Reversal doesn't occur until after the end of this segment
				newDirection = true;
				segmentStepLimit = reverseStartStep = netStepsThisSegment + 1;
				state = DMState::cartDecelNoReverse;
			}
		}

		nextStep = 1;
		if (nextStep < segmentStepLimit)
		{
			if (newDirection != direction)
			{
				direction = newDirection;
				directionChanged = true;
			}

#if 1	//DEBUG
			debugPrintf("New cart seg: state %u q=%.4e t0=%.4e p=%.4e ns=%" PRIu32 " ssl=%" PRIu32 "\n",
							(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit);
#endif
			return segments;
		}

		segments = segments->GetNext();						// skip this segment
	}
}

#if 0	///these will be removed

// Prepare this DM for a Cartesian axis move, returning true if there are steps to do
bool DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept
{
	distanceSoFar = 0.0;
	timeSoFar = 0.0;
	mp.cart.pressureAdvanceK = 0.0;
	// We can't use directionVector here because those values relate to Cartesian space, whereas we may be CoreXY etc.
	mp.cart.effectiveStepsPerMm =
#if SUPPORT_REMOTE_COMMANDS
									(dda.flags.isRemote) ? (float)totalSteps	// because totalDistance = 1.0
										: (float)totalSteps/dda.totalDistance;
#else
									(float)totalSteps/dda.totalDistance;
#endif
	mp.cart.effectiveMmPerStep = 1.0/mp.cart.effectiveStepsPerMm;
	isDelta = false;
	isExtruder = false;
	segments = dda.segments;
	nextStep = 1;									// must do this before calling NewCartesianSegment
	directionChanged = directionReversed = false;	// must clear these before we call NewCartesianSegment

	if (!NewCartesianSegment())
	{
		return false;
	}

	// Prepare for the first step
	nextStepTime = 0;
	stepsTakenThisSegment = 0;						// no steps taken yet since the start of the segment
	stepInterval = 0;								// to keep the debug output deterministic
	return CalcNextStepTimeFull();					// calculate the scheduled time of the first step
}

// If there are no steps to do, set nextStep = 0 so that DDARing::CurrentMoveCompleted doesn't add any steps to the movement accumulator
// We have already generated the extruder segments and we know that there are some
// effStepsPerMm is the number of extruder steps needed per mm of totalDistance before we apply pressure advance
// A note on accumulating partial extruder steps:
// We must only accumulate partial steps when the extrusion is forwards. If we try to accumulate partial steps on reverse extrusion too,
// things go horribly wrong under particular circumstances. We use the pressure advance flag as a proxy for forward extrusion.
// This means that partial extruder steps don't get accumulated on a reprime move, but that is probably a good thing because it will
// behave in a similar way to a retraction move.
bool DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params, float signedEffStepsPerMm) noexcept
{
	const float effStepsPerMm = fabsf(signedEffStepsPerMm);
	mp.cart.effectiveStepsPerMm = effStepsPerMm;
	const float effMmPerStep = 1.0/effStepsPerMm;
	mp.cart.effectiveMmPerStep = effMmPerStep;

	timeSoFar = 0.0;
	segments = dda.segments;
	isDelta = false;
	isExtruder = true;
	nextStep = 1;									// must do this before calling NewExtruderSegment
	totalSteps = 0;									// we don't use totalSteps but set it to 0 to avoid random values being printed by DebugPrint
	directionChanged = directionReversed = false;	// must clear these before we call NewExtruderSegment

	const size_t logicalDrive =
#if SUPPORT_REMOTE_COMMANDS
								(dda.flags.isRemote) ? drive : LogicalDriveToExtruder(drive);
#else
								LogicalDriveToExtruder(drive);
#endif
	ExtruderShaper& shaper = reprap.GetMove().GetExtruderShaper(logicalDrive);

	// distanceSoFar will accumulate the equivalent amount of totalDistance that the extruder moves forwards.
	// It would be equal to totalDistance if there was no pressure advance and no extrusion pending.
	if (dda.flags.usePressureAdvance)
	{
		const float extrusionPending = shaper.GetExtrusionPending();
		reprap.GetMove().UpdateExtrusionPendingLimits(extrusionPending);
		distanceSoFar = extrusionPending * effMmPerStep;
		mp.cart.pressureAdvanceK = shaper.GetKclocks();
	}
	else
	{
		mp.cart.pressureAdvanceK = 0.0;
		distanceSoFar =	0.0;
	}

#if 0	//DEBUG
	const float distanceBroughtForwards = distanceSoFar;	// for debug use only
#endif
	if (!NewExtruderSegment())						// if no steps to do
	{
		if (dda.flags.usePressureAdvance)
		{
			shaper.SetExtrusionPending(distanceSoFar * effStepsPerMm);
#if 0	// DEBUG
			if (reprap.Debug(Module::Dda) && (shaper.GetExtrusionPending() > 1.0 || shaper.GetExtrusionPending() < -1.0))
			{
				AtomicCriticalSectionLocker lock;
				debugPrintf("pex xpend=%.2f effsm=%.2f dbf=%.3f\n", (double)shaper.GetExtrusionPending(), (double)effStepsPerMm, (double)distanceBroughtForwards);
				char ch = '0';
				for (const MoveSegment *seg = dda.segments; seg != nullptr; seg = seg->GetNext())
				{
					seg->DebugPrint(ch);
					++ch;
				}
			}
#endif
		}
		return false;								// quit if no steps to do
	}

	// Prepare for the first step
	nextStepTime = 0;
	stepsTakenThisSegment = 0;						// no steps taken yet since the start of the segment
	stepInterval = 0;								// to keep the debug output deterministic
	return CalcNextStepTimeFull();					// calculate the scheduled time of the first step
}

#endif

// Version of fastSqrtf that allows for slightly negative operands caused by rounding error
static inline float fastLimSqrtf(float f) noexcept
{
#if 1
	return fastSqrtf(f);							// the fastSqrtf function in RRFLibraries already returns zero if the operand is negative
#else
	return (f > 0.0) ? fastSqrtf(f) : 0.0;
#endif
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
			MoveSegment *oldSegment = currentSegment;
			segments = currentSegment = oldSegment->GetNext();
			MoveSegment::Release(oldSegment);
			currentSegment = NewCartesianSegment();
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
		nextCalcStepTime = t0 + (float)(nextStep + (int32_t)stepsTillRecalc) * p;
		break;

	case DMState::cartAccel:									// Cartesian accelerating
		nextCalcStepTime = t0 + fastLimSqrtf(q + p * (float)(nextStep + (int32_t)stepsTillRecalc));
		break;

	case DMState::cartDecelForwardsReversing:
		if (nextStep + (int32_t)stepsTillRecalc < reverseStartStep)
		{
			nextCalcStepTime = t0 - fastLimSqrtf(q + p * (float)(nextStep + (int32_t)stepsTillRecalc));
			break;
		}

		CheckDirection(true);
		state = DMState::cartDecelReverse;
		// no break
	case DMState::cartDecelReverse:								// Cartesian decelerating, reverse motion. Convert the steps to int32_t because the net steps may be negative.
		{
			const int32_t netSteps = 2 * reverseStartStep - nextStep - 1;
			nextCalcStepTime = t0 + fastLimSqrtf(q + p * (float)(netSteps - (int32_t)stepsTillRecalc));
		}
		break;

	case DMState::cartDecelNoReverse:							// Cartesian decelerating with no reversal
		nextCalcStepTime = t0 - fastLimSqrtf(q + p * (float)(nextStep + (int32_t)stepsTillRecalc));
		break;

	default:
		return false;
	}

#if 0	//DEBUG
	if (std::isnan(nextCalcStepTime) || nextCalcStepTime < 0.0)
	{
//		debugPrintf("step err3\n");
		state = DMState::stepError3;
		return false;
	}
#endif

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
		const int32_t stepsLate = segmentStepLimit  - nextCalcStep;
		if (stepsLate > maxStepsLate) { maxStepsLate = stepsLate; }
	}

	if (nextStep == 1)
	{
		nextStepTime = iNextCalcStepTime;									// shiftFactor must be 0
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
	}

#if 0	//DEBUG
	if (isExtruder && stepInterval < 20 /*&& nextStep + stepsTillRecalc + 1 < totalSteps*/)
	{
		state = DMState::stepError;
		return false;
	}
#endif

	nextStepTime = iNextCalcStepTime - (stepsTillRecalc * stepInterval) + segments->GetStartTime();
	return true;
}

// End
