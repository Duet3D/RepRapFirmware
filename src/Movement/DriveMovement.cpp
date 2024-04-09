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

#if SUPPORT_LINEAR_DELTA
# include "Kinematics/LinearDeltaKinematics.h"
#endif

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
		debugPrintf("DM%c%s dir=%c next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32 " ssl=%" PRIu32 " A=%.4e B=%.4e C=%.4e",
						c, errText, (direction) ? 'F' : 'B', nextStep, reverseStartStep, stepInterval, segmentStepLimit,
							(double)q, (double)t0, (double)p);
#if SUPPORT_LINEAR_DELTA
		if (isDelta)
		{
			debugPrintf(" hmz0s=%.4e drev=%.4e\n",
						(double)mp.delta.fHmz0s, (double)mp.delta.reverseStartDistance);
		}
		else
#endif
		{
			debugPrintf("\n");
		}
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
	mmPerStep = 1.0/p_stepsPerMm;
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
		if (segments->IsLinear())
		{
			// n * mmPerStep = distanceCarriedForwards + u * t
			// Therefore t = -distanceCarriedForwards/u + n * mmPerStep/u
			// Calculate the t0 and p coefficients such that t = t0 + p*n
			t0 = -distanceCarriedForwards /segments->GetU();
			p = mmPerStep/segments->GetU();
			q = 0.0;								// to make the debug output consistent
			const float distance = distanceCarriedForwards + segments->GetU() * segments->GetDuration();
			const int32_t netSteps = (int32_t)(distance * stepsPerMm);
			if (netSteps < 0)
			{
				netSteps = -netSteps;
				direction = false;
			}
			else
			{
				direction = true;
			}
			reverseStartStep = segmentStepLimit = netSteps + 1;
			state = DMState::cartLinear;
		}
		else
		{
			// n * mmPerStep = distanceCarriedForwards + u * t + 0.5 * a * t^2
			// Therefore 0.5 * t^2 + u * t/a + (distanceCarriedForwards - mmPerStep * n)/a = 0
			// Therefore t = -u/a +/- sqrt((u/a)^2 - 2 * (distanceCarriedForwards - mmPerStep * n)/a)
			// Calculate the t0, p and q coefficients for an accelerating or decelerating move such that t = t0 + sqrt(p*n + q)
			t0 = -segments->GetU()/segments->GetA();
			p = 2 * mmPerStep/segments->GetA();
			q = fsquare(t0) - 2 * distanceCarriedForwards/segments->GetA();
			const float netDistance = distanceCarriedForwards + (segments->GetU() + 0.5 * segments->GetA() * segments->GetDuration) * segments->GetDuration();
			const float timeToReverse = segments->GetU()/(-segments->GetA());
			if (timeToReverse < 0.0)
			{
				// Any reversal is in the past
				qq;
				state = DMState::cartAccelNoReverse;
			}
			else if (timeToReverse < segments->duration)
			{
				// Reversal is in this segment, but it may be beyond the last step we are going to take
				const float distanceToReverse = segments->GetDistanceToReverse();
				if (qq)
				{
					qq;
					state = DMState::cartAccelReversing;
				}
				else
				{
					qq;
					state = DMState::cartAccelNoReverse;
				}
			}
			else
			{
				qq;
				state = DMState::cartAccelNoReverse;
			}
		}

		// Save the movement limit in steps
		const int32_t netStepsToSegmentEnd = (int32_t)floorf(segments->GetDistance() * stepsPerMm);
		segmentStepLimit = qq;		//TODO calculate segmentStepLimit and reverseStartStep
		reverseStartStep = qq;

		nextStep = 1;
		if (nextStep < segmentStepLimit)
		{
			qq;										//TODO set direction, clear direction changed
#if 0	//DEBUG
			if (__get_BASEPRI() == 0)
			{
				debugPrintf("New cart seg: state %u A=%.4e B=%.4e C=%.4e ns=%" PRIu32 " ssl=%" PRIu32 "\n",
							(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit);
			}
#endif
			return segments;
		}

		segments = segments->GetNext();						// skip this segment
	}
}

#if SUPPORT_LINEAR_DELTA

// This is called when segments has just been changed to a new segment. Return the new segment to execute, or nullptr.
MoveSegment *DriveMovement::NewDeltaSegment() noexcept
{
	while (true)
	{
		if (segments == nullptr)
		{
			return nullptr;
		}

		// Work out whether we reverse in this segment and the movement limit in steps.
		// First check whether the first step in this segment is the previously-calculated reverse start step, and if so then do the reversal.
		if (nextStep == reverseStartStep)
		{
			direction = false;					// we must have been going up, so now we are going down
			directionChanged = directionReversed = true;
		}

		if (segments->GetNext() == nullptr)
		{
			// This is the last segment, so the phase step limit is the number of total steps, and we can avoid some calculation
			segmentStepLimit = totalSteps + 1;
			state = (reverseStartStep <= totalSteps && nextStep < reverseStartStep) ? DMState::deltaForwardsReversing : DMState::deltaNormal;
		}
		else
		{
			// Work out how many whole steps we have moved up or down at the end of this segment
			const DeltaMoveSegment *const deltaSeg = (const DeltaMoveSegment*)segments;
			const float sDx = distanceSoFar * deltaSeg->GetDv()[0];
			const float sDy = distanceSoFar * deltaSeg->GetDv()[1];
			int32_t netStepsAtEnd = (int32_t)floorf(fastSqrtf(deltaSeg->GetfDSquaredMinusAsquaredMinusBsquaredTimesSsquared() - fsquare(stepsPerMm) * (sDx * (sDx + deltaSeg->GetfTwoA())
																+ sDy * (sDy + deltaSeg->GetfTwoB())))
													+ (distanceSoFar * deltaSeg->GetDv()[2] - deltaSeg->GetH0MinusZ0()) * stepsPerMm);

			// If there is a reversal then we only ever move up by (reverseStartStep - 1) steps, so netStepsAtEnd should be less than reverseStartStep.
			// However, because of rounding error, it might possibly be equal.
			// If there is no reversal then reverseStartStep is set to totalSteps + 1, so netStepsAtEnd must again be less than reverseStartStep.
			if (netStepsAtEnd >= reverseStartStep)
			{
				netStepsAtEnd = reverseStartStep - 1;							// correct the rounding error - we know that reverseStartStep cannot be 0 so subtracting 1 is safe
			}

			if (!direction)
			{
				// We are going down so any reversal has already happened
				state = DMState::deltaNormal;
				segmentStepLimit = (nextStep >= reverseStartStep)
									? (2 * reverseStartStep) - netStepsAtEnd	// we went up (reverseStartStep-1) steps, now we are going down to netStepsAtEnd
										: -netStepsAtEnd;						// we are just going down to netStepsAtEnd
			}
			else if (distanceSoFar <= mp.delta.reverseStartDistance)
			{
				// This segment is purely upwards motion of the tower
				state = DMState::deltaNormal;
				segmentStepLimit = netStepsAtEnd + 1;
			}
			else
			{
				// This segment ends with reverse motion
				segmentStepLimit = (2 * reverseStartStep) - netStepsAtEnd;
				state = DMState::deltaForwardsReversing;
			}
		}

		if (segmentStepLimit > nextStep)
		{
			return segments;
		}

		segments = segments->GetNext();
	}
}

#endif // SUPPORT_LINEAR_DELTA

// This is called for an extruder driver when segments has just been changed to a new segment. Return the new segment to execute, or nullptr.
MoveSegment *DriveMovement::NewExtruderSegment() noexcept
{
	while (true)
	{
		if (segments == nullptr)
		{
			return nullptr;
		}

		if (segments->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * stepNumber
			state = DMState::cartLinear;
			reverseStartStep = segmentStepLimit = (int32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm) + 1;
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * stepNumber)
			distanceSoFar += segments->GetSpeedChange() * mp.cart.pressureAdvanceK;		// add the extra extrusion due to pressure advance to the extrusion done at the end of this move
			const int32_t netStepsAtSegmentEnd = (int32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm);
			const float endSpeed = segments->GetEndSpeed(mp.cart.pressureAdvanceK);
			if (segments->IsAccelerating())
			{
				state = DMState::cartAccel;
				reverseStartStep = segmentStepLimit = netStepsAtSegmentEnd + 1;
			}
			else
			{
				// This is a decelerating segment. If it includes pressure advance then it may include reversal.
				if (endSpeed >= 0.0)
				{
					state = DMState::cartDecelNoReverse;					// this segment is forwards throughout
					reverseStartStep = segmentStepLimit = netStepsAtSegmentEnd + 1;
					CheckDirection(false);
				}
				else
				{
					const float startSpeed = segments->GetStartSpeed(mp.cart.pressureAdvanceK);
					if (startSpeed <= 0.0)
					{
						state = DMState::cartDecelReverse;					// this segment is reverse throughout
						reverseStartStep = nextStep;
						CheckDirection(true);
					}
					else
					{
						// This segment starts forwards and then reverses. Either or both of the forward and reverse segments may be small enough to need no steps.
						const float distanceToReverse = segments->GetDistanceToReverse() + startDistance;
						const int32_t netStepsToReverse = (int32_t)(distanceToReverse * mp.cart.effectiveStepsPerMm);
						if (nextStep <= netStepsToReverse)
						{
							// There is at least one step before we reverse
							reverseStartStep = netStepsToReverse + 1;
							state = DMState::cartDecelForwardsReversing;
							CheckDirection(false);
						}
						else
						{
							// There is no significant forward phase, so start in reverse
							reverseStartStep = nextStep;					// they are probably equal anyway, but just in case...
							state = DMState::cartDecelReverse;
							CheckDirection(true);
						}
					}
					segmentStepLimit = (2 * reverseStartStep) - netStepsAtSegmentEnd - 1;
				}
			}
		}

#if 0	//DEBUG
		if (__get_BASEPRI() == 0)
		{
			debugPrintf("New ex seg: state %u A=%.4e B=%.4e C=%.4e ns=%" PRIi32 " ssl=%" PRIi32 " rss=%" PRIi32 "\n",
						(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit, reverseStartStep);
		}
#endif
		if (nextStep < segmentStepLimit)
		{
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
	reverseStartStep = totalSteps + 1;				// no reverse phase
	return CalcNextStepTimeFull();					// calculate the scheduled time of the first step
}

#if SUPPORT_LINEAR_DELTA

// Prepare this DM for a Delta axis move, returning true if there are steps to do
bool DriveMovement::PrepareDeltaAxis(const DDA& dda, const PrepParams& params) noexcept
{
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float A = params.initialX - params.dparams->GetTowerX(drive);
	const float B = params.initialY - params.dparams->GetTowerY(drive);
	const float aAplusbB = A * dda.directionVector[X_AXIS] + B * dda.directionVector[Y_AXIS];
	const float dSquaredMinusAsquaredMinusBsquared = params.dparams->GetDiagonalSquared(drive) - fsquare(A) - fsquare(B);
	const float h0MinusZ0 = fastSqrtf(dSquaredMinusAsquaredMinusBsquared);

	mp.delta.h0MinusZ0 = h0MinusZ0;
	mp.delta.fTwoA = 2.0 * A;
	mp.delta.fTwoB = 2.0 * B;
	mp.delta.fHmz0s = h0MinusZ0 * stepsPerMm;
	mp.delta.fMinusAaPlusBbTimesS = -(aAplusbB * stepsPerMm);
	mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared = dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm);

	// Calculate the distance at which we need to reverse direction.
	if (params.a2plusb2 <= 0.0)
	{
		// Pure Z movement. We can't use the main calculation because it divides by params.a2plusb2.
		direction = (dda.directionVector[Z_AXIS] >= 0.0);
		mp.delta.reverseStartDistance = (direction) ? dda.totalDistance + 1.0 : -1.0;	// so that we never reverse and NewDeltaSegment knows which way we are going
		reverseStartStep = totalSteps + 1;
	}
	else
	{
		// The distance to reversal is the solution to a quadratic equation. One root corresponds to the carriages being below the bed,
		// the other root corresponds to the carriages being above the bed.
		const float drev = ((dda.directionVector[Z_AXIS] * fastSqrtf(params.a2plusb2 * params.dparams->GetDiagonalSquared(drive) - fsquare(A * dda.directionVector[Y_AXIS] - B * dda.directionVector[X_AXIS])))
							- aAplusbB)/params.a2plusb2;
		mp.delta.reverseStartDistance = drev;
		if (drev <= 0.0)
		{
			// No reversal, going down
			reverseStartStep = totalSteps + 1;
			direction = false;
		}
		else if (drev >= dda.totalDistance)
		{
			// No reversal, going up
			reverseStartStep = totalSteps + 1;
			direction = true;
		}
		else																	// the reversal point is within range
		{
			// Calculate how many steps we need to move up before reversing
			const float hrev = dda.directionVector[Z_AXIS] * drev + fastSqrtf(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - params.a2plusb2 * fsquare(drev));
			const int32_t numStepsUp = (int32_t)((hrev - mp.delta.h0MinusZ0) * stepsPerMm);

			// We may be going down but almost at the peak height already, in which case we don't really have a reversal.
			// However, we could be going up by a whole step due to rounding, so we need to check the direction
			if (numStepsUp < 1)
			{
				if (direction)
				{
					mp.delta.reverseStartDistance = dda.totalDistance + 1.0;	// indicate that there is no reversal
				}
				else
				{
					mp.delta.reverseStartDistance = -1.0;						// so that we know we have reversed already
					reverseStartStep = totalSteps + 1;
				}
			}
			else if (direction && numStepsUp <= totalSteps)
			{
				// If numStepsUp == totalSteps then the reverse segment is too small to do.
				// If numStepsUp < totalSteps then there has been a rounding error, because we are supposed to move up more than the calculated number of steps we move up.
				// This can happen if the calculated reversal is very close to the end of the move, because we round the final step positions to the nearest step, which may be up.
				// Either way, don't do a reverse segment.
				reverseStartStep = totalSteps + 1;
				mp.delta.reverseStartDistance = dda.totalDistance + 1.0;
			}
			else
			{
				reverseStartStep = numStepsUp + 1;

				// Correct the initial direction and the total number of steps
				if (direction)
				{
					// Net movement is up, so we will go up first and then down by a lesser amount
					totalSteps = (2 * numStepsUp) - totalSteps;
				}
				else
				{
					// Net movement is down, so we will go up first and then down by a greater amount
					direction = true;
					totalSteps = (2 * numStepsUp) + totalSteps;
				}
			}
		}
	}

	distanceSoFar = 0.0;
	timeSoFar = 0.0;
	isDelta = true;
	isExtruder = false;
	segments = dda.segments;
	nextStep = 1;									// must do this before calling NewDeltaSegment
	directionChanged = directionReversed = false;	// must clear these before we call NewDeltaSegment

	if (!NewDeltaSegment())
	{
		return false;
	}

	// Prepare for the first step
	nextStepTime = 0;
	stepsTakenThisSegment = 0;						// no steps taken yet since the start of the segment
	return CalcNextStepTimeFull();					// calculate the scheduled time of the first step
}

#endif	// SUPPORT_LINEAR_DELTA

// Prepare this DM for an extruder move, returning true if there are steps to do
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
// Return true if all OK, false to abort this move because the calculation has gone wrong
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
			MoveSegment *oldSegment = currentSegment;
			segments = currentSegment = oldSegment->GetNext();
			MoveSegment::Release(oldSegment);
			if (isExtruder)
			{
				{
					AtomicCriticalSectionLocker lock;										// avoid a race with GetNetStepsTaken called by filament monitor code
					nextStep = nextStep - 2 * (segmentStepLimit - reverseStartStep);		// set nextStep to the net steps taken in the original direction (this may make nextStep negative)
					CheckDirection(false);													// so that GetNetStepsTaken returns the correct value
				}
				const bool wasUsingPA = oldSegment->UsePressureAdvance();
				currentSegment = NewExtruderSegment();
				if (currentSegment != nullptr)
				{
					if (wasUsingPA)
					{
						const int32_t netStepsDone = nextStep - 1;
						const float stepsCarriedForward = (distanceSoFar - (float)netStepsDone * mp.cart.effectiveMmPerStep) * mp.cart.effectiveStepsPerMm;
						extruderShaper.SetExtrusionPending(stepsCarriedForward);
					}
					state = DMState::idle;
					return false;
				}
			}
			else
			{
				currentSegment =
#if SUPPORT_LINEAR_DELTA
								(isDelta) ? NewDeltaSegment() :
#endif
									NewCartesianSegment();
				if (currentSegment == nullptr)
				{
					state = DMState::idle;
					return false;
				}
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
			if (reverseStartStep < segmentStepLimit && nextStep < reverseStartStep)
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
		nextCalcStepTime = t0 + fastLimSqrtf(d0 + p * (float)(nextStep + (int32_t)stepsTillRecalc));
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
			const int32_t netSteps = (2 * (reverseStartStep - 1)) - nextStep;
			nextCalcStepTime = t0 + fastLimSqrtf(q + p * (float)(netSteps - (int32_t)stepsTillRecalc));
		}
		break;

	case DMState::cartDecelNoReverse:							// Cartesian decelerating with no reversal
		nextCalcStepTime = t0 - fastLimSqrtf(d0 + p * (float)(nextStep + (int32_t)stepsTillRecalc));
		break;

#if SUPPORT_LINEAR_DELTA
	case DMState::deltaForwardsReversing:						// moving forwards
		if (nextStep == reverseStartStep)
		{
			direction = false;
			directionChanged = directionReversed = true;
			state = DMState::deltaNormal;
		}
		// no break
	case DMState::deltaNormal:
		// Calculate d*s where d = distance the head has travelled, s = steps/mm for this drive
		{
			const float steps = (float)(1u << shiftFactor);
			if (direction)
			{
				mp.delta.fHmz0s += steps;						// get new carriage height above Z in steps
			}
			else
			{
				mp.delta.fHmz0s -= steps;						// get new carriage height above Z in steps
			}

			const float hmz0sc = mp.delta.fHmz0s * ((const DeltaMoveSegment*)segments)->GetDv()[2];
			const float t1 = ((const DeltaMoveSegment*)segments)->GetfMinusAaPlusBbTimesS() + hmz0sc;
			const float t2a = ((const DeltaMoveSegment*)segments)->GetfDSquaredMinusAsquaredMinusBsquaredTimesSsquared() - fsquare(mp.delta.fHmz0s) + fsquare(t1);
			// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
			const float t2 = fastLimSqrtf(t2a);
			const float ds = (direction) ? t1 - t2 : t1 + t2;

			// Now feed ds into the step algorithm for Cartesian motion
			if (ds < 0.0)
			{
				state = DMState::stepError2;
				return false;
			}

			const float pCds = p * ds;
			nextCalcStepTime = (segments->IsLinear()) ? t0 + pCds
								: (segments->IsAccelerating()) ? t0 + fastLimSqrtf(q + pCds)
									 : t0 - fastLimSqrtf(q + pCds);
		}
		break;
#endif

	default:
		return false;
	}

#if 0	//DEBUG
	if (std::isnan(nextCalcStepTime) || nextCalcStepTime < 0.0)
	{
		state = DMState::stepError;
		nextStep += 140000000 + stepsTillRecalc;			// so we can tell what happened in the debug print
		distanceSoFar = nextCalcStepTime;					//DEBUG
		return false;
	}
#endif

	uint32_t iNextCalcStepTime = (uint32_t)nextCalcStepTime;

	if (iNextCalcStepTime > segments->GetDuration())
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// Very rarely on a delta, the penultimate step may also be calculated late. Allow for that here in case it affects Cartesian axes too.
		// 2023-12-06: we now allow any step to be late but we record the maximum number
		// Don't use totalSteps here because it isn't valid for extruders
		if (segments->GetNext() == nullptr)
		{
			iNextCalcStepTime = segments->GetDuration();
			const int32_t nextCalcStep = nextStep + (int32_t)stepsTillRecalc;
			const int32_t stepsLate = segmentStepLimit - nextCalcStep;
			if (stepsLate > maxStepsLate) { maxStepsLate = stepsLate; }
		}
		else
		{
			// We don't expect any segment except the last to have late steps
			state = DMState::stepError3;
			stepInterval = iNextCalcStepTime;				//DEBUG
			return false;
		}
	}

	// When crossing between movement phases with high microstepping, due to rounding errors the next step may appear to be due before the last one
	stepInterval = (iNextCalcStepTime > nextStepTime)
					? (iNextCalcStepTime - nextStepTime) >> shiftFactor	// calculate the time per step, ready for next time
					: 0;

#if 0	//DEBUG
	if (isExtruder && stepInterval < 20 /*&& nextStep + stepsTillRecalc + 1 < totalSteps*/)
	{
		state = DMState::stepError;
		nextStep += 130000000 + stepsTillRecalc;			// so we can tell what happened in the debug print
		return false;
	}
#endif

	nextStepTime = iNextCalcStepTime - (stepsTillRecalc * stepInterval) + segments->GetStartTime();
	return true;
}

// End
