/*
 * DriveMovement.cpp
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#include "DriveMovement.h"
#include "DDA.h"
#include "Move.h"
#include "StepTimer.h"
#include <Platform/RepRap.h>
#include <Math/Isqrt.h>
#include "Kinematics/LinearDeltaKinematics.h"

// Static members

DriveMovement *DriveMovement::freeList = nullptr;
unsigned int DriveMovement::numCreated = 0;

void DriveMovement::InitialAllocate(unsigned int num) noexcept
{
	while (num > numCreated)
	{
		freeList = new DriveMovement(freeList);
		++numCreated;
	}
}

// Allocate a DM, from the freelist if possible, else create a new one
DriveMovement *DriveMovement::Allocate(size_t p_drive, DMState st) noexcept
{
	DriveMovement * dm = freeList;
	if (dm != nullptr)
	{
		freeList = dm->nextDM;
		dm->nextDM = nullptr;
	}
	else
	{
		dm = new DriveMovement(nullptr);
		++numCreated;
	}
	dm->drive = (uint8_t)p_drive;
	dm->state = st;
	return dm;
}

// Constructors
DriveMovement::DriveMovement(DriveMovement *next) noexcept : nextDM(next)
{
}

// Non static members

// This is called when currentSegment has just been changed to a new segment. Return true if there is a new segment to execute.
bool DriveMovement::NewCartesianSegment() noexcept
{
	while(true)
	{
		if (currentSegment == nullptr)
		{
			return false;
		}

		const float startDistance = distanceSoFar;
		const float startTime = timeSoFar;

		// Work out the movement limit in steps
		distanceSoFar += currentSegment->GetSegmentLength();
		timeSoFar += currentSegment->GetSegmentTime();

		pC = currentSegment->CalcC(mp.cart.effectiveMmPerStep);
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * stepNumber
			pB = currentSegment->CalcLinearB(startDistance, startTime);
			state = DMState::cartLinear;
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * stepNumber)
			pA = currentSegment->CalcNonlinearA(startDistance);
			pB = currentSegment->CalcNonlinearB(startTime);
			state = DMState::cartAccelOrDecelNoReverse;
		}

		phaseStepLimit = (uint32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm);
		if (nextStep < phaseStepLimit)
		{
			return true;
		}

		currentSegment = currentSegment->GetNext();						// skip this segment
	}
}

// This is called when currentSegment has just been changed to a new segment. Return true if there is a new segment to execute.
bool DriveMovement::NewExtruderSegment() noexcept
{
	while(true)
	{
		if (currentSegment == nullptr)
		{
			return false;
		}

		const float startDistance = distanceSoFar;
		const float startTime = timeSoFar;

		// Work out the movement limit in steps
		distanceSoFar += currentSegment->GetSegmentLength();
		timeSoFar += currentSegment->GetSegmentTime();

		pC = currentSegment->CalcC(mp.cart.effectiveMmPerStep);
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * stepNumber
			pB = currentSegment->CalcLinearB(startDistance, startTime);
			phaseStepLimit = (uint32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm);
			state = DMState::cartLinear;
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * stepNumber)
			pA = currentSegment->CalcNonlinearA(startDistance);
			pB = currentSegment->CalcNonlinearB(startTime, mp.cart.pressureAdvanceK);
			if (currentSegment->IsAccelerating())
			{
				// Extruders have a single acceleration segment. We need to add the extra extrusion distance due to pressure advance to the extrusion distance.
				distanceSoFar += mp.cart.extraExtrusionDistance;
				phaseStepLimit = (uint32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm);
				state = DMState::cartAccelOrDecelNoReverse;
			}
			else
			{
				// This is a decelerating segment. If it includes pressure advance then it may include reversal.
				phaseStepLimit = totalSteps;					// there is only one decelerating segment for extruders and it is at the end
				state = DMState::cartDecelForwardsReversing;	// assume tnat it may reverse
			}
		}

		if (nextStep < phaseStepLimit)
		{
			return true;
		}

		currentSegment = currentSegment->GetNext();						// skip this segment
	}
}

// This is called when currentSegment has just been changed to a new segment. Return true if there is a new segment to execute.
bool DriveMovement::NewDeltaSegment(const DDA& dda) noexcept
{
	while(true)
	{
		if (currentSegment == nullptr)
		{
			return false;
		}

		pC = currentSegment->CalcCDelta();
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * distanceMoved
			pB = currentSegment->CalcLinearB(distanceSoFar, timeSoFar);
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * distanceMoved)
			pA = currentSegment->CalcNonlinearA(distanceSoFar);
			pB = currentSegment->CalcNonlinearB(timeSoFar, 0.0);
		}

		distanceSoFar += currentSegment->GetSegmentLength();
		timeSoFar += currentSegment->GetSegmentTime();

		// Work out whether we reverse in this segment and the movement limit in steps
		const float sDx = distanceSoFar * dda.directionVector[0];
		const float sDy = distanceSoFar * dda.directionVector[1];
		const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
		const float netStepsAtEnd = fastSqrtf(mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(stepsPerMm) * (sDx * (sDx + mp.delta.fTwoA) + sDy * (sDy + mp.delta.fTwoB)))
								 + (distanceSoFar * dda.directionVector[2] - mp.delta.h0MinusZ0) * stepsPerMm;

		if (distanceSoFar <= mp.delta.reverseStartDistance)
		{
			// This segment is purely upwards motion of the tower
			state = DMState::deltaForwardsNoReverse;
			phaseStepLimit = (uint32_t)netStepsAtEnd;
		}
		else
		{
			// This segment ends with reverse motion
			phaseStepLimit = 2 * reverseStartStep - (uint32_t)netStepsAtEnd;
			state = DMState::deltaForwardsReversing;
		}

		if (phaseStepLimit > nextStep)
		{
			return true;
		}

		currentSegment = currentSegment->GetNext();
	}
}

// Prepare this DM for a Cartesian axis move, returning true if there are steps to do
bool DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept
{
	distanceSoFar = 0.0;
	timeSoFar = 0.0;
	mp.cart.pressureAdvanceK = 0.0;
	mp.cart.effectiveStepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive) * dda.directionVector[drive];
	mp.cart.effectiveMmPerStep = 1.0/mp.cart.effectiveStepsPerMm;
	isDelta = false;
	isExtruder = false;
	currentSegment = dda.axisSegments;

	if (!NewCartesianSegment())
	{
		return false;
	}

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	reverseStartStep = totalSteps + 1;				// no reverse phase
	return CalcNextStepTime(dda);
}

// Prepare this DM for a Delta axis move, returning true if there are steps to do
bool DriveMovement::PrepareDeltaAxis(const DDA& dda, const PrepParams& params) noexcept
{
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float A = params.initialX - params.dparams->GetTowerX(drive);
	const float B = params.initialY - params.dparams->GetTowerY(drive);
	const float aAplusbB = A * dda.directionVector[X_AXIS] + B * dda.directionVector[Y_AXIS];
	const float dSquaredMinusAsquaredMinusBsquared = params.dparams->GetDiagonalSquared(drive) - fsquare(A) - fsquare(B);
	mp.delta.h0MinusZ0 = fastSqrtf(dSquaredMinusAsquaredMinusBsquared);
#if DM_USE_FPU
	mp.delta.fTwoA = 2.0 * A;
	mp.delta.fTwoB = 2.0 * B;
	mp.delta.fHmz0s = mp.delta.h0MinusZ0 * stepsPerMm;
	mp.delta.fMinusAaPlusBbTimesS = -(aAplusbB * stepsPerMm);
	mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared = dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm);
#else
	qq;	// incomplete!
	mp.delta.hmz0sK = roundS32(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
	mp.delta.minusAaPlusBbTimesKs = -roundS32(aAplusbB * stepsPerMm * DriveMovement::K2);
	mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared = roundS64(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));
#endif

	// Calculate the distance at which we need to reverse direction.
	if (params.a2plusb2 <= 0.0)
	{
		// Pure Z movement. We can't use the main calculation because it divides by a2plusb2.
		direction = (dda.directionVector[Z_AXIS] >= 0.0);
		mp.delta.reverseStartDistance = dda.totalDistance + 1.0;	// so that we never reverse
		reverseStartStep = totalSteps + 1;
	}
	else
	{
		// The distance to reversal is the solution to a quadratic equation. One root corresponds to the carriages being below the bed,
		// the other root corresponds to the carriages being above the bed.
		const float drev = ((dda.directionVector[Z_AXIS] * fastSqrtf(params.a2plusb2 * params.dparams->GetDiagonalSquared(drive) - fsquare(A * dda.directionVector[Y_AXIS] - B * dda.directionVector[X_AXIS])))
							- aAplusbB)/params.a2plusb2;
		mp.delta.reverseStartDistance = drev;
		if (drev > 0.0 && drev < dda.totalDistance)					// if the reversal point is within range
		{
			// Calculate how many steps we need to move up before reversing
			const float hrev = dda.directionVector[Z_AXIS] * drev + fastSqrtf(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - params.a2plusb2 * fsquare(drev));
			const int32_t numStepsUp = (int32_t)((hrev - mp.delta.h0MinusZ0) * stepsPerMm);

			// We may be almost at the peak height already, in which case we don't really have a reversal.
			if (numStepsUp < 1 || (direction && (uint32_t)numStepsUp <= totalSteps))
			{
				reverseStartStep = totalSteps + 1;
				direction = false;
			}
			else
			{
				reverseStartStep = (uint32_t)numStepsUp + 1;

				// Correct the initial direction and the total number of steps
				if (direction)
				{
					// Net movement is up, so we will go up a bit and then down by a lesser amount
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
		else
		{
			// No reversal
			reverseStartStep = totalSteps + 1;
			direction = (drev <= 0.0);
		}
	}

	distanceSoFar = 0.0;
	timeSoFar = 0.0;
	isDelta = true;
	currentSegment = dda.axisSegments;
	mp.cart.extraExtrusionDistance = 0.0;

	if (!NewDeltaSegment(dda))
	{
		return false;
	}

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	return CalcNextStepTime(dda);
}

// Prepare this DM for an extruder move, returning true if there are steps to do
bool DriveMovement::PrepareExtruder(const DDA& dda) noexcept
{
	const ExtruderShaper& shaper = reprap.GetMove().GetExtruderShaper(LogicalDriveToExtruder(drive));
	distanceSoFar = shaper.GetExtrusionPending();
	timeSoFar = 0.0;

	if (dda.flags.usePressureAdvance)
	{
		mp.cart.pressureAdvanceK = shaper.GetK();
		mp.cart.extraExtrusionDistance = mp.cart.pressureAdvanceK * dda.acceleration;	//TODO use the last speed at which we did any real extrusion instead
	}
	else
	{
		mp.cart.pressureAdvanceK = mp.cart.extraExtrusionDistance = 0.0;
	}

	mp.cart.effectiveStepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive) * dda.directionVector[drive];
	mp.cart.effectiveMmPerStep = 1.0/mp.cart.effectiveStepsPerMm;
	isDelta = false;
	isExtruder = true;
	currentSegment = dda.extruderSegments;

	// Determine whether there is a reverse phase during deceleration and set up reverseStartStep and totalSteps
	qq;

	if (!NewExtruderSegment())
	{
		return false;
	}

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	reverseStartStep = totalSteps + 1;				// no reverse phase
	return CalcNextStepTime(dda);
}

void DriveMovement::DebugPrint() const noexcept
{
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32 " A=%g B=%g C=%" PRIu32 "\n",
						c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval, (double)pA, (double)pB, (uint32_t)pC);
		if (isDelta)
		{
			debugPrintf("hmz0s=%.2f minusAaPlusBbTimesS=%.2f dSquaredMinusAsquaredMinusBsquared=%.2f\n",
							(double)mp.delta.fHmz0s, (double)mp.delta.fMinusAaPlusBbTimesS, (double)mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared);
		}
		else
		{
			debugPrintf("pa=%.2f\n", (double)mp.cart.pressureAdvanceK);
		}
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// We have already incremented nextStep and checked that it does not exceed totalSteps, so at least one more step is due
// Return true if all OK, false to abort this move because the calculation has gone wrong
bool DriveMovement::CalcNextStepTimeFull(const DDA &dda) noexcept
pre(nextStep <= totalSteps; stepsTillRecalc == 0)
{
	uint32_t stepsToLimit;

	// If there are no more steps left in this segment, skip to the next segment
	if ((stepsToLimit = phaseStepLimit - nextStep) == 0)
	{
		currentSegment = currentSegment->GetNext();
		const bool more = (isDelta) ? NewDeltaSegment(dda)
							: (isExtruder) ? NewExtruderSegment()
								: NewCartesianSegment();
		if (!more)
		{
			state = DMState::stepError;
			stepInterval = 20000000 + nextStepTime;		// so we can tell what happened in the debug print
			return false;
		}
	}

	uint32_t shiftFactor = 0;					// assume single stepping
	uint32_t nextCalcStepTime;

	// Work out the time of the step
	switch (state)
	{
	case DMState::cartLinear:					// linear steady speed
		// Work out how many steps to calculate at a time.
		if (stepsToLimit > 1 && stepInterval < DDA::MinCalcIntervalCartesian)
		{
			if (stepInterval < DDA::MinCalcIntervalCartesian/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;				// octal stepping
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;				// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;				// double stepping
			}
		}

		stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate
		nextCalcStepTime = pB + (float)(nextStep + stepsTillRecalc) * pC;
		break;

	case DMState::cartDecelForwardsReversing:
		if (reverseStartStep <= totalSteps)
		{
			if (nextStep == reverseStartStep)
			{
				direction = false;
				directionChanged = true;
				state = DMState::deltaReverse;
			}
			else
			{
				stepsToLimit = reverseStartStep - nextStep;
			}
		}
		// no break
	case DMState::cartAccelOrDecelNoReverse:	// Cartesian accelerating, or decelerating with no reversal
	case DMState::cartDecelReverse:				// Cartesian decelerating, reverse motion
		// Work out how many steps to calculate at a time.
		if (stepsToLimit > 1 && stepInterval < DDA::MinCalcIntervalCartesian)
		{
			if (stepInterval < DDA::MinCalcIntervalCartesian/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;				// octal stepping
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;				// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;				// double stepping
			}
		}

		stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate
		nextCalcStepTime = pB - fastSqrtf(pA + pC * (float)(nextStep + stepsTillRecalc));
		break;

	case DMState::deltaForwardsReversing:			// moving forwards
		if (reverseStartStep <= totalSteps)
		{
			if (nextStep == reverseStartStep)
			{
				direction = false;
				directionChanged = true;
				state = DMState::deltaReverse;
			}
			else
			{
				stepsToLimit = reverseStartStep - nextStep;
			}
		}
		// no break
	case DMState::deltaForwardsNoReverse:
	case DMState::deltaReverse:			// reversing on this and subsequent steps
		// Work out how many steps to calculate at a time.
		if (stepInterval < DDA::MinCalcIntervalDelta)
		{
			if (stepInterval < DDA::MinCalcIntervalDelta/8 && stepsToLimit > 16)
			{
				shiftFactor = 4;		// hexadecimal stepping
			}
			else if (stepInterval < DDA::MinCalcIntervalDelta/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;		// octal stepping
			}
			else if (stepInterval < DDA::MinCalcIntervalDelta/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;		// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;		// double stepping
			}
		}

		stepsTillRecalc = (1u << shiftFactor) - 1;						// store number of additional steps to generate

		// Calculate d*s where d = distance the head has travelled, s = steps/mm for this drive
		{
			float steps = float(1u << shiftFactor);
			if (!direction)
			{
				steps = -steps;
			}
			mp.delta.fHmz0s += steps;									// get new carriage height above Z in steps

			const float hmz0sc = mp.delta.fHmz0s * dda.directionVector[Z_AXIS];
			const float t1 = mp.delta.fMinusAaPlusBbTimesS + hmz0sc;
			const float t2a = mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(mp.delta.fHmz0s) + fsquare(t1);
			// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
			const float t2 = (t2a > 0.0) ? fastSqrtf(t2a) : 0.0;
			const float ds = (direction) ? t1 - t2 : t1 + t2;

			// Now feed ds into the step algorithm for Cartesian motion without elasticity compensation
			if (ds < 0.0)
			{
				state = DMState::stepError;
				nextStep += 1000000;									// so that we can tell what happened in the debug print
				return false;
			}

			const float pCds = pC * ds;
			nextCalcStepTime = pB + ((currentSegment->IsLinear()) ? pCds : fastSqrtf(pA + pCds));
		}
		break;

	default:
		return false;
	}

	// When crossing between movement phases with high microstepping, due to rounding errors the next step may appear to be due before the last one
	stepInterval = (nextCalcStepTime > nextStepTime)
					? (nextCalcStepTime - nextStepTime) >> shiftFactor	// calculate the time per step, ready for next time
					: 0;
#if EVEN_STEPS
	nextStepTime = nextCalcStepTime - (stepsTillRecalc * stepInterval);
#else
	nextStepTime = nextCalcStepTime;
#endif

	if (nextCalcStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// Very rarely on a delta, the penultimate step may also be calculated late. Allow for that here in case it affects Cartesian axes too.
		if (nextStep + 1 >= totalSteps)
		{
			nextStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any step except the last to be late
			state = DMState::stepError;
			stepInterval = 10000000 + nextStepTime;				// so we can tell what happened in the debug print
			return false;
		}
	}

	return true;
}

// End
