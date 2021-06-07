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

// This is called when currentSegment has just been changed to a new segment
void DriveMovement::NewCartesianSegment(float startFraction, float startTime) noexcept
{
	if (currentSegment->IsLinear())
	{
		// Set up pB, pC such that time = pB * stepNumber + pC
		// time = startTime + (moveFraction - startFraction) * currentSegment->GetUDivD
		pA = 0.0;									// for debug
		pB = currentSegment->GetDDivU()/(float)totalSteps;
		pC = startTime - (startFraction * currentSegment->GetDDivU());
		state = DMState::cartLinear;
	}
	else
	{
		// Set up pA, pB, pC such that for forward motion, time = sqrt(pA + pb * stepNumber) + pC
		// time = startTime - currentSegment->GetUDivA() + sqrt(currentSegment->GetUDivA()^2 + currentSegment->GetTwoDDivA() * (moveFraction - startFraction))
		//qq;	// TODO pressure advance!
		pB = currentSegment->GetTwoDDivA()/(float)totalSteps;
		pA = fsquare(currentSegment->GetUDivA()) - (currentSegment->GetTwoDDivA() * startFraction);
		pC = startTime - currentSegment->GetUDivA();
		state = (currentSegment->GetTwoDDivA() > 0.0) ? DMState::cartAccel : DMState::cartDecelNoReverse;
	}

	// Work out the movement limit in steps
	phaseStepLimit = (currentSegment->IsLast()) ? totalSteps + 1 : (currentSegment->GetDistanceLimit() * totalSteps) + 1;
}

void DriveMovement::NewExtruderSegment(const DDA& dda, float startFraction, float startTime) noexcept
{
	NewCartesianSegment(startFraction,  startTime);		//TODO TEMP! need to add pressure advance
}

void DriveMovement::NewDeltaSegment(const DDA& dda, float startFraction, float startTime) noexcept
{
	if (currentSegment->IsLinear())
	{
		// Set up pB, pC such that time = pB * moveFraction + pC
		// time = startTime + (moveFraction - startFraction) * currentSegment->GetUDivD
		pB = currentSegment->GetDDivU();
		pC = startTime - startFraction * currentSegment->GetDDivU();
	}
	else
	{
		// Set up pA, pB, pC such that for forward motion, time = sqrt(pA + pb * moveFraction) + pC
		// time = startTime - currentSegment->GetUDivA() + sqrt(currentSegment->GetUDivA()^2 + currentSegment->GetTwoDDivA() * (moveFraction - startFraction))
		pA = fsquare(currentSegment->GetUDivA()) - currentSegment->GetTwoDDivA() * startFraction;
		pB = currentSegment->GetTwoDDivA();
		pC = startTime - currentSegment->GetUDivA();
	}
	//TODO need to set a linear/not linear flag

	// Work out the movement limit in steps
	const float distanceLimit = currentSegment->GetDistanceLimit();
	const float sDx = distanceLimit * dda.directionVector[0];
	const float sDy = distanceLimit * dda.directionVector[1];
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float netStepsAtEnd = fastSqrtf(mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(stepsPerMm) * (sDx * (sDx + mp.delta.fTwoA) + sDy * (sDy + mp.delta.fTwoB)))
							 + (distanceLimit * dda.directionVector[2] -mp. delta.h0MinusZ0) * stepsPerMm;
	if (direction)
	{
		// Going up
		state = DMState::deltaForwards;
		if ((uint32_t)netStepsAtEnd > reverseStartStep)
		{
			// We reverse direction during this segment
			//reverseInThisSegment = true;
			phaseStepLimit = reverseStartStep;
		}
		else if (currentSegment->IsLast())
		{
			phaseStepLimit = totalSteps + 1;
		}
		else
		{
			phaseStepLimit = (uint32_t)netStepsAtEnd + 1;
		}
	}
	else
	{
		// Going down
		state = DMState::deltaReverse;
		if (currentSegment->IsLast())
		{
			phaseStepLimit = totalSteps + 1;
		}
		else
		{
			phaseStepLimit = (uint32_t)(netStepsAtEnd);
		}
	}
}

// Prepare this DM for a Cartesian axis move, returning true if there are steps to do
bool DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept
{
	usesCommonSegments = true;
	isDelta = false;
	mp.cart.pressureAdvance = 0.0;

	currentSegment = dda.segments;
	phaseStartTime = 0.0;
	do
	{
		NewCartesianSegment(0.0, 0.0);
	} while (phaseStepLimit == 0);

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
		reverseStartStep = totalSteps + 1;
		reverseStartDistance = 2 * dda.totalDistance;
	}
	else
	{
		// The distance to reversal is the solution to a quadratic equation. One root corresponds to the carriages being below the bed,
		// the other root corresponds to the carriages being above the bed.
		const float drev = ((dda.directionVector[Z_AXIS] * fastSqrtf(params.a2plusb2 * params.dparams->GetDiagonalSquared(drive) - fsquare(A * dda.directionVector[Y_AXIS] - B * dda.directionVector[X_AXIS])))
							- aAplusbB)/params.a2plusb2;
		if (drev > 0.0 && drev < dda.totalDistance)		// if the reversal point is within range
		{
			// Calculate how many steps we need to move up before reversing
			reverseStartDistance = drev;
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
			reverseStartDistance = 2 * dda.totalDistance;
			direction = (drev <= 0.0);
		}
	}

	currentSegment = dda.segments;
	phaseStartTime = 0.0;
	NewCartesianSegment(0.0, 0.0);

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	isDelta = true;
	usesCommonSegments = true;
	return CalcNextStepTime(dda);
}

// Prepare this DM for an extruder move, returning true if there are steps to do
bool DriveMovement::PrepareExtruder(const DDA& dda, MoveSegment *segs) noexcept
{
	currentSegment = segs;
	isDelta = false;
	usesCommonSegments = false;
	phaseStartTime = 0.0;
	NewExtruderSegment(dda, 0.0, 0.0);

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
			debugPrintf("pa=%.2f\n", (double)mp.cart.pressureAdvance);
		}
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// We have already incremented nextStep
// Return true if all OK, false to abort this move because the calculation has gone wrong
// This is also used for extruders on delta machines.
bool DriveMovement::CalcNextStepTimeFull(const DDA &dda) noexcept
pre(nextStep < totalSteps; stepsTillRecalc == 0)
{
	// Work out how many steps to calculate at a time.
	uint32_t shiftFactor = 0;		// assume single stepping
	uint32_t nextCalcStepTime;
	uint32_t stepsToLimit = phaseStepLimit - nextStep;

	// Work out the time of the step
	switch (state)
	{
	case DMState::cartDecelReversing:				// Cartesian decelerating, reverse on next step
		// no break
		direction = !direction;
		directionChanged = true;
		phaseStepLimit = nextStep + 1;	//TODO!!!
		state = DMState::cartDecelReverse;
		// no break
	case DMState::cartDecelReverse:			// Cartesian decelerating, reverse motion
	case DMState::cartDecelNoReverse:		// Cartesian decelerating
		if (stepsToLimit > 1 && stepInterval < DDA::MinCalcIntervalCartesian)
		{
			if (stepInterval < DDA::MinCalcIntervalCartesian/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;		// octal stepping
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;		// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;		// double stepping
			}
		}

		stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate
		nextCalcStepTime = pC - fastSqrtf(pA + pB * (float)(nextStep + stepsTillRecalc));
		break;

	case DMState::cartAccel:			// Cartesian accelerating
		if (stepsToLimit > 1 && stepInterval < DDA::MinCalcIntervalCartesian)
		{
			if (stepInterval < DDA::MinCalcIntervalCartesian/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;		// octal stepping
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;		// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;		// double stepping
			}
		}

		stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate
		nextCalcStepTime = pC + fastSqrtf(pA + pB * (float)(nextStep + stepsTillRecalc));
		break;

	case DMState::cartLinear:				// linear steady speed
		if (stepsToLimit > 1 && stepInterval < DDA::MinCalcIntervalCartesian)
		{
			if (stepInterval < DDA::MinCalcIntervalCartesian/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;		// octal stepping
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;		// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;		// double stepping
			}
		}

		stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate
		nextCalcStepTime = (float)(nextStep + stepsTillRecalc) * pB + pC;
		break;

	case DMState::cartDecelExpectReverse:	// linear decelerating, reverse expected in this segment
		if (stepsToLimit > 1 && stepInterval < DDA::MinCalcIntervalCartesian)
		{
			if (stepInterval < DDA::MinCalcIntervalCartesian/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;		// octal stepping
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;		// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;		// double stepping
			}
		}

		stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate
		nextCalcStepTime = fastSqrtf(pA + pB * (float)(nextStep + stepsTillRecalc)) + pC;
		if (stepsToLimit == 1)
		{
			// This is the last step in this phase
			state = DMState::cartDecelReversing;
			stepsToLimit = 2;	//TODO TEMP stop us starting a new segment
		}
		break;

	case DMState::deltaForwards:		// moving forwards
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
	case DMState::deltaReverse:			// reversing on this and subsequent steps
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

		stepsTillRecalc = (1u << shiftFactor) - 1;					// store number of additional steps to generate

		// Calculate d*s*K as an integer, where d = distance the head has travelled, s = steps/mm for this drive, K = a power of 2 to reduce the rounding errors
		// K here means K2
		// mp.delta.hmz0sk = (number of steps by which the carriage is higher than Z) * K2
		{
			float steps = float(1u << shiftFactor);
			if (!direction)
			{
				steps = -steps;
			}
			mp.delta.fHmz0s += steps;									// get new carriage height above Z in steps
		}

		{
			const float hmz0sc = mp.delta.fHmz0s * dda.directionVector[Z_AXIS];
			const float t1 = mp.delta.fMinusAaPlusBbTimesS + hmz0sc;
			// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
			const float t2a = mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(mp.delta.fHmz0s) + fsquare(t1);
			const float t2 = (t2a > 0.0) ? fastSqrtf(t2a) : 0.0;
			const float ds = (direction) ? t1 - t2 : t1 + t2;

			// Now feed dsK into a modified version of the step algorithm for Cartesian motion without elasticity compensation
			if (ds < 0.0)
			{
				state = DMState::stepError;
				nextStep += 1000000;									// so that we can tell what happened in the debug print
				return false;
			}

			switch (state)
			{
			case DMState::deltaForwards:
//				nextCalcStepTime = currentSegment->CalcForwardStepTime(ds);
				break;

			case DMState::deltaReverse:
//				nextCalcStepTime = currentSegment->CalcReverseStepTime(ds);
				break;

			default:
				return false;
			}
		}
//		break;
		return false;	//TODO TEMP!!!

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

	// If there are no more steps left in this segment, skip to the next segment
	// A shared segment may have no more steps for a particular drive, so we may need to skip more segments
	while (stepsToLimit == 0)
	{
		const float startDistance = currentSegment->GetDistanceLimit();
		phaseStartTime += currentSegment->GetSegmentTime();
		MoveSegment * const oldCurrentSegment = currentSegment;
		currentSegment = currentSegment->GetNext();
		if (!usesCommonSegments)
		{
			MoveSegment::Release(oldCurrentSegment);
		}

		if (currentSegment == nullptr)
		{
			state = DMState::stepError;
			stepInterval = 20000000 + nextStepTime;				// so we can tell what happened in the debug print
			return false;
		}

		if (isDelta)
		{
			NewDeltaSegment(dda, startDistance, phaseStartTime);
		}
		else
		{
			NewCartesianSegment(startDistance, phaseStartTime);
		}
		stepsToLimit = phaseStepLimit - nextStep;

	}
	return true;
}

// End
