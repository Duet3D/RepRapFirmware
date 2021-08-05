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

#if !MS_USE_FPU

// The code in this file relies on right shift of a signed operand being arithmetic shift
// Shifting generates fewer instructions than dividing by a constant power of 2 in ARM gcc even though gcc converts the division to other instructions.
// Arithmetic shift rounds towards minus infinity, so it doesn't give quite the same result as dividing when there is a remainder.
static_assert(((int32_t)-21 >> 1) == (int32_t)-11);
static_assert(((int64_t)-10000000001 >> 1) == (int64_t)-5000000001);

#endif

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

void DriveMovement::DebugPrint() const noexcept
{
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
#if MS_USE_FPU
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32 " psl=%" PRIu32 " A=%.4e B=%.4e C=%.4e dsf=%.4e tsf=%.1f",
						c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval, phaseStepLimit,
							(double)pA, (double)pB, (double)pC, (double)distanceSoFar, (double)timeSoFar);
		if (isDelta)
		{
			debugPrintf(" hmz0s=%.4e minusAaPlusBbTimesS=%.4e dSquaredMinusAsquaredMinusBsquared=%.4e drev=%.4e\n",
							(double)mp.delta.fHmz0s, (double)mp.delta.fMinusAaPlusBbTimesS, (double)mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared, (double)mp.delta.reverseStartDistance);
		}
		else if (isExtruder)
		{
			debugPrintf(" pa=%" PRIu32 " eed=%.4e ebf=%.4e\n", (uint32_t)mp.cart.pressureAdvanceK, (double)mp.cart.extraExtrusionDistance, (double)mp.cart.extrusionBroughtForwards);
		}
		else
		{
			debugPrintf("\n");
		}
#else
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32 " psl=%" PRIu32 " A=%" PRIi64 " B=%" PRIi32 " C=%" PRIi32 " dsf=%" PRIu32 " tsf=%" PRIu32,
						c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval, phaseStepLimit,
							iA, iB, iC, iDistanceSoFar, iTimeSoFar);
		if (isDelta)
		{
			debugPrintf(" hmz0sk=%" PRIi32 " minusAaPlusBbTimesS=%" PRIi32 " dSquaredMinusAsquaredMinusBsquared=%" PRIi64 " drev=%" PRIu32 "\n",
							mp.delta.hmz0sK, mp.delta.minusAaPlusBbTimesKs, mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared, mp.delta.iReverseStartDistance);
		}
		else if (isExtruder)
		{
			debugPrintf(" pa=%" PRIu32 " eed=%" PRIu32 " ebf=%.4e\n", mp.cart.iPressureAdvanceK, mp.cart.iExtraExtrusionDistance, mp.cart.extrusionBroughtForwards);
		}
		else
		{
			debugPrintf("\n");
		}
#endif
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// This is called when currentSegment has just been changed to a new segment. Return true if there is a new segment to execute.
bool DriveMovement::NewCartesianSegment() noexcept
{
	while (true)
	{
		if (currentSegment == nullptr)
		{
			return false;
		}

		// Work out the movement limit in steps
#if MS_USE_FPU
		pC = currentSegment->CalcC(mp.cart.effectiveMmPerStep);
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * stepNumber
			pB = currentSegment->CalcLinearB(distanceSoFar, timeSoFar);
			state = DMState::cartLinear;
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * stepNumber)
			pA = currentSegment->CalcNonlinearA(distanceSoFar);
			pB = currentSegment->CalcNonlinearB(timeSoFar);
			state = (currentSegment->IsAccelerating()) ? DMState::cartAccel : DMState::cartDecelNoReverse;
		}

		distanceSoFar += currentSegment->GetSegmentLength();
		timeSoFar += currentSegment->GetSegmentTime();

		phaseStepLimit = (currentSegment->GetNext() == nullptr) ? totalSteps + 1 : (uint32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm) + 1;
#else
		iC = currentSegment->CalcC(mp.cart.iEffectiveMmPerStepTimesK);
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * stepNumber
			iB = currentSegment->CalcLinearB(iDistanceSoFar, iTimeSoFar);
			state = DMState::cartLinear;
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * stepNumber)
			iA = currentSegment->CalcNonlinearA(iDistanceSoFar);
			iB = currentSegment->CalcNonlinearB(iTimeSoFar);
			state = (currentSegment->IsAccelerating()) ? DMState::cartAccel : DMState::cartDecelNoReverse;
		}

		iDistanceSoFar += currentSegment->GetSegmentLength();
		iTimeSoFar += currentSegment->GetSegmentTime();

		phaseStepLimit = (currentSegment->GetNext() == nullptr) ? totalSteps + 1 : (uint32_t)(((iDistanceSoFar * (uint64_t)mp.cart.iEffectiveStepsPerMmTimesK)) >> MoveSegment::SFstepsPerMm) + 1;
#endif

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
	while (true)
	{
		if (currentSegment == nullptr)
		{
			return false;
		}

		const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
		pC = currentSegment->GetC()/stepsPerMm;		//TODO store the reciprocal to avoid the division
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * (distanceMoved * steps/mm)
			pB = currentSegment->CalcLinearB(distanceSoFar, timeSoFar);
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * (distanceMoved * steps/mm))
			pA = currentSegment->CalcNonlinearA(distanceSoFar);
			pB = currentSegment->CalcNonlinearB(timeSoFar);
		}

		const float startDistance = distanceSoFar;
		distanceSoFar += currentSegment->GetSegmentLength();
		timeSoFar += currentSegment->GetSegmentTime();

		// Work out whether we reverse in this segment and the movement limit in steps
		const float sDx = distanceSoFar * dda.directionVector[0];
		const float sDy = distanceSoFar * dda.directionVector[1];
		const int32_t netStepsAtEnd = (int32_t)(fastSqrtf(mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(stepsPerMm) * (sDx * (sDx + mp.delta.fTwoA) + sDy * (sDy + mp.delta.fTwoB)))
								 	 	 	 	 + (distanceSoFar * dda.directionVector[2] - mp.delta.h0MinusZ0) * stepsPerMm);

		if (mp.delta.reverseStartDistance <= startDistance)
		{
			// This segment is purely downwards motion and we want the greater of the two quadratic solutions. There may have been upwards motion earlier in the move.
			if (direction)
			{
				direction = false;
				directionChanged = true;
			}
			state = DMState::deltaReverse;
			phaseStepLimit = (currentSegment->GetNext() == nullptr) ? totalSteps + 1
								: (reverseStartStep <= totalSteps) ? (uint32_t)((int32_t)(2 * reverseStartStep) - netStepsAtEnd)
									: 1 - netStepsAtEnd;
		}
		else if (distanceSoFar <= mp.delta.reverseStartDistance)
		{
			// This segment is purely upwards motion of the tower and we want the lower quadratic solution
			state = DMState::deltaForwardsNoReverse;
			phaseStepLimit = (currentSegment->GetNext() == nullptr) ? totalSteps + 1 : (uint32_t)(netStepsAtEnd + 1);
		}
		else
		{
			// This segment ends with reverse motion. We want the lower quadratic solution initially.
			phaseStepLimit = (currentSegment->GetNext() == nullptr) ? totalSteps + 1 : (uint32_t)((int32_t)(2 * reverseStartStep) - netStepsAtEnd);
			state = DMState::deltaForwardsReversing;
		}

		if (phaseStepLimit > nextStep)
		{
			return true;
		}

		currentSegment = currentSegment->GetNext();
	}
}

// This is called when currentSegment has just been changed to a new segment. Return true if there is a new segment to execute.
bool DriveMovement::NewExtruderSegment() noexcept
{
	while (true)
	{
		if (currentSegment == nullptr)
		{
			return false;
		}

#if MS_USE_FPU
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
			pA = currentSegment->CalcNonlinearA(startDistance, mp.cart.pressureAdvanceK);
			pB = currentSegment->CalcNonlinearB(startTime, mp.cart.pressureAdvanceK);
			if (currentSegment->IsAccelerating())
			{
				// Extruders have a single acceleration segment. We need to add the extra extrusion distance due to pressure advance to the extrusion distance.
				distanceSoFar += mp.cart.extraExtrusionDistance;
				state = DMState::cartAccel;
			}
			else
			{
				// This is the single decelerating segment. If it includes pressure advance then it may include reversal.
				state = DMState::cartDecelForwardsReversing;			// assume that it may reverse
			}
		}

		phaseStepLimit = ((currentSegment->GetNext() == nullptr) ? totalSteps : (uint32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm)) + 1;
#else
		const uint32_t startDistance = iDistanceSoFar;
		const uint32_t startTime = iTimeSoFar;

		// Work out the movement limit in steps
		iDistanceSoFar += currentSegment->GetSegmentLength();
		iTimeSoFar += currentSegment->GetSegmentTime();

		iC = currentSegment->CalcC(mp.cart.iEffectiveMmPerStepTimesK);
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * stepNumber
			iB = currentSegment->CalcLinearB(startDistance, startTime);
			state = DMState::cartLinear;
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * stepNumber)
			iA = currentSegment->CalcNonlinearA(startDistance, mp.cart.iPressureAdvanceK);
			iB = currentSegment->CalcNonlinearB(startTime, mp.cart.iPressureAdvanceK);
			if (currentSegment->IsAccelerating())
			{
				// Extruders have a single acceleration segment. We need to add the extra extrusion distance due to pressure advance to the extrusion distance.
				iDistanceSoFar += mp.cart.iExtraExtrusionDistance;
				state = DMState::cartAccel;
			}
			else
			{
				// This is the single decelerating segment. If it includes pressure advance then it may include reversal.
				state = DMState::cartDecelForwardsReversing;			// assume that it may reverse
			}
		}

		phaseStepLimit = ((currentSegment->GetNext() == nullptr) ? totalSteps : (uint32_t)((iDistanceSoFar * (uint64_t)mp.cart.iEffectiveStepsPerMmTimesK)) >> MoveSegment::SFstepsPerMm) + 1;
#endif

		if (nextStep < phaseStepLimit)
		{
			return true;
		}

		currentSegment = currentSegment->GetNext();						// skip this segment
	}
}

// Prepare this DM for a Cartesian axis move, returning true if there are steps to do
bool DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept
{
#if MS_USE_FPU
	distanceSoFar = 0.0;
	timeSoFar = 0.0;
	mp.cart.pressureAdvanceK = 0.0;
	// We can't use directionVector here because those values relate to Cartesian space, whereas we may be CoreXY etc.
	mp.cart.effectiveStepsPerMm = (float)totalSteps/dda.totalDistance;
	mp.cart.effectiveMmPerStep = 1.0/mp.cart.effectiveStepsPerMm;
#else
	iDistanceSoFar = 0;
	iTimeSoFar = 0;
	mp.cart.iPressureAdvanceK = 0;
	// We can't use directionVector here because those values relate to Cartesian space, whereas we may be CoreXY etc.
	const float stepsTimesK = (float)((uint64_t)totalSteps << MoveSegment::SFstepsPerMm);
	mp.cart.iEffectiveStepsPerMmTimesK = stepsTimesK/dda.totalDistance;
	mp.cart.iEffectiveMmPerStepTimesK = dda.totalDistance/stepsTimesK;
#endif
	isDelta = false;
	isExtruder = false;
	currentSegment = (dda.shapedSegments != nullptr) ? dda.shapedSegments : dda.unshapedSegments;
	nextStep = 0;									// must do this before calling NewCartesianSegment

	if (!NewCartesianSegment())
	{
		return false;
	}

	// Prepare for the first step
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
#if MS_USE_FPU
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
		if (drev > 0.0 && drev < dda.totalDistance)						// if the reversal point is within range
		{
			// Calculate how many steps we need to move up before reversing
			const float hrev = dda.directionVector[Z_AXIS] * drev + fastSqrtf(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - params.a2plusb2 * fsquare(drev));
			const int32_t numStepsUp = (int32_t)((hrev - mp.delta.h0MinusZ0) * stepsPerMm);

			// We may be almost at the peak height already, in which case we don't really have a reversal.
			if (numStepsUp < 1)
			{
				mp.delta.reverseStartDistance = -1.0;					// so that we know we have reversed already
				reverseStartStep = totalSteps + 1;
				direction = false;
			}
			else
			{
				reverseStartStep = (uint32_t)numStepsUp + 1;

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
		else
		{
			// No reversal
			reverseStartStep = totalSteps + 1;
			direction = (drev >= 0.0);
		}
	}

	distanceSoFar = 0.0;
	timeSoFar = 0.0;
	isDelta = true;
	currentSegment = (dda.shapedSegments != nullptr) ? dda.shapedSegments : dda.unshapedSegments;

	nextStep = 0;									// must do this before calling NewDeltaSegment
	if (!NewDeltaSegment(dda))
	{
		return false;
	}

	// Prepare for the first step
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	return CalcNextStepTime(dda);
}

// Prepare this DM for an extruder move, returning true if there are steps to do
// We have already generated the extruder segments and we know that there are some
bool DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params) noexcept
{
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float effStepsPerMm = stepsPerMm * fabsf(dda.directionVector[drive]);
	const float effMmPerStep = 1.0/stepsPerMm;

	ExtruderShaper& shaper = reprap.GetMove().GetExtruderShaper(LogicalDriveToExtruder(drive));
	float forwardDistance =	mp.cart.extrusionBroughtForwards = shaper.GetExtrusionPending()/dda.directionVector[drive];
	float reverseDistance;

#if MS_USE_FPU
	mp.cart.effectiveStepsPerMm = effStepsPerMm;
	mp.cart.effectiveMmPerStep = effMmPerStep;
	distanceSoFar = forwardDistance;
	timeSoFar = 0.0;

	// Calculate the total forward and reverse movement distances
	if (dda.flags.usePressureAdvance && shaper.GetKclocks() > 0.0)
	{
		// We are using nonzero pressure advance. Movement must be forwards.
		mp.cart.pressureAdvanceK = shaper.GetKclocks();
		mp.cart.extraExtrusionDistance = mp.cart.pressureAdvanceK * (dda.topSpeed - dda.startSpeed);
		forwardDistance += mp.cart.extraExtrusionDistance;

		// Check if there is a reversal in the deceleration segment
		// There is at most one deceleration segment in the unshaped segments
		const MoveSegment *decelSeg = dda.unshapedSegments;
		while (decelSeg != nullptr && (decelSeg->IsLinear() || decelSeg->IsAccelerating()))
		{
			decelSeg = decelSeg->GetNext();
		}

		if (decelSeg == nullptr)
		{
			forwardDistance += dda.totalDistance;			// no deceleration segment
			reverseDistance = 0.0;
		}
		else
		{
			const float initialDecelSpeed = dda.topSpeed - mp.cart.pressureAdvanceK * params.unshaped.deceleration;
			if (initialDecelSpeed <= 0.0)
			{
				// The entire deceleration segment is in reverse
				forwardDistance += params.unshaped.decelStartDistance;
				reverseDistance = ((0.5 * params.unshaped.deceleration * params.unshaped.decelClocks) - initialDecelSpeed) * params.unshaped.decelClocks;
			}
			else
			{
				const float timeToReverse = initialDecelSpeed * ((-0.5) * decelSeg->GetC());	// 'c' is -2/deceleration, so -0.5*c is 1/deceleration
				if (timeToReverse < params.unshaped.decelClocks)
				{
					// There is a reversal, although it could be tiny
					const float distanceToReverse = fsquare(initialDecelSpeed) * decelSeg->GetC() * (-0.25);	// because (v^2-u^2) = 2as, so if v=0 then s=-u^2/2a = u^2/2d = -0.25*u^2*c
					forwardDistance += params.unshaped.decelStartDistance + distanceToReverse;
					reverseDistance = 0.5 * params.unshaped.deceleration * fsquare(params.unshaped.decelClocks - timeToReverse);	// because s = 0.5*a*t^2
				}
				else
				{
					// No reversal
					forwardDistance += dda.totalDistance - (mp.cart.pressureAdvanceK * params.unshaped.deceleration * params.unshaped.decelClocks);
					reverseDistance = 0.0;
				}
			}
		}
	}
	else
	{
		// No pressure advance. Movement may be backwards but this still counts as forward distance in the calculations.
		mp.cart.pressureAdvanceK = mp.cart.extraExtrusionDistance = 0.0;
		forwardDistance += dda.totalDistance;
		reverseDistance = 0.0;
	}
#else
	mp.cart.iEffectiveStepsPerMmTimesK = lrintf(effStepsPerMm * (float)(1u << MoveSegment::SFstepsPerMm));
	mp.cart.iEffectiveMmPerStepTimesK = lrintf(effMmPerStep * (float)(1u << MoveSegment::SFstepsPerMm));
	iTimeSoFar = 0;

	// Calculate the total forward and reverse movement distances
	//TODO distances as integer?

	if (dda.flags.usePressureAdvance && shaper.GetKclocks() > 0.0)
	{
		// We are using nonzero pressure advance. Movement must be forwards.
		mp.cart.iPressureAdvanceK = shaper.GetKclocks();
		const float extraExtrusionDistance = (float)mp.cart.iPressureAdvanceK * (dda.topSpeed - dda.startSpeed);
		mp.cart.iExtraExtrusionDistance = lrintf(extraExtrusionDistance * (float)(1u << MoveSegment::SFdistance));
		forwardDistance += extraExtrusionDistance;

		// Check if there is a reversal in the deceleration segment
		// There is at most one deceleration segment in the unshaped segments
		const MoveSegment *decelSeg = dda.unshapedSegments;
		while (decelSeg != nullptr && (decelSeg->IsLinear() || decelSeg->IsAccelerating()))
		{
			decelSeg = decelSeg->GetNext();
		}

		if (decelSeg == nullptr)
		{
			forwardDistance += dda.totalDistance;			// no deceleration segment
			reverseDistance = 0.0;
		}
		else
		{
			const float initialDecelSpeed = dda.topSpeed - (float)mp.cart.iPressureAdvanceK * params.unshaped.deceleration;
			if (initialDecelSpeed <= 0.0)
			{
				// The entire deceleration segment is in reverse
				forwardDistance += params.unshaped.decelStartDistance;
				reverseDistance = ((0.5 * params.unshaped.deceleration * params.unshaped.decelClocks) - initialDecelSpeed) * params.unshaped.decelClocks;
			}
			else
			{
				const float timeToReverse = initialDecelSpeed * ((-0.5) * decelSeg->GetC());	// 'c' is -2/deceleration, so -0.5*c is 1/deceleration
				if (timeToReverse < params.unshaped.decelClocks)
				{
					// There is a reversal, although it could be tiny
					const float distanceToReverse = fsquare(initialDecelSpeed) * decelSeg->GetC() * (-0.25);	// because (v^2-u^2) = 2as, so if v=0 then s=-u^2/2a = u^2/2d = -0.25*u^2*c
					forwardDistance += params.unshaped.decelStartDistance + distanceToReverse;
					reverseDistance = 0.5 * params.unshaped.deceleration * fsquare(params.unshaped.decelClocks - timeToReverse);	// because s = 0.5*a*t^2
				}
				else
				{
					// No reversal
					forwardDistance += dda.totalDistance - ((float)mp.cart.iPressureAdvanceK * params.unshaped.deceleration * params.unshaped.decelClocks);
					reverseDistance = 0.0;
				}
			}
		}
	}
	else
	{
		// No pressure advance. Movement may be backwards but this still counts as forward distance in the calculations.
		mp.cart.iPressureAdvanceK = mp.cart.iExtraExtrusionDistance = 0;
		forwardDistance += dda.totalDistance;
		reverseDistance = 0.0;
	}
#endif

	// Check whether there are any steps at all
	const float forwardSteps = forwardDistance * effStepsPerMm;
	if (reverseDistance > 0.0)
	{
		const float netDistance = forwardDistance - reverseDistance;
		const int32_t iFwdSteps = (int32_t)forwardSteps;
		int32_t netSteps = (int32_t)(netDistance * effStepsPerMm);
		if (netSteps == 0 && iFwdSteps == 0)
		{
			// No movement at all
			shaper.SetExtrusionPending(netDistance * dda.directionVector[drive]);
			return false;
		}

		// Note, netSteps may be negative for e.g. a deceleration-only move
		if (netSteps == iFwdSteps)
		{
			// The reverse segment is very small, so ignore it
			totalSteps = (uint32_t)iFwdSteps;
			reverseStartStep = totalSteps + 1;
		}
		else
		{
			// We know that netSteps <= iFwdSteps
			reverseStartStep = iFwdSteps + 1;
			// Round up netSteps because we don't want to overshoot the reverse movement.
			const float extrusionPending = netDistance - (float)netSteps * effMmPerStep;
			if (extrusionPending > 0.05 * effMmPerStep)
			{
				++netSteps;
			}
			totalSteps = (uint32_t)((int32_t)(2 * reverseStartStep) - netSteps - 2);
		}
		shaper.SetExtrusionPending((netDistance - (float)netSteps * effMmPerStep) * dda.directionVector[drive]);
	}
	else
	{
		if (forwardSteps >= 1.0)
		{
			totalSteps = (uint32_t)forwardSteps;
			shaper.SetExtrusionPending((forwardDistance - (float)totalSteps * effMmPerStep) * dda.directionVector[drive]);
		}
		else
		{
			// No steps at all, or negative forward steps which I think should be impossible unless the steps/mm is changed
			totalSteps = 0;
			shaper.SetExtrusionPending(forwardDistance * dda.directionVector[drive]);
			return false;
		}
		reverseStartStep = totalSteps + 1;			// no reverse phase
	}

	currentSegment = dda.unshapedSegments;
	isDelta = false;
	isExtruder = true;

	nextStep = 0;									// must do this before calling NewExtruderSegment
	if (!NewExtruderSegment())
	{
		return false;								// this should not happen because we have already determined that there are steps to do
	}

	// Prepare for the first step
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	return CalcNextStepTime(dda);
}

#if MS_USE_FPU

// Version of fastSqrtf that allows for slightly negative operands cause dby rounding error
static inline float fastLimSqrtf(float f) noexcept
{
	return (f > 0.0) ? fastSqrtf(f) : 0.0;
}

#endif

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// We have already incremented nextStep and checked that it does not exceed totalSteps, so at least one more step is due
// Return true if all OK, false to abort this move because the calculation has gone wrong
bool DriveMovement::CalcNextStepTimeFull(const DDA &dda) noexcept
pre(nextStep <= totalSteps; stepsTillRecalc == 0)
{
	uint32_t shiftFactor = 0;									// assume single stepping

	{
		uint32_t stepsToLimit = phaseStepLimit - nextStep;
		// If there are no more steps left in this segment, skip to the next segment
		if (stepsToLimit == 0)
		{
			currentSegment = currentSegment->GetNext();
			const bool more = (isDelta) ? NewDeltaSegment(dda)
								: (isExtruder) ? NewExtruderSegment()
									: NewCartesianSegment();
			if (!more)
			{
				state = DMState::stepError;
				nextStep += 100000000;							// so we can tell what happened in the debug print
				return false;
			}
			// Leave shiftFactor set to 0 so that we compute a single step time, because the interval will have changed
		}
		else
		{
			if (reverseStartStep < phaseStepLimit && nextStep < reverseStartStep)
			{
				stepsToLimit = reverseStartStep - nextStep;
			}

			if (stepsToLimit > 1 && stepInterval < DDA::MinCalcInterval)
			{
				if (stepInterval < DDA::MinCalcInterval/4 && stepsToLimit > 8)
				{
					shiftFactor = 3;							// octal stepping
				}
				else if (stepInterval < DDA::MinCalcInterval/2 && stepsToLimit > 4)
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
		nextCalcStepTime = pB + (float)(nextStep + stepsTillRecalc) * pC;
		break;

	case DMState::cartAccel:									// Cartesian accelerating
		nextCalcStepTime = pB + fastLimSqrtf(pA + pC * (float)(nextStep + stepsTillRecalc));
		break;

	case DMState::cartDecelForwardsReversing:
		if (nextStep + stepsTillRecalc < reverseStartStep)
		{
			nextCalcStepTime = pB - fastLimSqrtf(pA + pC * (float)(nextStep + stepsTillRecalc));
			break;
		}

		direction = false;
		directionChanged = true;
		state = DMState::cartDecelReverse;
		// no break
	case DMState::cartDecelReverse:								// Cartesian decelerating, reverse motion. Convert the steps to int32_t because the net steps may be negative.
		nextCalcStepTime = pB + fastLimSqrtf(pA + pC * (float)((2 * (int32_t)(reverseStartStep - 1)) - (int32_t)(nextStep + stepsTillRecalc)));
		break;

	case DMState::cartDecelNoReverse:							// Cartesian accelerating with no reversal
		nextCalcStepTime = pB - fastLimSqrtf(pA + pC * (float)(nextStep + stepsTillRecalc));
		break;

	case DMState::deltaForwardsReversing:						// moving forwards
		if (nextStep == reverseStartStep)
		{
			direction = false;
			directionChanged = true;
			state = DMState::deltaReverse;
		}
		// no break
	case DMState::deltaForwardsNoReverse:
	case DMState::deltaReverse:									// reversing on this and subsequent steps
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

			const float hmz0sc = mp.delta.fHmz0s * dda.directionVector[Z_AXIS];
			const float t1 = mp.delta.fMinusAaPlusBbTimesS + hmz0sc;
			const float t2a = mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(mp.delta.fHmz0s) + fsquare(t1);
			// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
			const float t2 = fastLimSqrtf(t2a);
			const float ds = (direction) ? t1 - t2 : t1 + t2;

			// Now feed ds into the step algorithm for Cartesian motion
			if (ds < 0.0)
			{
				state = DMState::stepError;
				nextStep += 110000000;							// so that we can tell what happened in the debug print
				return false;
			}

			const float pCds = pC * ds;
			nextCalcStepTime = (currentSegment->IsLinear()) ? pB + pCds
								: (currentSegment->IsAccelerating()) ? pB + fastLimSqrtf(pA + pCds)
									 : pB - fastLimSqrtf(pA + pCds);
			if (currentSegment->IsLinear()) { pA = ds; }	//DEBUG
		}
		break;

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
	if (iNextCalcStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// Very rarely on a delta, the penultimate step may also be calculated late. Allow for that here in case it affects Cartesian axes too.
		if (nextStep + stepsTillRecalc + 1 >= totalSteps)
		{
			iNextCalcStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any step except the last to be late
			state = DMState::stepError;
			nextStep += 120000000 + stepsTillRecalc;		// so we can tell what happened in the debug print
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

#if EVEN_STEPS
	nextStepTime = iNextCalcStepTime - (stepsTillRecalc * stepInterval);
#else
	nextStepTime = iNextCalcStepTime;
#endif

	return true;
}

// End
