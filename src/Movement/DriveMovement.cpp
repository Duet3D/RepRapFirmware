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

// Prepare this DM for a Cartesian axis move, returning true if there are steps to do
bool DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept
{
	const float stepsPerMm = (float)totalSteps/dda.totalDistance;
#if DM_USE_FPU
	fTwoCsquaredTimesMmPerStepDivA = (float)((double)(StepTimer::StepClockRateSquared * 2)/((double)stepsPerMm * (double)dda.acceleration));
	fTwoCsquaredTimesMmPerStepDivD = (float)((double)(StepTimer::StepClockRateSquared * 2)/((double)stepsPerMm * (double)dda.deceleration));
#else
	twoCsquaredTimesMmPerStepDivA = roundU64((double)(StepTimer::StepClockRateSquared * 2)/((double)stepsPerMm * (double)dda.acceleration));
	twoCsquaredTimesMmPerStepDivD = roundU64((double)(StepTimer::StepClockRateSquared * 2)/((double)stepsPerMm * (double)dda.deceleration));
#endif

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)(params.accelDistance * stepsPerMm) + 1;
	mp.cart.compensationClocks = mp.cart.accelCompensationClocks = 0;

	// Constant speed phase parameters
#if DM_USE_FPU
	fMmPerStepTimesCdivtopSpeed = (float)StepTimer::StepClockRate/(stepsPerMm * dda.topSpeed);
#else
	mmPerStepTimesCKdivtopSpeed = roundU32(((float)StepTimer::StepClockRate * K1)/(stepsPerMm * dda.topSpeed));
#endif

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (params.decelDistance * stepsPerMm < 0.5)
	{
		mp.cart.decelStartStep = totalSteps + 1;
#if DM_USE_FPU
		fTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
		twoDistanceToStopTimesCsquaredDivD = 0;
#endif
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)(params.decelStartDistance * stepsPerMm) + 1;
#if DM_USE_FPU
		fTwoDistanceToStopTimesCsquaredDivD = fsquare(params.fTopSpeedTimesCdivD) + (params.decelStartDistance * (StepTimer::StepClockRateSquared * 2))/dda.deceleration;
#else
		twoDistanceToStopTimesCsquaredDivD = isquare64(params.topSpeedTimesCdivD) + roundU64((params.decelStartDistance * (StepTimer::StepClockRateSquared * 2))/dda.deceleration);
#endif
	}

	// No reverse phase
	reverseStartStep = totalSteps + 1;
#if DM_USE_FPU
	mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
	mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0;
#endif

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	isDelta = false;
	state = (mp.cart.accelStopStep > 1) ? DMState::accel0
				: (mp.cart.decelStartStep > 1) ? DMState::steady
				  : DMState::decel0;
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
	const float h0MinusZ0 = fastSqrtf(dSquaredMinusAsquaredMinusBsquared);
#if DM_USE_FPU
	mp.delta.fHmz0s = h0MinusZ0 * stepsPerMm;
	mp.delta.fMinusAaPlusBbTimesS = -(aAplusbB * stepsPerMm);
	mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared = dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm);
	fTwoCsquaredTimesMmPerStepDivA = (float)((double)(2 * StepTimer::StepClockRateSquared)/((double)stepsPerMm * (double)dda.acceleration));
	fTwoCsquaredTimesMmPerStepDivD = (float)((double)(2 * StepTimer::StepClockRateSquared)/((double)stepsPerMm * (double)dda.deceleration));
#else
	mp.delta.hmz0sK = roundS32(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
	mp.delta.minusAaPlusBbTimesKs = -roundS32(aAplusbB * stepsPerMm * DriveMovement::K2);
	mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared = roundS64(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));
	twoCsquaredTimesMmPerStepDivA = roundU64((double)(2 * StepTimer::StepClockRateSquared)/((double)stepsPerMm * (double)dda.acceleration));
	twoCsquaredTimesMmPerStepDivD = roundU64((double)(2 * StepTimer::StepClockRateSquared)/((double)stepsPerMm * (double)dda.deceleration));
#endif

	// Calculate the distance at which we need to reverse direction.
	if (params.a2plusb2 <= 0.0)
	{
		// Pure Z movement. We can't use the main calculation because it divides by a2plusb2.
		direction = (dda.directionVector[Z_AXIS] >= 0.0);
		reverseStartStep = totalSteps + 1;
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
			const float hrev = dda.directionVector[Z_AXIS] * drev + fastSqrtf(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - params.a2plusb2 * fsquare(drev));
			const int32_t numStepsUp = (int32_t)((hrev - h0MinusZ0) * stepsPerMm);

			// We may be almost at the peak height already, in which case we don't really have a reversal.
			if (numStepsUp < 1 || (direction && (uint32_t)numStepsUp <= totalSteps))
			{
				reverseStartStep = totalSteps + 1;
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
			reverseStartStep = totalSteps + 1;
		}
	}

	// Acceleration phase parameters
#if DM_USE_FPU
	mp.delta.fAccelStopDs = params.accelDistance * stepsPerMm;
#else
	mp.delta.accelStopDsK = roundU32(params.accelDistance * stepsPerMm * K2);
#endif

	// Constant speed phase parameters
#if DM_USE_FPU
	fMmPerStepTimesCdivtopSpeed = (float)StepTimer::StepClockRate/(stepsPerMm * dda.topSpeed);
#else
	mmPerStepTimesCKdivtopSpeed = roundU32(((float)StepTimer::StepClockRate * K1)/(stepsPerMm * dda.topSpeed));
#endif

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (params.decelDistance * stepsPerMm < 0.5)
	{
#if DM_USE_FPU
		mp.delta.fDecelStartDs = std::numeric_limits<float>::max();
		fTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
		mp.delta.decelStartDsK = 0xFFFFFFFF;
		twoDistanceToStopTimesCsquaredDivD = 0;
#endif
	}
	else
	{
#if DM_USE_FPU
		mp.delta.fDecelStartDs = params.decelStartDistance * stepsPerMm;
		fTwoDistanceToStopTimesCsquaredDivD = fsquare(params.fTopSpeedTimesCdivD) + (params.decelStartDistance * (StepTimer::StepClockRateSquared * 2))/dda.deceleration;
#else
		mp.delta.decelStartDsK = roundU32(params.decelStartDistance * stepsPerMm * K2);
		twoDistanceToStopTimesCsquaredDivD = isquare64(params.topSpeedTimesCdivD) + roundU64((params.decelStartDistance * (StepTimer::StepClockRateSquared * 2))/dda.deceleration);
#endif
	}

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	//TODO input shaping for delta motion
	isDelta = true;
	return CalcNextStepTime(dda);
}

// Prepare this DM for an extruder move, returning true if there are steps to do
bool DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params, float& extrusionPending, float speedChange, bool doCompensation) noexcept
{
	// Calculate the requested extrusion amount and a few other things
	float dv = dda.directionVector[drive];
	float extrusionRequired = dda.totalDistance * dv;
	const size_t extruder = LogicalDriveToExtruder(drive);

#if SUPPORT_NONLINEAR_EXTRUSION
	// Add the nonlinear extrusion correction to totalExtrusion
	if (dda.flags.isPrintingMove)
	{
		float a, b, limit;
		if (reprap.GetPlatform().GetExtrusionCoefficients(extruder, a, b, limit))
		{
			const float averageExtrusionSpeed = (extrusionRequired * StepTimer::StepClockRate)/dda.clocksNeeded;
			const float factor = 1.0 + min<float>((averageExtrusionSpeed * a) + (averageExtrusionSpeed * averageExtrusionSpeed * b), limit);
			extrusionRequired *= factor;
		}
	}
#endif

	// Add on any fractional extrusion pending from the previous move
	extrusionRequired += extrusionPending;
	dv = extrusionRequired/dda.totalDistance;
	direction = (extrusionRequired >= 0.0);

	const float rawStepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float effectiveStepsPerMm = fabsf(dv) * rawStepsPerMm;

	float compensationTime;
	float accelCompensationDistance;

	if (doCompensation && direction)
	{
		// Calculate the pressure advance parameters
		compensationTime = reprap.GetPlatform().GetPressureAdvance(extruder);
		const float compensationClocks = compensationTime * (float)StepTimer::StepClockRate;
		mp.cart.compensationClocks = roundU32(compensationClocks);
		mp.cart.accelCompensationClocks = roundU32(compensationClocks * params.accelCompFactor);

#ifdef COMPENSATE_SPEED_CHANGES
		// If there is a speed change at the start of the move, theoretically we should instantly advance or retard the filament by the associated compensation amount.
		// We can't do that, so increase or decrease the extrusion factor instead, so that at least the extrusion will be correct by the end of the move.
		const float factor = 1.0 + (speedChange * compensationTime)/dda.totalDistance;
		stepsPerMm *= factor;
#endif
		// Calculate the net total extrusion to allow for compensation. It may be negative.
		extrusionRequired += (dda.endSpeed - dda.startSpeed) * compensationTime * dv;

		// Calculate the acceleration phase parameters
		accelCompensationDistance = compensationTime * (dda.topSpeed - dda.startSpeed);
		mp.cart.accelStopStep = (uint32_t)((params.accelDistance + accelCompensationDistance) * effectiveStepsPerMm) + 1;
	}
	else
	{
		accelCompensationDistance = compensationTime = 0.0;
		mp.cart.compensationClocks = mp.cart.accelCompensationClocks = 0;

		// Calculate the acceleration phase parameters
		mp.cart.accelStopStep = (uint32_t)(params.accelDistance * effectiveStepsPerMm) + 1;
	}

	int32_t netSteps = lrintf(extrusionRequired * rawStepsPerMm);
	extrusionPending = extrusionRequired - (float)netSteps/rawStepsPerMm;

	if (!direction)
	{
		netSteps = -netSteps;
	}

	// Note, netSteps may be negative at this point if we are applying pressure advance
#if DM_USE_FPU
	fTwoCsquaredTimesMmPerStepDivA = (double)(StepTimer::StepClockRateSquared * 2)/((double)effectiveStepsPerMm * (double)dda.acceleration);
	fTwoCsquaredTimesMmPerStepDivD = (double)(StepTimer::StepClockRateSquared * 2)/((double)effectiveStepsPerMm * (double)dda.deceleration);
#else
	twoCsquaredTimesMmPerStepDivA = roundU64((double)(StepTimer::StepClockRateSquared * 2)/((double)effectiveStepsPerMm * (double)dda.acceleration));
	twoCsquaredTimesMmPerStepDivD = roundU64((double)(StepTimer::StepClockRateSquared * 2)/((double)effectiveStepsPerMm * (double)dda.deceleration));
#endif

	// Constant speed phase parameters
#if DM_USE_FPU
	fMmPerStepTimesCdivtopSpeed = (float)StepTimer::StepClockRate/(effectiveStepsPerMm * dda.topSpeed);
#else
	mmPerStepTimesCKdivtopSpeed = (uint32_t)(((float)StepTimer::StepClockRate * K1)/(effectiveStepsPerMm * dda.topSpeed));
#endif

	// Calculate the deceleration and reverse phase parameters and update totalSteps
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (params.decelDistance * effectiveStepsPerMm < 0.5)		// if less than 1 deceleration step
	{
		totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
		mp.cart.decelStartStep = reverseStartStep = totalSteps + 1;
#if DM_USE_FPU
		mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0.0;
		fTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
		mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0;
		twoDistanceToStopTimesCsquaredDivD = 0;
#endif
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)((params.decelStartDistance + accelCompensationDistance) * effectiveStepsPerMm) + 1;
#if DM_USE_FPU
		const float initialDecelSpeedTimesCdivD = params.fTopSpeedTimesCdivD - (float)mp.cart.compensationClocks;
		const float initialDecelSpeedTimesCdivDSquared = fsquare(initialDecelSpeedTimesCdivD);
		fTwoDistanceToStopTimesCsquaredDivD =
			initialDecelSpeedTimesCdivDSquared + ((params.decelStartDistance + accelCompensationDistance) * (float)(StepTimer::StepClockRateSquared * 2))/dda.deceleration;
#else
		const int32_t initialDecelSpeedTimesCdivD = (int32_t)params.topSpeedTimesCdivD - (int32_t)mp.cart.compensationClocks;	// signed because it may be negative and we square it
		const uint64_t initialDecelSpeedTimesCdivDSquared = isquare64(initialDecelSpeedTimesCdivD);
		twoDistanceToStopTimesCsquaredDivD =
			initialDecelSpeedTimesCdivDSquared + roundU64(((params.decelStartDistance + accelCompensationDistance) * (float)(StepTimer::StepClockRateSquared * 2))/dda.deceleration);
#endif

		// See whether there is a reverse phase
		const float compensationSpeedChange = dda.deceleration * compensationTime;
		const uint32_t stepsBeforeReverse = (compensationSpeedChange > dda.topSpeed)
											? mp.cart.decelStartStep - 1
#if DM_USE_FPU
											: (uint32_t)(fTwoDistanceToStopTimesCsquaredDivD/fTwoCsquaredTimesMmPerStepDivD);
#else
											: twoDistanceToStopTimesCsquaredDivD/twoCsquaredTimesMmPerStepDivD;
#endif
		if (dda.endSpeed < compensationSpeedChange && (int32_t)stepsBeforeReverse > netSteps)
		{
			reverseStartStep = stepsBeforeReverse + 1;
			totalSteps = (uint32_t)((int32_t)(2 * stepsBeforeReverse) - netSteps);
#if DM_USE_FPU
			mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = (2 * stepsBeforeReverse) * fTwoCsquaredTimesMmPerStepDivD - fTwoDistanceToStopTimesCsquaredDivD;
#else
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD =
					(int64_t)((2 * stepsBeforeReverse) * twoCsquaredTimesMmPerStepDivD) - (int64_t)twoDistanceToStopTimesCsquaredDivD;
#endif
		}
		else
		{
			// There is no reverse phase. Check that we can actually do the last step requested.
			if (netSteps > (int32_t)stepsBeforeReverse)
			{
				netSteps = (int32_t)stepsBeforeReverse;
			}
			totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
			reverseStartStep = totalSteps + 1;
#if DM_USE_FPU
			mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0;
#endif
		}
	}

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	state = (mp.cart.accelStopStep > 1) ? DMState::accel0
				: (mp.cart.decelStartStep > 1) ? DMState::steady
					: (reverseStartStep > 1) ? DMState::decel0
						: DMState::reversing;
	isDelta = false;
	return CalcNextStepTime(dda);
}

#if SUPPORT_REMOTE_COMMANDS

// Prepare this DM for an extruder move. The caller has already checked that pressure advance is enabled.
//TODO are values in the DDA in the correct units for this code?
bool DriveMovement::PrepareRemoteExtruder(const DDA& dda, const PrepParams& params) noexcept
{
	// Calculate the pressure advance parameters
	const float compensationTime = reprap.GetPlatform().EutGetRemotePressureAdvance(drive);
	const float compensationClocks = compensationTime * (float)StepTimer::StepClockRate;
	mp.cart.compensationClocks = roundU32(compensationClocks);
	mp.cart.accelCompensationClocks = roundU32(compensationClocks * params.accelCompFactor);

	// Recalculate the net total step count to allow for compensation. It may be negative.
	const float compensationDistance = (dda.endSpeed - dda.startSpeed) * compensationTime;
	int32_t netSteps = lrintf((1.0 + compensationDistance) * totalSteps);

	// Calculate the acceleration phase parameters
	const float accelCompensationDistance = compensationTime * (dda.topSpeed - dda.startSpeed);
	mp.cart.accelStopStep = (uint32_t)((params.accelDistance + accelCompensationDistance) * totalSteps) + 1;

#if DM_USE_FPU
	fTwoCsquaredTimesMmPerStepDivA = (double)2.0/((double)totalSteps * (double)dda.acceleration);
	fTwoCsquaredTimesMmPerStepDivD = (double)2.0/((double)totalSteps * (double)dda.deceleration);
#else
	twoCsquaredTimesMmPerStepDivA = roundU64((double)2.0/((double)totalSteps * (double)dda.acceleration));
	twoCsquaredTimesMmPerStepDivD = roundU64((double)2.0/((double)totalSteps * (double)dda.deceleration));
#endif

	// Constant speed phase parameters
#if DM_USE_FPU
	fMmPerStepTimesCdivtopSpeed = 1.0/(totalSteps * dda.topSpeed);
#else
	mmPerStepTimesCKdivtopSpeed = (uint32_t)((float)K1/(totalSteps * dda.topSpeed));
#endif

	// Calculate the deceleration and reverse phase parameters and update totalSteps
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (params.decelDistance * totalSteps < 0.5)		// if less than 1 deceleration step
	{
		totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
		mp.cart.decelStartStep = reverseStartStep = netSteps + 1;
#if DM_USE_FPU
		mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0.0;
		fTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
		mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0;
		twoDistanceToStopTimesCsquaredDivD = 0;
#endif
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)((params.decelStartDistance + accelCompensationDistance) * totalSteps) + 1;
#if DM_USE_FPU
		const float initialDecelSpeedTimesCdivD = params.fTopSpeedTimesCdivD - (float)mp.cart.compensationClocks;
		const float initialDecelSpeedTimesCdivDSquared = fsquare(initialDecelSpeedTimesCdivD);
		fTwoDistanceToStopTimesCsquaredDivD = initialDecelSpeedTimesCdivDSquared + ((params.decelStartDistance + accelCompensationDistance) * 2)/dda.deceleration;
#else
		const int32_t initialDecelSpeedTimesCdivD = (int32_t)params.topSpeedTimesCdivD - (int32_t)mp.cart.compensationClocks;	// signed because it may be negative and we square it
		const uint64_t initialDecelSpeedTimesCdivDSquared = isquare64(initialDecelSpeedTimesCdivD);
		twoDistanceToStopTimesCsquaredDivD = initialDecelSpeedTimesCdivDSquared + roundU64(((params.decelStartDistance + accelCompensationDistance) * 2)/dda.deceleration);
#endif

		// See whether there is a reverse phase
		const float compensationSpeedChange = dda.deceleration * compensationTime;
		const uint32_t stepsBeforeReverse = (compensationSpeedChange > dda.topSpeed)
											? mp.cart.decelStartStep - 1
#if DM_USE_FPU
											: fTwoDistanceToStopTimesCsquaredDivD/fTwoCsquaredTimesMmPerStepDivD;
#else
											: twoDistanceToStopTimesCsquaredDivD/twoCsquaredTimesMmPerStepDivD;
#endif
		if (dda.endSpeed < compensationSpeedChange && (int32_t)stepsBeforeReverse > netSteps)
		{
			reverseStartStep = stepsBeforeReverse + 1;
			totalSteps = (uint32_t)((int32_t)(2 * stepsBeforeReverse) - netSteps);
#if DM_USE_FPU
			mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = ((2 * stepsBeforeReverse) *fTwoCsquaredTimesMmPerStepDivD) - fTwoDistanceToStopTimesCsquaredDivD;
#else
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD =
					(int64_t)((2 * stepsBeforeReverse) * twoCsquaredTimesMmPerStepDivD) - (int64_t)twoDistanceToStopTimesCsquaredDivD;
#endif
		}
		else
		{
			// There is no reverse phase. Check that we can actually do the last step requested.
			if (netSteps > (int32_t)stepsBeforeReverse)
			{
				netSteps = (int32_t)stepsBeforeReverse;
			}
			reverseStartStep = netSteps + 1;
			totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
#if DM_USE_FPU
			mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0;
#endif
		}
	}

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	state = (mp.cart.accelStopStep > 1) ? DMState::accel0
				: (mp.cart.decelStartStep > 1) ? DMState::steady
					: (reverseStartStep > 1) ? DMState::decel0
						: DMState::reversing;
	isDelta = false;
	return CalcNextStepTime(dda);
}

#endif

void DriveMovement::DebugPrint() const noexcept
{
	char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
#if DM_USE_FPU
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32
					" 2dtstc2diva=%.2f\n",
					c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval,
					(double)fTwoDistanceToStopTimesCsquaredDivD);
#else
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32
					" 2dtstc2diva=%" PRIu64 "\n",
					c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval,
					twoDistanceToStopTimesCsquaredDivD);
#endif
		if (isDelta)
		{
#if DM_USE_FPU
			debugPrintf("hmz0s=%.2f minusAaPlusBbTimesS=%.2f dSquaredMinusAsquaredMinusBsquared=%.2f\n"
						"2c2mmsda=%.2f 2c2mmsdd=%.2f asds=%.2f dsds=%.2f mmstcdts=%.2f\n",
						(double)mp.delta.fHmz0s, (double)mp.delta.fMinusAaPlusBbTimesS, (double)mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared,
						(double)fTwoCsquaredTimesMmPerStepDivA, (double)fTwoCsquaredTimesMmPerStepDivD, (double)mp.delta.fAccelStopDs, (double)mp.delta.fDecelStartDs, (double)fMmPerStepTimesCdivtopSpeed
						);
#else
			debugPrintf("hmz0sK=%" PRIi32 " minusAaPlusBbTimesKs=%" PRIi32 " dSquaredMinusAsquaredMinusBsquared=%" PRId64 "\n"
						"2c2mmsda=%" PRIu64 " 2c2mmsdd=%" PRIu64 " asdsk=%" PRIu32 " dsdsk=%" PRIu32 " mmstcdts=%" PRIu32 "\n",
						mp.delta.hmz0sK, mp.delta.minusAaPlusBbTimesKs, mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared,
						twoCsquaredTimesMmPerStepDivA, twoCsquaredTimesMmPerStepDivD, mp.delta.accelStopDsK, mp.delta.decelStartDsK, mmPerStepTimesCKdivtopSpeed
						);
#endif
		}
		else
		{
#if DM_USE_FPU
			debugPrintf("accelStopStep=%" PRIu32 " decelStartStep=%" PRIu32 " 2c2mmsda=%.2f 2c2mmsdd=%.2f\n"
						"mmPerStepTimesCdivtopSpeed=%.2f fmsdmtstdca2=%.2f cc=%" PRIu32 " acc=%" PRIu32 "\n",
						mp.cart.accelStopStep, mp.cart.decelStartStep, (double)fTwoCsquaredTimesMmPerStepDivA, (double)fTwoCsquaredTimesMmPerStepDivD,
						(double)fMmPerStepTimesCdivtopSpeed, (double)mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD, mp.cart.compensationClocks, mp.cart.accelCompensationClocks
						);
#else
			debugPrintf("accelStopStep=%" PRIu32 " decelStartStep=%" PRIu32 " 2c2mmsda=%" PRIu64 " 2c2mmsdd=%" PRIu64 "\n"
						"mmPerStepTimesCdivtopSpeed=%" PRIu32 " fmsdmtstdca2=%" PRId64 " cc=%" PRIu32 " acc=%" PRIu32 "\n",
						mp.cart.accelStopStep, mp.cart.decelStartStep, twoCsquaredTimesMmPerStepDivA, twoCsquaredTimesMmPerStepDivD,
						mmPerStepTimesCKdivtopSpeed, mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD, mp.cart.compensationClocks, mp.cart.accelCompensationClocks
						);
#endif
		}
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do.
// This is also used for extruders on delta machines.
bool DriveMovement::CalcNextStepTimeCartesianFull(const DDA &dda) noexcept
pre(nextStep < totalSteps; stepsTillRecalc == 0)
{
	// Work out how many steps to calculate at a time.
	uint32_t shiftFactor = 0;		// assume single stepping
	uint32_t nextCalcStepTime;
	switch (state)
	{
	case DMState::accel0:	// acceleration phase
		{
			const uint32_t stepsToLimit = mp.cart.accelStopStep - nextStep;
			if (stepsToLimit == 1)
			{
				// This is the last step in this phase
				state = (mp.cart.decelStartStep > mp.cart.accelStopStep) ? DMState::steady
						: (reverseStartStep > mp.cart.accelStopStep) ? DMState::decel0
							: DMState::reversing;
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian)
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
			const uint32_t nextCalcStep = nextStep + stepsTillRecalc;
#if DM_USE_FPU
			const float adjustedStartSpeedTimesCdivA = (float)(dda.afterPrepare.startSpeedTimesCdivA + mp.cart.compensationClocks);
			nextCalcStepTime = (uint32_t)(fastSqrtf(fsquare(adjustedStartSpeedTimesCdivA) + (fTwoCsquaredTimesMmPerStepDivA * nextCalcStep)) - adjustedStartSpeedTimesCdivA);
#else
			const uint32_t adjustedStartSpeedTimesCdivA = dda.afterPrepare.startSpeedTimesCdivA + mp.cart.compensationClocks;
			nextCalcStepTime = isqrt64(isquare64(adjustedStartSpeedTimesCdivA) + (twoCsquaredTimesMmPerStepDivA * nextCalcStep)) - adjustedStartSpeedTimesCdivA;
#endif
		}
		break;

	case DMState::steady:	// steady speed phase
		{
			const uint32_t stepsToLimit = mp.cart.decelStartStep - nextStep;
			if (stepsToLimit == 1)
			{
				state = (reverseStartStep > mp.cart.decelStartStep) ? DMState::decel0
							: DMState::reversing;
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian)
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
			const uint32_t nextCalcStep = nextStep + stepsTillRecalc;
			nextCalcStepTime =
#if DM_USE_FPU
					(uint32_t)(  (int32_t)(fMmPerStepTimesCdivtopSpeed * nextCalcStep)
							   + dda.afterPrepare.extraAccelerationClocks
							   - (int32_t)mp.cart.accelCompensationClocks
							  );
#else
					(uint32_t)(  (int32_t)(((uint64_t)mmPerStepTimesCKdivtopSpeed * nextCalcStep)/K1)
							   + dda.afterPrepare.extraAccelerationClocks
							   - (int32_t)mp.cart.accelCompensationClocks
							  );
#endif
		}
		break;

	case DMState::decel0:	// deceleration phase, not reversed yet
		{
			const uint32_t stepsToLimit = reverseStartStep - nextStep;
			if (stepsToLimit == 1)
			{
				state = DMState::reversing;
			}
			else if (stepInterval < DDA::MinCalcIntervalCartesian)
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
			const uint32_t nextCalcStep = nextStep + stepsTillRecalc;
			const uint32_t adjustedTopSpeedTimesCdivDPlusDecelStartClocks = dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks - mp.cart.compensationClocks;
#if DM_USE_FPU
			const float temp = fTwoCsquaredTimesMmPerStepDivD * nextCalcStep;
			// Allow for possible rounding error when the end speed is zero or very small
			nextCalcStepTime = (temp < fTwoDistanceToStopTimesCsquaredDivD)
							? adjustedTopSpeedTimesCdivDPlusDecelStartClocks - (uint32_t)(fastSqrtf(fTwoDistanceToStopTimesCsquaredDivD - temp))
							: adjustedTopSpeedTimesCdivDPlusDecelStartClocks;
#else
			const uint64_t temp = twoCsquaredTimesMmPerStepDivD * nextCalcStep;
			// Allow for possible rounding error when the end speed is zero or very small
			nextCalcStepTime = (temp < twoDistanceToStopTimesCsquaredDivD)
							? adjustedTopSpeedTimesCdivDPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivD - temp)
							: adjustedTopSpeedTimesCdivDPlusDecelStartClocks;
#endif
		}
		break;

	case DMState::reversing:
		direction = !direction;
		directionChanged = true;
		state = DMState::reverse;
		// no break
	case DMState::reverse:	// reverse phase
		{
			const uint32_t stepsToLimit = totalSteps + 1 - nextStep;
			if (stepInterval < DDA::MinCalcIntervalCartesian)
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
			const uint32_t nextCalcStep = nextStep + stepsTillRecalc;
			const uint32_t adjustedTopSpeedTimesCdivDPlusDecelStartClocks = dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks - mp.cart.compensationClocks;
			nextCalcStepTime = adjustedTopSpeedTimesCdivDPlusDecelStartClocks
#if DM_USE_FPU
								+ (uint32_t)(fastSqrtf((fTwoCsquaredTimesMmPerStepDivD * nextCalcStep) - mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD));
#else
								+ isqrt64((int64_t)(twoCsquaredTimesMmPerStepDivD * nextCalcStep) - mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD);
#endif
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

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
// Return true if there are more steps to do
bool DriveMovement::CalcNextStepTimeDeltaFull(const DDA &dda) noexcept
pre(nextStep < totalSteps; stepsTillRecalc == 0)
{
	// Work out how many steps to calculate at a time.
	// The last step before reverseStartStep must be single stepped to make sure that we don't reverse the direction too soon.
	// The simulator suggests that at 200steps/mm, the minimum step pulse interval for 400mm/sec movement is 4.5us
	uint32_t shiftFactor = 0;		// assume single stepping
	if (stepInterval < DDA::MinCalcIntervalDelta)
	{
		const uint32_t stepsToLimit = ((nextStep < reverseStartStep && reverseStartStep <= totalSteps)
										? reverseStartStep
										: totalSteps
									  ) - nextStep;
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

	if (nextStep == reverseStartStep)
	{
		direction = false;
		directionChanged = true;
	}

	// Calculate d*s*K as an integer, where d = distance the head has travelled, s = steps/mm for this drive, K = a power of 2 to reduce the rounding errors
	// K here means K2
	// mp.delta.hmz0sk = (number of steps by which the carriage is higher than Z) * K2
	{
#if DM_USE_FPU
		float steps = float(1u << shiftFactor);
		if (!direction)
		{
			steps = -steps;
		}
		mp.delta.fHmz0s += steps;								// get new carriage height above Z in steps
#else
		int32_t shiftedK2 = (int32_t)(K2 << shiftFactor);
		if (!direction)
		{
			shiftedK2 = -shiftedK2;
		}
		mp.delta.hmz0sK += shiftedK2;							// get K2 * (new carriage height above Z in steps)
#endif
	}

#if DM_USE_FPU
	const float hmz0sc = mp.delta.fHmz0s * dda.directionVector[Z_AXIS];
	const float t1 = mp.delta.fMinusAaPlusBbTimesS + hmz0sc;
	// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
	const float t2a = mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(mp.delta.fHmz0s) + fsquare(t1);
	const float t2 = (t2a > 0.0) ? fastSqrtf(t2a) : 0.0;
	const float ds = (direction) ? t1 - t2 : t1 + t2;
#else
	// In the following, cKc is the Z movement fraction of the total move scaled by Kc
	const int32_t hmz0scK = (int32_t)(((int64_t)mp.delta.hmz0sK * dda.afterPrepare.cKc)/Kc);
	const int32_t t1 = mp.delta.minusAaPlusBbTimesKs + hmz0scK;
	// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
	const int64_t t2a = mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - (int64_t)isquare64(mp.delta.hmz0sK) + (int64_t)isquare64(t1);
	const int32_t t2 = (t2a > 0) ? isqrt64(t2a) : 0;
	const int32_t dsK = (direction) ? t1 - t2 : t1 + t2;
#endif

	// Now feed dsK into a modified version of the step algorithm for Cartesian motion without elasticity compensation
#if DM_USE_FPU
	if (ds < 0.0)
#else
	if (dsK < 0)
#endif
	{
		state = DMState::stepError;
		nextStep += 1000000;									// so that we can tell what happened in the debug print
		return false;
	}

	uint32_t nextCalcStepTime;
#if DM_USE_FPU
	if (ds < mp.delta.fAccelStopDs)
	{
		// Acceleration phase
		nextCalcStepTime = (uint32_t)(fastSqrtf(fsquare((float)dda.afterPrepare.startSpeedTimesCdivA) + (fTwoCsquaredTimesMmPerStepDivA * ds))) - dda.afterPrepare.startSpeedTimesCdivA;
	}
	else if (ds < mp.delta.fDecelStartDs)
	{
		// Steady speed phase
		nextCalcStepTime = (uint32_t)(fMmPerStepTimesCdivtopSpeed * ds) + dda.afterPrepare.extraAccelerationClocks;
	}
	else
	{
		const float temp = fTwoCsquaredTimesMmPerStepDivD * ds;
		// Because of possible rounding error when the end speed is zero or very small, we need to check that the square root will work OK
		nextCalcStepTime = (temp < fTwoDistanceToStopTimesCsquaredDivD)
						? dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks - lrintf(fastSqrtf(fTwoDistanceToStopTimesCsquaredDivD - temp))
						: dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks;
	}
#else
	if ((uint32_t)dsK < mp.delta.accelStopDsK)
	{
		// Acceleration phase
		nextCalcStepTime = isqrt64(isquare64(dda.afterPrepare.startSpeedTimesCdivA) + (twoCsquaredTimesMmPerStepDivA * (uint32_t)dsK)/K2) - dda.afterPrepare.startSpeedTimesCdivA;
	}
	else if ((uint32_t)dsK < mp.delta.decelStartDsK)
	{
		// Steady speed phase
		nextCalcStepTime = (uint32_t)(  (int32_t)(((uint64_t)mmPerStepTimesCKdivtopSpeed * (uint32_t)dsK)/(K1 * K2))
								  + dda.afterPrepare.extraAccelerationClocks
								 );
	}
	else
	{
		const uint64_t temp = (twoCsquaredTimesMmPerStepDivD * (uint32_t)dsK)/K2;
		// Because of possible rounding error when the end speed is zero or very small, we need to check that the square root will work OK
		nextCalcStepTime = (temp < twoDistanceToStopTimesCsquaredDivD)
						? dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivD - temp)
						: dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks;
	}
#endif

	// When crossing between movement phases with high microstepping, due to rounding errors the next step may appear to be due before the last one.
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
		// Very rarely, the penultimate step may be calculated late, so allow for that too.
		if (nextStep + 1 >= totalSteps)
		{
			nextStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any steps except the last two to be late
			state = DMState::stepError;
			stepInterval = 10000000 + nextStepTime;		// so we can tell what happened in the debug print
			return false;
		}
	}
	return true;
}

// End
