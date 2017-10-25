/*
 * DriveMovement.cpp
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#include "DriveMovement.h"
#include "DDA.h"
#include "Move.h"
#include "RepRap.h"
#include "Libraries/Math/Isqrt.h"
#include "Kinematics/LinearDeltaKinematics.h"

// Static members

DriveMovement *DriveMovement::freeList = nullptr;
int DriveMovement::numFree = 0;
int DriveMovement::minFree = 0;

void DriveMovement::InitialAllocate(unsigned int num)
{
	while (num != 0)
	{
		freeList = new DriveMovement(freeList);
		++numFree;
		--num;
	}
	ResetMinFree();
}

DriveMovement *DriveMovement::Allocate(size_t drive, DMState st)
{
	DriveMovement *dm = freeList;
	if (dm != nullptr)
	{
		freeList = dm->nextDM;
		--numFree;
		if (numFree < minFree)
		{
			minFree = numFree;
		}
		dm->nextDM = nullptr;
		dm->drive = (uint8_t)drive;
		dm->state = st;
	}
	return dm;
}

// Constructors
DriveMovement::DriveMovement(DriveMovement *next) : nextDM(next)
{
}

// Non static members

// Prepare this DM for a Cartesian axis move
void DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params)
{
	const float stepsPerMm = (float)totalSteps/dda.totalDistance;
	mp.cart.twoCsquaredTimesMmPerStepDivA = (uint64_t)(((float)DDA::stepClockRate * (float)DDA::stepClockRate)/(stepsPerMm * dda.acceleration)) * 2;

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)(dda.accelDistance * stepsPerMm) + 1;
	startSpeedTimesCdivA = params.startSpeedTimesCdivA;

	// Constant speed phase parameters
	mp.cart.mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)DDA::stepClockRate * K1)/(stepsPerMm * dda.topSpeed));
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = params.accelClocksMinusAccelDistanceTimesCdivTopSpeed;

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)
	{
		mp.cart.decelStartStep = totalSteps + 1;
		topSpeedTimesCdivAPlusDecelStartClocks = 0;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)(params.decelStartDistance * stepsPerMm) + 1;
		topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivAPlusDecelStartClocks;
		const uint64_t initialDecelSpeedTimesCdivASquared = isquare64(params.topSpeedTimesCdivA);
		twoDistanceToStopTimesCsquaredDivA = initialDecelSpeedTimesCdivASquared + (uint64_t)((params.decelStartDistance * (DDA::stepClockRateSquared * 2))/dda.acceleration);
	}

	// No reverse phase
	reverseStartStep = totalSteps + 1;
	mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
}

// Prepare this DM for a Delta axis move
void DriveMovement::PrepareDeltaAxis(const DDA& dda, const PrepParams& params)
{
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float A = params.initialX - params.dparams->GetTowerX(drive);
	const float B = params.initialY - params.dparams->GetTowerY(drive);
	const float aAplusbB = A * dda.directionVector[X_AXIS] + B * dda.directionVector[Y_AXIS];
	const float dSquaredMinusAsquaredMinusBsquared = params.diagonalSquared - fsquare(A) - fsquare(B);
	const float h0MinusZ0 = sqrtf(dSquaredMinusAsquaredMinusBsquared);
	mp.delta.hmz0sK = (int32_t)(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
	mp.delta.minusAaPlusBbTimesKs = -(int32_t)(aAplusbB * stepsPerMm * DriveMovement::K2);
	mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared =
			(int64_t)(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));

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
		const float drev = ((dda.directionVector[Z_AXIS] * sqrtf(params.a2b2D2 - fsquare(A * dda.directionVector[Y_AXIS] - B * dda.directionVector[X_AXIS])))
							- aAplusbB)/params.a2plusb2;
		if (drev > 0.0 && drev < dda.totalDistance)		// if the reversal point is within range
		{
			// Calculate how many steps we need to move up before reversing
			const float hrev = dda.directionVector[Z_AXIS] * drev + sqrtf(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - params.a2plusb2 * fsquare(drev));
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

	mp.delta.twoCsquaredTimesMmPerStepDivAK = (uint32_t)((float)DDA::stepClockRateSquared/(stepsPerMm * dda.acceleration * (K2/2)));

	// Acceleration phase parameters
	mp.delta.accelStopDsK = (uint32_t)(dda.accelDistance * stepsPerMm * K2);
	startSpeedTimesCdivA = params.startSpeedTimesCdivA;

	// Constant speed phase parameters
	mp.delta.mmPerStepTimesCdivtopSpeedK = (uint32_t)(((float)DDA::stepClockRate * K1)/(stepsPerMm * dda.topSpeed));
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = params.accelClocksMinusAccelDistanceTimesCdivTopSpeed;

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)
	{
		mp.delta.decelStartDsK = 0xFFFFFFFF;
		topSpeedTimesCdivAPlusDecelStartClocks = 0;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		mp.delta.decelStartDsK = (uint32_t)(params.decelStartDistance * stepsPerMm * K2);
		topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivAPlusDecelStartClocks;
		const uint64_t initialDecelSpeedTimesCdivASquared = isquare64(params.topSpeedTimesCdivA);
		twoDistanceToStopTimesCsquaredDivA = initialDecelSpeedTimesCdivASquared + (uint64_t)((params.decelStartDistance * (DDA::stepClockRateSquared * 2))/dda.acceleration);
	}
}

// Prepare this DM for an extruder move
void DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params, bool doCompensation)
{
	const float dv = dda.directionVector[drive];
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive) * fabsf(dv);
	mp.cart.twoCsquaredTimesMmPerStepDivA = (uint64_t)(((float)DDA::stepClockRate * (float)DDA::stepClockRate)/(stepsPerMm * dda.acceleration)) * 2;

	// Calculate the pressure advance parameter
	const float compensationTime = (doCompensation && dv > 0.0) ? reprap.GetPlatform().GetPressureAdvance(drive - reprap.GetGCodes().GetTotalAxes()) : 0.0;
	const uint32_t compensationClocks = (uint32_t)(compensationTime * DDA::stepClockRate);

	// Calculate the net total step count to allow for compensation. It may be negative.
	// Note that we add totalSteps in floating point mode, to round the number of steps down consistently
	const float compensationDistance = (dda.endSpeed - dda.startSpeed) * compensationTime;
	const int32_t netSteps = (int32_t)(compensationDistance * stepsPerMm) + (int32_t)totalSteps;

	// Calculate the acceleration phase parameters
	const float accelCompensationDistance = compensationTime * (dda.topSpeed - dda.startSpeed);

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)((dda.accelDistance + accelCompensationDistance) * stepsPerMm) + 1;
	startSpeedTimesCdivA = params.startSpeedTimesCdivA + compensationClocks;

	// Constant speed phase parameters
	mp.cart.mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)DDA::stepClockRate * K1)/(stepsPerMm * dda.topSpeed));
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)params.accelClocksMinusAccelDistanceTimesCdivTopSpeed - (int32_t)(compensationClocks * params.compFactor);

	// Calculate the deceleration and reverse phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)		// if less than 1 deceleration step
	{
		totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
		mp.cart.decelStartStep = reverseStartStep = netSteps + 1;
		topSpeedTimesCdivAPlusDecelStartClocks = 0;
		mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)((params.decelStartDistance + accelCompensationDistance) * stepsPerMm) + 1;
		const int32_t initialDecelSpeedTimesCdivA = (int32_t)params.topSpeedTimesCdivA - (int32_t)compensationClocks;	// signed because it may be negative and we square it
		const uint64_t initialDecelSpeedTimesCdivASquared = isquare64(initialDecelSpeedTimesCdivA);
		topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivAPlusDecelStartClocks - compensationClocks;
		twoDistanceToStopTimesCsquaredDivA =
			initialDecelSpeedTimesCdivASquared + (uint64_t)(((params.decelStartDistance + accelCompensationDistance) * (DDA::stepClockRateSquared * 2))/dda.acceleration);

		// Calculate the move distance to the point of zero speed, where reverse motion starts
		const float initialDecelSpeed = dda.topSpeed - dda.acceleration * compensationTime;
		const float reverseStartDistance = (initialDecelSpeed > 0.0)
												? fsquare(initialDecelSpeed)/(2 * dda.acceleration) + params.decelStartDistance
												: params.decelStartDistance;
		// Reverse phase parameters
		if (reverseStartDistance >= dda.totalDistance)
		{
			// No reverse phase
			totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
			reverseStartStep = netSteps + 1;
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		}
		else
		{
			reverseStartStep = (initialDecelSpeed < 0.0)
								? mp.cart.decelStartStep
								: (twoDistanceToStopTimesCsquaredDivA/mp.cart.twoCsquaredTimesMmPerStepDivA) + 1;
			// Because the step numbers are rounded down, we may sometimes get a situation in which netSteps = 1 and reverseStartStep = 1.
			// This would lead to totalSteps = -1, which must be avoided.
			const int32_t overallSteps = (int32_t)(2 * (reverseStartStep - 1)) - netSteps;
			if (overallSteps > 0)
			{
				totalSteps = overallSteps;
				mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA =
						(int64_t)((2 * (reverseStartStep - 1)) * mp.cart.twoCsquaredTimesMmPerStepDivA) - (int64_t)twoDistanceToStopTimesCsquaredDivA;
			}
			else
			{
				totalSteps = (uint32_t)max<int32_t>(netSteps, 0);
				reverseStartStep = totalSteps + 1;
				mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
			}
		}
	}
}

void DriveMovement::DebugPrint(char c, bool isDeltaMovement) const
{
	if (state != DMState::idle)
	{
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32 " sstcda=%" PRIu32 " "
					"acmadtcdts=%" PRIi32 " tstcdapdsc=%" PRIu32 " 2dtstc2diva=%" PRIu64 "\n",
					c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval, startSpeedTimesCdivA,
					accelClocksMinusAccelDistanceTimesCdivTopSpeed, topSpeedTimesCdivAPlusDecelStartClocks, twoDistanceToStopTimesCsquaredDivA);

		if (isDeltaMovement)
		{
			debugPrintf("hmz0sK=%" PRIi32 " minusAaPlusBbTimesKs=%" PRIi32 " dSquaredMinusAsquaredMinusBsquared=%" PRId64 "\n"
						"2c2mmsdak=%" PRIu32 " asdsk=%" PRIu32 " dsdsk=%" PRIu32 " mmstcdts=%" PRIu32 "\n",
						mp.delta.hmz0sK, mp.delta.minusAaPlusBbTimesKs, mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared,
						mp.delta.twoCsquaredTimesMmPerStepDivAK, mp.delta.accelStopDsK, mp.delta.decelStartDsK, mp.delta.mmPerStepTimesCdivtopSpeedK
						);
		}
		else
		{
			debugPrintf("accelStopStep=%" PRIu32 " decelStartStep=%" PRIu32 " 2CsqtMmPerStepDivA=%" PRIu64 "\n"
						"mmPerStepTimesCdivtopSpeed=%" PRIu32 " fmsdmtstdca2=%" PRId64 "\n",
						mp.cart.accelStopStep, mp.cart.decelStartStep, mp.cart.twoCsquaredTimesMmPerStepDivA,
						mp.cart.mmPerStepTimesCdivtopSpeed, mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA
						);
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
bool DriveMovement::CalcNextStepTimeCartesianFull(const DDA &dda, bool live)
pre(nextStep < totalSteps; stepsTillRecalc == 0)
{
	// Work out how many steps to calculate at a time.
	// The last step before reverseStartStep must be single stepped to make sure that we don't reverse the direction too soon.
	uint32_t shiftFactor = 0;		// assume single stepping
	if (stepInterval < DDA::MinCalcIntervalCartesian)
	{
		uint32_t stepsToLimit = ((nextStep <= reverseStartStep && reverseStartStep <= totalSteps)
									? reverseStartStep
									: totalSteps
								) - nextStep;
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
	const uint32_t lastStepTime = nextStepTime;					// pick up the time of the last step
	if (nextCalcStep < mp.cart.accelStopStep)
	{
		// acceleration phase
		nextStepTime = isqrt64(isquare64(startSpeedTimesCdivA) + (mp.cart.twoCsquaredTimesMmPerStepDivA * nextCalcStep)) - startSpeedTimesCdivA;
	}
	else if (nextCalcStep < mp.cart.decelStartStep)
	{
		// steady speed phase
		nextStepTime = (uint32_t)((int32_t)(((uint64_t)mp.cart.mmPerStepTimesCdivtopSpeed * nextCalcStep)/K1) + accelClocksMinusAccelDistanceTimesCdivTopSpeed);
	}
	else if (nextCalcStep < reverseStartStep)
	{
		// deceleration phase, not reversed yet
		const uint64_t temp = mp.cart.twoCsquaredTimesMmPerStepDivA * nextCalcStep;
		// Allow for possible rounding error when the end speed is zero or very small
		nextStepTime = (twoDistanceToStopTimesCsquaredDivA > temp)
						? topSpeedTimesCdivAPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivA - temp)
						: topSpeedTimesCdivAPlusDecelStartClocks;
	}
	else
	{
		// deceleration phase, reversing or already reversed
		if (nextCalcStep == reverseStartStep)
		{
			direction = !direction;
			if (live)
			{
				reprap.GetPlatform().SetDirection(drive, direction);
			}
		}
		nextStepTime = topSpeedTimesCdivAPlusDecelStartClocks
							+ isqrt64((int64_t)(mp.cart.twoCsquaredTimesMmPerStepDivA * nextCalcStep) - mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA);
	}

	stepInterval = (nextStepTime - lastStepTime) >> shiftFactor;	// calculate the time per step, ready for next time

	if (nextStepTime > dda.clocksNeeded)
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
bool DriveMovement::CalcNextStepTimeDeltaFull(const DDA &dda, bool live)
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
		if (live)
		{
			reprap.GetPlatform().SetDirection(drive, false);	// going down now
		}
	}

	// Calculate d*s*K as an integer, where d = distance the head has travelled, s = steps/mm for this drive, K = a power of 2 to reduce the rounding errors
	{
		int32_t shiftedK2 = (int32_t)(K2 << shiftFactor);
		if (!direction)
		{
			shiftedK2 = -shiftedK2;
		}
		mp.delta.hmz0sK += shiftedK2;
	}

	const int32_t hmz0scK = (int32_t)(((int64_t)mp.delta.hmz0sK * dda.cKc)/Kc);
	const int32_t t1 = mp.delta.minusAaPlusBbTimesKs + hmz0scK;
	// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
	const int64_t t2a = mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - (int64_t)isquare64(mp.delta.hmz0sK) + (int64_t)isquare64(t1);
	const int32_t t2 = (t2a > 0) ? isqrt64(t2a) : 0;
	const int32_t dsK = (direction) ? t1 - t2 : t1 + t2;

	// Now feed dsK into a modified version of the step algorithm for Cartesian motion without elasticity compensation
	if (dsK < 0)
	{
		state = DMState::stepError;
		nextStep += 1000000;						// so that we can tell what happened in the debug print
		return false;
	}

	const uint32_t lastStepTime = nextStepTime;		// pick up the time of the last step
	if ((uint32_t)dsK < mp.delta.accelStopDsK)
	{
		// Acceleration phase
		nextStepTime = isqrt64(isquare64(startSpeedTimesCdivA) + ((uint64_t)mp.delta.twoCsquaredTimesMmPerStepDivAK * (uint32_t)dsK)) - startSpeedTimesCdivA;
	}
	else if ((uint32_t)dsK < mp.delta.decelStartDsK)
	{
		// Steady speed phase
		nextStepTime = (uint32_t)((int32_t)(((uint64_t)mp.delta.mmPerStepTimesCdivtopSpeedK * (uint32_t)dsK)/(K1 * K2)) + accelClocksMinusAccelDistanceTimesCdivTopSpeed);
	}
	else
	{
		const uint64_t temp = (uint64_t)mp.delta.twoCsquaredTimesMmPerStepDivAK * (uint32_t)dsK;
		// Because of possible rounding error when the end speed is zero or very small, we need to check that the square root will work OK
		nextStepTime = (temp < twoDistanceToStopTimesCsquaredDivA)
						? topSpeedTimesCdivAPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivA - temp)
						: topSpeedTimesCdivAPlusDecelStartClocks;
	}

	stepInterval = (nextStepTime - lastStepTime) >> shiftFactor;	// calculate the time per step, ready for next time

	if (nextStepTime > dda.clocksNeeded)
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

// Reduce the speed of this movement. Called to reduce the homing speed when we detect we are near the endstop for a drive.
void DriveMovement::ReduceSpeed(const DDA& dda, float inverseSpeedFactor)
{
	if (dda.isDeltaMovement)
	{
		// Force the linear motion phase
		mp.delta.accelStopDsK = 0;
		mp.delta.decelStartDsK = 0xFFFFFFFF;

		// Adjust the speed
		mp.delta.mmPerStepTimesCdivtopSpeedK = (uint32_t)(inverseSpeedFactor * mp.delta.mmPerStepTimesCdivtopSpeedK);

		// Adjust the acceleration clocks to as to maintain continuity of movement
		const int32_t hmz0scK = (int32_t)(((int64_t)mp.delta.hmz0sK * dda.cKc)/Kc);
		const int32_t t1 = mp.delta.minusAaPlusBbTimesKs + hmz0scK;
		const int32_t t2 = isqrt64(isquare64(t1) + mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - isquare64(mp.delta.hmz0sK));
		const int32_t dsK = (direction) ? t1 - t2 : t1 + t2;
		accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)nextStepTime - (int32_t)(((uint64_t)mp.delta.mmPerStepTimesCdivtopSpeedK * (uint32_t)dsK)/(K1 * K2));
	}
	else
	{
		// Force the linear motion phase
		mp.cart.decelStartStep = totalSteps + 1;
		mp.cart.accelStopStep = 0;

		// Adjust the speed
		mp.cart.mmPerStepTimesCdivtopSpeed = (uint32_t)(inverseSpeedFactor * mp.cart.mmPerStepTimesCdivtopSpeed);

		// Adjust the acceleration clocks to as to maintain continuity of movement
		accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)nextStepTime - (int32_t)(((uint64_t)mp.cart.mmPerStepTimesCdivtopSpeed * nextStep)/K1);
	}
}

// End
