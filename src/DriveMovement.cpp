/*
 * DriveMovement.cpp
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#include "RepRapFirmware.h"

// Prepare this DM for a Cartesian axis move
void DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params, size_t drive)
{
	const float stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(drive) * fabs(reprap.GetMove()->MotorFactor(drive, dda.directionVector));
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
	mp.cart.reverseStartStep = totalSteps + 1;
	mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
}

// Prepare this DM for a Delta axis move
void DriveMovement::PrepareDeltaAxis(const DDA& dda, const PrepParams& params, size_t drive)
{
	const float stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(drive);
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
void DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params, size_t drive, bool doCompensation)
{
	const float dv = dda.directionVector[drive];
	const float stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(drive) * fabs(dv);
	mp.cart.twoCsquaredTimesMmPerStepDivA = (uint64_t)(((float)DDA::stepClockRate * (float)DDA::stepClockRate)/(stepsPerMm * dda.acceleration)) * 2;

	// Calculate the elasticity compensation parameter (not needed for axis movements, but we do them anyway to keep the code simple)
	const float compensationTime = (doCompensation && dv > 0.0) ? reprap.GetPlatform()->GetElasticComp(drive - AXES) : 0.0;
	uint32_t compensationClocks = (uint32_t)(compensationTime * DDA::stepClockRate);

	// Calculate the net total step count to allow for compensation. It may be negative.
	// Note that we add totalSteps in floating point mode, to round the number of steps down consistently
	int32_t netSteps = (int32_t)(((dda.endSpeed - dda.startSpeed) * compensationTime * stepsPerMm) + totalSteps);

	// Calculate the acceleration phase parameters
	const float accelCompensationDistance = compensationTime * (dda.topSpeed - dda.startSpeed);
	const float accelCompensationSteps = accelCompensationDistance * stepsPerMm;

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)((dda.accelDistance * stepsPerMm) + accelCompensationSteps) + 1;
	startSpeedTimesCdivA = params.startSpeedTimesCdivA + compensationClocks;

	// Constant speed phase parameters
	mp.cart.mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)DDA::stepClockRate * K1)/(stepsPerMm * dda.topSpeed));
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)params.accelClocksMinusAccelDistanceTimesCdivTopSpeed - (int32_t)(compensationClocks * params.compFactor);

	// Calculate the deceleration and reverse phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)		// if less than 1 deceleration step
	{
		totalSteps = (uint)max<int32_t>(netSteps, 0);
		mp.cart.decelStartStep = mp.cart.reverseStartStep = netSteps + 1;
		topSpeedTimesCdivAPlusDecelStartClocks = 0;
		mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		twoDistanceToStopTimesCsquaredDivA = 0;
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)((params.decelStartDistance * stepsPerMm) + accelCompensationSteps) + 1;
		const int32_t initialDecelSpeedTimesCdivA = (int32_t)params.topSpeedTimesCdivA - (int32_t)compensationClocks;	// signed because it may be negative and we square it
		const uint64_t initialDecelSpeedTimesCdivASquared = isquare64(initialDecelSpeedTimesCdivA);
		topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivAPlusDecelStartClocks - compensationClocks;
		twoDistanceToStopTimesCsquaredDivA =
			initialDecelSpeedTimesCdivASquared + (uint64_t)(((params.decelStartDistance + accelCompensationDistance) * (DDA::stepClockRateSquared * 2))/dda.acceleration);

		const float initialDecelSpeed = dda.topSpeed - dda.acceleration * compensationTime;
		const float reverseStartDistance = (initialDecelSpeed > 0.0)
												? fsquare(initialDecelSpeed)/(2 * dda.acceleration) + params.decelStartDistance
												: params.decelStartDistance;

		// Reverse phase parameters
		if (reverseStartDistance >= dda.totalDistance)
		{
			// No reverse phase
			totalSteps = (uint)max<int32_t>(netSteps, 0);
			mp.cart.reverseStartStep = netSteps + 1;
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
		}
		else
		{
			mp.cart.reverseStartStep = (initialDecelSpeed < 0.0)
									? mp.cart.decelStartStep
									: (twoDistanceToStopTimesCsquaredDivA/mp.cart.twoCsquaredTimesMmPerStepDivA) + 1;
			// Because the step numbers are rounded down, we may sometimes get a situation in which netSteps = 1 and reverseStartStep = 1.
			// This would lead to totalSteps = -1, which must be avoided.
			int32_t overallSteps = (int32_t)(2 * (mp.cart.reverseStartStep - 1)) - netSteps;
			if (overallSteps > 0)
			{
				totalSteps = overallSteps;
				mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA =
						(int64_t)((2 * (mp.cart.reverseStartStep - 1)) * mp.cart.twoCsquaredTimesMmPerStepDivA) - (int64_t)twoDistanceToStopTimesCsquaredDivA;
			}
			else
			{
				totalSteps = (uint)max<int32_t>(netSteps, 0);
				mp.cart.reverseStartStep = totalSteps + 1;
				mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA = 0;
			}
		}
	}
}

void DriveMovement::DebugPrint(char c, bool isDeltaMovement) const
{
	if (state != DMState::idle)
	{
		debugPrintf("DM%c%s dir=%c steps=%u next=%u interval=%u sstcda=%u "
					"acmadtcdts=%d tstcdapdsc=%u 2dtstc2diva=%" PRIu64 "\n",
					c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, stepInterval, startSpeedTimesCdivA,
					accelClocksMinusAccelDistanceTimesCdivTopSpeed, topSpeedTimesCdivAPlusDecelStartClocks, twoDistanceToStopTimesCsquaredDivA);

		if (isDeltaMovement)
		{
			debugPrintf("revss=%d hmz0sK=%d minusAaPlusBbTimesKs=%d dSquaredMinusAsquaredMinusBsquared=%" PRId64 "\n"
						"2c2mmsdak=%u asdsk=%u dsdsk=%u mmstcdts=%u\n",
						mp.delta.reverseStartStep, mp.delta.hmz0sK, mp.delta.minusAaPlusBbTimesKs, mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared,
						mp.delta.twoCsquaredTimesMmPerStepDivAK, mp.delta.accelStopDsK, mp.delta.decelStartDsK, mp.delta.mmPerStepTimesCdivtopSpeedK
						);
		}
		else
		{
			debugPrintf("accelStopStep=%u decelStartStep=%u revStartStep=%u nextStep=%u nextStepTime=%u 2CsqtMmPerStepDivA=%" PRIu64 "\n",
						mp.cart.accelStopStep, mp.cart.decelStartStep, mp.cart.reverseStartStep, nextStep, nextStepTime, mp.cart.twoCsquaredTimesMmPerStepDivA
						);
			debugPrintf(" mmPerStepTimesCdivtopSpeed=%u fmsdmtstdca2=%" PRId64 "\n",
						mp.cart.mmPerStepTimesCdivtopSpeed, mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA
						);
		}
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// The remaining functions are speed-critical, so use full optimisation
#pragma GCC optimize ("O3")

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do.
// This is also used for extruders on delta machines.
bool DriveMovement::CalcNextStepTimeCartesian(const DDA &dda, size_t drive, bool live)
{
	if (nextStep >= totalSteps)
	{
		state = DMState::idle;
		return false;
	}

	++nextStep;
	if (stepsTillRecalc != 0)
	{
		--stepsTillRecalc;								// doing double/quad/octal stepping
	}
	else
	{
		// Work out how many steps to calculate at a time.
		uint32_t shiftFactor;
		if (stepInterval < DDA::MinCalcInterval)
		{
			uint32_t stepsToLimit = ((nextStep <= mp.cart.reverseStartStep && mp.cart.reverseStartStep <= totalSteps)
										? mp.cart.reverseStartStep
										: totalSteps
									) - nextStep;
			if (stepInterval < DDA::MinCalcInterval/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;		// octal stepping
			}
			else if (stepInterval < DDA::MinCalcInterval/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;		// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;		// double stepping
			}
			else
			{
				shiftFactor = 0;		// single stepping
			}
		}
		else
		{
			shiftFactor = 0;			// single stepping
		}
		stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate

		uint32_t nextCalcStep = nextStep + stepsTillRecalc;
		uint32_t lastStepTime = nextStepTime;			// pick up the time of the last step
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
		else if (nextCalcStep < mp.cart.reverseStartStep)
		{
			// deceleration phase, not reversed yet
			uint64_t temp = mp.cart.twoCsquaredTimesMmPerStepDivA * nextCalcStep;
			// Allow for possible rounding error when the end speed is zero or very small
			nextStepTime = (twoDistanceToStopTimesCsquaredDivA > temp)
							? topSpeedTimesCdivAPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivA - temp)
							: topSpeedTimesCdivAPlusDecelStartClocks;
		}
		else
		{
			// deceleration phase, reversing or already reversed
			if (nextCalcStep == mp.cart.reverseStartStep)
			{
				direction = !direction;
				if (live)
				{
					reprap.GetPlatform()->SetDirection(drive, direction);
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
			// Very rarely on a delta, the penultimate step may be calculated late. Allow for that here in case it affects Cartesian axes too.
			if (nextStep == totalSteps || nextStep + 1 == totalSteps)
			{
				nextStepTime = dda.clocksNeeded;
			}
			else
			{
				// We don't expect any step except the last to be late
				state = DMState::stepError;
				if (reprap.Debug(moduleMove))
				{
					stepInterval = 10000000 + nextStepTime;				// so we can tell what happened in debug
					return false;
				}
			}
		}
	}
	return true;
}

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
bool DriveMovement::CalcNextStepTimeDelta(const DDA &dda, size_t drive, bool live)
{
	if (nextStep >= totalSteps)
	{
		state = DMState::idle;
		return false;
	}

	++nextStep;
	if (stepsTillRecalc != 0)
	{
		--stepsTillRecalc;			// we are doing double or quad stepping
	}
	else
	{
		// Work out how many steps to calculate at a time.
		// The simulator suggests that at 200steps/mm, the minimum step pulse interval for 400mm/sec movement is 4.5us
		uint32_t shiftFactor;
		if (stepInterval < DDA::MinCalcInterval)
		{
			uint32_t stepsToLimit = ((nextStep <= mp.delta.reverseStartStep && mp.delta.reverseStartStep <= totalSteps)
										? mp.delta.reverseStartStep
										: totalSteps
									) - nextStep;
			if (stepInterval < DDA::MinCalcInterval/8 && stepsToLimit > 16)
			{
				shiftFactor = 4;		// octal stepping
			}
			else if (stepInterval < DDA::MinCalcInterval/4 && stepsToLimit > 8)
			{
				shiftFactor = 3;		// octal stepping
			}
			else if (stepInterval < DDA::MinCalcInterval/2 && stepsToLimit > 4)
			{
				shiftFactor = 2;		// quad stepping
			}
			else if (stepsToLimit > 2)
			{
				shiftFactor = 1;		// double stepping
			}
			else
			{
				shiftFactor = 0;		// single stepping
			}
		}
		else
		{
			shiftFactor = 0;			// single stepping
		}
		stepsTillRecalc = (1u << shiftFactor) - 1;					// store number of additional steps to generate

		if (nextStep == mp.delta.reverseStartStep)
		{
			direction = false;
			if (live)
			{
				reprap.GetPlatform()->SetDirection(drive, false);	// going down now
			}
		}

		// Calculate d*s*K as an integer, where d = distance the head has travelled, s = steps/mm for this drive, K = a power of 2 to reduce the rounding errors
		if (direction)
		{
			mp.delta.hmz0sK += (int32_t)(K2 << shiftFactor);
		}
		else
		{
			mp.delta.hmz0sK -= (int32_t)(K2 << shiftFactor);
		}

		const int32_t hmz0scK = (int32_t)(((int64_t)mp.delta.hmz0sK * dda.cKc)/Kc);
		const int32_t t1 = mp.delta.minusAaPlusBbTimesKs + hmz0scK;
		// Due to rounding error we can end up trying to take the square root of a negative number
		const int64_t t2a = (int64_t)isquare64(t1) + mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - (int64_t)isquare64(mp.delta.hmz0sK);
		const int32_t t2 = (t2a > 0) ? isqrt64(t2a) : 0;
		const int32_t dsK = (direction) ? t1 - t2 : t1 + t2;

		// Now feed dsK into a modified version of the step algorithm for Cartesian motion without elasticity compensation
		if (dsK < 0)
		{
			state = DMState::stepError;
			nextStep += 1000000;						// so that we can tell what happened in the debug print
			return false;
		}

		uint32_t lastStepTime = nextStepTime;			// pick up the time of the last step
		if ((uint32_t)dsK < mp.delta.accelStopDsK)
		{
			nextStepTime = isqrt64(isquare64(startSpeedTimesCdivA) + ((uint64_t)mp.delta.twoCsquaredTimesMmPerStepDivAK * (uint32_t)dsK)) - startSpeedTimesCdivA;
		}
		else if ((uint32_t)dsK < mp.delta.decelStartDsK)
		{
			nextStepTime = (uint32_t)((int32_t)(((uint64_t)mp.delta.mmPerStepTimesCdivtopSpeedK * (uint32_t)dsK)/(K1 * K2)) + accelClocksMinusAccelDistanceTimesCdivTopSpeed);
		}
		else
		{
			uint64_t temp = (uint64_t)mp.delta.twoCsquaredTimesMmPerStepDivAK * (uint32_t)dsK;
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
			// Very rarely, the penultimate step may be calculated late too.
			if (nextStep == totalSteps || nextStep + 1 == totalSteps)
			{
				nextStepTime = dda.clocksNeeded;
			}
			else
			{
				// We don't expect any step except the last to be late
				state = DMState::stepError;
				if (reprap.Debug(moduleMove))
				{
					stepInterval = 10000000 + nextStepTime;		// so we can tell what happened in debug
					return false;
				}
			}
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
