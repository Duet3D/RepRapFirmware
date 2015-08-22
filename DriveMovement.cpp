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
	const float stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(drive) * fabs(dda.directionVector[drive]);
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
void DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params, size_t drive)
{
	const float stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(drive) * fabs(dda.directionVector[drive]);
	mp.cart.twoCsquaredTimesMmPerStepDivA = (uint64_t)(((float)DDA::stepClockRate * (float)DDA::stepClockRate)/(stepsPerMm * dda.acceleration)) * 2;

	// Calculate the elasticity compensation parameter (not needed for axis movements, but we do them anyway to keep the code simple)
	const float compensationTime = reprap.GetPlatform()->GetElasticComp(drive);
	uint32_t compensationClocks = (uint32_t)(compensationTime * DDA::stepClockRate);
	const float accelCompensationDistance = compensationTime * (dda.topSpeed - dda.startSpeed);
	const float accelCompensationSteps = accelCompensationDistance * stepsPerMm;

	// Calculate the net total step count to allow for compensation (may be negative)
	// Note that we add totalSteps in floating point mode, to round the number of steps down consistently
	int32_t netSteps = (int32_t)(((dda.endSpeed - dda.startSpeed) * compensationTime * stepsPerMm) + totalSteps);

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)((dda.accelDistance * stepsPerMm) + accelCompensationSteps) + 1;
	startSpeedTimesCdivA = params.startSpeedTimesCdivA + compensationClocks;

	// Constant speed phase parameters
	mp.cart.mmPerStepTimesCdivtopSpeed = (uint32_t)(((float)DDA::stepClockRate * K1)/(stepsPerMm * dda.topSpeed));
	accelClocksMinusAccelDistanceTimesCdivTopSpeed = (int32_t)params.accelClocksMinusAccelDistanceTimesCdivTopSpeed - (int32_t)(compensationClocks * params.compFactor);

	// Deceleration and reverse phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)
	{
		totalSteps = netSteps;
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
		const float reverseStartDistance = (initialDecelSpeed > 0.0) ? fsquare(initialDecelSpeed)/(2 * dda.acceleration) + params.decelStartDistance : params.decelStartDistance;

		// Reverse phase parameters
		if (reverseStartDistance >= dda.totalDistance)
		{
			// No reverse phase
			totalSteps = netSteps;
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
	if (moving || stepError)
	{
		debugPrintf("DM%c%s dir=%c steps=%u next=%u interval=%u sstcda=%u "
					"acmadtcdts=%d tstcdapdsc=%u 2dtstc2diva=%" PRIu64 "\n",
					c, (stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, stepInterval, startSpeedTimesCdivA,
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

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
// This is also used for extruders on delta machines
uint32_t DriveMovement::CalcNextStepTimeCartesian(const DDA &dda, size_t drive)
{
	if (nextStep >= totalSteps)
	{
		moving = false;
		return NoStepTime;
	}

	++nextStep;
	if (stepsTillRecalc > 1 && nextStep != totalSteps)
	{
		--stepsTillRecalc;
		nextStepTime += stepInterval;
	}
	else
	{
		uint32_t lastStepTime = nextStepTime;			// pick up the time of the last step
		if (nextStep < mp.cart.accelStopStep)
		{
			nextStepTime = isqrt64(isquare64(startSpeedTimesCdivA) + (mp.cart.twoCsquaredTimesMmPerStepDivA * nextStep)) - startSpeedTimesCdivA;
		}
		else if (nextStep < mp.cart.decelStartStep)
		{
			nextStepTime = (uint32_t)((int32_t)(((uint64_t)mp.cart.mmPerStepTimesCdivtopSpeed * nextStep)/K1) + accelClocksMinusAccelDistanceTimesCdivTopSpeed);
		}
		else if (nextStep < mp.cart.reverseStartStep)
		{
			uint64_t temp = mp.cart.twoCsquaredTimesMmPerStepDivA * nextStep;
			// Allow for possible rounding error when the end speed is zero or very small
			nextStepTime = (twoDistanceToStopTimesCsquaredDivA > temp)
							? topSpeedTimesCdivAPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivA - temp)
							: topSpeedTimesCdivAPlusDecelStartClocks;
		}
		else
		{
			if (nextStep == mp.cart.reverseStartStep)
			{
				reprap.GetPlatform()->SetDirection(drive, !direction);
			}
			nextStepTime = topSpeedTimesCdivAPlusDecelStartClocks
								+ isqrt64((int64_t)(mp.cart.twoCsquaredTimesMmPerStepDivA * nextStep) - mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA);

		}

		if (stepsTillRecalc == 1)
		{
			--stepsTillRecalc;			// we can't trust the interval
		}
		else
		{
			// Check for steps that are too fast, this normally indicates a problem with the calculation
			int32_t interval = (int32_t)nextStepTime - (int32_t)lastStepTime;
			if (interval < DDA::MinStepInterval)
			{
				stepError = true;
				if (reprap.Debug(moduleMove))
				{
					stepInterval = (uint32_t)interval;
					return NoStepTime;
				}
			}
			else if (interval < DDA::MinCalcInterval)
			{
				// If the step interval is very short, flag not to recalculate it next time
				stepInterval = (uint32_t)interval;
				stepsTillRecalc = DDA::MinCalcInterval/stepInterval + 1;
			}
		}
	}

	if (nextStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		if (nextStep == totalSteps)
		{
			nextStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any step except the last to be late
			stepError = true;
			if (reprap.Debug(moduleMove))
			{
				stepInterval = 10000000 + nextStepTime;		// so we can tell what happened in debug
				return NoStepTime;
			}
		}
	}
	return nextStepTime;
}

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
uint32_t DriveMovement::CalcNextStepTimeDelta(const DDA &dda, size_t drive)
{
	if (nextStep >= totalSteps)
	{
		moving = false;
		return NoStepTime;
	}

	++nextStep;
	if (stepsTillRecalc > 1 && nextStep != mp.delta.reverseStartStep && nextStep != totalSteps)
	{
		--stepsTillRecalc;
		nextStepTime += stepInterval;

		// We can avoid most of the calculation, but we still need to update mp.delta.hmz0sk
		if (direction)
		{
			mp.delta.hmz0sK += (int32_t)K2;
		}
		else
		{
			mp.delta.hmz0sK -= (int32_t)K2;
		}
	}
	else
	{
		uint32_t lastStepTime = nextStepTime;			// pick up the time of the last step
		if (nextStep == mp.delta.reverseStartStep)
		{
			direction = false;
			reprap.GetPlatform()->SetDirection(drive, false);		// going down now
			stepsTillRecalc = 1;						// we can't trust the interval at the inflexion point
		}

		// Calculate d*s*K as an integer, where d = distance the head has travelled, s = steps/mm for this drive, K = a power of 2 to reduce the rounding errors
		if (direction)
		{
			mp.delta.hmz0sK += (int32_t)K2;
		}
		else
		{
			mp.delta.hmz0sK -= (int32_t)K2;
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
			stepError = true;
			nextStep += 1000000;		// so that we can tell what happened in the debug print
			return NoStepTime;
		}
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

		if (stepsTillRecalc == 1)
		{
			--stepsTillRecalc;			// we can't trust the interval
		}
		else
		{
			// Check for steps that are too fast, this normally indicates a problem with the calculation
			int32_t interval = (int32_t)nextStepTime - (int32_t)lastStepTime;
			if (interval < DDA::MinStepInterval)
			{
				stepError = true;
				if (reprap.Debug(moduleMove))
				{
					stepInterval = (uint32_t)interval;
					return NoStepTime;
				}
			}
			else if (interval < DDA::MinCalcInterval)
			{
				// If the step interval is very short, flag not to recalculate it next time
				stepInterval = (uint32_t)interval;
				stepsTillRecalc = DDA::MinCalcInterval/stepInterval + 1;
			}
		}
	}

	if (nextStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		if (nextStep == totalSteps)
		{
			nextStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any step except the last to be late
			stepError = true;
			if (reprap.Debug(moduleMove))
			{
				stepInterval = 10000000 + nextStepTime;		// so we can tell what happened in debug
				return NoStepTime;
			}
		}
	}

	return nextStepTime;
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
