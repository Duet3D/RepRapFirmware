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
		mp.delta.decelStartDsK = (uint32_t)(dda.totalDistance * stepsPerMm * K2) * 2;		// the *2 is to make sure it is definitely high enough
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
		debugPrintf("DM%c%s dir=%c steps=%u next=%u sstcda=%u "
					"acmadtcdts=%d tstcdapdsc=%u tstdca2=%" PRIu64 "\n",
					c, (stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, startSpeedTimesCdivA,
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
uint32_t DriveMovement::CalcNextStepTimeCartesian(size_t drive)
{
	if (nextStep >= totalSteps)
	{
		moving = false;
		return NoStepTime;
	}

	uint32_t lastStepTime = nextStepTime;			// pick up the time of the last step
	++nextStep;
	if (nextStep < mp.cart.accelStopStep)
	{
		nextStepTime = isqrt(isquare64(startSpeedTimesCdivA) + (mp.cart.twoCsquaredTimesMmPerStepDivA * nextStep)) - startSpeedTimesCdivA;
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
						? topSpeedTimesCdivAPlusDecelStartClocks - isqrt(twoDistanceToStopTimesCsquaredDivA - temp)
						: topSpeedTimesCdivAPlusDecelStartClocks;
	}
	else
	{
		if (nextStep == mp.cart.reverseStartStep)
		{
			reprap.GetPlatform()->SetDirection(drive, !direction);
		}
		nextStepTime = topSpeedTimesCdivAPlusDecelStartClocks
							+ isqrt((int64_t)(mp.cart.twoCsquaredTimesMmPerStepDivA * nextStep) - mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA);

	}

	if ((int32_t)nextStepTime < (int32_t)(lastStepTime + DDA::MinStepTime) && nextStep > 1)
	{
		stepError = true;
		return NoStepTime;
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

	uint32_t lastStepTime = nextStepTime;			// pick up the time of the last step
	++nextStep;
	if (nextStep == mp.delta.reverseStartStep)
	{
		direction = false;
		reprap.GetPlatform()->SetDirection(drive, false);		// going down now
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
	const int32_t t2 = isqrt(isquare64(t1) + mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - isquare64(mp.delta.hmz0sK));
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
		nextStepTime = isqrt(isquare64(startSpeedTimesCdivA) + ((uint64_t)mp.delta.twoCsquaredTimesMmPerStepDivAK * (uint32_t)dsK)) - startSpeedTimesCdivA;
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
						? topSpeedTimesCdivAPlusDecelStartClocks - isqrt(twoDistanceToStopTimesCsquaredDivA - temp)
						: topSpeedTimesCdivAPlusDecelStartClocks;
	}

	if ((int32_t)nextStepTime < (int32_t)(lastStepTime + DDA::MinStepTime) && nextStep > 1)
	{
		stepError = true;
//		debugPrintf("%u %u %u %d %d %d %d\n", nextStep, nextStepTime, lastStepTime, dsK, t1, t2, mp.delta.hmz0sK);
		return NoStepTime;
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
		const int32_t t2 = isqrt(isquare64(t1) + mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - isquare64(mp.delta.hmz0sK));
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

// Fast 64-bit integer square root function
/* static */ uint32_t DriveMovement::isqrt(uint64_t num)
{
//irqflags_t flags = cpu_irq_save();
//uint32_t t2 = Platform::GetInterruptClocks();
	uint32_t numHigh = (uint32_t)(num >> 32);
	if (numHigh != 0)
	{
		uint32_t resHigh = 0;

#define iter64a(N) 								\
		{										\
			uint32_t temp = resHigh + (1 << N);	\
			if (numHigh >= temp << N)			\
			{									\
				numHigh -= temp << N;			\
				resHigh |= 2 << N;				\
			}									\
		}

		// We need to do 16 iterations
		iter64a(15); iter64a(14); iter64a(13); iter64a(12);
		iter64a(11); iter64a(10); iter64a(9); iter64a(8);
		iter64a(7); iter64a(6); iter64a(5); iter64a(4);
		iter64a(3); iter64a(2); iter64a(1); iter64a(0);

		// resHigh is twice the square root of the msw, in the range 0..2^17-1
		uint64_t res = (uint64_t)resHigh << 16;
		uint64_t numAll = ((uint64_t)numHigh << 32) | (uint32_t)num;

#define iter64b(N) 								\
		{										\
			uint64_t temp = res | (1 << N);		\
			if (numAll >= temp << N)			\
			{									\
				numAll -= temp << N;			\
				res |= 2 << N;					\
			}									\
		}

		// We need to do 16 iterations.
		// After the last iteration, numAll may be between 0 and (1 + 2 * res) inclusive.
		// So to take square roots of numbers up to 64 bits, we need to do all these iterations using 64 bit maths.
		// If we restricted the input to e.g. 48 bits, then we could do some of the final iterations using 32-bit maths.
		iter64b(15); iter64b(14); iter64b(13); iter64b(12);
		iter64b(11); iter64b(10); iter64b(9); iter64b(8);
		iter64b(7); iter64b(6); iter64b(5); iter64b(4);
		iter64b(3); iter64b(2); iter64b(1); iter64b(0);

		uint32_t rslt = (uint32_t)(res >> 1);

//uint32_t t3 = Platform::GetInterruptClocks() - t2; if (t3 < minCalcTime) minCalcTime = t3; if (t3 > maxCalcTime) maxCalcTime = t3;
//cpu_irq_restore(flags);
//uint64_t num3 = (uint64_t)rslt * rslt; if (num3 > num || (num - num3) > 2*rslt) {++sqrtErrors; lastNum = num; lastRes = rslt; }
		return rslt;
	}
	else
	{
		// 32-bit square root
		uint32_t num32 = (uint32_t)num;
		uint32_t res32 = 0;

		// Thanks to Wilco Dijksra for this efficient ARM algorithm
#define iter32(N) 								\
		{										\
			uint32_t temp = res32 | (1 << N);	\
			if (num32 >= temp << N)				\
			{									\
				num32 -= temp << N;				\
				res32 |= 2 << N;				\
			}									\
		}

		// We need to do 16 iterations
		iter32(15); iter32(14); iter32(13); iter32(12);
		iter32(11); iter32(10); iter32(9); iter32(8);
		iter32(7); iter32(6); iter32(5); iter32(4);
		iter32(3); iter32(2); iter32(1); iter32(0);

		res32 >>= 1;

//uint32_t t3 = Platform::GetInterruptClocks() - t2; if (t3 < minCalcTime) minCalcTime = t3; if (t3 > maxCalcTime) maxCalcTime = t3;
//cpu_irq_restore(flags);
//uint64_t num3 = (uint64_t)res32 * res32; if (num3 > num || (num - num3) > 2*res32) {++sqrtErrors; lastNum = num; lastRes = res32; }
		return res32;
	}
}

// End
