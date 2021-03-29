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
	currentSegment = dda.segments;
	NewLinearSegment(0.0);

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	return CalcNextStepTime(dda);
}

// This is called when currentSegment has just been changed to a new segment
void DriveMovement::NewLinearSegment(float startDistance) noexcept
{
	const float distanceLimit = currentSegment->GetDistanceLimit();

	// Work out the movement limit in steps
	if (currentSegment->IsLast())
	{
		phaseStepLimit = totalSteps + 1;
		if (reverseStartDistance < distanceLimit && startDistance < reverseStartDistance)
		{
			if (nextStep == reverseStartStep)
			{
				direction = !direction;
				reverseInThisSegment = false;
				state = DMState::reverse;
			}
			else
			{
				reverseInThisSegment = true;
				state = DMState::forwards;
			}
		}
		else
		{
			reverseInThisSegment = false;
			state = DMState::forwards;
		}
	}
	else
	{
		const float thisPhaseStepLimit = (currentSegment->GetDistanceLimit() * totalSteps) + 1;
		if ((float)reverseStartStep <= thisPhaseStepLimit && nextStep <= reverseStartStep)
		{
			if (nextStep == reverseStartStep)
			{
				direction = !direction;
				reverseInThisSegment = false;
				state = DMState::reverse;
				phaseStepLimit = qq;
			}
			else
			{
				reverseInThisSegment = true;
				state = DMState::forwards;
				phaseStepLimit = qq;
			}
		}
		else
		{
			reverseInThisSegment = false;
			state = DMState::forwards;
			phaseStepLimit = thisPhaseStepLimit;
		}
	}
}

void DriveMovement::NewDeltaSegment(const DDA& dda, float startDistance) noexcept
{
	// Work out the movement limit in steps
	const float distanceLimit = currentSegment->GetDistanceLimit();
	const float sDx = distanceLimit * dda.directionVector[0];
	const float sDy = distanceLimit * dda.directionVector[1];
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float netStepsAtEnd = fastSqrtf(delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(stepsPerMm) * (sDx * (sDx + delta.fTwoA) + sDy * (sDy + delta.fTwoB)))
							 + (distanceLimit * dda.directionVector[2] - delta.h0MinusZ0) * stepsPerMm;
	if (direction)
	{
		// Going up
		if ((uint32_t)netStepsAtEnd > reverseStartStep)
		{
			// We reverse direction during this segment
			reverseInThisSegment = true;
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
		if (currentSegment->IsLast())
		{
			phaseStepLimit = totalSteps + 1;
		}
		else
		{
			phaseStepLimit = (uint32_t)(netStepsAtEnd);
			qq;
		}
	}
}

// Prepare this DM for a Delta axis move, returning true if there are steps to do
bool DriveMovement::PrepareDeltaAxis(const DDA& dda, const PrepParams& params) noexcept
{
	const float stepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float A = params.initialX - params.dparams->GetTowerX(drive);
	const float B = params.initialY - params.dparams->GetTowerY(drive);
	const float aAplusbB = A * dda.directionVector[X_AXIS] + B * dda.directionVector[Y_AXIS];
	const float dSquaredMinusAsquaredMinusBsquared = params.dparams->GetDiagonalSquared(drive) - fsquare(A) - fsquare(B);
	delta.h0MinusZ0 = fastSqrtf(dSquaredMinusAsquaredMinusBsquared);
#if DM_USE_FPU
	delta.fTwoA = 2.0 * A;
	delta.fTwoB = 2.0 * B;
	delta.fHmz0s = delta.h0MinusZ0 * stepsPerMm;
	delta.fMinusAaPlusBbTimesS = -(aAplusbB * stepsPerMm);
	delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared = dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm);
#else
	delta.hmz0sK = roundS32(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
	delta.minusAaPlusBbTimesKs = -roundS32(aAplusbB * stepsPerMm * DriveMovement::K2);
	delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared = roundS64(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));
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
			const int32_t numStepsUp = (int32_t)((hrev - delta.h0MinusZ0) * stepsPerMm);

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

	// Prepare for the first step
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	isDelta = true;
	return CalcNextStepTime(dda);
}

// Prepare this DM for an extruder move, returning true if there are steps to do
bool DriveMovement::PrepareExtruder(DDA& dda, const PrepParams& params, float& extrusionPending, float speedChange, bool doCompensation) noexcept
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

	float compensationTime;

	if (doCompensation && direction)
	{
		// Calculate the pressure advance parameters
		compensationTime = reprap.GetPlatform().GetPressureAdvance(extruder);

		// Calculate the net total extrusion to allow for compensation. It may be negative.
		extrusionRequired += (dda.endSpeed - dda.startSpeed) * compensationTime * dv;
	}
	else
	{
		compensationTime = 0.0;
	}

	const float rawStepsPerMm = reprap.GetPlatform().DriveStepsPerUnit(drive);
	const float effectiveStepsPerMm = fabsf(dv) * rawStepsPerMm;
	const int32_t netSteps = int32_t(extrusionRequired * rawStepsPerMm);
	extrusionPending = extrusionRequired - (float)netSteps/rawStepsPerMm;

	return PrepareExtruderCommon(dda, params, effectiveStepsPerMm, netSteps, compensationTime);
}

#if SUPPORT_REMOTE_COMMANDS

// Prepare this DM for an extruder move. The caller has already checked that pressure advance is enabled.
bool DriveMovement::PrepareRemoteExtruder(DDA& dda, const PrepParams& params) noexcept
{
	// Calculate the pressure advance parameters
	const float compensationTime = reprap.GetPlatform().EutGetRemotePressureAdvance(drive);

	// Recalculate the net total step count to allow for compensation. It may be negative.
	const float compensationDistance = (dda.endSpeed - dda.startSpeed) * compensationTime;
	const int32_t netSteps = (int32_t)((1.0 + compensationDistance) * totalSteps);

	const float effectiveStepsPerMm = 1.0/totalSteps;
	return PrepareExtruderCommon(dda, params, effectiveStepsPerMm, netSteps, compensationTime);
}

#endif

// Common code for preparing extruder moves generated local and those received via CAN
bool DriveMovement::PrepareExtruderCommon(DDA& dda, const PrepParams& params, float effectiveStepsPerMm, int32_t netSteps, float compensationTime) noexcept
{
	const float compensationClocks = compensationTime * StepTimer::StepClockRate;
	const float accelExtraDistance = compensationTime * dda.acceleration * params.accelTime;
	const float totalAccelDistance = params.accelDistance + accelExtraDistance;
	float actualTotalDistance = params.decelStartDistance + accelExtraDistance;			// this starts off being the decel start distance and ends up being the total distance

	// Deceleration phase
	MoveSegment *segs = nullptr;
	bool hasReversePhase;
	if (params.decelDistance > 0.0)
	{
		// If we are using pressure advance then there may be a reverse phase. The motion constants are the same for both forward and reverse phases.
		// If there is a reverse phase then the stored distance limit is the point at which we reverse. The reverse phase will the be executed until the step count is correct.
		const float decelCompensationSpeedReduction = dda.deceleration * compensationTime;
		float distanceLimit;
		reverseStartDistance = fsquare(dda.topSpeed - decelCompensationSpeedReduction)/(2.0 * dda.deceleration) + actualTotalDistance;
		hasReversePhase = dda.endSpeed < decelCompensationSpeedReduction;
		bool hasForwardPhase = dda.topSpeed > decelCompensationSpeedReduction;
		if (hasForwardPhase && hasReversePhase)
		{
			const float forwardDistance = fsquare(dda.topSpeed - decelCompensationSpeedReduction)/(2.0 * dda.deceleration);
			const float reverseDistance = fsquare(decelCompensationSpeedReduction - dda.endSpeed)/(2.0 * dda.deceleration);
			distanceLimit = actualTotalDistance + forwardDistance;
			actualTotalDistance = distanceLimit + reverseDistance;
		}
		else if (hasForwardPhase)
		{
			const float forwardDistance = (fsquare(decelCompensationSpeedReduction - dda.topSpeed) - fsquare(decelCompensationSpeedReduction - dda.endSpeed))/(2.0 * dda.deceleration);
			distanceLimit = actualTotalDistance + forwardDistance;
			actualTotalDistance = distanceLimit;
		}
		else if (hasReversePhase)
		{
			const float reverseDistance = (fsquare(decelCompensationSpeedReduction - dda.endSpeed) - fsquare(decelCompensationSpeedReduction - dda.topSpeed))/(2.0 * dda.deceleration);
			distanceLimit = actualTotalDistance;
			actualTotalDistance += reverseDistance;
		}

		const float uDivDminusP = (dda.topSpeed * StepTimer::StepClockRate)/dda.deceleration - compensationClocks;
		segs = MoveSegment::Allocate(nullptr);
		segs->SetNonLinear(actualTotalDistance/dda.totalDistance,
							reverseStartDistance,
								(2.0 * actualTotalDistance)/(dda.deceleration * StepTimer::StepClockRate),
								params.accelClocks + params.steadyClocks + uDivDminusP);
		segs->SetLast();
	}

	// Steady phase
	if (params.steadyClocks > 0.0)
	{
		const float ts = dda.topSpeed * StepTimer::StepClockRate;
		MoveSegment * const oldSegs = segs;
		segs = MoveSegment::Allocate(segs);
		segs->SetLinear((params.decelStartDistance + accelExtraDistance)/dda.totalDistance, dda.totalDistance/ts, totalAccelDistance);
		if (oldSegs == nullptr)
		{
			segs->SetLast();
		}
	}

	// Acceleration phase
	if (params.accelDistance > 0)
	{
		const float uDivAplusP = (dda.startSpeed/dda.acceleration + compensationTime) * StepTimer::StepClockRate;
		MoveSegment * const oldSegs = segs;
		segs = MoveSegment::Allocate(segs);
		segs->SetNonLinear(totalAccelDistance/dda.totalDistance,
							fsquare(uDivAplusP),
							(2.0 * dda.totalDistance)/(dda.acceleration * StepTimer::StepClockRateSquared),
							-uDivAplusP);
		if (oldSegs == nullptr)
		{
			segs->SetLast();
		}
	}

	// Calculate the total movement, adjusted for pressure advance
	if (hasReversePhase)
	{
		const uint32_t forwardSteps = (uint32_t)(reverseStartDistance * effectiveStepsPerMm);
		reverseStartStep = forwardSteps + 1;
		totalSteps = 2 * forwardSteps - netSteps;
	}
	else
	{
		totalSteps = netSteps;
		reverseStartStep = totalSteps + 1;
	}

	// Prepare for the first step
	currentSegment = segs;
	dda.AppendSegments(segs);
	nextStep = 0;
	nextStepTime = 0;
	stepInterval = 999999;							// initialise to a large value so that we will calculate the time for just one step
	stepsTillRecalc = 0;							// so that we don't skip the calculation
	state = (segs == nullptr) ? DMState::idle
			: (segs->IsLast() && reverseStartStep == 1) ? DMState::reversing
				: DMState::forwards;
	isDelta = false;
	return CalcNextStepTime(dda);
}

void DriveMovement::DebugPrint() const noexcept
{
	const char c = (drive < reprap.GetGCodes().GetTotalAxes()) ? reprap.GetGCodes().GetAxisLetters()[drive] : (char)('0' + LogicalDriveToExtruder(drive));
	if (state != DMState::idle)
	{
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32 "\n",
						c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval);
		if (isDelta)
		{
#if DM_USE_FPU
			debugPrintf("hmz0s=%.2f minusAaPlusBbTimesS=%.2f dSquaredMinusAsquaredMinusBsquared=%.2f\n",
							(double)delta.fHmz0s, (double)delta.fMinusAaPlusBbTimesS, (double)delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared);
#else
			debugPrintf("hmz0sK=%" PRIi32 " minusAaPlusBbTimesKs=%" PRIi32 " dSquaredMinusAsquaredMinusBsquared=%" PRId64 "\n",
							delta.hmz0sK, delta.minusAaPlusBbTimesKs, delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared);
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
	case DMState::forwards:	// acceleration phase
		{
			const uint32_t stepsToLimit = ((reverseInThisSegment) ? reverseStartStep : phaseStepLimit) - nextStep;
			if (stepsToLimit == 1)
			{
				// This is the last step in this phase
				state = (reverseInThisSegment) ? DMState::reversing : DMState::stopping;
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
			nextCalcStepTime = currentSegment->CalcForwardStepTime((float)nextCalcStep/(float)totalSteps);
		}
		break;

	case DMState::reversing:
		direction = !direction;
		directionChanged = true;
		state = DMState::reverse;
		// no break
	case DMState::reverse:	// reverse phase
		{
			const uint32_t stepsToLimit = phaseStepLimit - nextStep;
			if (stepsToLimit == 1)
			{
				// This is the last step in this phase
				state = DMState::stopping;
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
			nextCalcStepTime = currentSegment->CalcReverseStepTime((float)nextCalcStep/(float)totalSteps);
		}
		break;

	case DMState::stopping:
		if (currentSegment->IsLast())
		{
			return false;
		}
		else
		{
			const float startDistance = currentSegment->GetDistanceLimit();
			currentSegment = currentSegment->GetNext();
			NewLinearSegment(startDistance);
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
		delta.fHmz0s += steps;									// get new carriage height above Z in steps
#else
		int32_t shiftedK2 = (int32_t)(K2 << shiftFactor);
		if (!direction)
		{
			shiftedK2 = -shiftedK2;
		}
		delta.hmz0sK += shiftedK2;								// get K2 * (new carriage height above Z in steps)
#endif
	}

#if DM_USE_FPU
	const float hmz0sc = delta.fHmz0s * dda.directionVector[Z_AXIS];
	const float t1 = delta.fMinusAaPlusBbTimesS + hmz0sc;
	// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
	const float t2a = delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(delta.fHmz0s) + fsquare(t1);
	const float t2 = (t2a > 0.0) ? fastSqrtf(t2a) : 0.0;
	const float ds = (direction) ? t1 - t2 : t1 + t2;
#else
	// In the following, cKc is the Z movement fraction of the total move scaled by Kc
	const int32_t hmz0scK = (int32_t)(((int64_t)delta.hmz0sK * dda.afterPrepare.cKc)/Kc);
	const int32_t t1 = delta.minusAaPlusBbTimesKs + hmz0scK;
	// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
	const int64_t t2a = delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - (int64_t)isquare64(delta.hmz0sK) + (int64_t)isquare64(t1);
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
	switch (state)
	{
	case DMState::forwards:
#if DM_USE_FPU
		nextCalcStepTime = currentSegment->CalcForwardStepTime(ds);
#else
		nextCalcStepTime = currentSegment->CalcForwardStepTime(dsk);
#endif
		break;

	case DMState::reversing:
		direction = !direction;
		directionChanged = true;
		state = DMState::reverse;
		// no break
	case DMState::reverse:
#if DM_USE_FPU
		nextCalcStepTime = currentSegment->CalcReverseStepTime(ds);
#else
		nextCalcStepTime = currentSegment->CalcReverseStepTime(dsk);
#endif
		break;

	case DMState::stopping:
		if (currentSegment->IsLast())
		{
			return false;
		}
		else
		{
			const float startDistance = currentSegment->GetDistanceLimit();
			currentSegment = currentSegment->GetNext();
			NewDeltaSegment(dda, startDistance);
		}
		break;

	default:
		return false;
	}

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
