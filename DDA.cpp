/*
 * DDA.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "RepRapFirmware.h"


DDA::DDA(DDA* n) : state(empty), next(n), prev(nullptr)
{
	memset(ddm, 0, sizeof(ddm));	//DEBUG to clear stepError field
}

// Return the number of clocks this DDA still needs to execute.
// This could be slightly negative, if the move is overdue for completion.
int32_t DDA::GetTimeLeft() const
//pre(state == executing || state == frozen || state == completed)
{
	return (state == completed) ? 0
			: (state == executing) ? (int32_t)(moveStartTime + clocksNeeded - Platform::GetInterruptClocks())
			: (int32_t)clocksNeeded;
}

void DDA::DebugPrintVector(const char *name, const float *vec, size_t len) const
{
	debugPrintf("%s=", name);
	for (size_t i = 0; i < len; ++i)
	{
		debugPrintf("%c%f", ((i == 0) ? '[' : ' '), vec[i]);
	}
	debugPrintf("]");
}

void DDA::DebugPrint() const
{
	debugPrintf("DDA:");
	if (endCoordinatesValid)
	{
		DebugPrintVector(" end", endCoordinates, AXES);
	}

	debugPrintf(" d=%f", totalDistance);
	DebugPrintVector(" vec", directionVector, 5);
	debugPrintf(" a=%f reqv=%f topv=%f startv=%f endv=%f\n"
				"daccel=%f ddecel=%f fstep=%u\n",
				acceleration, requestedSpeed, topSpeed, startSpeed, endSpeed,
				accelDistance, decelDistance, firstStepTime);
//	reprap.GetPlatform()->GetLine()->Flush();
	ddm[0].DebugPrint('x', isDeltaMovement);
	ddm[1].DebugPrint('y', isDeltaMovement);
	ddm[2].DebugPrint('z', isDeltaMovement);
	ddm[3].DebugPrint('1', false);
	ddm[4].DebugPrint('2', false);
//	reprap.GetPlatform()->GetLine()->Flush();
}

// This is called by Move to initialize all DDAs
void DDA::Init()
{
	// Set the endpoints to zero, because Move asks for them.
	// They will be wrong if we are on a delta. We take care of that when we process the M665 command in config.g.
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		endPoint[drive] = 0;
		ddm[drive].stepError = false;
	}
	state = empty;
	endCoordinatesValid = false;
}

// Set up a real move. Return true if it represents real movement, else false.
bool DDA::Init(const float nextMove[], EndstopChecks ce, bool doMotorMapping, FilePosition fPos)
{
	// 1. Compute the new endpoints and the movement vector
	const int32_t *positionNow = prev->DriveCoordinates();
	if (doMotorMapping)
	{
		const Move *move = reprap.GetMove();
		move->MotorTransform(nextMove, endPoint);			// transform the axis coordinates if on a delta or CoreXY printer
		isDeltaMovement = move->IsDeltaMode()
							&& (endPoint[X_AXIS] != positionNow[X_AXIS] || endPoint[Y_AXIS] != positionNow[Y_AXIS] || endPoint[Z_AXIS] != positionNow[Z_AXIS]);
	}
	else
	{
		isDeltaMovement = false;
	}

	bool realMove = false, xyMoving = false;
	float accelerations[DRIVES];
	const float *normalAccelerations = reprap.GetPlatform()->Accelerations();
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		accelerations[drive] = normalAccelerations[drive];
		if (drive >= AXES || !doMotorMapping)
		{
			endPoint[drive] = Move::MotorEndPointToMachine(drive, nextMove[drive]);
		}

		int32_t delta;
		if (drive < AXES)
		{
			endCoordinates[drive] = nextMove[drive];					// this will be wrong if we are doing a special move
			delta = endPoint[drive] - positionNow[drive];
		}
		else
		{
			delta = endPoint[drive];
		}

		DriveMovement& dm = ddm[drive];
		if (drive < AXES && isDeltaMovement)
		{
			directionVector[drive] = nextMove[drive] - prev->GetEndCoordinate(drive, false);
			dm.moving = true;							// on a delta printer, if one tower moves then we assume they all do
		}
		else
		{
			directionVector[drive] = (float)delta/reprap.GetPlatform()->DriveStepsPerUnit(drive);
			dm.moving = (delta != 0);
		}

		if (dm.moving)
		{
			dm.totalSteps = labs(delta);				// for now this is the number of net steps, but gets adjusted later if there is a reverse in direction
			dm.direction = (delta >= 0);				// for now this is the direction of net movement, but gets adjusted later if it is a delta movement
			realMove = true;

			if (drive < Z_AXIS)
			{
				xyMoving = true;
			}

			if (drive >= AXES && xyMoving)
			{
				float compensationTime = reprap.GetPlatform()->GetElasticComp(drive);
				if (compensationTime > 0.0)
				{
					// Compensation causes instant velocity changes equal to acceleration * k, so we may need to limit the acceleration
					accelerations[drive] = min<float>(accelerations[drive], reprap.GetPlatform()->ConfiguredInstantDv(drive)/compensationTime);
				}
			}
		}
	}

	// 2. Throw it away if there's no real movement.
	if (!realMove)
	{
		return false;
	}

	// 3. Store some values
	endStopsToCheck = ce;
	filePos = fPos;
	// The end coordinates will be valid at the end of this move if it does not involve endstop checks and is not a special move on a delta printer
	endCoordinatesValid = (ce == 0) && (doMotorMapping || !reprap.GetMove()->IsDeltaMode());

	// 4. Normalise the direction vector and compute the amount of motion.
	// If there is any XYZ movement, then we normalise it so that the total XYZ movement has unit length.
	// This means that the user gets the feed rate that he asked for. It also makes the delta calculations simpler.
	if (xyMoving || ddm[Z_AXIS].moving)
	{
		totalDistance = Normalise(directionVector, DRIVES, AXES);
		if (isDeltaMovement)
		{
			// The following are only needed when doing delta movements. We could defer computing them until Prepare(), which would make simulation faster.
			a2plusb2 = fsquare(directionVector[X_AXIS]) + fsquare(directionVector[Y_AXIS]);
			cKc = (int32_t)(directionVector[Z_AXIS] * DriveMovement::Kc);

			const DeltaParameters& dparams = reprap.GetMove()->GetDeltaParams();
			const float initialX = prev->GetEndCoordinate(X_AXIS, false);
			const float initialY = prev->GetEndCoordinate(Y_AXIS, false);
			const float diagonalSquared = fsquare(dparams.GetDiagonal());
			const float a2b2D2 = a2plusb2 * diagonalSquared;

			for (size_t drive = 0; drive < AXES; ++drive)
			{
				const float A = initialX - dparams.GetTowerX(drive);
				const float B = initialY - dparams.GetTowerY(drive);
				const float stepsPerMm = reprap.GetPlatform()->DriveStepsPerUnit(drive);
				DriveMovement& dm = ddm[drive];
				const float aAplusbB = A * directionVector[X_AXIS] + B * directionVector[Y_AXIS];
				const float dSquaredMinusAsquaredMinusBsquared = diagonalSquared - fsquare(A) - fsquare(B);
				float h0MinusZ0 = sqrtf(dSquaredMinusAsquaredMinusBsquared);
				dm.mp.delta.hmz0sK = (int32_t)(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
				dm.mp.delta.minusAaPlusBbTimesKs = -(int32_t)(aAplusbB * stepsPerMm * DriveMovement::K2);
				dm.mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared =
						(int64_t)(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));

				// Calculate the distance at which we need to reverse direction.
				if (a2plusb2 <= 0.0)
				{
					// Pure Z movement. We can't use the main calculation because it divides by a2plusb2.
					dm.direction = (directionVector[Z_AXIS] >= 0.0);
					dm.mp.delta.reverseStartStep = dm.totalSteps + 1;
				}
				else
				{
					const float drev = ((directionVector[Z_AXIS] * sqrt(a2b2D2 - fsquare(A * directionVector[Y_AXIS] - B * directionVector[X_AXIS])))
										- aAplusbB)/a2plusb2;
					if (drev > 0.0 && drev < totalDistance)		// if the reversal point is within range
					{
						// Calculate how many steps we need to move up before reversing
						float hrev = directionVector[Z_AXIS] * drev + sqrt(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - a2plusb2 * fsquare(drev));
						int32_t numStepsUp = (int32_t)((hrev - h0MinusZ0) * stepsPerMm);

						// We may be almost at the peak height already, in which case we don't really have a reversal.
						// We must not set reverseStartStep to 1, because then we would set the direction when Prepare() calls CalcStepTime(), before the previous move finishes.
						if (numStepsUp < 1 || (dm.direction && (uint32_t)numStepsUp <= dm.totalSteps))
						{
							dm.mp.delta.reverseStartStep = dm.totalSteps + 1;
						}
						else
						{
							dm.mp.delta.reverseStartStep = (uint32_t)numStepsUp + 1;

							// Correct the initial direction and the total number of steps
							if (dm.direction)
							{
								// Net movement is up, so we will go up a bit and then down by a lesser amount
								dm.totalSteps = (2 * numStepsUp) - dm.totalSteps;
							}
							else
							{
								// Net movement is down, so we will go up first and then down by a greater amount
								dm.direction = true;
								dm.totalSteps = (2 * numStepsUp) + dm.totalSteps;
							}
						}
					}
					else
					{
						dm.mp.delta.reverseStartStep = dm.totalSteps + 1;
					}
				}
			}
		}
	}
	else
	{
		totalDistance = Normalise(directionVector, DRIVES, DRIVES);
	}

	// 5. Compute the maximum acceleration available and maximum top speed
	float normalisedDirectionVector[DRIVES];			// Used to hold a unit-length vector in the direction of motion
	memcpy(normalisedDirectionVector, directionVector, sizeof(normalisedDirectionVector));
	Absolute(normalisedDirectionVector, DRIVES);
	acceleration = VectorBoxIntersection(normalisedDirectionVector, accelerations, DRIVES);

	// Set the speed to the smaller of the requested and maximum speed.
	// Also enforce a minimum speed of 0.5mm/sec. We need a minimum speed to avoid overflow in the movement calculations.
	requestedSpeed = max<float>(0.5, min<float>(nextMove[DRIVES], VectorBoxIntersection(normalisedDirectionVector, reprap.GetPlatform()->MaxFeedrates(), DRIVES)));

	// On a Cartesian printer, it is OK to limit the X and Y speeds and accelerations independently, and in consequence to allow greater values
	// for diagonal moves. On a delta, this is not OK and any movement in the XY plane should be limited to the X/Y axis values, which we assume to be equal.
	if (isDeltaMovement)
	{
		const float xyFactor = sqrt(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[X_AXIS]));
		const float maxSpeed = reprap.GetPlatform()->MaxFeedrates()[X_AXIS];
		if (requestedSpeed * xyFactor > maxSpeed)
		{
			requestedSpeed = maxSpeed/xyFactor;
		}

		const float maxAcceleration = normalAccelerations[X_AXIS];
		if (acceleration * xyFactor > maxAcceleration)
		{
			acceleration = maxAcceleration/xyFactor;
		}
	}

	// 6. Calculate the provisional accelerate and decelerate distances and the top speed
	endSpeed = 0.0;					// until the next move asks us to adjust it

	if (prev->state != provisional)
	{
		// There is no previous move that we can adjust, so this move must start at zero speed.
		startSpeed = 0.0;
	}
	else
	{
		// Try to meld this move to the previous move to avoid stop/start
		// Assuming that this move ends with zero speed, calculate the maximum possible starting speed: u^2 = v^2 - 2as
		float maxStartSpeed = sqrtf(acceleration * totalDistance * 2.0);
		prev->targetNextSpeed = min<float>(maxStartSpeed, requestedSpeed);
		DoLookahead(prev);
		startSpeed = prev->targetNextSpeed;
	}

	RecalculateMove();
	state = provisional;
	return true;
}

float DDA::GetMotorPosition(size_t drive) const
{
	return Move::MotorEndpointToPosition(endPoint[drive], drive);
}

void DDA::DoLookahead(DDA *laDDA)
//pre(state == provisional)
{
//	if (reprap.Debug(moduleDda)) debugPrintf("Adjusting, %f\n", laDDA->targetNextSpeed);
	unsigned int laDepth = 0;
	bool goingUp = true;

	for(;;)					// this loop is used to nest lookahead without making recursive calls
	{
		bool recurse = false;
		if (goingUp)
		{
			// We have been asked to adjust the end speed of this move to targetStartSpeed
			if (laDDA->topSpeed == laDDA->requestedSpeed)
			{
				// This move already reaches its top speed, so just need to adjust the deceleration part
				laDDA->endSpeed = laDDA->requestedSpeed;
				laDDA->CalcNewSpeeds();
			}
			else if (laDDA->decelDistance == laDDA->totalDistance && laDDA->prev->state == provisional)
			{
				// This move doesn't reach its requested speed, so we may have to adjust the previous move as well to get optimum behaviour
				laDDA->endSpeed = laDDA->requestedSpeed;
				laDDA->CalcNewSpeeds();
				laDDA->prev->targetNextSpeed = min<float>(sqrtf((laDDA->endSpeed * laDDA->endSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance)), laDDA->requestedSpeed);
				recurse = true;
			}
			else
			{
				// This move doesn't reach its requested speed, but we can't adjust the previous one
				laDDA->endSpeed = min<float>(sqrtf((laDDA->startSpeed * laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance)), laDDA->requestedSpeed);
				laDDA->CalcNewSpeeds();
			}
		}
		else
		{
			laDDA->startSpeed = laDDA->prev->targetNextSpeed;
			float maxEndSpeed = sqrtf((laDDA->startSpeed * laDDA->startSpeed) + (2 * laDDA->acceleration * laDDA->totalDistance));
			if (maxEndSpeed < laDDA->endSpeed)
			{
				// Oh dear, we were too optimistic! Have another go.
				laDDA->endSpeed = maxEndSpeed;
				laDDA->CalcNewSpeeds();
			}
		}

		if (recurse)
		{
			laDDA = laDDA->prev;
			++laDepth;
			if (reprap.Debug(moduleDda)) debugPrintf("Recursion start %u\n", laDepth);
		}
		else
		{
			laDDA->RecalculateMove();

			if (laDepth == 0)
			{
//				if (reprap.Debug(moduleDda)) debugPrintf("Complete, %f\n", laDDA->targetNextSpeed);
				return;
			}

			laDDA = laDDA->next;
			--laDepth;
			goingUp = false;
		}
	}
}

// Recalculate the top speed, acceleration distance and deceleration distance, and whether we can pause after this move
void DDA::RecalculateMove()
{
	accelDistance = ((requestedSpeed * requestedSpeed) - (startSpeed * startSpeed))/(2.0 * acceleration);
	decelDistance = ((requestedSpeed * requestedSpeed) - (endSpeed * endSpeed))/(2.0 * acceleration);
	if (accelDistance + decelDistance >= totalDistance)
	{
		// It's an accelerate-decelerate move. If V is the peak speed, then (V^2 - u^2)/2a + (V^2 - v^2)/2a = distance.
		// So (2V^2 - u^2 - v^2)/2a = distance
		// So V^2 = a * distance + 0.5(u^2 + v^2)
		float vsquared = (acceleration * totalDistance) + 0.5 * ((startSpeed * startSpeed) + (endSpeed * endSpeed));
		// Calculate accelerate distance from: V^2 = u^2 + 2as
		if (vsquared >= 0.0)
		{
			accelDistance = max<float>((vsquared - (startSpeed * startSpeed))/(2.0 * acceleration), 0.0);
			decelDistance = totalDistance - accelDistance;
			topSpeed = sqrtf(vsquared);
		}
		else if (startSpeed < endSpeed)
		{
			// This would ideally never happen, but might because of rounding errors
			accelDistance = totalDistance;
			decelDistance = 0.0;
			topSpeed = endSpeed;
		}
		else
		{
			// This would ideally never happen, but might because of rounding errors
			accelDistance = 0.0;
			decelDistance = totalDistance;
			topSpeed = startSpeed;
		}
	}
	else
	{
		topSpeed = requestedSpeed;
	}

	canPause = (endStopsToCheck == 0);
	if (canPause && endSpeed != 0.0)
	{
		const Platform *p = reprap.GetPlatform();
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			if (ddm[drive].moving && endSpeed * fabs(directionVector[drive]) > p->ActualInstantDv(drive))
			{
				canPause = false;
				break;
			}
		}
	}
}

void DDA::CalcNewSpeeds()
{
	// Decide what speed we would really like to start at. There are several possibilities:
	// 1. If the top speed is already the requested speed, use the requested speed.
	// 2. Else if this is a deceleration-only move and the previous move is not frozen, we may be able to increase the start speed,
	//    so use the requested speed again.
	// 3. Else the start speed must be pinned, so use the lower of the maximum speed we can accelerate to and the requested speed.

	// We may have to make multiple passes, because reducing one of the speeds may solve some problems but actually make matters worse on another axis.
	bool limited;
	do
	{
//		debugPrintf("  Pass, start=%f end=%f\n", targetStartSpeed, endSpeed);
		limited = false;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			const float thisMoveFraction = directionVector[drive];
			const float nextMoveFraction = next->directionVector[drive];
			const DriveMovement& thisMoveDm = ddm[drive];
			const DriveMovement& nextMoveDm = next->ddm[drive];
			if (thisMoveDm.moving || nextMoveDm.moving)
			{
				float thisMoveSpeed = endSpeed * thisMoveFraction;
				float nextMoveSpeed = targetNextSpeed * nextMoveFraction;
				float idealDeltaV = fabsf(thisMoveSpeed - nextMoveSpeed);
				float maxDeltaV = reprap.GetPlatform()->ActualInstantDv(drive);
				if (idealDeltaV > maxDeltaV)
				{
					// This drive can't change speed fast enough, so reduce the start and/or end speeds
					// This algorithm sometimes converges very slowly, requiring many passes.
					// To ensure it converges at all, and to speed up convergence, we over-adjust the speed to achieve an even lower deltaV.
					maxDeltaV *= 0.8;
					if ((thisMoveFraction >= 0.0) == (nextMoveFraction >= 0.0))
					{
						// Drive moving in the same direction for this move and the next one, so we must reduce speed of the faster one
						if (fabsf(thisMoveSpeed) > fabsf(nextMoveSpeed))
						{
							endSpeed = (fabsf(nextMoveSpeed) + maxDeltaV)/fabsf(thisMoveFraction);
						}
						else
						{
							targetNextSpeed = (fabsf(thisMoveSpeed) + maxDeltaV)/fabsf(nextMoveFraction);
						}
					}
					else if (fabsf(thisMoveSpeed) * 2 < maxDeltaV)
					{
						targetNextSpeed = (maxDeltaV - fabsf(thisMoveSpeed))/fabsf(nextMoveFraction);
					}
					else if (fabsf(nextMoveSpeed) * 2 < maxDeltaV)
					{
						endSpeed = (maxDeltaV - fabsf(nextMoveSpeed))/fabsf(thisMoveFraction);
					}
					else
					{
						targetNextSpeed = maxDeltaV/(2 * fabsf(nextMoveFraction));
						endSpeed = maxDeltaV/(2 * fabsf(thisMoveFraction));
					}
					limited = true;
					// Most conflicts are between X and Y. So if we just did Y, start another pass immediately to save time.
					if (drive == 1)
					{
						break;
					}
				}
			}
		}
	} while (limited);
}

// This is called by Move::CurrentMoveCompleted to update the live coordinates from the move that has just finished
bool DDA::FetchEndPosition(volatile int32_t ep[DRIVES], volatile float endCoords[AXES])
{
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		ep[drive] = endPoint[drive];
	}
	if (endCoordinatesValid)
	{
		for (size_t axis = 0; axis < AXES; ++axis)
		{
			endCoords[axis] = endCoordinates[axis];
		}
	}
	return endCoordinatesValid;
}

void DDA::SetPositions(const float move[DRIVES])
{
	reprap.GetMove()->EndPointToMachine(move, endPoint, DRIVES);
	for (size_t axis = 0; axis < AXES; ++axis)
	{
		endCoordinates[axis] = move[axis];
	}
	endCoordinatesValid = true;
}

// Get a Cartesian end coordinate from this move
float DDA::GetEndCoordinate(size_t drive, bool disableDeltaMapping)
//pre(disableDeltaMapping || drive < AXES)
{
	if (disableDeltaMapping)
	{
		return Move::MotorEndpointToPosition(endPoint[drive], drive);
	}
	else
	{
		if (drive < AXES && !endCoordinatesValid)
		{
			reprap.GetMove()->MachineToEndPoint(endPoint, endCoordinates, AXES);
			endCoordinatesValid = true;
		}
		return endCoordinates[drive];
	}
}

// Calculate the time needed for this move. Called instead of Prepare when we are in simulation mode.
float DDA::CalcTime() const
{
	return (topSpeed - startSpeed)/acceleration								// acceleration time
			+ (totalDistance - accelDistance - decelDistance)/topSpeed		// steady speed time
			+ (topSpeed - endSpeed)/acceleration;
}

// Prepare this DDA for execution.
// This must not be called with interrupts disabled, because it calls Platform::EnableDrive.
void DDA::Prepare()
{
//debugPrintf("Prep\n");
//reprap.GetPlatform()->GetLine()->Flush();

	PrepParams params;
	params.decelStartDistance = totalDistance - decelDistance;

	// Convert the accelerate/decelerate distances to times
	const float accelStopTime = (topSpeed - startSpeed)/acceleration;
	const float decelStartTime = accelStopTime + (params.decelStartDistance - accelDistance)/topSpeed;
	const float totalTime = decelStartTime + (topSpeed - endSpeed)/acceleration;
	clocksNeeded = (uint32_t)(totalTime * stepClockRate);

	params.startSpeedTimesCdivA = (uint32_t)((startSpeed * stepClockRate)/acceleration);
	params.topSpeedTimesCdivA = (uint32_t)((topSpeed * stepClockRate)/acceleration);
	params.decelStartClocks = (uint32_t)(decelStartTime * stepClockRate);
	params.topSpeedTimesCdivAPlusDecelStartClocks = params.topSpeedTimesCdivA + params.decelStartClocks;
	params.accelClocksMinusAccelDistanceTimesCdivTopSpeed = (uint32_t)((accelStopTime - (accelDistance/topSpeed)) * stepClockRate);
	params.compFactor = 1.0 - startSpeed/topSpeed;

	firstStepTime = DriveMovement::NoStepTime;
	goingSlow = false;

	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		DriveMovement& dm = ddm[drive];
		if (dm.moving)
		{
			reprap.GetPlatform()->EnableDrive(drive);
			if (drive >= AXES)
			{
				dm.PrepareExtruder(*this, params, drive);

				// Check for sensible values, print them if they look dubious
				if (reprap.Debug(moduleDda)
					&& (   dm.totalSteps > 1000000
						|| dm.mp.cart.reverseStartStep < dm.mp.cart.decelStartStep
						|| (dm.mp.cart.reverseStartStep <= dm.totalSteps
								&& dm.mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA > (int64_t)(dm.mp.cart.twoCsquaredTimesMmPerStepDivA * dm.mp.cart.reverseStartStep))
					  )
				   )
				{
					DebugPrint();
					reprap.GetPlatform()->GetLine()->Flush();
				}
			}
			else if (isDeltaMovement)
			{
				dm.PrepareDeltaAxis(*this, params, drive);

				// Check for sensible values, print them if they look dubious
				if (reprap.Debug(moduleDda) && dm.totalSteps > 1000000)
				{
					DebugPrint();
					reprap.GetPlatform()->GetLine()->Flush();
				}
			}
			else
			{
				dm.PrepareCartesianAxis(*this, params, drive);

				// Check for sensible values, print them if they look dubious
				if (reprap.Debug(moduleDda) && dm.totalSteps > 1000000)
				{
					DebugPrint();
					reprap.GetPlatform()->GetLine()->Flush();
				}
			}

			// Prepare for the first step
			dm.nextStep = 0;
			dm.nextStepTime = 0;
			dm.stepError = false;							// clear any previous step error before we call CalcNextStep
			dm.stepsTillRecalc = 1;
			uint32_t st = (isDeltaMovement && drive < AXES)
							? dm.CalcNextStepTimeDelta(*this, drive)
							: dm.CalcNextStepTimeCartesian(drive);
			if (st < firstStepTime)
			{
				firstStepTime = st;
			}
		}
	}

	if (reprap.Debug(moduleDda) && reprap.Debug(moduleMove))	// temp show the prepared DDA if debug enabled for both modules
	{
		DebugPrint();
		reprap.GetPlatform()->GetLine()->Flush();
	}
//debugPrintf("Done\n");
//reprap.GetPlatform()->GetLine()->Flush();

	state = frozen;					// must do this last so that the ISR doesn't start executing it before we have finished setting it up
}

// The remaining functions are speed-critical, so use full optimisation
#pragma GCC optimize ("O3")

// Start executing the move, returning true if Step() needs to be called immediately. Must be called with interrupts disabled, to avoid a race condition.
bool DDA::Start(uint32_t tim)
//pre(state == frozen)
{
	moveStartTime = tim;
	state = executing;

	if (firstStepTime == DriveMovement::NoStepTime)
	{
		// No steps are pending. This should not happen!
		state = completed;
		return false;
	}
	else
	{
		unsigned int extrusions = 0, retractions = 0;		// bitmaps of extruding and retracting drives
		for (size_t i = 0; i < DRIVES; ++i)
		{
			DriveMovement& dm = ddm[i];
			if (dm.moving)
			{
				reprap.GetPlatform()->SetDirection(i, dm.direction);
				if (i >= AXES)
				{
					if (dm.direction == FORWARDS)
					{
						extrusions |= (1 << (i - AXES));
					}
					else
					{
						retractions |= (1 << (i - AXES));
					}
				}
			}
		}

		Platform *platform = reprap.GetPlatform();
		if (extrusions != 0 || retractions != 0)
		{
			const unsigned int prohibitedMovements = reprap.GetProhibitedExtruderMovements(extrusions, retractions);
			if (prohibitedMovements != 0)
			{
				for (size_t i = 0; i < DRIVES - AXES; ++i)
				{
					if (prohibitedMovements & (1 << i))
					{
						ddm[i + AXES].moving = false;
					}
				}
			}
			platform->ExtrudeOn();
		}
		else
		{
			platform->ExtrudeOff();
		}
		return platform->ScheduleInterrupt(firstStepTime + moveStartTime);
	}
}

extern uint32_t maxReps;

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again with the DDA of the next move, otherwise false.
bool DDA::Step()
{
	if (state != executing)
	{
		return false;
	}

	bool repeat;
	uint32_t numReps = 0;
	do
	{
		++numReps;
		if (numReps > maxReps)
		{
			maxReps = numReps;
		}
		uint32_t now = Platform::GetInterruptClocks() - moveStartTime;			// how long since the move started

		if ((endStopsToCheck & ZProbeActive) != 0)								// if the Z probe is enabled in this move
		{
			// Check whether the Z probe has been triggered. On a delta at least, this must be done separately from endstop checks,
			// because we have both a high endstop and a Z probe, and the Z motor is not the same thing as the Z axis.
			switch (reprap.GetPlatform()->GetZProbeResult())
			{
			case EndStopHit::lowHit:
				MoveAborted(now);												// set the state to completed and recalculate the endpoints
				reprap.GetMove()->ZProbeTriggered(this);
				break;

			case EndStopHit::lowNear:
				ReduceHomingSpeed();
				break;

			default:
				break;
			}
		}

		uint32_t nextInterruptTime = DriveMovement::NoStepTime;
		if (state != completed)
		{
			for (size_t drive = 0; drive < DRIVES; ++drive)
			{
				DriveMovement& dm = ddm[drive];
				if (dm.moving && !dm.stepError)
				{
					// Hit anything?
					if ((endStopsToCheck & (1 << drive)) != 0)
					{
						switch(reprap.GetPlatform()->Stopped(drive))
						{
						case EndStopHit::lowHit:
							endStopsToCheck &= ~(1 << drive);					// clear this check so that we can check for more
							if (endStopsToCheck == 0)							// if no more endstops to check
							{
								MoveAborted(now);
							}
							else
							{
								StopDrive(drive);
							}
							reprap.GetMove()->HitLowStop(drive, this);
							break;

						case EndStopHit::highHit:
							endStopsToCheck &= ~(1 << drive);					// clear this check so that we can check for more
							if (endStopsToCheck == 0)							// if no more endstops to check
							{
								MoveAborted(now);
							}
							else
							{
								StopDrive(drive);
							}
							reprap.GetMove()->HitHighStop(drive, this);
							break;

						case EndStopHit::lowNear:
							// Only reduce homing speed if there are no more axes to be homed.
							// This allows us to home X and Y simultaneously.
							if (endStopsToCheck == (1 << drive))
							{
								ReduceHomingSpeed();
							}
							break;

						default:
							break;
						}
					}

					uint32_t st0 = dm.nextStepTime;
					if (now + minInterruptInterval >= st0)
					{
						reprap.GetPlatform()->StepHigh(drive);
						uint32_t st1 = (isDeltaMovement && drive < AXES) ? dm.CalcNextStepTimeDelta(*this, drive) : dm.CalcNextStepTimeCartesian(drive);
						if (st1 < nextInterruptTime)
						{
							nextInterruptTime = st1;
						}
						reprap.GetPlatform()->StepLow(drive);
//uint32_t t3 = Platform::GetInterruptClocks() - t2;
//if (t3 > maxCalcTime) maxCalcTime = t3;
//if (t3 < minCalcTime) minCalcTime = t3;
					}
					else if (st0 < nextInterruptTime)
					{
						nextInterruptTime = st0;
					}
				}
			}
		}

		if (nextInterruptTime == DriveMovement::NoStepTime)
		{
			state = completed;
		}

		if (state == completed)
		{
			uint32_t finishTime = Platform::GetInterruptClocks();
			Move *move = reprap.GetMove();
			move->CurrentMoveCompleted();				// tell Move that the current move is complete
			return move->StartNextMove(finishTime);		// schedule the next move
		}
		repeat = reprap.GetPlatform()->ScheduleInterrupt(nextInterruptTime + moveStartTime);
	} while (repeat);
	return false;
}

// Stop a drive and re-calculate the corresponding endpoint
void DDA::StopDrive(size_t drive)
{
	DriveMovement& dm = ddm[drive];
	if (dm.moving)
	{
		int32_t stepsLeft = dm.totalSteps - dm.nextStep + 1;
		if (dm.direction)
		{
			endPoint[drive] -= stepsLeft;			// we were going forwards
		}
		else
		{
			endPoint[drive] += stepsLeft;			// we were going backwards
		}
		dm.moving = false;
		endCoordinatesValid = false;				// the XYZ position is no longer valid
	}
}

// This is called when we abort a move because we have hit an endstop.
// It adjusts the end points of the current move to account for how far through the move we got.
void DDA::MoveAborted(uint32_t clocksFromStart)
{
	for (size_t drive = 0; drive < AXES; ++drive)
	{
		StopDrive(drive);
	}
	state = completed;
}

// Reduce the speed of this move to the indicated speed.
// This is called from the ISR, so interrupts are disabled and nothing else can mess with us.
// As this is only called for homing moves and with very low speeds, we assume that we don't need acceleration or deceleration phases.
void DDA::ReduceHomingSpeed()
{
	if (!goingSlow)
	{
		goingSlow = true;
		const float factor = 3.0;				// the factor by which we are reducing the speed
		topSpeed /= factor;
		for (size_t drive = 0; drive < DRIVES; ++drive)
		{
			DriveMovement& dm = ddm[drive];
			if (dm.moving)
			{
				dm.ReduceSpeed(*this, factor);
			}
		}
	}
}

void DDA::PrintIfHasStepError()
{
	bool printed = false;
	for (size_t drive = 0; drive < DRIVES; ++drive)
	{
		if (ddm[drive].stepError)
		{
			if (!printed)
			{
				DebugPrint();
				printed = true;
			}
			ddm[drive].stepError = false;
		}
	}
}

// Take a unit positive-hyperquadrant vector, and return the factor needed to obtain
// length of the vector as projected to touch box[].
float DDA::VectorBoxIntersection(const float v[], const float box[], size_t dimensions)
{
	// Generate a vector length that is guaranteed to exceed the size of the box
	float biggerThanBoxDiagonal = 2.0*Magnitude(box, dimensions);
	float magnitude = biggerThanBoxDiagonal;
	for (size_t d = 0; d < dimensions; d++)
	{
		if (biggerThanBoxDiagonal*v[d] > box[d])
		{
			float a = box[d]/v[d];
			if (a < magnitude)
			{
				magnitude = a;
			}
		}
	}
	return magnitude;
}

// Normalise a vector with dim1 dimensions so that it is unit in the first dim2 dimensions, and also return its previous magnitude in dim2 dimensions
float DDA::Normalise(float v[], size_t dim1, size_t dim2)
{
	float magnitude = Magnitude(v, dim2);
	if (magnitude <= 0.0)
	{
		return 0.0;
	}
	Scale(v, 1.0/magnitude, dim1);
	return magnitude;
}

// Return the magnitude of a vector
float DDA::Magnitude(const float v[], size_t dimensions)
{
	float magnitude = 0.0;
	for (size_t d = 0; d < dimensions; d++)
	{
		magnitude += v[d]*v[d];
	}
	magnitude = sqrtf(magnitude);
	return magnitude;
}

// Multiply a vector by a scalar
void DDA::Scale(float v[], float scale, size_t dimensions)
{
	for(size_t d = 0; d < dimensions; d++)
	{
		v[d] = scale*v[d];
	}
}

// Move a vector into the positive hyperquadrant
void DDA::Absolute(float v[], size_t dimensions)
{
	for(size_t d = 0; d < dimensions; d++)
	{
		v[d] = fabsf(v[d]);
	}
}

// End
