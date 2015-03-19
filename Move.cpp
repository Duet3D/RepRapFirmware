/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "RepRapFirmware.h"

void DeltaParameters::Init()
{
    deltaMode = false;
	diagonal = 0.0;
	radius = 0.0;
	printRadius = defaultPrintRadius;
	homedHeight = defaultDeltaHomedHeight;
	isEquilateral = true;

    for (size_t axis = 0; axis < AXES; ++axis)
    {
    	endstopAdjustments[axis] = 0.0;
    	towerX[axis] = towerY[axis] = 0.0;
    }
}

void DeltaParameters::SetRadius(float r)
{
	radius = r;
	isEquilateral = true;

	const float cos30 = sqrtf(3.0)/2.0;
	const float sin30 = 0.5;

	towerX[A_AXIS] = -(r * cos30);
	towerX[B_AXIS] = r * cos30;
	towerX[C_AXIS] = 0.0;

	towerY[A_AXIS] = towerY[B_AXIS] = -(r * sin30);
	towerY[C_AXIS] = r;

	Recalc();
}

void DeltaParameters::Recalc()
{
	deltaMode = (radius > 0.0 && diagonal > radius);
	if (deltaMode)
	{
		homedCarriageHeight = homedHeight + sqrtf(fsquare(diagonal) - fsquare(radius));
		Xbc = towerX[C_AXIS] - towerX[B_AXIS];
		Xca = towerX[A_AXIS] - towerX[C_AXIS];
		Xab = towerX[B_AXIS] - towerX[A_AXIS];
		Ybc = towerY[C_AXIS] - towerY[B_AXIS];
		Yca = towerY[A_AXIS] - towerY[C_AXIS];
		Yab = towerY[B_AXIS] - towerY[A_AXIS];
		coreFa = fsquare(towerX[A_AXIS]) + fsquare(towerY[A_AXIS]);
		coreFb = fsquare(towerX[B_AXIS]) + fsquare(towerY[B_AXIS]);
		coreFc = fsquare(towerX[C_AXIS]) + fsquare(towerY[C_AXIS]);
		Q = 2 * (Xca * Yab - Xab * Yca);
		Q2 = fsquare(Q);
	}
}

// Calculate the motor position for a single tower from a Cartesian coordinate
float DeltaParameters::Transform(const float machinePos[AXES], size_t axis) const
{
	return machinePos[Z_AXIS]
	          + sqrt(fsquare(diagonal) - fsquare(machinePos[X_AXIS] - towerX[axis]) - fsquare(machinePos[Y_AXIS] - towerY[axis]));
}

void DeltaParameters::InverseTransform(float Ha, float Hb, float Hc, float machinePos[]) const
{
	const float Fa = coreFa + fsquare(Ha);
	const float Fb = coreFb + fsquare(Hb);
	const float Fc = coreFc + fsquare(Hc);

//	debugPrintf("Ha=%f Hb=%f Hc=%f Fa=%f Fb=%f Fc=%f Xbc=%f Xca=%f Xab=%f Ybc=%f Yca=%f Yab=%f\n",
//				Ha, Hb, Hc, Fa, Fb, Fc, Xbc, Xca, Xab, Ybc, Yca, Yab);

	// Setup PQRSU such that x = -(S - uz)/P, y = (P - Rz)/Q
	const float P = (Xbc * Fa) + (Xca * Fb) + (Xab * Fc);
	const float S = (Ybc * Fa) + (Yca * Fb) + (Yab * Fc);

	const float R = 2 * ((Xbc * Ha) + (Xca * Hb) + (Xab * Hc));
	const float U = 2 * ((Ybc * Ha) + (Yca * Hb) + (Yab * Hc));

//	debugPrintf("P= %f R=%f S=%f U=%f Q=%f\n", P, R, S, U, Q);

	const float R2 = fsquare(R), U2 = fsquare(U);

	float A = U2 + R2 + Q2;
	float minusHalfB = S * U + P * R + Ha * Q2 + towerX[A_AXIS] * U * Q - towerY[A_AXIS] * R * Q;
	float C = fsquare(S + towerX[A_AXIS] * Q) + fsquare(P - towerY[A_AXIS] * Q) + (fsquare(Ha) - fsquare(diagonal)) * Q2;

//	debugPrintf("A=%f minusHalfB=%f C=%f\n", A, minusHalfB, C);

	float z = (minusHalfB - sqrtf(fsquare(minusHalfB) - A * C)) / A;
	machinePos[X_AXIS] = (U * z - S) / Q;
	machinePos[Y_AXIS] = (P - R * z) / Q;
	machinePos[Z_AXIS] = z;
}

// Compute the derivative of height with respect to a parameter
float DeltaParameters::ComputeDerivative(unsigned int deriv, const float machinePos[AXES])
{
	const float perturb = 0.2;			// perturbation amount in mm
	DeltaParameters hiParams(*this), loParams(*this);
	switch(deriv)
	{
	case 0:
	case 1:
	case 2:
		break;

	case 3:
	case 4:
		hiParams.towerX[deriv - 3] += perturb;
		loParams.towerX[deriv - 3] -= perturb;
		break;

	case 5:
		hiParams.towerY[C_AXIS] += perturb;
		loParams.towerY[C_AXIS] -= perturb;
		break;
	}

	hiParams.Recalc();
	loParams.Recalc();

	float ha = Transform(machinePos, A_AXIS);
	float hb = Transform(machinePos, B_AXIS);
	float hc = Transform(machinePos, C_AXIS);

	float newPos[AXES];
	hiParams.InverseTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
	float zHi = newPos[Z_AXIS];
	loParams.InverseTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
	float zLo = newPos[Z_AXIS];

	return (zHi - zLo)/(2 * perturb);
}

// Perform 6-factor adjustment
void DeltaParameters::SixFactorAdjust(float ea, float eb, float ec, float xa, float xb, float yc)
{
	const float eav = (ea + eb + ec)/3;
	endstopAdjustments[A_AXIS] += ea - eav;
	endstopAdjustments[B_AXIS] += eb - eav;
	endstopAdjustments[C_AXIS] += ec - eav;
	homedHeight += eav;
	towerX[A_AXIS] += xa;
	towerX[B_AXIS] += xb;
	towerY[C_AXIS] += yc;
	isEquilateral = false;
	Recalc();
}

void DeltaParameters::PrintParameters(StringRef& reply, bool full)
{
	reply.printf("Endstops X%.2f Y%.2f Z%.2f, height %.2f, ", endstopAdjustments[A_AXIS], endstopAdjustments[B_AXIS], endstopAdjustments[C_AXIS], homedHeight);
	if (isEquilateral && !full)
	{
		reply.catf("radius %.2f\n", radius);
	}
	else
	{
		reply.catf("towers (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)\n",
						towerX[A_AXIS], towerY[A_AXIS], towerX[B_AXIS], towerY[B_AXIS], towerX[C_AXIS], towerY[C_AXIS]);
	}
}


Move::Move(Platform* p, GCodes* g) : currentDda(NULL)
{
	active = false;

	// Build the DDA ring
	DDA *dda = new DDA(NULL);
	ddaRingGetPointer = ddaRingAddPointer = dda;
	for (size_t i = 1; i < DdaRingLength; i++)
	{
		DDA *oldDda = dda;
		dda = new DDA(dda);
		oldDda->SetPrevious(dda);
	}
	ddaRingAddPointer->SetNext(dda);
	dda->SetPrevious(ddaRingAddPointer);
}

void Move::Init()
{
	// Reset Cartesian mode
	deltaParams.Init();
	coreXYMode = 0;

	// Empty the ring
	ddaRingGetPointer = ddaRingAddPointer;
	DDA *dda = ddaRingAddPointer;
	do
	{
		dda->Init();
		dda = dda->GetNext();
	} while (dda != ddaRingAddPointer);

	currentDda = nullptr;
	addNoMoreMoves = false;

	// Clear the transforms
	SetIdentityTransform();
	tanXY = tanYZ = tanXZ = 0.0;

	// Put the origin on the lookahead ring with default velocity in the previous position to the first one that will be used.
	// Do this by calling SetLiveCoordinates and SetPositions, so that the motor coordinates will be correct too even on a delta.
	float move[DRIVES];
	for (size_t i = 0; i < DRIVES; i++)
	{
		move[i] = 0.0;
		reprap.GetPlatform()->SetDirection(i, FORWARDS);		// DC: I don't see any reason why we do this
	}
	SetLiveCoordinates(move);
	SetPositions(move);

	size_t slow = reprap.GetPlatform()->SlowestDrive();
	currentFeedrate = reprap.GetPlatform()->HomeFeedRate(slow);

	// Set up default bed probe points. This is only a guess, because we don't know the bed size yet.
	for (size_t point = 0; point < NUMBER_OF_PROBE_POINTS; point++)
	{
		xBedProbePoints[point] = (0.3 + 0.6*(float)(point%2))*reprap.GetPlatform()->AxisMaximum(X_AXIS);
		yBedProbePoints[point] = (0.0 + 0.9*(float)(point/2))*reprap.GetPlatform()->AxisMaximum(Y_AXIS);
		zBedProbePoints[point] = 0.0;
		probePointSet[point] = unset;
	}

	xRectangle = 1.0/(0.8*reprap.GetPlatform()->AxisMaximum(X_AXIS));
	yRectangle = xRectangle;

	longWait = reprap.GetPlatform()->Time();
	idleTimeout = defaultIdleTimeout;
	iState = IdleState::idle;
	idleCount = 0;

	simulating = false;
	simulationTime = 0.0;

	active = true;
}

void Move::Exit()
{
	reprap.GetPlatform()->Message(BOTH_MESSAGE, "Move class exited.\n");
	active = false;
}

void Move::Spin()
{
	if (!active)
	{
		return;
	}

	if (idleCount < 1000)
	{
		++idleCount;
	}

	// See if we can add another move to the ring
	if (!addNoMoreMoves && ddaRingAddPointer->GetState() == DDA::empty)
	{
		DDA *dda = ddaRingAddPointer;
		if (reprap.Debug(moduleMove))
		{
			dda->PrintIfHasStepError();
		}

		// In order to react faster to speed and extrusion rate changes, only add more moves if the total duration of
		// all un-frozen moves is less than 2 seconds, or the total duration of all but the first un-frozen move is
		// less than 0.5 seconds.
		float unPreparedTime = 0.0;
		float prevMoveTime = 0.0;
		for(;;)
		{
			dda = dda->GetPrevious();
			if (dda->GetState() != DDA::provisional)
			{
				break;
			}
			unPreparedTime += prevMoveTime;
			prevMoveTime = dda->CalcTime();
		}

		if (unPreparedTime < 0.5 || unPreparedTime + prevMoveTime < 2.0)
		{
			// If there's a G Code move available, add it to the DDA ring for processing.
			float nextMove[DRIVES + 1];
			EndstopChecks endStopsToCheck;
			uint8_t moveType;
			FilePosition filePos;
			if (reprap.GetGCodes()->ReadMove(nextMove, endStopsToCheck, moveType, filePos))
			{
				currentFeedrate = nextMove[DRIVES];		// might be G1 with just an F field
				bool doMotorMapping = (moveType == 0) || (moveType == 1 && !IsDeltaMode());
				if (doMotorMapping)
				{
					Transform(nextMove);
				}
				if (ddaRingAddPointer->Init(nextMove, endStopsToCheck, doMotorMapping, filePos))
				{
					ddaRingAddPointer = ddaRingAddPointer->GetNext();
					idleCount = 0;
				}
			}
		}
	}

	if (simulating)
	{
		if (idleCount > 10 && !DDARingEmpty())
		{
			// No move added this time, so simulate executing one already in the queue
			DDA *dda = ddaRingGetPointer;
			simulationTime += dda->CalcTime();
			liveCoordinatesValid = dda->FetchEndPosition(const_cast<int32_t*>(liveEndPoints), const_cast<float *>(liveCoordinates));
			dda->Release();
			ddaRingGetPointer = ddaRingGetPointer->GetNext();
		}
	}
	else
	{
		// See whether we need to kick off a move
		DDA *cdda = currentDda;											// currentDda is volatile, so copy it
		if (cdda == nullptr)
		{
			// No DDA is executing, so start executing a new one if possible
			if (idleCount > 10)											// better to have a few moves in the queue so that we can do lookahead
			{
				DDA *dda = ddaRingGetPointer;
				if (dda->GetState() == DDA::provisional)
				{
					dda->Prepare();
				}
				if (dda->GetState() == DDA::frozen)
				{
					cpu_irq_disable();										// must call StartNextMove and Interrupt with interrupts disabled
					if (StartNextMove(Platform::GetInterruptClocks()))		// start the next move if none is executing already
					{
						Interrupt();
					}
					cpu_irq_enable();
					iState = IdleState::busy;
				}
				else if (iState == IdleState::busy && !reprap.GetGCodes()->IsPaused() && idleTimeout > 0.0)
				{
					lastMoveTime = reprap.GetPlatform()->Time();			// record when we first noticed that the machine was idle
					iState = IdleState::timing;
				}
				else if (iState == IdleState::timing && reprap.GetPlatform()->Time() - lastMoveTime >= idleTimeout)
				{
					// Put all drives in idle hold
					for (size_t drive = 0; drive < DRIVES; ++drive)
					{
						reprap.GetPlatform()->SetDriveIdle(drive);
					}
					iState = IdleState::idle;
				}
			}
		}
		else
		{
			// See whether we need to prepare any moves
			int32_t preparedTime = 0;
			DDA::DDAState st;
			while ((st = cdda->GetState()) == DDA:: completed || st == DDA::executing || st == DDA::frozen)
			{
				preparedTime += cdda->GetTimeLeft();
				cdda = cdda->GetNext();
			}

			// If the number of prepared moves will execute in less than the minimum time, prepare another move
			while (st == DDA::provisional && preparedTime < (int32_t)(DDA::stepClockRate/8))		// prepare moves one eighth of a second ahead of when they will be needed
			{
				cdda->Prepare();
				preparedTime += cdda->GetTimeLeft();
				cdda = cdda->GetNext();
				st = cdda->GetState();
			}
		}
	}

	reprap.GetPlatform()->ClassReport(longWait);
}

// Pause the print as soon as we can.
// Return the file position of the first queue move we are going to skip, or noFilePosition we we are not skipping any moves.
// If we skipped any moves then we update 'positions' to the positions and feed rate expected for the next move, else we leave them alone.
FilePosition Move::PausePrint(float positions[DRIVES+1])
{
	// Find a move we can pause after.
	// Ideally, we would adjust a move if necessary and possible so that we can pause after it, but for now we don't do that.
	// There are a few possibilities:
	// 1. There are no moves in the queue.
	// 2. There is a currently-executing move, and possibly some more in the queue.
	// 3. There are moves in the queue, but we haven't started executing them yet. Unlikely, but possible.

	const DDA *savedDdaRingAddPointer = ddaRingAddPointer;

	// First, see if there is a currently-executing move, and if so, whether we can safely pause at the end of it
	cpu_irq_disable();
	DDA *dda = currentDda;
	if (dda != nullptr)
	{
		if (dda->CanPause())
		{
			ddaRingAddPointer = dda->GetNext();
		}
		else
		{
			// We can't safely pause after the currently-executing move because its end speed is too high so we may miss steps.
			// Search for the next move that we can safely stop after.
			dda = ddaRingGetPointer;
			while (dda != ddaRingAddPointer)
			{
				if (dda->CanPause())
				{
					ddaRingAddPointer = dda->GetNext();
					break;
				}
				dda = dda->GetNext();
			}
		}
	}
	else
	{
		ddaRingAddPointer = ddaRingGetPointer;
	}
	cpu_irq_enable();

	FilePosition fPos = noFilePosition;
	if (ddaRingAddPointer != savedDdaRingAddPointer)
	{
		// We are going to skip some moves. dda points to the last move we are going to print.
		for (size_t axis = 0; axis < AXES; ++axis)
		{
			positions[axis] = dda->GetEndCoordinate(axis, false);
		}
		positions[DRIVES] = dda->GetRequestedSpeed();

		dda = ddaRingAddPointer;
		do
		{
			if (fPos == noFilePosition)
			{
				fPos = dda->GetFilePosition();
			}
			dda->Release();
			dda = dda->GetNext();
		}
		while (dda != savedDdaRingAddPointer);
	}
	else
	{
		GetCurrentUserPosition(positions, 0);
	}

	return fPos;
}

uint32_t maxStepTime=0, maxCalcTime=0, minCalcTime = 999, maxReps = 0;

void Move::Diagnostics()
{
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Move Diagnostics:\n");

	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "MaxStepClocks: %u, minCalcClocks: %u, maxCalcClocks: %u, maxReps: %u\n",
										maxStepTime, minCalcTime, maxCalcTime, maxReps);
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Simulation time: %f\n", simulationTime);
	maxStepTime = maxCalcTime = maxReps = 0;
	minCalcTime = 999;

#if 0
  if(active)
    platform->Message(HOST_MESSAGE, " active\n");
  else
    platform->Message(HOST_MESSAGE, " not active\n");

  platform->Message(HOST_MESSAGE, " look ahead ring count: ");
  snprintf(scratchString, STRING_LENGTH, "%d\n", lookAheadRingCount);
  platform->Message(HOST_MESSAGE, scratchString);
  if(dda == NULL)
    platform->Message(HOST_MESSAGE, " dda: NULL\n");
  else
  {
    if(dda->Active())
      platform->Message(HOST_MESSAGE, " dda: active\n");
    else
      platform->Message(HOST_MESSAGE, " dda: not active\n");

  }
  if(ddaRingLocked)
    platform->Message(HOST_MESSAGE, " dda ring is locked\n");
  else
    platform->Message(HOST_MESSAGE, " dda ring is not locked\n");
  if(addNoMoreMoves)
    platform->Message(HOST_MESSAGE, " addNoMoreMoves is true\n\n");
  else
    platform->Message(HOST_MESSAGE, " addNoMoreMoves is false\n\n");
#endif
}

// These are the actual numbers we want in the positions, so don't transform them.
void Move::SetPositions(const float move[DRIVES])
{
	if (DDARingEmpty())
	{
		ddaRingAddPointer->GetPrevious()->SetPositions(move);
	}
	else
	{
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "SetPositions called when DDA ring not empty\n");
	}
}

void Move::EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const
{
	MotorTransform(coords, ep);
	for (size_t drive = AXES; drive < numDrives; ++drive)
	{
		ep[drive] = MotorEndPointToMachine(drive, coords[drive]);
	}
}

void Move::SetFeedrate(float feedRate)
{
	if (DDARingEmpty())
	{
		DDA *lastMove = ddaRingAddPointer->GetPrevious();
		currentFeedrate = feedRate;
		lastMove->SetFeedRate(feedRate);
	}
	else
	{
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "SetFeedrate called when DDA ring not empty\n");
	}
}

// Returns steps from units (mm) for a particular drive
int32_t Move::MotorEndPointToMachine(size_t drive, float coord)
{
	return (int32_t)roundf(coord * reprap.GetPlatform()->DriveStepsPerUnit(drive));
}

// Convert motor coordinates to machine coordinates
// Used after homing and after individual motor moves.
// This is computationally expensive on a delta, so only call it when necessary, and never from the step ISR.
void Move::MachineToEndPoint(const int32_t motorPos[], float machinePos[], size_t numDrives) const
{
	const float *stepsPerUnit = reprap.GetPlatform()->GetDriveStepsPerUnit();

	// Convert the axes
	if (IsDeltaMode())
	{
		deltaParams.InverseTransform(motorPos[A_AXIS]/stepsPerUnit[A_AXIS], motorPos[B_AXIS]/stepsPerUnit[B_AXIS], motorPos[C_AXIS]/stepsPerUnit[C_AXIS], machinePos);

		// We don't do inverse transforms very often, so if debugging is enabled, print them
		if (reprap.Debug(moduleMove))
		{
			debugPrintf("Inverse transformed %d %d %d to %f %f %f\n", motorPos[0], motorPos[1], motorPos[2], machinePos[0], machinePos[1], machinePos[2]);
		}
	}
	else
	{
		switch (coreXYMode)
		{
		case 1:		// CoreXY
			machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerUnit[Y_AXIS]) - (motorPos[Y_AXIS] * stepsPerUnit[X_AXIS]))/(2 * stepsPerUnit[X_AXIS] * stepsPerUnit[Y_AXIS]);
			machinePos[Y_AXIS] = ((motorPos[X_AXIS] * stepsPerUnit[Y_AXIS]) + (motorPos[Y_AXIS] * stepsPerUnit[X_AXIS]))/(2 * stepsPerUnit[X_AXIS] * stepsPerUnit[Y_AXIS]);
			machinePos[Z_AXIS] = motorPos[Z_AXIS]/stepsPerUnit[Z_AXIS];
			break;

		case 2:		// CoreXZ
			machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerUnit[Z_AXIS]) - (motorPos[Z_AXIS] * stepsPerUnit[X_AXIS]))/(2 * stepsPerUnit[X_AXIS] * stepsPerUnit[Z_AXIS]);
			machinePos[Y_AXIS] = motorPos[Y_AXIS]/stepsPerUnit[Y_AXIS];
			machinePos[Z_AXIS] = ((motorPos[X_AXIS] * stepsPerUnit[Z_AXIS]) + (motorPos[Z_AXIS] * stepsPerUnit[X_AXIS]))/(2 * stepsPerUnit[X_AXIS] * stepsPerUnit[Z_AXIS]);
			break;

		case 3:		// CoreYZ
			machinePos[X_AXIS] = motorPos[X_AXIS]/stepsPerUnit[X_AXIS];
			machinePos[Y_AXIS] = ((motorPos[Y_AXIS] * stepsPerUnit[Z_AXIS]) - (motorPos[Z_AXIS] * stepsPerUnit[Y_AXIS]))/(2 * stepsPerUnit[Y_AXIS] * stepsPerUnit[Z_AXIS]);
			machinePos[Z_AXIS] = ((motorPos[Y_AXIS] * stepsPerUnit[Z_AXIS]) + (motorPos[Z_AXIS] * stepsPerUnit[Y_AXIS]))/(2 * stepsPerUnit[Y_AXIS] * stepsPerUnit[Z_AXIS]);
			break;

		default:
			machinePos[X_AXIS] = motorPos[X_AXIS]/stepsPerUnit[X_AXIS];
			machinePos[Y_AXIS] = motorPos[Y_AXIS]/stepsPerUnit[Y_AXIS];
			machinePos[Z_AXIS] = motorPos[Z_AXIS]/stepsPerUnit[Z_AXIS];
			break;
		}
	}

	// Convert the extruders
	for (size_t drive = AXES; drive < numDrives; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerUnit[drive];
	}
}

// Convert Cartesian coordinates to delta motor steps
void Move::MotorTransform(const float machinePos[AXES], int32_t motorPos[AXES]) const
{
	if (IsDeltaMode())
	{
		for (size_t axis = 0; axis < AXES; ++axis)
		{
			motorPos[axis] = MotorEndPointToMachine(axis, deltaParams.Transform(machinePos, axis));
		}

		if (reprap.Debug(moduleMove) && reprap.Debug(moduleDda))
		{
			debugPrintf("Transformed %f %f %f to %d %d %d\n", machinePos[0], machinePos[1], machinePos[2], motorPos[0], motorPos[1], motorPos[2]);
		}
	}
	else
	{
		switch (coreXYMode)
		{
		case 1:
			motorPos[X_AXIS] = MotorEndPointToMachine(X_AXIS, machinePos[X_AXIS] + machinePos[Y_AXIS]);
			motorPos[Y_AXIS] = MotorEndPointToMachine(Y_AXIS, machinePos[Y_AXIS] - machinePos[X_AXIS]);
			motorPos[Z_AXIS] = MotorEndPointToMachine(Z_AXIS, machinePos[Z_AXIS]);
			break;

		case 2:
			motorPos[X_AXIS] = MotorEndPointToMachine(X_AXIS, machinePos[X_AXIS] + machinePos[Z_AXIS]);
			motorPos[Y_AXIS] = MotorEndPointToMachine(Y_AXIS, machinePos[Y_AXIS]);
			motorPos[Z_AXIS] = MotorEndPointToMachine(Z_AXIS, machinePos[Z_AXIS] - machinePos[X_AXIS]);
			break;

		case 3:
			motorPos[X_AXIS] = MotorEndPointToMachine(X_AXIS, machinePos[X_AXIS]);
			motorPos[Y_AXIS] = MotorEndPointToMachine(Y_AXIS, machinePos[Y_AXIS] + machinePos[Z_AXIS]);
			motorPos[Z_AXIS] = MotorEndPointToMachine(Z_AXIS, machinePos[Z_AXIS] - machinePos[Y_AXIS]);
			break;

		default:
			motorPos[X_AXIS] = MotorEndPointToMachine(X_AXIS, machinePos[X_AXIS]);
			motorPos[Y_AXIS] = MotorEndPointToMachine(Y_AXIS, machinePos[Y_AXIS]);
			motorPos[Z_AXIS] = MotorEndPointToMachine(Z_AXIS, machinePos[Z_AXIS]);
			break;
		}
	}
}

// Do the Axis transform BEFORE the bed transform
void Move::AxisTransform(float xyzPoint[AXES]) const
{
	xyzPoint[X_AXIS] = xyzPoint[X_AXIS] + tanXY*xyzPoint[Y_AXIS] + tanXZ*xyzPoint[Z_AXIS];
	xyzPoint[Y_AXIS] = xyzPoint[Y_AXIS] + tanYZ*xyzPoint[Z_AXIS];
}

// Invert the Axis transform AFTER the bed transform
void Move::InverseAxisTransform(float xyzPoint[AXES]) const
{
	xyzPoint[Y_AXIS] = xyzPoint[Y_AXIS] - tanYZ*xyzPoint[Z_AXIS];
	xyzPoint[X_AXIS] = xyzPoint[X_AXIS] - (tanXY*xyzPoint[Y_AXIS] + tanXZ*xyzPoint[Z_AXIS]);
}

void Move::Transform(float xyzPoint[AXES]) const
{
	AxisTransform(xyzPoint);
	BedTransform(xyzPoint);
}

void Move::InverseTransform(float xyzPoint[AXES]) const
{
	InverseBedTransform(xyzPoint);
	InverseAxisTransform(xyzPoint);
}

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[AXES]) const
{
	if (!identityBedTransform)
	{
		switch(NumberOfProbePoints())
		{
		case 0:
			return;

		case 3:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] + aX*xyzPoint[X_AXIS] + aY*xyzPoint[Y_AXIS] + aC;
			break;

		case 4:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] + SecondDegreeTransformZ(xyzPoint[X_AXIS], xyzPoint[Y_AXIS]);
			break;

		case 5:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] + TriangleZ(xyzPoint[X_AXIS], xyzPoint[Y_AXIS]);
			break;

		default:
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "BedTransform: wrong number of sample points.");
		}
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[AXES]) const
{
	if (!identityBedTransform)
	{
		switch(NumberOfProbePoints())
		{
		case 0:
			return;

		case 3:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] - (aX*xyzPoint[X_AXIS] + aY*xyzPoint[Y_AXIS] + aC);
			break;

		case 4:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] - SecondDegreeTransformZ(xyzPoint[X_AXIS], xyzPoint[Y_AXIS]);
			break;

		case 5:
			xyzPoint[Z_AXIS] = xyzPoint[Z_AXIS] - TriangleZ(xyzPoint[X_AXIS], xyzPoint[Y_AXIS]);
			break;

		default:
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "InverseBedTransform: wrong number of sample points.");
		}
	}
}

void Move::SetIdentityTransform()
{
	identityBedTransform = true;
}

float Move::AxisCompensation(int8_t axis) const
{
	switch(axis)
	{
		case X_AXIS:
			return tanXY;

		case Y_AXIS:
			return tanYZ;

		case Z_AXIS:
			return tanXZ;

		default:
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Axis compensation requested for non-existent axis.\n");
	}
	return 0.0;
}

void Move::SetAxisCompensation(int8_t axis, float tangent)
{
	switch(axis)
	{
	case X_AXIS:
		tanXY = tangent;
		break;
	case Y_AXIS:
		tanYZ = tangent;
		break;
	case Z_AXIS:
		tanXZ = tangent;
		break;
	default:
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "SetAxisCompensation: dud axis.\n");
	}
}

void Move::BarycentricCoordinates(size_t p1, size_t p2, size_t p3, float x, float y, float& l1, float& l2, float& l3) const
{
	float y23 = baryYBedProbePoints[p2] - baryYBedProbePoints[p3];
	float x3 = x - baryXBedProbePoints[p3];
	float x32 = baryXBedProbePoints[p3] - baryXBedProbePoints[p2];
	float y3 = y - baryYBedProbePoints[p3];
	float x13 = baryXBedProbePoints[p1] - baryXBedProbePoints[p3];
	float y13 = baryYBedProbePoints[p1] - baryYBedProbePoints[p3];
	float iDet = 1.0 / (y23 * x13 + x32 * y13);
	l1 = (y23 * x3 + x32 * y3) * iDet;
	l2 = (-y13 * x3 + x13 * y3) * iDet;
	l3 = 1.0 - l1 - l2;
}

/*
 * Interpolate on a triangular grid.  The triangle corners are indexed:
 *
 *   ^  [1]      [2]
 *   |
 *   Y      [4]
 *   |
 *   |  [0]      [3]
 *      -----X---->
 *
 */
float Move::TriangleZ(float x, float y) const
{
	for (size_t i = 0; i < 4; i++)
	{
		size_t j = (i + 1) % 4;
		float l1, l2, l3;
		BarycentricCoordinates(i, j, 4, x, y, l1, l2, l3);
		if (l1 > TRIANGLE_0 && l2 > TRIANGLE_0 && l3 > TRIANGLE_0)
		{
			return l1 * baryZBedProbePoints[i] + l2 * baryZBedProbePoints[j] + l3 * baryZBedProbePoints[4];
		}
	}
	reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Triangle interpolation: point outside all triangles!\n");
	return 0.0;
}

// Calibrate or set the bed equation after probing.
// sParam is the value of the S parameter in the G30 command that provoked this call.
void Move::FinishedBedProbing(int sParam, StringRef& reply)
{
	if (reprap.Debug(moduleMove))
	{
		debugPrintf("Z probe offsets:");
		for (size_t i = 0; i < NumberOfProbePoints(); ++i)
		{
			debugPrintf(" %.2f", zBedProbePoints[i]);
		}
		debugPrintf("\n");
	}

	bool ok = false;
	switch (sParam)
	{
	case 0:
		SetProbedBedEquation(reply);
		ok = true;
		break;

	case 4:
		// On a delta, this calibrates the endstop adjustments and delta radius automatically after a 4 point probe.
		// Probe points 1,2,3 must be near the bases of the X, Y and Z towers in that order. Point 4 must be in the centre.
		if (IsDeltaMode() && NumberOfProbePoints() >= 4)
		{
			FourPointDeltaCalibration(reply);
			ok = true;
		}
		break;

	case 6:
		// On a delta, this calibrates the endstop adjustments and tower positions automatically after a 6 point probe.
		if (IsDeltaMode() && NumberOfProbePoints() >= 6)
		{
			SixPointDeltaCalibration(reply);
			ok = true;
		}
		break;

	default:
		break;
	}

	if (!ok)
	{
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Bed probe type %d with %d points not supported on this machine\n", sParam, NumberOfProbePoints());
	}
}

void Move::SetProbedBedEquation(StringRef& reply)
{
	switch(NumberOfProbePoints())
	{
	case 3:
		/*
		 * Transform to a plane
		 */

		{
			float x10 = xBedProbePoints[1] - xBedProbePoints[0];
			float y10 = yBedProbePoints[1] - yBedProbePoints[0];
			float z10 = zBedProbePoints[1] - zBedProbePoints[0];
			float x20 = xBedProbePoints[2] - xBedProbePoints[0];
			float y20 = yBedProbePoints[2] - yBedProbePoints[0];
			float z20 = zBedProbePoints[2] - zBedProbePoints[0];
			float a = y10 * z20 - z10 * y20;
			float b = z10 * x20 - x10 * z20;
			float c = x10 * y20 - y10 * x20;
			float d = -(xBedProbePoints[1] * a + yBedProbePoints[1] * b + zBedProbePoints[1] * c);
			aX = -a / c;
			aY = -b / c;
			aC = -d / c;
			identityBedTransform = false;
		}
		break;

	case 4:
		/*
		 * Transform to a ruled-surface quadratic.  The corner points for interpolation are indexed:
		 *
		 *   ^  [1]      [2]
		 *   |
		 *   Y
		 *   |
		 *   |  [0]      [3]
		 *      -----X---->
		 *
		 *   These are the scaling factors to apply to x and y coordinates to get them into the
		 *   unit interval [0, 1].
		 */
		xRectangle = 1.0 / (xBedProbePoints[3] - xBedProbePoints[0]);
		yRectangle = 1.0 / (yBedProbePoints[1] - yBedProbePoints[0]);
		identityBedTransform = false;
		break;

	case 5:
		for(int8_t i = 0; i < 4; i++)
		{
			float x10 = xBedProbePoints[i] - xBedProbePoints[4];
			float y10 = yBedProbePoints[i] - yBedProbePoints[4];
			float z10 = zBedProbePoints[i] - zBedProbePoints[4];
			baryXBedProbePoints[i] = xBedProbePoints[4] + 2.0 * x10;
			baryYBedProbePoints[i] = yBedProbePoints[4] + 2.0 * y10;
			baryZBedProbePoints[i] = zBedProbePoints[4] + 2.0 * z10;
		}
		baryXBedProbePoints[4] = xBedProbePoints[4];
		baryYBedProbePoints[4] = yBedProbePoints[4];
		baryZBedProbePoints[4] = zBedProbePoints[4];
		identityBedTransform = false;
		break;

	default:
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Attempt to set bed compensation before all probe points have been recorded.");
	}

	reply.copy("Bed equation fits points");
	for (size_t point = 0; point < NumberOfProbePoints(); point++)
	{
		reply.catf(" [%.1f, %.1f, %.3f]", xBedProbePoints[point], yBedProbePoints[point], zBedProbePoints[point]);
	}
	reply.cat("\n");
}

// Do 4-point delta calibration. We adjust the 3 endstop corrections and the delta radius.
void Move::FourPointDeltaCalibration(StringRef& reply)
{
	const float averageEdgeHeight = (zBedProbePoints[0] + zBedProbePoints[1] + zBedProbePoints[2])/3.0;
	const float averageEndstopOffset = (deltaParams.GetEndstopAdjustment(X_AXIS) + deltaParams.GetEndstopAdjustment(Y_AXIS) + deltaParams.GetEndstopAdjustment(Z_AXIS))/3.0;
	float probeRadiusSquared = 0.0;

	// Adjust the endstops to account for the differences in reading, while setting the average of the new values to zero
	for (size_t axis = 0; axis < 3; ++axis)
	{
		deltaParams.SetEndstopAdjustment(axis, deltaParams.GetEndstopAdjustment(axis) + averageEdgeHeight - averageEndstopOffset - zBedProbePoints[axis]);
		probeRadiusSquared += fsquare(xBedProbePoints[axis]) + fsquare(yBedProbePoints[axis]);
	}

	// Adjust the delta radius to make the bed appear flat
	const float edgeDistance = deltaParams.GetRadius() - sqrt(probeRadiusSquared/3.0);
	const float factor = edgeDistance/sqrt(fsquare(deltaParams.GetDiagonal()) - fsquare(edgeDistance))
							- deltaParams.GetRadius()/sqrt(fsquare(deltaParams.GetDiagonal()) - fsquare(deltaParams.GetRadius()));
	const float diff = zBedProbePoints[3] - averageEdgeHeight;
	deltaParams.SetRadius(deltaParams.GetRadius() + diff/factor);

	// Adjust the homed height to account for the error at the centre and the change in average endstop correction
	deltaParams.SetHomedHeight(deltaParams.GetHomedHeight() + averageEndstopOffset - zBedProbePoints[3]);

	// Print the parameters so the user can see when they have converged
	deltaParams.PrintParameters(reply, false);
}

// Do 6-point delta calibration. We adjust the X positions of the front two towers, the Y position of the rear tower, and the three endstop corrections.
void Move::SixPointDeltaCalibration(StringRef& reply)
{
	if (reprap.Debug(moduleMove))
	{
		deltaParams.PrintParameters(scratchString, true);
		debugPrintf("%s\n", scratchString.Pointer());
	}

	// Build a 6x7 matrix of derivatives with respect to xa, xb, yc, za, zb, zc and the height errors.
	float matrix[6][7];
	for (size_t i = 0; i < 6; ++i)
	{
		float machinePos[3];
		machinePos[0] = xBedProbePoints[i];
		machinePos[1] = yBedProbePoints[i];
		machinePos[2] = 0.0;						// the height doesn't matter
		for (size_t j = 0; j < 6; ++j)
		{
			matrix[i][j] = deltaParams.ComputeDerivative(j, machinePos);
		}
		matrix[i][6] = -zBedProbePoints[i];
	}

	if (reprap.Debug(moduleMove))
	{
		PrintMatrix("Raw matrix", matrix);
	}

	for (size_t i = 0; i < 6; ++i)
	{
		// Swap the rows around for stable Gauss-Jordan elimination
		float vmax = fabs(matrix[i][i]);
		for (size_t j = i + 1; j < 6; ++j)
		{
			const float rmax = fabs(matrix[j][i]);
			if (rmax > vmax)
			{
				// swap rows i and j
				for (size_t k = i; k < 7; ++k)
				{
					swap(matrix[i][k], matrix[j][k]);
				}
				vmax = rmax;
			}
		}

		// Use row i to eliminate the ith element from previous and subsequent rows
		float v = matrix[i][i];
		for (size_t j = 0; j < i; ++j)
		{
			float factor = matrix[j][i]/v;
//			matrix[j][i] = 0.0;
//			for (size_t k = i + 1; k < 7; ++k)
			for (size_t k = i; k < 7; ++k)
			{
				matrix[j][k] -= matrix[i][k] * factor;
			}
		}

		for (size_t j = i + 1; j < 6; ++j)
		{
			float factor = matrix[j][i]/v;
//			matrix[j][i] = 0.0;
//			for (size_t k = i + 1; k < 7; ++k)
			for (size_t k = i; k < 7; ++k)
			{
				matrix[j][k] -= matrix[i][k] * factor;
			}
		}
	}

	if (reprap.Debug(moduleMove))
	{
		PrintMatrix("Diagonalised matrix", matrix);
	}

	deltaParams.SixFactorAdjust(matrix[0][6]/matrix[0][0], matrix[1][6]/matrix[1][1], matrix[2][6]/matrix[2][2],
			matrix[3][6]/matrix[3][3], matrix[4][6]/matrix[4][4], matrix[5][6]/matrix[5][5]);

	deltaParams.PrintParameters(reply, true);
}

/*
 * Transform to a ruled-surface quadratic.  The corner points for interpolation are indexed:
 *
 *   ^  [1]      [2]
 *   |
 *   Y
 *   |
 *   |  [0]      [3]
 *      -----X---->
 *
 *   The values of x and y are transformed to put them in the interval [0, 1].
 */
float Move::SecondDegreeTransformZ(float x, float y) const
{
	x = (x - xBedProbePoints[0])*xRectangle;
	y = (y - yBedProbePoints[0])*yRectangle;
	return (1.0 - x)*(1.0 - y)*zBedProbePoints[0] + x*(1.0 - y)*zBedProbePoints[3] + (1.0 - x)*y*zBedProbePoints[1] + x*y*zBedProbePoints[2];
}

// This is the function that's called by the timer interrupt to step the motors.
void Move::Interrupt()
{
	bool again = true;
	while (again && currentDda != nullptr)
	{
		again = currentDda->Step();
	}
}

// This is called from the step ISR when the current move has been completed
void Move::CurrentMoveCompleted()
{
	// Save the current motor coordinates, and the machine Cartesian coordinates if known
	liveCoordinatesValid = currentDda->FetchEndPosition(const_cast<int32_t*>(liveEndPoints), const_cast<float *>(liveCoordinates));

	currentDda->Release();
	currentDda = nullptr;
	ddaRingGetPointer = ddaRingGetPointer->GetNext();
}

// Start the next move. Must be called with interrupts disabled, to avoid a race condition.
bool Move::StartNextMove(uint32_t startTime)
{
	if (ddaRingGetPointer->GetState() == DDA::frozen)
	{
		currentDda = ddaRingGetPointer;
		return currentDda->Start(startTime);
	}
	else
	{
		return false;
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitLowStop(size_t drive, DDA* hitDDA)
{
	if (drive < AXES && !IsDeltaMode())		// should always be true
	{
		float hitPoint;
		if (drive == Z_AXIS)
		{
			// Special case of doing a G1 S1 Z move on a Cartesian printer. This is not how we normally home the Z axis, we use G30 instead.
			// But I think it used to work, so let's not break it.
			hitPoint = reprap.GetPlatform()->ZProbeStopHeight();
		}
		else
		{
			hitPoint = reprap.GetPlatform()->AxisMinimum(drive);
		}
		int32_t coord = MotorEndPointToMachine(drive, hitPoint);
		hitDDA->SetDriveCoordinate(coord, drive);
		reprap.GetGCodes()->SetAxisIsHomed(drive);
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitHighStop(size_t drive, DDA* hitDDA)
{
	if (drive < AXES)		// should always be true
	{
		float position = (IsDeltaMode())
							? deltaParams.GetHomedCarriageHeight(drive)
							        // this is a delta printer, so the motor is at the homed carriage height for this drive
							: reprap.GetPlatform()->AxisMaximum(drive);
									// this is a Cartesian printer, so we're at the maximum for this axis
		hitDDA->SetDriveCoordinate(MotorEndPointToMachine(drive, position), drive);
		reprap.GetGCodes()->SetAxisIsHomed(drive);
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
// The move has already been aborted when this is called, so the endpoints in the DDA are the current motor positions.
void Move::ZProbeTriggered(DDA* hitDDA)
{
	// Currently, we don't need to do anything here
}

// Return the untransformed machine coordinates
void Move::GetCurrentMachinePosition(float m[DRIVES + 1], bool disableMotorMapping) const
{
	DDA *lastQueuedMove = ddaRingAddPointer->GetPrevious();
	for (size_t i = 0; i < DRIVES; i++)
	{
		if (i < AXES)
		{
			m[i] = lastQueuedMove->GetEndCoordinate(i, disableMotorMapping);
		}
		else
		{
			m[i] = 0.0;
		}
	}
	m[DRIVES] = currentFeedrate;
}

/*static*/ float Move::MotorEndpointToPosition(int32_t endpoint, size_t drive)
{
	return ((float)(endpoint))/reprap.GetPlatform()->DriveStepsPerUnit(drive);
}

// Return the transformed machine coordinates
void Move::GetCurrentUserPosition(float m[DRIVES + 1], uint8_t moveType) const
{
	GetCurrentMachinePosition(m, moveType == 2 || (moveType == 1 && IsDeltaMode()));
	if (moveType == 0)
	{
		InverseTransform(m);
	}
}

// Return the current live XYZ and extruder coordinates
// Interrupts are assumed enabled on entry, so do not call this from an ISR
void Move::LiveCoordinates(float m[DRIVES])
{
	// The live coordinates and live endpoints are modified by the ISR, to be careful to get a self-consistent set of them
	cpu_irq_disable();
	if (liveCoordinatesValid)
	{
		// All coordinates are valid, so copy them across
		memcpy(m, const_cast<const float *>(liveCoordinates), sizeof(m[0]) * DRIVES);
		cpu_irq_enable();
	}
	else
	{
		// Only the extruder coordinates are valid, so we need to convert the motor endpoints to coordinates
		memcpy(m + AXES, const_cast<const float *>(liveCoordinates + AXES), sizeof(m[0]) * (DRIVES - AXES));
		int32_t tempEndPoints[AXES];
		memcpy(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints));
		cpu_irq_enable();
		MachineToEndPoint(tempEndPoints, m, AXES);		// this is slow, so do it with interrupts enabled

		// If the ISR has not updated the endpoints, store the live coordinates back so that we don't need to do it again
		cpu_irq_disable();
		if (memcmp(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints)) == 0)
		{
			memcpy(const_cast<float *>(liveCoordinates), m, sizeof(m[0]) * AXES);
			liveCoordinatesValid = true;
		}
		cpu_irq_enable();
	}
	InverseTransform(m);
}

// These are the actual numbers that we want to be the coordinates, so don't transform them.
// Interrupts are assumed enabled on entry, so do not call this from an ISR
void Move::SetLiveCoordinates(const float coords[DRIVES])
{
	cpu_irq_disable();
	for(size_t drive = 0; drive < DRIVES; drive++)
	{
		liveCoordinates[drive] = coords[drive];
	}
	liveCoordinatesValid = true;
	EndPointToMachine(coords, const_cast<int32_t *>(liveEndPoints), AXES);
	cpu_irq_enable();
}

void Move::SetXBedProbePoint(int index, float x)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(BOTH_MESSAGE, "Z probe point X index out of range.\n");
		return;
	}
	xBedProbePoints[index] = x;
	probePointSet[index] |= xSet;
}

void Move::SetYBedProbePoint(int index, float y)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(BOTH_MESSAGE, "Z probe point Y index out of range.\n");
		return;
	}
	yBedProbePoints[index] = y;
	probePointSet[index] |= ySet;
}

void Move::SetZBedProbePoint(int index, float z)
{
	if(index < 0 || index >= NUMBER_OF_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(BOTH_MESSAGE, "Z probe point Z index out of range.\n");
		return;
	}
	zBedProbePoints[index] = z;
	probePointSet[index] |= zSet;
}

float Move::XBedProbePoint(int index) const
{
	return xBedProbePoints[index];
}

float Move::YBedProbePoint(int index) const
{
	return yBedProbePoints[index];
}

float Move::ZBedProbePoint(int index) const
{
	return zBedProbePoints[index];
}

bool Move::AllProbeCoordinatesSet(int index) const
{
	return probePointSet[index] == (xSet | ySet | zSet);
}

bool Move::XYProbeCoordinatesSet(int index) const
{
	return (probePointSet[index]  & xSet) && (probePointSet[index]  & ySet);
}

int Move::NumberOfProbePoints() const
{
	for(int i = 0; i < NUMBER_OF_PROBE_POINTS; i++)
	{
		if(!AllProbeCoordinatesSet(i))
		{
			return i;
		}
	}
	return NUMBER_OF_PROBE_POINTS;
}

int Move::NumberOfXYProbePoints() const
{
	for(int i = 0; i < NUMBER_OF_PROBE_POINTS; i++)
	{
		if(!XYProbeCoordinatesSet(i))
		{
			return i;
		}
	}
	return NUMBER_OF_PROBE_POINTS;
}

// Enter or leave simulation mode
void Move::Simulate(bool sim)
{
	simulating = sim;
	if (sim)
	{
		simulationTime = 0.0;
	}
}

// For debugging
void Move::PrintCurrentDda() const
{
	if (currentDda != nullptr)
	{
		currentDda->DebugPrint();
		reprap.GetPlatform()->GetLine()->Flush();
	}
}

const char* Move::GetGeometryString() const
{
	return (IsDeltaMode()) ? "delta"
			: (coreXYMode == 1) ? "coreXY"
			: (coreXYMode == 2) ? "coreXZ"
			: (coreXYMode == 3) ? "coreYZ"
			: "cartesian";
}

/*static*/ void Move::PrintMatrix(const char* s, float matrix[6][7])
{
	debugPrintf("%s\n", s);
	for (size_t i = 0; i < 6; ++i)
	{
		debugPrintf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f = %6.3f\n", matrix[i][0], matrix[i][1], matrix[i][2], matrix[i][3], matrix[i][4], matrix[i][5], matrix[i][6]);
	}
}

// End
