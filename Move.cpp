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

    for (size_t axis = 0; axis < AXES; ++axis)
    {
    	endstopAdjustments[axis] = 0.0;
    	towerX[axis] = towerY[axis] = 0.0;
    }
}

void DeltaParameters::SetRadius(float r)
{
	radius = r;

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

Move::Move(Platform* p, GCodes* g) : currentDda(NULL)
{
	active = false;

	// Build the DDA ring
	DDA *dda = new DDA(NULL);
	ddaRingGetPointer = ddaRingAddPointer = dda;
	for(size_t i = 1; i < DdaRingLength; i++)
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
		reprap.GetPlatform()->SetDirection(i, FORWARDS, false);
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

	lastTime = reprap.GetPlatform()->Time();
	longWait = lastTime;

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

	// See if we can add another move to the ring
	if (!addNoMoreMoves && ddaRingAddPointer->GetState() == DDA::empty)
	{
		if (reprap.Debug(moduleMove))
		{
			ddaRingAddPointer->PrintIfHasStepError();
		}
		// If there's a G Code move available, add it to the DDA ring for processing.
		float nextMove[DRIVES + 1];
		EndstopChecks endStopsToCheck;
		bool noDeltaMapping;
		if (reprap.GetGCodes()->ReadMove(nextMove, endStopsToCheck, noDeltaMapping))
		{
			currentFeedrate = nextMove[DRIVES];		// might be G1 with just an F field
			if (!noDeltaMapping || !IsDeltaMode())
			{
				Transform(nextMove);
			}
			if (ddaRingAddPointer->Init(nextMove, endStopsToCheck, IsDeltaMode() && !noDeltaMapping))
			{
				ddaRingAddPointer = ddaRingAddPointer->GetNext();
			}
		}
	}

	// See whether we need to kick off a move
	DDA *cdda = currentDda;										// currentDda is volatile, so copy it
	if (cdda == nullptr)
	{
		// No DDA is executing, so start executing a new one if possible
		DDA *dda = ddaRingGetPointer;
		if (dda->GetState() == DDA::provisional)
		{
			dda->Prepare();
		}
		if (StartNextMove(Platform::GetInterruptClocks()))		// start the next move if none is executing already
		{
			cpu_irq_disable();
			Interrupt();
			cpu_irq_enable();
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

	reprap.GetPlatform()->ClassReport("Move", longWait, moduleMove);
}

uint32_t maxStepTime=0, maxCalcTime=0, minCalcTime = 999, maxReps = 0;

void Move::Diagnostics()
{
	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "Move Diagnostics:\n");

	reprap.GetPlatform()->AppendMessage(BOTH_MESSAGE, "MaxStepClocks: %u, minCalcClocks: %u, maxCalcClocks: %u, maxReps: %u\n",
										maxStepTime, minCalcTime, maxCalcTime, maxReps);
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
	if (IsDeltaMode())
	{
		DeltaTransform(coords, ep);
		for (size_t drive = AXES; drive < numDrives; ++drive)
		{
			ep[drive] = MotorEndPointToMachine(drive, coords[drive]);
		}
	}
	else
	{
		for (size_t drive = 0; drive < DRIVES; drive++)
		{
			ep[drive] = MotorEndPointToMachine(drive, coords[drive]);
		}
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
// This is computationally expensive on a delta, so only call it when necessary, and never from the step ISR.
void Move::MachineToEndPoint(const int32_t motorPos[], float machinePos[], size_t numDrives) const
{
	if (IsDeltaMode())
	{
		InverseDeltaTransform(motorPos, machinePos);			// convert the axes
		for (size_t drive = AXES; drive < numDrives; ++drive)
		{
			machinePos[drive] = MotorEndpointToPosition(motorPos[drive], drive);
		}
	}
	else
	{
		for (size_t drive = 0; drive < numDrives; ++drive)
		{
			machinePos[drive] = MotorEndpointToPosition(motorPos[drive], drive);
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

// Convert motor step positions to Cartesian machine coordinates.
// Used after homing and after individual motor moves.
// Because this is computationally expensive, we only call it when necessary, and never from the step ISR.
void Move::InverseDeltaTransform(const int32_t motorPos[AXES], float machinePos[AXES]) const
{
	deltaParams.InverseTransform(
			MotorEndpointToPosition(motorPos[A_AXIS], A_AXIS),
			MotorEndpointToPosition(motorPos[B_AXIS], B_AXIS),
			MotorEndpointToPosition(motorPos[C_AXIS], C_AXIS),
			machinePos);

	// We don't do inverse transforms very often, so if debugging is enabled, print them
	if (reprap.Debug(moduleMove))
	{
		debugPrintf("Inverse transformed %d %d %d to %f %f %f\n", motorPos[0], motorPos[1], motorPos[2], machinePos[0], machinePos[1], machinePos[2]);
	}
}

// Convert Cartesian coordinates to delta motor steps
void Move::DeltaTransform(const float machinePos[AXES], int32_t motorPos[AXES]) const
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

// Start the next move.
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
void Move::GetCurrentMachinePosition(float m[DRIVES + 1], bool disableDeltaMapping) const
{
	DDA *lastQueuedMove = ddaRingAddPointer->GetPrevious();
	for (size_t i = 0; i < DRIVES; i++)
	{
		if (i < AXES)
		{
			m[i] = lastQueuedMove->GetEndCoordinate(i, disableDeltaMapping);
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
void Move::GetCurrentUserPosition(float m[DRIVES + 1], bool disableDeltaMapping) const
{
	GetCurrentMachinePosition(m, disableDeltaMapping);
	if (!disableDeltaMapping)
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

// For debugging
void Move::PrintCurrentDda() const
{
	if (currentDda != nullptr)
	{
		currentDda->DebugPrint();
		reprap.GetPlatform()->GetLine()->Flush();
	}
}

// End
