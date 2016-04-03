/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "RepRapFirmware.h"

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
	for (size_t axis = 0; axis < AXES; ++axis)
	{
		axisFactors[axis] = 1.0;
	}
	deltaProbing = false;

	// Empty the ring
	ddaRingGetPointer = ddaRingCheckPointer = ddaRingAddPointer;
	DDA *dda = ddaRingAddPointer;
	do
	{
		dda->Init();
		dda = dda->GetNext();
	} while (dda != ddaRingAddPointer);

	currentDda = nullptr;
	addNoMoreMoves = false;
	stepErrors = 0;

	// Clear the transforms
	SetIdentityTransform();
	tanXY = tanYZ = tanXZ = 0.0;

	// Put the origin on the lookahead ring with default velocity in the previous position to the first one that will be used.
	// Do this by calling SetLiveCoordinates and SetPositions, so that the motor coordinates will be correct too even on a delta.
	float move[DRIVES];
	for (size_t i = 0; i < DRIVES; i++)
	{
		move[i] = 0.0;
		liveEndPoints[i] = 0;									// not actually right for a delta, but better than printing random values in response to M114
		reprap.GetPlatform()->SetDirection(i, FORWARDS);		// DC: I don't see any reason why we do this
	}
	SetLiveCoordinates(move);
	SetPositions(move);

	// Set up default bed probe points. This is only a guess, because we don't know the bed size yet.
	for (size_t point = 0; point < MAX_PROBE_POINTS; point++)
	{
		if (point < 4)
		{
			xBedProbePoints[point] = (0.3 + 0.6*(float)(point%2))*reprap.GetPlatform()->AxisMaximum(X_AXIS);
			yBedProbePoints[point] = (0.0 + 0.9*(float)(point/2))*reprap.GetPlatform()->AxisMaximum(Y_AXIS);
		}
		zBedProbePoints[point] = 0.0;
		probePointSet[point] = unset;
	}

	xRectangle = 1.0/(0.8*reprap.GetPlatform()->AxisMaximum(X_AXIS));
	yRectangle = xRectangle;

	longWait = reprap.GetPlatform()->Time();
	idleTimeout = DEFAULT_IDLE_TIMEOUT;
	iState = IdleState::idle;
	idleCount = 0;

	simulating = false;
	simulationTime = 0.0;

	active = true;
}

void Move::Exit()
{
	reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Move class exited.\n");
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

	// Check for DDA errors to print if Move debug is enabled
	while (ddaRingCheckPointer->GetState() == DDA::completed)
	{
		if (ddaRingCheckPointer->HasStepError())
		{
			if (reprap.Debug(moduleMove))
			{
				ddaRingCheckPointer->DebugPrint();
			}
			++stepErrors;
			reprap.GetPlatform()->LogError(ErrorCode::BadMove);
		}
		ddaRingCheckPointer->Free();
		ddaRingCheckPointer = ddaRingCheckPointer->GetNext();
	}

	// See if we can add another move to the ring
	if (!addNoMoreMoves && ddaRingAddPointer->GetState() == DDA::empty)
	{
		DDA *dda = ddaRingAddPointer;

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
			GCodes::RawMove nextMove;
			if (reprap.GetGCodes()->ReadMove(nextMove))
			{
				// We have a new move

#if 0	//*** This code is not finished yet ***
				// If we are doing bed compensation and the move crosses a compensation boundary by a significant amount,
				// segment it so that we can apply proper bed compensation
				// Issues here:
				// 1. Are there enough DDAs? need to make nextMove static and remember whether we have the remains of a move in there.
				// 2. Pause/restart: if we restart a segmented move when we have already executed part of it, we will extrude too much.
				// Perhaps remember how much of the last move we executed? Or always insist on completing all the segments in a move?
				bool isSegmented;
				do
				{
					GCodes::RawMove tempMove = nextMove;
					isSegmented = SegmentMove(tempMove);
					if (isSegmented)
					{
						// Extruder moves are relative, so we need to adjust the extrusion amounts in the original move
						for (size_t drive = AXES; drive < DRIVES; ++drive)
						{
							nextMove.coords[drive] -= tempMove.coords[drive];
						}
					}
					bool doMotorMapping = (moveType == 0) || (moveType == 1 && !IsDeltaMode());
					if (doMotorMapping)
					{
						Transform(tempMove);
					}
					if (ddaRingAddPointer->Init(tempMove.coords, nextMove.feedRate, nextMove.endStopsToCheck, doMotorMapping, nextMove.filePos))
					{
						ddaRingAddPointer = ddaRingAddPointer->GetNext();
						idleCount = 0;
					}
				} while (isSegmented);
#else	// Use old code
				bool doMotorMapping = (nextMove.moveType == 0) || (nextMove.moveType == 1 && !IsDeltaMode());
				if (doMotorMapping)
				{
					Transform(nextMove.coords);
				}
				if (ddaRingAddPointer->Init(&nextMove, doMotorMapping))
				{
					ddaRingAddPointer = ddaRingAddPointer->GetNext();
					idleCount = 0;
				}
#endif
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
			dda->Complete();
			ddaRingGetPointer = ddaRingGetPointer->GetNext();
		}
	}
	else if (!deltaProbing)
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
					reprap.GetPlatform()->SetDrivesIdle();					// put all drives in idle hold
					iState = IdleState::idle;
				}
			}
		}
		else
		{
			// See whether we need to prepare any moves
			int32_t preparedTime = 0;
			DDA::DDAState st;
			while ((st = cdda->GetState()) == DDA::completed || st == DDA::executing || st == DDA::frozen)
			{
				preparedTime += cdda->GetTimeLeft();
				cdda = cdda->GetNext();
				if (cdda == ddaRingAddPointer)
				{
					break;
				}
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
// Returns the file position of the first queue move we are going to skip, or noFilePosition we we are not skipping any moves.
// We update 'positions' to the positions and feed rate expected for the next move, and the amount of extrusion in the moves we skipped.
// If we are not skipping any moves then the feed rate is left alone, therefore the caller should set this up first.
FilePosition Move::PausePrint(float positions[DRIVES+1])
{
	// Find a move we can pause after.
	// Ideally, we would adjust a move if necessary and possible so that we can pause after it, but for now we don't do that.
	// There are a few possibilities:
	// 1. There are no moves in the queue.
	// 2. There is a currently-executing move, and possibly some more in the queue.
	// 3. There are moves in the queue, but we haven't started executing them yet. Unlikely, but possible.

	// First, see if there is a currently-executing move, and if so, whether we can safely pause at the end of it
	const DDA *savedDdaRingAddPointer = ddaRingAddPointer;
	cpu_irq_disable();
	DDA *dda = currentDda;
	if (dda != nullptr)
	{
		// A move is being executed. See if we can safely pause at the end of it.
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
					if (ddaRingAddPointer->GetState() == DDA::frozen)
					{
						// Change the state so that the ISR won't start executing this move
						ddaRingAddPointer->Free();
					}
					break;
				}
				dda = dda->GetNext();
			}
		}
	}
	else
	{
		// No move being executed
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
		for (size_t drive = AXES; drive < DRIVES; ++drive)
		{
			positions[drive] = 0.0;		// clear out extruder movement
		}
		positions[DRIVES] = dda->GetRequestedSpeed();

		// Free the DDAs for the moves we are going to skip, and work out how much extrusion they would have performed
		dda = ddaRingAddPointer;
		do
		{
			for (size_t drive = AXES; drive < DRIVES; ++drive)
			{
				positions[drive] += dda->GetEndCoordinate(drive, true);		// update the amount of extrusion we are going to skip
			}
			if (fPos == noFilePosition)
			{
				fPos = dda->GetFilePosition();
			}
			dda->Free();
			dda = dda->GetNext();
		}
		while (dda != savedDdaRingAddPointer);
	}
	else
	{
		GetCurrentUserPosition(positions, 0);		// gets positions and clears out extrusion values
	}

	return fPos;
}

uint32_t maxReps = 0;

#if 0
extern uint32_t sqSum1, sqSum2, sqCount, sqErrors, lastRes1, lastRes2;
extern uint64_t lastNum;
#endif

void Move::Diagnostics()
{
	reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Move Diagnostics:\n");
	reprap.GetPlatform()->MessageF(GENERIC_MESSAGE, "MaxReps: %u, StepErrors: %u\n", maxReps, stepErrors);
	maxReps = 0;

#if 0
	if (sqCount != 0)
	{
		reprap.GetPlatform()->AppendMessage(GENERIC_MESSAGE, "Average sqrt times %.2f, %.2f, count %u,  errors %u, last %" PRIu64 " %u %u\n",
				(float)sqSum1/sqCount, (float)sqSum2/sqCount, sqCount, sqErrors, lastNum, lastRes1, lastRes2);
		sqSum1 = sqSum2 = sqCount = sqErrors = 0;
	}
#endif
}

// These are the actual numbers we want in the positions, so don't transform them.
void Move::SetPositions(const float move[DRIVES])
{
	if (DDARingEmpty())
	{
		ddaRingAddPointer->GetPrevious()->SetPositions(move, DRIVES);
	}
	else
	{
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "SetPositions called when DDA ring not empty\n");
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

#if 0
		// We don't do inverse transforms very often, so if debugging is enabled, print them
		if (reprap.Debug(moduleMove))
		{
			debugPrintf("Inverse transformed %d %d %d to %f %f %f\n", motorPos[0], motorPos[1], motorPos[2], machinePos[0], machinePos[1], machinePos[2]);
		}
#endif

	}
	else
	{
		switch (coreXYMode)
		{
		case 1:		// CoreXY
			machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerUnit[Y_AXIS]) - (motorPos[Y_AXIS] * stepsPerUnit[X_AXIS]))
										/(2 * axisFactors[X_AXIS] * stepsPerUnit[X_AXIS] * stepsPerUnit[Y_AXIS]);
			machinePos[Y_AXIS] = ((motorPos[X_AXIS] * stepsPerUnit[Y_AXIS]) + (motorPos[Y_AXIS] * stepsPerUnit[X_AXIS]))
										/(2 * axisFactors[Y_AXIS] * stepsPerUnit[X_AXIS] * stepsPerUnit[Y_AXIS]);
			machinePos[Z_AXIS] = motorPos[Z_AXIS]/stepsPerUnit[Z_AXIS];
			break;

		case 2:		// CoreXZ
			machinePos[X_AXIS] = ((motorPos[X_AXIS] * stepsPerUnit[Z_AXIS]) - (motorPos[Z_AXIS] * stepsPerUnit[X_AXIS]))
										/(2 * axisFactors[X_AXIS] * stepsPerUnit[X_AXIS] * stepsPerUnit[Z_AXIS]);
			machinePos[Y_AXIS] = motorPos[Y_AXIS]/stepsPerUnit[Y_AXIS];
			machinePos[Z_AXIS] = ((motorPos[X_AXIS] * stepsPerUnit[Z_AXIS]) + (motorPos[Z_AXIS] * stepsPerUnit[X_AXIS]))
										/(2 * axisFactors[Z_AXIS] * stepsPerUnit[X_AXIS] * stepsPerUnit[Z_AXIS]);
			break;

		case 3:		// CoreYZ
			machinePos[X_AXIS] = motorPos[X_AXIS]/stepsPerUnit[X_AXIS];
			machinePos[Y_AXIS] = ((motorPos[Y_AXIS] * stepsPerUnit[Z_AXIS]) - (motorPos[Z_AXIS] * stepsPerUnit[Y_AXIS]))
										/(2 * axisFactors[Y_AXIS] * stepsPerUnit[Y_AXIS] * stepsPerUnit[Z_AXIS]);
			machinePos[Z_AXIS] = ((motorPos[Y_AXIS] * stepsPerUnit[Z_AXIS]) + (motorPos[Z_AXIS] * stepsPerUnit[Y_AXIS]))
										/(2 * axisFactors[Z_AXIS] * stepsPerUnit[Y_AXIS] * stepsPerUnit[Z_AXIS]);
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

// Convert Cartesian coordinates to motor steps
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
		for (size_t axis = 0; axis < AXES; ++axis)
		{
			motorPos[axis] = MotorEndPointToMachine(axis, MotorFactor(axis, machinePos));
		}
	}
}

// Calculate the movement fraction for a single axis motor of a Cartesian or CoreXY printer
float Move::MotorFactor(size_t drive, const float directionVector[]) const
{
	// NB we could simplify this code by building a matrix and using matrix multiply
	switch(drive)
	{
	case X_AXIS:
		switch(coreXYMode)
		{
		case 1:			// CoreXY
			return (directionVector[X_AXIS] * axisFactors[X_AXIS]) + (directionVector[Y_AXIS] * axisFactors[Y_AXIS]);
		case 2:			// CoreXZ
			return (directionVector[X_AXIS] * axisFactors[X_AXIS]) + (directionVector[Z_AXIS] * axisFactors[Z_AXIS]);
		default:
			break;
		}
		break;

	case Y_AXIS:
		switch(coreXYMode)
		{
		case 1:			// CoreXY
			return (directionVector[Y_AXIS] * axisFactors[Y_AXIS]) - (directionVector[X_AXIS] * axisFactors[X_AXIS]);
		case 3:			// CoreYZ
			return (directionVector[Y_AXIS] * axisFactors[Y_AXIS]) + (directionVector[Z_AXIS] * axisFactors[Z_AXIS]);
		default:
			break;
		}
		break;

	case Z_AXIS:
		switch(coreXYMode)
		{
		case 2:			// CoreXZ
			return (directionVector[Z_AXIS] * axisFactors[Z_AXIS]) - (directionVector[X_AXIS] * axisFactors[X_AXIS]);
		case 3:			// CoreYZ
			return (directionVector[Z_AXIS] * axisFactors[Z_AXIS]) - (directionVector[Y_AXIS] * axisFactors[Y_AXIS]);
		default:
			break;
		}
		break;

	default:
		break;
	}
	return directionVector[drive];
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
	switch(numBedCompensationPoints)
	{
	case 0:
		break;

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
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "BedTransform: wrong number of sample points.");
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[AXES]) const
{
	switch(numBedCompensationPoints)
	{
	case 0:
		break;

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
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "InverseBedTransform: wrong number of sample points.");
	}
}

void Move::SetIdentityTransform()
{
	numBedCompensationPoints = 0;
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
			reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Axis compensation requested for non-existent axis.\n");
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
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "SetAxisCompensation: dud axis.\n");
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
		if (l1 > TRIANGLE_ZERO && l2 > TRIANGLE_ZERO && l3 > TRIANGLE_ZERO)
		{
			return l1 * baryZBedProbePoints[i] + l2 * baryZBedProbePoints[j] + l3 * baryZBedProbePoints[4];
		}
	}
	reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Triangle interpolation: point outside all triangles!\n");
	return 0.0;
}

// Calibrate or set the bed equation after probing.
// sParam is the value of the S parameter in the G30 command that provoked this call.
void Move::FinishedBedProbing(int sParam, StringRef& reply)
{
	const int numPoints = NumberOfProbePoints();
	if (sParam < 0)
	{
		// A negative sParam just prints the probe heights
		reply.copy("Bed probe heights:");
		float sum = 0.0;
		float sumOfSquares = 0.0;
		for (size_t i = 0; (int)i < numPoints; ++i)
		{
			reply.catf(" %.3f", zBedProbePoints[i]);
			sum += zBedProbePoints[i];
			sumOfSquares += fsquare(zBedProbePoints[i]);
		}
		const float mean = sum/numPoints;
		reply.catf(", mean %.3f, deviation from mean %.3f\n", mean, sqrt(sumOfSquares/numPoints - fsquare(mean)));
	}
	else if (numPoints < sParam)
	{
		reprap.GetPlatform()->MessageF(GENERIC_MESSAGE,
				"Bed calibration error: %d factor calibration requested but only %d points provided\n", sParam, numPoints);
	}
	else
	{
		if (reprap.Debug(moduleMove))
		{
			debugPrintf("Z probe offsets:");
			float sum = 0.0;
			float sumOfSquares = 0.0;
			for (size_t i = 0; (int)i < numPoints; ++i)
			{
				debugPrintf(" %.3f", zBedProbePoints[i]);
				sum += zBedProbePoints[i];
				sumOfSquares += fsquare(zBedProbePoints[i]);
			}
			const float mean = sum/numPoints;
			debugPrintf(", mean %.3f, deviation from mean %.3f\n", mean, sqrt(sumOfSquares/numPoints - fsquare(mean)));
		}

		if (sParam == 0)
		{
			sParam = numPoints;
		}

		if (IsDeltaMode())
		{
			DoDeltaCalibration(sParam, reply);
		}
		else
		{
			SetProbedBedEquation(sParam, reply);
		}

		// Clear out the Z heights so that we don't re-use old points.
		// This allows us to use different numbers of probe point on different occasions.
		for (size_t i = 0; i < MAX_PROBE_POINTS; ++i)
		{
			probePointSet[i] &= ~zSet;
		}
	}
}

void Move::SetProbedBedEquation(size_t numPoints, StringRef& reply)
{
	switch(numPoints)
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
		break;

	case 5:
		for (size_t i = 0; i < 4; i++)
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
		break;

	default:
		reprap.GetPlatform()->MessageF(GENERIC_MESSAGE, "Bed calibration error: %d points provided but only 3, 4 and 5 supported\n", numPoints);
		return;
	}

    numBedCompensationPoints = numPoints;

	reply.copy("Bed equation fits points");
	for (size_t point = 0; point < numPoints; point++)
	{
		reply.catf(" [%.1f, %.1f, %.3f]", xBedProbePoints[point], yBedProbePoints[point], zBedProbePoints[point]);
	}
	reply.cat("\n");
}

// Perform 3-, 4-, 6- or 7-factor delta adjustment
void Move::AdjustDeltaParameters(const float v[], size_t numFactors)
{
	// Save the old home carriage heights
	float homedCarriageHeights[AXES];
	for (size_t drive = 0; drive < AXES; ++drive)
	{
		homedCarriageHeights[drive] = deltaParams.GetHomedCarriageHeight(drive);
	}

	deltaParams.Adjust(numFactors, v);	// adjust the delta parameters

	// Adjust the motor endpoints to allow for the change in endstop adjustments
	DDA *lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const int32_t *endCoordinates = lastQueuedMove->DriveCoordinates();
	const float *driveStepsPerUnit = reprap.GetPlatform()->GetDriveStepsPerUnit();

	for (size_t drive = 0; drive < AXES; ++drive)
	{
		const float heightAdjust = deltaParams.GetHomedCarriageHeight(drive) - homedCarriageHeights[drive];
		int32_t ep = endCoordinates[drive] + (int32_t)(heightAdjust * driveStepsPerUnit[drive]);
		lastQueuedMove->SetDriveCoordinate(ep, drive);
		liveEndPoints[drive] = ep;
	}

	liveCoordinatesValid = false;		// force the live XYZ position to be recalculated
}

// Do delta calibration. We adjust the three endstop corrections, and either the delta radius,
// or the X positions of the front two towers, the Y position of the rear tower, and the diagonal rod length.
void Move::DoDeltaCalibration(size_t numFactors, StringRef& reply)
{
	const size_t NumDeltaFactors = 7;		// number of delta machine factors we can adjust
	const size_t numPoints = NumberOfProbePoints();

	if (numFactors != 3 && numFactors != 4 && numFactors != 6 && numFactors != 7)
	{
		reprap.GetPlatform()->MessageF(GENERIC_MESSAGE, "Delta calibration error: %d factors requested but only 3, 4, 6 and 7 supported\n", numFactors);
		return;
	}

	if (reprap.Debug(moduleMove))
	{
		deltaParams.PrintParameters(scratchString);
		debugPrintf("%s\n", scratchString.Pointer());
	}

	// The following is for printing out the calculation time, see later
	//uint32_t startTime = reprap.GetPlatform()->GetInterruptClocks();

	// Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
	FixedMatrix<float, MAX_DELTA_PROBE_POINTS, AXES> probeMotorPositions;
	float corrections[MAX_DELTA_PROBE_POINTS];
	float initialSumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		corrections[i] = 0.0;
		float machinePos[AXES];
		float xp = xBedProbePoints[i], yp = yBedProbePoints[i];
		if (probePointSet[i] & xyCorrected)
		{
			// The point was probed with the sensor at the specified XY coordinates, so subtract the sensor offset to get the head position
			const ZProbeParameters& zparams = reprap.GetPlatform()->GetZProbeParameters();
			xp -= zparams.xOffset;
			yp -= zparams.yOffset;
		}
		machinePos[X_AXIS] = xp;
		machinePos[Y_AXIS] = yp;
		machinePos[Z_AXIS] = 0.0;

		probeMotorPositions(i, A_AXIS) = deltaParams.Transform(machinePos, A_AXIS);
		probeMotorPositions(i, B_AXIS) = deltaParams.Transform(machinePos, B_AXIS);
		probeMotorPositions(i, C_AXIS) = deltaParams.Transform(machinePos, C_AXIS);

		initialSumOfSquares += fsquare(zBedProbePoints[i]);
	}

	// Do 1 or more Newton-Raphson iterations
	unsigned int iteration = 0;
	float expectedRmsError;
	for (;;)
	{
		// Build a Nx7 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
		FixedMatrix<float, MAX_DELTA_PROBE_POINTS, NumDeltaFactors> derivativeMatrix;
		for (size_t i = 0; i < numPoints; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				derivativeMatrix(i, j) =
					deltaParams.ComputeDerivative(j, probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS));
			}
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Derivative matrix", derivativeMatrix, numPoints, numFactors);
		}

		// Now build the normal equations for least squares fitting
		FixedMatrix<float, NumDeltaFactors, NumDeltaFactors + 1> normalMatrix;
		for (size_t i = 0; i < numFactors; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				float temp = derivativeMatrix(0, i) * derivativeMatrix(0, j);
				for (size_t k = 1; k < numPoints; ++k)
				{
					temp += derivativeMatrix(k, i) * derivativeMatrix(k, j);
				}
				normalMatrix(i, j) = temp;
			}
			float temp = derivativeMatrix(0, i) * -(zBedProbePoints[0] + corrections[0]);
			for (size_t k = 1; k < numPoints; ++k)
			{
				temp += derivativeMatrix(k, i) * -(zBedProbePoints[k] + corrections[k]);
			}
			normalMatrix(i, numFactors) = temp;
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Normal matrix", normalMatrix, numFactors, numFactors + 1);
		}

		float solution[NumDeltaFactors];
		normalMatrix.GaussJordan(solution, numFactors);

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Solved matrix", normalMatrix, numFactors, numFactors + 1);
			PrintVector("Solution", solution, numFactors);

			// Calculate and display the residuals
			float residuals[MAX_DELTA_PROBE_POINTS];
			for (size_t i = 0; i < numPoints; ++i)
			{
				residuals[i] = zBedProbePoints[i];
				for (size_t j = 0; j < numFactors; ++j)
				{
					residuals[i] += solution[j] * derivativeMatrix(i, j);
				}
			}

			PrintVector("Residuals", residuals, numPoints);
		}


		AdjustDeltaParameters(solution, numFactors);

		// Calculate the expected probe heights using the new parameters
		{
			float expectedResiduals[MAX_DELTA_PROBE_POINTS];
			float sumOfSquares = 0.0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				for (size_t axis = 0; axis < AXES; ++axis)
				{
					probeMotorPositions(i, axis) += solution[axis];
				}
				float newPosition[AXES];
				deltaParams.InverseTransform(probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS), newPosition);
				corrections[i] = newPosition[Z_AXIS];
				expectedResiduals[i] = zBedProbePoints[i] + newPosition[Z_AXIS];
				sumOfSquares += fsquare(expectedResiduals[i]);
			}

			expectedRmsError = sqrt(sumOfSquares/numPoints);

			if (reprap.Debug(moduleMove))
			{
				PrintVector("Expected probe error", expectedResiduals, numPoints);
			}
		}

		// Decide whether to do another iteration Two is slightly better than one, but three doesn't improve things.
		// Alternatively, we could stop when the expected RMS error is only slightly worse than the RMS of the residuals.
		++iteration;
		if (iteration == 2) break;
	}

	// Print out the calculation time
	//debugPrintf("Time taken %dms\n", (reprap.GetPlatform()->GetInterruptClocks() - startTime) * 1000 / DDA::stepClockRate);
	if (reprap.Debug(moduleMove))
	{
		deltaParams.PrintParameters(scratchString);
		debugPrintf("%s\n", scratchString.Pointer());
	}

	reply.printf("Calibrated %d factors using %d points, deviation before %.3f after %.3f\n",
			numFactors, numPoints, sqrt(initialSumOfSquares/numPoints), expectedRmsError);
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

static void ShortDelay()
{
	for (unsigned int i = 0; i < 10; ++i)
	{
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
	}
}

// This is the function that's called by the timer interrupt to step the motors.
void Move::Interrupt()
{
	if (deltaProbing)
	{
		bool again = true;
		while (again)
		{
			if (reprap.GetPlatform()->GetZProbeResult() == EndStopHit::lowHit)
			{
				deltaProbe.Trigger();
			}

			bool dir = deltaProbe.GetDirection();
			Platform *platform = reprap.GetPlatform();
			platform->SetDirection(X_AXIS, dir);
			platform->SetDirection(Y_AXIS, dir);
			platform->SetDirection(Z_AXIS, dir);
			ShortDelay();
			platform->StepHigh(X_AXIS);
			platform->StepHigh(Y_AXIS);
			platform->StepHigh(Z_AXIS);
			ShortDelay();
			platform->StepLow(X_AXIS);
			platform->StepLow(Y_AXIS);
			platform->StepLow(Z_AXIS);
			uint32_t tim = deltaProbe.CalcNextStepTime();
			again = (tim != 0xFFFFFFFF && platform->ScheduleInterrupt(tim + deltaProbingStartTime));
		}
	}
	else
	{
		bool again = true;
		while (again && currentDda != nullptr)
		{
			again = currentDda->Step();
		}
	}
}

// This is called from the step ISR when the current move has been completed
void Move::CurrentMoveCompleted()
{
	// Save the current motor coordinates, and the machine Cartesian coordinates if known
	liveCoordinatesValid = currentDda->FetchEndPosition(const_cast<int32_t*>(liveEndPoints), const_cast<float *>(liveCoordinates));

	currentDda->Complete();
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
		reprap.GetPlatform()->ExtrudeOff();	// turn off ancilliary PWM
		return false;
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitLowStop(size_t axis, DDA* hitDDA)
{
	if (axis < AXES && !IsDeltaMode())		// should always be true
	{
		float hitPoint;
		if (axis == Z_AXIS)
		{
			// Special case of doing a G1 S1 Z move on a Cartesian printer. This is not how we normally home the Z axis, we use G30 instead.
			// But I think it used to work, so let's not break it.
			hitPoint = reprap.GetPlatform()->ZProbeStopHeight();
		}
		else
		{
			hitPoint = reprap.GetPlatform()->AxisMinimum(axis);
		}
		JustHomed(axis, hitPoint, hitDDA);
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitHighStop(size_t axis, DDA* hitDDA)
{
	if (axis < AXES)		// should always be true
	{
		float hitPoint = (IsDeltaMode())
							? deltaParams.GetHomedCarriageHeight(axis)
							        // this is a delta printer, so the motor is at the homed carriage height for this drive
							: reprap.GetPlatform()->AxisMaximum(axis);
									// this is a Cartesian printer, so we're at the maximum for this axis
		JustHomed(axis, hitPoint, hitDDA);
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::JustHomed(size_t axisHomed, float hitPoint, DDA* hitDDA)
{
	if (IsCoreXYAxis(axisHomed))
	{
		float tempCoordinates[AXES];
		for (size_t axis = 0; axis < AXES; ++axis)
		{
			tempCoordinates[axis] = hitDDA->GetEndCoordinate(axis, false);
		}
		tempCoordinates[axisHomed] = hitPoint;
		hitDDA->SetPositions(tempCoordinates, AXES);
	}
	else
	{
		hitDDA->SetDriveCoordinate(MotorEndPointToMachine(axisHomed, hitPoint), axisHomed);
	}
	reprap.GetGCodes()->SetAxisIsHomed(axisHomed);

}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
// The move has already been aborted when this is called, so the endpoints in the DDA are the current motor positions.
void Move::ZProbeTriggered(DDA* hitDDA)
{
	// Currently, we don't need to do anything here
}

// Return the untransformed machine coordinates
void Move::GetCurrentMachinePosition(float m[DRIVES], bool disableMotorMapping) const
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
}

/*static*/ float Move::MotorEndpointToPosition(int32_t endpoint, size_t drive)
{
	return ((float)(endpoint))/reprap.GetPlatform()->DriveStepsPerUnit(drive);
}

// Is filament being extruded?
bool Move::IsExtruding() const
{
	cpu_irq_disable();
	bool rslt = currentDda != nullptr && currentDda->IsPrintingMove();
	cpu_irq_enable();
	return rslt;
}

// Return the transformed machine coordinates, leaving the feed rate at m[DRIVES] alone
void Move::GetCurrentUserPosition(float m[DRIVES], uint8_t moveType) const
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

void Move::SetXBedProbePoint(size_t index, float x)
{
	if (index >= MAX_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Z probe point X index out of range.\n");
		return;
	}
	xBedProbePoints[index] = x;
	probePointSet[index] |= xSet;
}

void Move::SetYBedProbePoint(size_t index, float y)
{
	if (index >= MAX_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Z probe point Y index out of range.\n");
		return;
	}
	yBedProbePoints[index] = y;
	probePointSet[index] |= ySet;
}

void Move::SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError)
{
	if (index >= MAX_PROBE_POINTS)
	{
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Z probe point Z index out of range.\n");
	}
	else
	{
		zBedProbePoints[index] = z;
		probePointSet[index] |= zSet;
		if (wasXyCorrected)
		{
			probePointSet[index] |= xyCorrected;
		}
		else
		{
			probePointSet[index] &= ~xyCorrected;
		}
		if (wasError)
		{
			probePointSet[index] |= probeError;
		}
		else
		{
			probePointSet[index] &= ~probeError;
		}
	}
}

float Move::XBedProbePoint(size_t index) const
{
	return xBedProbePoints[index];
}

float Move::YBedProbePoint(size_t index) const
{
	return yBedProbePoints[index];
}

float Move::ZBedProbePoint(size_t index) const
{
	return zBedProbePoints[index];
}

bool Move::AllProbeCoordinatesSet(int index) const
{
	return (probePointSet[index] & (xSet | ySet | zSet)) == (xSet | ySet | zSet);
}

bool Move::XYProbeCoordinatesSet(int index) const
{
	return (probePointSet[index]  & xSet) && (probePointSet[index]  & ySet);
}

size_t Move::NumberOfProbePoints() const
{
	for(size_t i = 0; i < MAX_PROBE_POINTS; i++)
	{
		if(!AllProbeCoordinatesSet(i))
		{
			return i;
		}
	}
	return MAX_PROBE_POINTS;
}

size_t Move::NumberOfXYProbePoints() const
{
	for(size_t i = 0; i < MAX_PROBE_POINTS; i++)
	{
		if(!XYProbeCoordinatesSet(i))
		{
			return i;
		}
	}
	return MAX_PROBE_POINTS;
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

// Return true if the specified axis shares its motors with another. Safe to call for extruders as well as axes.
bool Move::IsCoreXYAxis(size_t axis) const
{
	switch(coreXYMode)
	{
	case 1:
		return axis == X_AXIS || axis == Y_AXIS;
	case 2:
		return axis == X_AXIS || axis == Z_AXIS;
	case 3:
		return axis == Y_AXIS || axis == Z_AXIS;
	default:
		return false;
	}
}

// Do a delta probe returning -1 if still probing, 0 if failed, 1 if success
int Move::DoDeltaProbe(float frequency, float amplitude, float rate, float distance)
{
	if (deltaProbing)
	{
		if (deltaProbe.Finished())
		{
			deltaProbing = false;
			return (deltaProbe.Overran()) ? 0 : 1;
		}
	}
	else
	{
		if (currentDda != nullptr || !DDARingEmpty())
		{
			return 0;
		}
		if (!deltaProbe.Init(frequency, amplitude, rate, distance))
		{
			return 0;
		}

		const uint32_t firstInterruptTime = deltaProbe.Start();
		if (firstInterruptTime != 0xFFFFFFFF)
		{
			Platform *platform = reprap.GetPlatform();
			platform->EnableDrive(X_AXIS);
			platform->EnableDrive(Y_AXIS);
			platform->EnableDrive(Z_AXIS);
			deltaProbing = true;
			iState = IdleState::busy;
			const irqflags_t flags = cpu_irq_save();
			deltaProbingStartTime = platform->GetInterruptClocks();
			if (platform->ScheduleInterrupt(firstInterruptTime + deltaProbingStartTime))
			{
				Interrupt();
			}
			cpu_irq_restore(flags);
		}
	}
	return -1;
}

/*static*/ void Move::PrintMatrix(const char* s, const MathMatrix<float>& m, size_t maxRows, size_t maxCols)
{
	debugPrintf("%s\n", s);
	if (maxRows == 0)
	{
		maxRows = m.rows();
	}
	if (maxCols == 0)
	{
		maxCols = m.cols();
	}

	for (size_t i = 0; i < maxRows; ++i)
	{
		for (size_t j = 0; j < maxCols; ++j)
		{
			debugPrintf("%7.3f%c", m(i, j), (j == maxCols - 1) ? '\n' : ' ');
		}
	}
}

/*static*/ void Move::PrintVector(const char *s, const float *v, size_t numElems)
{
	debugPrintf("%s:", s);
	for (size_t i = 0; i < numElems; ++i)
	{
		debugPrintf(" %7.3f", v[i]);
	}
	debugPrintf("\n");
}

// End
