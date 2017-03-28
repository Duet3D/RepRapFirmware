/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "Move.h"
#include "Platform.h"
#include "RepRap.h"

Move::Move(Platform* p, GCodes* g) : currentDda(NULL), grid(zBedProbePoints), scheduledMoves(0), completedMoves(0)
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
	for (size_t axis = 0; axis < MAX_AXES; ++axis)
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
	stepErrors = 0;
	numLookaheadUnderruns = numPrepareUnderruns = 0;

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
	for (size_t point = 0; point < ARRAY_SIZE(zBedProbePoints); point++)
	{
		if (point < 4)
		{
			xBedProbePoints[point] = (0.1 + 0.8 * (float)(point%2)) * reprap.GetPlatform()->AxisMaximum(X_AXIS);
			yBedProbePoints[point] = (0.1 + 0.8 * (float)(point/2)) * reprap.GetPlatform()->AxisMaximum(Y_AXIS);
		}
		else if (point == 4)
		{
			xBedProbePoints[point] = 0.5 * reprap.GetPlatform()->AxisMaximum(X_AXIS);
			yBedProbePoints[point] = 0.5 * reprap.GetPlatform()->AxisMaximum(Y_AXIS);
		}
		zBedProbePoints[point] = 0.0;
		if (point < ARRAY_SIZE(probePointSet))
		{
			probePointSet[point] = unset;
		}
	}

	xRectangle = 1.0/(0.8 * reprap.GetPlatform()->AxisMaximum(X_AXIS));
	yRectangle = xRectangle;
	useTaper = false;

	longWait = reprap.GetPlatform()->Time();
	idleTimeout = DEFAULT_IDLE_TIMEOUT;
	iState = IdleState::idle;
	idleCount = 0;

	simulationMode = 0;
	simulationTime = 0.0;
	longestGcodeWaitInterval = 0;
	waitingForMove = false;

	active = true;
}

void Move::Exit()
{
	reprap.GetPlatform()->Message(HOST_MESSAGE, "Move class exited.\n");
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
		if (ddaRingCheckPointer->Free())
		{
			++numLookaheadUnderruns;
		}
		ddaRingCheckPointer = ddaRingCheckPointer->GetNext();
	}

	// See if we can add another move to the ring
	if (
#if SUPPORT_ROLAND
		   !reprap.GetRoland()->Active() &&
#endif
		   ddaRingAddPointer->GetState() == DDA::empty
		&& ddaRingAddPointer->GetNext()->GetState() != DDA::provisional		// function Prepare needs to access the endpoints in the previous move, so don't change them
	   )
	{
		// In order to react faster to speed and extrusion rate changes, only add more moves if the total duration of
		// all un-frozen moves is less than 2 seconds, or the total duration of all but the first un-frozen move is
		// less than 0.5 seconds.
		const DDA *dda = ddaRingAddPointer;
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
				if (waitingForMove)
				{
					waitingForMove = false;
					const uint32_t timeWaiting = millis() - gcodeWaitStartTime;
					if (timeWaiting > longestGcodeWaitInterval)
					{
						longestGcodeWaitInterval = timeWaiting;
					}
				}
				// We have a new move
				if (simulationMode < 2)		// in simulation mode 2 and higher, we don't process incoming moves beyond this point
				{
					const bool doMotorMapping = (nextMove.moveType == 0) || (nextMove.moveType == 1 && !IsDeltaMode());
					if (doMotorMapping)
					{
						Transform(nextMove.coords, nextMove.xAxes, nextMove.moveType == 0);
					}
					if (ddaRingAddPointer->Init(nextMove, doMotorMapping))
					{
						ddaRingAddPointer = ddaRingAddPointer->GetNext();
						idleCount = 0;
						scheduledMoves++;
					}
				}
			}
			else
			{
				// We wanted another move, but none was available
				if (currentDda != nullptr && !waitingForMove)
				{
					gcodeWaitStartTime = millis();
					waitingForMove = true;
				}
			}
		}
	}

	if (!deltaProbing)
	{
		// See whether we need to kick off a move
		if (currentDda == nullptr)
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
					if (simulationMode != 0)
					{
						currentDda = dda;								// pretend we are executing this move
					}
					else
					{
						Platform::DisableStepInterrupt();					// should be disabled already because we weren't executing a move, but make sure
						if (StartNextMove(Platform::GetInterruptClocks()))	// start the next move
						{
							Interrupt();
						}
					}
					iState = IdleState::busy;
				}
				else if (!simulationMode != 0 && iState == IdleState::busy && !reprap.GetGCodes()->IsPaused() && idleTimeout > 0.0)
				{
					lastMoveTime = reprap.GetPlatform()->Time();			// record when we first noticed that the machine was idle
					iState = IdleState::timing;
				}
				else if (!simulationMode != 0 && iState == IdleState::timing && reprap.GetPlatform()->Time() - lastMoveTime >= idleTimeout)
				{
					reprap.GetPlatform()->SetDriversIdle();					// put all drives in idle hold
					iState = IdleState::idle;
				}
			}
		}

		DDA *cdda = currentDda;											// currentDda is volatile, so copy it
		if (cdda != nullptr)
		{
			// See whether we need to prepare any moves
			int32_t preparedTime = 0;
			uint32_t preparedCount = 0;
			DDA::DDAState st;
			while ((st = cdda->GetState()) == DDA::completed || st == DDA::executing || st == DDA::frozen)
			{
				preparedTime += cdda->GetTimeLeft();
				++preparedCount;
				cdda = cdda->GetNext();
				if (cdda == ddaRingAddPointer)
				{
					break;
				}
			}

			// If the number of prepared moves will execute in less than the minimum time, prepare another move
			while (st == DDA::provisional
					&& preparedTime < (int32_t)(DDA::stepClockRate/8)	// prepare moves one eighth of a second ahead of when they will be needed
					&& preparedCount < DdaRingLength/2					// but don't prepare more than half the ring
				  )
			{
				cdda->Prepare();
				preparedTime += cdda->GetTimeLeft();
				++preparedCount;
				cdda = cdda->GetNext();
				st = cdda->GetState();
			}

			// If we are simulating and the move pipeline is reasonably full, simulate completion of the current move
			if (simulationMode != 0 && idleCount >= 10)
			{
				// Simulate completion of the current move
//DEBUG
//currentDda->DebugPrint();
				simulationTime += currentDda->CalcTime();
				CurrentMoveCompleted();
			}
		}
	}

	reprap.GetPlatform()->ClassReport(longWait);
}

// Pause the print as soon as we can.
// Returns the file position of the first queue move we are going to skip, or noFilePosition we we are not skipping any moves.
// We update 'positions' to the positions and feed rate expected for the next move, and the amount of extrusion in the moves we skipped.
// If we are not skipping any moves then the feed rate is left alone, therefore the caller should set this up first.
FilePosition Move::PausePrint(float positions[DRIVES], float& pausedFeedRate, uint32_t xAxes)
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
	FilePosition fPos = noFilePosition;
	if (dda != nullptr)
	{
		// A move is being executed. See if we can safely pause at the end of it.
		if (dda->CanPauseAfter())
		{
			fPos = dda->GetFilePosition();
			ddaRingAddPointer = dda->GetNext();
		}
		else
		{
			// We can't safely pause after the currently-executing move because its end speed is too high so we may miss steps.
			// Search for the next move that we can safely stop after.
			dda = ddaRingGetPointer;
			while (dda != ddaRingAddPointer)
			{
				if (dda->CanPauseAfter())
				{
					fPos = dda->GetFilePosition();
					ddaRingAddPointer = dda->GetNext();
					if (ddaRingAddPointer->GetState() == DDA::frozen)
					{
						// Change the state so that the ISR won't start executing this move
						(void)ddaRingAddPointer->Free();
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

	if (ddaRingAddPointer != savedDdaRingAddPointer)
	{
		const size_t numAxes = reprap.GetGCodes()->GetNumAxes();

		// We are going to skip some moves. dda points to the last move we are going to print.
		for (size_t axis = 0; axis < numAxes; ++axis)
		{
			positions[axis] = dda->GetEndCoordinate(axis, false);
		}
		for (size_t drive = numAxes; drive < DRIVES; ++drive)
		{
			positions[drive] = 0.0;		// clear out extruder movement
		}
		pausedFeedRate = dda->GetRequestedSpeed();

		// Free the DDAs for the moves we are going to skip, and work out how much extrusion they would have performed
		dda = ddaRingAddPointer;
		do
		{
			for (size_t drive = numAxes; drive < DRIVES; ++drive)
			{
				positions[drive] += dda->GetEndCoordinate(drive, true);		// update the amount of extrusion we are going to skip
			}
			(void)dda->Free();
			dda = dda->GetNext();
			scheduledMoves--;
		}
		while (dda != savedDdaRingAddPointer);
	}
	else
	{
		GetCurrentUserPosition(positions, 0, xAxes);		// gets positions and clears out extrusion values
	}

	return fPos;
}

uint32_t maxReps = 0;

#if 0
// For debugging
extern uint32_t sqSum1, sqSum2, sqCount, sqErrors, lastRes1, lastRes2;
extern uint64_t lastNum;
#endif

void Move::Diagnostics(MessageType mtype)
{
	Platform * const p = reprap.GetPlatform();
	p->Message(mtype, "=== Move ===\n");
	p->MessageF(mtype, "MaxReps: %u, StepErrors: %u, MaxWait: %ums, Underruns: %u, %u\n",
						maxReps, stepErrors, longestGcodeWaitInterval, numLookaheadUnderruns, numPrepareUnderruns);
	maxReps = 0;
	numLookaheadUnderruns = numPrepareUnderruns = 0;
	longestGcodeWaitInterval = 0;

	reprap.GetPlatform()->MessageF(mtype, "Scheduled moves: %u, completed moves: %u\n", scheduledMoves, completedMoves);

	// Show the current probe position heights and type of bed compensation in use
	p->Message(mtype, "Bed compensation in use: ");
	if (numBedCompensationPoints == 0)
	{
		p->MessageF(mtype, "%s\n", (grid.UsingHeightMap()) ? "mesh" : "none");
	}
	else
	{
		p->MessageF(mtype, "%d point\n", numBedCompensationPoints);
	}
	p->Message(mtype, "Bed probe heights:");
	// To keep the response short so that it doesn't get truncates when sending it via HTTP, we only show the first 5 bed probe points
	for (size_t i = 0; i < 5; ++i)
	{
		p->MessageF(mtype, " %.3f", ZBedProbePoint(i));
	}
	p->Message(mtype, "\n");

#if DDA_LOG_PROBE_CHANGES
	// Temporary code to print Z probe trigger positions
	p->Message(mtype, "Probe change coordinates:");
	char ch = ' ';
	for (size_t i = 0; i < DDA::numLoggedProbePositions; ++i)
	{
		float xyzPos[MIN_AXES];
		MachineToEndPoint(DDA::loggedProbePositions + (MIN_AXES * i), xyzPos, MIN_AXES);
		p->MessageF(mtype, "%c%.2f,%.2f", ch, xyzPos[X_AXIS], xyzPos[Y_AXIS]);
		ch = ',';
	}
	p->Message(mtype, "\n");
#endif

#if 0
	// For debugging
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
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	for (size_t drive = numAxes; drive < numDrives; ++drive)
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
	const float * const stepsPerUnit = reprap.GetPlatform()->GetDriveStepsPerUnit();

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

	// Convert any additional axes and the extruders
	for (size_t drive = MIN_AXES; drive < numDrives; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerUnit[drive];
	}
}

// Convert Cartesian coordinates to motor steps
void Move::MotorTransform(const float machinePos[MAX_AXES], int32_t motorPos[MAX_AXES]) const
{
	if (IsDeltaMode())
	{
		for (size_t axis = 0; axis < DELTA_AXES; ++axis)
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
		const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
		for (size_t axis = 0; axis < numAxes; ++axis)
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
void Move::AxisTransform(float xyzPoint[MAX_AXES]) const
{
	//TODO should we transform U axis instead of/as well as X if we are in dual carriage mode?
	xyzPoint[X_AXIS] += tanXY*xyzPoint[Y_AXIS] + tanXZ*xyzPoint[Z_AXIS];
	xyzPoint[Y_AXIS] += tanYZ*xyzPoint[Z_AXIS];
}

// Invert the Axis transform AFTER the bed transform
void Move::InverseAxisTransform(float xyzPoint[MAX_AXES]) const
{
	//TODO should we transform U axis instead of/as well as X if we are in dual carriage mode?
	xyzPoint[Y_AXIS] -= tanYZ*xyzPoint[Z_AXIS];
	xyzPoint[X_AXIS] -= (tanXY*xyzPoint[Y_AXIS] + tanXZ*xyzPoint[Z_AXIS]);
}

void Move::Transform(float xyzPoint[MAX_AXES], uint32_t xAxes, bool useBedCompensation) const
{
	AxisTransform(xyzPoint);
	if (useBedCompensation)
	{
		BedTransform(xyzPoint, xAxes);
	}
}

void Move::InverseTransform(float xyzPoint[MAX_AXES], uint32_t xAxes) const
{
	InverseBedTransform(xyzPoint, xAxes);
	InverseAxisTransform(xyzPoint);
}

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[MAX_AXES], uint32_t xAxes) const
{
	if (!useTaper || xyzPoint[Z_AXIS] < taperHeight)
	{
		float zCorrection = 0.0;
		const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
		unsigned int numXAxes = 0;

		// Transform the Z coordinate based on the average correction for each axis used as an X axis.
		// We are assuming that the tool Y offsets are small enough to be ignored.
		for (uint32_t axis = 0; axis < numAxes; ++axis)
		{
			if ((xAxes & (1u << axis)) != 0)
			{
				const float xCoord = xyzPoint[axis];
				switch(numBedCompensationPoints)
				{
				case 0:
					zCorrection += grid.GetInterpolatedHeightError(xCoord, xyzPoint[Y_AXIS]);
					break;

				case 3:
					zCorrection += aX * xCoord + aY * xyzPoint[Y_AXIS] + aC;
					break;

				case 4:
					zCorrection += SecondDegreeTransformZ(xCoord, xyzPoint[Y_AXIS]);
					break;

				case 5:
					zCorrection += TriangleZ(xCoord, xyzPoint[Y_AXIS]);
					break;

				default:
					break;
				}
				++numXAxes;
			}
		}

		if (numXAxes > 1)
		{
			zCorrection /= numXAxes;			// take an average
		}

		xyzPoint[Z_AXIS] += (useTaper) ? (taperHeight - xyzPoint[Z_AXIS]) * recipTaperHeight * zCorrection : zCorrection;
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[MAX_AXES], uint32_t xAxes) const
{
	float zCorrection = 0.0;
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	unsigned int numXAxes = 0;

	// Transform the Z coordinate based on the average correction for each axis used as an X axis.
	// We are assuming that the tool Y offsets are small enough to be ignored.
	for (uint32_t axis = 0; axis < numAxes; ++axis)
	{
		if ((xAxes & (1u << axis)) != 0)
		{
			const float xCoord = xyzPoint[axis];
			switch(numBedCompensationPoints)
			{
			case 0:
				zCorrection += grid.GetInterpolatedHeightError(xCoord, xyzPoint[Y_AXIS]);
				break;

			case 3:
				zCorrection += aX * xCoord + aY * xyzPoint[Y_AXIS] + aC;
				break;

			case 4:
				zCorrection += SecondDegreeTransformZ(xCoord, xyzPoint[Y_AXIS]);
				break;

			case 5:
				zCorrection += TriangleZ(xCoord, xyzPoint[Y_AXIS]);
				break;

			default:
				break;
			}
			++numXAxes;
		}
	}

	if (numXAxes > 1)
	{
		zCorrection /= numXAxes;					// take an average
	}

	if (!useTaper || zCorrection >= taperHeight)	// need check on zCorrection to avoid possible divide by zero
	{
		xyzPoint[Z_AXIS] -= zCorrection;
	}
	else
	{
		const float zreq = (xyzPoint[Z_AXIS] - zCorrection)/(1.0 - (zCorrection * recipTaperHeight));
		if (zreq < taperHeight)
		{
			xyzPoint[Z_AXIS] = zreq;
		}
	}
}

void Move::SetIdentityTransform()
{
	numBedCompensationPoints = 0;
	grid.ClearGridHeights();
	grid.UseHeightMap(false);
}

void Move::SetTaperHeight(float h)
{
	useTaper = (h > 1.0);
	if (useTaper)
	{
		taperHeight = h;
		recipTaperHeight = 1.0/h;
	}
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
	const float y23 = baryYBedProbePoints[p2] - baryYBedProbePoints[p3];
	const float x3 = x - baryXBedProbePoints[p3];
	const float x32 = baryXBedProbePoints[p3] - baryXBedProbePoints[p2];
	const float y3 = y - baryYBedProbePoints[p3];
	const float x13 = baryXBedProbePoints[p1] - baryXBedProbePoints[p3];
	const float y13 = baryYBedProbePoints[p1] - baryYBedProbePoints[p3];
	const float iDet = 1.0 / (y23 * x13 + x32 * y13);
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
		const size_t j = (i + 1) % 4;
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
			if ((probePointSet[i] & (xSet | ySet | zSet | probeError)) != (xSet | ySet | zSet))
			{
				reply.cat(" failed");
			}
			else
			{
				reply.catf(" %.3f", zBedProbePoints[i]);
				sum += zBedProbePoints[i];
				sumOfSquares += fsquare(zBedProbePoints[i]);
			}
		}
		const float mean = sum/numPoints;
		reply.catf(", mean %.3f, deviation from mean %.3f", mean, sqrt(sumOfSquares/numPoints - fsquare(mean)));
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

		// Check that all probe points are set and there were no errors
		bool hadError = false;
		for (size_t i = 0; (int)i < numPoints; ++i)
		{
			if ((probePointSet[i] & (xSet | ySet | zSet | probeError)) != (xSet | ySet | zSet))
			{
				hadError = true;
				break;
			}
		}

		if (hadError)
		{
			reply.cat("Compensation or calibration cancelled due to probing errors");
		}
		else if (IsDeltaMode())
		{
			DoDeltaCalibration(sParam, reply);
		}
		else
		{
			SetProbedBedEquation(sParam, reply);
		}
	}

	// Clear out the Z heights so that we don't re-use old points.
	// This allows us to use different numbers of probe point on different occasions.
	for (size_t i = 0; i < MaxProbePoints; ++i)
	{
		probePointSet[i] &= ~zSet;
	}
}

// Check that the probe points are in the right order
bool Move::GoodProbePointOrdering(size_t numPoints) const
{
	if (numPoints >= 2 && yBedProbePoints[1] <= yBedProbePoints[0])
	{
		return false;
	}
	if (numPoints >= 3 && xBedProbePoints[2] <= xBedProbePoints[1])
	{
		return false;
	}
	if (numPoints >= 4 && yBedProbePoints[3] >= yBedProbePoints[2])
	{
		return false;
	}
	if (numPoints >= 4 && xBedProbePoints[0] >= xBedProbePoints[3])
	{
		return false;
	}
	if (numPoints >= 5
		&& (   xBedProbePoints[4] <= xBedProbePoints[0]
			|| xBedProbePoints[4] <= xBedProbePoints[1]
			|| xBedProbePoints[4] >= xBedProbePoints[2]
			|| xBedProbePoints[4] >= xBedProbePoints[3]
			|| yBedProbePoints[4] <= yBedProbePoints[0]
			|| yBedProbePoints[4] >= yBedProbePoints[1]
			|| yBedProbePoints[4] >= yBedProbePoints[2]
			|| yBedProbePoints[4] <= yBedProbePoints[3]
		   )
	   )
	{
		return false;
	}
	return true;
}

void Move::SetProbedBedEquation(size_t numPoints, StringRef& reply)
{
	if (!GoodProbePointOrdering(numPoints))
	{
		reply.printf("Error: probe points P0 to P%u must be in clockwise order starting near X=0 Y=0", min<unsigned int>(numPoints, 4) - 1);
		if (numPoints >= 5)
		{
			reply.cat(" and P4 must be near the centre");
		}
	}
	else
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
	}
	reply.cat("\n");
}

// Perform 3-, 4-, 6- or 7-factor delta adjustment
void Move::AdjustDeltaParameters(const floatc_t v[], size_t numFactors)
{
	// Save the old home carriage heights
	float homedCarriageHeights[DELTA_AXES];
	for (size_t drive = 0; drive < DELTA_AXES; ++drive)
	{
		homedCarriageHeights[drive] = deltaParams.GetHomedCarriageHeight(drive);
	}

	deltaParams.Adjust(numFactors, v);	// adjust the delta parameters

	// Adjust the motor endpoints to allow for the change in endstop adjustments
	DDA *lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const int32_t *endCoordinates = lastQueuedMove->DriveCoordinates();
	const float *driveStepsPerUnit = reprap.GetPlatform()->GetDriveStepsPerUnit();

	for (size_t drive = 0; drive < DELTA_AXES; ++drive)
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
	const size_t NumDeltaFactors = 9;		// maximum number of delta machine factors we can adjust
	const size_t numPoints = NumberOfProbePoints();

	if (numFactors < 3 || numFactors > NumDeltaFactors || numFactors == 5)
	{
		reprap.GetPlatform()->MessageF(GENERIC_MESSAGE, "Delta calibration error: %d factors requested but only 3, 4, 6, 7, 8 and 9 supported\n", numFactors);
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
	FixedMatrix<floatc_t, MaxDeltaCalibrationPoints, DELTA_AXES> probeMotorPositions;
	floatc_t corrections[MaxDeltaCalibrationPoints];
	float initialSumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		corrections[i] = 0.0;
		float machinePos[DELTA_AXES];
		float zp = GetProbeCoordinates(i, machinePos[X_AXIS], machinePos[Y_AXIS], (probePointSet[i] & xyCorrected) != 0);
		machinePos[Z_AXIS] = 0.0;

		probeMotorPositions(i, A_AXIS) = deltaParams.Transform(machinePos, A_AXIS);
		probeMotorPositions(i, B_AXIS) = deltaParams.Transform(machinePos, B_AXIS);
		probeMotorPositions(i, C_AXIS) = deltaParams.Transform(machinePos, C_AXIS);

		initialSumOfSquares += fsquare(zp);
	}

	// Do 1 or more Newton-Raphson iterations
	unsigned int iteration = 0;
	float expectedRmsError;
	for (;;)
	{
		// Build a Nx9 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
		FixedMatrix<floatc_t, MaxDeltaCalibrationPoints, NumDeltaFactors> derivativeMatrix;
		for (size_t i = 0; i < numPoints; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				const size_t adjustedJ = (numFactors == 8 && j >= 6) ? j + 1 : j;		// skip diagonal rod length if doing 8-factor calibration
				derivativeMatrix(i, j) =
					deltaParams.ComputeDerivative(adjustedJ, probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS));
			}
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Derivative matrix", derivativeMatrix, numPoints, numFactors);
		}

		// Now build the normal equations for least squares fitting
		FixedMatrix<floatc_t, NumDeltaFactors, NumDeltaFactors + 1> normalMatrix;
		for (size_t i = 0; i < numFactors; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				floatc_t temp = derivativeMatrix(0, i) * derivativeMatrix(0, j);
				for (size_t k = 1; k < numPoints; ++k)
				{
					temp += derivativeMatrix(k, i) * derivativeMatrix(k, j);
				}
				normalMatrix(i, j) = temp;
			}
			floatc_t temp = derivativeMatrix(0, i) * -(zBedProbePoints[0] + corrections[0]);
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

		floatc_t solution[NumDeltaFactors];
		normalMatrix.GaussJordan(solution, numFactors);

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Solved matrix", normalMatrix, numFactors, numFactors + 1);
			PrintVector("Solution", solution, numFactors);

			// Calculate and display the residuals
			floatc_t residuals[MaxDeltaCalibrationPoints];
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
			floatc_t expectedResiduals[MaxDeltaCalibrationPoints];
			floatc_t sumOfSquares = 0.0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				for (size_t axis = 0; axis < DELTA_AXES; ++axis)
				{
					probeMotorPositions(i, axis) += solution[axis];
				}
				float newPosition[DELTA_AXES];
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

		// Decide whether to do another iteration. Two is slightly better than one, but three doesn't improve things.
		// Alternatively, we could stop when the expected RMS error is only slightly worse than the RMS of the residuals.
		++iteration;
		if (iteration == 2)
		{
			break;
		}
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

// This is the function that is called by the timer interrupt to step the motors when we are using the experimental delta probe.
// The movements are quite slow so it is not time-critical.
void Move::DeltaProbeInterrupt()
{
	bool again;
	do
	{
		if (reprap.GetPlatform()->GetZProbeResult() == EndStopHit::lowHit)
		{
			deltaProbe.Trigger();
		}

		const bool dir = deltaProbe.GetDirection();
		Platform * const platform = reprap.GetPlatform();
		platform->SetDirection(X_AXIS, dir);
		platform->SetDirection(Y_AXIS, dir);
		platform->SetDirection(Z_AXIS, dir);
		ShortDelay();
		const uint32_t steppersMoving = platform->GetDriversBitmap(X_AXIS) | platform->GetDriversBitmap(Y_AXIS) | platform->GetDriversBitmap(Z_AXIS);
		Platform::StepDriversHigh(steppersMoving);
		ShortDelay();
		Platform::StepDriversLow();
		const uint32_t tim = deltaProbe.CalcNextStepTime();
		again = (tim != 0xFFFFFFFF && platform->ScheduleInterrupt(tim + deltaProbingStartTime));
	} while (again);
}

// This is called from the step ISR when the current move has been completed
void Move::CurrentMoveCompleted()
{
	// Save the current motor coordinates, and the machine Cartesian coordinates if known
	liveCoordinatesValid = currentDda->FetchEndPosition(const_cast<int32_t*>(liveEndPoints), const_cast<float *>(liveCoordinates));

	currentDda->Complete();
	currentDda = nullptr;
	ddaRingGetPointer = ddaRingGetPointer->GetNext();
	completedMoves++;
}

// Try to start another move. Must be called with interrupts disabled, to avoid a race condition.
bool Move::TryStartNextMove(uint32_t startTime)
{
	const DDA::DDAState st = ddaRingGetPointer->GetState();
	if (st == DDA::frozen)
	{
		return StartNextMove(startTime);
	}
	else
	{
		if (st == DDA::provisional)
		{
			// There are more moves available, but they are not prepared yet. Signal an underrun.
			++numPrepareUnderruns;
		}
		reprap.GetPlatform()->ExtrudeOff();	// turn off ancilliary PWM
		return false;
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitLowStop(size_t axis, DDA* hitDDA)
{
	if (axis < reprap.GetGCodes()->GetNumAxes() && !IsDeltaMode())		// should always be true
	{
		JustHomed(axis, reprap.GetPlatform()->AxisMinimum(axis), hitDDA);
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitHighStop(size_t axis, DDA* hitDDA)
{
	if (axis < reprap.GetGCodes()->GetNumAxes())		// should always be true
	{
		const float hitPoint = (IsDeltaMode())
								? deltaParams.GetHomedCarriageHeight(axis)	// this is a delta printer, so the motor is at the homed carriage height for this drive
								: reprap.GetPlatform()->AxisMaximum(axis);	// this is a Cartesian printer, so we're at the maximum for this axis
		JustHomed(axis, hitPoint, hitDDA);
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::JustHomed(size_t axisHomed, float hitPoint, DDA* hitDDA)
{
	if (IsCoreXYAxis(axisHomed))
	{
		float tempCoordinates[CART_AXES];
		for (size_t axis = 0; axis < CART_AXES; ++axis)
		{
			tempCoordinates[axis] = hitDDA->GetEndCoordinate(axis, false);
		}
		tempCoordinates[axisHomed] = hitPoint;
		hitDDA->SetPositions(tempCoordinates, CART_AXES);
	}
	else
	{
		hitDDA->SetDriveCoordinate(MotorEndPointToMachine(axisHomed, hitPoint), axisHomed);
	}
	reprap.GetGCodes()->SetAxisIsHomed(axisHomed);

}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR should be declared 'volatile'.
// The move has already been aborted when this is called, so the endpoints in the DDA are the current motor positions.
void Move::ZProbeTriggered(DDA* hitDDA)
{
	reprap.GetGCodes()->MoveStoppedByZProbe();
}

// Return the untransformed machine coordinates
void Move::GetCurrentMachinePosition(float m[DRIVES], bool disableMotorMapping) const
{
	DDA * const lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();
	for (size_t i = 0; i < DRIVES; i++)
	{
		if (i < numAxes)
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
	const bool rslt = currentDda != nullptr && currentDda->IsPrintingMove();
	cpu_irq_enable();
	return rslt;
}

// Return the transformed machine coordinates
void Move::GetCurrentUserPosition(float m[DRIVES], uint8_t moveType, uint32_t xAxes) const
{
	GetCurrentMachinePosition(m, moveType == 2 || (moveType == 1 && IsDeltaMode()));
	if (moveType == 0)
	{
		InverseTransform(m, xAxes);
	}
}

// Return the current live XYZ and extruder coordinates
// Interrupts are assumed enabled on entry, so do not call this from an ISR
void Move::LiveCoordinates(float m[DRIVES], uint32_t xAxes)
{
	// The live coordinates and live endpoints are modified by the ISR, so be careful to get a self-consistent set of them
	const size_t numAxes = reprap.GetGCodes()->GetNumAxes();		// do this before we disable interrupts
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
		memcpy(m + numAxes, const_cast<const float *>(liveCoordinates + numAxes), sizeof(m[0]) * (DRIVES - numAxes));
		int32_t tempEndPoints[MAX_AXES];
		memcpy(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints));
		cpu_irq_enable();
		MachineToEndPoint(tempEndPoints, m, numAxes);		// this is slow, so do it with interrupts enabled

		// If the ISR has not updated the endpoints, store the live coordinates back so that we don't need to do it again
		cpu_irq_disable();
		if (memcmp(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints)) == 0)
		{
			memcpy(const_cast<float *>(liveCoordinates), m, sizeof(m[0]) * numAxes);
			liveCoordinatesValid = true;
		}
		cpu_irq_enable();
	}
	InverseTransform(m, xAxes);
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
	EndPointToMachine(coords, const_cast<int32_t *>(liveEndPoints), reprap.GetGCodes()->GetNumAxes());
	cpu_irq_enable();
}

void Move::ResetExtruderPositions()
{
	cpu_irq_disable();
	for(size_t eDrive = reprap.GetGCodes()->GetNumAxes(); eDrive < DRIVES; eDrive++)
	{
		liveCoordinates[eDrive] = 0.0;
	}
	cpu_irq_enable();
}

void Move::SetXBedProbePoint(size_t index, float x)
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Z probe point X index out of range.\n");
		return;
	}
	xBedProbePoints[index] = x;
	probePointSet[index] |= xSet;
}

void Move::SetYBedProbePoint(size_t index, float y)
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Z probe point Y index out of range.\n");
		return;
	}
	yBedProbePoints[index] = y;
	probePointSet[index] |= ySet;
}

void Move::SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError)
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform()->Message(GENERIC_MESSAGE, "Z probe point Z index out of range.\n");
	}
	else
	{
		grid.UseHeightMap(false);
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

// This returns the (X, Y) points to probe the bed at probe point count.  When probing, it returns false.
// If called after probing has ended it returns true, and the Z coordinate probed is also returned.
// If 'wantNozzlePosition is true then we return the nozzle position when the point is probed, else we return the probe point itself
float Move::GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const
{
	x = xBedProbePoints[count];
	y = yBedProbePoints[count];
	if (wantNozzlePosition)
	{
		const ZProbeParameters& rp = reprap.GetPlatform()->GetCurrentZProbeParameters();
		x -= rp.xOffset;
		y -= rp.yOffset;
	}
	return zBedProbePoints[count];
}

size_t Move::NumberOfProbePoints() const
{
	for(size_t i = 0; i < MaxProbePoints; i++)
	{
		if(!AllProbeCoordinatesSet(i))
		{
			return i;
		}
	}
	return MaxProbePoints;
}

size_t Move::NumberOfXYProbePoints() const
{
	for(size_t i = 0; i < MaxProbePoints; i++)
	{
		if(!XYProbeCoordinatesSet(i))
		{
			return i;
		}
	}
	return MaxProbePoints;
}

// Enter or leave simulation mode
void Move::Simulate(uint8_t simMode)
{
	simulationMode = simMode;
	if (simMode)
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
			Platform * const platform = reprap.GetPlatform();
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

/*static*/ void Move::PrintMatrix(const char* s, const MathMatrix<floatc_t>& m, size_t maxRows, size_t maxCols)
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
			debugPrintf("%7.4f%c", m(i, j), (j == maxCols - 1) ? '\n' : ' ');
		}
	}
}

/*static*/ void Move::PrintVector(const char *s, const floatc_t *v, size_t numElems)
{
	debugPrintf("%s:", s);
	for (size_t i = 0; i < numElems; ++i)
	{
		debugPrintf(" %7.4f", v[i]);
	}
	debugPrintf("\n");
}

// End
