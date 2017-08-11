/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "Move.h"
#include "Platform.h"
#include "RepRap.h"
#include "Kinematics/LinearDeltaKinematics.h"	// temporary

Move::Move() : currentDda(nullptr), scheduledMoves(0), completedMoves(0)
{
	active = false;

	kinematics = Kinematics::Create(KinematicsType::cartesian);			// default to Cartesian

	// Build the DDA ring
	DDA *dda = new DDA(nullptr);
	ddaRingGetPointer = ddaRingAddPointer = dda;
	for (size_t i = 1; i < DdaRingLength; i++)
	{
		DDA *oldDda = dda;
		dda = new DDA(dda);
		oldDda->SetPrevious(dda);
	}
	ddaRingAddPointer->SetNext(dda);
	dda->SetPrevious(ddaRingAddPointer);

	DriveMovement::InitialAllocate(NumDms);
}

void Move::Init()
{
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
	{
		float move[DRIVES];
		for (size_t i = 0; i < DRIVES; i++)
		{
			move[i] = 0.0;
			liveEndPoints[i] = 0;								// not actually right for a delta, but better than printing random values in response to M114
		}
		SetLiveCoordinates(move);
		SetPositions(move);
	}

	for (size_t i = 0; i < ARRAY_SIZE(extrusionAccumulators); ++i)
	{
		extrusionAccumulators[i] = 0;
	}

	usingMesh = false;
	useTaper = false;

	longWait = reprap.GetPlatform().Time();
	idleTimeout = DEFAULT_IDLE_TIMEOUT;
	iState = IdleState::idle;
	idleCount = 0;

	simulationMode = 0;
	simulationTime = 0.0;
	longestGcodeWaitInterval = 0;
	waitingForMove = specialMoveAvailable = false;

	active = true;
}

void Move::Exit()
{
	Platform::DisableStepInterrupt();

	// Clear the DDA ring so that we don't report any moves as pending
	currentDda = nullptr;
	while (ddaRingGetPointer != ddaRingAddPointer)
	{
		ddaRingGetPointer->Complete();
		ddaRingGetPointer = ddaRingGetPointer->GetNext();
	}

	while (ddaRingCheckPointer->GetState() == DDA::completed)
	{
		(void)ddaRingCheckPointer->Free();
		ddaRingCheckPointer = ddaRingCheckPointer->GetNext();
	}
	active = false;												// don't accept any more moves
}

void Move::Spin()
{
	if (!active)
	{
		GCodes::RawMove nextMove;
		(void) reprap.GetGCodes().ReadMove(nextMove);			// throw away any move that GCodes tries to pass us
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
			reprap.GetPlatform().LogError(ErrorCode::BadMove);
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
		&& DriveMovement::NumFree() >= (int)DRIVES							// check that we won't run out of DMs
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
			// First check for a special move
			if (specialMoveAvailable)
			{
				if (simulationMode < 2)
				{
					if (ddaRingAddPointer->Init(specialMoveCoords))
					{
						ddaRingAddPointer = ddaRingAddPointer->GetNext();
						idleCount = 0;
					}
				}
				specialMoveAvailable = false;
			}
			else
			{
				// If there's a G Code move available, add it to the DDA ring for processing.
				GCodes::RawMove nextMove;
				if (reprap.GetGCodes().ReadMove(nextMove))
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
						if (nextMove.moveType == 0)
						{
							AxisAndBedTransform(nextMove.coords, nextMove.xAxes, nextMove.yAxes, true);
						}
						if (ddaRingAddPointer->Init(nextMove, !IsRawMotorMove(nextMove.moveType)))
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
	}

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
			else if (!simulationMode != 0 && iState == IdleState::busy && !reprap.GetGCodes().IsPaused() && idleTimeout > 0.0)
			{
				lastMoveTime = reprap.GetPlatform().Time();			// record when we first noticed that the machine was idle
				iState = IdleState::timing;
			}
			else if (!simulationMode != 0 && iState == IdleState::timing && reprap.GetPlatform().Time() - lastMoveTime >= idleTimeout)
			{
				reprap.GetPlatform().SetDriversIdle();					// put all drives in idle hold
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
			currentDda->Complete();
			CurrentMoveCompleted();
		}
	}

	reprap.GetPlatform().ClassReport(longWait);
}

// Try to push some babystepping through the lookahead queue
float Move::PushBabyStepping(float amount)
{
	return ddaRingAddPointer->AdvanceBabyStepping(amount);
}

// Change the kinematics to the specified type if it isn't already
// If it is already correct leave its parameters alone.
// This violates our rule on no dynamic memory allocation after the initialisation phase,
// however this function is normally called only when M665, M667 and M669 commands in config.g are processed.
bool Move::SetKinematics(KinematicsType k)
{
	if (kinematics->GetKinematicsType() != k)
	{
		Kinematics *nk = Kinematics::Create(k);
		if (nk == nullptr)
		{
			return false;
		}
		delete kinematics;
		kinematics = nk;
	}
	return true;
}

// Return true if this is a raw motor move
bool Move::IsRawMotorMove(uint8_t moveType) const
{
	return moveType == 2 || ((moveType == 1 || moveType == 3) && kinematics->GetHomingMode() != Kinematics::HomingMode::homeCartesianAxes);
}

// Return true if the specified point is accessible to the Z probe
bool Move::IsAccessibleProbePoint(float x, float y) const
{
	const ZProbeParameters& params = reprap.GetPlatform().GetCurrentZProbeParameters();
	return kinematics->IsReachable(x - params.xOffset, y - params.yOffset);
}

// Pause the print as soon as we can, returning true if we are able to.
// Returns the file position of the first queue move we are going to skip, or noFilePosition we we are not skipping any moves.
// We update 'positions' to the positions and feed rate expected for the next move, and the amount of extrusion in the moves we skipped.
// If we are not skipping any moves then the feed rate, virtual extruder position and iobits are left alone, therefore the caller should set them up first.
bool Move::PausePrint(RestorePoint& rp)
{
	// Find a move we can pause after.
	// Ideally, we would adjust a move if necessary and possible so that we can pause after it, but for now we don't do that.
	// There are a few possibilities:
	// 1. There is no currently executing move and no moves in the queue, and GCodes does not have a move for us.
	//    Pause immediately. Resume from the current file position.
	// 2. There is no currently executing move and no moves in the queue, and GCodes has a move for us but that move has not been started.
	//    Pause immediately. Discard the move that GCodes gas for us, and resume from the start file position of that move.
	// 3. There is no currently executing move and no moves in the queue, and GCodes has a move for that has not been started.
	//    We must complete that move and then pause
	// 5. There is no currently-executing move but there are moves in the queue. Unlikely, but possible.
	//    If the first move in the queue is the first segment in its move, pause immediately, resume from its start address. Otherwise proceed as in case 5.
	// 4. There is a currently-executing move, possibly some moves in the queue, and GCodes may have a whole or partial move for us.
	//    See if we can pause after any of them and before the next. If we can, resume from the start position of the following move.
	//    If we can't, then the last move in the queue must be part of a multi-segment move and GCodes has the rest. We must finish that move and then pause.
	//
	// So on return we need to signal one of the following to GCodes:
	// 1. We have skipped some moves in the queue. Pass back the file address of the first move we have skipped, the feed rate at the start of that move
	//    and the iobits at the start of that move, and return true.
	// 2. All moves in the queue need to be executed. Also any move held by GCodes needs to be completed it is it not the first segment. Return false.
	//
	// In general, we can pause after a move if it is the last segment and its end speed is slow enough.
	// We can pause before a move if it is the first segment in that move.

	const DDA *savedDdaRingAddPointer = ddaRingAddPointer;
	bool pauseOkHere;

	cpu_irq_disable();
	DDA *dda = currentDda;
	if (dda == nullptr)
	{
		pauseOkHere = true;								// no move was executing, so we have already paused here whether it was a good idea or not.
		dda = ddaRingGetPointer;
	}
	else
	{
		pauseOkHere = dda->CanPauseAfter();
		dda = dda->GetNext();
	}

	while (dda != savedDdaRingAddPointer)
	{
		if (pauseOkHere && dda->CanPauseBefore())
		{
			// We can pause before executing this move
			(void)dda->Free();							// free the DDA we are going to pause before it so that it won't get executed after we enable interrupts
			ddaRingAddPointer = dda;
			break;
		}
		pauseOkHere = dda->CanPauseAfter();
		dda = dda->GetNext();
	}

	cpu_irq_enable();

	if (ddaRingAddPointer == savedDdaRingAddPointer)
	{
		return false;									// we can't skip any moves
	}

	// We are going to skip some moves. Get the end coordinate of the previous move.
	DDA * const prevDda = ddaRingAddPointer->GetPrevious();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = prevDda->GetEndCoordinate(axis, false);
	}

	InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetXAxes(), prevDda->GetYAxes());	// we assume that xAxes hasn't changed between the moves

	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
	{
		rp.moveCoords[drive] = 0.0;						// clear out extruder movement
	}

	dda = ddaRingAddPointer;
	rp.feedRate = dda->GetRequestedSpeed();
	rp.virtualExtruderPosition = dda->GetVirtualExtruderPosition();
	rp.filePos = dda->GetFilePosition();

#if SUPPORT_IOBITS
	rp.ioBits = dda->GetIoBits();
#endif

	// Free the DDAs for the moves we are going to skip, and work out how much extrusion they would have performed
	do
	{
		for (size_t drive = numTotalAxes; drive < DRIVES; ++drive)
		{
			rp.moveCoords[drive] += dda->GetEndCoordinate(drive, true);		// update the amount of extrusion we are going to skip
		}
		(void)dda->Free();
		dda = dda->GetNext();
		scheduledMoves--;
	}
	while (dda != savedDdaRingAddPointer);

	return true;
}

uint32_t maxReps = 0;

#if 0
// For debugging
extern uint32_t sqSum1, sqSum2, sqCount, sqErrors, lastRes1, lastRes2;
extern uint64_t lastNum;
#endif

void Move::Diagnostics(MessageType mtype)
{
	Platform& p = reprap.GetPlatform();
	p.Message(mtype, "=== Move ===\n");
	p.MessageF(mtype, "MaxReps: %u, StepErrors: %u, FreeDm: %d, MinFreeDm %d, MaxWait: %ums, Underruns: %u, %u\n",
						maxReps, stepErrors, DriveMovement::NumFree(), DriveMovement::MinFree(), longestGcodeWaitInterval, numLookaheadUnderruns, numPrepareUnderruns);
	maxReps = 0;
	numLookaheadUnderruns = numPrepareUnderruns = 0;
	longestGcodeWaitInterval = 0;
	DriveMovement::ResetMinFree();

	reprap.GetPlatform().MessageF(mtype, "Scheduled moves: %u, completed moves: %u\n", scheduledMoves, completedMoves);

#if defined(__ALLIGATOR__)
	// Motor Fault Diagnostic
	reprap.GetPlatform().MessageF(mtype, "Motor Fault status: %s\n", digitalRead(MotorFaultDetectPin) ? "none" : "FAULT detected!" );
#endif

	// Show the current probe position heights and type of bed compensation in use
	p.Message(mtype, "Bed compensation in use: ");
	if (usingMesh)
	{
		p.Message(mtype, "mesh\n");
	}
	else if (probePoints.GetNumBedCompensationPoints() != 0)
	{
		p.MessageF(mtype, "%d point\n", probePoints.GetNumBedCompensationPoints());
	}
	else
	{
		p.Message(mtype, "none\n");
	}

	p.Message(mtype, "Bed probe heights:");
	// To keep the response short so that it doesn't get truncated when sending it via HTTP, we only show the first 5 bed probe points
	for (size_t i = 0; i < 5; ++i)
	{
		p.MessageF(mtype, " %.3f", probePoints.GetZHeight(i));
	}
	p.Message(mtype, "\n");

#if DDA_LOG_PROBE_CHANGES
	// Temporary code to print Z probe trigger positions
	p.Message(mtype, "Probe change coordinates:");
	char ch = ' ';
	for (size_t i = 0; i < DDA::numLoggedProbePositions; ++i)
	{
		float xyzPos[XYZ_AXES];
		MotorStepsToCartesian(DDA::loggedProbePositions + (XYZ_AXES * i), XYZ_AXES, XYZ_AXES, xyzPos);
		p.MessageF(mtype, "%c%.2f,%.2f", ch, xyzPos[X_AXIS], xyzPos[Y_AXIS]);
		ch = ',';
	}
	p.Message(mtype, "\n");
#endif

#if 0
	// For debugging
	if (sqCount != 0)
	{
		p.AppendMessage(GENERIC_MESSAGE, "Average sqrt times %.2f, %.2f, count %u,  errors %u, last %" PRIu64 " %u %u\n",
				(float)sqSum1/sqCount, (float)sqSum2/sqCount, sqCount, sqErrors, lastNum, lastRes1, lastRes2);
		sqSum1 = sqSum2 = sqCount = sqErrors = 0;
	}
#endif
}

// Set the current position to be this
void Move::SetNewPosition(const float positionNow[DRIVES], bool doBedCompensation)
{
	float newPos[DRIVES];
	memcpy(newPos, positionNow, sizeof(newPos));			// copy to local storage because Transform modifies it
	AxisAndBedTransform(newPos, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes(), doBedCompensation);
	SetLiveCoordinates(newPos);
	SetPositions(newPos);
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
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "SetPositions called when DDA ring not empty\n");
	}
}

void Move::EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const
{
	if (CartesianToMotorSteps(coords, ep))
	{
		const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t drive = numAxes; drive < numDrives; ++drive)
		{
			ep[drive] = MotorEndPointToMachine(drive, coords[drive]);
		}
	}
}

// Convert distance to steps for a particular drive
int32_t Move::MotorEndPointToMachine(size_t drive, float coord)
{
	return lrintf(coord * reprap.GetPlatform().DriveStepsPerUnit(drive));
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// This is computationally expensive on a delta or SCARA machine, so only call it when necessary, and never from the step ISR.
void Move::MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	kinematics->MotorStepsToCartesian(motorPos, reprap.GetPlatform().GetDriveStepsPerUnit(), numVisibleAxes, numTotalAxes, machinePos);
}

// Convert Cartesian coordinates to motor steps, axes only, returning true if successful.
// Used to perform movement and G92 commands.
bool Move::CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes]) const
{
	const bool b = kinematics->CartesianToMotorSteps(machinePos, reprap.GetPlatform().GetDriveStepsPerUnit(), reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), motorPos);
	if (reprap.Debug(moduleMove) && reprap.Debug(moduleDda))
	{
		if (b)
		{
			debugPrintf("Transformed %.2f %.2f %.2f to %d %d %d\n", machinePos[0], machinePos[1], machinePos[2], motorPos[0], motorPos[1], motorPos[2]);
		}
		else
		{
			debugPrintf("Unable to transform %.2f %.2f %.2f\n", machinePos[0], machinePos[1], machinePos[2]);
		}
	}
	return b;
}

void Move::AxisAndBedTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes, bool useBedCompensation) const
{
	AxisTransform(xyzPoint, xAxes, yAxes);
	if (useBedCompensation)
	{
		BedTransform(xyzPoint, xAxes, yAxes);
	}
}

void Move::InverseAxisAndBedTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	InverseBedTransform(xyzPoint, xAxes, yAxes);
	InverseAxisTransform(xyzPoint, xAxes, yAxes);
}

// Do the Axis transform BEFORE the bed transform
void Move::AxisTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	// Identify the lowest Y axis
	const size_t NumVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t yAxis = Y_AXIS; yAxis < NumVisibleAxes; ++yAxis)
	{
		if (IsBitSet(yAxes, yAxis))
		{
			// Found a Y axis. Use this one when correcting the X coordinate.
			for (size_t axis = 0; axis < NumVisibleAxes; ++axis)
			{
				if (IsBitSet(xAxes, axis))
				{
					xyzPoint[axis] += tanXY*xyzPoint[yAxis] + tanXZ*xyzPoint[Z_AXIS];
				}
				if (IsBitSet(yAxes, axis))
				{
					xyzPoint[axis] += tanYZ*xyzPoint[Z_AXIS];
				}
			}
			break;
		}
	}
}

// Invert the Axis transform AFTER the bed transform
void Move::InverseAxisTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	// Identify the lowest Y axis
	const size_t NumVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t yAxis = Y_AXIS; yAxis < NumVisibleAxes; ++yAxis)
	{
		if (IsBitSet(yAxes, yAxis))
		{
			// Found a Y axis. Use this one when correcting the X coordinate.
			for (size_t axis = 0; axis < NumVisibleAxes; ++axis)
			{
				if (IsBitSet(yAxes, axis))
				{
					xyzPoint[axis] -= tanYZ*xyzPoint[Z_AXIS];
				}
				if (IsBitSet(xAxes, axis))
				{
					xyzPoint[axis] -= (tanXY*xyzPoint[yAxis] + tanXZ*xyzPoint[Z_AXIS]);
				}
			}
			break;
		}
	}
}

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	if (!useTaper || xyzPoint[Z_AXIS] < taperHeight)
	{
		float zCorrection = 0.0;
		const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
		unsigned int numCorrections = 0;

		// Transform the Z coordinate based on the average correction for each axis used as an X axis.
		// We are assuming that the tool Y offsets are small enough to be ignored.
		for (uint32_t xAxis = 0; xAxis < numAxes; ++xAxis)
		{
			if (IsBitSet(xAxes, xAxis))
			{
				const float xCoord = xyzPoint[xAxis];
				for (uint32_t yAxis = 0; yAxis < numAxes; ++yAxis)
				{
					if (IsBitSet(yAxes, yAxis))
					{
						const float yCoord = xyzPoint[yAxis];
						if (usingMesh)
						{
							zCorrection += grid.GetInterpolatedHeightError(xCoord, yCoord);
						}
						else
						{
							zCorrection += probePoints.GetInterpolatedHeightError(xCoord, yCoord);
						}
						++numCorrections;
					}
				}
			}
		}

		if (numCorrections > 1)
		{
			zCorrection /= numCorrections;			// take an average
		}

		xyzPoint[Z_AXIS] += (useTaper) ? (taperHeight - xyzPoint[Z_AXIS]) * recipTaperHeight * zCorrection : zCorrection;
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	float zCorrection = 0.0;
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	unsigned int numCorrections = 0;

	// Transform the Z coordinate based on the average correction for each axis used as an X axis.
	// We are assuming that the tool Y offsets are small enough to be ignored.
	for (uint32_t xAxis = 0; xAxis < numAxes; ++xAxis)
	{
		if (IsBitSet(xAxes, xAxis))
		{
			const float xCoord = xyzPoint[xAxis];
			for (uint32_t yAxis = 0; yAxis < numAxes; ++yAxis)
			{
				if (IsBitSet(yAxes, yAxis))
				{
					const float yCoord = xyzPoint[yAxis];
					if (usingMesh)
					{
						zCorrection += grid.GetInterpolatedHeightError(xCoord, yCoord);
					}
					else
					{
						zCorrection += probePoints.GetInterpolatedHeightError(xCoord, yCoord);
					}
					++numCorrections;
				}
			}
		}
	}

	if (numCorrections > 1)
	{
		zCorrection /= numCorrections;				// take an average
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
	probePoints.SetIdentity();
	grid.ClearGridHeights();
	grid.UseHeightMap(false);
	usingMesh = false;
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

// Enable mesh bed compensation
bool Move::UseMesh(bool b)
{
	usingMesh = grid.UseHeightMap(b);
	return usingMesh;
}

float Move::AxisCompensation(unsigned int axis) const
{
	return (axis < ARRAY_SIZE(tangents)) ? tangents[axis] : 0.0;
}

void Move::SetAxisCompensation(unsigned int axis, float tangent)
{
	if (axis < ARRAY_SIZE(tangents))
	{
		tangents[axis] = tangent;
	}
}

// Calibrate or set the bed equation after probing.
// sParam is the value of the S parameter in the G30 command that provoked this call.
// Caller already owns the GCode movement lock.
void Move::FinishedBedProbing(int sParam, StringRef& reply)
{
	const size_t numPoints = probePoints.NumberOfProbePoints();

	if (sParam < 0)
	{
		// A negative sParam just prints the probe heights
		probePoints.ReportProbeHeights(numPoints, reply);
	}
	else if (numPoints < (size_t)sParam)
	{
		reprap.GetPlatform().MessageF(GENERIC_MESSAGE, "Bed calibration error: %d factor calibration requested but only %d points provided\n", sParam, numPoints);
	}
	else
	{
		if (reprap.Debug(moduleMove))
		{
			probePoints.DebugPrint(numPoints);
		}

		if (sParam == 0)
		{
			sParam = numPoints;
		}

		if (!probePoints.GoodProbePoints(numPoints))
		{
			reply.cat("Compensation or calibration cancelled due to probing errors");
		}
		else if (kinematics->SupportsAutoCalibration())
		{
			kinematics->DoAutoCalibration(sParam, probePoints, reply);
		}
		else
		{
			probePoints.SetProbedBedEquation(sParam, reply);
		}
	}

	// Clear out the Z heights so that we don't re-use old points.
	// This allows us to use different numbers of probe point on different occasions.
	probePoints.ClearProbeHeights();
}

// Perform motor endpoint adjustment
void Move::AdjustMotorPositions(const float_t adjustment[], size_t numMotors)
{
	DDA * const lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const int32_t * const endCoordinates = lastQueuedMove->DriveCoordinates();
	const float * const driveStepsPerUnit = reprap.GetPlatform().GetDriveStepsPerUnit();

	for (size_t drive = 0; drive < DELTA_AXES; ++drive)
	{
		const int32_t ep = endCoordinates[drive] + (int32_t)(adjustment[drive] * driveStepsPerUnit[drive]);
		lastQueuedMove->SetDriveCoordinate(ep, drive);
		liveEndPoints[drive] = ep;
	}

	liveCoordinatesValid = false;		// force the live XYZ position to be recalculated
}

// This is called from the step ISR when the current move has been completed
void Move::CurrentMoveCompleted()
{
	// Save the current motor coordinates, and the machine Cartesian coordinates if known
	liveCoordinatesValid = currentDda->FetchEndPosition(const_cast<int32_t*>(liveEndPoints), const_cast<float *>(liveCoordinates));
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t drive = numAxes; drive < DRIVES; ++drive)
	{
		extrusionAccumulators[drive - numAxes] += currentDda->GetStepsTaken(drive);
	}
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
		reprap.GetPlatform().ExtrudeOff();	// turn off ancilliary PWM
		return false;
	}
}

// Return the untransformed machine coordinates
void Move::GetCurrentMachinePosition(float m[DRIVES], bool disableMotorMapping) const
{
	DDA * const lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
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
	return ((float)(endpoint))/reprap.GetPlatform().DriveStepsPerUnit(drive);
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
void Move::GetCurrentUserPosition(float m[DRIVES], uint8_t moveType, AxesBitmap xAxes, AxesBitmap yAxes) const
{
	GetCurrentMachinePosition(m, IsRawMotorMove(moveType));
	if (moveType == 0)
	{
		InverseAxisAndBedTransform(m, xAxes, yAxes);
	}
}

// Return the current live XYZ and extruder coordinates
// Interrupts are assumed enabled on entry
void Move::LiveCoordinates(float m[DRIVES], AxesBitmap xAxes, AxesBitmap yAxes)
{
	// The live coordinates and live endpoints are modified by the ISR, so be careful to get a self-consistent set of them
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();		// do this before we disable interrupts
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();			// do this before we disable interrupts
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
		memcpy(m + numTotalAxes, const_cast<const float *>(liveCoordinates + numTotalAxes), sizeof(m[0]) * (DRIVES - numTotalAxes));
		int32_t tempEndPoints[MaxAxes];
		memcpy(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints));
		cpu_irq_enable();

		MotorStepsToCartesian(tempEndPoints, numVisibleAxes, numTotalAxes, m);		// this is slow, so do it with interrupts enabled

		// If the ISR has not updated the endpoints, store the live coordinates back so that we don't need to do it again
		cpu_irq_disable();
		if (memcmp(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints)) == 0)
		{
			memcpy(const_cast<float *>(liveCoordinates), m, sizeof(m[0]) * numVisibleAxes);
			liveCoordinatesValid = true;
		}
		cpu_irq_enable();
	}
	InverseAxisAndBedTransform(m, xAxes, yAxes);
}

// These are the actual numbers that we want to be the coordinates, so don't transform them.
// The caller must make sure that no moves are in progress or pending when calling this
void Move::SetLiveCoordinates(const float coords[DRIVES])
{
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		liveCoordinates[drive] = coords[drive];
	}
	liveCoordinatesValid = true;
	EndPointToMachine(coords, const_cast<int32_t *>(liveEndPoints), reprap.GetGCodes().GetVisibleAxes());
}

void Move::ResetExtruderPositions()
{
	cpu_irq_disable();
	for (size_t eDrive = reprap.GetGCodes().GetTotalAxes(); eDrive < DRIVES; eDrive++)
	{
		liveCoordinates[eDrive] = 0.0;
	}
	cpu_irq_enable();
}

int32_t Move::GetAccumulatedExtrusion(size_t extruder)
{
	const size_t drive = extruder + reprap.GetGCodes().GetTotalAxes();
	if (drive < DRIVES)
	{
		const irqflags_t flags = cpu_irq_save();
		const int32_t ret = extrusionAccumulators[extruder];
		const DDA * const cdda = currentDda;						// capture volatile variable
		const int32_t adjustment = (cdda != nullptr) ? cdda->GetStepsTaken(drive) : 0;
		extrusionAccumulators[extruder] = -adjustment;
		cpu_irq_restore(flags);
		return ret + adjustment;
	}

	return 0.0;
}

void Move::SetXYBedProbePoint(size_t index, float x, float y)
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Z probe point index out of range.\n");
	}
	else
	{
		probePoints.SetXYBedProbePoint(index, x, y);
	}
}

void Move::SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError)
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Z probe point Z index out of range.\n");
	}
	else
	{
		probePoints.SetZBedProbePoint(index, z, wasXyCorrected, wasError);
	}
}

// This returns the (X, Y) points to probe the bed at probe point count.  When probing, it returns false.
// If called after probing has ended it returns true, and the Z coordinate probed is also returned.
// If 'wantNozzlePosition is true then we return the nozzle position when the point is probed, else we return the probe point itself
float Move::GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const
{
	x = probePoints.GetXCoord(count);
	y = probePoints.GetYCoord(count);
	if (wantNozzlePosition)
	{
		const ZProbeParameters& rp = reprap.GetPlatform().GetCurrentZProbeParameters();
		x -= rp.xOffset;
		y -= rp.yOffset;
	}
	return probePoints.GetZHeight(count);
}

// Enter or leave simulation mode
void Move::Simulate(uint8_t simMode)
{
	simulationMode = simMode;
	if (simMode != 0)
	{
		simulationTime = 0.0;
	}
}

// Adjust the leadscrews
// This is only ever called after bed probing, so we can assume that no such move is already pending.
void Move::AdjustLeadscrews(const floatc_t corrections[])
{
	for (float& smc : specialMoveCoords)
	{
		smc = 0.0;
	}
	const AxisDriversConfig& config = reprap.GetPlatform().GetAxisDriversConfig(Z_AXIS);
	for (size_t i = 0; i < config.numDrivers; ++i)
	{
		specialMoveCoords[config.driverNumbers[i]] = corrections[i];
	}
	specialMoveAvailable = true;
}

#ifdef DUET_NG

// Write settings for resuming the print
// The GCodes module deals with the head position so all we need worry about is the bed compensation
// We don't handle random probe point bed compensation, and we assume that if a height map is being used it is the default one.
bool Move::WriteResumeSettings(FileStore *f) const
{
	return kinematics->WriteResumeSettings(f) && (!usingMesh || f->Write("G29 S1\n"));
}

#endif

// For debugging
void Move::PrintCurrentDda() const
{
	if (currentDda != nullptr)
	{
		currentDda->DebugPrint();
	}
}

// End
