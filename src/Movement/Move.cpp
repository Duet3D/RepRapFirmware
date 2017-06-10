/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#include "Move.h"
#include "Platform.h"
#include "RepRap.h"

Move::Move() : currentDda(NULL), scheduledMoves(0), completedMoves(0)
{
	active = false;

	kinematics = Kinematics::Create(KinematicsType::cartesian);			// default to Cartesian

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
		reprap.GetPlatform().SetDirection(i, FORWARDS);			// DC: I don't see any reason why we do this
	}
	SetLiveCoordinates(move);
	SetPositions(move);

	usingMesh = false;
	useTaper = false;

	longWait = reprap.GetPlatform().Time();
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
	reprap.GetPlatform().Message(HOST_MESSAGE, "Move class exited.\n");
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
					const bool doMotorMapping = (nextMove.moveType == 0) || (nextMove.moveType == 1 && !IsDeltaMode());
					if (doMotorMapping)
					{
						AxisAndBedTransform(nextMove.coords, nextMove.xAxes, nextMove.moveType == 0);
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
			CurrentMoveCompleted();
		}
	}

	reprap.GetPlatform().ClassReport(longWait);
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

// Return true if the specified point is accessible to the Z probe
bool Move::IsAccessibleProbePoint(float x, float y) const
{
	const ZProbeParameters& params = reprap.GetPlatform().GetCurrentZProbeParameters();
	return kinematics->IsReachable(x - params.xOffset, y - params.yOffset);
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
		const size_t numAxes = reprap.GetGCodes().GetTotalAxes();

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
	Platform& p = reprap.GetPlatform();
	p.Message(mtype, "=== Move ===\n");
	p.MessageF(mtype, "MaxReps: %u, StepErrors: %u, MaxWait: %ums, Underruns: %u, %u\n",
						maxReps, stepErrors, longestGcodeWaitInterval, numLookaheadUnderruns, numPrepareUnderruns);
	maxReps = 0;
	numLookaheadUnderruns = numPrepareUnderruns = 0;
	longestGcodeWaitInterval = 0;

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
	return (int32_t)roundf(coord * reprap.GetPlatform().DriveStepsPerUnit(drive));
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

void Move::AxisAndBedTransform(float xyzPoint[MaxAxes], uint32_t xAxes, bool useBedCompensation) const
{
	AxisTransform(xyzPoint);
	if (useBedCompensation)
	{
		BedTransform(xyzPoint, xAxes);
	}
}

void Move::InverseAxisAndBedTransform(float xyzPoint[MaxAxes], uint32_t xAxes) const
{
	InverseBedTransform(xyzPoint, xAxes);
	InverseAxisTransform(xyzPoint);
}

// Do the Axis transform BEFORE the bed transform
void Move::AxisTransform(float xyzPoint[MaxAxes]) const
{
	//TODO should we transform U axis instead of/as well as X if we are in dual carriage mode?
	xyzPoint[X_AXIS] += tanXY*xyzPoint[Y_AXIS] + tanXZ*xyzPoint[Z_AXIS];
	xyzPoint[Y_AXIS] += tanYZ*xyzPoint[Z_AXIS];
}

// Invert the Axis transform AFTER the bed transform
void Move::InverseAxisTransform(float xyzPoint[MaxAxes]) const
{
	//TODO should we transform U axis instead of/as well as X if we are in dual carriage mode?
	xyzPoint[Y_AXIS] -= tanYZ*xyzPoint[Z_AXIS];
	xyzPoint[X_AXIS] -= (tanXY*xyzPoint[Y_AXIS] + tanXZ*xyzPoint[Z_AXIS]);
}

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[MaxAxes], uint32_t xAxes) const
{
	if (!useTaper || xyzPoint[Z_AXIS] < taperHeight)
	{
		float zCorrection = 0.0;
		const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
		unsigned int numXAxes = 0;

		// Transform the Z coordinate based on the average correction for each axis used as an X axis.
		// We are assuming that the tool Y offsets are small enough to be ignored.
		for (uint32_t axis = 0; axis < numAxes; ++axis)
		{
			if ((xAxes & (1u << axis)) != 0)
			{
				const float xCoord = xyzPoint[axis];
				if (usingMesh)
				{
					zCorrection += grid.GetInterpolatedHeightError(xCoord, xyzPoint[Y_AXIS]);
				}
				else
				{
					zCorrection += probePoints.GetInterpolatedHeightError(xCoord, xyzPoint[Y_AXIS]);
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
void Move::InverseBedTransform(float xyzPoint[MaxAxes], uint32_t xAxes) const
{
	float zCorrection = 0.0;
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	unsigned int numXAxes = 0;

	// Transform the Z coordinate based on the average correction for each axis used as an X axis.
	// We are assuming that the tool Y offsets are small enough to be ignored.
	for (uint32_t axis = 0; axis < numAxes; ++axis)
	{
		if ((xAxes & (1u << axis)) != 0)
		{
			const float xCoord = xyzPoint[axis];
			if (usingMesh)
			{
				zCorrection += grid.GetInterpolatedHeightError(xCoord, xyzPoint[Y_AXIS]);
			}
			else
			{
				zCorrection += probePoints.GetInterpolatedHeightError(xCoord, xyzPoint[Y_AXIS]);

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
			reprap.GetPlatform().Message(GENERIC_MESSAGE, "Axis compensation requested for non-existent axis.\n");
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
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "SetAxisCompensation: dud axis.\n");
	}
}

// Calibrate or set the bed equation after probing.
// sParam is the value of the S parameter in the G30 command that provoked this call.
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
		reprap.GetPlatform().ExtrudeOff();	// turn off ancilliary PWM
		return false;
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitLowStop(size_t axis, DDA* hitDDA)
{
	if (axis < reprap.GetGCodes().GetTotalAxes() && !IsDeltaMode())		// should always be true
	{
		JustHomed(axis, reprap.GetPlatform().AxisMinimum(axis), hitDDA);
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::HitHighStop(size_t axis, DDA* hitDDA)
{
	if (axis < reprap.GetGCodes().GetTotalAxes())		// should always be true
	{
		const float hitPoint = (IsDeltaMode())
								? ((LinearDeltaKinematics*)kinematics)->GetHomedCarriageHeight(axis)	// this is a delta printer, so the motor is at the homed carriage height for this drive
								: reprap.GetPlatform().AxisMaximum(axis);	// this is a Cartesian printer, so we're at the maximum for this axis
		JustHomed(axis, hitPoint, hitDDA);
	}
}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR must be declared 'volatile'.
void Move::JustHomed(size_t axisHomed, float hitPoint, DDA* hitDDA)
{
	if (IsCoreXYAxis(axisHomed))
	{
		float tempCoordinates[XYZ_AXES];
		for (size_t axis = 0; axis < XYZ_AXES; ++axis)
		{
			tempCoordinates[axis] = hitDDA->GetEndCoordinate(axis, false);
		}
		tempCoordinates[axisHomed] = hitPoint;
		hitDDA->SetPositions(tempCoordinates, XYZ_AXES);
	}
	else
	{
		hitDDA->SetDriveCoordinate(MotorEndPointToMachine(axisHomed, hitPoint), axisHomed);
	}
	reprap.GetGCodes().SetAxisIsHomed(axisHomed);

}

// This is called from the step ISR. Any variables it modifies that are also read by code outside the ISR should be declared 'volatile'.
// The move has already been aborted when this is called, so the endpoints in the DDA are the current motor positions.
void Move::ZProbeTriggered(DDA* hitDDA)
{
	reprap.GetGCodes().MoveStoppedByZProbe();
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
void Move::GetCurrentUserPosition(float m[DRIVES], uint8_t moveType, uint32_t xAxes) const
{
	GetCurrentMachinePosition(m, moveType == 2 || (moveType == 1 && IsDeltaMode()));
	if (moveType == 0)
	{
		InverseAxisAndBedTransform(m, xAxes);
	}
}

// Return the current live XYZ and extruder coordinates
// Interrupts are assumed enabled on entry
void Move::LiveCoordinates(float m[DRIVES], uint32_t xAxes)
{
	// The live coordinates and live endpoints are modified by the ISR, so be careful to get a self-consistent set of them
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();		// do this before we disable interrupts
	cpu_irq_disable();
	if (liveCoordinatesValid)
	{
		// All coordinates are valid, so copy them across
		memcpy(m, const_cast<const float *>(liveCoordinates), sizeof(m[0]) * DRIVES);
		cpu_irq_enable();
	}
	else
	{
		const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();		// do this before we disable interrupts
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
	InverseAxisAndBedTransform(m, xAxes);
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

// Return true if the specified axis shares its motors with another. Safe to call for extruders as well as axes.
bool Move::IsCoreXYAxis(size_t axis) const
{
	switch(kinematics->GetKinematicsType())
	{
	case KinematicsType::coreXY:
		return axis == X_AXIS || axis == Y_AXIS;
	case KinematicsType::coreXZ:
		return axis == X_AXIS || axis == Z_AXIS;
	default:
		return false;
	}
}

// End
