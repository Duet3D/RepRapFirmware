/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David

 A note on bed levelling:

 As at version 1.21 we support two types of bed compensation:
 1. The old 3, 4 and 5-point compensation using a RandomProbePointSet. We will probably discontinue this soon.
 2. Mesh bed levelling

 There is an interaction between using G30 to home Z or set a precise Z=0 height just before a print, and bed compensation.
 Consider the following sequence:
 1. Home Z, using either G30 or an endstop.
 2. Run G29 to generate a height map. If the Z=0 point has drifted off, the height map may have a Z offset.
 3. Use G30 to get an accurate Z=0 point. We want to keep the shape of the height map, but get rid of the offset.
 4. Run G29 to generate a height map. This should generate a height map with on offset at the point we just probed.
 5. Cancel bed compensation. The height at the point we just probed should be zero.

 So as well as maintaining a height map, we maintain a Z offset from it. The procedure is:
 1. Whenever bed compensation is not being used, the Z offset should be zero.
 2. Whenever we run G29 to probe the bed, we have a choice:
 (a) accept that the map may have a height offset; and set the Z offset to zero. This is what we do currently.
 (b) normalise the height map to zero, adjust the Z=0 origin, and set the Z offset to zero.
 3. When we run G30 to reset the Z=0 height, and we have a height map loaded, we adjust the Z offset to be the negative of the
    height map indication of that point.
 4. If we now cancel the height map, we also clear the Z offset, and the height at the point we probed remains correct.
 5. If we now run G29 to probe again, the height map should have near zero offset at the point we probed, if there has been no drift.

 Before we introduced the Z offset, at step 4 we would have a potentially large Z error as if the G30 hadn't been run,
 and at step 5 the new height map would have an offset again.

 */

#include "Move.h"
#include "Platform.h"
#include "Tools/Tool.h"

constexpr uint32_t UsualMinimumPreparedTime = StepClockRate/10;			// 100ms
constexpr uint32_t AbsoluteMinimumPreparedTime = StepClockRate/20;		// 50ms

Move::Move() : currentDda(nullptr), active(false), scheduledMoves(0), completedMoves(0)
{
	kinematics = Kinematics::Create(KinematicsType::cartesian);			// default to Cartesian

	// Build the DDA ring
	DDA *dda = new DDA(nullptr);
	ddaRingGetPointer = ddaRingAddPointer = dda;
	for (size_t i = 1; i < DdaRingLength; i++)
	{
		DDA * const oldDda = dda;
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
	numLookaheadUnderruns = numPrepareUnderruns = numLookaheadErrors = 0;

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

	for (size_t i = 0; i < MaxExtruders; ++i)
	{
		extrusionAccumulators[i] = 0;
		extruderNonPrinting[i] = false;
		extrusionPending[i] = 0.0;
	}

	usingMesh = false;
	useTaper = false;
	zShift = 0.0;

	idleTimeout = DefaultIdleTimeout;
	moveState = MoveState::idle;
	idleCount = 0;

	simulationMode = 0;
	simulationTime = 0.0;
	longestGcodeWaitInterval = 0;
	specialMoveAvailable = false;

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

	// Recycle the DDAs for completed moves, checking for DDA errors to print if Move debug is enabled
	while (ddaRingCheckPointer->GetState() == DDA::completed)
	{
		// Check for step errors and record/print them if we have any, before we lose the DMs
		if (ddaRingCheckPointer->HasStepError())
		{
			if (reprap.Debug(moduleMove))
			{
				ddaRingCheckPointer->DebugPrintAll();
			}
			++stepErrors;
			reprap.GetPlatform().LogError(ErrorCode::BadMove);
		}

		// Now release the DMs and check for underrun
		if (ddaRingCheckPointer->Free())
		{
			++numLookaheadUnderruns;
		}
		ddaRingCheckPointer = ddaRingCheckPointer->GetNext();
	}

	// See if we can add another move to the ring
	bool canAddMove = (
#if SUPPORT_ROLAND
						  !reprap.GetRoland()->Active() &&
#endif
						  ddaRingAddPointer->GetState() == DDA::empty
					   && ddaRingAddPointer->GetNext()->GetState() != DDA::provisional		// function Prepare needs to access the endpoints in the previous move, so don't change them
					   && DriveMovement::NumFree() >= (int)DRIVES							// check that we won't run out of DMs
					  );
	if (canAddMove)
	{
		// In order to react faster to speed and extrusion rate changes, only add more moves if the total duration of
		// all un-frozen moves is less than 2 seconds, or the total duration of all but the first un-frozen move is less than 0.5 seconds.
		const DDA *dda = ddaRingAddPointer;
		uint32_t unPreparedTime = 0;
		uint32_t prevMoveTime = 0;
		for(;;)
		{
			dda = dda->GetPrevious();
			if (dda->GetState() != DDA::provisional)
			{
				break;
			}
			unPreparedTime += prevMoveTime;
			prevMoveTime = dda->GetClocksNeeded();
		}

		canAddMove = (unPreparedTime < StepClockRate/2 || unPreparedTime + prevMoveTime < 2 * StepClockRate);
	}

	if (canAddMove)
	{
		// OK to add another move. First check if a special move is available.
		if (specialMoveAvailable)
		{
			if (simulationMode < 2)
			{
				if (ddaRingAddPointer->Init(specialMoveCoords))
				{
					ddaRingAddPointer = ddaRingAddPointer->GetNext();
					if (moveState == MoveState::idle || moveState == MoveState::timing)
					{
						// We were previously idle, so we have a state change
						moveState = MoveState::collecting;
						const uint32_t now = millis();
						const uint32_t timeWaiting = now - lastStateChangeTime;
						if (timeWaiting > longestGcodeWaitInterval)
						{
							longestGcodeWaitInterval = timeWaiting;
						}
						lastStateChangeTime = now;
					}
				}
			}
			specialMoveAvailable = false;
		}
		else
		{
			// If there's a G Code move available, add it to the DDA ring for processing.
			GCodes::RawMove nextMove;
			if (reprap.GetGCodes().ReadMove(nextMove))		// if we have a new move
			{
				if (simulationMode < 2)		// in simulation mode 2 and higher, we don't process incoming moves beyond this point
				{
#if 0	// disabled this because it causes jerky movements on the SCARA printer
					// Add on the extrusion left over from last time.
					const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
					for (size_t drive = numAxes; drive < DRIVES; ++drive)
					{
						nextMove.coords[drive] += extrusionPending[drive - numAxes];
					}
#endif
					if (nextMove.moveType == 0)
					{
						AxisAndBedTransform(nextMove.coords, nextMove.xAxes, nextMove.yAxes, true);
					}

					if (ddaRingAddPointer->Init(nextMove, !IsRawMotorMove(nextMove.moveType)))
					{
						ddaRingAddPointer = ddaRingAddPointer->GetNext();
						idleCount = 0;
						scheduledMoves++;
						if (moveState == MoveState::idle || moveState == MoveState::timing)
						{
							moveState = MoveState::collecting;
							const uint32_t now = millis();
							const uint32_t timeWaiting = now - lastStateChangeTime;
							if (timeWaiting > longestGcodeWaitInterval)
							{
								longestGcodeWaitInterval = timeWaiting;
							}
							lastStateChangeTime = now;
						}
					}

#if 0	// see above
					// Save the amount of extrusion not done
					for (size_t drive = numAxes; drive < DRIVES; ++drive)
					{
						extrusionPending[drive - numAxes] = nextMove.coords[drive];
					}
#endif
				}
			}
		}
	}

	// If we are simulating, simulate completion of the current move.
	// Do this here rather than at the end, so that when simulating, currentDda is non-null for most of the time and IsExtruding() returns the correct value
	{
		DDA *cdda;													// currentDda is declared volatile, so copy it in the next line
		if (simulationMode != 0 && (cdda = currentDda) != nullptr)
		{
			simulationTime += (float)cdda->GetClocksNeeded()/StepClockRate;
			cdda->Complete();
			CurrentMoveCompleted();
		}
	}

	// See whether we need to kick off a move
	if (currentDda == nullptr)
	{
		// No DDA is executing, so start executing a new one if possible
		if (!canAddMove || idleCount > 10)							// better to have a few moves in the queue so that we can do lookahead
		{
			// Prepare one move and execute it. We assume that we will enter the next if-block before it completes, giving us time to prepare more moves.
			Platform::DisableStepInterrupt();						// should be disabled already because we weren't executing a move, but make sure
			DDA * const dda = ddaRingGetPointer;					// capture volatile variable
			if (dda->GetState() == DDA::provisional)
			{
				dda->Prepare(simulationMode);
			}
			if (dda->GetState() == DDA::frozen)
			{
				if (simulationMode != 0)
				{
					currentDda = dda;								// pretend we are executing this move
				}
				else
				{
					if (StartNextMove(Platform::GetInterruptClocks()))	// start the next move
					{
						Interrupt();
					}
				}
				moveState = MoveState::executing;
			}
			else if (simulationMode == 0)
			{
				if (moveState == MoveState::executing && !reprap.GetGCodes().IsPaused())
				{
					lastStateChangeTime = millis();					// record when we first noticed that the machine was idle
					moveState = MoveState::timing;
				}
				else if (moveState == MoveState::timing && millis() - lastStateChangeTime >= idleTimeout)
				{
					reprap.GetPlatform().SetDriversIdle();			// put all drives in idle hold
					moveState = MoveState::idle;
				}
			}
		}
	}

	DDA *cdda = currentDda;											// currentDda is volatile, so copy it
	if (cdda != nullptr)
	{
		// See whether we need to prepare any moves. First count how many prepared or executing moves we have and how long they will take.
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

		// If the number of prepared moves will execute in less than the minimum time, prepare another move.
		// Try to avoid preparing deceleration-only moves
		while (st == DDA::provisional
				&& preparedTime < (int32_t)UsualMinimumPreparedTime		// prepare moves one eighth of a second ahead of when they will be needed
				&& preparedCount < DdaRingLength/2 - 1					// but don't prepare as much as half the ring
			  )
		{
			if (cdda->IsGoodToPrepare() || preparedTime < (int32_t)AbsoluteMinimumPreparedTime)
			{
				cdda->Prepare(simulationMode);
			}
			preparedTime += cdda->GetTimeLeft();
			++preparedCount;
			cdda = cdda->GetNext();
			st = cdda->GetState();
		}
	}
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
	const ZProbe& params = reprap.GetPlatform().GetCurrentZProbeParameters();
	return kinematics->IsReachable(x - params.xOffset, y - params.yOffset, false);
}

// Pause the print as soon as we can, returning true if we are able to skip any moves and updating 'rp' to the first move we skipped.
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
	// 2. All moves in the queue need to be executed. Also any move held by GCodes needs to be completed it is it not the first segment.
	//    Update the restore point with the coordinates and iobits as at the end of the previous move and return false.
	//    The extruder position, file position and feed rate are not filled in.
	//
	// In general, we can pause after a move if it is the last segment and its end speed is slow enough.
	// We can pause before a move if it is the first segment in that move.
	// The caller should set up rp.feedrate to the default feed rate for the file gcode source before calling this.

	const DDA * const savedDdaRingAddPointer = ddaRingAddPointer;
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
		if (pauseOkHere)
		{
			// We can pause before executing this move
			ddaRingAddPointer = dda;
			break;
		}
		pauseOkHere = dda->CanPauseAfter();
		dda = dda->GetNext();
	}

	cpu_irq_enable();

	// We may be going to skip some moves. Get the end coordinate of the previous move.
	DDA * const prevDda = ddaRingAddPointer->GetPrevious();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = prevDda->GetEndCoordinate(axis, false);
	}

	InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetXAxes(), prevDda->GetYAxes());	// we assume that xAxes hasn't changed between the moves

#if SUPPORT_IOBITS
	rp.ioBits = dda->GetIoBits();
#endif

	if (ddaRingAddPointer == savedDdaRingAddPointer)
	{
		return false;									// we can't skip any moves
	}

	dda = ddaRingAddPointer;
	rp.proportionDone = dda->GetProportionDone(false);	// get the proportion of the current multi-segment move that has been completed
	if (dda->UsingStandardFeedrate())
	{
		rp.feedRate = dda->GetRequestedSpeed();
	}
	rp.virtualExtruderPosition = dda->GetVirtualExtruderPosition();
	rp.filePos = dda->GetFilePosition();

	// Free the DDAs for the moves we are going to skip
	do
	{
		(void)dda->Free();
		dda = dda->GetNext();
		scheduledMoves--;
	}
	while (dda != savedDdaRingAddPointer);

	return true;
}

#if HAS_VOLTAGE_MONITOR

// Pause the print immediately, returning true if we were able to skip or abort any moves and setting up to the move we aborted
bool Move::LowPowerPause(RestorePoint& rp)
{
	const DDA * const savedDdaRingAddPointer = ddaRingAddPointer;
	bool abortedMove = false;

	cpu_irq_disable();
	DDA *dda = currentDda;
	if (dda != nullptr && dda->GetFilePosition() != noFilePosition)
	{
		// We are executing a move that has a file address, so we can interrupt it
		Platform::DisableStepInterrupt();
		dda->MoveAborted();
		CurrentMoveCompleted();							// updates live endpoints, extrusion, ddaRingGetPointer, currentDda etc.
		--completedMoves;								// this move wasn't really completed
		abortedMove = true;
	}
	else
	{
		if (dda == nullptr)
		{
			// No move is being executed
			dda = ddaRingGetPointer;
		}
		while (dda != savedDdaRingAddPointer)
		{
			if (dda->GetFilePosition() != noFilePosition)
			{
				break;									// we can pause before executing this move
			}
			dda = dda->GetNext();
		}
	}

	cpu_irq_enable();

	if (dda == savedDdaRingAddPointer)
	{
		return false;									// we can't skip any moves
	}

	// We are going to skip some moves, or part of a move.
	// Store the parameters of the first move we are going to execute when we resume
	rp.feedRate = dda->GetRequestedSpeed();
	rp.virtualExtruderPosition = dda->GetVirtualExtruderPosition();
	rp.filePos = dda->GetFilePosition();
	rp.proportionDone = dda->GetProportionDone(abortedMove);	// store how much of the complete multi-segment move's extrusion has been done

#if SUPPORT_IOBITS
	rp.ioBits = dda->GetIoBits();
#endif

	ddaRingAddPointer = (abortedMove) ? dda->GetNext() : dda;

	// Get the end coordinates of the last move that was or will be completed, or the coordinates of the current move when we aborted it.
	DDA * const prevDda = ddaRingAddPointer->GetPrevious();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = prevDda->GetEndCoordinate(axis, false);
	}

	InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetXAxes(), prevDda->GetYAxes());	// we assume that xAxes and yAxes have't changed between the moves

	// Free the DDAs for the moves we are going to skip
	for (dda = ddaRingAddPointer; dda != savedDdaRingAddPointer; dda = dda->GetNext())
	{
		(void)dda->Free();
		scheduledMoves--;
	}

	return true;
}

#endif

void Move::Diagnostics(MessageType mtype)
{
	Platform& p = reprap.GetPlatform();
	p.Message(mtype, "=== Move ===\n");
	p.MessageF(mtype, "Hiccups: %" PRIu32 ", StepErrors: %u, LaErrors: %u, FreeDm: %d, MinFreeDm %d, MaxWait: %" PRIu32 "ms, Underruns: %u, %u\n",
						DDA::numHiccups, stepErrors, numLookaheadErrors, DriveMovement::NumFree(), DriveMovement::MinFree(), longestGcodeWaitInterval, numLookaheadUnderruns, numPrepareUnderruns);
	DDA::numHiccups = 0;
	numLookaheadUnderruns = numPrepareUnderruns = numLookaheadErrors = 0;
	longestGcodeWaitInterval = 0;
	DriveMovement::ResetMinFree();

	reprap.GetPlatform().MessageF(mtype, "Scheduled moves: %" PRIu32 ", completed moves: %" PRIu32 "\n", scheduledMoves, completedMoves);

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
		p.MessageF(mtype, " %.3f", (double)probePoints.GetZHeight(i));
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
		reprap.GetPlatform().Message(ErrorMessage, "SetPositions called when DDA ring not empty\n");
	}
}

void Move::EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const
{
	if (CartesianToMotorSteps(coords, ep, true))
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
bool Move::CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const
{
	const bool b = kinematics->CartesianToMotorSteps(machinePos, reprap.GetPlatform().GetDriveStepsPerUnit(),
														reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), motorPos, isCoordinated);
	if (reprap.Debug(moduleMove) && reprap.Debug(moduleDda))
	{
		if (b)
		{
			debugPrintf("Transformed %.2f %.2f %.2f to %" PRIu32 " %" PRIu32 " %" PRIu32 "\n", (double)machinePos[0], (double)machinePos[1], (double)machinePos[2], motorPos[0], motorPos[1], motorPos[2]);
		}
		else
		{
			debugPrintf("Unable to transform %.2f %.2f %.2f\n", (double)machinePos[0], (double)machinePos[1], (double)machinePos[2]);
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

// Get the height error at an XY position
float Move::GetInterpolatedHeightError(float xCoord, float yCoord) const
{
	return (usingMesh) ? heightMap.GetInterpolatedHeightError(xCoord, yCoord) : probePoints.GetInterpolatedHeightError(xCoord, yCoord);
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
						zCorrection += GetInterpolatedHeightError(xCoord, yCoord);
						++numCorrections;
					}
				}
			}
		}

		if (numCorrections > 1)
		{
			zCorrection /= numCorrections;			// take an average
		}

		zCorrection += zShift;
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
					zCorrection += GetInterpolatedHeightError(xCoord, yCoord);
					++numCorrections;
				}
			}
		}
	}

	if (numCorrections > 1)
	{
		zCorrection /= numCorrections;				// take an average
	}

	zCorrection += zShift;

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

// Normalise the bed transform to have zero height error at these coordinates
void Move::SetZeroHeightError(const float coords[MaxAxes])
{
	float tempCoords[MaxAxes];
	memcpy(tempCoords, coords, sizeof(tempCoords));
	AxisTransform(tempCoords, DefaultXAxisMapping, DefaultYAxisMapping);
	zShift = -GetInterpolatedHeightError(tempCoords[X_AXIS], tempCoords[Y_AXIS]);
}

void Move::SetIdentityTransform()
{
	probePoints.SetIdentity();
	heightMap.ClearGridHeights();
	heightMap.UseHeightMap(false);
	usingMesh = false;
	zShift = 0.0;
}

// Load the height map from file, returning true if an error occurred with the error reason appended to the buffer
bool Move::LoadHeightMapFromFile(FileStore *f, const StringRef& r)
{
	const bool ret = heightMap.LoadFromFile(f, r);
	if (ret)
	{
		heightMap.ClearGridHeights();							// make sure we don't end up with a partial height map
	}
	else
	{
		zShift = 0.0;
	}
	return ret;
}

// Save the height map to a file returning true if an error occurred
bool Move::SaveHeightMapToFile(FileStore *f) const
{
	return heightMap.SaveToFile(f, zShift);
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
	usingMesh = heightMap.UseHeightMap(b);
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

// Calibrate or set the bed equation after probing, returning true if an error occurred
// sParam is the value of the S parameter in the G30 command that provoked this call.
// Caller already owns the GCode movement lock.
bool Move::FinishedBedProbing(int sParam, const StringRef& reply)
{
	bool error = false;
	const size_t numPoints = probePoints.NumberOfProbePoints();

	if (sParam < 0)
	{
		// A negative sParam just prints the probe heights
		probePoints.ReportProbeHeights(numPoints, reply);
	}
	else if (numPoints < (size_t)sParam)
	{
		reply.printf("Bed calibration : %d factor calibration requested but only %d points provided\n", sParam, numPoints);
		error = true;
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
			reply.copy("Compensation or calibration cancelled due to probing errors");
			error = true;
		}
		else if (kinematics->SupportsAutoCalibration())
		{
			error = kinematics->DoAutoCalibration(sParam, probePoints, reply);
		}
		else
		{
			error = probePoints.SetProbedBedEquation(sParam, reply);
		}
	}

	// Clear out the Z heights so that we don't re-use old points.
	// This allows us to use different numbers of probe point on different occasions.
	probePoints.ClearProbeHeights();
	return error;
}

// Perform motor endpoint adjustment
void Move::AdjustMotorPositions(const float_t adjustment[], size_t numMotors)
{
	DDA * const lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const int32_t * const endCoordinates = lastQueuedMove->DriveCoordinates();
	const float * const driveStepsPerUnit = reprap.GetPlatform().GetDriveStepsPerUnit();

	for (size_t drive = 0; drive < numMotors; ++drive)
	{
		const int32_t ep = endCoordinates[drive] + lrintf(adjustment[drive] * driveStepsPerUnit[drive]);
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
		if (currentDda->IsNonPrintingExtruderMove(drive))
		{
			extruderNonPrinting[drive - numAxes] = true;
		}
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
void Move::GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const
{
	DDA * const lastQueuedMove = ddaRingAddPointer->GetPrevious();
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t i = 0; i < MaxAxes; i++)
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
void Move::GetCurrentUserPosition(float m[MaxAxes], uint8_t moveType, AxesBitmap xAxes, AxesBitmap yAxes) const
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

// Get the accumulated extruder motor steps taken by an extruder since the last call. Used by the filament monitoring code.
// Returns the number of motor steps moves since the last call, and nonPrinting is true if we are currently executing an extruding but non-printing move
// or we completed one since the last call.
int32_t Move::GetAccumulatedExtrusion(size_t extruder, bool& nonPrinting)
{
	const size_t drive = extruder + reprap.GetGCodes().GetTotalAxes();
	if (drive < DRIVES)
	{
		const irqflags_t flags = cpu_irq_save();
		const int32_t ret = extrusionAccumulators[extruder];
		const DDA * const cdda = currentDda;						// capture volatile variable
		const int32_t adjustment = (cdda == nullptr) ? 0 : cdda->GetStepsTaken(drive);
		if (adjustment == 0)
		{
			// Either there is no current move, or we are at the very start of this move, or it doesn't involve this extruder e.g. a travel move
			nonPrinting = extruderNonPrinting[extruder];
		}
		else if (cdda->IsPrintingMove())
		{
			nonPrinting = extruderNonPrinting[extruder];
			extruderNonPrinting[extruder] = false;
		}
		else
		{
			nonPrinting = true;
		}
		extrusionAccumulators[extruder] = -adjustment;
		cpu_irq_restore(flags);
		return ret + adjustment;
	}

	nonPrinting = true;
	return 0.0;
}

void Move::SetXYBedProbePoint(size_t index, float x, float y)
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform().Message(ErrorMessage, "Z probe point index out of range.\n");
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
		reprap.GetPlatform().Message(ErrorMessage, "Z probe point Z index out of range.\n");
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
		const ZProbe& rp = reprap.GetPlatform().GetCurrentZProbeParameters();
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

// Return the idle timeout in seconds
float Move::IdleTimeout() const
{
	return (float)idleTimeout * 0.001;
}

// Set the idle timeout in seconds
void Move::SetIdleTimeout(float timeout)
{
	idleTimeout = (uint32_t)lrintf(timeout * 1000.0);
}

// Write settings for resuming the print
// The GCodes module deals with the head position so all we need worry about is the bed compensation
// We don't handle random probe point bed compensation, and we assume that if a height map is being used it is the default one.
bool Move::WriteResumeSettings(FileStore *f) const
{
	return kinematics->WriteResumeSettings(f) && (!usingMesh || f->Write("G29 S1\n"));
}

// For debugging
void Move::PrintCurrentDda() const
{
	if (currentDda != nullptr)
	{
		currentDda->DebugPrintAll();
	}
}

// End
