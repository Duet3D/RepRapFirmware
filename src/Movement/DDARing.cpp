/*
 * DDARing.cpp
 *
 *  Created on: 28 Feb 2019
 *      Author: David
 */

#include "DDARing.h"
#include "RepRap.h"
#include "Move.h"

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanInterface.h"
#endif

constexpr uint32_t UsualMinimumPreparedTime = StepTimer::StepClockRate/10;			// 100ms
constexpr uint32_t AbsoluteMinimumPreparedTime = StepTimer::StepClockRate/20;		// 50ms

DDARing::DDARing() : scheduledMoves(0), completedMoves(0)
{
}

// This can be called in the constructor for class Move
void DDARing::Init1(unsigned int numDdas)
{
	numDdasInRing = numDdas;

	// Build the DDA ring
	DDA *dda = new DDA(nullptr);
	addPointer = dda;
	for (size_t i = 1; i < numDdas; i++)
	{
		DDA * const oldDda = dda;
		dda = new DDA(dda);
		oldDda->SetPrevious(dda);
	}
	addPointer->SetNext(dda);
	dda->SetPrevious(addPointer);

	getPointer = checkPointer = addPointer;
	currentDda = nullptr;
}

// This must be called from Move::Init because it indirectly refers to the GCodes module, which must therefore be initialised first
void DDARing::Init2()
{
	stepErrors = 0;
	numLookaheadUnderruns = numPrepareUnderruns = numLookaheadErrors = 0;

	// Put the origin on the lookahead ring with default velocity in the previous position to the first one that will be used.
	// Do this by calling SetLiveCoordinates and SetPositions, so that the motor coordinates will be correct too even on a delta.
	{
		float pos[MaxTotalDrivers];
		for (size_t i = 0; i < MaxTotalDrivers; i++)
		{
			pos[i] = 0.0;
			liveEndPoints[i] = 0;								// not actually right for a delta, but better than printing random values in response to M114
		}
		SetLiveCoordinates(pos);
		SetPositions(pos);
	}

	for (size_t i = 0; i < MaxExtruders; ++i)
	{
		extrusionAccumulators[i] = 0;
		extrusionPending[i] = 0.0;
	}
	extrudersPrinting = false;
	simulationTime = 0.0;
}

void DDARing::Exit()
{
	// Clear the DDA ring so that we don't report any moves as pending
	currentDda = nullptr;
	while (getPointer != addPointer)
	{
		getPointer->Complete();
		getPointer = getPointer->GetNext();
	}

	while (checkPointer->GetState() == DDA::completed)
	{
		(void)checkPointer->Free();
		checkPointer = checkPointer->GetNext();
	}
}

void DDARing::RecycleDDAs()
{
	// Recycle the DDAs for completed moves, checking for DDA errors to print if Move debug is enabled
	while (checkPointer->GetState() == DDA::completed)
	{
		// Check for step errors and record/print them if we have any, before we lose the DMs
		if (checkPointer->HasStepError())
		{
			if (reprap.Debug(moduleMove))
			{
				checkPointer->DebugPrintAll("rd");
			}
			++stepErrors;
			reprap.GetPlatform().LogError(ErrorCode::BadMove);
		}

		// Now release the DMs and check for underrun
		if (checkPointer->Free())
		{
			++numLookaheadUnderruns;
		}
		checkPointer = checkPointer->GetNext();
	}
}

bool DDARing::CanAddMove() const
{
	 if (   addPointer->GetState() == DDA::empty
		 && addPointer->GetNext()->GetState() != DDA::provisional		// function Prepare needs to access the endpoints in the previous move, so don't change them
		)
	 {
			// In order to react faster to speed and extrusion rate changes, only add more moves if the total duration of
			// all un-frozen moves is less than 2 seconds, or the total duration of all but the first un-frozen move is less than 0.5 seconds.
			const DDA *dda = addPointer;
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

			return (unPreparedTime < StepTimer::StepClockRate/2 || unPreparedTime + prevMoveTime < 2 * StepTimer::StepClockRate);
	 }
	 return false;
}

// Add a new move, returning true if it represents real movement
bool DDARing::AddStandardMove(GCodes::RawMove &nextMove, bool doMotorMapping)
{
	if (addPointer->InitStandardMove(*this, nextMove, doMotorMapping))
	{
		addPointer = addPointer->GetNext();
		scheduledMoves++;
		return true;
	}
	return false;
}

// Add a leadscrew levelling motor move
bool DDARing::AddSpecialMove(float feedRate, const float coords[])
{
	if (addPointer->InitLeadscrewMove(*this, feedRate, coords))
	{
		addPointer = addPointer->GetNext();
		scheduledMoves++;
		return true;
	}
	return false;
}

void DDARing::Spin(uint8_t simulationMode, bool shouldStartMove)
{
	// If we are simulating, simulate completion of the current move.
	// Do this here rather than at the end, so that when simulating, currentDda is non-null for most of the time and IsExtruding() returns the correct value
	if (simulationMode != 0)
	{
		DDA * const cdda = currentDda;								// currentDda is declared volatile, so copy it in the next line
		if (cdda != nullptr)
		{
			simulationTime += (float)cdda->GetClocksNeeded()/StepTimer::StepClockRate;
			cdda->Complete();
			CurrentMoveCompleted();
		}
	}

	// See whether we need to kick off a move
	DDA *cdda = currentDda;											// capture volatile variable
	if (cdda == nullptr)
	{
		// No DDA is executing, so start executing a new one if possible
		if (shouldStartMove || !CanAddMove())
		{
			PrepareMoves(getPointer, 0, 0, simulationMode);
			DDA * const dda = getPointer;							// capture volatile variable
			if (dda->GetState() == DDA::frozen)
			{
				if (simulationMode != 0)
				{
					currentDda = dda;								// pretend we are executing this move
				}
				else
				{
					Platform& p = reprap.GetPlatform();
					SetBasePriority(NvicPriorityStep);				// shut out step interrupt
					StartNextMove(p, StepTimer::GetInterruptClocksInterruptsDisabled());	// start the next move
					std::optional<uint32_t> nextInterruptTime = GetNextInterruptTime();
					if (nextInterruptTime.has_value())
					{
						if (StepTimer::ScheduleStepInterrupt(nextInterruptTime.value()))
						{
							Interrupt(p);
						}
					}
					SetBasePriority(0);
				}
			}
		}
	}
	else
	{
		// See whether we need to prepare any moves. First count how many prepared or executing moves we have and how long they will take.
		int32_t preparedTime = 0;
		unsigned int preparedCount = 0;
		DDA::DDAState st;
		while ((st = cdda->GetState()) == DDA::completed || st == DDA::executing || st == DDA::frozen)
		{
			preparedTime += cdda->GetTimeLeft();
			++preparedCount;
			cdda = cdda->GetNext();
			if (cdda == addPointer)
			{
				return;												// all moves are already prepared
			}
		}

		PrepareMoves(cdda, preparedTime, preparedCount, simulationMode);
	}
}

// Prepare some moves. moveTimeLeft is the total length remaining of moves that are already executing or prepared.
void DDARing::PrepareMoves(DDA *firstUnpreparedMove, int32_t moveTimeLeft, unsigned int alreadyPrepared, uint8_t simulationMode)
{
	// If the number of prepared moves will execute in less than the minimum time, prepare another move.
	// Try to avoid preparing deceleration-only moves too early
	while (	  firstUnpreparedMove->GetState() == DDA::provisional
		   && DriveMovement::NumFree() >= (int)MaxTotalDrivers		// check that we won't run out of DMs
		   && moveTimeLeft < (int32_t)UsualMinimumPreparedTime		// prepare moves one eighth of a second ahead of when they will be needed
		   && alreadyPrepared * 2 < numDdasInRing					// but don't prepare more than half the ring
		   && (firstUnpreparedMove->IsGoodToPrepare() || moveTimeLeft < (int32_t)AbsoluteMinimumPreparedTime)
#if SUPPORT_CAN_EXPANSION
		   && CanInterface::CanPrepareMove()
#endif
		  )
	{
		firstUnpreparedMove->Prepare(simulationMode, extrusionPending);
		moveTimeLeft += firstUnpreparedMove->GetTimeLeft();
		++alreadyPrepared;
		firstUnpreparedMove = firstUnpreparedMove->GetNext();
	}
}

// Return true if this DDA ring is idle
bool DDARing::IsIdle() const
{
	return currentDda == nullptr && getPointer->GetState() == DDA::empty;
}

// Try to push some babystepping through the lookahead queue, returning the amount pushed
float DDARing::PushBabyStepping(size_t axis, float amount)
{
	return addPointer->AdvanceBabyStepping(*this, axis, amount);
}

// Try to start another move. Must be called with interrupts disabled, to avoid a race condition.
void DDARing::TryStartNextMove(Platform& p, uint32_t startTime)
{
	const DDA::DDAState st = getPointer->GetState();
	if (st == DDA::frozen)
	{
		StartNextMove(p, startTime);
	}
	else
	{
		if (st == DDA::provisional)
		{
			++numPrepareUnderruns;					// there are more moves available, but they are not prepared yet. Signal an underrun.
		}
		p.ExtrudeOff();								// turn off ancillary PWM
#if SUPPORT_LASER
		if (reprap.GetGCodes().GetMachineType() == MachineType::laser)
		{
			p.SetLaserPwm(0);						// turn off the laser
		}
#endif
	}
}

void DDARing::Interrupt(Platform& p)
{
	DDA* const cdda = currentDda;					// capture volatile variable
	if (cdda != nullptr)
	{
		cdda->StepDrivers(p);						// check endstops if necessary and step the drivers
		if (cdda->GetState() == DDA::completed)
		{
			// The following finish time is wrong if we aborted the move because of endstop or Z probe checks.
			// However, following a move that checks endstops or the Z probe, we always wait for the move to complete before we schedule another, so this doesn't matter.
			const uint32_t finishTime = cdda->GetMoveFinishTime();	// calculate when this move should finish
			CurrentMoveCompleted();					// tell the DDA ring that the current move is complete
			TryStartNextMove(p, finishTime);		// schedule the next move
		}
	}
}

// Insert a brief pause to avoid processor overload
void DDARing::InsertHiccup(uint32_t delayClocks)
{
	DDA* const cdda = currentDda;					// capture volatile variable
	if (cdda != nullptr)
	{
		cdda->InsertHiccup(delayClocks);
		for (DDA *nextDda = cdda->GetNext(); nextDda->GetState() == DDA::frozen; nextDda = nextDda->GetNext())
		{
			nextDda->InsertHiccup(delayClocks);
		}
	}
}

// This is called from the step ISR when the current move has been completed
void DDARing::CurrentMoveCompleted()
{
	// Save the current motor coordinates, and the machine Cartesian coordinates if known
	liveCoordinatesValid = currentDda->FetchEndPosition(const_cast<int32_t*>(liveEndPoints), const_cast<float *>(liveCoordinates));
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t drive = numAxes; drive < MaxTotalDrivers; ++drive)
	{
		extrusionAccumulators[drive - numAxes] += currentDda->GetStepsTaken(drive);
	}
	currentDda = nullptr;

	getPointer = getPointer->GetNext();
	completedMoves++;
}

int32_t DDARing::GetAccumulatedExtrusion(size_t extruder, size_t drive, bool& isPrinting)
{
	const uint32_t basepri = ChangeBasePriority(NvicPriorityStep);
	const int32_t ret = extrusionAccumulators[extruder];
	const DDA * const cdda = currentDda;						// capture volatile variable
	const int32_t adjustment = (cdda == nullptr) ? 0 : cdda->GetStepsTaken(drive);
	extrusionAccumulators[extruder] = -adjustment;
	isPrinting = extrudersPrinting;
	RestoreBasePriority(basepri);
	return ret + adjustment;
}

// Return the untransformed machine coordinates
void DDARing::GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const
{
	DDA * const lastQueuedMove = addPointer->GetPrevious();
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

// These are the actual numbers we want in the positions, so don't transform them.
void DDARing::SetPositions(const float move[MaxTotalDrivers])
{
	if (   getPointer == addPointer								// by itself this means the ring is empty or full
		&& addPointer->GetState() == DDA::DDAState::empty
	   )
	{
		addPointer->GetPrevious()->SetPositions(move, MaxTotalDrivers);
	}
	else
	{
		reprap.GetPlatform().Message(ErrorMessage, "SetPositions called when DDA ring not empty\n");
	}
}

// Perform motor endpoint adjustment
void DDARing::AdjustMotorPositions(const float adjustment[], size_t numMotors)
{
	DDA * const lastQueuedMove = addPointer->GetPrevious();
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

// Return the current live XYZ and extruder coordinates
// Interrupts are assumed enabled on entry
void DDARing::LiveCoordinates(float m[MaxTotalDrivers])
{
	// The live coordinates and live endpoints are modified by the ISR, so be careful to get a self-consistent set of them
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();		// do this before we disable interrupts
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();			// do this before we disable interrupts
	cpu_irq_disable();
	if (liveCoordinatesValid)
	{
		// All coordinates are valid, so copy them across
		memcpy(m, const_cast<const float *>(liveCoordinates), sizeof(m[0]) * MaxTotalDrivers);
		cpu_irq_enable();
	}
	else
	{
		// Only the extruder coordinates are valid, so we need to convert the motor endpoints to coordinates
		memcpy(m + numTotalAxes, const_cast<const float *>(liveCoordinates + numTotalAxes), sizeof(m[0]) * (MaxTotalDrivers - numTotalAxes));
		int32_t tempEndPoints[MaxAxes];
		memcpy(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints));
		cpu_irq_enable();

		reprap.GetMove().MotorStepsToCartesian(tempEndPoints, numVisibleAxes, numTotalAxes, m);		// this is slow, so do it with interrupts enabled

		// If the ISR has not updated the endpoints, store the live coordinates back so that we don't need to do it again
		cpu_irq_disable();
		if (memcmp(tempEndPoints, const_cast<const int32_t*>(liveEndPoints), sizeof(tempEndPoints)) == 0)
		{
			memcpy(const_cast<float *>(liveCoordinates), m, sizeof(m[0]) * numVisibleAxes);
			liveCoordinatesValid = true;
		}
		cpu_irq_enable();
	}
}

// These are the actual numbers that we want to be the coordinates, so don't transform them.
// The caller must make sure that no moves are in progress or pending when calling this
void DDARing::SetLiveCoordinates(const float coords[MaxTotalDrivers])
{
	for (size_t drive = 0; drive < MaxTotalDrivers; drive++)
	{
		liveCoordinates[drive] = coords[drive];
	}
	liveCoordinatesValid = true;
	reprap.GetMove().EndPointToMachine(coords, const_cast<int32_t *>(liveEndPoints), reprap.GetGCodes().GetVisibleAxes());
}

void DDARing::ResetExtruderPositions()
{
	const size_t totalAxes = reprap.GetGCodes().GetTotalAxes();
	cpu_irq_disable();
	for (size_t eDrive = totalAxes; eDrive < MaxTotalDrivers; eDrive++)
	{
		liveCoordinates[eDrive] = 0.0;
	}
	cpu_irq_enable();
}

float DDARing::GetTopSpeed() const
{
	DDA* const cdda = currentDda;					// capture volatile variable
	return (cdda != nullptr) ? cdda->GetTopSpeed() : 0.0;
}

float DDARing::GetRequestedSpeed() const
{
	DDA* const cdda = currentDda;					// capture volatile variable
	return (cdda != nullptr) ? cdda->GetRequestedSpeed() : 0.0;
}

// Pause the print as soon as we can, returning true if we are able to skip any moves and updating 'rp' to the first move we skipped.
bool DDARing::PauseMoves(RestorePoint& rp)
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

	const DDA * const savedDdaRingAddPointer = addPointer;
	bool pauseOkHere;

	cpu_irq_disable();
	DDA *dda = currentDda;
	if (dda == nullptr)
	{
		pauseOkHere = true;								// no move was executing, so we have already paused here whether it was a good idea or not.
		dda = getPointer;
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
			addPointer = dda;
			break;
		}
		pauseOkHere = dda->CanPauseAfter();
		dda = dda->GetNext();
	}

	cpu_irq_enable();

	// We may be going to skip some moves. Get the end coordinate of the previous move.
	DDA * const prevDda = addPointer->GetPrevious();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = prevDda->GetEndCoordinate(axis, false);
	}

	reprap.GetMove().InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetXAxes(), prevDda->GetYAxes());	// we assume that xAxes hasn't changed between the moves

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = dda->GetLaserPwmOrIoBits();
#endif

	if (addPointer == savedDdaRingAddPointer)
	{
		return false;									// we can't skip any moves
	}

	dda = addPointer;
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

#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT

// Pause the print immediately, returning true if we were able to
bool DDARing::LowPowerOrStallPause(RestorePoint& rp)
{
	const DDA * const savedDdaRingAddPointer = addPointer;
	bool abortedMove = false;

	cpu_irq_disable();
	DDA *dda = currentDda;
	if (dda != nullptr && dda->GetFilePosition() != noFilePosition)
	{
		// We are executing a move that has a file address, so we can interrupt it
		StepTimer::DisableStepInterrupt();
#if SUPPORT_LASER
		if (reprap.GetGCodes().GetMachineType() == MachineType::laser)
		{
			reprap.GetPlatform().SetLaserPwm(0);
		}
#endif
		dda->MoveAborted();
		CurrentMoveCompleted();							// updates live endpoints, extrusion, ddaRingGetPointer, currentDda etc.
		--completedMoves;								// this move wasn't really completed
		--scheduledMoves;								// ...but it is no longer scheduled either
		abortedMove = true;
	}
	else
	{
		if (dda == nullptr)
		{
			// No move is being executed
			dda = getPointer;
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

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = dda->GetLaserPwmOrIoBits();
#endif

	addPointer = (abortedMove) ? dda->GetNext() : dda;

	// Get the end coordinates of the last move that was or will be completed, or the coordinates of the current move when we aborted it.
	DDA * const prevDda = addPointer->GetPrevious();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = prevDda->GetEndCoordinate(axis, false);
	}

	reprap.GetMove().InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetXAxes(), prevDda->GetYAxes());	// we assume that xAxes and yAxes have't changed between the moves

	// Free the DDAs for the moves we are going to skip
	for (dda = addPointer; dda != savedDdaRingAddPointer; dda = dda->GetNext())
	{
		(void)dda->Free();
		scheduledMoves--;
	}

	return true;
}

#endif

void DDARing::Diagnostics(MessageType mtype, const char *prefix)
{
	reprap.GetPlatform().MessageF(mtype, "=== %sDDARing ===\nScheduled moves: %" PRIu32 ", completed moves: %" PRIu32 ", StepErrors: %u, LaErrors: %u, Underruns: %u, %u\n",
		prefix, scheduledMoves, completedMoves, stepErrors, numLookaheadErrors, numLookaheadUnderruns, numPrepareUnderruns);
	stepErrors = numLookaheadUnderruns = numPrepareUnderruns = numLookaheadErrors = 0;
}

// End
