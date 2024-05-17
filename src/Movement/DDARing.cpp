/*
 * DDARing.cpp
 *
 *  Created on: 28 Feb 2019
 *      Author: David
 */

#include "DDARing.h"
#include <Platform/RepRap.h>
#include "Move.h"
#include "MoveDebugFlags.h"
#include "RawMove.h"
#include <Platform/Tasks.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Tools/Tool.h>

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanMotion.h"
#endif

constexpr uint32_t MoveStartPollInterval = 10;					// delay in milliseconds between checking whether we should start moves

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(DDARing, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(DDARing, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry DDARing::objectModelTable[] =
{
	// DDARing each group, these entries must be in alphabetical order
	// 0. DDARing members
	{ "gracePeriod",			OBJECT_MODEL_FUNC(self->gracePeriod * MillisToSeconds, 3),			ObjectModelEntryFlags::none },
	{ "length",					OBJECT_MODEL_FUNC((int32_t)self->numDdasInRing), 					ObjectModelEntryFlags::none },
};

constexpr uint8_t DDARing::objectModelTableDescriptor[] = { 1, 2 };

DEFINE_GET_OBJECT_MODEL_TABLE(DDARing)

DDARing::DDARing() noexcept : gracePeriod(DefaultGracePeriod), scheduledMoves(0), completedMoves(0), numHiccups(0)
{
}

// This can be called in the constructor for class Move
void DDARing::Init1(unsigned int numDdas) noexcept
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

	timer.SetCallback(DDARing::TimerCallback, CallbackParameter(this));
}

// This must be called from Move::Init, not from the Move constructor, because it indirectly refers to the GCodes module which must therefore be initialised first
void DDARing::Init2() noexcept
{
	stepErrors = 0;
	numLookaheadUnderruns = numPrepareUnderruns = numNoMoveUnderruns = numLookaheadErrors = 0;
	waitingForRingToEmpty = false;

	// Put the origin on the lookahead ring with default velocity in the previous position to the first one that will be used.
	// Do this by calling SetLiveCoordinates and SetPositions, so that the motor coordinates will be correct too even on a delta.
	{
		float pos[MaxAxesPlusExtruders];
		for (size_t i = 0; i < MaxAxesPlusExtruders; i++)
		{
			pos[i] = 0.0;
		}
		SetPositions(pos);
	}

	for (volatile int32_t& macc : movementAccumulators)
	{
		macc = 0;
	}
	extrudersPrinting = false;
	simulationTime = 0.0;
}

void DDARing::Exit() noexcept
{
	timer.CancelCallback();

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

GCodeResult DDARing::ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	uint32_t numDdasWanted = 0, numDMsWanted = 0;
	gb.TryGetUIValue('P', numDdasWanted, seen);
	gb.TryGetUIValue('S', numDMsWanted, seen);
	gb.TryGetUIValue('R', gracePeriod, seen);
	if (seen)
	{
		if (!reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}

		int64_t memoryNeeded = 0;										// use int64_t for the multiplication to guard against overflow (issue 939)
		if (numDdasWanted > numDdasInRing)
		{
			memoryNeeded += (int64_t)(numDdasWanted - numDdasInRing) * (sizeof(DDA) + 8);
		}
		if (numDMsWanted > DriveMovement::NumCreated())
		{
			memoryNeeded += (int64_t)(numDMsWanted - DriveMovement::NumCreated()) * (sizeof(DriveMovement) + 8);
		}
		if (memoryNeeded != 0)
		{
			memoryNeeded += 1024;					// allow some margin
			const ptrdiff_t memoryAvailable = Tasks::GetNeverUsedRam();
			if (memoryNeeded >= memoryAvailable)
			{
				reply.printf("insufficient RAM (available %d, needed %" PRIi64 ")", memoryAvailable, memoryNeeded);
				return GCodeResult::error;
			}

			// Allocate the extra DDAs and put them in the ring.
			// We must be careful that addPointer->next points to the same DDA as before.
			//TODO can we combine this with the code in Init1?
			while (numDdasWanted > numDdasInRing)
			{
				DDA * const newDda = new DDA(addPointer);
				newDda->SetPrevious(addPointer->GetPrevious());
				addPointer->GetPrevious()->SetNext(newDda);
				addPointer->SetPrevious(newDda);
				++numDdasInRing;
			}

			// Allocate the extra DMs
			DriveMovement::InitialAllocate(numDMsWanted);		// this will only create any extra ones wanted
		}
		reprap.MoveUpdated();
	}
	else
	{
		reply.printf("DDAs %u, DMs %u, GracePeriod %" PRIu32, numDdasInRing, DriveMovement::NumCreated(), gracePeriod);
	}
	return GCodeResult::ok;
}

void DDARing::RecycleDDAs() noexcept
{
	// Recycle the DDAs for completed moves, checking for DDA errors to print if Move debug is enabled
	while (checkPointer->GetState() == DDA::completed && checkPointer != currentDda)	// we haven't finished with a completed DDA until it is no longer the current DDA!
	{
		// Check for step errors and record/print them if we have any, before we lose the DMs
		if (checkPointer->HasStepError())
		{
			if (reprap.GetDebugFlags(Module::Move).IsAnyBitSet(MoveDebugFlags::PrintBadMoves, MoveDebugFlags::PrintAllMoves))
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

bool DDARing::CanAddMove() const noexcept
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

			return (unPreparedTime < StepClockRate/2 || unPreparedTime + prevMoveTime < 2 * StepClockRate);
	 }
	 return false;
}

// Add a new move, returning true if it represents real movement
bool DDARing::AddStandardMove(const RawMove &nextMove, bool doMotorMapping) noexcept
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
bool DDARing::AddSpecialMove(float feedRate, const float coords[MaxDriversPerAxis]) noexcept
{
	if (addPointer->InitLeadscrewMove(*this, feedRate, coords))
	{
		addPointer = addPointer->GetNext();
		scheduledMoves++;
		return true;
	}
	return false;
}

#if SUPPORT_ASYNC_MOVES

// Add an asynchronous motor move
bool DDARing::AddAsyncMove(const AsyncMove& nextMove) noexcept
{
	if (addPointer->InitAsyncMove(*this, nextMove))
	{
		addPointer = addPointer->GetNext();
		scheduledMoves++;
		return true;
	}
	return false;
}

#endif

// Try to process moves in the ring. Called by the Move task.
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or TaskBase::TimeoutUnlimited if there are no unprepared moves left.
uint32_t DDARing::Spin(SimulationMode simulationMode, bool waitingForSpace, bool shouldStartMove) noexcept
{
	DDA *cdda = currentDda;											// capture volatile variable

	// If we are simulating, simulate completion of the current move.
	// Do this here rather than at the end, so that when simulating, currentDda is non-null for most of the time and IsExtruding() returns the correct value
	if (simulationMode != SimulationMode::off && cdda != nullptr)
	{
		simulationTime += (float)cdda->GetClocksNeeded() * (1.0/StepClockRate);
		if (simulationMode == SimulationMode::debug && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::SimulateSteppingDrivers))
		{
			cdda->LatePrepareExtruders();
			do
			{
				cdda->SimulateSteppingDrivers(reprap.GetPlatform());
			} while (cdda->GetState() != DDA::completed);
		}
		else
		{
			cdda->Complete();
		}
		CurrentMoveCompleted();										// this sets currentDda to nullptr and advances getPointer
		DDA * const gp  = getPointer;								// capture volatile variable
		if (gp->GetState() == DDA::frozen)
		{
			cdda = currentDda = gp;									// set up the next move to be simulated
		}
		else
		{
			cdda = nullptr;
		}
	}

	// If we are already moving, see whether we need to prepare any more moves
	if (cdda != nullptr)
	{
		const DDA* const currentMove = cdda;						// save for later

		// Count how many prepared or executing moves we have and how long they will take
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
				return (simulationMode == SimulationMode::off)
						? TaskBase::TimeoutUnlimited				// all the moves we have are already prepared, so nothing to do until new moves arrive
							: 0;
			}
		}

		uint32_t ret = PrepareMoves(cdda, preparedTime, preparedCount, simulationMode);
		if (simulationMode >= SimulationMode::normal)
		{
			return 0;
		}

		if (waitingForSpace)
		{
			// The Move task told us it is waiting for space in the ring, so we need to wake it up soon after we expect the move to finish
			const uint32_t moveTime = currentMove->GetClocksNeeded()/StepClockRate + 1;		// the move time plus 1ms
			if (moveTime < ret)
			{
				ret = moveTime;
			}
		}

		return ret;
	}

	// No DDA is executing, so start executing a new one if possible
	DDA * dda = getPointer;											// capture volatile variable
	if (   shouldStartMove											// if the Move code told us that we should start a move in any case...
		|| waitingForSpace											// ...or the Move code told us it was waiting for space in the ring...
		|| waitingForRingToEmpty									// ...or GCodes is waiting for all moves to finish...
		|| dda->IsCheckingEndstops()								// ...or checking endstops, so we can't schedule the following move
#if SUPPORT_REMOTE_COMMANDS
		|| dda->GetState() == DDA::frozen							// ...or the move has already been frozen (it's probably a remote move)
#endif
	   )
	{
		uint32_t ret = PrepareMoves(dda, 0, 0, simulationMode);

		if (dda->GetState() == DDA::completed)
		{
			// We prepared the move but found there was nothing to do because endstops are already triggered
			getPointer = dda = dda->GetNext();
			completedMoves++;
		}
		else if (dda->GetState() == DDA::frozen)
		{
			if (simulationMode != SimulationMode::off)
			{
				currentDda = dda;									// pretend we are executing this move
				return 0;											// we don't want any delay because we want Spin() to be called again soon to complete this move
			}

			Platform& p = reprap.GetPlatform();
			SetBasePriority(NvicPriorityStep);						// shut out step interrupt
			if (waitingForSpace)
			{
				// The Move task told us it is waiting for space in the ring, so wake it up soon after we expect the move to finish
				const uint32_t moveTime = getPointer->GetClocksNeeded()/StepClockRate + 1;	// the move time plus 1ms
				if (moveTime < ret)
				{
					ret = moveTime;
				}
			}
			const bool wakeLaser = StartNextMove(p, StepTimer::GetTimerTicks());
			if (ScheduleNextStepInterrupt())
			{
				Interrupt(p);
			}
			SetBasePriority(0);
			liveCoordinatesValid = false;

#if SUPPORT_LASER || SUPPORT_IOBITS
			if (wakeLaser)
			{
				Move::WakeLaserTask();
			}
			else
			{
				p.SetLaserPwm(0);
			}
#else
			(void)wakeLaser;
#endif
		}
		return ret;
	}

	return (dda->GetState() == DDA::provisional)
			? MoveStartPollInterval									// there are moves in the queue but it is not time to prepare them yet
				: TaskBase::TimeoutUnlimited;						// the queue is empty, nothing to do until new moves arrive
}

// Prepare some moves. moveTimeLeft is the total length remaining of moves that are already executing or prepared.
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or TaskBase::TimeoutUnlimited if there are no unprepared moves left.
uint32_t DDARing::PrepareMoves(DDA *firstUnpreparedMove, int32_t moveTimeLeft, unsigned int alreadyPrepared, SimulationMode simulationMode) noexcept
{
	// If the number of prepared moves will execute in less than the minimum time, prepare another move.
	// Try to avoid preparing deceleration-only moves too early
	while (	  firstUnpreparedMove->GetState() == DDA::provisional
		   && moveTimeLeft < (int32_t)DDA::UsualMinimumPreparedTime		// prepare moves one tenth of a second ahead of when they will be needed
		   && alreadyPrepared * 2 < numDdasInRing						// but don't prepare more than half the ring, to handle accelerate/decelerate moves in small segments
		   && (firstUnpreparedMove->IsGoodToPrepare() || moveTimeLeft < (int32_t)DDA::AbsoluteMinimumPreparedTime)
#if SUPPORT_CAN_EXPANSION
		   && CanMotion::CanPrepareMove()
#endif
		  )
	{
		firstUnpreparedMove->Prepare(simulationMode);
		moveTimeLeft += firstUnpreparedMove->GetTimeLeft();
		++alreadyPrepared;
		firstUnpreparedMove = firstUnpreparedMove->GetNext();
	}

	// Decide how soon we want to be called again to prepare further moves
	if (firstUnpreparedMove->GetState() == DDA::provisional)
	{
		// There are more moves waiting to be prepared, so ask to be woken up early
		if (simulationMode != SimulationMode::off)
		{
			return 1;
		}

		const int32_t clocksTillWakeup = moveTimeLeft - (int32_t)DDA::UsualMinimumPreparedTime;						// calculate how long before we run out of prepared moves, less the usual advance prepare time
		return (clocksTillWakeup <= 0) ? 2 : min<uint32_t>((uint32_t)clocksTillWakeup/(StepClockRate/1000), 2);		// wake up at that time, but delay for at least 2 ticks
	}

	// There are no moves waiting to be prepared
	return TaskBase::TimeoutUnlimited;
}

// Return true if this DDA ring is idle
bool DDARing::IsIdle() const noexcept
{
	return currentDda == nullptr && getPointer->GetState() == DDA::empty;
}

// Try to push some babystepping through the lookahead queue, returning the amount pushed
// Caution! Thus is called with scheduling locked, therefore it must make no FreeRTOS calls, or call anything that makes them
float DDARing::PushBabyStepping(size_t axis, float amount) noexcept
{
	return addPointer->AdvanceBabyStepping(*this, axis, amount);
}

// ISR for the step interrupt
void DDARing::Interrupt(Platform& p) noexcept
{
	DDA* cdda = currentDda;										// capture volatile variable
	if (cdda != nullptr)
	{
		uint32_t now = StepTimer::GetTimerTicks();
		const uint32_t isrStartTime = now;
		for (;;)
		{
			// Generate a step for the current move
			cdda->StepDrivers(p, now);							// check endstops if necessary and step the drivers
			if (cdda->GetState() == DDA::completed)
			{
#if SUPPORT_CAN_EXPANSION
				if (cdda->IsCheckingEndstops())
				{
					CanMotion::FinishMoveUsingEndstops();		// Tell CAN-connected drivers to revert their position
				}
#endif
				OnMoveCompleted(cdda, p);
				cdda = currentDda;
				if (cdda == nullptr)
				{
					break;
				}
			}

			// Schedule a callback at the time when the next step is due, and quit unless it is due immediately
			if (!cdda->ScheduleNextStepInterrupt(timer))
			{
				break;
			}

			// The next step is due immediately. Check whether we have been in this ISR for too long already and need to take a break
			now = StepTimer::GetTimerTicks();
			const uint32_t clocksTaken = now - isrStartTime;
			if (clocksTaken >= DDA::MaxStepInterruptTime)
			{
				// Force a break by updating the move start time.
				++numHiccups;
#if SUPPORT_CAN_EXPANSION
				uint32_t cumulativeHiccupTime = 0;
#endif
				for (uint32_t hiccupTime = DDA::HiccupTime; ; hiccupTime += DDA::HiccupIncrement)
				{
#if SUPPORT_CAN_EXPANSION
					cumulativeHiccupTime += cdda->InsertHiccup(now + hiccupTime);
#else
					cdda->InsertHiccup(now + hiccupTime);
#endif
					// Reschedule the next step interrupt. This time it should succeed if the hiccup time was long enough.
					if (!cdda->ScheduleNextStepInterrupt(timer))
					{
#if SUPPORT_CAN_EXPANSION
						CanMotion::InsertHiccup(cumulativeHiccupTime);
#endif
						return;
					}
					// We probably had an interrupt that delayed us further. Recalculate the hiccup length, also we increase the hiccup time on each iteration.
					now = StepTimer::GetTimerTicks();
				}
			}
		}
	}
}

// DDARing timer callback function
/*static*/ void DDARing::TimerCallback(CallbackParameter p) noexcept
{
	static_cast<DDARing*>(p.vp)->Interrupt(reprap.GetPlatform());
}

// This is called when the state has been set to 'completed'. Step interrupts must be disabled or locked out when calling this.
void DDARing::OnMoveCompleted(DDA *cdda, Platform& p) noexcept
{
	bool wakeLaserTask = false;

#if SUPPORT_SCANNING_PROBES
	if (cdda->IsScanningProbeMove())
	{
		reprap.GetMove().SetProbeReadingNeeded();
		wakeLaserTask = true;						// wake the laser task to take a reading
	}
#endif

	// The following finish time is wrong if we aborted the move because of endstop or Z probe checks.
	// However, following a move that checks endstops or the Z probe, we always wait for the move to complete before we schedule another, so this doesn't matter.
	const uint32_t finishTime = cdda->GetMoveFinishTime();	// calculate when this move should finish

	CurrentMoveCompleted();							// tell the DDA ring that the current move is complete

	// Try to start a new move
	const DDA::DDAState st = getPointer->GetState();
	if (st == DDA::frozen)
	{
#if SUPPORT_LASER || SUPPORT_IOBITS
		if (StartNextMove(p, finishTime))
		{
			wakeLaserTask = true;
		}
#else
		(void)StartNextMove(p, finishTime);
#endif
	}
	else
	{
		if (st == DDA::provisional)
		{
			++numPrepareUnderruns;					// there are more moves available, but they are not prepared yet. Signal an underrun.
		}
		else if (!waitingForRingToEmpty)
		{
			++numNoMoveUnderruns;
		}
		p.ExtrudeOff();								// turn off ancillary PWM
		if (cdda->GetTool() != nullptr)
		{
			cdda->GetTool()->StopFeedForward();
		}
#if SUPPORT_LASER
		if (reprap.GetGCodes().GetMachineType() == MachineType::laser)
		{
			p.SetLaserPwm(0);						// turn off the laser
		}
#endif
		waitingForRingToEmpty = false;
	}

	if (wakeLaserTask)
	{
		Move::WakeLaserTaskFromISR();
	}
}

// This is called from the step ISR when the current move has been completed. It may also be called from other places.
void DDARing::CurrentMoveCompleted() noexcept
{
	DDA * const cdda = currentDda;					// capture volatile variable

#if SUPPORT_REMOTE_COMMANDS
	for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
	{
		lastMoveStepsTaken[driver] = cdda->GetStepsTaken(driver);
	}
#endif

	// Accumulate the extrusion from this move
	for (size_t drive = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); drive < MaxAxesPlusExtruders; ++drive)
	{
		liveCoordinates[drive] += cdda->GetRawEndCoordinate(drive);
	}

	// Disable interrupts before we touch any extrusion accumulators until after we set currentDda to null, in case the filament monitor interrupt has higher priority than ours
	{
		AtomicCriticalSectionLocker lock;
		cdda->UpdateMovementAccumulators(movementAccumulators);
		if (cdda->IsCheckingEndstops())
		{
			Move::WakeMoveTaskFromISR();			// wake the Move task if we were checking endstops
		}
		currentDda = nullptr;						// once we have done this, the DDA can be recycled by the Move task
	}

	getPointer = getPointer->GetNext();
	completedMoves++;
}

// Tell the DDA ring that the caller is waiting for it to empty. Returns true if it is already empty.
bool DDARing::SetWaitingToEmpty() noexcept
{
	waitingForRingToEmpty = true;					// set this first to avoid a possible race condition
	const bool ret = IsIdle();
	if (ret)
	{
		waitingForRingToEmpty = false;
	}
	return ret;
}

// Get the number of steps taken by a logical drive since the last time we called this function for that drive
int32_t DDARing::GetAccumulatedMovement(size_t drive, bool& isPrinting) noexcept
{
	AtomicCriticalSectionLocker lock;							// we don't want a move to complete and the ISR update the movement accumulators while we are doing this
	const DDA * const cdda = currentDda;						// capture volatile variable
	const int32_t ret = movementAccumulators[drive];
	const int32_t adjustment = (cdda == nullptr) ? 0 : cdda->GetStepsTaken(drive);
	movementAccumulators[drive] = -adjustment;
	isPrinting = extrudersPrinting;
	return ret + adjustment;
}

// Return the untransformed machine coordinates
void DDARing::GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const noexcept
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

#if SUPPORT_ASYNC_MOVES

// Return the machine coordinates of just some axes in the last queued move.
// On machines with nonlinear kinematics this is quite likely to return coordinates slightly different from the original ones.
void DDARing::GetPartialMachinePosition(float m[MaxAxes], AxesBitmap whichAxes) const noexcept
{
	DDA * const lastQueuedMove = addPointer->GetPrevious();
	whichAxes.Iterate([m, lastQueuedMove](unsigned int axis, unsigned int count) { m[axis] = lastQueuedMove->GetEndCoordinate(axis, false); });
}

#endif

// Set the initial machine coordinates for the next move to be added to the specified values, by setting the final coordinates of the last move in the queue
// The last move in the queue must have already been set up by the Move process before this is called.
void DDARing::SetPositions(const float move[MaxAxesPlusExtruders]) noexcept
{
	AtomicCriticalSectionLocker lock;
	liveCoordinatesValid = false;
	addPointer->GetPrevious()->SetPositions(move);
}

// Reset the extruder positions. Should only be called when the DDA ring is empty.
void DDARing::ResetExtruderPositions() noexcept
{
	AtomicCriticalSectionLocker lock;
	liveCoordinatesValid = false;
	for (size_t eDrive = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); eDrive < MaxAxesPlusExtruders; eDrive++)
	{
		liveCoordinates[eDrive] = 0.0;
	}
}

// Perform motor endpoint adjustment
void DDARing::AdjustMotorPositions(const float adjustment[], size_t numMotors) noexcept
{
	DDA * const lastQueuedMove = addPointer->GetPrevious();
	const int32_t * const endCoordinates = lastQueuedMove->DriveCoordinates();
	const float * const driveStepsPerUnit = reprap.GetPlatform().GetDriveStepsPerUnit();

	for (size_t drive = 0; drive < numMotors; ++drive)
	{
		const int32_t ep = endCoordinates[drive] + lrintf(adjustment[drive] * driveStepsPerUnit[drive]);
		lastQueuedMove->SetDriveCoordinate(ep, drive);
	}

	liveCoordinatesValid = false;		// force the live XYZ position to be recalculated
}

// Get the live motor positions
void DDARing::GetCurrentMotorPositions(int32_t pos[MaxAxesPlusExtruders]) const noexcept
{
	AtomicCriticalSectionLocker lock;
	const DDA *const cdda = currentDda;									// capture volatile variable
	if (cdda == nullptr)
	{
		getPointer->GetPrevious()->FetchEndPoints(pos);
	}
	else
	{
		cdda->FetchCurrentPositions(pos);
	}
}

// Fetch the current axis and extruder coordinates
// Interrupts are assumed enabled on entry
void DDARing::LiveCoordinates(float m[MaxAxesPlusExtruders]) noexcept
{
	// The live coordinates and live endpoints are modified by the ISR, so be careful to get a self-consistent set of them
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();		// do this before we disable interrupts
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();			// do this before we disable interrupts
	IrqDisable();
	if (liveCoordinatesValid)
	{
		// All coordinates are valid, so copy them across
		memcpyf(m, const_cast<const float *>(liveCoordinates), MaxAxesPlusExtruders);
		IrqEnable();
	}
	else
	{
		// Get the positions of each motor
		int32_t currentMotorPositions[MaxAxesPlusExtruders];
		GetCurrentMotorPositions(currentMotorPositions);
		const DDA * const cdda = currentDda;
		IrqEnable();

		reprap.GetMove().MotorStepsToCartesian(currentMotorPositions, numVisibleAxes, numTotalAxes, m);		// this is slow, so do it with interrupts enabled

		const size_t numExtruders = reprap.GetGCodes().GetNumExtruders();
		if (cdda != nullptr)
		{
			// Add extrusion so far in the current move to the accumulated extrusion
			for (size_t i = MaxAxesPlusExtruders - numExtruders; i < MaxAxesPlusExtruders; ++i)
			{
				m[i] = liveCoordinates[i] + currentMotorPositions[i] / reprap.GetPlatform().DriveStepsPerUnit(i);
			}
		}
		else
		{
			for (size_t i = MaxAxesPlusExtruders - numExtruders; i < MaxAxesPlusExtruders; ++i)
			{
				m[i] = liveCoordinates[i];
			}

			// Optimisation: if no movement, save the positions for next time
			IrqDisable();
			if (currentDda == nullptr)
			{
				memcpyf(const_cast<float *>(liveCoordinates), m, numVisibleAxes);
				liveCoordinatesValid = true;
			}
			IrqEnable();
		}
	}
}

float DDARing::GetRequestedSpeedMmPerSec() const noexcept
{
	const DDA* const cdda = currentDda;					// capture volatile variable
	return (cdda != nullptr) ? cdda->GetRequestedSpeedMmPerSec() : 0.0;
}

float DDARing::GetTopSpeedMmPerSec() const noexcept
{
	const DDA* const cdda = currentDda;					// capture volatile variable
	return (cdda != nullptr) ? cdda->GetTopSpeedMmPerSec() : 0.0;
}

float DDARing::GetAccelerationMmPerSecSquared() const noexcept
{
	const DDA* const cdda = currentDda;					// capture volatile variable
	return (cdda != nullptr) ? cdda->GetAccelerationMmPerSecSquared() : 0.0;
}

float DDARing::GetDecelerationMmPerSecSquared() const noexcept
{
	const DDA* const cdda = currentDda;					// capture volatile variable
	return (cdda != nullptr) ? cdda->GetDecelerationMmPerSecSquared() : 0.0;
}

float DDARing::GetTotalExtrusionRate() const noexcept
{
	const DDA* const cdda = currentDda;					// capture volatile variable
	return (cdda != nullptr) ? cdda->GetTotalExtrusionRate() : 0.0;
}

// Pause the print as soon as we can.
// If we are able to skip any moves, return true and update ms.pauseRestorePoint to the first move we skipped.
// If we can't skip any moves, update just the coordinates and laser PWM in ms.pauseRestorePoint and return false.
bool DDARing::PauseMoves(MovementState& ms) noexcept
{
	// Find a move we can pause after.
	// Ideally, we would adjust a move if necessary and possible so that we can pause after it, but for now we don't do that.
	// There are a few possibilities:
	// 1. There is no currently executing move and no moves in the queue, and GCodes does not have a move for us.
	//    Pause immediately. Resume from the current file position.
	// 2. There is no currently executing move and no moves in the queue, and GCodes has a move for us but that move has not been started.
	//    Pause immediately. Discard the move that GCodes has for us, and resume from the start file position of that move.
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

	TaskCriticalSectionLocker lock;						// prevent the Move task changing data while we look at it

	const DDA * const savedDdaRingAddPointer = addPointer;
	bool pauseOkHere;

	IrqDisable();
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

	while (dda != savedDdaRingAddPointer)				// while there are queued moves
	{
		if (pauseOkHere)								// if we can pause before executing the move that dda refers to
		{
			addPointer = dda;
			dda->Free();								// set the move status to empty so that when we re-enable interrupts the ISR doesn't start executing it
			break;
		}
		pauseOkHere = dda->CanPauseAfter();
		dda = dda->GetNext();
	}

	IrqEnable();

	// We may be going to skip some moves. Get the end coordinate of the previous move.
	DDA * const prevDda = addPointer->GetPrevious();
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	RestorePoint& rp = ms.GetPauseRestorePoint();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		rp.moveCoords[axis] = prevDda->GetEndCoordinate(axis, false);
	}

	reprap.GetMove().InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetTool());

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = dda->GetLaserPwmOrIoBits();
#endif

	if (addPointer == savedDdaRingAddPointer)
	{
		return false;										// we can't skip any moves
	}

	dda = addPointer;
	rp.proportionDone = dda->GetProportionDone(false);		// get the proportion of the current multi-segment move that has been completed
	rp.initialUserC0 = dda->GetInitialUserC0();
	rp.initialUserC1 = dda->GetInitialUserC1();
	const float rawFeedRate = (dda->UsingStandardFeedrate()) ? dda->GetRequestedSpeedMmPerClock() : ms.feedRate;	// this is the requested feed rate after applying the speed factor
	rp.feedRate = rawFeedRate/ms.speedFactor;				// correct it for the speed factor, assuming that the speed factor hasn't changed
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
bool DDARing::LowPowerOrStallPause(RestorePoint& rp) noexcept
{
	TaskCriticalSectionLocker lock;						// prevent the Move task changing data while we look at it

	const DDA * const savedDdaRingAddPointer = addPointer;
	bool abortedMove = false;

	IrqDisable();
	DDA *dda = currentDda;
	if (dda != nullptr && dda->GetFilePosition() != noFilePosition)
	{
		// We are executing a move that has a file address, so we can interrupt it
		timer.CancelCallback();
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

	IrqEnable();

	if (dda == savedDdaRingAddPointer)
	{
		return false;									// we can't skip any moves
	}

	// We are going to skip some moves, or part of a move.
	// Store the parameters of the first move we are going to execute when we resume
	rp.feedRate = dda->GetRequestedSpeedMmPerClock();
	rp.virtualExtruderPosition = dda->GetVirtualExtruderPosition();
	rp.filePos = dda->GetFilePosition();
	rp.proportionDone = dda->GetProportionDone(abortedMove);	// store how much of the complete multi-segment move's extrusion has been done
	rp.initialUserC0 = dda->GetInitialUserC0();
	rp.initialUserC1 = dda->GetInitialUserC1();

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

	reprap.GetMove().InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetTool());

	// Free the DDAs for the moves we are going to skip
	for (dda = addPointer; dda != savedDdaRingAddPointer; dda = dda->GetNext())
	{
		(void)dda->Free();
		scheduledMoves--;
	}

	return true;
}

#endif

void DDARing::Diagnostics(MessageType mtype, unsigned int ringNumber) noexcept
{
	const DDA * const cdda = currentDda;
	reprap.GetPlatform().MessageF(mtype,
									"=== DDARing %u ===\nScheduled moves %" PRIu32 ", completed %" PRIu32 ", hiccups %" PRIu32 ", stepErrors %u, LaErrors %u, Underruns [%u, %u, %u], CDDA state %d\n",
									ringNumber, scheduledMoves, completedMoves, numHiccups, stepErrors, numLookaheadErrors, numLookaheadUnderruns, numPrepareUnderruns, numNoMoveUnderruns,
									(cdda == nullptr) ? -1 : (int)cdda->GetState());
	numHiccups = stepErrors = numLookaheadUnderruns = numPrepareUnderruns = numNoMoveUnderruns = numLookaheadErrors = 0;
}

#if SUPPORT_LASER

// Manage the laser power. Return the number of ticks until we should be called again, or 0 to be called at the start of the next move.
uint32_t DDARing::ManageLaserPower() const noexcept
{
	SetBasePriority(NvicPriorityStep);							// lock out step interrupts
	DDA * const cdda = currentDda;								// capture volatile variable
	if (cdda != nullptr)
	{
		const uint32_t ret = cdda->ManageLaserPower();
		SetBasePriority(0);
		return ret;
	}

	// If we get here then there is no active laser move
	SetBasePriority(0);
	reprap.GetPlatform().SetLaserPwm(0);						// turn off the laser
	return 0;
}

#endif

#if SUPPORT_REMOTE_COMMANDS

// Add a move from the ATE to the movement queue
void DDARing::AddMoveFromRemote(const CanMessageMovementLinear& msg) noexcept
{
	if (addPointer->GetState() == DDA::empty)
	{
		if (addPointer->InitFromRemote(msg))
		{
			addPointer = addPointer->GetNext();
			scheduledMoves++;
		}
	}
}

// Add a move from the ATE to the movement queue
void DDARing::AddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept
{
	if (addPointer->GetState() == DDA::empty)
	{
		if (addPointer->InitFromRemote(msg))
		{
			addPointer = addPointer->GetNext();
			scheduledMoves++;
		}
	}
}

void DDARing::StopDrivers(uint16_t whichDrives) noexcept
{
	const uint32_t oldPrio = ChangeBasePriority(NvicPriorityStep);
	DDA *cdda = currentDda;							// capture volatile
	if (cdda != nullptr)
	{
		cdda->StopDrivers(whichDrives);
		if (cdda->GetState() == DDA::completed)
		{
			CurrentMoveCompleted();					// tell the DDA ring that the current move is complete
		}
	}
	RestoreBasePriority(oldPrio);
}

#endif

// End
