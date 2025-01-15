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
#include <Platform/Platform.h>
#include <Platform/Tasks.h>
#include <Platform/PortControl.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Tools/Tool.h>

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanMotion.h"
#endif

/* Note on how the DDA ring works, using the new step-generation code that implements late input shaping:
 * A DDA represents a straight-line move with at least one of an acceleration segment, a steady speed segment, and a deceleration segment.
 * A single G0 or G1 command may be represented by a single DDA, or by multiple DDAs when the move has been segmented.
 *
 * DDAs are added to a ring in response to G0, G1, G2 and G3 commands and when RRF generates movement automatically (e.g. probing moves).
 * A newly-added DDA is in state 'provisional' and has its end speed set to zero. In this state its speed, acceleration and deceleration can be modified.
 * These modifications happen as other DDAs are added to the ring and the DDAs are adjusted to give a smooth transition between them.
 *
 * Shortly before a move is due to be executed, DDA::Prepare is called. This causes the move parameters to be frozen.
 * Move segments are generated, and/or the move details are sent to CAN-connected expansion boards. The DDA state is set to "scheduled".
 *
 * The scheduled DDA remains in the ring until the time for it to finish executing has passed, in order that we can report on
 * the parameters of the currently-executing move, e.g. requested and top speeds, extrusion rate, and extrusion amount for the filament monitor.
 *
 * When a move requires that endstops and/or Z probes are active, all other moves are completed before starting it, and no new moves are allowed
 * to be added to the ring until it completes. So it is the only move in the ring with state 'scheduled'.
 */

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

DDARing::DDARing() noexcept : gracePeriod(DefaultGracePeriod), scheduledMoves(0), completedMoves(0)
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

	getPointer = addPointer;
	lastFeedForwardTool = nullptr;
	lastAverageExtrusionSpeed = 0.0;
}

// This must be called from Move::Init, not from the Move constructor, because it indirectly refers to the GCodes module which must therefore be initialised first
void DDARing::Init2() noexcept
{
	numLookaheadUnderruns = numPrepareUnderruns = numNoMoveUnderruns = numLookaheadErrors = 0;
	waitingForRingToEmpty = false;
	simulationTime = 0.0;
}

void DDARing::Exit() noexcept
{
	// Clear the DDA ring so that we don't report any moves as pending
	DDA *gp;										// use a local variable to avoid loading volatile variable getPointer too often
	while ((gp = getPointer) != addPointer)
	{
		gp->Free();
		getPointer = gp = gp->GetNext();
	}
}

GCodeResult DDARing::ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	uint32_t numDdasWanted = 0;
	gb.TryGetUIValue('P', numDdasWanted, seen);
	gb.TryGetUIValue('R', gracePeriod, seen);
	if (seen)
	{
		if (!reprap.GetGCodes().LockAllMovementSystemsAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}

		if (numDdasWanted > numDdasInRing)
		{
			// Use int64_t for the multiplication to guard against overflow (issue 939)
			int64_t memoryNeeded = (int64_t)((numDdasWanted - numDdasInRing) * (sizeof(DDA) + 8));
			memoryNeeded += 1024;					// allow some margin
			const ptrdiff_t memoryAvailable = Tasks::GetNeverUsedRam();
			if (memoryNeeded >= memoryAvailable)
			{
				reply.printf("insufficient RAM (available %d, needed %" PRIu64 ")", memoryAvailable, memoryNeeded);
				return GCodeResult::error;
			}

			// Allocate the extra DDAs and put them in the ring.
			// We must be careful that addPointer->prev points to the same DDA as before because we fetch the endpoints from there.
			//TODO can we combine this with the code in Init1?
			while (numDdasWanted > numDdasInRing)
			{
				TaskCriticalSectionLocker lock;
				DDA * const newDda = new DDA(addPointer->GetNext());
				addPointer->GetNext()->SetPrevious(newDda);
				addPointer->SetNext(newDda);
				newDda->SetPrevious(addPointer);
				++numDdasInRing;
			}
		}
		reprap.MoveUpdated();
	}
	else
	{
		reply.printf("DDAs %u, GracePeriod %" PRIu32, numDdasInRing, gracePeriod);
	}
	return GCodeResult::ok;
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
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or MoveTiming::StandardMoveWakeupInterval if there are no unprepared moves left.
uint32_t DDARing::Spin(SimulationMode simulationMode, bool signalMoveCompletion, bool shouldStartMove) noexcept
{
	DDA *cdda = getPointer;											// capture volatile variable

	// If we are simulating, simulate completion of the current move
	if (simulationMode >= SimulationMode::normal)
	{
		// Simulate completion of one move
		if (cdda->IsCommitted())
		{
			simulationTime += (float)cdda->GetClocksNeeded() * (1.0/StepClockRate);
			++completedMoves;
			if (cdda->Free())
			{
				++numLookaheadUnderruns;
			}
			getPointer = cdda = cdda->GetNext();
		}
	}
	else
	{
		// See if we can retire any completed moves
		while (cdda->IsCommitted() && cdda->HasExpired())
		{
			++completedMoves;
			//debugPrintf("Retiring move: now=%" PRIu32 " start=%" PRIu32 " dur=%" PRIu32 "\n", StepTimer::GetMovementTimerTicks(), cdda->GetMoveStartTime(), cdda->GetClocksNeeded());
			if (cdda->Free())
			{
				++numLookaheadUnderruns;
			}
			getPointer = cdda = cdda->GetNext();
		}
	}

	// If we are already moving, see whether we need to prepare any more moves
	if (cdda->IsCommitted())										// if we have started executing moves
	{
		const DDA* const currentMove = cdda;						// save for later

		// Count how many prepared or executing moves we have and how long they will take
		uint32_t preparedTime = 0;
		unsigned int preparedCount = 0;
		while (cdda->IsCommitted())
		{
			preparedTime += cdda->GetTimeLeft();
			++preparedCount;
			cdda = cdda->GetNext();
		}

		const uint32_t ret = (cdda->GetState() == DDA::provisional)
						? PrepareMoves(cdda, preparedTime, preparedCount, simulationMode)
							: MoveTiming::StandardMoveWakeupInterval;

		if (simulationMode != SimulationMode::off)
		{
			return 0;
		}

		if (signalMoveCompletion || waitingForRingToEmpty || currentMove->IsIsolatedMove())
		{
			// Wake up the Move task shortly after we expect the current move to finish
			const int32_t moveTicksLeft = currentMove->GetMoveFinishTime() - StepTimer::GetMovementTimerTicks();
			if (moveTicksLeft < 0)
			{
				return 0;
			}

			const uint32_t moveTime = (uint32_t)moveTicksLeft/(StepClockRate/1000) + 1;	// 1ms ticks until the move finishes plus 1ms
			if (moveTime < ret)
			{
				return moveTime;
			}
		}

		return ret;
	}

	// No DDA is committed, so commit a new one if possible
	if (   shouldStartMove											// if the Move code told us that we should start a move in any case...
		|| waitingForRingToEmpty									// ...or GCodes is waiting for all moves to finish...
		|| cdda->IsIsolatedMove()									// ...or checking endstops or another isolated move, so we can't schedule the following move
	   )
	{
		const uint32_t ret = PrepareMoves(cdda, 0, 0, simulationMode);
		if (cdda->IsCommitted())
		{
			if (simulationMode != SimulationMode::off)
			{
				return 0;											// we don't want any delay because we want Spin() to be called again soon to complete this move
			}

			reprap.GetMove().WakeLaserTask();						// tell the laser task about this move
			if (signalMoveCompletion || waitingForRingToEmpty || cdda->IsIsolatedMove())
			{
				// Wake up the Move task shortly after we expect the current move to finish
				const int32_t moveTicksLeft = cdda->GetMoveFinishTime() - StepTimer::GetMovementTimerTicks();
				if (moveTicksLeft < 0)
				{
					return 0;
				}

				const uint32_t moveTime = (uint32_t)moveTicksLeft/(StepClockRate/1000) + 1;	// 1ms ticks until the move finishes plus 1ms
				if (moveTime < ret)
				{
					return moveTime;
				}
			}
		}
		return ret;
	}

	return (cdda->GetState() == DDA::provisional)
			? MoveStartPollInterval									// there are moves in the queue but it is not time to prepare them yet
				: MoveTiming::StandardMoveWakeupInterval;			// the queue is empty, nothing to do until new moves arrive
}

// Prepare some moves. moveTimeLeft is the total length remaining of moves that are already executing or prepared.
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or MoveTiming::StandardMoveWakeupInterval if there are no unprepared moves left.
uint32_t DDARing::PrepareMoves(DDA *firstUnpreparedMove, uint32_t moveTimeLeft, unsigned int alreadyPrepared, SimulationMode simulationMode) noexcept
{
	// If the already-prepared moves will execute in less than the minimum time, prepare another move.
	// Try to avoid preparing deceleration-only moves too early
	while (	  firstUnpreparedMove->GetState() == DDA::provisional
		   && moveTimeLeft < MoveTiming::UsualMinimumPreparedTime	// prepare moves one tenth of a second ahead of when they will be needed
		   && alreadyPrepared * 2 < numDdasInRing					// but don't prepare more than half the ring, to handle accelerate/decelerate moves in small segments
#if SUPPORT_CAN_EXPANSION
		   && CanMotion::CanPrepareMove()
#endif
		  )
	{
		firstUnpreparedMove->Prepare(*this, simulationMode);
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

		const int32_t clocksTillWakeup = (int32_t)(moveTimeLeft - MoveTiming::UsualMinimumPreparedTime);			// calculate how long before we run out of prepared moves, less the usual advance prepare time
		return (clocksTillWakeup <= 0) ? 2 : max<uint32_t>((uint32_t)clocksTillWakeup/(StepClockRate/1000), 2);		// wake up at that time, but delay for at least 2 ticks
	}

	// There are no moves waiting to be prepared
	return MoveTiming::StandardMoveWakeupInterval;
}

// Return true if this DDA ring is idle
bool DDARing::IsIdle() const noexcept
{
	return getPointer->GetState() == DDA::empty;
}

// Try to push some babystepping through the lookahead queue, returning the amount pushed
// Caution! Thus is called with scheduling locked, therefore it must make no FreeRTOS calls, or call anything that makes them
float DDARing::PushBabyStepping(size_t axis, float amount) noexcept
{
	const float ret = addPointer->AdvanceBabyStepping(*this, axis, amount);
	startCoordinates[axis] += ret;
	return ret;
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

// Return the untransformed machine coordinates
void DDARing::GetCurrentMachinePosition(float m[MaxAxes]) const noexcept
{
	addPointer->GetPrevious()->GetEndCoordinates(m);
}

void DDARing::GetLastEndpoints(LogicalDrivesBitmap logicalDrives, int32_t returnedEndpoints[MaxAxesPlusExtruders]) const noexcept
{
	logicalDrives.Iterate([this, returnedEndpoints](unsigned int drive, unsigned int count) noexcept { returnedEndpoints[drive] = addPointer->GetPrevious()->DriveCoordinates()[drive]; } );
}

int32_t DDARing::GetLastEndpoint(size_t drive) const noexcept
{
	return addPointer->GetPrevious()->DriveCoordinates()[drive];
}

// Set the endpoints of some drives that we have just allocated. The drives must not be owned in the previous move!
void DDARing::SetLastEndpoints(LogicalDrivesBitmap logicalDrives, const int32_t *_ecv_array ep) noexcept
{
	DDA *prev = addPointer->GetPrevious();
	logicalDrives.Iterate([prev, ep](unsigned int drive, unsigned int count) noexcept
							{
								prev->SetDriveCoordinate(drive, ep[drive]);
							});
}

void DDARing::SetLastEndpoint(size_t drive, int32_t ep) noexcept
{
	addPointer->GetPrevious()->SetDriveCoordinate(drive, ep);
}

// Update the start coordinates for the next move.
// Called after a raw motor move has changed the endpoints.
void DDARing::UpdateStartCoordinates(const float coords[MaxAxes]) noexcept
{
	memcpyf(startCoordinates, coords, MaxAxes);
}

// Get the DDA that should currently be executing, or nullptr if no move from this ring should be executing
DDA *_ecv_null DDARing::GetCurrentDDA() const noexcept
{
	TaskCriticalSectionLocker lock;
	DDA *cdda = getPointer;
	const uint32_t now = StepTimer::GetMovementTimerTicks();
	while (cdda->IsCommitted())
	{
		const uint32_t timeRunning = now - cdda->GetMoveStartTime();
		if ((int32_t)timeRunning < 0) { break; }			// move has not started yet
		if (timeRunning < cdda->GetClocksNeeded()) { return cdda; }
		cdda = cdda->GetNext();								// move has completed so look at the next one
	}
	return nullptr;
}

// Get various data for reporting in the OM
float DDARing::GetRequestedSpeedMmPerSec() const noexcept
{
	const DDA *_ecv_null const cdda = GetCurrentDDA();
	return (cdda != nullptr) ? cdda->GetRequestedSpeedMmPerSec() : 0.0;
}

float DDARing::GetTopSpeedMmPerSec() const noexcept
{
	const DDA *_ecv_null const cdda = GetCurrentDDA();
	return (cdda != nullptr) ? cdda->GetTopSpeedMmPerSec() : 0.0;
}

// Get the (peak) acceleration for reporting in the object model
float DDARing::GetAccelerationMmPerSecSquared() const noexcept
{
	const DDA *_ecv_null const cdda = GetCurrentDDA();
	return (cdda != nullptr) ? cdda->GetAccelerationMmPerSecSquared() : 0.0;
}

// Get the (peak) deceleration for reporting in the object model
float DDARing::GetDecelerationMmPerSecSquared() const noexcept
{
	const DDA *_ecv_null const cdda = GetCurrentDDA();
	return (cdda != nullptr) ? cdda->GetDecelerationMmPerSecSquared() : 0.0;
}

float DDARing::GetTotalExtrusionRate() const noexcept
{
	const DDA *_ecv_null const cdda = GetCurrentDDA();
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
	// 1. We have skipped some moves in the queue. Update the pause restore point with the end coordinates of the last move we executed, the file address of the first move we have skipped
	// the feed rate at the start of that move, and the iobits at the start of that move, and return true.
	// 2. All moves in the queue need to be executed. Also any move held by GCodes needs to be completed it is it not the first segment.
	//    Update the pause restore point with the coordinates and iobits as at the end of the previous move and return false.
	//    The extruder position, file position and feed rate are not filled in.
	//
	// In general, we can pause after a move if it is the last segment and its end speed is slow enough.
	// We can pause before a move if it is the first segment in that move.
	// The caller should set up rp.feedrate to the default feed rate for the file gcode source before calling this.

	TaskCriticalSectionLocker lock;							// prevent the Move task changing data while we look at it

	const DDA * const savedDdaRingAddPointer = addPointer;

	IrqDisable();
	DDA *dda = getPointer;
	if (dda != savedDdaRingAddPointer)
	{
		bool pauseOkHere = dda->CanPauseAfter();
		dda = dda->GetNext();

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
	}

	IrqEnable();

	// We may be going to skip some moves. Get the end coordinate of the previous move.
	DDA * const prevDda = addPointer->GetPrevious();
	ms.UpdateOwnedDriveLastEndpoints(prevDda->DriveCoordinates());
	prevDda->GetEndCoordinates(startCoordinates);
	RestorePoint& rp = ms.GetPauseRestorePoint();
	memcpyf(rp.moveCoords, startCoordinates, ARRAY_SIZE(rp.moveCoords));
	reprap.GetMove().InverseAxisAndBedTransform(rp.moveCoords, prevDda->GetTool());

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = dda->GetLaserPwmOrIoBits();
#endif

	if (addPointer == savedDdaRingAddPointer)
	{
		return false;										// we can't skip any moves
	}

	dda = addPointer;
	rp.proportionDone = dda->GetProportionDone();			// get the proportion of the current multi-segment move that has been completed
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
bool DDARing::LowPowerOrStallPause(MovementState& ms) noexcept
{
	TaskCriticalSectionLocker lock;						// prevent the Move task changing data while we look at it

	const DDA * const savedDdaRingAddPointer = addPointer;
	bool abortedMove = false;

	IrqDisable();
	DDA *_ecv_null dda = GetCurrentDDA();
	if (dda != nullptr && dda->GetFilePosition() != noFilePosition)
	{
		// We are executing a move that has a file address, so we can interrupt it
		reprap.GetMove().CancelStepping();
		abortedMove = true;
#if SUPPORT_LASER
		if (reprap.GetGCodes().GetMachineType() == MachineType::laser)
		{
			reprap.GetPlatform().SetLaserPwm(0);
		}
#endif
		--scheduledMoves;								// this move is no longer scheduled
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
	RestorePoint& rp = ms.GetPauseRestorePoint();
	rp.feedRate = dda->GetRequestedSpeedMmPerClock();
	rp.virtualExtruderPosition = dda->GetVirtualExtruderPosition();
	rp.filePos = dda->GetFilePosition();
	rp.proportionDone = dda->GetProportionDone();		// store how much of the complete multi-segment move's extrusion has been done
	rp.initialUserC0 = dda->GetInitialUserC0();
	rp.initialUserC1 = dda->GetInitialUserC1();

#if SUPPORT_LASER || SUPPORT_IOBITS
	rp.laserPwmOrIoBits = dda->GetLaserPwmOrIoBits();
#endif

	addPointer = (abortedMove) ? dda->GetNext() : _ecv_not_null(dda);

	// Get the end coordinates of the last move that was or will be completed, or the coordinates of the current move when we aborted it.
	DDA * const prevDda = addPointer->GetPrevious();
	ms.UpdateOwnedDriveLastEndpoints(prevDda->DriveCoordinates());
	prevDda->GetEndCoordinates(startCoordinates);
	memcpyf(rp.moveCoords, startCoordinates, ARRAY_SIZE(rp.moveCoords));
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
	reprap.GetPlatform().MessageF(mtype,
									"=== DDARing %u ===\nScheduled moves %" PRIu32 ", completed %" PRIu32 ", LaErrors %u, Underruns [%u, %u, %u]\n",
									ringNumber, scheduledMoves, completedMoves, numLookaheadErrors, numLookaheadUnderruns, numPrepareUnderruns, numNoMoveUnderruns
								 );
	numLookaheadUnderruns = numPrepareUnderruns = numNoMoveUnderruns = numLookaheadErrors = 0;
	reprap.GetGCodes().GetMovementState(ringNumber).Diagnostics(mtype);
}

#if SUPPORT_LASER

// Manage the laser power. Return the number of ticks until we should be called again, or portMAX_DELAY to be called at the start of the next move.
uint32_t DDARing::ManageLaserPower() noexcept
{
	SetBasePriority(NvicPriorityStep);							// lock out step interrupts
	const DDA *_ecv_null const cdda = GetCurrentDDA();					// capture volatile variable
	if (cdda != nullptr)
	{
		const uint32_t ret = cdda->ManageLaserPower();
		SetBasePriority(0);
		return ret;
	}

	// If we get here then there is no active laser move
	SetBasePriority(0);
	reprap.GetPlatform().SetLaserPwm(0);						// turn off the laser
	return portMAX_DELAY;
}

#endif

// Manage the IOBITS (G1 P parameter) and extruder heater feedforward. Called by the Laser task. Return the number of ticks until we should be called again, up to portMAX_DELAY.
uint32_t DDARing::ManageIOBitsAndFeedForward() noexcept
{
#if SUPPORT_IOBITS
	PortControl& pc = reprap.GetPortControl();
	bool doneIoBits = !pc.IsConfigured();
#endif
	bool doneFeedForward = false;
	bool setFeedForward = false;
	uint32_t nextWakeupDelay = StepClockRate;

	SetBasePriority(NvicPriorityStep);
	DDA *cdda = getPointer;
	const uint32_t now = StepTimer::GetMovementTimerTicks();
	const Tool *_ecv_null feedForwardTool;
	float feedForwardAverageExtrusionSpeed = 0.0;

	while (cdda->IsCommitted())
	{
		const int32_t timeToMoveStart = (int32_t)(cdda->GetMoveStartTime() - now);				// get the time to the start of the move, negative if the move has started
		const int32_t timeToMoveEnd = timeToMoveStart + (int32_t)cdda->GetClocksNeeded();		// get the time to the move ended, negative if the move has ended
#if SUPPORT_IOBITS
		if (!doneIoBits && timeToMoveStart < (int32_t)pc.GetAdvanceClocks() && timeToMoveEnd > (int32_t)pc.GetAdvanceClocks())
		{
			// This move is current from the perspective of IOBits
			if (!cdda->HaveDoneIoBits())
			{
				pc.UpdatePorts(cdda->GetIoBits());
				cdda->SetDoneIoBits();
			}
			nextWakeupDelay = min<uint32_t>(nextWakeupDelay, (uint32_t)timeToMoveEnd - pc.GetAdvanceClocks());
			doneIoBits = true;
			if (doneFeedForward)
			{
				break;
			}
		}
		if (!doneFeedForward)
#endif
		{
			feedForwardTool = cdda->GetTool();
			if (feedForwardTool != nullptr && timeToMoveStart < (int32_t)feedForwardTool->GetFeedForwardAdvanceClocks() && timeToMoveEnd > (int32_t)feedForwardTool->GetFeedForwardAdvanceClocks())
			{
				// This move is current from the perspective of feedforward
				if (!cdda->HaveDoneFeedForward())
				{
					// Don't set feedforward here because we have set a very high base priority and we may need to send CAN messages. Just record that we need to set it.
					cdda->SetDoneFeedForward();
					feedForwardAverageExtrusionSpeed = cdda->GetAverageExtrusionSpeed();
					setFeedForward = true;
				}
				nextWakeupDelay = min<uint32_t>(nextWakeupDelay, (uint32_t)timeToMoveEnd > feedForwardTool->GetFeedForwardAdvanceClocks());
				doneFeedForward = true;
#if SUPPORT_IOBITS
				if (doneIoBits)
#endif
				{
					break;
				}
			}
		}
		cdda = cdda->GetNext();
	}

#if SUPPORT_IOBITS
	if (!doneIoBits)
	{
		pc.UpdatePorts(0);															// no move active so turn off all IOBITS ports
	}
#endif

	SetBasePriority(0);

	if (setFeedForward)
	{
		if (feedForwardTool != lastFeedForwardTool || fabsf(feedForwardAverageExtrusionSpeed - lastAverageExtrusionSpeed) > lastAverageExtrusionSpeed * 0.05)
		{
			feedForwardTool->ApplyExtrusionFeedForward(feedForwardAverageExtrusionSpeed);
			lastFeedForwardTool = feedForwardTool;
			lastAverageExtrusionSpeed = feedForwardAverageExtrusionSpeed;
		}
	}
	else if (!doneFeedForward && lastFeedForwardTool != nullptr && lastAverageExtrusionSpeed != 0.0)
	{
		lastFeedForwardTool->StopExtrusionFeedForward();							// no move with a tool active so cancel the last feedforward we commanded
		lastFeedForwardTool = nullptr;
		lastAverageExtrusionSpeed = 0.0;
	}

	return (nextWakeupDelay + StepClockRate/1000 - 1)/(StepClockRate/1000);			// convert step clocks to milliseconds, rounding up
}

#if SUPPORT_REMOTE_COMMANDS

// Add a move from the ATE to the movement queue
void DDARing::AddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept
{
	if (addPointer->GetState() == DDA::empty)
	{
		if (addPointer->InitFromRemote(*this, msg))
		{
			addPointer = addPointer->GetNext();
			scheduledMoves++;
		}
	}
}

#endif

// End
