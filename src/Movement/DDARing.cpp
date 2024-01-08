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

	extrudersPrinting = false;
	simulationTime = 0.0;
}

void DDARing::Exit() noexcept
{
	// Clear the DDA ring so that we don't report any moves as pending
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
			int64_t memoryNeeded = (int64_t)(numDdasWanted - numDdasInRing) * (sizeof(DDA) + 8);
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
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or TaskBase::TimeoutUnlimited if there are no unprepared moves left.
uint32_t DDARing::Spin(SimulationMode simulationMode, bool waitingForSpace, bool shouldStartMove) noexcept
{
	DDA *cdda = getPointer;											// capture volatile variable

	// If we are simulating, simulate completion of the current move.
	// Do this here rather than at the end, so that when simulating, currentDda is non-null for most of the time and IsExtruding() returns the correct value
	if (simulationMode != SimulationMode::off)
	{
		// Simulate one move
		if (cdda->GetState() == DDA::committed)
		{
			simulationTime += (float)cdda->GetClocksNeeded() * (1.0/StepClockRate);
			cdda->Complete();
			getPointer = cdda = cdda->GetNext();
		}
	}
	else
	{
		// See if we can retire any completed moves
		while (cdda->GetState() == DDA::completed || cdda->HasExpired())
		{
			if (cdda->Free())
			{
				++numLookaheadUnderruns;
			}
			getPointer = cdda = cdda->GetNext();
		}
	}

	// If we are already moving, see whether we need to prepare any more moves
	if (cdda->GetState() == DDA::committed)
	{
		const DDA* const currentMove = cdda;						// save for later

		// Count how many prepared or executing moves we have and how long they will take
		uint32_t preparedTime = 0;
		unsigned int preparedCount = 0;
		while (cdda->GetState() == DDA::committed)
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
	if (   shouldStartMove											// if the Move code told us that we should start a move in any case...
		|| waitingForSpace											// ...or the Move code told us it was waiting for space in the ring...
		|| waitingForRingToEmpty									// ...or GCodes is waiting for all moves to finish...
		|| cdda->IsCheckingEndstops()								// ...or checking endstops, so we can't schedule the following move
#if SUPPORT_REMOTE_COMMANDS
		|| cdda->GetState() == DDA::committed						// ...or the move has already been committed (it's probably a remote move)
#endif
	   )
	{
		uint32_t ret = PrepareMoves(cdda, 0, 0, simulationMode);

		if (cdda->GetState() == DDA::completed)
		{
			// We prepared the move but found there was nothing to do because endstops are already triggered
			getPointer = cdda = cdda->GetNext();
			completedMoves++;
		}
		else if (cdda->GetState() == DDA::committed)
		{
			if (simulationMode != SimulationMode::off)
			{
				return 0;											// we don't want any delay because we want Spin() to be called again soon to complete this move
			}

			if (waitingForSpace)
			{
				// The Move task told us it is waiting for space in the ring, so wake it up soon after we expect the move to finish
				const uint32_t moveTime = getPointer->GetClocksNeeded()/StepClockRate + 1;	// the move time plus 1ms
				if (moveTime < ret)
				{
					ret = moveTime;
				}
			}
		}
		return ret;
	}

	return (cdda->GetState() == DDA::provisional)
			? MoveStartPollInterval									// there are moves in the queue but it is not time to prepare them yet
				: TaskBase::TimeoutUnlimited;						// the queue is empty, nothing to do until new moves arrive
}

// Prepare some moves. moveTimeLeft is the total length remaining of moves that are already executing or prepared.
// Return the maximum time in milliseconds that should elapse before we prepare further unprepared moves that are already in the ring, or TaskBase::TimeoutUnlimited if there are no unprepared moves left.
uint32_t DDARing::PrepareMoves(DDA *firstUnpreparedMove, uint32_t moveTimeLeft, unsigned int alreadyPrepared, SimulationMode simulationMode) noexcept
{
	// If the number of prepared moves will execute in less than the minimum time, prepare another move.
	// Try to avoid preparing deceleration-only moves too early
	while (	  firstUnpreparedMove->GetState() == DDA::provisional
		   && moveTimeLeft < (int32_t)MoveTiming::UsualMinimumPreparedTime	// prepare moves one tenth of a second ahead of when they will be needed
		   && alreadyPrepared * 2 < numDdasInRing						// but don't prepare more than half the ring, to handle accelerate/decelerate moves in small segments
		   && (firstUnpreparedMove->IsGoodToPrepare() || moveTimeLeft < MoveTiming::AbsoluteMinimumPreparedTime)
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

		const int32_t clocksTillWakeup = moveTimeLeft - MoveTiming::UsualMinimumPreparedTime;						// calculate how long before we run out of prepared moves, less the usual advance prepare time
		return (clocksTillWakeup <= 0) ? 2 : min<uint32_t>((uint32_t)clocksTillWakeup/(StepClockRate/1000), 2);		// wake up at that time, but delay for at least 2 ticks
	}

	// There are no moves waiting to be prepared
	return TaskBase::TimeoutUnlimited;
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
	return addPointer->AdvanceBabyStepping(*this, axis, amount);
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
	addPointer->GetPrevious()->SetPositions(move);
}

// Get the DDA that should currently be executing, or nullptr if no move from this ring should be executing
const DDA *DDARing::GetCurrentDDA() const noexcept
{
	TaskCriticalSectionLocker lock;
	const DDA *cdda = checkPointer;
	while (cdda->GetState() == DDA::completed)
	{
		cdda = cdda->GetNext();
	}
	const uint32_t now = StepTimer::GetTimerTicks();
	while (cdda->GetState() == DDA::committed)
	{
		const uint32_t timeRunning = cdda->GetMoveStartTime() - now;
		if ((int32_t)timeRunning < 0) { break; }			// move has not started yet
		if (timeRunning < cdda->GetClocksNeeded()) { return cdda; }
		cdda = cdda->GetNext();
	}
	return nullptr;
}

// Get various data for reporting in the OM
float DDARing::GetRequestedSpeedMmPerSec() const noexcept
{
	const DDA* const cdda = GetCurrentDDA();
	return (cdda != nullptr) ? cdda->GetRequestedSpeedMmPerSec() : 0.0;
}

float DDARing::GetTopSpeedMmPerSec() const noexcept
{
	const DDA* const cdda = GetCurrentDDA();
	return (cdda != nullptr) ? cdda->GetTopSpeedMmPerSec() : 0.0;
}

float DDARing::GetAccelerationMmPerSecSquared() const noexcept
{
	const DDA* const cdda = GetCurrentDDA();
	return (cdda != nullptr) ? cdda->GetAccelerationMmPerSecSquared() : 0.0;
}

float DDARing::GetDecelerationMmPerSecSquared() const noexcept
{
	const DDA* const cdda = GetCurrentDDA();
	return (cdda != nullptr) ? cdda->GetDecelerationMmPerSecSquared() : 0.0;
}

float DDARing::GetTotalExtrusionRate() const noexcept
{
	const DDA* const cdda = GetCurrentDDA();
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
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	RestorePoint& rp = ms.pauseRestorePoint;
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

#if 0	//TODO
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
#endif

void DDARing::Diagnostics(MessageType mtype, unsigned int ringNumber) noexcept
{
	reprap.GetPlatform().MessageF(mtype,
									"=== DDARing %u ===\nScheduled moves %" PRIu32 ", completed %" PRIu32 ", hiccups %" PRIu32 ", stepErrors %u, LaErrors %u, Underruns [%u, %u, %u]\n",
									ringNumber, scheduledMoves, completedMoves, numHiccups, stepErrors, numLookaheadErrors, numLookaheadUnderruns, numPrepareUnderruns, numNoMoveUnderruns
								 );
	numHiccups = stepErrors = numLookaheadUnderruns = numPrepareUnderruns = numNoMoveUnderruns = numLookaheadErrors = 0;
}

#if SUPPORT_LASER

// Manage the laser power. Return the number of ticks until we should be called again, or 0 to be called at the start of the next move.
uint32_t DDARing::ManageLaserPower() noexcept
{
	SetBasePriority(NvicPriorityStep);							// lock out step interrupts
	const DDA * const cdda = GetCurrentDDA();					// capture volatile variable
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

#if SUPPORT_IOBITS

// Manage the IOBITS (G1 P parameter)
uint32_t DDARing::ManageIOBits() noexcept
{
	PortControl& pc = reprap.GetPortControl();
	if (pc.IsConfigured())
	{
		SetBasePriority(NvicPriorityStep);
		const DDA * cdda = GetCurrentDDA();
		if (cdda == nullptr)
		{
			// Movement has stopped, so turn all ports off
			SetBasePriority(0);
			pc.UpdatePorts(0);
			return 0;
		}

		// Find the DDA whose IO port bits we should set now
		const uint32_t now = StepTimer::GetTimerTicks() + pc.GetAdvanceClocks();
		uint32_t moveEndTime = cdda->GetMoveStartTime();
		DDA::DDAState st = cdda->GetState();
		do
		{
			moveEndTime += cdda->GetClocksNeeded();
			if ((int32_t)(moveEndTime - now) >= 0)
			{
				SetBasePriority(0);
				pc.UpdatePorts(cdda->GetIoBits());
				return (moveEndTime - now + StepClockRate/1000 - 1)/(StepClockRate/1000);
			}
			cdda = cdda->GetNext();
			st = cdda->GetState();
		} while (st == DDA::committed);

		SetBasePriority(0);
		pc.UpdatePorts(0);
		return 0;
	}
	return 0;
}

#endif

#if SUPPORT_REMOTE_COMMANDS

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

#endif

// End
