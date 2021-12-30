/*
 * CanMotion.cpp
 *
 *  Created on: 11 Aug 2019
 *      Author: David
 */

#include "CanMotion.h"

#if SUPPORT_CAN_EXPANSION

#include <CanMessageBuffer.h>
#include <CanMessageFormats.h>
#include "CanInterface.h"
#include <General/FreelistManager.h>

namespace CanMotion
{
	enum class DriverStopState : uint8_t { inactive = 0, active, stopRequested, stopSent };

	// Class to record drivers active and requests to stop them
	class DriversStopList
	{
	public:
		DECLARE_FREELIST_NEW_DELETE(DriversStopList)

		DriversStopList(DriversStopList *p_next, CanAddress p_ba) noexcept : next(p_next), boardAddress(p_ba), sentRevertRequest(0) { }

		DriversStopList *next;
		CanAddress boardAddress;
		uint8_t numDrivers;
		bool sentRevertRequest;
		volatile DriverStopState stopStates[MaxLinearDriversPerCanSlave];
		volatile int32_t stopSteps[MaxLinearDriversPerCanSlave];
	};

	static CanMessageBuffer urgentMessageBuffer(nullptr);
	static CanMessageBuffer *movementBufferList = nullptr;
	static DriversStopList *volatile stopList = nullptr;
	static uint32_t currentMoveClocks;
	static volatile uint32_t hiccupToInsert = 0;
	static volatile bool revertAll = false;
	static volatile bool revertedAll = false;
	static volatile uint32_t whenRevertedAll;
	static Mutex stopListMutex;
	static uint8_t nextSeq[CanId::MaxCanAddress + 1] = { 0 };

	static CanMessageBuffer *GetBuffer(const PrepParams& params, DriverId canDriver) noexcept;
	static void InternalStopDriverWhenProvisional(DriverId driver) noexcept;
	static bool InternalStopDriverWhenMoving(DriverId driver, int32_t steps) noexcept;
}

void CanMotion::Init() noexcept
{
	movementBufferList = nullptr;
	stopListMutex.Create("stopList");
}

// This is called by DDA::Prepare at the start of preparing a movement
void CanMotion::StartMovement() noexcept
{
	// There shouldn't be any movement buffers in the list, but free any that there may be
	for (;;)
	{
		CanMessageBuffer *p = movementBufferList;
		if (p == nullptr)
		{
			break;
		}
		movementBufferList = p->next;
		CanMessageBuffer::Free(p);
	}

	// Free up any stop list items left over from the previous move
	MutexLocker lock(stopListMutex);

	revertAll = revertedAll = false;
	for (;;)
	{
		DriversStopList *p = stopList;
		if (p == nullptr)
		{
			break;
		}
		stopList = p->next;
		delete p;
	}
}

CanMessageBuffer *CanMotion::GetBuffer(const PrepParams& params, DriverId canDriver) noexcept
{
	if (canDriver.localDriver >= MaxLinearDriversPerCanSlave)
	{
		return nullptr;
	}

	CanMessageBuffer* buf = movementBufferList;
	while (buf != nullptr && buf->id.Dst() != canDriver.boardAddress)
	{
		buf = buf->next;
	}

	if (buf == nullptr)
	{
		// Allocate a new movement buffer
		buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			reprap.GetPlatform().Message(ErrorMessage, "Out of CAN buffers\n");
			return nullptr;		//TODO error handling
		}

		buf->next = movementBufferList;
		movementBufferList = buf;

#if USE_REMOTE_INPUT_SHAPING
		auto move = buf->SetupRequestMessage<CanMessageMovementLinearShaped>(0, CanId::MasterAddress, canDriver.boardAddress);
#else
		auto move = buf->SetupRequestMessage<CanMessageMovementLinear>(0, CanInterface::GetCurrentMasterAddress(), canDriver.boardAddress);
#endif

		// Common parameters
		if (buf->next == nullptr)
		{
			// This is the first CAN-connected board for this movement
			move->accelerationClocks = (uint32_t)params.unshaped.accelClocks;
			move->steadyClocks = (uint32_t)params.unshaped.steadyClocks;
			move->decelClocks = (uint32_t)params.unshaped.decelClocks;
			currentMoveClocks = move->accelerationClocks + move->steadyClocks + move->decelClocks;
		}
		else
		{
			// Save some maths by using the values from the previous buffer
#if USE_REMOTE_INPUT_SHAPING
			move->accelerationClocks = buf->next->msg.moveLinearShaped.accelerationClocks;
			move->steadyClocks = buf->next->msg.moveLinearShaped.steadyClocks;
			move->decelClocks = buf->next->msg.moveLinearShaped.decelClocks;
#else
			move->accelerationClocks = buf->next->msg.moveLinear.accelerationClocks;
			move->steadyClocks = buf->next->msg.moveLinear.steadyClocks;
			move->decelClocks = buf->next->msg.moveLinear.decelClocks;
#endif
		}
		move->initialSpeedFraction = params.initialSpeedFraction;
		move->finalSpeedFraction = params.finalSpeedFraction;
#if USE_REMOTE_INPUT_SHAPING
		move->numDriversMinusOne = canDriver.localDriver;
		move->moveTypes = 0;
		move->shaperAccelPhasesMinusOne = params.shapingPlan.accelSegments - 1;
		move->shaperDecelPhasesMinusOne = params.shapingPlan.decelSegments - 1;
#else
		move->pressureAdvanceDrives = 0;
		move->numDrivers = canDriver.localDriver + 1;
		move->zero = 0;
#endif

		// Clear out the per-drive fields. Can't use a range-based FOR loop on a packed struct.
		for (size_t drive = 0; drive < ARRAY_SIZE(move->perDrive); ++drive)
		{
			move->perDrive[drive].Init();
		}
	}
#if USE_REMOTE_INPUT_SHAPING
	else if (canDriver.localDriver > buf->msg.moveLinearShaped.numDriversMinusOne)
	{
		buf->msg.moveLinearShaped.numDriversMinusOne = canDriver.localDriver;
	}
#else
	else if (canDriver.localDriver >= buf->msg.moveLinear.numDrivers)
	{
		buf->msg.moveLinear.numDrivers = canDriver.localDriver + 1;
	}
#endif
	return buf;
}

// This is called by DDA::Prepare for each active CAN DM in the move
#if USE_REMOTE_INPUT_SHAPING
void CanMotion::AddMovement(const PrepParams& params, DriverId canDriver, int32_t steps) noexcept
#else
void CanMotion::AddMovement(const PrepParams& params, DriverId canDriver, int32_t steps, bool usePressureAdvance) noexcept
#endif
{
	CanMessageBuffer * const buf = GetBuffer(params, canDriver);
	if (buf != nullptr)
	{
#if USE_REMOTE_INPUT_SHAPING
		buf->msg.moveLinearShaped.perDrive[canDriver.localDriver].iSteps = steps;
#else
		buf->msg.moveLinear.perDrive[canDriver.localDriver].steps = steps;
		if (usePressureAdvance)
		{
			buf->msg.moveLinear.pressureAdvanceDrives |= 1u << canDriver.localDriver;
		}
#endif
	}
}

#if USE_REMOTE_INPUT_SHAPING

void CanMotion::AddExtruderMovement(const PrepParams& params, DriverId canDriver, float extrusion, bool usePressureAdvance) noexcept
{
	CanMessageBuffer * const buf = GetBuffer(params, canDriver);
	if (buf != nullptr)
	{
		buf->msg.moveLinearShaped.perDrive[canDriver.localDriver].fDist = extrusion;
		const auto mt = (usePressureAdvance) ? CanMessageMovementLinearShaped::MoveType::extruderWithPa : CanMessageMovementLinearShaped::MoveType::extruderNoPa;
		buf->msg.moveLinearShaped.ChangeMoveTypeFromDefault(canDriver.localDriver, mt);
	}
}

#endif

// This is called by DDA::Prepare when all DMs for CAN drives have been processed. Return the calculated move time in steps, or 0 if there are no CAN moves
uint32_t CanMotion::FinishMovement(uint32_t moveStartTime, bool simulating, bool checkingEndstops) noexcept
{
	CanMessageBuffer *buf = movementBufferList;
	if (buf == nullptr)
	{
		return 0;
	}

	MutexLocker lock(stopListMutex);

	do
	{
		CanMessageBuffer * const nextBuffer = buf->next;				// must get this before sending the buffer, because sending the buffer releases it
		if (simulating)
		{
			CanMessageBuffer::Free(buf);
		}
		else
		{
#if USE_REMOTE_INPUT_SHAPING
			CanMessageMovementLinear& msg = buf->msg.moveLinearShaped;
#else
			CanMessageMovementLinear& msg = buf->msg.moveLinear;
#endif
			msg.whenToExecute = moveStartTime;
			uint8_t& seq = nextSeq[buf->id.Dst()];
			msg.seq = seq;
			seq = (seq + 1) & 0x7F;
			buf->dataLength = msg.GetActualDataLength();
			if (checkingEndstops)
			{
				// Set up the stop list
				DriversStopList * const sl = new DriversStopList(stopList, buf->id.Dst());
				const size_t nd = msg.numDrivers;
				sl->numDrivers = (uint8_t)nd;
				for (size_t i = 0; i < nd; ++i)
				{
					sl->stopStates[i] = (msg.perDrive[i].steps != 0) ? DriverStopState::active : DriverStopState::inactive;
				}
				stopList = sl;
			}
			CanInterface::SendMotion(buf);								// queues the buffer for sending and frees it when done
		}
		buf = nextBuffer;
	} while (buf != nullptr);

	movementBufferList = nullptr;
	return currentMoveClocks;
}

bool CanMotion::CanPrepareMove() noexcept
{
	return CanMessageBuffer::GetFreeBuffers() >= MaxCanBoards;
}

// This is called by the CanSender task to check if we have any urgent messages to send
// The only urgent messages we may have currently are messages to stop drivers, or to tell them that all drivers have now been stopped and they need to revert to the requested stop position.
CanMessageBuffer *CanMotion::GetUrgentMessage() noexcept
{
	if (!revertedAll)
	{
		MutexLocker lock(stopListMutex);					// make sure the list isn't being changed while we traverse it

		// We have to be careful of race conditions here. The stop list links won't change while we are scanning it because we hold the mutex,
		// but ISR may change the stop states to StopRequested up until the time at which it changes revertAll from false to true.
		const bool revertingAll = revertAll;
		for (DriversStopList *sl = stopList; sl != nullptr; sl = sl->next)
		{
			if (!sl->sentRevertRequest)						// if we've already reverted the drivers on this board, no more to do
			{
				// Set up a reversion message in case we are going to revert the drivers on this board
				auto revertMsg = urgentMessageBuffer.SetupRequestMessage<CanMessageRevertPosition>(0, CanInterface::GetCanAddress(), sl->boardAddress);
				uint16_t driversToStop = 0, driversToRevert = 0;
				size_t numDriversReverted = 0;
				for (size_t driver = 0; driver < sl->numDrivers; ++driver)
				{
					const DriverStopState ss = sl->stopStates[driver];
					if (ss == DriverStopState::stopRequested)
					{
						driversToStop |= 1u << driver;
						sl->stopStates[driver] = DriverStopState::stopSent;
					}
					else if (revertingAll && ss == DriverStopState::stopSent)
					{
						driversToRevert |= 1u << driver;
						revertMsg->finalStepCounts[numDriversReverted++] = sl->stopSteps[driver];
					}
				}

				if (driversToStop != 0)
				{
					auto stopMsg = urgentMessageBuffer.SetupRequestMessage<CanMessageStopMovement>(0, CanInterface::GetCanAddress(), sl->boardAddress);
					stopMsg->whichDrives = driversToStop;
					return &urgentMessageBuffer;
				}

				if (driversToRevert != 0)
				{
					sl->sentRevertRequest = true;
					revertMsg->whichDrives = driversToRevert;
					revertMsg->clocksAllowed = (StepClockRate * BasicDriverPositionRevertMillis)/1000;
					urgentMessageBuffer.dataLength = revertMsg->GetActualDataLength(numDriversReverted);
					return &urgentMessageBuffer;
				}
			}
		}

		// We found nothing to send
		if (revertingAll)
		{
			// All drivers have been stopped and reverted where requested
			whenRevertedAll = millis();
			revertedAll = true;
		}
	}
	return nullptr;
}

// The next 4 functions may be called from the step ISR, so they can't send CAN messages directly

void CanMotion::InsertHiccup(uint32_t numClocks) noexcept
{
	hiccupToInsert += numClocks;
	CanInterface::WakeAsyncSenderFromIsr();
}

void CanMotion::InternalStopDriverWhenProvisional(DriverId driver) noexcept
{
	// Search for the correct movement buffer
	CanMessageBuffer* buf = movementBufferList;
	while (buf != nullptr && buf->id.Dst() != driver.boardAddress)
	{
		buf = buf->next;
	}

	if (buf != nullptr)
	{
#if USE_REMOTE_INPUT_SHAPING
		buf->msg.moveLinearShaped.perDrive[driver.localDriver].steps = 0;
#else
		buf->msg.moveLinear.perDrive[driver.localDriver].steps = 0;
#endif
	}
}

bool CanMotion::InternalStopDriverWhenMoving(DriverId driver, int32_t steps) noexcept
{
	DriversStopList *sl = stopList;
	while (sl != nullptr)
	{
		if (sl->boardAddress == driver.boardAddress)
		{
			if (sl->stopStates[driver.localDriver] == DriverStopState::active)			// if active and stop not yet requested
			{
				sl->stopSteps[driver.localDriver] = steps;								// must assign this one first
				sl->stopStates[driver.localDriver] = DriverStopState::stopRequested;
				return true;
			}
			break;																		// we found the right board, no point in searching further
		}
		sl = sl->next;
	}
	return false;
}

// This is called from the step ISR with isBeingPrepared false, or from the Move task with isBeingPrepared true
void CanMotion::StopDriver(const DDA& dda, size_t axis, DriverId driver) noexcept
{
	if (dda.GetState() == DDA::DDAState::provisional)
	{
		InternalStopDriverWhenProvisional(driver);
	}
	else
	{
		const DriveMovement * const dm = dda.FindDM(axis);
		if (dm != nullptr)
		{
			if (InternalStopDriverWhenMoving(driver, dm->GetNetStepsTaken()))
			{
				CanInterface::WakeAsyncSenderFromIsr();
			}
		}
	}
}

// This is called from the step ISR with isBeingPrepared false, or from the Move task with isBeingPrepared true
void CanMotion::StopAxis(const DDA& dda, size_t axis) noexcept
{
	const Platform& p = reprap.GetPlatform();
	if (axis < reprap.GetGCodes().GetTotalAxes())
	{
		const AxisDriversConfig& cfg = p.GetAxisDriversConfig(axis);
		if (dda.GetState() == DDA::DDAState::provisional)
		{
			for (size_t i = 0; i < cfg.numDrivers; ++i)
			{
				const DriverId driver = cfg.driverNumbers[i];
				if (driver.IsRemote())
				{
					InternalStopDriverWhenProvisional(driver);
				}
			}
		}
		else
		{
			const DriveMovement * const dm = dda.FindDM(axis);
			if (dm != nullptr)
			{
				bool somethingStopped = false;
				const uint32_t steps = dm->GetNetStepsTaken();
				for (size_t i = 0; i < cfg.numDrivers; ++i)
				{
					const DriverId driver = cfg.driverNumbers[i];
					if (driver.IsRemote() && InternalStopDriverWhenMoving(driver, steps))
					{
						somethingStopped = true;
					}
				}
				if (somethingStopped)
				{
					CanInterface::WakeAsyncSenderFromIsr();
				}
			}
		}
	}
	else
	{
		const DriverId driver = p.GetExtruderDriver(LogicalDriveToExtruder(axis));
		StopDriver(dda, axis, driver);
	}
}

// This is called from the step ISR with isBeingPrepared false, or from the Move task with isBeingPrepared true
void CanMotion::StopAll(const DDA& dda) noexcept
{
	if (dda.GetState() == DDA::DDAState::provisional)
	{
		// We still send the messages so that the drives get enabled, but we set the steps to zero
		for (CanMessageBuffer *buf = movementBufferList; buf != nullptr; buf = buf->next)
		{
#if USE_REMOTE_INPUT_SHAPING
			buf->msg.moveLinearShaped.accelerationClocks = buf->msg.moveLinearShaped.decelClocks = buf->msg.moveLinearShaped.steadyClocks = 0;
			for (size_t drive = 0; drive < ARRAY_SIZE(buf->msg.moveLinearShaped.perDrive); ++drive)
			{
				buf->msg.moveLinearShaped.perDrive[drive].steps = 0;
			}
#else
			buf->msg.moveLinear.accelerationClocks = buf->msg.moveLinear.decelClocks = buf->msg.moveLinear.steadyClocks = 0;
			for (size_t drive = 0; drive < ARRAY_SIZE(buf->msg.moveLinear.perDrive); ++drive)
			{
				buf->msg.moveLinear.perDrive[drive].steps = 0;
			}
#endif
		}
	}
	else if (stopList != nullptr)
	{
		// Loop through the axes that are actually moving
		const GCodes& gc = reprap.GetGCodes();
		const size_t totalAxes = gc.GetTotalAxes();
		const Platform& p = reprap.GetPlatform();
		for (size_t axis = 0; axis < totalAxes; ++axis)
		{
			const DriveMovement* const dm = dda.FindDM(axis);
			if (dm != nullptr)
			{
				const uint32_t steps = dm->GetNetStepsTaken();
				const AxisDriversConfig& cfg = p.GetAxisDriversConfig(axis);
				for (size_t i = 0; i < cfg.numDrivers; ++i)
				{
					const DriverId driver = cfg.driverNumbers[i];
					if (driver.IsRemote())
					{
						(void)InternalStopDriverWhenMoving(driver, steps);
					}
				}
			}
		}
		const size_t numExtruders = gc.GetNumExtruders();
		for (size_t extruder = 0; extruder < numExtruders; ++extruder)
		{
			const DriverId driver = p.GetExtruderDriver(extruder);
			if (driver.IsRemote())
			{
				const DriveMovement* const dm = dda.FindDM(extruder);
				if (dm != nullptr)
				{
					(void)InternalStopDriverWhenMoving(driver, dm->GetNetStepsTaken());
				}
			}
		}

		revertAll = true;
		CanInterface::WakeAsyncSenderFromIsr();
	}
}

// This is called by the step ISR when a movement that uses endstops or a probe has completed.
// We must make sure that all boards have been told to adjust their stepper motor positions to the points at which we wanted them to stop.
void CanMotion::FinishMoveUsingEndstops() noexcept
{
	if (!revertAll)
	{
		revertAll = true;
		CanInterface::WakeAsyncSenderFromIsr();
	}
}

// This is called by the main task when it is waiting for the move to complete, after checking that the DDA ring is empty and there is no current move
bool CanMotion::FinishedReverting() noexcept
{
	return !revertAll || (revertedAll && millis() - whenRevertedAll >= TotalDriverPositionRevertMillis);
}

#endif

// End
