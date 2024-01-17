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

	static CanMessageBuffer urgentMessageBuffer;
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
	static void FreeMovementBuffers() noexcept;
}

void CanMotion::Init() noexcept
{
	movementBufferList = nullptr;
	stopListMutex.Create("stopList");
}

void CanMotion::FreeMovementBuffers() noexcept
{
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
}

// This is called by DDA::Prepare at the start of preparing a movement
void CanMotion::StartMovement() noexcept
{
	FreeMovementBuffers();					// there shouldn't be any movement buffers in the list, but free any that there may be

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
		auto move = buf->SetupRequestMessage<CanMessageMovementLinearShaped>(0, CanInterface::GetCurrentMasterAddress(), canDriver.boardAddress);

		// Common parameters
		if (buf->next == nullptr)
		{
			// This is the first CAN-connected board for this movement
			move->accelerationClocks = (uint32_t)params.accelClocks;
			move->steadyClocks = (uint32_t)params.steadyClocks;
			move->decelClocks = (uint32_t)params.decelClocks;
			currentMoveClocks = move->accelerationClocks + move->steadyClocks + move->decelClocks;
		}
		else
		{
			// Save some maths by using the values from the previous buffer
			move->accelerationClocks = buf->next->msg.moveLinear.accelerationClocks;
			move->steadyClocks = buf->next->msg.moveLinear.steadyClocks;
			move->decelClocks = buf->next->msg.moveLinear.decelClocks;
		}
		move->acceleration = params.acceleration/params.totalDistance;			// scale the acceleration to correspond to unit distance
		move->deceleration = params.deceleration/params.totalDistance;			// scale the deceleration to correspond to unit distance
		move->extruderDrives = 0;
		move->numDrivers = canDriver.localDriver + 1;
		move->zero = 0;
		move->shapingPlan = params.shapingPlan.condensedPlan;

		// Clear out the per-drive fields. Can't use a range-based FOR loop on a packed struct.
		for (size_t drive = 0; drive < ARRAY_SIZE(move->perDrive); ++drive)
		{
			move->perDrive[drive].Init();
		}
	}
	else if (canDriver.localDriver >= buf->msg.moveLinear.numDrivers)
	{
		buf->msg.moveLinear.numDrivers = canDriver.localDriver + 1;
	}
	return buf;
}

// This is called by DDA::Prepare for each active CAN DM in the move
void CanMotion::AddAxisMovement(const PrepParams& params, DriverId canDriver, int32_t steps) noexcept
{
	CanMessageBuffer * const buf = GetBuffer(params, canDriver);
	if (buf != nullptr)
	{
		buf->msg.moveLinearShaped.perDrive[canDriver.localDriver].steps = steps;
	}
}

void CanMotion::AddExtruderMovement(const PrepParams& params, DriverId canDriver, float extrusion, bool usePressureAdvance) noexcept
{
	CanMessageBuffer * const buf = GetBuffer(params, canDriver);
	if (buf != nullptr)
	{
		buf->msg.moveLinearShaped.perDrive[canDriver.localDriver].extrusion = extrusion;
		buf->msg.moveLinearShaped.extruderDrives |= 1u << canDriver.localDriver;
		buf->msg.moveLinearShaped.usePressureAdvance = usePressureAdvance;
	}
}

// This is called by DDA::Prepare when all DMs for CAN drives have been processed. Return the calculated move time in steps, or 0 if there are no CAN moves
uint32_t CanMotion::FinishMovement(const DDA& dda, uint32_t moveStartTime, bool simulating) noexcept
{
	uint32_t clocks = 0;
	if (simulating || dda.GetState() == DDA::completed)
	{
		FreeMovementBuffers();											// it turned out that there was nothing to move
	}
	else
	{
		CanMessageBuffer *buf = movementBufferList;
		if (buf != nullptr)
		{
			MutexLocker lock((dda.IsCheckingEndstops()) ? &stopListMutex : nullptr);
			do
			{
				CanMessageBuffer * const nextBuffer = buf->next;		// must get this before sending the buffer, because sending the buffer releases it
				CanMessageMovementLinearShaped& msg = buf->msg.moveLinearShaped;
				if (msg.HasMotion())
				{
					msg.whenToExecute = moveStartTime;
					uint8_t& seq = nextSeq[buf->id.Dst()];
					msg.seq = seq;
					seq = (seq + 1) & 0x7F;
					buf->dataLength = msg.GetActualDataLength();
					if (dda.IsCheckingEndstops())
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
					clocks = currentMoveClocks;
				}
				else
				{
					CanMessageBuffer::Free(buf);
				}
				buf = nextBuffer;
			} while (buf != nullptr);

			movementBufferList = nullptr;
		}
	}
	return clocks;
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
					//debugPrintf("Stopping drivers %u on board %u\n", driversToStop, sl->boardAddress);
					return &urgentMessageBuffer;
				}

				if (driversToRevert != 0)
				{
					sl->sentRevertRequest = true;
					revertMsg->whichDrives = driversToRevert;
					revertMsg->clocksAllowed = (StepClockRate * BasicDriverPositionRevertMillis)/1000;
					urgentMessageBuffer.dataLength = revertMsg->GetActualDataLength(numDriversReverted);
					//debugPrintf("Reverting drivers %u by %" PRIi32 " on board %u\n", driversToRevert,revertMsg->finalStepCounts[0], sl->boardAddress);
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
	CanInterface::WakeAsyncSender();
}

// Flag a CAN-connected driver as not moving when we haven't sent the movement message yet
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
		buf->msg.moveLinear.perDrive[driver.localDriver].steps = 0;
	}
}

// Tell a CAN-connected driver to stop moving after we have sent the movement message
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

// This is called from the step ISR with DDA state executing, or from the Move task with DDA state provisional
// Returns true if the caller needs to wake the sync sender task (we don't do that here because it causes problems in some contexts)
bool CanMotion::StopDriver(const DDA& dda, size_t axis, DriverId driver) noexcept
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
			return InternalStopDriverWhenMoving(driver, dm->GetNetStepsTaken());
		}
	}
	return false;
}

// This is called from the step ISR with DDA state executing, or from the Move task with DDA state provisional
// Returns true if the caller needs to wake the sync sender task (we don't do that here because it causes problems in some contexts)
bool CanMotion::StopAxis(const DDA& dda, size_t axis) noexcept
{
	const Platform& p = reprap.GetPlatform();
	if (axis < reprap.GetGCodes().GetTotalAxes())
	{
		bool wakeAsyncSender = false;
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
				const uint32_t steps = dm->GetNetStepsTaken();
				for (size_t i = 0; i < cfg.numDrivers; ++i)
				{
					const DriverId driver = cfg.driverNumbers[i];
					if (driver.IsRemote() && InternalStopDriverWhenMoving(driver, steps))
					{
						wakeAsyncSender = true;
					}
				}
			}
		}
		return wakeAsyncSender;
	}

	const DriverId exDriver = p.GetExtruderDriver(LogicalDriveToExtruder(axis));
	return StopDriver(dda, axis, exDriver);
}

// This is called from the step ISR with DDA state executing, or from the CAN receiver task with state executing, or from the Move task with DDA state provisional
// Returns true if the caller needs to wake the sync sender task (we don't do that here because it causes problems in some contexts)
bool CanMotion::StopAll(const DDA& dda) noexcept
{
	if (dda.GetState() == DDA::DDAState::provisional)
	{
		// We still send the messages so that the drives get enabled, but we set the steps to zero
		for (CanMessageBuffer *buf = movementBufferList; buf != nullptr; buf = buf->next)
		{
			buf->msg.moveLinear.accelerationClocks = buf->msg.moveLinear.decelClocks = buf->msg.moveLinear.steadyClocks = 0;
			for (size_t drive = 0; drive < ARRAY_SIZE(buf->msg.moveLinear.perDrive); ++drive)
			{
				buf->msg.moveLinear.perDrive[drive].steps = 0;
			}
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
		return true;
	}
	return false;
}

// This is called by the step ISR when a movement that uses endstops or a probe has completed.
// We must make sure that all boards have been told to adjust their stepper motor positions to the points at which we wanted them to stop.
void CanMotion::FinishMoveUsingEndstops() noexcept
{
	if (!revertAll)
	{
		revertAll = true;
		CanInterface::WakeAsyncSender();
	}
}

// This is called by the main task when it is waiting for the move to complete, after checking that the DDA ring is empty and there is no current move
bool CanMotion::FinishedReverting() noexcept
{
	return !revertAll || (revertedAll && millis() - whenRevertedAll >= TotalDriverPositionRevertMillis);
}

#endif

// End
