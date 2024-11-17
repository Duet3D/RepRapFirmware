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
#include <Platform/Platform.h>
#include <GCodes/GCodes.h>
#include <Movement/Move.h>
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
	static volatile bool revertAll = false;
	static volatile bool revertedAll = false;
	static volatile uint32_t whenRevertedAll;
	static Mutex stopListMutex;
	static uint8_t nextSeq[CanId::MaxCanAddress + 1] = { 0 };

	static CanMessageBuffer *GetBuffer(const PrepParams& params, DriverId canDriver) noexcept;
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
		auto move = buf->SetupRequestMessageNoRid<CanMessageMovementLinearShaped>(CanInterface::GetCurrentMasterAddress(), canDriver.boardAddress);

		// Common parameters
		if (buf->next == nullptr)
		{
			// This is the first CAN-connected board for this movement
			move->accelerationClocks = params.accelClocks;
			move->steadyClocks = params.steadyClocks;
			move->decelClocks = params.decelClocks;
			currentMoveClocks = params.TotalClocks();
		}
		else
		{
			// Save some maths by using the values from the previous buffer
			move->accelerationClocks = buf->next->msg.moveLinearShaped.accelerationClocks;
			move->steadyClocks = buf->next->msg.moveLinearShaped.steadyClocks;
			move->decelClocks = buf->next->msg.moveLinearShaped.decelClocks;
		}
		move->acceleration = params.acceleration/params.totalDistance;			// scale the acceleration to correspond to unit distance
		move->deceleration = params.deceleration/params.totalDistance;			// scale the deceleration to correspond to unit distance
		move->extruderDrives = 0;
		move->numDrivers = canDriver.localDriver + 1;
		move->zero1 = move->zero2 = 0;
		move->useLateInputShaping = params.useInputShaping;

		// Clear out the per-drive fields. Can't use a range-based FOR loop on a packed struct.
		for (size_t drive = 0; drive < ARRAY_SIZE(move->perDrive); ++drive)
		{
			move->perDrive[drive].Init();
		}
	}
	else if (canDriver.localDriver >= buf->msg.moveLinearShaped.numDrivers)
	{
		buf->msg.moveLinearShaped.numDrivers = canDriver.localDriver + 1;
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
	if (simulating)
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
				auto revertMsg = urgentMessageBuffer.SetupRequestMessageNoRid<CanMessageRevertPosition>(CanInterface::GetCanAddress(), sl->boardAddress);
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
					auto stopMsg = urgentMessageBuffer.SetupRequestMessageNoRid<CanMessageStopMovement>(CanInterface::GetCanAddress(), sl->boardAddress);
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
	//TODO sort out how we tell expansion boards about hiccups
//	hiccupToInsert += numClocks;
//	CanInterface::WakeAsyncSender();
}

// Flag a CAN-connected driver as not moving when we haven't sent the movement message yet
void CanMotion::StopDriverWhenProvisional(DriverId driver) noexcept
{
	// Search for the correct movement buffer
	CanMessageBuffer* buf = movementBufferList;
	while (buf != nullptr && buf->id.Dst() != driver.boardAddress)
	{
		buf = buf->next;
	}

	// If the move was found, set the steps to zero. We still send the message so that the drivers get enabled.
	if (buf != nullptr)
	{
		buf->msg.moveLinearShaped.perDrive[driver.localDriver].steps = 0;
	}
}

// Tell a CAN-connected driver to stop moving after we have sent the movement message
bool CanMotion::StopDriverWhenExecuting(DriverId driver, int32_t netStepsTaken) noexcept
{
	DriversStopList *sl = stopList;
	while (sl != nullptr)
	{
		if (sl->boardAddress == driver.boardAddress)
		{
			if (sl->stopStates[driver.localDriver] == DriverStopState::active)			// if active and stop not yet requested
			{
				sl->stopSteps[driver.localDriver] = netStepsTaken;						// must assign this one first
				sl->stopStates[driver.localDriver] = DriverStopState::stopRequested;
				return true;
			}
			break;																		// we found the right board, no point in searching further
		}
		sl = sl->next;
	}
	return false;
}

// Revert any stopped drivers that we haven't already and return true when there are no drivers to revert
bool CanMotion::RevertStoppedDrivers() noexcept
{
	if (!revertAll && !revertedAll)							// if not started reverting yet
	{
		revertAll = true;
		CanInterface::WakeAsyncSender();
		return false;
	}
	return !revertAll || (revertedAll && millis() - whenRevertedAll >= TotalDriverPositionRevertMillis);
}

#endif

// End
