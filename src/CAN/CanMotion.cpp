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

static CanMessageBuffer *movementBufferList = nullptr;
static CanMessageBuffer *urgentMessageBuffer = nullptr;

static volatile uint32_t hiccupToInsert = 0;
static CanDriversList driversToStop[2];
static size_t driversToStopIndexBeingFilled = 0;
static size_t indexOfNextDriverToStop = 0;
static volatile bool stopAllFlag = false;
static bool doingStopAll = false;
static LargeBitmap<CanId::MaxNormalAddress + 1> boardsActiveInLastMove;

void CanMotion::Init()
{
	movementBufferList = nullptr;
	urgentMessageBuffer = CanMessageBuffer::Allocate();
	boardsActiveInLastMove.ClearAll();
}

// This is called by DDA::Prepare at the start of preparing a movement
void CanMotion::StartMovement(const DDA& dda)
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
}

// This is called by DDA::Prepare for each active CAN DM in the move
// If steps == 0 then the drivers just need to be enabled
void CanMotion::AddMovement(const DDA& dda, const PrepParams& params, DriverId canDriver, int32_t steps, bool usePressureAdvance)
{
	// Search for the correct movement buffer
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
			return;		//TODO error handling
		}

		buf->next = movementBufferList;
		movementBufferList = buf;

		const CanRequestId rid = CanInterface::AllocateRequestId(canDriver.boardAddress);
		auto move = buf->SetupRequestMessage<CanMessageMovement>(rid, CanId::MasterAddress, canDriver.boardAddress);

		// Common parameters
		move->accelerationClocks = lrintf(params.accelTime * StepTimer::StepClockRate);
		move->steadyClocks = lrintf(params.steadyTime * StepTimer::StepClockRate);
		move->decelClocks = lrintf(params.decelTime * StepTimer::StepClockRate);
		move->initialSpeedFraction = params.initialSpeedFraction;
		move->finalSpeedFraction = params.finalSpeedFraction;
		move->pressureAdvanceDrives = 0;
		move->deltaDrives = 0;								//TODO
		move->endStopsToCheck = 0;							//TODO
		move->stopAllDrivesOnEndstopHit = false;			//TODO

		// Additional parameters for delta movements
		move->initialX = params.initialX;
		move->finalX = params.finalX;
		move->initialY = params.initialY;
		move->finalY = params.finalY;
		move->zMovement = params.zMovement;

		// Clear out the per-drive fields
		for (size_t drive = 0; drive < ARRAY_SIZE(move->perDrive); ++drive)
		{
			move->perDrive[drive].steps = 0;
		}
	}

	buf->msg.move.perDrive[canDriver.localDriver].steps = steps;
	if (usePressureAdvance)
	{
		buf->msg.move.pressureAdvanceDrives |= 1u << canDriver.localDriver;
	}
}

// This is called by DDA::Prepare when all DMs for CAN drives have been processed
void CanMotion::FinishMovement(uint32_t moveStartTime)
{
	boardsActiveInLastMove.ClearAll();
	CanMessageBuffer *buf;
	while ((buf = movementBufferList) != nullptr)
	{
		boardsActiveInLastMove.SetBit(buf->id.Dst());
		movementBufferList = buf->next;
		buf->msg.move.whenToExecute = moveStartTime;
		CanInterface::SendMotion(buf);				// queues the buffer for sending and frees it when done
	}
}

bool CanMotion::CanPrepareMove()
{
	return CanMessageBuffer::FreeBuffers() >= MaxCanBoards;
}

// This is called by the CanSender task to check if we have any urgent messages to send
CanMessageBuffer *CanMotion::GetUrgentMessage()
{
	if (stopAllFlag)
	{
		// Send a broadcast Stop All message first, followed by individual ones
		driversToStop[driversToStopIndexBeingFilled].Clear();
		driversToStop[driversToStopIndexBeingFilled ^ 1].Clear();
		auto msg = urgentMessageBuffer->SetupBroadcastMessage<CanMessageStopMovement>(CanInterface::GetCanAddress());
		msg->whichDrives = 0xFFFF;
		doingStopAll = true;
		stopAllFlag = false;
		indexOfNextDriverToStop = 0;
		return urgentMessageBuffer;
	}

	if (doingStopAll)
	{
		const unsigned int nextBoard = boardsActiveInLastMove.FindLowestSetBit();
		if (nextBoard < boardsActiveInLastMove.NumBits())
		{
			boardsActiveInLastMove.ClearBit(nextBoard);
			auto msg = urgentMessageBuffer->SetupRequestMessage<CanMessageStopMovement>(0, CanInterface::GetCanAddress(), nextBoard);
			msg->whichDrives = 0xFFFF;
			return urgentMessageBuffer;
		}
		doingStopAll = false;
	}

	if (driversToStop[driversToStopIndexBeingFilled ^ 1].GetNumEntries() == 0 && driversToStop[driversToStopIndexBeingFilled].GetNumEntries() != 0)
	{
		driversToStopIndexBeingFilled  = driversToStopIndexBeingFilled ^ 1;
	}

	if (driversToStop[driversToStopIndexBeingFilled ^ 1].GetNumEntries() != 0)
	{
		uint16_t drivers;
		const CanAddress board = driversToStop[driversToStopIndexBeingFilled ^ 1].GetNextBoardDriverBitmap(indexOfNextDriverToStop, drivers);
		if (board != CanId::NoAddress)
		{
			auto msg = urgentMessageBuffer->SetupRequestMessage<CanMessageStopMovement>(0, CanInterface::GetCanAddress(), board);
			msg->whichDrives = drivers;
			return urgentMessageBuffer;
		}
		driversToStop[driversToStopIndexBeingFilled ^ 1].Clear();
		indexOfNextDriverToStop = 0;
	}

	return nullptr;
}

// The next 4 functions may be called from the step ISR, so they can't send CAN messages directly

void CanMotion::InsertHiccup(uint32_t numClocks)
{
	hiccupToInsert += numClocks;
	CanInterface::WakeCanSender();
}

void CanMotion::StopDriver(DriverId driver)
{
	driversToStop[driversToStopIndexBeingFilled].AddEntry(driver);
	CanInterface::WakeCanSender();
}

void CanMotion::StopAxis(size_t axis)
{
	if (!stopAllFlag)
	{
		const AxisDriversConfig& cfg = reprap.GetPlatform().GetAxisDriversConfig(axis);
		for (size_t i = 0; i < cfg.numDrivers; ++i)
		{
			const DriverId driver = cfg.driverNumbers[i];
			if (driver.IsRemote())
			{
				driversToStop[driversToStopIndexBeingFilled].AddEntry(driver);
			}
		}
		CanInterface::WakeCanSender();
	}
}

void CanMotion::StopAll()
{
	stopAllFlag = true;
	CanInterface::WakeCanSender();
}

#endif

// End
