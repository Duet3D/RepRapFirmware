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

const unsigned int DriversPerCanBoard = 3;	// TEMPORARY until we do this dynamically!

const size_t NumCanBoards = (MaxCanDrivers + DriversPerCanBoard - 1)/DriversPerCanBoard;
static CanMessageBuffer *movementBufferList = nullptr;

void CanMotion::Init()
{
	movementBufferList = nullptr;
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
void CanMotion::AddMovement(const DDA& dda, const PrepParams& params, DriverId canDriver, int32_t steps)
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
		move->deltaDrives = 0;								//TODO
		move->endStopsToCheck = 0;							//TODO
		move->pressureAdvanceDrives = 0;					//TODO
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
}

// This is called by DDA::Prepare when all DMs for CAN drives have been processed
void CanMotion::FinishMovement(uint32_t moveStartTime)
{
	CanMessageBuffer *buf;
	while ((buf = movementBufferList) != nullptr)
	{
		movementBufferList = buf->next;
		buf->msg.move.whenToExecute = moveStartTime;
		CanInterface::SendMotion(buf);				// queues the buffer for sending and frees it when done
	}
}

bool CanMotion::CanPrepareMove()
{
	return CanMessageBuffer::FreeBuffers() >= NumCanBoards;
}

void CanMotion::InsertHiccup(uint32_t numClocks)
{
	//TODO
}

#endif

// End
