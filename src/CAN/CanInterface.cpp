/*
 * CanInterface.cpp
 *
 *  Created on: 19 Sep 2018
 *      Author: David
 */

#include "CanInterface.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMessageBuffer.h"
#include "CanSender.h"
#include "Movement/DDA.h"
#include "Movement/DriveMovement.h"

const unsigned int NumCanBuffers = 40;
const unsigned int DriversPerCanBoard = 3;	// TEMPORARY until we do this dynamically!

const size_t NumCanBoards = (MaxCanDrivers + DriversPerCanBoard - 1)/DriversPerCanBoard;

static CanMessageBuffer *movementBuffers[NumCanBoards];

// The following function is referenced by CanMessageBuffer in the CAN library
CanAddress GetCanAddress()
{
	return 0;				// the main board is always address 0
}

void CanInterface::Init()
{
	CanMessageBuffer::Init(NumCanBuffers);
	CanSender::Init();
	for (size_t i = 0; i < NumCanBoards; ++i)
	{
		movementBuffers[i] = nullptr;
	}
}

// This is called by DDA::Prepare at the start of preparing a movement
void CanInterface::StartMovement(const DDA& dda)
{
	for (CanMessageBuffer*& mb : movementBuffers)
	{
		CanMessageBuffer::Free(mb);
	}
}

// This is called by DDA::Prepare for each active CAN DM in the move
// If steps == 0 then the drivers just need to be enabled
void CanInterface::AddMovement(const DDA& dda, const PrepParams& params, DriverId canDriver, int32_t steps)
{
	const size_t expansionBoardNumber = canDriver.boardAddress;
	CanMessageBuffer*& buf = movementBuffers[expansionBoardNumber - 1];
	if (buf == nullptr)
	{
		buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			return;		//TODO error handling
		}

		auto move = buf->SetupRequestMessage<CanMessageMovement>(expansionBoardNumber);

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
		for (size_t drive = 0; drive < DriversPerCanBoard; ++drive)
		{
			move->perDrive[drive].steps = 0;
		}
	}

	buf->msg.move.perDrive[canDriver.localDriver].steps = steps;
}

// This is called by DDA::Prepare when all DMs for CAN drives have been processed
void CanInterface::FinishMovement(uint32_t moveStartTime)
{
	for (CanMessageBuffer*& buf : movementBuffers)
	{
		if (buf != nullptr)
		{
			buf->msg.move.whenToExecute = moveStartTime;
			CanSender::Send(buf);					// queues the buffer for sending and frees it when done
			buf = nullptr;
		}
	}
}

bool CanInterface::CanPrepareMove()
{
	return CanMessageBuffer::FreeBuffers() >= NumCanBoards;
}

void CanInterface::InsertHiccup(uint32_t numClocks)
{
	//TODO
}

#endif

// End
