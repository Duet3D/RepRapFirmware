/*
 * CanMotion.cpp
 *
 *  Created on: 11 Aug 2019
 *      Author: David
 */

#include "CanMotion.h"
#include <CanMessageBuffer.h>
#include <CanMessageFormats.h>
#include "CanInterface.h"

const unsigned int DriversPerCanBoard = 3;	// TEMPORARY until we do this dynamically!

const size_t NumCanBoards = (MaxCanDrivers + DriversPerCanBoard - 1)/DriversPerCanBoard;
static CanMessageBuffer *movementBuffers[NumCanBoards];

void CanMotion::Init()
{
	for (size_t i = 0; i < NumCanBoards; ++i)
	{
		movementBuffers[i] = nullptr;
	}
}

// This is called by DDA::Prepare at the start of preparing a movement
void CanMotion::StartMovement(const DDA& dda)
{
	for (CanMessageBuffer*& mb : movementBuffers)
	{
		CanMessageBuffer::Free(mb);
	}
}

// This is called by DDA::Prepare for each active CAN DM in the move
// If steps == 0 then the drivers just need to be enabled
void CanMotion::AddMovement(const DDA& dda, const PrepParams& params, DriverId canDriver, int32_t steps)
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

		auto move = buf->SetupRequestMessage<CanMessageMovement>(CanId::MasterAddress, expansionBoardNumber);

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
void CanMotion::FinishMovement(uint32_t moveStartTime)
{
	for (CanMessageBuffer*& buf : movementBuffers)
	{
		if (buf != nullptr)
		{
			buf->msg.move.whenToExecute = moveStartTime;
			CanInterface::Send(buf);					// queues the buffer for sending and frees it when done
			buf = nullptr;
		}
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

// End
