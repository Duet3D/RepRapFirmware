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

const size_t NumCanBoards = (MaxCanDrivers + DriversPerCanBoard - 1)/DriversPerCanBoard;

static CanMessageBuffer *movementBuffers[NumCanBoards];

// CanMovementMessage is declared in project Duet3Expansion, so we need to implement its members here
void CanMovementMessage::DebugPrint()
{
	debugPrintf("Can: %08" PRIx32 " %08" PRIx32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %.2f %.2f:",
		timeNow, moveStartTime, accelerationClocks, steadyClocks, decelClocks, (double)initialSpeedFraction, (double)finalSpeedFraction);
	for (size_t i = 0; i < DriversPerCanBoard; ++i)
	{
		debugPrintf(" %" PRIi32, perDrive[i].steps);
	}
	debugPrintf("\n");
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
void CanInterface::AddMovement(const DDA& dda, const PrepParams& params, size_t canDriver, int32_t steps)
{
	const size_t expansionBoardNumber = canDriver/DriversPerCanBoard;
	CanMessageBuffer*& buf = movementBuffers[expansionBoardNumber];
	if (buf == nullptr)
	{
		buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			return;		//TODO error handling
		}

		buf->expansionBoardId = expansionBoardNumber + 1;

		// Common parameters
		buf->msg.accelerationClocks = lrintf(params.accelTime * StepTimer::StepClockRate);
		buf->msg.steadyClocks = lrintf(params.steadyTime * StepTimer::StepClockRate);
		buf->msg.decelClocks = lrintf(params.decelTime * StepTimer::StepClockRate);
		buf->msg.initialSpeedFraction = params.initialSpeedFraction;
		buf->msg.finalSpeedFraction = params.finalSpeedFraction;
		buf->msg.flags.deltaDrives = 0;							//TODO
		buf->msg.flags.endStopsToCheck = 0;						//TODO
		buf->msg.flags.pressureAdvanceDrives = 0;				//TODO
		buf->msg.flags.stopAllDrivesOnEndstopHit = false;		//TODO
		// Additional parameters for delta movements
		buf->msg.initialX = params.initialX;
		buf->msg.finalX = params.finalX;
		buf->msg.initialY = params.initialY;
		buf->msg.finalY = params.finalY;
		buf->msg.zMovement = params.zMovement;

		// Clear out the per-drive fields
		for (size_t drive = 0; drive < DriversPerCanBoard; ++drive)
		{
			buf->msg.perDrive[drive].steps = 0;
		}
	}

	buf->msg.perDrive[canDriver % DriversPerCanBoard].steps = steps;
}

// This is called by DDA::Prepare when all DMs for CAN drives` have been processed
void CanInterface::FinishMovement(uint32_t moveStartTime)
{
	for (CanMessageBuffer*& buf : movementBuffers)
	{
		if (buf != nullptr)
		{
			buf->msg.moveStartTime = moveStartTime;
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
