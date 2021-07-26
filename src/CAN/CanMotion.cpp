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
static uint32_t currentMoveClocks;

#if 0
static unsigned int numMotionMessagesSentLast = 0;
#endif

static volatile uint32_t hiccupToInsert = 0;
static CanDriversList driversToStop[2];
static size_t driversToStopIndexBeingFilled = 0;
static size_t indexOfNextDriverToStop = 0;
static volatile bool stopAllFlag = false;
static bool doingStopAll = false;
static LargeBitmap<CanId::MaxCanAddress + 1> boardsActiveInLastMove;
static uint8_t nextSeq[CanId::MaxCanAddress + 1] = { 0 };

void CanMotion::Init() noexcept
{
	movementBufferList = nullptr;
	urgentMessageBuffer = CanMessageBuffer::Allocate();
	boardsActiveInLastMove.ClearAll();
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
}

CanMessageBuffer *GetBuffer(const PrepParams& params, DriverId canDriver) noexcept
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
		auto move = buf->SetupRequestMessage<CanMessageMovementLinear>(0, CanId::MasterAddress, canDriver.boardAddress);
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
uint32_t CanMotion::FinishMovement(uint32_t moveStartTime) noexcept
{
	boardsActiveInLastMove.ClearAll();
	CanMessageBuffer *buf = movementBufferList;
	if (buf == nullptr)
	{
		return 0;
	}

#if 0
	numMotionMessagesSentLast = 0;
#endif
	do
	{
		boardsActiveInLastMove.SetBit(buf->id.Dst());					//TODO should we set this if there were no steps for drives on the board, just drives to be enabled?
#if USE_REMOTE_INPUT_SHAPING
		buf->msg.moveLinearShaped.whenToExecute = moveStartTime;
		uint8_t& seq = nextSeq[buf->id.Dst()];
		buf->msg.moveLinearShaped.seq = seq;
		seq = (seq + 1) & 0x7F;
		buf->dataLength = buf->msg.moveLinearShaped.GetActualDataLength();
#else
		buf->msg.moveLinear.whenToExecute = moveStartTime;
		uint8_t& seq = nextSeq[buf->id.Dst()];
		buf->msg.moveLinear.seq = seq;
		seq = (seq + 1) & 0x7F;
		buf->dataLength = buf->msg.moveLinear.GetActualDataLength();
#endif
		CanMessageBuffer * const nextBuffer = buf->next;				// must get this before sending the buffer, because sending the buffer releases it
		CanInterface::SendMotion(buf);									// queues the buffer for sending and frees it when done
#if 0
		++numMotionMessagesSentLast;
#endif
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
CanMessageBuffer *CanMotion::GetUrgentMessage() noexcept
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
		CanDriversBitmap drivers;
		const CanAddress board = driversToStop[driversToStopIndexBeingFilled ^ 1].GetNextBoardDriverBitmap(indexOfNextDriverToStop, drivers);
		if (board != CanId::NoAddress)
		{
			auto msg = urgentMessageBuffer->SetupRequestMessage<CanMessageStopMovement>(0, CanInterface::GetCanAddress(), board);
			msg->whichDrives = drivers.GetRaw();
			return urgentMessageBuffer;
		}
		driversToStop[driversToStopIndexBeingFilled ^ 1].Clear();
		indexOfNextDriverToStop = 0;
	}

	return nullptr;
}

// The next 4 functions may be called from the step ISR, so they can't send CAN messages directly

void CanMotion::InsertHiccup(uint32_t numClocks) noexcept
{
	hiccupToInsert += numClocks;
	CanInterface::WakeAsyncSenderFromIsr();
}

void CanMotion::StopDriver(bool isBeingPrepared, DriverId driver) noexcept
{
	if (isBeingPrepared)
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
	else
	{
		driversToStop[driversToStopIndexBeingFilled].AddEntry(driver);
		CanInterface::WakeAsyncSenderFromIsr();
	}
}

void CanMotion::StopAxis(bool isBeingPrepared, size_t axis) noexcept
{
	const AxisDriversConfig& cfg = reprap.GetPlatform().GetAxisDriversConfig(axis);
	if (isBeingPrepared)
	{
		for (size_t i = 0; i < cfg.numDrivers; ++i)
		{
			const DriverId driver = cfg.driverNumbers[i];
			if (driver.IsRemote())
			{
				StopDriver(true, driver);
			}
		}
	}
	else if (!stopAllFlag)
	{
		for (size_t i = 0; i < cfg.numDrivers; ++i)
		{
			const DriverId driver = cfg.driverNumbers[i];
			if (driver.IsRemote())
			{
				driversToStop[driversToStopIndexBeingFilled].AddEntry(driver);
			}
		}
		CanInterface::WakeAsyncSenderFromIsr();
	}
}

void CanMotion::StopAll(bool isBeingPrepared) noexcept
{
	if (isBeingPrepared)
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
	else
	{
		stopAllFlag = true;
		CanInterface::WakeAsyncSenderFromIsr();
	}
}

#endif

// End
