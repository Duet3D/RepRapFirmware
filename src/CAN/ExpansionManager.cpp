/*
 * ExpansionManager.cpp
 *
 *  Created on: 4 Feb 2020
 *      Author: David
 */

#include "ExpansionManager.h"

#if SUPPORT_CAN_EXPANSION

#include <CAN/CanInterface.h>
#include <RepRap.h>
#include <Platform.h>

ExpansionBoardData::ExpansionBoardData() noexcept : typeName(nullptr), state(BoardState::unknown)
{
	mcuTemp.min = mcuTemp.max = mcuTemp.current = vin.max = vin.min = vin.current = v12.max = v12.min = v12.current = 0.0;
}

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(ExpansionManager, __VA_ARGS__)

constexpr ObjectModelTableEntry ExpansionManager::objectModelTable[] =
{
	// 0. boards[] members
	{ "canAddress",			OBJECT_MODEL_FUNC((int32_t)(&(self->FindIndexedBoard(context.GetLastIndex())) - self->boards)),					ObjectModelEntryFlags::none },
	{ "firmwareFileName",	OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::firmwareFileName),	ObjectModelEntryFlags::none },
	{ "firmwareVersion",	OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::firmwareVersion),	ObjectModelEntryFlags::none },
	{ "maxMotors",			OBJECT_MODEL_FUNC_NOSELF((int32_t)NumDirectDrivers),															ObjectModelEntryFlags::verbose },
	{ "mcuTemp",			OBJECT_MODEL_FUNC(self, 1),																						ObjectModelEntryFlags::live },
	{ "shortName",			OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::shortName),			ObjectModelEntryFlags::none },
	{ "state",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).state.ToString()),								ObjectModelEntryFlags::none },
	{ "v12",				OBJECT_MODEL_FUNC(self, 3),																						ObjectModelEntryFlags::live },
	{ "vIn",				OBJECT_MODEL_FUNC(self, 2),																						ObjectModelEntryFlags::live },

	// 1. mcuTemp members
	{ "current",			OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).mcuTemp.current, 1),							ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).mcuTemp.max, 1),								ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).mcuTemp.min, 1),								ObjectModelEntryFlags::none },

	// 2. vIn members
	{ "current",			OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).vin.current, 1),								ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).vin.max, 1),									ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).vin.min, 1),									ObjectModelEntryFlags::none },

	// 3. v12 members
	{ "current",			OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).v12.current, 1),								ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).v12.max, 1),									ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).v12.min, 1),									ObjectModelEntryFlags::none },
};

constexpr uint8_t ExpansionManager::objectModelTableDescriptor[] =
{
	4,				// number of sections
	9,				// section 0: boards[]
	3,				// section 1: mcuTemp
	3,				// section 2: vIn
	3				// section 3: v12
};

DEFINE_GET_OBJECT_MODEL_TABLE(ExpansionManager)

#endif

ExpansionManager::ExpansionManager() noexcept : numExpansionBoards(0), numBoardsFlashing(0), lastIndexSearched(0), lastAddressFound(0)
{
	// the boards table array is initialised by its constructor
}

// Update the state of a board
void ExpansionManager::UpdateBoardState(CanAddress address, BoardState newState) noexcept
{
	ExpansionBoardData& board = boards[address];
	TaskCriticalSectionLocker lock;

	const BoardState oldState = board.state;
	if (newState != oldState)
	{
		board.state = newState;
		if (oldState == BoardState::unknown)
		{
			++numExpansionBoards;
			lastIndexSearched = 0;
		}
		else if (oldState == BoardState::flashing && numBoardsFlashing != 0)
		{
			--numBoardsFlashing;
		}

		if (newState == BoardState::flashing)
		{
			++numBoardsFlashing;
		}
		else if (newState == BoardState::unknown && numExpansionBoards != 0)
		{
			--numExpansionBoards;
			lastIndexSearched = 0;
		}
		reprap.BoardsUpdated();
	}
}

// Process an announcement from an expansion board and free the message buffer that it arrived in
void ExpansionManager::ProcessAnnouncement(CanMessageBuffer *buf) noexcept
{
	const CanAddress src = buf->id.Src();
	if (src <= CanId::MaxCanAddress)
	{
		ExpansionBoardData& board = boards[src];
		String<StringLength100> boardTypeAndFirmwareVersion;
		boardTypeAndFirmwareVersion.copy(buf->msg.announce.boardTypeAndFirmwareVersion, CanMessageAnnounce::GetMaxTextLength(buf->dataLength));
		UpdateBoardState(src, BoardState::unknown);
		if (board.typeName == nullptr || strcmp(board.typeName, boardTypeAndFirmwareVersion.c_str()) != 0)
		{
			const char *newTypeName = nullptr;
			for (const ExpansionBoardData& data : boards)
			{
				if (data.typeName != nullptr && strcmp(boardTypeAndFirmwareVersion.c_str(), data.typeName) == 0)
				{
					newTypeName = data.typeName;
					break;
				}
			}
			if (newTypeName == nullptr)
			{
				char * const temp = new char[boardTypeAndFirmwareVersion.strlen() + 1];
				strcpy(temp, boardTypeAndFirmwareVersion.c_str());
				newTypeName = temp;
			}
			board.typeName = newTypeName;
		}
		UpdateBoardState(src, BoardState::running);
	}
	buf->SetupResponseMessage<CanMessageAcknowledgeAnnounce>(0, CanId::MasterAddress, src);
	CanInterface::SendResponse(buf);
}

// Tell an expansion board to update
GCodeResult ExpansionManager::UpdateRemoteFirmware(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanInterface::CheckCanAddress(boardAddress, gb);

	// Ask the board for its type and check we have the firmware file for it
	CanMessageBuffer * const buf1 = CanInterface::AllocateBuffer(gb);
	CanRequestId rid1 = CanInterface::AllocateRequestId(boardAddress);
	auto msg1 = buf1->SetupRequestMessage<CanMessageReturnInfo>(rid1, CanId::MasterAddress, (CanAddress)boardAddress);
	msg1->type = CanMessageReturnInfo::typeBoardName;
	const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf1, rid1, reply);
	if (rslt != GCodeResult::ok)
	{
		return rslt;
	}

	String<StringLength50> firmwareFilename;
	firmwareFilename.copy("Duet3Firmware_");
	firmwareFilename.cat(reply.c_str());
	reply.Clear();
	firmwareFilename.cat(".bin");

	// Do not ask Linux for a file here because that would create a deadlock.
	// If blocking calls to Linux are supposed to be made from the Spin loop, the Linux interface,
	// or at least the code doing SPI data transfers, has to be moved to a separate task first
#if HAS_MASS_STORAGE
	// It's fine to check if the file exists on the local SD though
	if (
# if HAS_LINUX_INTERFACE
			!reprap.UsingLinuxInterface() &&
# endif
			!reprap.GetPlatform().FileExists(DEFAULT_SYS_DIR, firmwareFilename.c_str()))
	{
		reply.printf("Firmware file %s not found", firmwareFilename.c_str());
		return GCodeResult::error;
	}
#endif

	CanMessageBuffer * const buf2 = CanInterface::AllocateBuffer(gb);
	const CanRequestId rid2 = CanInterface::AllocateRequestId(boardAddress);
	auto msg2 = buf2->SetupRequestMessage<CanMessageUpdateYourFirmware>(rid2, CanId::MasterAddress, (CanAddress)boardAddress);
	msg2->boardId = (uint8_t)boardAddress;
	msg2->invertedBoardId = (uint8_t)~boardAddress;
	UpdateBoardState(boardAddress, BoardState::flashing);
	return CanInterface::SendRequestAndGetStandardReply(buf2, rid2, reply);
}

void ExpansionManager::UpdateFinished(CanAddress address) noexcept
{
	UpdateBoardState(address, BoardState::resetting);
}

void ExpansionManager::UpdateFailed(CanAddress address) noexcept
{
	UpdateBoardState(address, BoardState::flashFailed);
}

GCodeResult ExpansionManager::ResetRemote(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanInterface::CheckCanAddress(boardAddress, gb);
	CanMessageBuffer * const buf = CanInterface::AllocateBuffer(gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	buf->SetupRequestMessage<CanMessageReset>(rid, CanId::MasterAddress, (uint8_t)boardAddress);
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

const ExpansionBoardData& ExpansionManager::FindIndexedBoard(unsigned int index) const noexcept
{
	if (index == 0 || index > numExpansionBoards)
	{
		return boards[0];
	}
	if (lastIndexSearched > index)
	{
		lastIndexSearched = 0;
	}

	TaskCriticalSectionLocker lock;
	unsigned int address = (lastIndexSearched == 0) ? 0 : lastAddressFound;
	unsigned int currentIndex = lastIndexSearched;
	while (currentIndex < index)
	{
		++address;
		if (address == ARRAY_SIZE(boards))
		{
			return boards[0];
		}
		if (boards[address].state != BoardState::unknown)
		{
			++currentIndex;
		}
	}

	lastIndexSearched = index;
	lastAddressFound = address;
	return boards[address];
}

void ExpansionManager::EmergencyStop() noexcept
{
	for (unsigned int i = 0; i < 1000; ++i)
	{
		CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
		if (buf != nullptr)
		{
			// Send an individual message to each known expansion board
			for (CanAddress addr = 1; addr <= CanId::MaxCanAddress; ++addr)
			{
				if (boards[addr].state == BoardState::running)
				{
					buf->SetupRequestMessage<CanMessageEmergencyStop>(0, CanId::MasterAddress, addr);
					CanInterface::SendMessageNoReplyNoFree(buf);
				}
			}

			// Finally, send a broadcast message in case we missed any, and free the buffer
			buf->SetupBroadcastMessage<CanMessageEmergencyStop>(CanId::MasterAddress);
			CanInterface::SendBroadcast(buf);
			break;
		}
		delay(1);				// wait for a buffer to become available
	}
}

#endif

// End
