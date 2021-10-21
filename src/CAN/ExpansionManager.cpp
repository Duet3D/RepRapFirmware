/*
 * ExpansionManager.cpp
 *
 *  Created on: 4 Feb 2020
 *      Author: David
 */

#include "ExpansionManager.h"

#if SUPPORT_CAN_EXPANSION

#include <CAN/CanInterface.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(ExpansionManager, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...) OBJECT_MODEL_FUNC_IF_BODY(ExpansionManager, __VA_ARGS__)

constexpr ObjectModelTableEntry ExpansionManager::objectModelTable[] =
{
	// 0. boards[] members
	{ "accelerometer",		OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).hasAccelerometer, self, 4),					ObjectModelEntryFlags::none },
	{ "canAddress",			OBJECT_MODEL_FUNC((int32_t)(&(self->FindIndexedBoard(context.GetLastIndex())) - self->boards)),					ObjectModelEntryFlags::none },
	{ "firmwareDate",		OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::firmwareDate),		ObjectModelEntryFlags::none },
	{ "firmwareFileName",	OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::firmwareFileName),	ObjectModelEntryFlags::none },
	{ "firmwareVersion",	OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::firmwareVersion),	ObjectModelEntryFlags::none },
	{ "maxMotors",			OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).numDrivers),							ObjectModelEntryFlags::verbose },
	{ "mcuTemp",			OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).hasMcuTemp, self, 1),						ObjectModelEntryFlags::live },
	{ "shortName",			OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::shortName),			ObjectModelEntryFlags::none },
	{ "state",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).state.ToString()),								ObjectModelEntryFlags::none },
	{ "uniqueId",			OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).uniqueId.IsValid(),
													self->FindIndexedBoard(context.GetLastIndex()).uniqueId),								ObjectModelEntryFlags::none },
	{ "v12",				OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).hasV12, self, 3),							ObjectModelEntryFlags::live },
	{ "vIn",				OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).hasVin, self, 2),							ObjectModelEntryFlags::live },

	// 1. mcuTemp members
	{ "current",			OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).mcuTemp.current, 1),							ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).mcuTemp.maximum, 1),							ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).mcuTemp.minimum, 1),							ObjectModelEntryFlags::none },

	// 2. vIn members
	{ "current",			OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).vin.current, 1),								ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).vin.maximum, 1),								ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).vin.minimum, 1),								ObjectModelEntryFlags::none },

	// 3. v12 members
	{ "current",			OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).v12.current, 1),								ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).v12.maximum, 1),								ObjectModelEntryFlags::none },
	{ "min",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).v12.minimum, 1),								ObjectModelEntryFlags::none },

	// 4. accelerometer members
	{ "points",				OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).accelerometerLastRunDataPoints),		ObjectModelEntryFlags::none },
	{ "runs",				OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).accelerometerRuns),					ObjectModelEntryFlags::none },
};

constexpr uint8_t ExpansionManager::objectModelTableDescriptor[] =
{
	5,				// number of sections
	12,				// section 0: boards[]
	3,				// section 1: mcuTemp
	3,				// section 2: vIn
	3,				// section 3: v12
	2				// section 4: accelerometer
};

DEFINE_GET_OBJECT_MODEL_TABLE(ExpansionManager)

#endif

ExpansionBoardData::ExpansionBoardData() noexcept
	: typeName(nullptr), hasMcuTemp(false), hasVin(false), hasV12(false), hasAccelerometer(false),
	  accelerometerRuns(0), accelerometerLastRunDataPoints(0),
	  state(BoardState::unknown), numDrivers(0)
{
}

ExpansionManager::ExpansionManager() noexcept : numExpansionBoards(0), numBoardsFlashing(0), lastIndexSearched(0), lastAddressFound(0)
{
	// The boards table array is initialised by its constructor. Note, boards[0] is not used.
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

// Process an announcement from an expansion board. Don't free the message buffer that it arrived in
void ExpansionManager::ProcessAnnouncement(CanMessageBuffer *buf, bool isNewFormat) noexcept
{
	const CanAddress src = buf->id.Src();
	if (src <= CanId::MaxCanAddress)
	{
		ExpansionBoardData& board = boards[src];
		board.hasVin = board.hasV12 = board.hasMcuTemp = false;
		String<StringLength100> boardTypeAndFirmwareVersion;
		if (isNewFormat)
		{
			boardTypeAndFirmwareVersion.copy(buf->msg.announceNew.boardTypeAndFirmwareVersion, CanMessageAnnounceNew::GetMaxTextLength(buf->dataLength));
		}
		else
		{
			boardTypeAndFirmwareVersion.copy(buf->msg.announceOld.boardTypeAndFirmwareVersion, CanMessageAnnounceOld::GetMaxTextLength(buf->dataLength));
		}
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
			if (isNewFormat)
			{
				board.numDrivers = buf->msg.announceNew.numDrivers;
				board.uniqueId.SetFromRemote(buf->msg.announceNew.uniqueId);
			}
			else
			{
				board.numDrivers = buf->msg.announceOld.numDrivers;
				board.uniqueId.Clear();
			}
		}
		UpdateBoardState(src, BoardState::running);
	}
	buf->SetupResponseMessage<CanMessageAcknowledgeAnnounce>(0, CanInterface::GetCanAddress(), src);
	CanInterface::SendResponseNoFree(buf);
}

// Process a board status report
void ExpansionManager::ProcessBoardStatusReport(const CanMessageBuffer *buf) noexcept
{
	const CanAddress address = buf->id.Src();
	ExpansionBoardData& board = boards[address];
	if (board.state != BoardState::running && board.state != BoardState::flashing)
	{
		UpdateBoardState(address, BoardState::running);
	}

	const CanMessageBoardStatus msg = buf->msg.boardStatus;

	// We must process the data in the correct order, to ensure that we pick up the right values
	size_t index = 0;
	board.hasVin = msg.hasVin;
	if (msg.hasVin)
	{
		board.vin = msg.values[index++];
		board.hasVin = true;
	}
	board.hasV12 = msg.hasV12;
	if (msg.hasV12)
	{
		board.v12 = msg.values[index++];
	}
	board.hasMcuTemp = msg.hasMcuTemp;
	if (msg.hasMcuTemp)
	{
		board.mcuTemp = msg.values[index++];
	}
	board.hasAccelerometer = msg.hasAccelerometer;
}

// Return a pointer to the expansion board, if it is present
const ExpansionBoardData *ExpansionManager::GetBoardDetails(uint8_t address) const noexcept
{
	return (address < ARRAY_SIZE(boards) && boards[address].state == BoardState::running) ? &boards[address] : nullptr;
}

// Tell an expansion board to update
GCodeResult ExpansionManager::UpdateRemoteFirmware(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanInterface::CheckCanAddress(boardAddress, gb);

	const unsigned int moduleNumber = (gb.Seen('S')) ? gb.GetLimitedUIValue('S', 4) : 0;
	if (moduleNumber != (unsigned int)FirmwareModule::main && moduleNumber != (unsigned int)FirmwareModule::bootloader)
	{
		reply.printf("Unknown module number %u", moduleNumber);
		return GCodeResult::error;
	}

	// Ask the board for its type and check we have the firmware file for it
	{
		CanMessageBuffer * const buf1 = CanInterface::AllocateBuffer(&gb);
		CanRequestId rid1 = CanInterface::AllocateRequestId(boardAddress, buf1);
		auto msg1 = buf1->SetupRequestMessage<CanMessageReturnInfo>(rid1, CanInterface::GetCanAddress(), (CanAddress)boardAddress);

		msg1->type = (moduleNumber == (unsigned int)FirmwareModule::bootloader) ? CanMessageReturnInfo::typeBootloaderName : CanMessageReturnInfo::typeBoardName;
		{
			const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf1, rid1, reply);
			if (rslt != GCodeResult::ok)
			{
				return rslt;
			}
		}
	}

	String<StringLength50> firmwareFilename;
	firmwareFilename.copy((moduleNumber == 3) ? "Duet3Bootloader-" : "Duet3Firmware_");
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
			!reprap.GetPlatform().FileExists(FIRMWARE_DIRECTORY, firmwareFilename.c_str()))
	{
		reply.printf("Firmware file %s not found", firmwareFilename.c_str());
		return GCodeResult::error;
	}
#endif

	CanMessageBuffer * const buf2 = CanInterface::AllocateBuffer(&gb);
	const CanRequestId rid2 = CanInterface::AllocateRequestId(boardAddress, buf2);
	auto msg2 = buf2->SetupRequestMessage<CanMessageUpdateYourFirmware>(rid2, CanInterface::GetCanAddress(), (CanAddress)boardAddress);
	msg2->boardId = (uint8_t)boardAddress;
	msg2->invertedBoardId = (uint8_t)~boardAddress;
	msg2->module = moduleNumber;
	const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf2, rid2, reply);
	if (rslt == GCodeResult::ok)
	{
		UpdateBoardState(boardAddress, BoardState::flashing);
	}
	return rslt;
}

void ExpansionManager::UpdateFinished(CanAddress address) noexcept
{
	UpdateBoardState(address, BoardState::resetting);
}

void ExpansionManager::UpdateFailed(CanAddress address) noexcept
{
	UpdateBoardState(address, BoardState::flashFailed);
}

void ExpansionManager::AddAccelerometerRun(CanAddress address, unsigned int numDataPoints) noexcept
{
	boards[address].accelerometerLastRunDataPoints = numDataPoints;
	++boards[address].accelerometerRuns;
	reprap.BoardsUpdated();
}

GCodeResult ExpansionManager::ResetRemote(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	CanInterface::CheckCanAddress(boardAddress, gb);
	CanMessageBuffer * const buf = CanInterface::AllocateBuffer(&gb);
	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress, buf);
	buf->SetupRequestMessage<CanMessageReset>(rid, CanInterface::GetCanAddress(), (uint8_t)boardAddress);
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
	CanMessageBuffer *buf;
	while ((buf = CanMessageBuffer::Allocate()) == nullptr)
	{
		delay(1);
	}

//	debugPrintf("Allocated buffer\n");
	// Send an individual message to each known expansion board
	for (CanAddress addr = 1; addr <= CanId::MaxCanAddress; ++addr)
	{
		if (boards[addr].state == BoardState::running)
		{
			buf->SetupRequestMessage<CanMessageEmergencyStop>(0, CanInterface::GetCanAddress(), addr);
			CanInterface::SendMessageNoReplyNoFree(buf);
		}
	}
//	debugPrintf("sent individual messages\n");

	// Finally, send a broadcast message in case we missed any, and free the buffer
	buf->SetupBroadcastMessage<CanMessageEmergencyStop>(CanInterface::GetCanAddress());
	CanInterface::SendBroadcastNoFree(buf);

	CanMessageBuffer::Free(buf);
//	debugPrintf("sent broadcast\n");

	delay(10);							// allow time for the broadcast to be sent
	CanInterface::Shutdown();
}

#endif

// End
