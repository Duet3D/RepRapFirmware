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
#include <Platform/Event.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

ReadWriteLock ExpansionManager::boardsLock;

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)							OBJECT_MODEL_FUNC_BODY(ExpansionManager, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)			OBJECT_MODEL_FUNC_IF_BODY(ExpansionManager, _condition, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_ARRAY_IF(_condition, ...)		OBJECT_MODEL_FUNC_ARRAY_IF_BODY(ExpansionManager, _condition, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry ExpansionManager::objectModelArrayTable[] =
{
	// 0. Drivers
	{
		&boardsLock,
		[] (const ObjectModel *self, const ObjectExplorationContext& context) noexcept -> size_t
				{ return ((const ExpansionManager*)self)->FindIndexedBoard(context.GetLastIndex()).numDrivers; },
		[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue
				{ return ExpressionValue(&((const ExpansionManager*)self)->FindIndexedBoard(context.GetIndex(1)).driverData[context.GetLastIndex()]); }
	}
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(ExpansionManager)

constexpr ObjectModelTableEntry ExpansionManager::objectModelTable[] =
{
	// 0. boards[] members
	{ "accelerometer",		OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).hasAccelerometer, self, 4),					ObjectModelEntryFlags::none },
	{ "canAddress",			OBJECT_MODEL_FUNC((int32_t)(&(self->FindIndexedBoard(context.GetLastIndex())) - self->boards)),					ObjectModelEntryFlags::none },
	{ "closedLoop",			OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).hasClosedLoop, self, 5),					ObjectModelEntryFlags::none },
	{ "drivers",			OBJECT_MODEL_FUNC_ARRAY_IF(self->FindIndexedBoard(context.GetLastIndex()).HasDrivers(), 0),						ObjectModelEntryFlags::live },
	{ "firmwareDate",		OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::firmwareDate),		ObjectModelEntryFlags::none },
	{ "firmwareFileName",	OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName,
												(self->FindIndexedBoard(context.GetLastIndex()).usesUf2Binary)
												? ExpansionDetail::firmwareFileNameUf2
													: ExpansionDetail::firmwareFileNameBin),												ObjectModelEntryFlags::none },
	{ "firmwareVersion",	OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::firmwareVersion),	ObjectModelEntryFlags::none },
	{ "freeRam",			OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).neverUsedRam),						ObjectModelEntryFlags::live },
	{ "inductiveSensor",	OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).hasInductiveSensor, self, 6),				ObjectModelEntryFlags::none },
	{ "maxMotors",			OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).numDrivers),							ObjectModelEntryFlags::none },
	{ "mcuTemp",			OBJECT_MODEL_FUNC_IF(self->FindIndexedBoard(context.GetLastIndex()).hasMcuTemp, self, 1),						ObjectModelEntryFlags::live },
	{ "name",				OBJECT_MODEL_FUNC(self->FindIndexedBoard(context.GetLastIndex()).typeName, ExpansionDetail::longName),			ObjectModelEntryFlags::none },
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
	{ "orientation",		OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).accelerometerOrientation),			ObjectModelEntryFlags::none },
	{ "points",				OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).accelerometerLastRunDataPoints),		ObjectModelEntryFlags::none },
	{ "runs",				OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).accelerometerRuns),					ObjectModelEntryFlags::none },

	// 5. closedLoop members
	{ "points",				OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).closedLoopLastRunDataPoints),			ObjectModelEntryFlags::none },
	{ "runs",				OBJECT_MODEL_FUNC((int32_t)self->FindIndexedBoard(context.GetLastIndex()).closedLoopRuns),						ObjectModelEntryFlags::none },

	// 6. inductiveSensor members (none yet)
};

constexpr uint8_t ExpansionManager::objectModelTableDescriptor[] =
{
	7,				// number of sections
	17,				// section 0: boards[]
	3,				// section 1: mcuTemp
	3,				// section 2: vIn
	3,				// section 3: v12
	3,				// section 4: accelerometer
	2,				// section 5: closed loop
	0,				// section 6: inductive sensor
};

DEFINE_GET_OBJECT_MODEL_TABLE(ExpansionManager)

#endif

ExpansionBoardData::ExpansionBoardData() noexcept
	: typeName(nullptr), neverUsedRam(0),
	  accelerometerLastRunDataPoints(0), closedLoopLastRunDataPoints(0),
	  whenLastStatusReportReceived(0),
	  driverData(nullptr),
	  accelerometerRuns(0), closedLoopRuns(0),
	  hasMcuTemp(false), hasVin(false), hasV12(false), hasAccelerometer(false),
	  state(BoardState::unknown), numDrivers(0)
{
}

ExpansionManager::ExpansionManager() noexcept : numExpansionBoards(0), numBoardsFlashing(0), lastIndexSearched(0), lastAddressFound(0)
{
	// The boards table array is initialised by its constructor. Note, boards[0] is not used.
}

// Update the state of a board. Caller should have a write lock on boardsLock before calling this.
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
			lastAddressFound = 0;
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
			lastAddressFound = 0;
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
		{
			WriteLocker lock(boardsLock);

			board.neverUsedRam = 0;
			board.whenLastStatusReportReceived = millis();
			if (board.state == BoardState::running)
			{
				Event::AddEvent(EventType::expansion_reconnect, 0, src, 0, "");
			}
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
				// To save memory, see if we already have another board with the same type name
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
				DeleteArray(board.driverData);
				if (isNewFormat)
				{
					board.numDrivers = buf->msg.announceNew.numDrivers;
					board.usesUf2Binary = buf->msg.announceNew.usesUf2Binary;
					board.uniqueId.SetFromRemote(buf->msg.announceNew.uniqueId);
				}
				else
				{
					board.numDrivers = buf->msg.announceOld.numDrivers;
					board.usesUf2Binary = false;
					board.uniqueId.Clear();
				}
				board.driverData = new DriverData[board.numDrivers];
			}
			UpdateBoardState(src, BoardState::running);
		}

		// Tell the sending board that we don't need any more announcements from it
		buf->SetupRequestMessage<CanMessageAcknowledgeAnnounce>(0, CanInterface::GetCanAddress(), src);
		CanInterface::SendMessageNoReplyNoFree(buf);
	}
}

// Process a board status report
void ExpansionManager::ProcessBoardStatusReport(const CanMessageBuffer *buf) noexcept
{
	const CanAddress address = buf->id.Src();
	ExpansionBoardData& board = boards[address];
	board.whenLastStatusReportReceived = millis();
	if (board.state != BoardState::running && board.state != BoardState::flashing)
	{
		WriteLocker lock(boardsLock);
		UpdateBoardState(address, BoardState::running);
	}

	const CanMessageBoardStatus& msg = buf->msg.boardStatus;
	board.neverUsedRam = msg.neverUsedRam;

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
	board.hasClosedLoop = msg.hasClosedLoop;
	board.hasInductiveSensor = msg.hasInductiveSensor;

	size_t offset = msg.GetAnalogHandlesOffset();
	for (unsigned int i = 0; i < msg.numAnalogHandles && offset + sizeof(AnalogHandleData) < buf->dataLength; ++i)
	{
		AnalogHandleData data;
		memcpy(&data, (const uint8_t*)&msg + offset, sizeof(AnalogHandleData));
		offset += sizeof(AnalogHandleData);
		// Currently only Z probes use analog handles, so ask the EndstopsManager to deal with it
		if (data.handle.u.parts.type == RemoteInputHandle::typeZprobe)
		{
			reprap.GetPlatform().GetEndstops().HandleRemoteAnalogZProbeValueChange(address, data.handle.u.parts.major, data.handle.u.parts.minor, data.reading);
		}
	}
}

// Process a drive status report
void ExpansionManager::ProcessDriveStatusReport(const CanMessageBuffer *buf) noexcept
{
	const CanAddress address = buf->id.Src();
	ExpansionBoardData& board = boards[address];
	if (board.HasDrivers())
	{
		const CanMessageDriversStatus& msg = buf->msg.driversStatus;
		for (size_t driver = 0; driver < min<size_t>(board.numDrivers, msg.numDriversReported); ++driver)
		{
			DriverData& dd = board.driverData[driver];
			if (msg.hasClosedLoopData)
			{
				dd.status.all = msg.closedLoopData[driver].status;
				dd.averageCurrentFraction = msg.closedLoopData[driver].averageCurrentFraction;
				dd.maxCurrentFraction = msg.closedLoopData[driver].maxCurrentFraction;
				dd.rmsPositionError = msg.closedLoopData[driver].rmsPositionError;
				dd.maxAbsPositionError = msg.closedLoopData[driver].maxAbsPositionError;
				dd.haveClosedLoopData = true;
			}
			else
			{
				dd.status.all = msg.openLoopData[driver].status;
			}
		}

		// TODO
		(void)msg;
	}
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
	uint8_t extra;
	{
		CanMessageBuffer * const buf1 = CanInterface::AllocateBuffer(&gb);
		CanRequestId rid1 = CanInterface::AllocateRequestId(boardAddress, buf1);
		auto msg1 = buf1->SetupRequestMessage<CanMessageReturnInfo>(rid1, CanInterface::GetCanAddress(), (CanAddress)boardAddress);

		msg1->type = (moduleNumber == (unsigned int)FirmwareModule::bootloader) ? CanMessageReturnInfo::typeBootloaderName : CanMessageReturnInfo::typeBoardName;
		{
			const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf1, rid1, reply, &extra);
			if (rslt != GCodeResult::ok)
			{
				return rslt;
			}
		}
	}

	String<StringLength50> firmwareFilename;
	firmwareFilename.copy((moduleNumber == 3) ? "Duet3Bootloader-" : "Duet3Firmware_");
	firmwareFilename.cat(reply.c_str());
	// If we are updating the main firmware binary, set the extension to ".uf2" if the expansion board requested it or (for backwards compatibility) if it is a Duet 3 Mini
	firmwareFilename.cat(moduleNumber == 0 && ((extra & 0x01) != 0 || strcmp(reply.c_str(), "Mini5plus") == 0) ? ".uf2" : ".bin");

	reply.Clear();

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	if (!reprap.GetPlatform().FileExists(FIRMWARE_DIRECTORY, firmwareFilename.c_str()))
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
		WriteLocker lock(boardsLock);
		UpdateBoardState(boardAddress, BoardState::flashing);
	}
	return rslt;
}

void ExpansionManager::UpdateFinished(CanAddress address) noexcept
{
	WriteLocker lock(boardsLock);
	UpdateBoardState(address, BoardState::resetting);
}

void ExpansionManager::UpdateFailed(CanAddress address) noexcept
{
	WriteLocker lock(boardsLock);
	UpdateBoardState(address, BoardState::flashFailed);
}

void ExpansionManager::AddAccelerometerRun(CanAddress address, unsigned int numDataPoints) noexcept
{
	boards[address].accelerometerLastRunDataPoints = numDataPoints;
	++boards[address].accelerometerRuns;
	reprap.BoardsUpdated();
}

void ExpansionManager::AddClosedLoopRun(CanAddress address, unsigned int numDataPoints) noexcept
{
	boards[address].closedLoopLastRunDataPoints = numDataPoints;
	++boards[address].closedLoopRuns;
	reprap.BoardsUpdated();
}

void ExpansionManager::SaveAccelerometerOrientation(CanAddress address, uint8_t orientation) noexcept
{
	boards[address].accelerometerOrientation = orientation;
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
	// The common case is where we are looking for the same board as last time, so check for that first
	if (index == lastIndexSearched)
	{
		const unsigned int addr = lastAddressFound;
		if (index == lastIndexSearched)					// check it again in case we got interrupted
		{
			return boards[addr];
		}
	}

	// If index 0 or out of range, return the dummy entry for the main board
	if (index == 0 || index > numExpansionBoards)
	{
		return boards[0];
	}

	TaskCriticalSectionLocker lock;

	// If we are looking for a board earlier in the table than the last one, restart the search from the beginning
	if (lastIndexSearched > index)
	{
		lastIndexSearched = 0;
		lastAddressFound = 0;
	}

	unsigned int address = lastAddressFound;
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

// Check whether we have lost contact with any expansion boards
void ExpansionManager::Spin() noexcept
{
	for (CanAddress addr = 1; addr <= CanId::MaxCanAddress; ++addr)
	{
		ExpansionBoardData& board = boards[addr];
		if (board.state == BoardState::running)
		{
			// We can get interrupted here by the CanReceive task, which may update 'board.whenLastStatusReportReceived'.
			// So read and save that value before we call millis().
			const uint32_t lastTimeReceived = board.whenLastStatusReportReceived;	// capture volatile variable before we call millis()
			if (millis() - lastTimeReceived > StatusMessageTimeoutMillis)
			{
				{
					WriteLocker lock(boardsLock);
					UpdateBoardState(addr, BoardState::timedOut);
				}
				Event::AddEvent(EventType::expansion_timeout, 0, addr, 0, "");
			}
		}
	}
}

void ExpansionManager::EmergencyStop() noexcept
{
	CanMessageBuffer buf;

	// Send an individual message to each known expansion board
	for (CanAddress addr = 1; addr <= CanId::MaxCanAddress; ++addr)
	{
		if (boards[addr].state == BoardState::running)
		{
			buf.SetupRequestMessage<CanMessageEmergencyStop>(0, CanInterface::GetCanAddress(), addr);
			CanInterface::SendMessageNoReplyNoFree(&buf);
		}
	}

	// Finally, send a broadcast message in case we missed any
	buf.SetupBroadcastMessage<CanMessageEmergencyStop>(CanInterface::GetCanAddress());
	CanInterface::SendBroadcastNoFree(&buf);

	delay(10);							// allow time for the broadcast to be sent
	CanInterface::Shutdown();
}

#endif

// End
