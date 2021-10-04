/*
 * ExpansionManager.h
 *
 *  Created on: 4 Feb 2020
 *      Author: David
 */

#ifndef SRC_CAN_EXPANSIONMANAGER_H_
#define SRC_CAN_EXPANSIONMANAGER_H_

#include <RepRapFirmware.h>

#if SUPPORT_CAN_EXPANSION

#include <ObjectModel/ObjectModel.h>
#include <CanId.h>
#include <CanMessageBuffer.h>
#include <General/NamedEnum.h>
#include <Platform/UniqueId.h>

NamedEnum(BoardState, uint8_t, unknown, flashing, flashFailed, resetting, running);

struct ExpansionBoardData
{
	ExpansionBoardData() noexcept;

	const char *typeName;
	UniqueId uniqueId;
	MinCurMax mcuTemp, vin, v12;
	uint16_t hasMcuTemp : 1,
			 hasVin : 1,
			 hasV12 : 1,
			 hasAccelerometer : 1,
			 spare : 12;
	uint16_t accelerometerRuns;
	uint16_t accelerometerLastRunDataPoints;
	BoardState state;
	uint8_t numDrivers;
};

class ExpansionManager INHERIT_OBJECT_MODEL
{
public:
	ExpansionManager() noexcept;

	unsigned int GetNumExpansionBoards() const noexcept { return numExpansionBoards; }
	const ExpansionBoardData *GetBoardDetails(uint8_t address) const noexcept;

	void ProcessAnnouncement(CanMessageBuffer *buf, bool isNewFormat) noexcept;
	void ProcessBoardStatusReport(const CanMessageBuffer *buf) noexcept;

	// Firmware update and related functions
	GCodeResult ResetRemote(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult UpdateRemoteFirmware(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	void UpdateFinished(CanAddress address) noexcept;
	void UpdateFailed(CanAddress address) noexcept;
	void AddAccelerometerRun(CanAddress address, unsigned int numDataPoints) noexcept;
	bool IsFlashing() const noexcept { return numBoardsFlashing != 0; }

	void EmergencyStop() noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	const ExpansionBoardData& FindIndexedBoard(unsigned int index) const noexcept;
	void UpdateBoardState(CanAddress address, BoardState newState) noexcept;

	unsigned int numExpansionBoards;
	unsigned int numBoardsFlashing;
	mutable volatile unsigned int lastIndexSearched;		// the last board index we searched for, or 0 if invalid
	mutable volatile unsigned int lastAddressFound;			// if lastIndexSearched is nonzero, this is the corresponding board address we found
	ExpansionBoardData boards[CanId::MaxCanAddress + 1];	// the first entry is a dummy one
};

#endif	// SUPPORT_CAN_EXPANSION

#endif /* SRC_CAN_EXPANSIONMANAGER_H_ */
