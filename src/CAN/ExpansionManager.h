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
#include <GCodes/GCodeResult.h>
#include <General/NamedEnum.h>

NamedEnum(BoardState, uint8_t, unknown, flashing, flashFailed, resetting, running);

struct ExpansionBoardData
{
	ExpansionBoardData() noexcept;

	const char *typeName;
	MinMaxCurrent mcuTemp, vin, v12;
	BoardState state;
};

class ExpansionManager INHERIT_OBJECT_MODEL
{
public:
	ExpansionManager() noexcept;

	unsigned int GetNumExpansionBoards() const noexcept { return numExpansionBoards; }
	void ProcessAnnouncement(CanMessageBuffer *buf) noexcept;

	// Firmware update and related functions
	GCodeResult ResetRemote(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult UpdateRemoteFirmware(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	void UpdateFinished(CanAddress address) noexcept;
	void UpdateFailed(CanAddress address) noexcept;
	bool IsFlashing() const noexcept { return numBoardsFlashing != 0; }

	void EmergencyStop() noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	void UpdateBoardState(CanAddress address, BoardState newState) noexcept;
	const ExpansionBoardData& FindIndexedBoard(unsigned int index) const noexcept;

	unsigned int numExpansionBoards;
	unsigned int numBoardsFlashing;
	mutable volatile unsigned int lastIndexSearched;		// the last board index we searched for, or 0 if invalid
	mutable volatile unsigned int lastAddressFound;			// if lastIndexSearched is nonzero, this is the corresponding board address we found
	ExpansionBoardData boards[CanId::MaxCanAddress + 1];	// the first entry is a dummy one
};

#endif	// SUPPORT_CAN_EXPANSION

#endif /* SRC_CAN_EXPANSIONMANAGER_H_ */
