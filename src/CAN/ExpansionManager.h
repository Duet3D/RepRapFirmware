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
#include <Movement/StepperDrivers/DriverData.h>

NamedEnum(BoardState, uint8_t, unknown, flashing, flashFailed, resetting, running, timedOut);

struct ExpansionBoardData
{
	ExpansionBoardData() noexcept;

	bool HasDrivers() const noexcept { return numDrivers != 0 && driverData != nullptr; }

	const char *_ecv_array typeName;
	int32_t neverUsedRam;
	MinCurMax mcuTemp, vin, v12;
	uint32_t accelerometerLastRunDataPoints;
	uint32_t closedLoopLastRunDataPoints;
	volatile uint32_t whenLastStatusReportReceived;
	UniqueId uniqueId;
	DriverData *driverData;									// an array numDrivers long of objects, or nullptr if numDrivers is zero
	uint16_t accelerometerRuns;
	uint16_t closedLoopRuns;
	uint16_t hasMcuTemp : 1,
			 hasVin : 1,
			 hasV12 : 1,
			 hasAccelerometer : 1,
			 hasClosedLoop : 1,
			 hasInductiveSensor : 1,
			 usesUf2Binary : 1,
			 spare : 9;
	BoardState state;
	uint8_t numDrivers;
	uint8_t accelerometerOrientation = DefaultAccelerometerOrientation;
};

class ExpansionManager INHERIT_OBJECT_MODEL
{
public:
	ExpansionManager() noexcept;

	unsigned int GetNumExpansionBoards() const noexcept { return numExpansionBoards; }
	const ExpansionBoardData *GetBoardDetails(uint8_t address) const noexcept;

	void ProcessAnnouncement(CanMessageBuffer *buf, bool isNewFormat) noexcept;
	void ProcessBoardStatusReport(const CanMessageBuffer *buf) noexcept;
	void ProcessDriveStatusReport(const CanMessageBuffer *buf) noexcept;

	// Firmware update and related functions
	GCodeResult ResetRemote(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult UpdateRemoteFirmware(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	void UpdateFinished(CanAddress address) noexcept;
	void UpdateFailed(CanAddress address) noexcept;
	void AddAccelerometerRun(CanAddress address, unsigned int numDataPoints) noexcept;
	void AddClosedLoopRun(CanAddress address, unsigned int numDataPoints) noexcept;
	void SaveAccelerometerOrientation(CanAddress address, uint8_t orientation) noexcept;
	bool IsFlashing() const noexcept { return numBoardsFlashing != 0; }

	void Spin() noexcept;

	void EmergencyStop() noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	static constexpr uint32_t StatusMessageTimeoutMillis = 5000;	// if we don't receive a board status message for this long we presume that communication has been lost

	const ExpansionBoardData& FindIndexedBoard(unsigned int index) const noexcept;
	void UpdateBoardState(CanAddress address, BoardState newState) noexcept;

	static ReadWriteLock boardsLock;

	unsigned int numExpansionBoards;
	unsigned int numBoardsFlashing;
	mutable volatile unsigned int lastIndexSearched;		// the last board index we searched for, or 0 if invalid
	mutable volatile unsigned int lastAddressFound;			// if lastIndexSearched is nonzero, this is the corresponding board address we found
	ExpansionBoardData boards[CanId::MaxCanAddress + 1];	// the first entry is a dummy one
};

#endif	// SUPPORT_CAN_EXPANSION

#endif /* SRC_CAN_EXPANSIONMANAGER_H_ */
