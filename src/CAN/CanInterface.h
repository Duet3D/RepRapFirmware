/*
 * CanInterface.h
 *
 *  Created on: 19 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANINTERFACE_H_
#define SRC_CAN_CANINTERFACE_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

#include "GCodes/GCodeResult.h"
#include "MessageType.h"
#include <CanId.h>

class CanMessageBuffer;
class DDA;
class DriveMovement;
struct PrepParams;

// Class to accumulate a set of values relating to CAN-connected drivers
class CanDriversData
{
public:
	CanDriversData();
	void AddEntry(DriverId id, uint16_t val);
	size_t GetNumEntries() const { return numEntries; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, uint16_t& driversBitmap) const;
	uint16_t GetElement(size_t n) const pre(n < GetnumEntries()) { return data[n].val; }

private:
	struct DriverDescriptor
	{
		DriverId driver;
		uint16_t val;
	};

	size_t numEntries;
	DriverDescriptor data[MaxCanDrivers];
};

class CanDriversList
{
public:
	CanDriversList();
	void AddEntry(DriverId id);
	size_t GetNumEntries() const { return numEntries; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, uint16_t& driversBitmap) const;

private:
	size_t numEntries;
	DriverId drivers[MaxCanDrivers];
};

namespace CanInterface
{
	static constexpr uint32_t CanResponseTimeout = 300;

	void Init();
	CanAddress GetCanAddress();
	GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer *buf, const StringRef& reply);
	void SendResponse(CanMessageBuffer *buf);

	GCodeResult RemoteDiagnostics(MessageType mt, uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply);
	GCodeResult UpdateRemoteFirmware(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply);

	// Motor control functions
	void SendMotion(CanMessageBuffer *buf);
	void DisableRemoteDrivers(const CanDriversList& drivers);
	void SetRemoteDriversIdle(const CanDriversList& drivers);
	bool SetRemoteStandstillCurrentPercent(const CanDriversData& data, const StringRef& reply);
	bool SetRemoteDriverCurrents(const CanDriversData& data, const StringRef& reply);
	bool SetRemoteDriverMicrostepping(const CanDriversData& data, const StringRef& reply);
	GCodeResult ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply);
	GCodeResult SetRemoteDriverStallParameters(const CanDriversList& drivers, GCodeBuffer& gb, const StringRef& reply);
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
