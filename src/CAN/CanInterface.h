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
#include <CanMessageFormats.h>

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
	CanDriversList() : numEntries(0) { }
	void Clear() { numEntries = 0; }
	void AddEntry(DriverId id);
	size_t GetNumEntries() const { return numEntries; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, uint16_t& driversBitmap) const;

private:
	size_t numEntries;
	DriverId drivers[MaxCanDrivers];
};

namespace CanInterface
{
	static constexpr uint32_t CanResponseTimeout = 1000;

	void Init();
	inline CanAddress GetCanAddress() { return CanId::MasterAddress; }
	CanRequestId AllocateRequestId(CanAddress destination);
	GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *extra = nullptr);
	void SendResponse(CanMessageBuffer *buf);
	void SendBroadcast(CanMessageBuffer *buf);

	GCodeResult GetRemoteFirmwareDetails(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply);
	GCodeResult RemoteDiagnostics(MessageType mt, uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply);

	GCodeResult UpdateRemoteFirmware(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply);
	bool IsFlashing();
	void UpdateStarting();
	void UpdateFinished();

	// Motor control functions
	void SendMotion(CanMessageBuffer *buf);
	void DisableRemoteDrivers(const CanDriversList& drivers);
	void SetRemoteDriversIdle(const CanDriversList& drivers);
	bool SetRemoteStandstillCurrentPercent(const CanDriversData& data, const StringRef& reply);
	bool SetRemoteDriverCurrents(const CanDriversData& data, const StringRef& reply);
	bool SetRemoteDriverMicrostepping(const CanDriversData& data, const StringRef& reply);
	bool SetRemotePressureAdvance(const CanDriversData& data, const StringRef& reply);
	GCodeResult ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply);
	GCodeResult SetRemoteDriverStallParameters(const CanDriversList& drivers, GCodeBuffer& gb, const StringRef& reply);
	void WakeCanSender();

	// Remote handle functions
	GCodeResult CreateHandle(CanAddress boardAddress, RemoteInputHandle h, const char *pinName, uint16_t threshold, uint16_t minInterval, bool& currentState, const StringRef& reply);
	GCodeResult DeleteHandle(CanAddress boardAddress, RemoteInputHandle h, const StringRef& reply);
	GCodeResult GetHandlePinName(CanAddress boardAddress, RemoteInputHandle h, bool& currentState, const StringRef& reply);
	GCodeResult EnableHandle(CanAddress boardAddress, RemoteInputHandle h, bool& currentState, const StringRef& reply);

	void Diagnostics(MessageType mtype);
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
