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

typedef Bitmap<uint16_t> CanDriversBitmap;

// Class to accumulate a set of values relating to CAN-connected drivers
class CanDriversData
{
public:
	CanDriversData() noexcept;
	void AddEntry(DriverId id, uint16_t val) noexcept;
	size_t GetNumEntries() const noexcept { return numEntries; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept;
	uint16_t GetElement(size_t n) const pre(n < GetnumEntries()) noexcept { return data[n].val; }

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
	CanDriversList() noexcept : numEntries(0) { }
	void Clear() noexcept { numEntries = 0; }
	void AddEntry(DriverId id) noexcept;
	size_t GetNumEntries() const noexcept { return numEntries; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept;

private:
	size_t numEntries;
	DriverId drivers[MaxCanDrivers];
};

namespace CanInterface
{
	static constexpr uint32_t CanResponseTimeout = 1000;

	// Low level functions
	void Init() noexcept;
	inline CanAddress GetCanAddress() noexcept { return CanId::MasterAddress; }
	CanRequestId AllocateRequestId(CanAddress destination) noexcept;
	GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *extra = nullptr) noexcept;
	void SendResponse(CanMessageBuffer *buf) noexcept;
	void SendBroadcast(CanMessageBuffer *buf) noexcept;
	void Diagnostics(MessageType mtype) noexcept;
	CanMessageBuffer *AllocateBuffer(const GCodeBuffer& gb) THROWS_GCODE_EXCEPTION;

	// Info functions
	GCodeResult GetRemoteFirmwareDetails(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) noexcept;
	GCodeResult RemoteDiagnostics(MessageType mt, uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) noexcept;
	GCodeResult RemoteM408(uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) noexcept;

	// Firmware update functions
	GCodeResult UpdateRemoteFirmware(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) noexcept;
	bool IsFlashing() noexcept;
	void UpdateStarting() noexcept;
	void UpdateFinished() noexcept;

	// Motor control functions
	void SendMotion(CanMessageBuffer *buf) noexcept;
	void DisableRemoteDrivers(const CanDriversList& drivers) noexcept;
	void SetRemoteDriversIdle(const CanDriversList& drivers) noexcept;
	bool SetRemoteStandstillCurrentPercent(const CanDriversData& data, const StringRef& reply) noexcept;
	bool SetRemoteDriverCurrents(const CanDriversData& data, const StringRef& reply) noexcept;
	bool SetRemoteDriverMicrostepping(const CanDriversData& data, const StringRef& reply) noexcept;
	bool SetRemotePressureAdvance(const CanDriversData& data, const StringRef& reply) noexcept;
	GCodeResult ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS_GCODE_EXCEPTION;
	GCodeResult SetRemoteDriverStallParameters(const CanDriversList& drivers, GCodeBuffer& gb, const StringRef& reply) THROWS_GCODE_EXCEPTION;
	void WakeCanSender() noexcept;

	// Remote handle functions
	GCodeResult CreateHandle(CanAddress boardAddress, RemoteInputHandle h, const char *pinName, uint16_t threshold, uint16_t minInterval, bool& currentState, const StringRef& reply) noexcept;
	GCodeResult DeleteHandle(CanAddress boardAddress, RemoteInputHandle h, const StringRef& reply) noexcept;
	GCodeResult GetHandlePinName(CanAddress boardAddress, RemoteInputHandle h, bool& currentState, const StringRef& reply) noexcept;
	GCodeResult EnableHandle(CanAddress boardAddress, RemoteInputHandle h, bool& currentState, const StringRef& reply) noexcept;

	// Misc functions
	GCodeResult WriteGpio(CanAddress boardAddress, uint8_t portNumber, float pwm, bool isServo, const GCodeBuffer& gb, const StringRef& reply) THROWS_GCODE_EXCEPTION;
	GCodeResult SetFastDataRate(GCodeBuffer& gb, const StringRef& reply)THROWS_GCODE_EXCEPTION;
	GCodeResult ChangeExpansionBoardAddress(GCodeBuffer& gb, const StringRef& reply)THROWS_GCODE_EXCEPTION;
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
