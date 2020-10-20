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
template<class T>class CanDriversData
{
public:
	CanDriversData() noexcept;
	void AddEntry(DriverId id, T val) noexcept;
	size_t GetNumEntries() const noexcept { return numEntries; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept;
	T GetElement(size_t n) const pre(n < GetnumEntries()) noexcept { return data[n].val; }

private:
	struct DriverDescriptor
	{
		DriverId driver;
		T val;
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
	bool IsEmpty() const noexcept { return numEntries == 0; }
	CanAddress GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept;

private:
	size_t numEntries;
	DriverId drivers[MaxCanDrivers];
};

namespace CanInterface
{
	constexpr uint32_t CanResponseTimeout = 1000;

	// Low level functions
	void Init() noexcept;
	void Shutdown() noexcept;

	inline CanAddress GetCanAddress() noexcept { return CanId::MasterAddress; }
	CanRequestId AllocateRequestId(CanAddress destination) noexcept;
	GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *extra = nullptr) noexcept;
	void SendResponseNoFree(CanMessageBuffer *buf) noexcept;
	void SendBroadcastNoFree(CanMessageBuffer *buf) noexcept;
	void SendMessageNoReplyNoFree(CanMessageBuffer *buf) noexcept;
	void Diagnostics(MessageType mtype) noexcept;
	CanMessageBuffer *AllocateBuffer(const GCodeBuffer* gb) THROWS(GCodeException);
	void CheckCanAddress(uint32_t address, const GCodeBuffer& gb) THROWS(GCodeException);

	// Info functions
	GCodeResult GetRemoteFirmwareDetails(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult RemoteDiagnostics(MessageType mt, uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult RemoteM408(uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	// Motor control functions
	void SendMotion(CanMessageBuffer *buf) noexcept;
	void DisableRemoteDrivers(const CanDriversList& drivers) noexcept;
	void SetRemoteDriversIdle(const CanDriversList& drivers) noexcept;
	GCodeResult SetRemoteStandstillCurrentPercent(const CanDriversData<float>& data, const StringRef& reply) noexcept;
	GCodeResult SetRemoteDriverCurrents(const CanDriversData<float>& data, const StringRef& reply) noexcept;
	GCodeResult SetRemotePressureAdvance(const CanDriversData<float>& data, const StringRef& reply) noexcept;
	GCodeResult SetRemoteDriverStepsPerMmAndMicrostepping(const CanDriversData<StepsPerUnitAndMicrostepping>& data, const StringRef& reply) noexcept;
	GCodeResult ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult GetSetRemoteDriverStallParameters(const CanDriversList& drivers, GCodeBuffer& gb, const StringRef& reply, OutputBuffer *& buf) THROWS(GCodeException);
	void WakeCanSender() noexcept;

	// Remote handle functions
	GCodeResult CreateHandle(CanAddress boardAddress, RemoteInputHandle h, const char *pinName, uint16_t threshold, uint16_t minInterval, bool& currentState, const StringRef& reply) noexcept;
	GCodeResult DeleteHandle(CanAddress boardAddress, RemoteInputHandle h, const StringRef& reply) noexcept;
	GCodeResult GetHandlePinName(CanAddress boardAddress, RemoteInputHandle h, bool& currentState, const StringRef& reply) noexcept;
	GCodeResult EnableHandle(CanAddress boardAddress, RemoteInputHandle h, bool enable, bool& currentState, const StringRef& reply) noexcept;
	GCodeResult ChangeHandleResponseTime(CanAddress boardAddress, RemoteInputHandle h, uint16_t responseMillis, bool &currentState, const StringRef &reply) noexcept;

	// Filament monitor functions
	GCodeResult CreateFilamentMonitor(DriverId driver, uint8_t type, const GCodeBuffer& gb, const StringRef& reply) noexcept;
	GCodeResult ConfigureFilamentMonitor(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult DeleteFilamentMonitor(DriverId driver, GCodeBuffer* gb, const StringRef& reply) noexcept;		// called from a destructor, so must not throw

	// Misc functions
	GCodeResult WriteGpio(CanAddress boardAddress, uint8_t portNumber, float pwm, bool isServo, const GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult ChangeAddressAndNormalTiming(GCodeBuffer& gb, const StringRef& reply)THROWS(GCodeException);
	GCodeResult ChangeFastTiming(GCodeBuffer& gb, const StringRef& reply)THROWS(GCodeException);
}

// Members of template class CanDriversData
template<class T> CanDriversData<T>::CanDriversData() noexcept
{
	numEntries = 0;
}

// Insert a new entry, keeping the list ordered
template<class T> void CanDriversData<T>::AddEntry(DriverId driver, T val) noexcept
{
	if (numEntries < ARRAY_SIZE(data))
	{
		// We could do a binary search here but the number of CAN drivers supported isn't huge, so linear search instead
		size_t insertPoint = 0;
		while (insertPoint < numEntries && data[insertPoint].driver < driver)
		{
			++insertPoint;
		}
		memmove(data + (insertPoint + 1), data + insertPoint, (numEntries - insertPoint) * sizeof(data[0]));
		data[insertPoint].driver = driver;
		data[insertPoint].val = val;
		++numEntries;
	}
}

// Get the details of the drivers on the next board and advance startFrom beyond the entries for this board
template<class T> CanAddress CanDriversData<T>::GetNextBoardDriverBitmap(size_t& startFrom, CanDriversBitmap& driversBitmap) const noexcept
{
	driversBitmap.Clear();
	if (startFrom >= numEntries)
	{
		return CanId::NoAddress;
	}
	const CanAddress boardAddress = data[startFrom].driver.boardAddress;
	do
	{
		driversBitmap.SetBit(data[startFrom].driver.localDriver);
		++startFrom;
	} while (startFrom < numEntries && data[startFrom].driver.boardAddress == boardAddress);
	return boardAddress;
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
