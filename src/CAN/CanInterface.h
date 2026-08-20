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

#include <CanId.h>
#include <CanMessageFormats.h>
#include "CanDriversData.h"

class CanMessageBuffer;
class DDA;
class DriveMovement;
struct PrepParams;

namespace CanInterface
{
	// Note: GetCanAddress() in this namespace is now declared in RepRapFirmware.h to overcome ordering issues
	constexpr uint32_t UsualResponseTimeout = 1000;				// how long we normally wait for a response, in milliseconds
	constexpr uint32_t UsualSendTimeout = 200;					// how long we normally wait to send a message, in milliseconds

	// Low level functions
	void Init() noexcept;
	void Shutdown() noexcept;
	inline CanAddress GetCurrentMasterAddress() noexcept { return CanId::MasterAddress; }		// currently fixed, but might change in future

	void ReportCanTiming(const StringRef& reply) noexcept;

#if SUPPORT_REMOTE_COMMANDS
	bool InExpansionMode() noexcept;
	bool InTestMode() noexcept;
	void SwitchToExpansionMode(CanAddress addr, bool useTestMode) noexcept;

	void SendAnnounce(CanMessageBuffer *buf) noexcept;
	void RaiseEvent(EventType type, uint16_t param, uint8_t device, const char *_ecv_array format, va_list vargs) noexcept;
	void MainBoardAcknowledgedAnnounce() noexcept;
	void LogIgnoredMovementMessage() noexcept;
	void CheckBrs(const CanMessageTimeSync& msg) noexcept;
#endif

	CanRequestId AllocateRequestId(CanAddress destination, CanMessageBuffer *buf) noexcept;
	GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *_ecv_null extra = nullptr, uint32_t *_ecv_null words = nullptr) noexcept;
	GCodeResult SendRequestAndGetCustomReply(CanMessageBuffer *buf, CanRequestId rid, const StringRef& reply, uint8_t *_ecv_null extra, uint32_t *_ecv_null words, CanMessageType replyType, function_ref_noexcept<void(const CanMessageBuffer*) noexcept> callback) noexcept;
	void SendResponseNoFree(CanMessageBuffer *buf) noexcept;
	void SendBroadcastNoFree(CanMessageBuffer *buf) noexcept;
	void SendMessageNoReplyNoFree(CanMessageBuffer *buf) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;
	CanMessageBuffer *AllocateBuffer(const GCodeBuffer *_ecv_null gb) THROWS(GCodeException);
	void CheckCanAddress(uint32_t address, const GCodeBuffer& gb) THROWS(GCodeException);

	uint16_t GetTimeStampCounter() noexcept;
	uint32_t Convert16bitReceivedTimeStampTo32bits(uint16_t ts) noexcept;

#if DUAL_CAN
	uint32_t SendPlainMessageNoFree(CanMessageBuffer *buf, uint32_t timeout = UsualSendTimeout) noexcept;
	bool ReceivePlainMessage(CanMessageBuffer *null buf, uint32_t timeout = UsualResponseTimeout) noexcept;
#endif

#if !SAME70
	uint16_t GetTimeStampPeriod() noexcept;					// return the period of the time stamp counter in units of 48MHz CAN clocks
#endif

	// Info functions
	GCodeResult GetRemoteFirmwareDetails(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult RemoteDiagnostics(MessageType mt, uint32_t boardAddress, unsigned int type, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult HandleM111(uint32_t boardAddress, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	// Motor control functions
	void SendMotion(CanMessageBuffer *buf) noexcept;
	GCodeResult EnableRemoteDrivers(const CanDriversList& drivers, const StringRef& reply) noexcept;
	void EnableRemoteDrivers(const CanDriversList& drivers) noexcept;
	GCodeResult DisableRemoteDrivers(const CanDriversList& drivers, const StringRef& reply) noexcept;
	void DisableRemoteDrivers(const CanDriversList& drivers) noexcept;
	void SetRemoteDriversIdle(const CanDriversList& drivers, float idleCurrentFactor) noexcept;
	GCodeResult SetRemoteStandstillCurrentPercent(const CanDriversData<float>& data, const StringRef& reply) noexcept;
	GCodeResult SetRemoteDriverCurrents(const CanDriversData<float>& data, const StringRef& reply) noexcept;
	GCodeResult SetRemotePressureAdvance(const CanDriversData<ShortPressureAdvanceParameters>& data, const StringRef& reply) noexcept;
	GCodeResult SetRemoteDriverStepsPerMmAndMicrostepping(const CanDriversData<StepsPerUnitAndMicrostepping>& data, const StringRef& reply) noexcept;
	GCodeResult ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) pre(driver.IsRemote());
	GCodeResult GetSetRemoteDriverStallParameters(const CanDriversList& drivers, GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf) THROWS(GCodeException);
	void EnableRemoteStallEndstop(DriverId did, float speed) THROWS(GCodeException) pre(did.IsRemote());
	void DisableRemoteStallEndstops(CanAddress boardId) noexcept;

#if 0	// not currently used
	unsigned int GetNumPendingMotionMessages() noexcept;
#endif
	void WakeAsyncSender() noexcept;
	void WakeAsyncSenderFromIsr() noexcept;

	// Remote handle functions
	GCodeResult CreateHandle(CanAddress boardAddress, RemoteInputHandle h, const char *_ecv_array pinName, uint16_t threshold, uint16_t minInterval, bool *_ecv_null currentState, const StringRef& reply) noexcept;
	GCodeResult DeleteHandle(CanAddress boardAddress, RemoteInputHandle h, const StringRef& reply) noexcept;
	GCodeResult GetHandlePinName(CanAddress boardAddress, RemoteInputHandle h, bool *_ecv_null currentState, const StringRef& reply) noexcept;
	GCodeResult EnableHandle(CanAddress boardAddress, RemoteInputHandle h, bool enable, bool *_ecv_null currentState, const StringRef& reply) noexcept;
	GCodeResult ChangeHandleResponseTime(CanAddress boardAddress, RemoteInputHandle h, uint32_t responseMillis, bool *_ecv_null currentState, const StringRef &reply) noexcept;
	GCodeResult ChangeHandleThreshold(CanAddress boardAddress, RemoteInputHandle h, int32_t threshold, bool *_ecv_null currentState, const StringRef &reply) noexcept;
	GCodeResult ChangeHandleSetTouchMode(CanAddress boardAddress, RemoteInputHandle h, uint32_t sensitivity, const StringRef &reply) noexcept;
	GCodeResult TareHandle(CanAddress boardAddress, RemoteInputHandle h, uint32_t mode, int32_t& baseline, const StringRef &reply) noexcept;
	GCodeResult SetHandleDriveLevel(CanAddress boardAddress, RemoteInputHandle h, uint32_t driveLevel, uint8_t &returnedDriveLevel, const StringRef &reply) noexcept;
	typedef void (*ReadHandlesCallbackFunction)(CallbackParameter param, RemoteInputHandle h, int32_t val) noexcept;
	GCodeResult ReadRemoteHandles(CanAddress boardAddress, RemoteInputHandle mask, RemoteInputHandle pattern, ReadHandlesCallbackFunction callback, CallbackParameter param, const StringRef &reply) noexcept;

	// Filament monitor functions
	GCodeResult CreateFilamentMonitor(DriverId driver, uint8_t type, const GCodeBuffer& gb, const StringRef& reply) noexcept;
	GCodeResult ConfigureFilamentMonitor(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult DeleteFilamentMonitor(DriverId driver, GCodeBuffer* gb, const StringRef& reply) noexcept;		// called from a destructor, so must not throw

	// Misc functions
	GCodeResult WriteGpio(CanAddress boardAddress, uint8_t portNumber, float pwm, bool isServo, const GCodeBuffer *gb, const StringRef& reply) noexcept;
	GCodeResult ChangeAddressAndNormalTiming(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult EnableCan(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
#if SUPPORT_ACCELEROMETERS
	GCodeResult StartAccelerometer(DriverId device, uint8_t axes, uint32_t numSamples, uint8_t mode, const GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
#endif
	GCodeResult StartClosedLoopDataCollection(DriverId device, uint16_t filter, uint16_t numSamples, uint16_t p_rateRequested, uint8_t p_movementRequested, uint8_t mode, const GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult ProcessM655(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	void UpdateStatusLed() noexcept;
#if SUPPORT_MULTICAST_DISCOVERY
	void SetStatusLedIdentify(uint32_t seconds) noexcept;
	void SetStatusLedNormal() noexcept;
#endif

#if DUAL_CAN
namespace ODrive {
	CanId ArbitrationId(DriverId driver, uint8_t cmd) noexcept;
	CanMessageBuffer *_ecv_null PrepareSimpleMessage(DriverId const driver, const StringRef& reply) noexcept;
	void FlushCanReceiveHardware() noexcept;
	bool GetExpectedSimpleMessage(CanMessageBuffer *buf, DriverId const driver, uint8_t const cmd, const StringRef& reply) noexcept;
}
#endif
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
