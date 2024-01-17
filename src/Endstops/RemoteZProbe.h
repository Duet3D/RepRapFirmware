/*
 * RemoteZProbe.h
 *
 *  Created on: 14 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_REMOTEZPROBE_H_
#define SRC_ENDSTOPS_REMOTEZPROBE_H_

#include "ZProbe.h"

#if SUPPORT_CAN_EXPANSION

#include <RemoteInputHandle.h>

class RemoteZProbe final : public ZProbe
{
public:
	DECLARE_FREELIST_NEW_DELETE(RemoteZProbe)

	RemoteZProbe(unsigned int num, CanAddress bn, ZProbeType p_type) noexcept : ZProbe(num, p_type), boardAddress(bn), lastValue(0), state(false) { }
	~RemoteZProbe() noexcept override;

	uint32_t GetRawReading() const noexcept override;
	bool SetProbing(bool isProbing) noexcept override;
	GCodeResult AppendPinNames(const StringRef& str) noexcept override;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException) override;
	GCodeResult Create(const StringRef& pinNames, const StringRef& reply) noexcept;
	void HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool newState, uint32_t reading) noexcept override;
	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply) override THROWS(GCodeException);

	// Process a remote reading that relates to this Z probe
	void UpdateRemoteReading(CanAddress src, uint8_t handleMinor, uint32_t reading) noexcept override;

	// Functions used only with modulated Z probes
	void SetIREmitter(bool on) const noexcept override { }

	// Functions used only with programmable Z probes
	GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept override;

	// Functions used only with scanning Z probes
	GCodeResult GetCalibratedReading(float& val) const noexcept override;
	GCodeResult CalibrateDriveLevel(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) override;
	float GetLatestHeight() const noexcept override;

	void ScanningProbeCallback(RemoteInputHandle h, uint32_t val) noexcept;

private:
	CanAddress boardAddress;
	RemoteInputHandle handle;
	uint32_t lastValue;							// the most recent value received from a scanning analog Z probe
	bool state;									// the state of a digital Z probe

	static constexpr uint32_t ActiveProbeReportInterval = 2;
	static constexpr uint32_t InactiveProbeReportInterval = 25;
};

#endif

#endif /* SRC_ENDSTOPS_REMOTEZPROBE_H_ */
