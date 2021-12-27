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

	RemoteZProbe(unsigned int num, CanAddress bn, ZProbeType p_type) noexcept : ZProbe(num, p_type), boardAddress(bn), state(false) { }
	~RemoteZProbe() noexcept override;

	void SetIREmitter(bool on) const noexcept override { }
	uint16_t GetRawReading() const noexcept override;
	bool SetProbing(bool isProbing) noexcept override;
	GCodeResult AppendPinNames(const StringRef& str) noexcept override;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException) override;
	GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept override;
	void HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool newState) noexcept override;

	GCodeResult Create(const StringRef& pinNames, const StringRef& reply) noexcept;

private:
	CanAddress boardAddress;
	RemoteInputHandle handle;
	bool state;

	static constexpr uint16_t ActiveProbeReportInterval = 2;
	static constexpr uint16_t InactiveProbeReportInterval = 25;
};

#endif

#endif /* SRC_ENDSTOPS_REMOTEZPROBE_H_ */
