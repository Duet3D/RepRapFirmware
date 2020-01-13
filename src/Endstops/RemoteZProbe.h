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

class RemoteZProbe final : public ZProbe
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<RemoteZProbe>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<RemoteZProbe>(p); }

	RemoteZProbe(unsigned int num, CanAddress bn, ZProbeType p_type) noexcept : ZProbe(num, p_type), boardAddress(bn) { }
	~RemoteZProbe() noexcept override;
	void SetIREmitter(bool on) const noexcept override { }
	uint16_t GetRawReading() const noexcept override { return 0; }
	void SetProbing(bool isProbing) const noexcept override;
	GCodeResult AppendPinNames(const StringRef& str) const noexcept override;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS_GCODE_EXCEPTION override;
	GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept override;

	GCodeResult Create(const StringRef& pinNames, const StringRef& reply) noexcept;

private:
	CanAddress boardAddress;
};

#endif

#endif /* SRC_ENDSTOPS_REMOTEZPROBE_H_ */
