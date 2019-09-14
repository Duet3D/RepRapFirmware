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
	void* operator new(size_t sz) { return Allocate<RemoteZProbe>(); }
	void operator delete(void* p) { Release<RemoteZProbe>(p); }

	RemoteZProbe(unsigned int num, CanAddress bn) : ZProbe(num, ZProbeType::none), boardAddress(bn) { }
	~RemoteZProbe() override;
	void SetIREmitter(bool on) const override { }
	uint16_t GetRawReading() const override { return 0; }
	void SetProbing(bool isProbing) const override;
	GCodeResult AppendPinNames(const StringRef& str) const override;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) override;
	GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) override;

	GCodeResult Create(const StringRef& pinNames, const StringRef& reply);

private:
	CanAddress boardAddress;
};

#endif

#endif /* SRC_ENDSTOPS_REMOTEZPROBE_H_ */
