/*
 * LocalZProbe.h
 *
 *  Created on: 14 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_LOCALZPROBE_H_
#define SRC_ENDSTOPS_LOCALZPROBE_H_

#include "ZProbe.h"
#include <SoftTimer.h>

class LocalZProbe final : public ZProbe
{
public:
	void* operator new(size_t sz) { return Allocate<LocalZProbe>(); }
	void operator delete(void* p) { Release<LocalZProbe>(p); }

	LocalZProbe(unsigned int num) : ZProbe(num, ZProbeType::none) { }
	~LocalZProbe() override;
	void SetIREmitter(bool on) const override;
	uint16_t GetRawReading() const override;
	void SetProbing(bool isProbing) const override;
	GCodeResult AppendPinNames(const StringRef& str) const override;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) override;
	GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) override;

private:
	IoPort inputPort;
	IoPort modulationPort;			// the modulation port we are using

	// Variable for programming Smart Effector and other programmable Z probes
	static bool TimerInterrupt(CallbackParameter param, uint32_t& when);
	bool Interrupt(uint32_t& when);

	SoftTimer timer;
	uint8_t progBytes[MaxZProbeProgramBytes];
	size_t numBytes;
	size_t bytesSent;
	unsigned int bitsSent;
	SoftTimer::Ticks bitTime;
	SoftTimer::Ticks startTime;

	static const unsigned int bitsPerSecond = 1000;
};

// If this is a dumb modulated IR probe, set the IR LED on or off. Called from the tick ISR, so inlined for speed.
inline void LocalZProbe::SetIREmitter(bool on) const
{
	if (type == ZProbeType::dumbModulated)
	{
		modulationPort.WriteDigital(on);
	}
}

#endif /* SRC_ENDSTOPS_LOCALZPROBE_H_ */
