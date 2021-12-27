/*
 * LocalZProbe.h
 *
 *  Created on: 14 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_LOCALZPROBE_H_
#define SRC_ENDSTOPS_LOCALZPROBE_H_

#include "ZProbe.h"
#include <Movement/StepTimer.h>

class LocalZProbe final : public ZProbe
{
public:
	DECLARE_FREELIST_NEW_DELETE(LocalZProbe)

	LocalZProbe(unsigned int num) noexcept : ZProbe(num, ZProbeType::none) { }
	~LocalZProbe() noexcept override;

	void SetIREmitter(bool on) const noexcept override;
	uint16_t GetRawReading() const noexcept override;
	bool SetProbing(bool isProbing) noexcept override;
	GCodeResult AppendPinNames(const StringRef& str) noexcept override;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException) override;
	GCodeResult SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply) noexcept override;

#if ALLOCATE_DEFAULT_PORTS
	bool AssignPorts(const char *pinNames, const StringRef& reply) noexcept;
#endif

private:
	IoPort inputPort;
	IoPort modulationPort;			// the modulation port we are using

	// Variable for programming Smart Effector and other programmable Z probes
	static void TimerInterrupt(CallbackParameter param) noexcept;
	void Interrupt() noexcept;

	StepTimer timer;
	uint8_t progBytes[MaxZProbeProgramBytes];
	size_t numBytes;
	size_t bytesSent;
	unsigned int bitsSent;
	StepTimer::Ticks bitTime;
	StepTimer::Ticks startTime;

	static const unsigned int bitsPerSecond = 1000;
};

// If this is a dumb modulated IR probe, set the IR LED on or off. Called from the tick ISR, so inlined for speed.
inline void LocalZProbe::SetIREmitter(bool on) const noexcept
{
	if (type == ZProbeType::dumbModulated)
	{
		modulationPort.WriteDigital(on);
	}
}

#endif /* SRC_ENDSTOPS_LOCALZPROBE_H_ */
