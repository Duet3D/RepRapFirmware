/*
 * DotStarLedStrip.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_DOTSTARLEDSTRIP_H_
#define SRC_LEDSTRIPS_DOTSTARLEDSTRIP_H_

#include "LocalLedStrip.h"

#if SUPPORT_LED_STRIPS && SUPPORT_DMA_DOTSTAR

class DotStarLedStrip : public LocalLedStrip
{
public:
	DotStarLedStrip() noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException) override;
	GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) override;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept override;
	GCodeResult HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
#endif

protected:
	size_t GetBytesPerLed() const noexcept override;

private:
	GCodeResult SendDotStarData(uint32_t data, uint32_t numLeds, bool following) noexcept;

	static constexpr uint32_t DefaultDotStarSpiClockFrequency = 1000000;	// 1MHz default

	unsigned int numRemaining = 0;											// how much of the current request remains after the current transfer
	unsigned int totalSent = 0;												// total amount of data sent since the start frame
	bool needStartFrame = true;
};

#endif

#endif /* SRC_LEDSTRIPS_DOTSTARLEDSTRIP_H_ */
