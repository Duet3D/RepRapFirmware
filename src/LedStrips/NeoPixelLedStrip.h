/*
 * NeoPixelLedStrip.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_NEOPIXELLEDSTRIP_H_
#define SRC_LEDSTRIPS_NEOPIXELLEDSTRIP_H_

#include "LocalLedStrip.h"

#if SUPPORT_LED_STRIPS

class NeoPixelLedStrip : public LocalLedStrip
{
public:
	NeoPixelLedStrip(bool p_isRGBW) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException) override;
	GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) override;
	bool IsBitBanged() const noexcept override;

protected:
	bool IsNeopixel() const noexcept override { return true; }

	static constexpr uint32_t DefaultNeoPixelSpiClockFrequency = 2500000;		// must be between about 2MHz and about 4MHz

	bool isRGBW;
};

#endif

#endif /* SRC_LEDSTRIPS_NEOPIXELLEDSTRIP_H_ */
