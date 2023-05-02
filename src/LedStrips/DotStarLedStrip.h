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
	bool IsBitBanged() const noexcept override { return false; }				// currently we only support DMA-driven DotStar strips

protected:
	bool IsNeopixel() const noexcept override { return false; }

	static constexpr uint32_t DefaultDotStarSpiClockFrequency = 1000000;		// 1MHz default
};

#endif

#endif /* SRC_LEDSTRIPS_DOTSTARLEDSTRIP_H_ */
