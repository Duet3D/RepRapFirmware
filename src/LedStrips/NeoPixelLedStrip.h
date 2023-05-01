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
	NeoPixelLedStrip(uint32_t p_freq) noexcept;

protected:
	bool IsNeopixel() const noexcept override { return true; }
};

#endif

#endif /* SRC_LEDSTRIPS_NEOPIXELLEDSTRIP_H_ */
