/*
 * DotStarLedStrip.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_DOTSTARLEDSTRIP_H_
#define SRC_LEDSTRIPS_DOTSTARLEDSTRIP_H_

#include "LocalLedStrip.h"

#if SUPPORT_LED_STRIPS

class DotStarLedStrip : public LocalLedStrip
{
public:
	DotStarLedStrip(uint32_t p_freq) noexcept;

protected:
	bool IsNeopixel() const noexcept override { return false; }
};

#endif

#endif /* SRC_LEDSTRIPS_DOTSTARLEDSTRIP_H_ */
