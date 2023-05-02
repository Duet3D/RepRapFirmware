/*
 * DotStarLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/DotStarLedStrip.h>

#if SUPPORT_LED_STRIPS && SUPPORT_DMA_DOTSTAR

DotStarLedStrip::DotStarLedStrip() noexcept
	: LocalLedStrip(LedStripType::DotStar, DefaultDotStarSpiClockFrequency)
{
}

// Configure this strip
GCodeResult DotStarLedStrip::Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

// Send a M150 command to this strip
GCodeResult DotStarLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

#endif

// End
