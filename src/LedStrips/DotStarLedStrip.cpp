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
	bool seen = false;
	GCodeResult rslt = CommonConfigure(gb, reply, pinName, seen);
	if (seen)
	{
		if (!UsesDma())
		{
			reply.copy("bit-banging not supported for DotStar LEDs");
			return GCodeResult::error;
		}
		// Nothing specific to configure for DotStar strips in Duet3D builds
		return rslt;
	}

	return CommonReportDetails(reply);
}

// Send a M150 command to this strip
GCodeResult DotStarLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

#endif

// End
