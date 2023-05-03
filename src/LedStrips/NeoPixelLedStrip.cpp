/*
 * NeoPixelLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/NeoPixelLedStrip.h>

#if SUPPORT_LED_STRIPS

NeoPixelLedStrip::NeoPixelLedStrip(bool p_isRGBW) noexcept
	: LocalLedStrip((p_isRGBW) ? LedStripType::NeoPixel_RGBW : LedStripType::NeoPixel_RGB, DefaultNeoPixelSpiClockFrequency), isRGBW(p_isRGBW)
{
}

// Configure or report on this strip
GCodeResult NeoPixelLedStrip::Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException)
{
	bool seen = false;
	GCodeResult rslt = CommonConfigure(gb, reply, pinName, seen);
	if (seen)
	{
		// Nothing specific to configure for Neopixel strips in Duet3D builds
		return rslt;
	}

	return CommonReportDetails(reply);
}

// Send a M150 command to this strip
GCodeResult NeoPixelLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

bool NeoPixelLedStrip::IsBitBanged() const noexcept
{
	//TODO
	return true;
}

#endif

// End
