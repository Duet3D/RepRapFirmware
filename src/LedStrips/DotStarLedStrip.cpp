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

#if SUPPORT_REMOTE_COMMANDS

GCodeResult DotStarLedStrip::Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept
{
	bool seen = false;
	GCodeResult rslt = CommonConfigure(parser, reply, seen, extra);
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

GCodeResult DotStarLedStrip::HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	if (DmaInProgress())													// if we are sending something
	{
		return GCodeResult::notFinished;
	}

	LedParams params;
	params.GetM150Params(parser);

	// If there are no LEDs to set, we have finished unless we need to send a start frame to DotStar LEDs
	if (params.numLeds == 0 && !needStartFrame && !params.following)
	{
		return GCodeResult::ok;
	}
	if (numRemaining != 0)
	{
		params.numLeds = numRemaining;
	}

# if USE_16BIT_SPI
	// Swap bytes for 16-bit SPI
	const uint32_t data = ((params.brightness & 0xF8) << 5) | (0xE0 << 8) | ((params.blue & 255)) | ((params.green & 255) << 24) | ((params.red & 255) << 16);
# else
	const uint32_t data = (params.brightness >> 3) | 0xE0 | ((params.blue & 255) << 8) | ((params.green & 255) << 16) | ((params.red & 255) << 24);
# endif
	return SendDotStarData(data, params.numLeds, params.following);
}

#endif

// Send a M150 command to this strip
GCodeResult DotStarLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	if (DmaInProgress())													// if we are sending something
	{
		return GCodeResult::notFinished;
	}

	LedParams params;
	params.GetM150Params(gb);

	// If there are no LEDs to set, we have finished unless we need to send a start frame to DotStar LEDs
	if (params.numLeds == 0 && !needStartFrame && !params.following)
	{
		return GCodeResult::ok;
	}
	if (numRemaining != 0)
	{
		params.numLeds = numRemaining;
	}

# if USE_16BIT_SPI
	// Swap bytes for 16-bit SPI
	const uint32_t data = ((params.brightness & 0xF8) << 5) | (0xE0 << 8) | ((params.blue & 255)) | ((params.green & 255) << 24) | ((params.red & 255) << 16);
# else
	const uint32_t data = (params.brightness >> 3) | 0xE0 | ((params.blue & 255) << 8) | ((params.green & 255) << 16) | ((params.red & 255) << 24);
# endif
	return SendDotStarData(data, params.numLeds, params.following);
}

// Return the number of buffer bytes we need per LED
size_t DotStarLedStrip::GetBytesPerLed() const noexcept
{
	return 4;					// we need to send 32 bits per LED
}

// Send data to DotStar LEDs
GCodeResult DotStarLedStrip::SendDotStarData(uint32_t data, uint32_t numLeds, bool following) noexcept
{
	// Set up the data in the DMA buffer.
	// Sending at least 32 zero bits (start frame) tells the LEDs that this is new data starting with the first LED in the strip.
	// The first 1 bit indicates the start of the data frame for the first LED.
	// There is a half-bit delay in each LED before the data is shifted out to the next LED. This means that we need to send at least an extra N/2 bits of data,
	// where N is the number of LEDs in the strip. The datasheet says to send 32 bits of 1 but this is only sufficient for up to 64 LEDs. Sending 1s can lead to a spurious
	// white LED at the end if we don't provide data for all the LEDs. So instead we send 32 or more bits of zeros.
	// See https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/ for more.

	const size_t MaxLedsPerBuffer = chunkBufferSize/4;
	unsigned int spaceLeft = MaxLedsPerBuffer;
	uint32_t *p = reinterpret_cast<uint32_t*>(chunkBuffer);
	if (needStartFrame)
	{
		*p++ = 0;															// start frame
		--spaceLeft;														// one less slot available for data
		totalSent = 0;
	}

	// Can we fit the remaining data and stop bits in the buffer?
	unsigned int numStopWordsNeeded = (following) ? 0 : min<unsigned int>((numLeds + totalSent + 63)/64, MaxLedsPerBuffer - 1);
	unsigned int thisChunk;
	if (numLeds + numStopWordsNeeded <= spaceLeft)
	{
		thisChunk = numLeds;
	}
	else
	{
		thisChunk = min<unsigned int>(spaceLeft, numLeds - 1);
		numStopWordsNeeded = 0;
	}

	numRemaining = numLeds - thisChunk;
	totalSent += thisChunk;
	needStartFrame = (numRemaining == 0 && !following);

	for (unsigned int i = 0; i < thisChunk; ++i)
	{
		// According to the Adafruit web site, current production uses the order BGR
		*p++ = data;														// LED frame
	}

	for (unsigned int i = 0; i < numStopWordsNeeded; ++i)
	{
		*p++ = 0;															// append some stop bits
	}

	DmaSendChunkBuffer(4 * (p - reinterpret_cast<uint32_t*>(chunkBuffer)));
	return (numRemaining == 0) ? GCodeResult::ok : GCodeResult::notFinished;
}
#endif

// End
