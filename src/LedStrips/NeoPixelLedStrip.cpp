/*
 * NeoPixelLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/NeoPixelLedStrip.h>

#if SUPPORT_LED_STRIPS

#include <Movement/StepTimer.h>

NeoPixelLedStrip::NeoPixelLedStrip(bool p_isRGBW) noexcept
	: LocalLedStrip((p_isRGBW) ? LedStripType::NeoPixel_RGBW : LedStripType::NeoPixel_RGB, DefaultNeoPixelSpiClockFrequency),
	  isRGBW(p_isRGBW)
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

#if SUPPORT_REMOTE_COMMANDS

GCodeResult NeoPixelLedStrip::Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept
{
	bool seen = false;
	GCodeResult rslt = CommonConfigure(parser, reply, seen, extra);
	if (seen)
	{
		// Nothing specific to configure for Neopixel strips in Duet3D builds
		return rslt;
	}

	return CommonReportDetails(reply);
}

GCodeResult NeoPixelLedStrip::HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
#if SUPPORT_DMA_NEOPIXEL
	if (DmaInProgress())													// if we are sending something
	{
		return GCodeResult::notFinished;
	}
#endif

	if (needStartDelay && StepTimer::GetTimerTicks() - whenTransferFinished < MinNeoPixelResetTicks)
	{
		return GCodeResult::notFinished;									// give the NeoPixels time to reset
	}

	LedParams params;
	params.GetM150Params(parser);
	params.ApplyBrightness();

#if SUPPORT_DMA_NEOPIXEL
	if (UsesDma())
	{
		SpiSendData(params);
	}
	else
#endif
	{
		BitBangData(params);
	}
	return GCodeResult::ok;
}

#endif

// Send a M150 command to this strip
GCodeResult NeoPixelLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
#if SUPPORT_DMA_NEOPIXEL
	if (DmaInProgress())													// if we are sending something
	{
		return GCodeResult::notFinished;
	}
#endif

	if (needStartDelay && StepTimer::GetTimerTicks() - whenTransferFinished < MinNeoPixelResetTicks)
	{
		return GCodeResult::notFinished;									// give the NeoPixels time to reset
	}

	LedParams params;
	params.GetM150Params(gb);
	params.ApplyBrightness();

#if SUPPORT_DMA_NEOPIXEL
	if (UsesDma())
	{
		SpiSendData(params);
	}
	else
#endif
	{
		BitBangData(params);
	}
	return GCodeResult::ok;
}

// Return the number of buffer bytes we need per LED
size_t NeoPixelLedStrip::GetBytesPerLed() const noexcept
{
	const size_t bytesPerLed = (isRGBW) ? 4 : 3;
	return (useDma) ? bytesPerLed * 4 : bytesPerLed;
}

#if SUPPORT_DMA_NEOPIXEL

// Encode one NeoPixel byte into the buffer.
// A 0 bit is encoded as 1000
// A 1 bit is encoded as 1110
// All encoding is MSB first
static void EncodeNeoPixelByte(uint8_t *p, uint8_t val) noexcept
{
	static constexpr uint8_t EncodedByte[4] = { 0b10001000, 0b10001110, 0b11101000, 0b11101110 };

# if USE_16BIT_SPI
	// Swap bytes for 16-bit DMA
	*p++ = EncodedByte[(val >> 4) & 3];
	*p++ = EncodedByte[val >> 6];
	*p++ = EncodedByte[val & 3];
	*p++ = EncodedByte[(val >> 2) & 3];
# else
	*p++ = EncodedByte[val >> 6];
	*p++ = EncodedByte[(val >> 4) & 3];
	*p++ = EncodedByte[(val >> 2) & 3];
	*p++ = EncodedByte[val & 3];
# endif
}

// Send data to NeoPixel LEDs by DMA to SPI
GCodeResult NeoPixelLedStrip::SpiSendData(const LedParams& params) noexcept
{
	const unsigned int bytesPerLed = (isRGBW) ? 16 : 12;
	unsigned int numLeds = params.numLeds;
	uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
	while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + chunkBufferSize)
	{
		EncodeNeoPixelByte(p, (uint8_t)params.green);
		p += 4;
		EncodeNeoPixelByte(p, (uint8_t)params.red);
		p += 4;
		EncodeNeoPixelByte(p, (uint8_t)params.blue);
		p += 4;
		if (isRGBW)
		{
			EncodeNeoPixelByte(p, (uint8_t)params.white);
			p += 4;
		}
		--numLeds;
		++numAlreadyInBuffer;
	}

	if (!params.following)
	{
		DmaSendChunkBuffer(bytesPerLed * numAlreadyInBuffer);		// send data by DMA to SPI
		numAlreadyInBuffer = 0;
		needStartDelay = true;
	}
	return GCodeResult::ok;
}
#endif

// Bit bang data to Neopixels. See https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/ for the timing requirements.
constexpr uint32_t NanosecondsToCycles(uint32_t ns) noexcept
{
	return (ns * (uint64_t)SystemCoreClockFreq)/1000000000u;
}

constexpr uint32_t T0H = NanosecondsToCycles(300);
constexpr uint32_t T1H = NanosecondsToCycles(700);
constexpr uint32_t TCY = NanosecondsToCycles(1200);

// Send data to NeoPixel LEDs by bit banging
#if SAME70
__attribute__((aligned(32)))			// needed to ensure consistent timing. SAME70 cache lines are 32 bytes each.
#elif SAME5x || SAM4E
__attribute__((aligned(16)))			// SAME5 and SAM4E cache lines are 16 bytes long
#endif
GCodeResult NeoPixelLedStrip::BitBangData(const LedParams& params) noexcept
{
	const unsigned int bytesPerLed = (isRGBW) ? 4 : 3;
	unsigned int numLeds = params.numLeds;
	uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
	while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + chunkBufferSize)
	{
		*p++ = (uint8_t)params.green;
		*p++ = (uint8_t)params.red;
		*p++ = (uint8_t)params.blue;
		if (isRGBW)
		{
			*p++ = (uint8_t)params.white;
		}
		--numLeds;
		++numAlreadyInBuffer;
	}

	if (!params.following)
	{
		const uint8_t *q = chunkBuffer;
		uint32_t nextDelay = TCY;
		const Pin pin = port.GetPin();
#if SAME70
		// We need to make sure that the critical parts of the following loop bodies fall within a single cache line, otherwise pulse timings sometimes go wrong
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
		asm volatile ("nop");
#endif
		IrqDisable();
		uint32_t lastTransitionTime = GetCurrentCycles();

		if (port.GetTotalInvert())
		{
			fastDigitalWriteHigh(pin);
			while (q < p)
			{
				uint8_t c = *q++;
				for (unsigned int i = 0; i < 8; ++i)
				{
					// The high-level time is critical, the low-level time is not.
					// On the SAME5x the high-level time easily gets extended too much, so do as little work as possible during that time.
					const uint32_t diff = ((c >> 7u) - 1u) & (T1H - T0H);
					uint32_t highTime = T1H - diff;
					lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay);
					fastDigitalWriteLow(pin);
					lastTransitionTime = DelayCycles(lastTransitionTime, highTime);
					fastDigitalWriteHigh(pin);
#if SAME70
					__ISB();					// this is to prevent the compiler re-ordering instructions
#endif
					nextDelay = TCY - highTime;
					c <<= 1;
				}
			}
		}
		else
		{
			fastDigitalWriteLow(pin);			// this is needed on Duet 2 at least, to prevent the first pulse being too long
			while (q < p)
			{
				uint8_t c = *q++;
				for (unsigned int i = 0; i < 8; ++i)
				{
					// The high-level time is critical, the low-level time is not.
					// On the SAME5x the high-level time easily gets extended too much, so do as little work as possible during that time.
					const uint32_t diff = ((c >> 7u) - 1u) & (T1H - T0H);
					uint32_t highTime = T1H - diff;
					lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay);
					fastDigitalWriteHigh(pin);
					lastTransitionTime = DelayCycles(lastTransitionTime, highTime);
					fastDigitalWriteLow(pin);
#if SAME70
					__ISB();					// this is to prevent the compiler re-ordering instructions
#endif
					nextDelay = TCY - highTime;
					c <<= 1;
				}
			}
		}
		IrqEnable();

		numAlreadyInBuffer = 0;
		whenTransferFinished = StepTimer::GetTimerTicks();
		needStartDelay = true;
	}
	return GCodeResult::ok;
}

#endif

// End
