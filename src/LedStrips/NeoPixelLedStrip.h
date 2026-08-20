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
	explicit NeoPixelLedStrip(bool p_isRGBW) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array _ecv_null pinName) THROWS(GCodeException) override;
	GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) override;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept override;
	GCodeResult HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
#endif

protected:
	size_t GetBytesPerLed() const noexcept override;

private:
	static constexpr uint32_t DefaultNeoPixelSpiClockFrequency = 3200000;				// each bit is encoded as 4 SPI bits, so T0H/T1L are one bit time and T1H/T0L are three
																						// and only 3.16-3.33MHz keeps all four inside the WS2812B windows, with 3.2MHz centring them
	static constexpr uint32_t MinNeoPixelResetTicks = (250 * StepClockRate)/1000000;	// 250us minimum Neopixel reset time on later chips

	bool IsRGBW() const noexcept { return type == LedStripType::NeoPixel_RGBW; }

	GCodeResult NeoPixelSendData(LedParams& params) noexcept;

	GCodeResult BitBangData(const LedParams& params) noexcept;
#if SUPPORT_DMA_NEOPIXEL
	GCodeResult SpiSendData(const LedParams& params) noexcept;
#endif

	unsigned int numAlreadyInBuffer = 0;												// number of pixels already store in the buffer
	bool needStartDelay = true;
};

#endif

#endif /* SRC_LEDSTRIPS_NEOPIXELLEDSTRIP_H_ */
