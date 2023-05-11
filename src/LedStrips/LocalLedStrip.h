/*
 * LocalLedStrip.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_LOCALLEDSTRIP_H_
#define SRC_LEDSTRIPS_LOCALLEDSTRIP_H_

#include "LedStripBase.h"

#if SUPPORT_LED_STRIPS

#if defined(DUET3_MB6HC) || defined(DUET3_MB6XD)
constexpr size_t DmaBufferSize = 240 * 16;						// DotStar LEDs use 4 bytes/LED, NeoPixel RGBW use 16 bytes/LED
#endif

#if SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

// We need an SPI port and DMA
# if LEDSTRIP_USES_USART
#  include <pdc/pdc.h>
#  include <pmc/pmc.h>
#  include <usart/usart.h>
# else
#  include <DmacManager.h>
#  if SAME5x
#   include <Hardware/IoPorts.h>
#   include <hri_mclk_e54.h>
#  elif SAME70
#   include <xdmac/xdmac.h>
#   include <pmc/pmc.h>
#  endif
# endif

# if SAME70
#  define USE_16BIT_SPI	1		// set to use 16-bit SPI transfers instead of 8-bit
# else
#  define USE_16BIT_SPI	0		// set to use 16-bit SPI transfers instead of 8-bit
# endif

# if USE_16BIT_SPI && LEDSTRIP_USES_USART
#  error Invalid combination
# endif

#endif	// SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

class LocalLedStrip : public LedStripBase
{
public:
	LocalLedStrip(LedStripType p_type, uint32_t p_freq) noexcept;
	~LocalLedStrip() override;

protected:
	DECLARE_OBJECT_MODEL

	struct LedParams
	{
		uint32_t red;
		uint32_t green;
		uint32_t blue;
		uint32_t white;
		uint32_t brightness;
		uint32_t numLeds;
		bool following;

		void GetM150Params(GCodeBuffer& gb) THROWS(GCodeException);
		void ApplyBrightness() noexcept;
#if SUPPORT_REMOTE_COMMANDS
		void GetM150Params(CanMessageGenericParser& parser) noexcept;
#endif
	};

	GCodeResult CommonConfigure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName, bool& seen) THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult CommonConfigure(CanMessageGenericParser& parser, const StringRef& reply, bool& seen, uint8_t& extra) noexcept;
#endif

	GCodeResult CommonReportDetails(const StringRef& reply) noexcept;
	bool MustStopMovement() const noexcept override { return !useDma; }
	virtual size_t GetBytesPerLed() const noexcept = 0;

#if SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR
	bool UsesDma() const noexcept { return useDma; }
	void DmaSendChunkBuffer(size_t numBytes) noexcept;					// DMA the data. Must be a multiple of 2 bytes if USE_16BIT_SPI is true.
	bool DmaInProgress() noexcept;										// Return true if DMA to the LEDs is in progress
	void SetupSpi() noexcept;											// Setup the SPI peripheral. Only call this when the busy flag is not set.
#endif

	IoPort port;
	uint32_t frequency;													// the SPI frequency we are using
	uint32_t whenTransferFinished = 0;									// the time in step clocks when we determined that the data transfer had finished

#if SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR
	bool useDma;
	bool dmaBusy = false;												// true if DMA was started and is not known to have finished
#else
	static constexpr bool useDma = false;
#endif

	uint32_t maxLeds = DefaultMaxNumLeds;
	size_t chunkBufferSize = 0;											// the size of the allocated buffer
	uint8_t *chunkBuffer = nullptr;										// pointer to 32-bit aligned buffer for holding the data to send

#if SAME70
	alignas(4) static __nocache uint8_t dmaBuffer[DmaBufferSize];		// buffer for sending data to LEDs by DMA on SAME7x processors
#endif

private:
	GCodeResult AllocateChunkBuffer(const StringRef& reply) noexcept;

	static constexpr size_t DefaultMaxNumLeds = 60;
};

#endif

#endif /* SRC_LEDSTRIPS_LOCALLEDSTRIP_H_ */
