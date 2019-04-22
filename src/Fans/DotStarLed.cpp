/*
 * DotStarLed.cpp
 *
 *  Created on: 18 Jul 2018
 *      Author: David
 */

#include "DotStarLed.h"

#if SUPPORT_DOTSTAR_LED

#include "GCodes/GCodeBuffer.h"
#include "sam/drivers/pdc/pdc.h"
#include "sam/drivers/usart/usart.h"

namespace DotStarLed
{
	const unsigned int MaxChunkSize = 30;					// maximum number of LEDs we DMA to in one go (most strips have 30 LEDs/metre)

	static unsigned int numRemaining = 0;					// how much of the current request remains after the current transfer
	static unsigned int totalSent = 0;						// total amount of data sent since the start frame
	static bool needStartFrame = true;						// true if we need to send a start frame with the next command
	static bool busy = false;
	static uint32_t chunkBuffer[MaxChunkSize];

	void Init()
	{
		// Set up the USART pins for SPI mode
		// The pins are already set up for SPI in the pins table
		ConfigurePin(DotStarMosiPin);
		ConfigurePin(DotStarSclkPin);

		// Enable the clock to the USART
		pmc_enable_periph_clk(DotStarUsartId);

		// Set the USART in SPI mode, with the clock high when inactive, data changing on the falling edge of the clock
		DotStarUsart->US_IDR = ~0u;
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
		DotStarUsart->US_MR = US_MR_USART_MODE_SPI_MASTER
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_CHMODE_NORMAL
						| US_MR_CPOL
						| US_MR_CLKO;
		DotStarUsart->US_BRGR = VARIANT_MCK/DotStarSpiClockFrequency;		// set SPI clock frequency
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

		// Initialise variables
		numRemaining = totalSent = 0;
		needStartFrame = true;
		busy = false;
	}

	GCodeResult SetColours(GCodeBuffer& gb, const StringRef& reply)
	{
		if (busy)											// if we sent something
		{
			if ((DotStarUsart->US_CSR & US_CSR_ENDTX) == 0)	// if we are still sending
			{
				return GCodeResult::notFinished;
			}
			busy = false;									// we finished the last transfer
		}

		bool seen = false;
		uint32_t red = 0, green = 0, blue = 0, brightness = 16, numLeds = MaxChunkSize, following = 0;
		gb.TryGetUIValue('R', red, seen);
		gb.TryGetUIValue('U', green, seen);
		gb.TryGetUIValue('B', blue, seen);
		gb.TryGetUIValue('Y', brightness, seen);
		gb.TryGetUIValue('S', numLeds, seen);
		gb.TryGetUIValue('F', following, seen);
		if (!seen || (numLeds == 0 && !needStartFrame && !following))
		{
			return GCodeResult::ok;
		}

		if (numRemaining != 0)
		{
			numLeds = numRemaining;												// we have come back to do another chunk
		}

		// Set up the data in the DMA buffer.
		// Sending at least 32 zero bits (start frame) tells the LEDs that this is new data starting with the first LED in the strip.
		// The first 1 bit indicates the start of the data frame for the first LED.
		// There is a half-bit delay in each LED before the data is shifted out to the next LED. This means that we need to send at least an extra N/2 bits of data,
		// where N is the number of LEDs in the strip. The datasheet says to send 32 bits of 1 but this is only sufficient for up to 64 LEDs. Sending 1s can lead to a spurious
		// white LED at the end if we don't provide data for all the LEDs. So instead we send 32 or more bits of zeros.
		// See https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/ for more.
		unsigned int spaceLeft = MaxChunkSize;
		uint32_t *p = chunkBuffer;
		if (needStartFrame)
		{
			*p++ = 0;															// start frame
			--spaceLeft;														// one less slot available for data
			totalSent = 0;
		}

		// Can we fit the remaining data and stop bits in the buffer?
		unsigned int numStopWordsNeeded = (following) ? 0 : min<unsigned int>((numLeds + totalSent + 63)/64, MaxChunkSize - 1);
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
		needStartFrame = (numRemaining == 0 && following == 0);

		for (unsigned int i = 0; i < thisChunk; ++i)
		{
			// According to the Adafruit web site, current production uses the order BGR
			*p++ = (brightness & 31) | 0xE0 | ((blue & 255) << 8) | ((green & 255) << 16) | ((red & 255) << 24);	// LED frame
		}

		for (unsigned int i = 0; i < numStopWordsNeeded; ++i)
		{
			*p++ = 0;															// append some stop bits
		}

		// DMA the data
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_TXDIS;			// reset transmitter and receiver, disable transmitter
		Pdc * const usartPdc = usart_get_pdc_base(DotStarUsart);
		usartPdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS;		// disable the PDC
		usartPdc->PERIPH_TPR = reinterpret_cast<uint32_t>(&chunkBuffer);
		usartPdc->PERIPH_TCR = 4 * (p - chunkBuffer);							// number of bytes to transfer
		usartPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;								// enable the PDC to send data

		DotStarUsart->US_CR = US_CR_TXEN;										// enable transmitter

		busy = true;
		return (numRemaining == 0) ? GCodeResult::ok : GCodeResult::notFinished;
	}
};

#endif

// End
