/*
 * DotStarLed.cpp
 *
 *  Created on: 18 Jul 2018
 *      Author: David
 */

#include "DotStarLed.h"

#if SUPPORT_DOTSTAR_LED

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/StepTimer.h>

#if DOTSTAR_USES_USART
# include <sam/drivers/pdc/pdc.h>
# include <sam/drivers/usart/usart.h>
#else
# include <Hardware/DmacManager.h>
#endif

namespace DotStarLed
{
	constexpr uint32_t DefaultDotStarSpiClockFrequency = 100000;		// try sending at 100kHz
	constexpr uint32_t DefaultNeoPixelSpiClockFrequency = 2500000;		// must be between about 2MHz and about 4MHz
	constexpr uint32_t DefaultSpiFrequencies[2] = { DefaultDotStarSpiClockFrequency, DefaultNeoPixelSpiClockFrequency };
	constexpr uint32_t MinNeoPixelResetTicks = (50 * StepTimer::StepClockRate)/1000000;		// 50us minimum Neopixel reset time

	constexpr size_t ChunkBufferSize = 720;								// the size of our DMA buffer. DotStar LEDs use 4 bytes/LED, NeoPixels use 12 bytes/LED.
	constexpr unsigned int MaxDotStarChunkSize = ChunkBufferSize/4;		// maximum number of DotStarLEDs we DMA to in one go. Most strips have 30 LEDs/metre.
	constexpr unsigned int MaxNeoPixelChunkSize = ChunkBufferSize/12;	// maximum number of NeoPixels we can support. A full ring contains 60.

	static uint32_t ledType = 0;										// 0 = DotStar, 1 = NeoPixel
	static uint32_t whenDmaFinished = 0;								// the time in step clocks when we determined that the DMA had finished
	static unsigned int numRemaining = 0;								// how much of the current request remains after the current transfer (DotStar only)
	static unsigned int totalSent = 0;									// total amount of data sent since the start frame (DotStar only)
	static unsigned int numAlreadyInBuffer = 0;							// number of pixels already store in the buffer (NeoPixel only)
	static bool needStartFrame = true;									// true if we need to send a start frame with the next command
	static bool busy = false;											// true if DMA was started and is not known to have finished
	alignas(4) static uint8_t chunkBuffer[ChunkBufferSize];				// buffer for sending data to LEDs

	// DMA the data
	static void DmaSendChunkBuffer(size_t numBytes) noexcept
	{
#if DOTSTAR_USES_USART

		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_TXDIS;			// reset transmitter and receiver, disable transmitter
		Pdc * const usartPdc = usart_get_pdc_base(DotStarUsart);
		usartPdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS;		// disable the PDC
		usartPdc->PERIPH_TPR = reinterpret_cast<uint32_t>(&chunkBuffer);
		usartPdc->PERIPH_TCR = numBytes;										// number of bytes to transfer
		usartPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;								// enable the PDC to send data

		DotStarUsart->US_CR = US_CR_TXEN;										// enable transmitter
#else
		xdmac_channel_disable(XDMAC, DmacChanDotStarTx);
		xdmac_channel_config_t p_cfg = {0, 0, 0, 0, 0, 0, 0, 0};
		p_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
						| XDMAC_CC_MBSIZE_SINGLE
						| XDMAC_CC_DSYNC_MEM2PER
						| XDMAC_CC_CSIZE_CHK_1
						| XDMAC_CC_DWIDTH_BYTE
						| XDMAC_CC_SIF_AHB_IF0
						| XDMAC_CC_DIF_AHB_IF1
						| XDMAC_CC_SAM_INCREMENTED_AM
						| XDMAC_CC_DAM_FIXED_AM
						| XDMAC_CC_PERID((uint32_t)DmaTrigSource::qspitx);
		p_cfg.mbr_ubc = numBytes;
		p_cfg.mbr_sa = reinterpret_cast<uint32_t>(chunkBuffer);
		p_cfg.mbr_da = reinterpret_cast<uint32_t>(&(QSPI->QSPI_TDR));
		xdmac_configure_transfer(XDMAC, DmacChanDotStarTx, &p_cfg);
		xdmac_channel_enable(XDMAC, DmacChanDotStarTx);
#endif
		busy = true;
	}

	// Return true if DMA to the LEDs is in progress
	static bool DmaInProgress() noexcept
	{
		if (busy)																// if we sent something
		{
#if DOTSTAR_USES_USART
			if ((DotStarUsart->US_CSR & US_CSR_ENDTX) != 0)						// if we are no longer sending
#else
			if ((xdmac_channel_get_interrupt_status(XDMAC, DmacChanDotStarTx) & XDMAC_CIS_BIS) != 0)	// if the last transfer has finished
#endif
			{
				busy = false;													// we finished the last transfer
				whenDmaFinished = StepTimer::GetTimerTicks();
			}
		}
		return busy;
	}

	// Setup the SPI peripheral. Only call this when the busy flag is not set.
	static void SetupSpi(uint32_t frequency) noexcept
	{
#if DOTSTAR_USES_USART
		// Set the USART in SPI mode, with the clock high when inactive, data changing on the falling edge of the clock
		DotStarUsart->US_IDR = ~0u;
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
		DotStarUsart->US_MR = US_MR_USART_MODE_SPI_MASTER
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_CHMODE_NORMAL
						| US_MR_CPOL
						| US_MR_CLKO;
		DotStarUsart->US_BRGR = SystemPeripheralClock()/frequency;		// set SPI clock frequency
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
#else
		// DotStar on Duet 3 uses the QSPI peripheral
		QSPI->QSPI_CR = QSPI_CR_SWRST;
		QSPI->QSPI_MR = 0;														// SPI mode, 8 bits per transfer
		QSPI->QSPI_SCR = QSPI_SCR_CPOL | QSPI_SCR_CPHA | QSPI_SCR_SCBR(SystemPeripheralClock()/frequency - 1);
		QSPI->QSPI_CR = QSPI_CR_QSPIEN;
#endif
	}

	// Send data to DotStar LEDs
	static GCodeResult SendDotStarData(uint32_t data, uint32_t numLeds, bool following) noexcept
	{
		// Set up the data in the DMA buffer.
		// Sending at least 32 zero bits (start frame) tells the LEDs that this is new data starting with the first LED in the strip.
		// The first 1 bit indicates the start of the data frame for the first LED.
		// There is a half-bit delay in each LED before the data is shifted out to the next LED. This means that we need to send at least an extra N/2 bits of data,
		// where N is the number of LEDs in the strip. The datasheet says to send 32 bits of 1 but this is only sufficient for up to 64 LEDs. Sending 1s can lead to a spurious
		// white LED at the end if we don't provide data for all the LEDs. So instead we send 32 or more bits of zeros.
		// See https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/ for more.
		unsigned int spaceLeft = MaxDotStarChunkSize;
		uint32_t *p = reinterpret_cast<uint32_t*>(chunkBuffer);
		if (needStartFrame)
		{
			*p++ = 0;															// start frame
			--spaceLeft;														// one less slot available for data
			totalSent = 0;
		}

		// Can we fit the remaining data and stop bits in the buffer?
		unsigned int numStopWordsNeeded = (following) ? 0 : min<unsigned int>((numLeds + totalSent + 63)/64, MaxDotStarChunkSize - 1);
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

	// Encode one NeoPixel byte into the buffer.
	// A 0 bit is encoded as 1000
	// A 1 bit is encoded as 1110
	// All encoding is MSB first
	static void EncodeNeoPixelByte(uint8_t *p, uint8_t val)
	{
		static constexpr uint8_t EncodedByte[4] = { 0b10001000, 0b10001110, 0b11101000, 0b11101110 };

		*p++ = EncodedByte[val >> 6];
		*p++ = EncodedByte[(val >> 4) & 3];
		*p++ = EncodedByte[(val >> 2) & 3];
		*p++ = EncodedByte[val & 3];
	}

	// Send data to NeoPixel LEDs
	static GCodeResult SendNeoPixelData(uint8_t red, uint8_t green, uint8_t blue, uint32_t numLeds, bool following) noexcept
	{
		uint8_t *p = chunkBuffer + (12 * numAlreadyInBuffer);
		while (numLeds != 0 && p <= chunkBuffer + ARRAY_SIZE(chunkBuffer) - 12)
		{
			EncodeNeoPixelByte(p, green);
			p += 4;
			EncodeNeoPixelByte(p, red);
			p += 4;
			EncodeNeoPixelByte(p, blue);
			p += 4;
			--numLeds;
			++numAlreadyInBuffer;
		}
		if (!following)
		{
			DmaSendChunkBuffer(12 * numAlreadyInBuffer);
			numAlreadyInBuffer = 0;
		}
		return GCodeResult::ok;
	}
}

void DotStarLed::Init() noexcept
{
	// Set up the USART or QSPI pins for SPI mode. The pins are already set up for SPI in the pins table
	ConfigurePin(DotStarMosiPin);
	ConfigurePin(DotStarSclkPin);

	// Enable the clock to the USART or SPI peripheral
	pmc_enable_periph_clk(DotStarClockId);

	SetupSpi(DefaultDotStarSpiClockFrequency);

	// Initialise variables
	numRemaining = totalSent = 0;
	needStartFrame = true;
	busy = false;
}

// This function handles M150
// For DotStar LEDs:
// 	We can handle an unlimited length LED strip, because we can send the data in multiple chunks.
//	So whenever we receive a m150 command, we send the data immediately, in multiple chunks if our DMA buffer is too small to send it as a single chunk.
//	To send multiple chunks, we process the command once per chunk, using numRemaining to keep track of how many more LEDs need to be written to
// For NeoPixel LEDs:
//	If there is a gap or more then about 9us in transmission, the string will reset and the next command will be taken as applying to the start of the strip.
//  Therefore we need to DMA the data for all LEDs in one go. So the maximum strip length is limited by the size of our DMA buffer.
//	We buffer up incoming data until we get a command with the Following parameter missing or set to zero, then we DMA it all.
GCodeResult DotStarLed::SetColours(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (DmaInProgress())													// if we are sending something
	{
		return GCodeResult::notFinished;
	}

	if (needStartFrame && ledType == 1 && StepTimer::GetTimerTicks() - whenDmaFinished < MinNeoPixelResetTicks)
	{
		return GCodeResult::notFinished;									// give the NeoPixels time to reset
	}

	bool seen = false;

	// Deal with changing the LED type first
	if (gb.Seen('X'))
	{
		seen = true;
		const uint32_t newType = gb.GetLimitedUIValue('X', 2);				// only types 0 and 1 are supported
		const bool typeChanged = (newType != ledType);
		bool setFrequency = typeChanged;

		uint32_t frequency = DefaultSpiFrequencies[newType];
		gb.TryGetUIValue('Q', frequency, setFrequency);
		if (setFrequency)
		{
			SetupSpi(frequency);
		}

		if (typeChanged)
		{
			ledType = newType;
			numRemaining = totalSent = numAlreadyInBuffer = 0;
			needStartFrame = true;

			if (ledType == 1)
			{
				// Set the SPI data output low to start a WS2812 reset sequence
				chunkBuffer[0] = 0;
				DmaSendChunkBuffer(1);
				return GCodeResult::notFinished;
			}
		}
	}

	// Get the RGB and brightness values
	uint32_t red = 0, green = 0, blue = 0, brightness = 128, numLeds = MaxDotStarChunkSize;
	bool following = false;
	gb.TryGetLimitedUIValue('R', red, seen, 256);
	gb.TryGetLimitedUIValue('U', green, seen, 256);
	gb.TryGetLimitedUIValue('B', blue, seen, 256);
	if (gb.Seen('P'))
	{
		brightness = gb.GetLimitedUIValue('P', 256);						// valid P values are 0-255
	}
	else if (gb.Seen('Y'))
	{
		brightness = gb.GetLimitedUIValue('Y',  32) << 3;					// valid Y values are 0-31
	}
	gb.TryGetUIValue('S', numLeds, seen);
	gb.TryGetBValue('F', following, seen);

	if (!seen || (numLeds == 0 && !needStartFrame && !following))
	{
		return GCodeResult::ok;
	}

	if (numRemaining != 0)
	{
		numLeds = numRemaining;												// we have come back to do another chunk (applies to DotStar only)
	}

	switch (ledType)
	{
	case 0:	// DotStar
		{
			const uint32_t data = (brightness >> 3) | 0xE0 | ((blue & 255) << 8) | ((green & 255) << 16) | ((red & 255) << 24);
			return SendDotStarData(data, numLeds, following);
		}

	case 1:	// NeoPixel
		{
			// Scale RGB by the brightness
			return SendNeoPixelData(	(uint8_t)((red * brightness + 255) >> 8),
										(uint8_t)((green * brightness + 255) >> 8),
										(uint8_t)((blue * brightness + 255) >> 8),
										numLeds, following
								   );
		}
	}
	return GCodeResult::ok;													// should never get here
}

#endif

// End
