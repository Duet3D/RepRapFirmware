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
#include <RepRap.h>
#include <GCodes/GCodes.h>

#if DOTSTAR_USES_USART
# include <sam/drivers/pdc/pdc.h>
# include <sam/drivers/usart/usart.h>
#else
# include <DmacManager.h>
# if SAME5x
#  include <Hardware/IoPorts.h>
# elif SAME70
#  include <sam/drivers/xdmac/xdmac.h>
# endif
#endif

#if SAME70
# define USE_16BIT_SPI	1		// set to use 16-bit SPI transfers instead of 8-bit
#else
# define USE_16BIT_SPI	0		// set to use 16-bit SPI transfers instead of 8-bit
#endif

#if USE_16BIT_SPI && DOTSTAR_USES_USART
# error Invalid combination
#endif

// Duet 5 Mini only supports NeoPixel, not DotStar. So the DotStar code is dead in the Duet 3 Mini build.

namespace DotStarLed
{
	constexpr uint32_t DefaultDotStarSpiClockFrequency = 100000;		// try sending at 100kHz
	constexpr uint32_t DefaultNeoPixelSpiClockFrequency = 2500000;		// must be between about 2MHz and about 4MHz
	constexpr uint32_t DefaultSpiFrequencies[2] = { DefaultDotStarSpiClockFrequency, DefaultNeoPixelSpiClockFrequency };
	constexpr uint32_t MinNeoPixelResetTicks = (50 * StepTimer::StepClockRate)/1000000;		// 50us minimum Neopixel reset time

	constexpr size_t ChunkBufferSize = 720;								// the size of our DMA buffer. DotStar LEDs use 4 bytes/LED, NeoPixels use 12 bytes/LED.
	constexpr unsigned int MaxDotStarChunkSize = ChunkBufferSize/4;		// maximum number of DotStarLEDs we DMA to in one go. Most strips have 30 LEDs/metre.
	constexpr unsigned int MaxNeoPixelChunkSize = ChunkBufferSize/12;	// maximum number of NeoPixels we can support. A full ring contains 60.

	static uint32_t ledType = 1;										// 0 = DotStar (not supported on Duet 3 Mini), 1 = NeoPixel, 2 = NeoPixel on Mini 12864 display (Duet 3 Mini only)
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
		usartPdc->PERIPH_TPR = reinterpret_cast<uint32_t>(chunkBuffer);
		usartPdc->PERIPH_TCR = numBytes;										// number of bytes to transfer
		usartPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;								// enable the PDC to send data
		DotStarUsart->US_CR = US_CR_TXEN;										// enable transmitter
#elif SAME5x
		//TODO use 16-bit mode to make the DMA more efficient, but that probably requires us to swap alternate bytes in the buffer
		DmacManager::DisableChannel(DmacChanDotStarTx);
		DmacManager::SetTriggerSource(DmacChanDotStarTx, DmaTrigSource::qspi_tx);
# if USE_16BIT_SPI
		DmacManager::SetBtctrl(DmacChanDotStarTx, DMAC_BTCTRL_STEPSIZE_X2 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_NOACT);
# else
		DmacManager::SetBtctrl(DmacChanDotStarTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_NOACT);
# endif
		DmacManager::SetSourceAddress(DmacChanDotStarTx, chunkBuffer);
		DmacManager::SetDestinationAddress(DmacChanDotStarTx, &QSPI->TXDATA.reg);
		DmacManager::SetDataLength(DmacChanDotStarTx, numBytes);				// must do this last!
		DmacManager::EnableChannel(DmacChanDotStarTx, DmacPrioDotStar);
#elif SAME70
		xdmac_channel_disable(XDMAC, DmacChanDotStarTx);
		xdmac_channel_config_t p_cfg = {0, 0, 0, 0, 0, 0, 0, 0};
		p_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
						| XDMAC_CC_MBSIZE_SINGLE
						| XDMAC_CC_DSYNC_MEM2PER
						| XDMAC_CC_CSIZE_CHK_1
# if USE_16BIT_SPI
						| XDMAC_CC_DWIDTH_HALFWORD
# else
						| XDMAC_CC_DWIDTH_BYTE
# endif
						| XDMAC_CC_SIF_AHB_IF0
						| XDMAC_CC_DIF_AHB_IF1
						| XDMAC_CC_SAM_INCREMENTED_AM
						| XDMAC_CC_DAM_FIXED_AM
						| XDMAC_CC_PERID((uint32_t)DmaTrigSource::qspitx);
# if USE_16BIT_SPI
		p_cfg.mbr_ubc = numBytes/2;			//TODO is this correct? The datasheet isn't clear.
#else
		p_cfg.mbr_ubc = numBytes;
#endif
		p_cfg.mbr_sa = reinterpret_cast<uint32_t>(chunkBuffer);
		p_cfg.mbr_da = reinterpret_cast<uint32_t>(&(QSPI->QSPI_TDR));
		xdmac_configure_transfer(XDMAC, DmacChanDotStarTx, &p_cfg);
		xdmac_channel_enable(XDMAC, DmacChanDotStarTx);
#else
# error Unsupported processor
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
#elif SAME5x
			if ((DmacManager::GetAndClearChannelStatus(DmacChanDotStarTx) & DMAC_CHINTFLAG_TCMPL) != 0)
#elif SAME70
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
		DotStarUsart->US_BRGR = SystemPeripheralClock()/frequency;				// set SPI clock frequency
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
#elif SAME5x
		// DotStar on Duet 3 Mini uses the QSPI peripheral
		QSPI->CTRLA.reg = QSPI_CTRLA_SWRST;										// software reset
# if USE_16BIT_SPI
		QSPI->CTRLB.reg = QSPI_CTRLB_DATALEN_16BITS;							// SPI mode, 8 bits per transfer
# else
		QSPI->CTRLB.reg = QSPI_CTRLB_DATALEN_8BITS;								// SPI mode, 8 bits per transfer
# endif
		QSPI->BAUD.reg = QSPI_BAUD_CPOL | QSPI_BAUD_CPHA | QSPI_BAUD_BAUD(SystemCoreClockFreq/frequency - 1);
		QSPI->CTRLA.reg = QSPI_CTRLA_ENABLE;
#elif SAME70
		// DotStar on Duet 3 uses the QSPI peripheral
		QSPI->QSPI_CR = QSPI_CR_SWRST;
# if USE_16BIT_SPI
		QSPI->QSPI_MR = QSPI_MR_NBBITS_16_BIT;									// SPI mode, 16 bits per transfer
# else
		QSPI->QSPI_MR = QSPI_MR_NBBITS_8_BIT;									// SPI mode, 8 bits per transfer
# endif
		QSPI->QSPI_SCR = QSPI_SCR_CPOL | QSPI_SCR_CPHA | QSPI_SCR_SCBR(SystemPeripheralClock()/frequency - 1);
		QSPI->QSPI_CR = QSPI_CR_QSPIEN;
#endif
	}

#ifndef DUET_5LC
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
#endif

	// Encode one NeoPixel byte into the buffer.
	// A 0 bit is encoded as 1000
	// A 1 bit is encoded as 1110
	// All encoding is MSB first
	static void EncodeNeoPixelByte(uint8_t *p, uint8_t val)
	{
		static constexpr uint8_t EncodedByte[4] = { 0b10001000, 0b10001110, 0b11101000, 0b11101110 };

#if USE_16BIT_SPI
		if (ledType == 1)
		{
			// Swap bytes for 16-bit DMA
			*p++ = EncodedByte[(val >> 4) & 3];
			*p++ = EncodedByte[val >> 6];
			*p++ = EncodedByte[val & 3];
			*p++ = EncodedByte[(val >> 2) & 3];
		}
		else
#endif
		{
			*p++ = EncodedByte[val >> 6];
			*p++ = EncodedByte[(val >> 4) & 3];
			*p++ = EncodedByte[(val >> 2) & 3];
			*p++ = EncodedByte[val & 3];
		}
	}

	// Send data to NeoPixel LEDs by DMA to SPI
	static GCodeResult SpiSendNeoPixelData(uint8_t red, uint8_t green, uint8_t blue, uint32_t numLeds, bool following) noexcept
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
			DmaSendChunkBuffer(12 * numAlreadyInBuffer);			// send data by DMA to SPI
			numAlreadyInBuffer = 0;
		}
		return GCodeResult::ok;
	}

#ifdef DUET_5LC
	constexpr uint32_t NanosecondsToCycles(uint32_t ns) noexcept
	{
		return (ns * (uint64_t)SystemCoreClockFreq)/1000000000u;
	}

	constexpr uint32_t T0H = NanosecondsToCycles(350);
	constexpr uint32_t T0L = NanosecondsToCycles(850);
	constexpr uint32_t T1H = NanosecondsToCycles(800);
	constexpr uint32_t T1L = NanosecondsToCycles(475);

	// Send data to NeoPixel LEDs by bit banging
	static GCodeResult BitBangNeoPixelData(uint8_t red, uint8_t green, uint8_t blue, uint32_t numLeds, bool following) noexcept
	{
		uint8_t *p = chunkBuffer + (3 * numAlreadyInBuffer);
		while (numLeds != 0 && p <= chunkBuffer + ARRAY_SIZE(chunkBuffer) - 3)
		{
			*p++ = green;
			*p++ = red;
			*p++ = blue;
			--numLeds;
			++numAlreadyInBuffer;
		}

		if (!following)
		{
			const uint8_t *q = chunkBuffer;
			uint32_t nextDelay = T0L;
			cpu_irq_disable();
			uint32_t lastTransitionTime = SysTick->VAL & 0x00FFFFFF;
			while (q < p)
			{
				uint8_t c = *q++;
				for (unsigned int i = 0; i < 8; ++i)
				{
					if (c & 0x80)
					{
						lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay);
						fastDigitalWriteHigh(LcdNeopixelOutPin);
						lastTransitionTime = DelayCycles(lastTransitionTime, T1H);
						fastDigitalWriteLow(LcdNeopixelOutPin);
						nextDelay = T1L;
					}
					else
					{
						lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay);
						fastDigitalWriteHigh(LcdNeopixelOutPin);
						lastTransitionTime = DelayCycles(lastTransitionTime, T0H);
						fastDigitalWriteLow(LcdNeopixelOutPin);
						nextDelay = T0L;
					}
					c <<= 1;
				}
			}
			cpu_irq_enable();
			numAlreadyInBuffer = 0;
			whenDmaFinished = StepTimer::GetTimerTicks();
		}
		return GCodeResult::ok;
	}
#endif
}

void DotStarLed::Init() noexcept
{
#if SAME5x
	SetPinFunction(NeopixelOutPin, NeopixelOutPinFunction);
	hri_mclk_set_AHBMASK_QSPI_bit(MCLK);
	hri_mclk_clear_AHBMASK_QSPI_2X_bit(MCLK);			// we don't need the 2x clock
	hri_mclk_set_APBCMASK_QSPI_bit(MCLK);
#else
	// Set up the USART or QSPI pins for SPI mode. The pins are already set up for SPI in the pins table
	ConfigurePin(DotStarMosiPin);
	ConfigurePin(DotStarSclkPin);

	// Enable the clock to the USART or SPI peripheral
	pmc_enable_periph_clk(DotStarClockId);
#endif

	SetupSpi(DefaultSpiFrequencies[ledType]);

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

	if (needStartFrame && (ledType == 1 || ledType == 2) && StepTimer::GetTimerTicks() - whenDmaFinished < MinNeoPixelResetTicks)
	{
		return GCodeResult::notFinished;									// give the NeoPixels time to reset
	}

	bool seen = false;

	// Deal with changing the LED type first
	if (gb.Seen('X'))
	{
		seen = true;
		const uint32_t newType = gb.GetLimitedUIValue('X',
#ifdef DUET_5LC
				3, 1														// types 1 and 2 are supported
#else
				2, 0														// types 0 and 1 are supported
#endif
			);
		const bool typeChanged = (newType != ledType);

		if (newType != 2)
		{
			bool setFrequency = typeChanged;

			uint32_t frequency = DefaultSpiFrequencies[newType];
			gb.TryGetUIValue('Q', frequency, setFrequency);
			if (setFrequency)
			{
				SetupSpi(frequency);
			}
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
#ifdef DUET_5LC
			else if (ledType == 2)
			{
				// Set the data output low to start a WS2812 reset sequence
				IoPort::SetPinMode(LcdNeopixelOutPin, PinMode::OUTPUT_LOW);
				whenDmaFinished = StepTimer::GetTimerTicks();
				return GCodeResult::notFinished;
			}
#endif
		}
	}

	if (ledType == 2)
	{
		// Interrupts are disabled while bit-banging the data, so make sure movement has stopped
		if (!reprap.GetGCodes().LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
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
#ifndef DUET_5LC
	case 0:	// DotStar
		{
#if USE_16BIT_SPI
			// Swap bytes for 16-bit SPI
			const uint32_t data = ((brightness >> 11) | (0xE0 << 8)) | ((blue & 255)) | ((green & 255) << 24) | ((red & 255) << 16);
#else
			const uint32_t data = ((brightness >> 3) | 0xE0) | ((blue & 255) << 8) | ((green & 255) << 16) | ((red & 255) << 24);
#endif
			return SendDotStarData(data, numLeds, following);
		}
#endif

	case 1:	// NeoPixel
		// Scale RGB by the brightness
		return SpiSendNeoPixelData(	(uint8_t)((red * brightness + 255) >> 8),
									(uint8_t)((green * brightness + 255) >> 8),
									(uint8_t)((blue * brightness + 255) >> 8),
									numLeds, following
							      );

#ifdef DUET_5LC
	case 2:
		// Scale RGB by the brightness
		return BitBangNeoPixelData(	(uint8_t)((red * brightness + 255) >> 8),
									(uint8_t)((green * brightness + 255) >> 8),
									(uint8_t)((blue * brightness + 255) >> 8),
									numLeds, following
							      );
#endif
	}
	return GCodeResult::ok;													// should never get here
}

#endif

// End
