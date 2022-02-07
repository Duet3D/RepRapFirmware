/*
 * DotStarLed.cpp
 *
 *  Created on: 18 Jul 2018
 *      Author: David
 */

#include "LedStripDriver.h"

#if SUPPORT_LED_STRIPS

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/StepTimer.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

// Define which types of LED strip this hardware supports
#define SUPPORT_DMA_NEOPIXEL		(defined(DUET3_MB6HC) || defined(DUET3_MB6XD) || defined(DUET3MINI) || defined(PCCB_10))
#define SUPPORT_DMA_DOTSTAR			(defined(DUET3_MB6HC) || defined(DUET3_MB6XD) || defined(PCCB_10))
#define SUPPORT_BITBANG_NEOPIXEL	(defined(DUET3MINI_V04) || defined(DUET_NG))

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

namespace LedStripDriver
{
	constexpr uint32_t DefaultDotStarSpiClockFrequency = 1000000;		// 1MHz default
	constexpr uint32_t DefaultNeoPixelSpiClockFrequency = 2500000;		// must be between about 2MHz and about 4MHz
	constexpr uint32_t MinNeoPixelResetTicks = (250 * StepClockRate)/1000000;	// 250us minimum Neopixel reset time on later chips

	// Define the size of the buffer used to accumulate a sequence of colours to send to the string
#if defined(DUET3_MB6HC) || defined(DUET3_MB6XD)
	// We have plenty of non-cached RAM left on Duet 3
	constexpr size_t ChunkBufferSize = 240 * 16;						// DotStar LEDs use 4 bytes/LED, NeoPixel RGBW use 16 bytes/LED
#elif defined(DUET3MINI)
	constexpr size_t ChunkBufferSize = 80 * 16;							// NeoPixel RGBW use 16 bytes/LED (increased to 80 LEDs for Justin)
#elif defined(DUET_NG)
	constexpr size_t ChunkBufferSize = 80 * 3;							// NeoPixel RGB use 3 bytes/LED
#else
	constexpr size_t ChunkBufferSize = 60 * 16;							// DotStar LEDs use 4 bytes/LED, NeoPixel RGBW use 16 bytes/LED
#endif

	enum class LedType : unsigned int
	{
		dotstar = 0,
		neopixelRGB,
		neopixelRGBBitBang,
		neopixelRGBW,
		neopixelRGBWBitBang
	};

	// In the following we set the text for the unavailable LED types to null, both to save flash memory and so we can test whether a type is supported
	constexpr const char *LedTypeNames[] =
	{
#if SUPPORT_DMA_DOTSTAR
		"DotStar on LED port",
#else
		nullptr,
#endif
#if SUPPORT_DMA_NEOPIXEL
		"NeoPixel RGB on LED port",
#else
		nullptr,
#endif
#if SUPPORT_BITBANG_NEOPIXEL
		"NeoPixel RGB bit-banged",
#else
		nullptr,
#endif
#if SUPPORT_DMA_NEOPIXEL
		"NeoPixel RGBW on LED port",
#else
		nullptr,
#endif
#if SUPPORT_BITBANG_NEOPIXEL
		"NeoPixel RGBW bit-banged"
#else
		nullptr
#endif
	};

#if SUPPORT_DMA_NEOPIXEL
	constexpr auto DefaultLedType = LedType::neopixelRGB;
#elif SUPPORT_BITBANG_NEOPIXEL
	constexpr auto DefaultLedType = LedType::neopixelRGBBitBang;
#endif

	static uint32_t currentFrequency;									// the SPI frequency we are using
	static LedType ledType = DefaultLedType;							// the type of LED strip currently configured
	static uint32_t whenDmaFinished = 0;								// the time in step clocks when we determined that the DMA had finished
	static unsigned int numRemaining = 0;								// how much of the current request remains after the current transfer (DotStar only)
	static unsigned int totalSent = 0;									// total amount of data sent since the start frame (DotStar only)
	static unsigned int numAlreadyInBuffer = 0;							// number of pixels already store in the buffer (NeoPixel only)
	static bool needStartFrame;											// true if we need to send a start frame with the next command
	static bool busy;													// true if DMA was started and is not known to have finished
	static bool needInit;

#if SAME70
	alignas(4) static __nocache uint8_t chunkBuffer[ChunkBufferSize];	// buffer for sending data to LEDs
#else
	alignas(4) static uint8_t chunkBuffer[ChunkBufferSize];				// buffer for sending data to LEDs
#endif

	static size_t MaxLedsPerBuffer() noexcept
	{
		switch (ledType)
		{
		case LedType::dotstar:
		case LedType::neopixelRGBWBitBang:
			return ChunkBufferSize/4;

		case LedType::neopixelRGBW:
			return ChunkBufferSize/16;

		case LedType::neopixelRGBBitBang:
			return ChunkBufferSize/3;

		case LedType::neopixelRGB:
		default:
			return ChunkBufferSize/12;
		}
	}

#if SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

	// DMA the data. Must be a multiple of 2 bytes if USE_16BIT_SPI is true.
	static void DmaSendChunkBuffer(size_t numBytes) noexcept
	{
# if LEDSTRIP_USES_USART
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_TXDIS;			// reset transmitter and receiver, disable transmitter
		Pdc * const usartPdc = usart_get_pdc_base(DotStarUsart);
		usartPdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS;		// disable the PDC
		usartPdc->PERIPH_TPR = reinterpret_cast<uint32_t>(chunkBuffer);
		usartPdc->PERIPH_TCR = numBytes;										// number of bytes to transfer
		usartPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;								// enable the PDC to send data
		DotStarUsart->US_CR = US_CR_TXEN;										// enable transmitter
# elif SAME5x
		DmacManager::DisableChannel(DmacChanDotStarTx);
		DmacManager::SetTriggerSource(DmacChanDotStarTx, DmaTrigSource::qspi_tx);
#  if USE_16BIT_SPI
		DmacManager::SetBtctrl(DmacChanDotStarTx, DMAC_BTCTRL_STEPSIZE_X2 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_NOACT);
#  else
		DmacManager::SetBtctrl(DmacChanDotStarTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_BYTE | DMAC_BTCTRL_BLOCKACT_NOACT);
#  endif
		DmacManager::SetSourceAddress(DmacChanDotStarTx, chunkBuffer);
		DmacManager::SetDestinationAddress(DmacChanDotStarTx, &QSPI->TXDATA.reg);
		DmacManager::SetDataLength(DmacChanDotStarTx, numBytes);				// must do this last!
		DmacManager::EnableChannel(DmacChanDotStarTx, DmacPrioDotStar);
# elif SAME70
		xdmac_channel_disable(XDMAC, DmacChanDotStarTx);
		xdmac_channel_config_t p_cfg = {0, 0, 0, 0, 0, 0, 0, 0};
		p_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
						| XDMAC_CC_MBSIZE_SINGLE
						| XDMAC_CC_DSYNC_MEM2PER
						| XDMAC_CC_CSIZE_CHK_1
#  if USE_16BIT_SPI
						| XDMAC_CC_DWIDTH_HALFWORD
#  else
						| XDMAC_CC_DWIDTH_BYTE
#  endif
						| XDMAC_CC_SIF_AHB_IF0
						| XDMAC_CC_DIF_AHB_IF1
						| XDMAC_CC_SAM_INCREMENTED_AM
						| XDMAC_CC_DAM_FIXED_AM
						| XDMAC_CC_PERID((uint32_t)DmaTrigSource::qspitx);
#  if USE_16BIT_SPI
		p_cfg.mbr_ubc = numBytes/2;
#  else
		p_cfg.mbr_ubc = numBytes;
#  endif
		p_cfg.mbr_sa = reinterpret_cast<uint32_t>(chunkBuffer);
		p_cfg.mbr_da = reinterpret_cast<uint32_t>(&(QSPI->QSPI_TDR));
		xdmac_configure_transfer(XDMAC, DmacChanDotStarTx, &p_cfg);
		xdmac_channel_enable(XDMAC, DmacChanDotStarTx);
# else
#  error Unsupported processor
# endif
		busy = true;
	}

	// Return true if DMA to the LEDs is in progress
	static bool DmaInProgress() noexcept
	{
		if (busy)																// if we sent something
		{
# if LEDSTRIP_USES_USART
			if ((DotStarUsart->US_CSR & US_CSR_ENDTX) != 0)						// if we are no longer sending
# elif SAME5x
			if ((DmacManager::GetAndClearChannelStatus(DmacChanDotStarTx) & DMAC_CHINTFLAG_TCMPL) != 0)
# elif SAME70
			if ((xdmac_channel_get_interrupt_status(XDMAC, DmacChanDotStarTx) & XDMAC_CIS_BIS) != 0)	// if the last transfer has finished
# endif
			{
				busy = false;													// we finished the last transfer
				whenDmaFinished = StepTimer::GetTimerTicks();
			}
		}
		return busy;
	}

	// Setup the SPI peripheral. Only call this when the busy flag is not set.
	static void SetupSpi() noexcept
	{
# if LEDSTRIP_USES_USART
		// Set the USART in SPI mode, with the clock high when inactive, data changing on the falling edge of the clock
		DotStarUsart->US_IDR = ~0u;
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
		DotStarUsart->US_MR = US_MR_USART_MODE_SPI_MASTER
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_CHMODE_NORMAL
						| US_MR_CPOL
						| US_MR_CLKO;
		DotStarUsart->US_BRGR = SystemPeripheralClock()/currentFrequency;				// set SPI clock frequency
		DotStarUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
# elif SAME5x
		// DotStar on Duet 3 Mini uses the QSPI peripheral
		QSPI->CTRLA.reg = QSPI_CTRLA_SWRST;										// software reset
#  if USE_16BIT_SPI
		QSPI->CTRLB.reg = QSPI_CTRLB_DATALEN_16BITS;							// SPI mode, 8 bits per transfer
#  else
		QSPI->CTRLB.reg = QSPI_CTRLB_DATALEN_8BITS;								// SPI mode, 8 bits per transfer
#  endif
		QSPI->BAUD.reg = QSPI_BAUD_CPOL | QSPI_BAUD_CPHA | QSPI_BAUD_BAUD(SystemCoreClockFreq/currentFrequency - 1);
		QSPI->CTRLA.reg = QSPI_CTRLA_ENABLE;
# elif SAME70
		// DotStar on Duet 3 uses the QSPI peripheral
		QSPI->QSPI_CR = QSPI_CR_SWRST;
#  if USE_16BIT_SPI
		QSPI->QSPI_MR = QSPI_MR_NBBITS_16_BIT;									// SPI mode, 16 bits per transfer
#  else
		QSPI->QSPI_MR = QSPI_MR_NBBITS_8_BIT;									// SPI mode, 8 bits per transfer
#  endif
		QSPI->QSPI_SCR = QSPI_SCR_CPOL | QSPI_SCR_CPHA | QSPI_SCR_SCBR(SystemPeripheralClock()/currentFrequency - 1);
		QSPI->QSPI_CR = QSPI_CR_QSPIEN;
		if (ledType == LedType::neopixelRGB || ledType == LedType::neopixelRGBW)
		{
			QSPI->QSPI_TDR = 0;													// send a word of zeros to set the data line low
		}
# endif
	}

#endif	// SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

#if SUPPORT_DMA_DOTSTAR
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
		unsigned int spaceLeft = MaxLedsPerBuffer();
		uint32_t *p = reinterpret_cast<uint32_t*>(chunkBuffer);
		if (needStartFrame)
		{
			*p++ = 0;															// start frame
			--spaceLeft;														// one less slot available for data
			totalSent = 0;
		}

		// Can we fit the remaining data and stop bits in the buffer?
		unsigned int numStopWordsNeeded = (following) ? 0 : min<unsigned int>((numLeds + totalSent + 63)/64, MaxLedsPerBuffer() - 1);
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

#if SUPPORT_DMA_NEOPIXEL
	// Encode one NeoPixel byte into the buffer.
	// A 0 bit is encoded as 1000
	// A 1 bit is encoded as 1110
	// All encoding is MSB first
	static void EncodeNeoPixelByte(uint8_t *p, uint8_t val)
	{
		static constexpr uint8_t EncodedByte[4] = { 0b10001000, 0b10001110, 0b11101000, 0b11101110 };

# if USE_16BIT_SPI
		if (ledType == LedType::neopixelRGB || ledType == LedType::neopixelRGBW)
		{
			// Swap bytes for 16-bit DMA
			*p++ = EncodedByte[(val >> 4) & 3];
			*p++ = EncodedByte[val >> 6];
			*p++ = EncodedByte[val & 3];
			*p++ = EncodedByte[(val >> 2) & 3];
		}
		else
# endif
		{
			*p++ = EncodedByte[val >> 6];
			*p++ = EncodedByte[(val >> 4) & 3];
			*p++ = EncodedByte[(val >> 2) & 3];
			*p++ = EncodedByte[val & 3];
		}
	}

	// Send data to NeoPixel LEDs by DMA to SPI
	static GCodeResult SpiSendNeoPixelData(uint8_t red, uint8_t green, uint8_t blue, uint8_t white, uint32_t numLeds, bool includeWhite, bool following) noexcept
	{
		const unsigned int bytesPerLed = (includeWhite) ? 16 : 12;
		uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
		while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + ARRAY_SIZE(chunkBuffer))
		{
			EncodeNeoPixelByte(p, green);
			p += 4;
			EncodeNeoPixelByte(p, red);
			p += 4;
			EncodeNeoPixelByte(p, blue);
			p += 4;
			if (includeWhite)
			{
				EncodeNeoPixelByte(p, white);
				p += 4;
			}
			--numLeds;
			++numAlreadyInBuffer;
		}

		if (!following)
		{
			DmaSendChunkBuffer(bytesPerLed * numAlreadyInBuffer);		// send data by DMA to SPI
			numAlreadyInBuffer = 0;
			needStartFrame = true;
		}
		return GCodeResult::ok;
	}
#endif

#if SUPPORT_BITBANG_NEOPIXEL
	constexpr uint32_t NanosecondsToCycles(uint32_t ns) noexcept
	{
		return (ns * (uint64_t)SystemCoreClockFreq)/1000000000u;
	}

	constexpr uint32_t T0H = NanosecondsToCycles(350);
	constexpr uint32_t T0L = NanosecondsToCycles(850);
	constexpr uint32_t T1H = NanosecondsToCycles(800);
	constexpr uint32_t T1L = NanosecondsToCycles(475);

	// Send data to NeoPixel LEDs by bit banging
	static GCodeResult BitBangNeoPixelData(uint8_t red, uint8_t green, uint8_t blue, uint8_t white, uint32_t numLeds, bool rgbw, bool following) noexcept
	{
		const unsigned int bytesPerLed = (rgbw) ? 4 : 3;
		uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
		while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + ARRAY_SIZE(chunkBuffer))
		{
			*p++ = green;
			*p++ = red;
			*p++ = blue;
			if (rgbw)
			{
				*p++ = white;
			}
			--numLeds;
			++numAlreadyInBuffer;
		}

		if (!following)
		{
			const uint8_t *q = chunkBuffer;
			uint32_t nextDelay = T0L;
			IrqDisable();
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
			IrqEnable();
			numAlreadyInBuffer = 0;
			whenDmaFinished = StepTimer::GetTimerTicks();
		}
		return GCodeResult::ok;
	}
#endif

}	// end namespace

void LedStripDriver::Init() noexcept
{
#if defined(DUET3MINI)
	SetPinFunction(NeopixelOutPin, NeopixelOutPinFunction);
	hri_mclk_set_AHBMASK_QSPI_bit(MCLK);
	hri_mclk_clear_AHBMASK_QSPI_2X_bit(MCLK);			// we don't need the 2x clock
	hri_mclk_set_APBCMASK_QSPI_bit(MCLK);
#elif defined(DUET3_MB6HC) || defined(DUET3_MB6XD) || defined(PCCB_10)
	SetPinFunction(DotStarMosiPin, DotStarPinMode);
	SetPinFunction(DotStarSclkPin, DotStarPinMode);
	pmc_enable_periph_clk(DotStarClockId);				// enable the clock to the USART or SPI peripheral
#endif

	currentFrequency = (ledType == LedType::dotstar) ? DefaultDotStarSpiClockFrequency : DefaultNeoPixelSpiClockFrequency;

#if SUPPORT_DMA_DOTSTAR || SUPPORT_DMA_NEOPIXEL
	SetupSpi();
#endif

	// Initialise variables
	needInit = true;
	busy = false;
}

// Return true if we must stop movement before we handle this command
bool LedStripDriver::MustStopMovement(GCodeBuffer& gb) noexcept
{
#if SUPPORT_BITBANG_NEOPIXEL
	try
	{
		const LedType lt = (gb.Seen('X')) ? (LedType)gb.GetLimitedUIValue('X', 0, ARRAY_SIZE(LedTypeNames)) : ledType;
		return (lt == LedType::neopixelRGBBitBang || lt == LedType::neopixelRGBWBitBang) && gb.SeenAny("RUBWPYSF");
	}
	catch (const GCodeException&)
	{
		return true;
	}
#else
	return false;
#endif
}

// This function handles M150
// For DotStar LEDs:
// 	We can handle an unlimited length LED strip, because we can send the data in multiple chunks.
//	So whenever we receive a M150 command, we send the data immediately, in multiple chunks if our DMA buffer is too small to send it as a single chunk.
//	To send multiple chunks, we process the command once per chunk, using numRemaining to keep track of how many more LEDs need to be written to
// For NeoPixel LEDs:
//	If there is a gap or more then about 9us in transmission, the string will reset and the next command will be taken as applying to the start of the strip.
//  Therefore we need to DMA the data for all LEDs in one go. So the maximum strip length is limited by the size of our DMA buffer.
//	We buffer up incoming data until we get a command with the Following parameter missing or set to zero, then we DMA it all.
GCodeResult LedStripDriver::SetColours(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
#if SUPPORT_BITBANG_NEOPIXEL
	// Interrupts are disabled while bit-banging data, which will mess up the step timing. So make sure movement has stopped if we are going to use bit-banging
	if (MustStopMovement(gb))
	{
		if (!reprap.GetGCodes().LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
	}
#endif

#if SUPPORT_DMA_DOTSTAR || SUPPORT_DMA_NEOPIXEL
	if (DmaInProgress())													// if we are sending something
	{
		return GCodeResult::notFinished;
	}

	if (   needStartFrame
		&& ledType != LedType::neopixelRGBBitBang && ledType != LedType::neopixelRGBWBitBang
		&& StepTimer::GetTimerTicks() - whenDmaFinished < MinNeoPixelResetTicks
	   )
	{
		return GCodeResult::notFinished;									// give the NeoPixels time to reset
	}
#endif

	// Deal with changing the LED type first
	bool seenType = false;
	if (gb.Seen('X'))
	{
		seenType = true;
		const uint32_t newType = gb.GetLimitedUIValue('X', 0, ARRAY_SIZE(LedTypeNames));
		const bool typeChanged = (newType != (unsigned int)ledType);

#if !SUPPORT_DMA_NEOPIXEL || !SUPPORT_DMA_DOTSTAR || !SUPPORT_BITBANG_NEOPIXEL
		// Check whether the new type is supported
		if (LedTypeNames[newType] == nullptr)
		{
			reply.copy("Unsupported LED strip type");
			return GCodeResult::error;
		}
#endif

#if SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR
# if SUPPORT_BITBANG_NEOPIXEL
		if (newType != (unsigned int)LedType::neopixelRGBBitBang && newType != (unsigned int)LedType::neopixelRGBWBitBang)
# endif
		{
			bool setFrequency = typeChanged;
			currentFrequency = (newType == (unsigned int)LedType::dotstar) ? DefaultDotStarSpiClockFrequency : DefaultNeoPixelSpiClockFrequency;
			gb.TryGetUIValue('Q', currentFrequency, setFrequency);
			if (setFrequency)
			{
				SetupSpi();
			}
		}
#endif

		if (typeChanged)
		{
			ledType = (LedType)newType;
			needInit = true;
		}
	}

	if (needInit)
	{
		// Either we changed the type, or this is first-time initialisation
		numRemaining = totalSent = numAlreadyInBuffer = 0;
		needInit = false;

#if SUPPORT_BITBANG_NEOPIXEL
		if (ledType == LedType::neopixelRGBBitBang || ledType == LedType::neopixelRGBWBitBang)
		{
			// Set the data output low to start a WS2812 reset sequence
			IoPort::SetPinMode(LcdNeopixelOutPin, PinMode::OUTPUT_LOW);
			whenDmaFinished = StepTimer::GetTimerTicks();
		}
#endif
		needStartFrame = true;
		return GCodeResult::notFinished;
	}

	// Get the RGB and brightness values
	uint32_t red = 0, green = 0, blue = 0, white = 0, brightness = 128;
	uint32_t numLeds = MaxLedsPerBuffer();
	bool following = false;
	bool seenColours = false;

	gb.TryGetLimitedUIValue('R', red, seenColours, 256);
	gb.TryGetLimitedUIValue('U', green, seenColours, 256);
	gb.TryGetLimitedUIValue('B', blue, seenColours, 256);
	gb.TryGetLimitedUIValue('W', white, seenColours, 256);					// W value is used by RGBW NeoPixels only

	if (gb.Seen('P'))
	{
		brightness = gb.GetLimitedUIValue('P', 256);						// valid P values are 0-255
	}
	else if (gb.Seen('Y'))
	{
		brightness = gb.GetLimitedUIValue('Y',  32) << 3;					// valid Y values are 0-31
	}

	gb.TryGetUIValue('S', numLeds, seenColours);
	gb.TryGetBValue('F', following, seenColours);

	if (!seenColours)
	{
		if (!seenType)
		{
			// Report the current configuration
			reply.printf("Led type is %s, frequency %.2fMHz", LedTypeNames[(unsigned int)ledType], (double)((float)currentFrequency * 0.000001));
		}
		return GCodeResult::ok;
	}

	// If there are no LEDs to set, we have finished unless we need to send a start frame to DotStar LEDs
	if (numLeds == 0
#if SUPPORT_DMA_DOTSTAR
		&& (ledType != LedType::dotstar || (!needStartFrame && !following))
#endif
		)
	{
		return GCodeResult::ok;
	}

	switch (ledType)
	{
	case LedType::dotstar:
#if SUPPORT_DMA_DOTSTAR
		{
			if (numRemaining != 0)
			{
				numLeds = numRemaining;
			}

# if USE_16BIT_SPI
			// Swap bytes for 16-bit SPI
			const uint32_t data = ((brightness & 0xF8) << 5) | (0xE0 << 8) | ((blue & 255)) | ((green & 255) << 24) | ((red & 255) << 16);
# else
			const uint32_t data = (brightness >> 3) | 0xE0 | ((blue & 255) << 8) | ((green & 255) << 16) | ((red & 255) << 24);
# endif
			return SendDotStarData(data, numLeds, following);
		}
#else
		break;
#endif

	case LedType::neopixelRGB:
	case LedType::neopixelRGBW:
#if SUPPORT_DMA_NEOPIXEL
		// Scale RGB by the brightness
		return SpiSendNeoPixelData(	(uint8_t)((red * brightness + 255) >> 8),
									(uint8_t)((green * brightness + 255) >> 8),
									(uint8_t)((blue * brightness + 255) >> 8),
									(uint8_t)((white * brightness + 255) >> 8),
									numLeds, (ledType == LedType::neopixelRGBW), following
							      );
#else
		break;
#endif

	case LedType::neopixelRGBBitBang:
	case LedType::neopixelRGBWBitBang:
#if SUPPORT_BITBANG_NEOPIXEL
		// Scale RGB by the brightness
		return BitBangNeoPixelData(	(uint8_t)((red * brightness + 255) >> 8),
									(uint8_t)((green * brightness + 255) >> 8),
									(uint8_t)((blue * brightness + 255) >> 8),
									(uint8_t)((white * brightness + 255) >> 8),
									numLeds, (ledType == LedType::neopixelRGBWBitBang), following
							      );
#else
		break;
#endif
	}
	return GCodeResult::ok;													// should never get here
}

#endif

// End
