/*
 * LocalLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/LocalLedStrip.h>

#if SUPPORT_LED_STRIPS

#include <Movement/StepTimer.h>

LocalLedStrip::LocalLedStrip(uint32_t p_freq) noexcept : currentFrequency(p_freq)
{
}

#if SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

// DMA the data. Must be a multiple of 2 bytes if USE_16BIT_SPI is true.
void LocalLedStrip::DmaSendChunkBuffer(size_t numBytes) noexcept
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
bool LocalLedStrip::DmaInProgress() noexcept
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
void LocalLedStrip::SetupSpi() noexcept
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
	DotStarUsart->US_BRGR = SystemPeripheralClock()/currentFrequency;		// set SPI clock frequency
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
	if (IsNeopixel())
	{
		QSPI->QSPI_TDR = 0;													// send a word of zeros to set the data line low
	}
# endif
}

#endif	// SUPPORT_DMA_NEOPIXEL || SUPPORT_DMA_DOTSTAR

#endif	// SUPPORT_LED_STRIPS

// End
