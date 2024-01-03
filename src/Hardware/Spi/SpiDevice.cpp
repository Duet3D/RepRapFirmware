/*
 * SpiDevice.cpp
 *
 *  Created on: 7 May 2022
 *      Author: David
 */

#include "SpiDevice.h"
#include <Hardware/IoPorts.h>
#include <AppNotifyIndices.h>

#if SAME5x
# include <Serial.h>
# include <peripheral_clk_config.h>
#elif USART_SPI
# if SAME70 || SAM4E || SAM4S
#  include <pmc/pmc.h>
# endif
# include <usart/usart.h>
#else
# include <spi/spi.h>
#endif

constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;
constexpr uint32_t SpiTimeout = 10000;

SpiDevice::SpiDevice(uint8_t sercomNum) noexcept
#if SAME5x
	: hardware(Serial::Sercoms[sercomNum]), sercomNumber(sercomNum)
#elif USART_SPI
	: hardware(USART_SSPI)			// we ignore the parameter and support just one shared SPI
#else
	: hardware(SHARED_SPI)			// we ignore the parameter and support just one shared SPI
#endif
{
#if SAME5x
	Serial::EnableSercomClock(sercomNum);

	// Set up the SERCOM
	const uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0);
	const uint32_t regCtrlB = 0;											// 8 bits, slave select disabled, receiver disabled for now
	const uint32_t regCtrlC = 0;											// not 32-bit mode

	if ((hardware->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_SWRST) == 0)
	{
		while (hardware->SPI.SYNCBUSY.reg & (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE)) { }
		if (hardware->SPI.CTRLA.reg & SERCOM_SPI_CTRLA_ENABLE)
		{
			hardware->SPI.CTRLA.reg &= ~SERCOM_SPI_CTRLA_ENABLE;
			while (hardware->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }
		}
		hardware->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_SWRST | (regCtrlA & SERCOM_SPI_CTRLA_MODE_Msk);
	}
	while (hardware->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_SWRST) { }

	hardware->SPI.CTRLA.reg = regCtrlA;
	hardware->SPI.CTRLB.reg = regCtrlB;
	hardware->SPI.CTRLC.reg = regCtrlC;
	hardware->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * DefaultSharedSpiClockFrequency) - 1);
	hardware->SPI.DBGCTRL.reg = SERCOM_SPI_DBGCTRL_DBGSTOP;					// baud rate generator is stopped when CPU halted by debugger

#if 0	// if using DMA
	// Set up the DMA descriptors
	// We use separate write-back descriptors, so we only need to set this up once, but it must be in SRAM
	DmacSetBtctrl(SspiRxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
	DmacSetSourceAddress(SspiRxDmaChannel, &(hardware->SPI.DATA.reg));
	DmacSetDestinationAddress(SspiRxDmaChannel, rcvData);
	DmacSetDataLength(SspiRxDmaChannel, ARRAY_SIZE(rcvData));

	DmacSetBtctrl(SspiTxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_STEPSIZE_X1);
	DmacSetSourceAddress(SspiTxDmaChannel, sendData);
	DmacSetDestinationAddress(SspiTxDmaChannel, &(hardware->SPI.DATA.reg));
	DmacSetDataLength(SspiTxDmaChannel, ARRAY_SIZE(sendData));

	DmacSetInterruptCallbacks(SspiRxDmaChannel, RxDmaCompleteCallback, nullptr, 0U);
#endif

	hardware->SPI.CTRLB.bit.RXEN = 1;

#elif USART_SPI

	pmc_enable_periph_clk(ID_SSPI);

	// Set USART in SPI master mode
	hardware->US_IDR = ~0u;
	hardware->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
	hardware->US_MR = US_MR_USART_MODE_SPI_MASTER
					| US_MR_USCLKS_MCK
					| US_MR_CHRL_8_BIT
					| US_MR_CHMODE_NORMAL;
	hardware->US_BRGR = SystemPeripheralClock()/DefaultSharedSpiClockFrequency;
	hardware->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

#else

	pmc_enable_periph_clk(SHARED_SPI_INTERFACE_ID);

	hardware->SPI_CR = SPI_CR_SPIDIS;
	hardware->SPI_CR = SPI_CR_SWRST;
	hardware->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;

#endif
}

void SpiDevice::Disable() const noexcept
{
#if SAME5x
	hardware->SPI.CTRLA.bit.ENABLE = 0;
	while (hardware->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }
#elif USART_SPI
	hardware->US_CR = US_CR_RXDIS | US_CR_TXDIS;			// disable transmitter and receiver
#else
	spi_disable(hardware);
#endif
}

void SpiDevice::Enable() const noexcept
{
#if SAME5x
	hardware->SPI.CTRLA.bit.ENABLE = 1;
	while (hardware->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }
#elif USART_SPI
	hardware->US_CR = US_CR_RXEN | US_CR_TXEN;				// enable transmitter and receiver
#else
	spi_enable(hardware);
#endif
}

// Wait for transmitter ready returning true if timed out
inline bool SpiDevice::waitForTxReady() const noexcept
{
	uint32_t timeout = SpiTimeout;
#if SAME5x
	while (!(hardware->SPI.INTFLAG.bit.DRE))
#elif USART_SPI
	while (!usart_is_tx_ready(hardware))
#else
	while (!spi_is_tx_ready(hardware))
#endif
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

// Wait for transmitter empty returning true if timed out
inline bool SpiDevice::waitForTxEmpty() const noexcept
{
	uint32_t timeout = SpiTimeout;
#if SAME5x
	while (!(hardware->SPI.INTFLAG.bit.TXC))
#elif USART_SPI
	while (!usart_is_tx_empty(hardware))
#else
	while (!spi_is_tx_empty(hardware))
#endif
	{
		if (!timeout--)
		{
			return true;
		}
	}
	return false;
}

// Wait for receive data available returning true if timed out
inline bool SpiDevice::waitForRxReady() const noexcept
{
	uint32_t timeout = SpiTimeout;
#if SAME5x
	while (!(hardware->SPI.INTFLAG.bit.RXC))
#elif USART_SPI
	while (!usart_is_rx_ready(hardware))
#else
	while (!spi_is_rx_ready(hardware))
#endif
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}

void SpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode
#if SAME5x
											, bool nineBits
#endif
										) const noexcept
{
	// We have to disable SPI device in order to change the baud rate, mode and character length
#if SAME5x
	Disable();
	// Round the clock frequency rate down. For example, using 60MHz clock, if we ask for 4MHz:
	// Without rounding, divisor = 60/(2*4) = 7, actual clock rate = 4.3MHz
	// With rounding, divisor = 67/8 = 8, actual clock rate = 3.75MHz
	// To get more accurate speeds we could increase the clock frequency to 100MHz
	hardware->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD((Serial::SercomFastGclkFreq + (2 * freq) - 1)/(2 * freq) - 1);
	hardware->SPI.CTRLB.bit.CHSIZE = (nineBits) ? 1 : 0;
	while (hardware->SPI.SYNCBUSY.bit.CTRLB) { }

	uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0);
	if (((uint8_t)mode & 2) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPOL;
	}
	if (((uint8_t)mode & 1) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPHA;
	}
	hardware->SPI.CTRLA.reg = regCtrlA;
	Enable();
#elif USART_SPI
	Disable();
	hardware->US_BRGR = SystemPeripheralClock()/freq;
	uint32_t mr = US_MR_USART_MODE_SPI_MASTER
					| US_MR_USCLKS_MCK
					| US_MR_CHRL_8_BIT
					| US_MR_CHMODE_NORMAL
					| US_MR_CLKO;
	if ((uint8_t)mode & 2)
	{
		mr |= US_MR_CPOL;
	}
	if (((uint8_t)mode & 1) == 0)							// the bit is called CPHA but is actually NPCHA
	{
		mr |= US_MR_CPHA;
	}
	hardware->US_MR = mr;
	hardware->US_CR = US_CR_RSTRX | US_CR_RSTTX;			// reset transmitter and receiver (required - see datasheet)
	Enable();
#else
	spi_reset(hardware);
	hardware->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;

	// Set SPI mode, clock frequency, CS not active after transfer, delay between transfers
	const uint16_t baud_div = (uint16_t)spi_calc_baudrate_div(freq, SystemPeripheralClock());
	uint32_t csr = SPI_CSR_SCBR(baud_div)				// Baud rate
					| SPI_CSR_BITS_8_BIT				// Transfer bit width
					| SPI_CSR_DLYBCT(0);      			// Transfer delay
	if (((uint8_t)mode & 0x01) == 0)
	{
		csr |= SPI_CSR_NCPHA;
	}
	if ((uint8_t)mode & 0x02)
	{
		csr |= SPI_CSR_CPOL;
	}
	hardware->SPI_CSR[PERIPHERAL_CHANNEL_ID] = csr;
	spi_enable(hardware);
#endif
}

// Send and receive data returning true if successful
bool SpiDevice::TransceivePacket(const uint8_t *_ecv_array null tx_data, uint8_t *_ecv_array null rx_data, size_t len) const noexcept
{
	// Clear any existing data
#if SAME5x
	(void)hardware->SPI.DATA.reg;
#elif USART_SPI
	(void)hardware->US_RHR;
#else
	(void)hardware->SPI_RDR;
#endif

	for (uint32_t i = 0; i < len; ++i)
	{
		uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
		if (waitForTxReady())			// we have to write the first byte after enabling the device without waiting for DRE to be set
		{
			return false;
		}

		// Write to transmit register
#if SAME5x
		hardware->SPI.DATA.reg = dOut;
#elif USART_SPI
		hardware->US_THR = dOut;
#else
		if (i + 1 == len)
		{
			dOut |= SPI_TDR_LASTXFER;
		}
		hardware->SPI_TDR = dOut;
#endif

		// Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
		if (rx_data != nullptr)
		{
			// Wait for receive register
			if (waitForRxReady())
			{
				return false;
			}

			// Get data from receive register
			const uint8_t dIn =
#if SAME5x
					(uint8_t)hardware->SPI.DATA.reg;
#elif USART_SPI
					(uint8_t)hardware->US_RHR;
#else
					(uint8_t)hardware->SPI_RDR;
#endif
			*rx_data++ = dIn;
		}
	}

	// Wait for transmitter empty, to make sure that the last clock pulse has finished
	waitForTxEmpty();

	// If we were not receiving, clear data from the receive buffer
	if (rx_data == nullptr)
	{
#if SAME5x
		// The SAME5x seems to buffer more than one received character
		while (hardware->SPI.INTFLAG.bit.RXC)
		{
			(void)hardware->SPI.DATA.reg;
		}
#elif USART_SPI
		(void)hardware->US_RHR;
#else
		(void)hardware->SPI_RDR;
#endif
	}

	return true;	// success
}

#if SAME5x && (defined(FMDC_V02) || defined(FMDC_V03))

// Send and receive data returning true if successful, using 16-bit data transfers (needed when using 9-bit characters). 'len' is in 16-bit words.
bool SpiDevice::TransceivePacketNineBit(const uint16_t *_ecv_array null tx_data, uint16_t *_ecv_array null rx_data, size_t len) noexcept
{
	// Clear any existing data
#if SAME5x
	(void)hardware->SPI.DATA.reg;
#elif USART_SPI
	(void)hardware->US_RHR;
#else
	(void)hardware->SPI_RDR;
#endif

#if SAME5x
	if (len >= 50 && rx_data == nullptr && tx_data != nullptr)
	{
		// Sending a large amount of data to LCD, so use DMA. Currently only the TFT LCD uses this device, so we use a fixed DMA channel number.
		DmacManager::DisableChannel(DmacChanLcdTx);
		DmacManager::SetSourceAddress(DmacChanLcdTx, tx_data);
		DmacManager::SetDestinationAddress(DmacChanLcdTx, &(hardware->SPI.DATA));
		DmacManager::SetBtctrl(DmacChanLcdTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_NOACT);
		DmacManager::SetDataLength(DmacChanLcdTx, len);
		DmacManager::SetTriggerSourceSercomTx(DmacChanLcdTx, sercomNumber);
		waitingTask = TaskBase::GetCallerTaskHandle();
		DmacManager::SetInterruptCallback(DmacChanLcdTx, SpiDevice::DmaComplete, CallbackParameter((void *)this));
		DmacManager::EnableCompletedInterrupt(DmacChanLcdTx);
		DmacManager::EnableChannel(DmacChanLcdTx, DmacPrioLcdTx);
		TaskBase::TakeIndexed(NotifyIndices::Spi, 10);			// maximum 3kb transfer should complete in about 2ms @ 14MHz clock speed
	}
	else
#endif
	{
		for (uint32_t i = 0; i < len; ++i)
		{
			uint32_t dOut = (tx_data == nullptr) ? 0x000001FF : (uint32_t)*tx_data++;
			if (waitForTxReady())			// we have to write the first byte after enabling the device without waiting for DRE to be set
			{
				return false;
			}

			// Write to transmit register
#if SAME5x
			hardware->SPI.DATA.reg = dOut;
#elif USART_SPI
			hardware->US_THR = dOut;
#else
			if (i + 1 == len)
			{
				dOut |= SPI_TDR_LASTXFER;
			}
			hardware->SPI_TDR = dOut;
#endif

			// Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
			if (rx_data != nullptr)
			{
				// Wait for receive register
				if (waitForRxReady())
				{
					return false;
				}

				// Get data from receive register
				const uint16_t dIn =
#if SAME5x
					(uint16_t)hardware->SPI.DATA.reg;
#elif USART_SPI
					(uint16_t)hardware->US_RHR;
#else
					(uint16_t)hardware->SPI_RDR;
#endif
				*rx_data++ = dIn;
			}
		}

	}

	// Wait for transmitter empty, to make sure that the last clock pulse has finished
	waitForTxEmpty();

	// If we were not receiving, clear data from the receive buffer
	if (rx_data == nullptr)
	{
#if SAME5x
		// The SAME5x seems to buffer more than one received character
		while (hardware->SPI.INTFLAG.bit.RXC)
		{
			(void)hardware->SPI.DATA.reg;
		}
#elif USART_SPI
		(void)hardware->US_RHR;
#else
		(void)hardware->SPI_RDR;
#endif
	}

	return true;	// success
}

void SpiDevice::DmaComplete(DmaCallbackReason reason) noexcept
{
	TaskBase::GiveFromISR(waitingTask, NotifyIndices::Spi);
	waitingTask = nullptr;
}

/*static*/ void SpiDevice::DmaComplete(CallbackParameter param, DmaCallbackReason reason) noexcept
{
	static_cast<SpiDevice*>(param.vp)->DmaComplete(reason);
}

#endif

// End

