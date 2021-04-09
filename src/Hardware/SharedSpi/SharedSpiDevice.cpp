/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#include "SharedSpiDevice.h"

#include <Hardware/IoPorts.h>

#if SAME5x
# include <DmacManager.h>
# include <Serial.h>
# include <peripheral_clk_config.h>
# include <hri_sercom_e54.h>
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

// SharedSpiDevice members

SharedSpiDevice::SharedSpiDevice(uint8_t sercomNum) noexcept
#if SAME5x
	: hardware(Serial::Sercoms[sercomNum])
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

	if (!hri_sercomusart_is_syncing(hardware, SERCOM_USART_SYNCBUSY_SWRST))
	{
		const uint32_t mode = regCtrlA & SERCOM_USART_CTRLA_MODE_Msk;
		if (hri_sercomusart_get_CTRLA_reg(hardware, SERCOM_USART_CTRLA_ENABLE))
		{
			hri_sercomusart_clear_CTRLA_ENABLE_bit(hardware);
			hri_sercomusart_wait_for_sync(hardware, SERCOM_USART_SYNCBUSY_ENABLE);
		}
		hri_sercomusart_write_CTRLA_reg(hardware, SERCOM_USART_CTRLA_SWRST | mode);
	}
	hri_sercomusart_wait_for_sync(hardware, SERCOM_USART_SYNCBUSY_SWRST);

	hri_sercomusart_write_CTRLA_reg(hardware, regCtrlA);
	hri_sercomusart_write_CTRLB_reg(hardware, regCtrlB);
	hri_sercomusart_write_CTRLC_reg(hardware, regCtrlC);
	hri_sercomusart_write_BAUD_reg(hardware, SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * DefaultSharedSpiClockFrequency) - 1));
	hri_sercomusart_write_DBGCTRL_reg(hardware, SERCOM_I2CM_DBGCTRL_DBGSTOP);			// baud rate generator is stopped when CPU halted by debugger

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

	mutex.Create("SPI");
}

// SharedSpiClient members

void SharedSpiDevice::Disable() const noexcept
{
#if SAME5x
	hardware->SPI.CTRLA.bit.ENABLE = 0;
	hri_sercomusart_wait_for_sync(hardware, SERCOM_USART_CTRLA_ENABLE);
#elif USART_SPI
	hardware->US_CR = US_CR_RXDIS | US_CR_TXDIS;			// disable transmitter and receiver
#else
	spi_disable(hardware);
#endif
}

void SharedSpiDevice::Enable() const noexcept
{
#if SAME5x
	hardware->SPI.CTRLA.bit.ENABLE = 1;
	hri_sercomusart_wait_for_sync(hardware, SERCOM_USART_CTRLA_ENABLE);
#elif USART_SPI
	hardware->US_CR = US_CR_RXEN | US_CR_TXEN;				// enable transmitter and receiver
#else
	spi_enable(hardware);
#endif
}

// Wait for transmitter ready returning true if timed out
inline bool SharedSpiDevice::waitForTxReady() const noexcept
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
inline bool SharedSpiDevice::waitForTxEmpty() const noexcept
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
inline bool SharedSpiDevice::waitForRxReady() const noexcept
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

void SharedSpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	// We have to disable SPI device in order to change the baud rate and mode
#if SAME5x
	Disable();
	// Round the clock frequency rate down. For example, using 60MHz clock, if we ask for 4MHz:
	// Without rounding, divisor = 60/(2*4) = 7, actual clock rate = 4.3MHz
	// With rounding, divisor = 67/8 = 8, actual clock rate = 3.75MHz
	// To get more accurate speeds we could increase the clock frequency to 100MHz
	hri_sercomusart_write_BAUD_reg(hardware, SERCOM_SPI_BAUD_BAUD((Serial::SercomFastGclkFreq + (2 * freq) - 1)/(2 * freq) - 1));

	uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0) | SERCOM_SPI_CTRLA_ENABLE;
	if (((uint8_t)mode & 2) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPOL;
	}
	if (((uint8_t)mode & 1) != 0)
	{
		regCtrlA |= SERCOM_SPI_CTRLA_CPHA;
	}
	hri_sercomusart_write_CTRLA_reg(hardware, regCtrlA);
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
bool SharedSpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
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

// Static members

SharedSpiDevice *SharedSpiDevice::mainSharedSpiDevice = nullptr;

void SharedSpiDevice::Init() noexcept
{
#if SAME5x
	pinMode(SharedSpiMosiPin, INPUT_PULLDOWN);
	pinMode(SharedSpiMisoPin, INPUT_PULLDOWN);
	pinMode(SharedSpiSclkPin, INPUT_PULLDOWN);
	SetPinFunction(SharedSpiMosiPin, SharedSpiPinFunction);
	SetPinFunction(SharedSpiMisoPin, SharedSpiPinFunction);
	SetPinFunction(SharedSpiSclkPin, SharedSpiPinFunction);
	mainSharedSpiDevice = new SharedSpiDevice(SharedSpiSercomNumber);
#elif USART_SPI
	SetPinFunction(APIN_USART_SSPI_SCK, USARTSPISckPeriphMode);
	SetPinFunction(APIN_USART_SSPI_MOSI, USARTSPIMosiPeriphMode);
	SetPinFunction(APIN_USART_SSPI_MISO, USARTSPIMisoPeriphMode);
	mainSharedSpiDevice = new SharedSpiDevice(0);
#else
	ConfigurePin(g_APinDescription[APIN_SHARED_SPI_SCK]);
	ConfigurePin(g_APinDescription[APIN_SHARED_SPI_MOSI]);
	ConfigurePin(g_APinDescription[APIN_SHARED_SPI_MISO]);
	mainSharedSpiDevice = new SharedSpiDevice(0);
#endif
}

// End
