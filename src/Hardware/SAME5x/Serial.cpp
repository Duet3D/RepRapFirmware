/*
 * Serial.cpp - simple serial driver for sending messages to an attached PanelDue
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#include "Serial.h"

#include "Peripherals.h"
#include <peripheral_clk_config.h>
#include <hal_gpio.h>
#include <atmel_start_pins.h>

#if defined(SAME51)
# include <hri_sercom_e51.h>
#elif defined(SAMC21)
# include <hri_sercom_c21.h>
#else
# error Unsupported processor
#endif

constexpr uint32_t DiagBaudRate = 57600;		// the baud rate we use

void Serial::EnableSercomClock(uint8_t sercomNumber)
{
	switch (sercomNumber)
	{

#if defined(SAME51)

	case 0:
		MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0;
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		break;

	case 1:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM1;
		break;

	case 2:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2;
		break;

	case 3:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM3;
		break;

	case 4:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM4;
		break;

	case 5:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5;
		break;

	case 6:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM6_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM6_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM6;
		break;
	case 7:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM7_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM7_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM7;
		break;

#elif defined(SAMC21)

	case 0:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM0;
		break;

	case 1:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM1;
		break;

	case 2:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM2;
		break;

	case 3:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM3;
		break;

	case 4:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM4;
		break;

	case 5:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, GCLK_PCHCTRL_GEN_GCLK3 | GCLK_PCHCTRL_CHEN);
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM5;
		break;

#else
# error Unsupported processor
#endif

	default:
		break;
	}
}

// Initialise the serial port. This does not set up the I/O pins.
void Serial::InitUart(uint8_t sercomNumber, uint32_t baudRate, uint8_t rxPad)
{
	EnableSercomClock(sercomNumber);
	Sercom * const sercom = GetSercom(sercomNumber);

	const uint32_t ctrla = (1u << SERCOM_USART_CTRLA_DORD_Pos)				// MSB first
						 | (0u << SERCOM_USART_CTRLA_CPOL_Pos)				// use rising clock edge
						 | (0u << SERCOM_USART_CTRLA_CMODE_Pos)				// async mode
						 | (0u << SERCOM_USART_CTRLA_FORM_Pos)				// usart frame, no parity
						 | (0u << SERCOM_USART_CTRLA_SAMPA_Pos)				// sample on clocks 7-8-9
						 | ((uint32_t)rxPad << SERCOM_USART_CTRLA_RXPO_Pos)	// receive data pad
						 | (0u << SERCOM_USART_CTRLA_TXPO_Pos)				// transmit on pad 0
						 | (0u << SERCOM_USART_CTRLA_SAMPR_Pos)				// 16x over sampling, normal baud rate generation
#ifdef SAME51
						 | (0u << SERCOM_USART_CTRLA_RXINV_Pos)				// don't invert receive data
						 | (0u << SERCOM_USART_CTRLA_TXINV_Pos)				// don't invert transmitted data
#endif
						 | (0u << SERCOM_USART_CTRLA_IBON_Pos)				// don't report buffer overflow early
						 | (0u << SERCOM_USART_CTRLA_RUNSTDBY_Pos)			// don't clock during standby
						 | (1u << SERCOM_USART_CTRLA_MODE_Pos)				// use internal clock
						 | (0u << SERCOM_USART_CTRLA_ENABLE_Pos)			// not enabled
						 | (0u << SERCOM_USART_CTRLA_SWRST_Pos);			// no reset
	if (!hri_sercomusart_is_syncing(sercom, SERCOM_USART_SYNCBUSY_SWRST))
	{
		const uint32_t mode = ctrla & SERCOM_USART_CTRLA_MODE_Msk;
		if (hri_sercomusart_get_CTRLA_reg(sercom, SERCOM_USART_CTRLA_ENABLE))
		{
			hri_sercomusart_clear_CTRLA_ENABLE_bit(sercom);
			hri_sercomusart_wait_for_sync(sercom, SERCOM_USART_SYNCBUSY_ENABLE);
		}
		hri_sercomusart_write_CTRLA_reg(sercom, SERCOM_USART_CTRLA_SWRST | mode);
	}
	hri_sercomusart_wait_for_sync(sercom, SERCOM_USART_SYNCBUSY_SWRST);

	sercom->USART.CTRLA.reg = ctrla;
	sercom->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
	sercom->USART.CTRLC.reg = 0u;
	const uint32_t baudReg = 65536u - (((uint64_t)65536 * 16 * baudRate)/SystemPeripheralClock);
	sercom->USART.BAUD.reg = baudReg;
	hri_sercomusart_set_CTRLA_ENABLE_bit(sercom);
	hri_sercomusart_wait_for_sync(sercom, SERCOM_USART_SYNCBUSY_ENABLE);
}

// Undo the initialisation, so that when we jump into the main firmware the USART can be initialised again
void Serial::Disable(uint8_t sercomNumber)
{
	Sercom * const sercom = GetSercom(sercomNumber);
	hri_sercomusart_clear_CTRLA_ENABLE_bit(sercom);
	hri_sercomusart_set_CTRLA_SWRST_bit(sercom);
}

Uart::Uart(uint8_t sercomNum, IRQn irqnum)
	: sercom(Serial::GetSercom(sercomNum)), txWaitingTask(nullptr), irqNumber(irqnum), sercomNumber(sercomNum)
{
}

// Initialise the UART. numRxSlots may be zero if we don't wish to receive.
void Uart::Init(size_t numTxSlots, size_t numRxSlots, uint32_t baudRate, uint8_t rxPad)
{
	txBuffer.Init(numTxSlots);
	rxBuffer.Init(numRxSlots);
	Serial::InitUart(sercomNumber, baudRate, rxPad);
	errors.all = 0;
	sercom->USART.INTENSET.reg = (numRxSlots > 1) ? SERCOM_USART_INTENSET_RXC | SERCOM_USART_INTENSET_ERROR : 0;
	NVIC_EnableIRQ(irqNumber);
#ifdef SAME51
	NVIC_EnableIRQ((IRQn)(irqNumber + 1));
	NVIC_EnableIRQ((IRQn)(irqNumber + 2));
	NVIC_EnableIRQ((IRQn)(irqNumber + 3));
#endif
}

// Non-blocking read, return 0 if no character available
char Uart::GetChar()
{
	char c;
	return (rxBuffer.GetItem(c)) ? c : 0;
}

// Write single character, blocking
void Uart::PutChar(char c)
{
	if (txBuffer.IsEmpty() && sercom->USART.INTFLAG.bit.DRE)
	{
		sercom->USART.DATA.reg = c;
	}
	else
	{
		for (;;)
		{
			if (txBuffer.PutItem(c))
			{
				sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
				break;
			}
			txWaitingTask = RTOSIface::GetCurrentTask();
			sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
			TaskBase::Take(50);
		}
	}
}

// Nonblocking write block
size_t Uart::TryPutBlock(const char* buffer, size_t buflen)
{
	const size_t written = txBuffer.PutBlock(buffer, buflen);
	sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
	return written;
}

// Blocking write block
void Uart::PutBlock(const char* buffer, size_t buflen)
{
	for (;;)
	{
		buflen -= txBuffer.PutBlock(buffer, buflen);
		if (buflen == 0)
		{
			sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
			break;
		}
		txWaitingTask = RTOSIface::GetCurrentTask();
		sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
		TaskBase::Take(50);
	}
}

// Blocking null-terminated string write
void Uart::PutString(const char *str)
{
	PutBlock(str, strlen(str));
}

// Get and clear the errors
Uart::ErrorFlags Uart::GetAndClearErrors()
{
	__disable_irq();
	const ErrorFlags errs = errors;
	errors.all = 0;
	__enable_irq();
	return errs;
}

// Interrupts from the SERCOM arrive here
void Uart::Interrupt()
{
	const uint8_t status = sercom->USART.INTFLAG.reg;
	if (status & SERCOM_USART_INTFLAG_RXC)
	{
		const char c = sercom->USART.DATA.reg;
		if (rxBuffer.PutItem(c))
		{
			errors.overrun = true;
		}
	}
	if (status & SERCOM_USART_INTFLAG_ERROR)
	{
		const uint16_t stat2 = sercom->USART.STATUS.reg;
		if (stat2 & SERCOM_USART_STATUS_BUFOVF)
		{
			errors.overrun = true;
		}
		if (stat2 & SERCOM_USART_STATUS_FERR)
		{
			errors.framing = true;
		}
		sercom->USART.STATUS.reg = stat2;
		sercom->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_ERROR;			// clear the error
	}
	if (status & SERCOM_USART_INTFLAG_DRE)
	{
		char c;
		if (txBuffer.GetItem(c))
		{
			sercom->USART.DATA.reg = c;
			if (txWaitingTask != nullptr && txBuffer.SpaceLeft() >= txBuffer.GetCapacity()/2)
			{
				TaskBase::GiveFromISR(txWaitingTask);
				txWaitingTask = nullptr;
			}
		}
		else
		{
			sercom->USART.INTENCLR.reg = SERCOM_USART_INTENSET_DRE;
			if (txWaitingTask != nullptr)
			{
				TaskBase::GiveFromISR(txWaitingTask);
				txWaitingTask = nullptr;
			}
		}
	}
}

// End
