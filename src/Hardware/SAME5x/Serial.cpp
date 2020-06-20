/*
 * Serial.cpp - simple serial driver for sending messages to an attached PanelDue
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#include "Serial.h"

#include <peripheral_clk_config.h>
#include <hal_gpio.h>
#include <AtmelStart_SAME5x/atmel_start_pins.h>

#include <hri_sercom_e54.h>

constexpr uint32_t DiagBaudRate = 57600;		// the baud rate we use

constexpr uint32_t SercomFastClock = GCLK_PCHCTRL_GEN_GCLK3;
constexpr uint32_t SercomSlowClock = GCLK_PCHCTRL_GEN_GCLK1;

void Serial::EnableSercomClock(uint8_t sercomNumber)
{
	switch (sercomNumber)
	{
	case 0:
		MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0;
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, SercomFastClock | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, SercomSlowClock | GCLK_PCHCTRL_CHEN);
		break;

	case 1:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, SercomFastClock | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, SercomSlowClock | GCLK_PCHCTRL_CHEN);
		MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM1;
		break;

	case 2:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, SercomFastClock | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, SercomSlowClock | GCLK_PCHCTRL_CHEN);
		MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2;
		break;

	case 3:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, SercomFastClock | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, SercomSlowClock | GCLK_PCHCTRL_CHEN);
		MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM3;
		break;

	case 4:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, SercomFastClock | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, SercomSlowClock | GCLK_PCHCTRL_CHEN);
		MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM4;
		break;

	case 5:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, SercomFastClock | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, SercomSlowClock | GCLK_PCHCTRL_CHEN);
		MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5;
		break;

	case 6:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM6_GCLK_ID_CORE, SercomFastClock | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM6_GCLK_ID_SLOW, SercomSlowClock | GCLK_PCHCTRL_CHEN);
		MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM6;
		break;

	case 7:
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM7_GCLK_ID_CORE, SercomFastClock | GCLK_PCHCTRL_CHEN);
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM7_GCLK_ID_SLOW, SercomSlowClock | GCLK_PCHCTRL_CHEN);
		MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM7;
		break;

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
						 | (0u << SERCOM_USART_CTRLA_RXINV_Pos)				// don't invert receive data
						 | (0u << SERCOM_USART_CTRLA_TXINV_Pos)				// don't invert transmitted data
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

// End
