/*
 * DmacManager.h
 *
 *  Created on: 12 Sep 2018
 *      Author: David
 */

#ifndef SRC_DMACMANAGER_H_
#define SRC_DMACMANAGER_H_

#include "RepRapFirmware.h"

#if SAME70

// DMAC peripheral identifiers for the SAME70 form table 36.1 in the data sheet. These don't seem to be defined anywhere in the ASF files.
enum class DmaTrigSource : uint32_t
{
	hsmci = 0,	// both transmit and receive
	spi0tx,
	spi0rx,
	spi1tx,
	spi1rx,
	qspitx,
	qspirx,
	usart0tx,
	usart0rx,
	usart1tx,
	usart1rx,
	usart2tx,
	usart2rx,
	pwm0tx,
	twihs0tx,
	twihs0rx,
	twihs1tx,
	twihs1rx,
	twihs2tx,
	twihs2rx,
	uart0tx,
	uart0rx,
	uart1tx,
	uart1rx,
	uart2tx,
	uart2rx,
	uart3tx,
	uart3rx,
	uart4tx,
	uart4rx,
	dacctx,
	unused1,	// ID 30 does not appear in the table
	ssctx,
	sscrx,
	pioarx,
	afec0rx,
	afec1rx,
	aestx,
	aesrx,
	pwm1tx,
	tc0rx,
	tc3rx,
	tc6rx,
	tc9rx,
	i2sc0txl,
	i2sc0rxl,
	i2sc1txl,
	i2sc1rxl,
	i2sc0txr,
	i2sc0rxr,
	i2sc1txr,
	i2sc1rxr,
	numPeripheralIds
};

static_assert((uint32_t)DmaTrigSource::numPeripheralIds == 52, "Error in peripheral ID table");

namespace DmacManager
{
	void Init();
	void SetInterruptCallback(const uint8_t channel, StandardCallbackFunction fn, CallbackParameter param);
}

#endif

#endif /* SRC_DMACMANAGER_H_ */
