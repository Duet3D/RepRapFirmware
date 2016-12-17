/*
 * Spi.h
 *
 *  Created on: 16 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_ETHERNET3_UTILITY_WIZSPI_H_
#define SRC_DUETNG_DUETETHERNET_ETHERNET3_UTILITY_WIZSPI_H_

#include "spi.h"

namespace WizSpi
{
	void Init();
	void Stop();
	void AssertSS();
	void ReleaseSS();
	uint8_t ExchangeByte(uint8_t b, bool isLast);
	spi_status_t ReadBurst(uint8_t* rx_data, size_t len);
	spi_status_t SendBurst(const uint8_t* tx_data, size_t len);
}

#endif /* SRC_DUETNG_DUETETHERNET_ETHERNET3_UTILITY_WIZSPI_H_ */
