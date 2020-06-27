/*
 * Serial.h
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include <Core.h>
#include <General/RingBuffer.h>
#include <RTOSIface/RTOSIface.h>
#include "Stream.h"

namespace Serial
{
	static Sercom * const Sercoms[] =
	{
		SERCOM0, SERCOM1, SERCOM2, SERCOM3, SERCOM4, SERCOM5, SERCOM6, SERCOM7
	};

	static constexpr IRQn const SercomIRQns[] =
	{
		SERCOM0_0_IRQn, SERCOM1_0_IRQn, SERCOM2_0_IRQn, SERCOM3_0_IRQn, SERCOM4_0_IRQn, SERCOM5_0_IRQn, SERCOM6_0_IRQn, SERCOM7_0_IRQn
	};

	inline Sercom *GetSercom(uint8_t sercomNumber) { return Sercoms[sercomNumber]; }
	inline constexpr IRQn GetSercomIRQn(uint8_t sercomNumber) { return SercomIRQns[sercomNumber]; }

	void EnableSercomClock(uint8_t sercomNumber);
	void InitUart(uint8_t sercomNumber, uint32_t baudRate, uint8_t rxPad, bool use32bitMode = false);
	void Disable(uint8_t sercomNumber);
}

#endif /* SRC_SERIAL_H_ */
