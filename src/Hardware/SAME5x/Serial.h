/*
 * Serial.h
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include "RepRapFirmware.h"
#include <General/RingBuffer.h>
#include <RTOSIface/RTOSIface.h>

namespace Serial
{
	static Sercom * const Sercoms[] =
	{
		SERCOM0, SERCOM1, SERCOM2, SERCOM3, SERCOM4, SERCOM5,
#ifdef SAME51
		SERCOM6, SERCOM7
#endif
	};

	inline Sercom *GetSercom(uint8_t sercomNumber) { return Sercoms[sercomNumber]; }
	void EnableSercomClock(uint8_t sercomNumber);
	void InitUart(uint8_t SercomNumber, uint32_t baudRate, uint8_t rxPad);
	void Disable(uint8_t sercomNumber);
}

class Uart
{
public:
	union ErrorFlags
	{
		uint8_t all;
		uint8_t overrun : 1,
		framing : 1;
	};

	Uart(uint8_t sercomNum, IRQn irqnum);

	// Initialise. numTxSlots and numRxSlots must be power of 2.
	void Init(size_t numTxSlots, size_t numRxSlots, uint32_t baudRate, uint8_t rxPad);

	// Non-blocking read, returns 0 if no char available.
	char GetChar();

	// Blocking write
	void PutChar(char c);

	// Non-blocking block write
	size_t TryPutBlock(const char *buffer, size_t buflen);

	// Blocking write
	void PutBlock(const char *buffer, size_t buflen);

	// Blocking null-terminated string write
	void PutString(const char *str);

	// ISR, must be called by the ISR for the SERCOM
	void Interrupt();

	// Get and clear the errors
	ErrorFlags GetAndClearErrors();

private:
	RingBuffer<char> txBuffer;
	RingBuffer<char> rxBuffer;
	Sercom * const sercom;
	volatile TaskHandle txWaitingTask;
	const IRQn irqNumber;
	const uint8_t sercomNumber;
	ErrorFlags errors;
};

#endif /* SRC_SERIAL_H_ */
