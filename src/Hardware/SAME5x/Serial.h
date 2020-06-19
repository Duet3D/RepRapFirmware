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

	inline Sercom *GetSercom(uint8_t sercomNumber) { return Sercoms[sercomNumber]; }
	void EnableSercomClock(uint8_t sercomNumber);
	void InitUart(uint8_t sercomNumber, uint32_t baudRate, uint8_t rxPad);
	void Disable(uint8_t sercomNumber);
}

class Uart : public Stream
{
public:
	typedef void (*InterruptCallbackFn)(Uart*) noexcept;
	union ErrorFlags
	{
		uint8_t all;
		uint8_t overrun : 1,
		framing : 1;
	};

	Uart(uint8_t sercomNum, IRQn irqnum) noexcept;

	// Overridden virtual functions
	int available() noexcept override;
	int read() noexcept override;
	void flush() noexcept override;
	size_t canWrite() const noexcept override;
	size_t readBytes(char *buffer, size_t length) noexcept override;

    size_t write(uint8_t) noexcept override;
    size_t write(const uint8_t *buffer, size_t size) noexcept override;		// this has a default implementation, but can be overridden for efficiency

    using Print::write; // pull in write(str) and write(buf, size) from Print

    // Initialise. numTxSlots and numRxSlots must be power of 2.
	void Init(size_t numTxSlots, size_t numRxSlots, uint32_t baudRate, uint8_t rxPad) noexcept;

	// Compatibility functions
	void begin(uint32_t baudRate) noexcept;
	void end() noexcept;
	void setInterruptPriority(uint32_t prio) const noexcept;
    InterruptCallbackFn SetInterruptCallback(InterruptCallbackFn f) noexcept;

	// Non-blocking read, returns 0 if no char available.
	char GetChar() noexcept;

	// Blocking write
	void PutChar(char c) noexcept;

	// Non-blocking block write
	size_t TryPutBlock(const char *buffer, size_t buflen) noexcept;

	// Blocking write
	void PutBlock(const char *buffer, size_t buflen) noexcept;

	// Blocking null-terminated string write
	void PutString(const char *str) noexcept;

	// ISR, must be called by the ISR for the SERCOM
	void Interrupt() noexcept;

	// Get and clear the errors
	ErrorFlags GetAndClearErrors() noexcept;

private:
	RingBuffer<char> txBuffer;
	RingBuffer<char> rxBuffer;
	Sercom * const sercom;
	volatile TaskHandle txWaitingTask;
	const IRQn irqNumber;
	const uint8_t sercomNumber;
	ErrorFlags errors;
    size_t numInterruptBytesMatched;
    InterruptCallbackFn interruptCallback;

    static constexpr uint8_t interruptSeq[2] = { 0xF0, 0x0F };
};

#endif /* SRC_SERIAL_H_ */
