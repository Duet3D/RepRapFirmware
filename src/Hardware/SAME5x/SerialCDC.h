/*
 * UsbSerial.h
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_SERIALCDC_H_
#define SRC_HARDWARE_SAME5X_SERIALCDC_H_

#include "Stream.h"
#include <General/RingBuffer.h>
#include <RTOSIface/RTOSIface.h>

class SerialCDC : public Stream
{
public:
	SerialCDC(Pin p, size_t numTxSlots, size_t numRxSlots) noexcept;

	// Overridden virtual functions
	int available() noexcept override;
	int read() noexcept override;
	void flush() noexcept override;
	size_t canWrite() const noexcept override;
    size_t write(uint8_t) noexcept override;
    size_t write(const uint8_t *buffer, size_t size) noexcept override;		// this has a default implementation, but can be overridden for efficiency

    using Print::write; // pull in write(str) and write(buf, size) from Print

    void Start() noexcept;
    bool IsConnected() const noexcept;

	// Compatibility functions
	void end() noexcept;

	// These are called by the callback functions
	void StartSending() noexcept;
	void StartReceiving() noexcept;
	void DataReceived(uint32_t count) noexcept;

private:

	RingBuffer<uint8_t> txBuffer;
	RingBuffer<uint8_t> rxBuffer;
	volatile TaskHandle txWaitingTask;
	const Pin vbusPin;
};

#endif /* SRC_HARDWARE_SAME5X_SERIALCDC_H_ */
