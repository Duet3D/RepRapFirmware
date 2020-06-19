/*
 * UsbSerial.h
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_USBSERIAL_H_
#define SRC_HARDWARE_SAME5X_USBSERIAL_H_

#include "Stream.h"

class UsbSerial : public Stream
{
public:
	UsbSerial();

	// Overridden virtual functions
	int available() noexcept override;
	int read() noexcept override;
	void flush() noexcept override;
	size_t canWrite() const noexcept override;
	size_t readBytes( char *buffer, size_t length) noexcept override;

    size_t write(uint8_t) noexcept override;
    size_t write(const uint8_t *buffer, size_t size) noexcept override;		// this has a default implementation, but can be overridden for efficiency

    using Print::write; // pull in write(str) and write(buf, size) from Print

    void Start(Pin vbusPin);
    bool IsConnected() const noexcept;

	// Compatibility functions
//	void begin(uint32_t baudRate) noexcept;
	void end() noexcept;
};

#endif /* SRC_HARDWARE_SAME5X_USBSERIAL_H_ */
