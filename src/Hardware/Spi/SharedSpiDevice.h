/*
 * SharedSpiDevice.h
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SPI_SHAREDSPIDEVICE_H_
#define SRC_HARDWARE_SPI_SHAREDSPIDEVICE_H_

#include "SpiDevice.h"
#include <RTOSIface/RTOSIface.h>

class SharedSpiDevice : public SpiDevice
{
public:
#if SAME5x || SAMC21
	SharedSpiDevice(uint8_t sercomNum, uint32_t dataInPad, uint32_t dataOutPad) noexcept;
#else
	explicit SharedSpiDevice(uint8_t spiInstanceNum) noexcept;
#endif

	// Get ownership of this SPI, return true if successful
	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }

	// Release ownership of this SPI
	void Release() noexcept { mutex.Release(); }

private:
	Mutex mutex;
};

#endif /* SRC_HARDWARE_SPI_SHAREDSPIDEVICE_H_ */
