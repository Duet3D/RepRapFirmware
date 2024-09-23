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
	explicit SharedSpiDevice(uint8_t sercomNum) noexcept;

	// Get ownership of this SPI, return true if successful
	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }

	// Release ownership of this SPI
	void Release() noexcept { mutex.Release(); }

	static void Init() noexcept;
	static SharedSpiDevice& GetMainSharedSpiDevice() noexcept { return *mainSharedSpiDevice; }

private:
	Mutex mutex;

	static SharedSpiDevice *_ecv_null mainSharedSpiDevice;
};

#endif /* SRC_HARDWARE_SPI_SHAREDSPIDEVICE_H_ */
