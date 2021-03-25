/*
 * SharedSpiDevice.h
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDSPI_SHAREDSPIDEVICE_H_
#define SRC_HARDWARE_SHAREDSPI_SHAREDSPIDEVICE_H_

#include <RepRapFirmware.h>
#include <RTOSIface/RTOSIface.h>
#include "SpiMode.h"

class SharedSpiDevice
{
public:
	SharedSpiDevice(uint8_t sercomNum) noexcept;

	void Disable() const noexcept;
	void Enable() const noexcept;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept;

	// Send and receive data returning true if successful. Caller must already own the mutex and have asserted CS.
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;

	// Get ownership of this SPI, return true if successful
	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }

	// Release ownership of this SPI
	void Release() noexcept { mutex.Release(); }

	static void Init() noexcept;
	static SharedSpiDevice& GetMainSharedSpiDevice() noexcept { return *mainSharedSpiDevice; }

private:
	bool waitForTxReady() const noexcept;
	bool waitForTxEmpty() const noexcept;
	bool waitForRxReady() const noexcept;

#if SAME5x
	Sercom * const hardware;
#elif USART_SPI
	Usart * const hardware;
#else
	Spi * const hardware;
#endif

	Mutex mutex;

	static SharedSpiDevice *mainSharedSpiDevice;
};

#endif /* SRC_HARDWARE_SHAREDSPI_SHAREDSPIDEVICE_H_ */
