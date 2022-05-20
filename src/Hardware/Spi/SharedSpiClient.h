/*
 * SharedSpiDevice.h
 *
 *  Created on: 1 Jul 2019
 *      Author: David
 *
 *  This currently supports only a single SPI channel. To support multiple SPI channels we would need to make the underlying SERCOM device
 *  configured in SPI mode a separate object, and have a pointer or reference to it in SharedSpiDevice.
 */

#ifndef SRC_HARDWARE_SPI_SHAREDSPICLIENT_H_
#define SRC_HARDWARE_SPI_SHAREDSPICLIENT_H_

#include <RepRapFirmware.h>
#include "SpiMode.h"
#include <RTOSIface/RTOSIface.h>

class SharedSpiDevice;

class SharedSpiClient
{
public:
	SharedSpiClient(SharedSpiDevice& dev, uint32_t clockFreq, SpiMode m, Pin p, bool polarity) noexcept;

	void SetCsPin(Pin p) noexcept { csPin = p; InitCsPin(); }
	void SetCsPolarity(bool b) noexcept { csActivePolarity = b; }
	void SetClockFrequency(uint32_t clockFreq) noexcept { clockFrequency = clockFreq; }

	uint32_t GetFrequency() const noexcept { return clockFrequency; }

	bool Select(uint32_t timeout = Mutex::TimeoutUnlimited) const noexcept;					// get SPI ownership and select the device, return true if successful
	void Deselect() const noexcept;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;
	bool ReadPacket(uint8_t *rx_data, size_t len) const noexcept { return TransceivePacket(nullptr, rx_data, len); }
	bool WritePacket(const uint8_t *tx_data, size_t len) const noexcept { return TransceivePacket(tx_data, nullptr, len); }

private:
	void InitCsPin() const noexcept;

	SharedSpiDevice& device;
	uint32_t clockFrequency;
	Pin csPin;
	SpiMode mode;
	bool csActivePolarity;
};

#endif /* SRC_HARDWARE_SPI_SHAREDSPICLIENT_H_ */
