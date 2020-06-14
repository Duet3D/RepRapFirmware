/*
 * SharedSpiDevice.h
 *
 *  Created on: 1 Jul 2019
 *      Author: David
 *
 *  This currently supports only a single SPI channel. To support multiple SPI channels we would need to make the underlying SERCOM device
 *  configured in SPI mode a separate object, and have a pointer or reference to it in SharedSpiDevice.
 */

#ifndef SRC_HARDWARE_SHAREDSPICLIENT_H_
#define SRC_HARDWARE_SHAREDSPICLIENT_H_

#include "RepRapFirmware.h"
#include <RTOSIface/RTOSIface.h>

enum class SpiMode : uint8_t
{
	mode0 = 0, mode1, mode2, mode3
};

// Definitions for backwards compatibility with RRF code
constexpr SpiMode SPI_MODE_0 = SpiMode::mode0;
constexpr SpiMode SPI_MODE_1 = SpiMode::mode1;
constexpr SpiMode SPI_MODE_2 = SpiMode::mode2;
constexpr SpiMode SPI_MODE_3 = SpiMode::mode3;

class SharedSpiDevice
{
public:
	SharedSpiDevice(uint8_t sercomNum);

	void Disable() const;
	void Enable() const;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const;
	bool Take(uint32_t timeout) const { return mutex.Take(timeout); }					// get ownership of this SPI, return true if successful
	void Release() const { mutex.Release(); }

private:
	bool waitForTxReady() const;
	bool waitForTxEmpty() const;
	bool waitForRxReady() const;

	Sercom * const hardware;
	Mutex mutex;
};

class SharedSpiClient
{
public:
	SharedSpiClient(SharedSpiDevice& dev, uint32_t clockFreq, SpiMode m, bool polarity);

	void InitMaster();
	bool Select(uint32_t timeout) const;												// get SPI ownership and select the device, return true if successful
	void Deselect() const;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const;
	void SetCsPin(Pin p) { csPin = p; }

private:
	SharedSpiDevice& device;
	uint32_t clockFrequency;
	Pin csPin;
	SpiMode mode;
	bool csActivePolarity;
};

#endif /* SRC_HARDWARE_SHAREDSPICLIENT_H_ */
