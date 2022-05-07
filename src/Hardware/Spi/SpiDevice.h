/*
 * SpiDevice.h
 *
 *  Created on: 7 May 2022
 *      Author: David
 */

#ifndef SRC_HARDWARE_SPI_SPIDEVICE_H_
#define SRC_HARDWARE_SPI_SPIDEVICE_H_

#include <RepRapFirmware.h>
#include "SpiMode.h"

// This class represents a master SPI interface, but not the associated CS pin(s).
// It is used as the base class for SharedSpiDevice. It can also be used by itself to control a non-shared SPI master.
class SpiDevice
{
public:
	SpiDevice(uint8_t sercomNum) noexcept;

	void Disable() const noexcept;
	void Enable() const noexcept;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept;

	// Send and receive data returning true if successful.
	// If this is a shared SPI device then the caller must already own the mutex.
	// Either way, caller must already have asserted CS for the selected SPI slave.
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;

#if SAME5x
	bool TransceivePacketNineBit(const uint16_t *tx_data, uint16_t *rx_data, size_t len) const noexcept;
#endif

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
};

#endif /* SRC_HARDWARE_SPI_SPIDEVICE_H_ */
