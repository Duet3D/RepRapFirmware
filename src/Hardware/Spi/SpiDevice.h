/*
 * SpiDevice.h
 *
 *  Created on: 7 May 2022
 *      Author: David
 */

#ifndef SRC_HARDWARE_SPI_SPIDEVICE_H_
#define SRC_HARDWARE_SPI_SPIDEVICE_H_

#include <RepRapFirmware.h>
#include <RTOSIface/RTOSIface.h>
#include <SPI/SpiMode.h>
#include <SPI/SpiParameters.h>

#if SAME5x || SAMC21
# include <DmacManager.h>
#endif

// This class represents a master SPI interface, but not the associated CS pin(s).
// It is used as the base class for SharedSpiDevice. It can also be used by itself to control a non-shared SPI master.
class SpiDevice
{
public:
	explicit SpiDevice(const SpiParameters& params) noexcept;

	void Disable() const noexcept;
	void Enable() const noexcept;

	// Set the clock frequency, SPI mode and character length. 9-bit mode is currently only implemented on the SAME5x.
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode
#if SAME5x || SAMC21
									, bool nineBits
#endif
								 ) const noexcept;

	// Send and receive data returning true if successful.
	// If this is a shared SPI device then the caller must already own the mutex.
	// Either way, caller must already have asserted CS for the selected SPI slave.
	bool TransceivePacket(const uint8_t *_ecv_array null tx_data, uint8_t *_ecv_array null rx_data, size_t len) noexcept;

#if SAME5x || SAMC21
	bool TransceivePacketNineBit(const uint16_t *_ecv_array null tx_data, uint16_t *_ecv_array null rx_data, size_t len) noexcept;

	static void DmaComplete(CallbackParameter param, DmaCallbackReason reason) noexcept;
#endif

private:
	bool waitForTxReady() const noexcept;
	bool waitForTxEmpty() const noexcept;
	bool waitForRxReady() const noexcept;

#if SAME5x || SAMC21
	void DmaComplete(DmaCallbackReason reason) noexcept;
#endif

#if SAME5x || SAMC21
	Sercom * const hardware;
	const uint8_t sercomNumber;
	TaskBase *null waitingTask = nullptr;
	DmaChannel dmaChanTx;
	DmaPriority dmaPrioTx;
#elif SAME70 || SAM4E || SAM4S
	Usart * const hardware;
#else
# error Unsupported configuration
#endif
};

#endif /* SRC_HARDWARE_SPI_SPIDEVICE_H_ */
