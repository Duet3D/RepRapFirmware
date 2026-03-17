/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#include "SharedSpiDevice.h"

// SharedSpiDevice members

#if SAME5x || SAMC21
SharedSpiDevice::SharedSpiDevice(const SpiParameters& params) noexcept : SpiDevice(params)
#else
SharedSpiDevice::SharedSpiDevice(uint8_t spiInstanceNum) noexcept : SpiDevice(spiInstanceNum)
#endif
{
	mutex.Create("SPI");
}

// End
