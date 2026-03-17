/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#include "SharedSpiDevice.h"

// SharedSpiDevice members

SharedSpiDevice::SharedSpiDevice(const SpiParameters& params) noexcept : SpiDevice(params)
{
	mutex.Create("SPI");
}

// End
