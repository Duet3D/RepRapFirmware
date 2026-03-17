/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#include "SharedSpiDevice.h"

// SharedSpiDevice members

SharedSpiDevice::SharedSpiDevice(uint8_t sercomNum) noexcept
	: SpiDevice(sercomNum)
{
	mutex.Create("SPI");
}

// End
