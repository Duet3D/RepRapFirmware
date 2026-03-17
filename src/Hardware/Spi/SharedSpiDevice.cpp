/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#include "SharedSpiDevice.h"

// SharedSpiDevice members

#if SAME5x || SAMC21
SharedSpiDevice::SharedSpiDevice(uint8_t sercomNum, DmaChannel dmaChan, DmaPriority dmaPrio, uint32_t dataInPad, uint32_t dataOutPad) noexcept : SpiDevice(sercomNum, dmaChan, dmaPrio, dataInPad, dataOutPad)
#else
SharedSpiDevice::SharedSpiDevice(uint8_t spiInstanceNum) noexcept : SpiDevice(spiInstanceNum)
#endif
{
	mutex.Create("SPI");
}

// End
