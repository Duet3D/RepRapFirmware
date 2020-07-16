/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 28 Jul 2019
 *      Author: David
 */

#include "SharedSpiClient.h"
#include "SharedSpiDevice.h"
#include <Hardware/IoPorts.h>

// SharedSpiDevice class members
SharedSpiClient::SharedSpiClient(SharedSpiDevice& dev, uint32_t clockFreq, SpiMode m, Pin p, bool polarity) noexcept
	: device(dev), clockFrequency(clockFreq), csPin(p), mode(m), csActivePolarity(polarity)
{
	InitCsPin();
}

void SharedSpiClient::InitCsPin() const noexcept
{
	if (csPin != NoPin)
	{
		IoPort::SetPinMode(csPin, (csActivePolarity) ? OUTPUT_LOW : OUTPUT_HIGH);
	}
}

// Get ownership of this SPI, return true if successful
bool SharedSpiClient::Select(uint32_t timeout) const noexcept
{
	const bool ok = device.Take(timeout);
	if (ok)
	{
		device.SetClockFrequencyAndMode(clockFrequency, mode);
		IoPort::WriteDigital(csPin, csActivePolarity);
	}
	return ok;
}

void SharedSpiClient::Deselect() const noexcept
{
	IoPort::WriteDigital(csPin, !csActivePolarity);
	device.Disable();
	device.Release();
}

bool SharedSpiClient::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
{
	return device.TransceivePacket(tx_data, rx_data, len);
}

// End
