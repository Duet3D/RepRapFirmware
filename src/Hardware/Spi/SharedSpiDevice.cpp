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

// Static members

SharedSpiDevice *SharedSpiDevice::mainSharedSpiDevice = nullptr;

void SharedSpiDevice::Init() noexcept
{
#if SAME5x
	pinMode(SharedSpiMosiPin, INPUT_PULLDOWN);
	pinMode(SharedSpiMisoPin, INPUT_PULLDOWN);
	pinMode(SharedSpiSclkPin, INPUT_PULLDOWN);
	SetPinFunction(SharedSpiMosiPin, SharedSpiPinFunction);
	SetPinFunction(SharedSpiMisoPin, SharedSpiPinFunction);
	SetPinFunction(SharedSpiSclkPin, SharedSpiPinFunction);
	SetDriveStrength(SharedSpiMosiPin, 2);
	SetDriveStrength(SharedSpiSclkPin, 2);								// some devices (e.g. TFT LCD font chip) need fast rise and fall times
	mainSharedSpiDevice = new SharedSpiDevice(SharedSpiSercomNumber);
#elif USART_SPI
	SetPinFunction(APIN_USART_SSPI_SCK, USARTSPISckPeriphMode);
	SetPinFunction(APIN_USART_SSPI_MOSI, USARTSPIMosiPeriphMode);
	SetPinFunction(APIN_USART_SSPI_MISO, USARTSPIMisoPeriphMode);
	mainSharedSpiDevice = new SharedSpiDevice(0);
#else
	ConfigurePin(g_APinDescription[APIN_SHARED_SPI_SCK]);
	ConfigurePin(g_APinDescription[APIN_SHARED_SPI_MOSI]);
	ConfigurePin(g_APinDescription[APIN_SHARED_SPI_MISO]);
	mainSharedSpiDevice = new SharedSpiDevice(0);
#endif
}

// End
