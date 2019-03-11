/*
 * SpiTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "SpiTemperatureSensor.h"
#include "Tasks.h"

SpiTemperatureSensor::SpiTemperatureSensor(unsigned int channel, const char *name, unsigned int relativeChannel, uint8_t spiMode, uint32_t clockFrequency)
	: TemperatureSensor(channel, name)
{
	device.csPin = SpiTempSensorCsPins[relativeChannel];
	device.csPolarity = false;						// active low chip select
	device.spiMode = spiMode;
	device.clockFrequency = clockFrequency;
#if defined(__LPC17xx__)
    device.sspChannel = TempSensorSSPChannel;		// use SSP0 on LPC
#endif
	lastTemperature = 0.0;
	lastResult = TemperatureError::notInitialised;
}

void SpiTemperatureSensor::InitSpi()
{
	sspi_master_init(&device, 8);
	lastReadingTime = millis();
}

// Send and receive 1 to 8 bytes of data and return the result as a single 32-bit word
TemperatureError SpiTemperatureSensor::DoSpiTransaction(const uint8_t dataOut[], size_t nbytes, uint32_t& rslt) const
{
	uint8_t rawBytes[8];
	spi_status_t sts;
	{
		MutexLocker lock(Tasks::GetSpiMutex(), 10);
		if (!lock)
		{
			return TemperatureError::busBusy;
		}

		sspi_master_setup_device(&device);
		delayMicroseconds(1);
		sspi_select_device(&device);
		delayMicroseconds(1);

		sts = sspi_transceive_packet(dataOut, rawBytes, nbytes);

		delayMicroseconds(1);
		sspi_deselect_device(&device);
		delayMicroseconds(1);
	}

	if (sts != SPI_OK)
	{
		return TemperatureError::timeout;
	}

	rslt = rawBytes[0];
	for (size_t i = 1; i < nbytes; ++i)
	{
		rslt <<= 8;
		rslt |= rawBytes[i];
	}

	return TemperatureError::success;
}

// End
