/*
 * SpiTemperatureSensor.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SPITEMPERATURESENSOR_H_
#define SRC_HEATING_SPITEMPERATURESENSOR_H_

#include "SensorWithPort.h"

#if SUPPORT_SPI_SENSORS

#include <Hardware/SharedSpi/SharedSpiClient.h>

class SpiTemperatureSensor : public SensorWithPort
{
protected:
	SpiTemperatureSensor(unsigned int sensorNum, const char *name, SpiMode spiMode, uint32_t clockFrequency) noexcept;

	bool ConfigurePort(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	bool ConfigurePort(const CanMessageGenericParser& parser, const StringRef& reply, bool& seen) noexcept;
#endif

	void InitSpi() noexcept;
	TemperatureError DoSpiTransaction(const uint8_t dataOut[], size_t nbytes, uint32_t& rslt) const noexcept
		pre(nbytes <= 8);

	SharedSpiClient device;
	uint32_t lastReadingTime;
	float lastTemperature;
	TemperatureError lastResult;
};

#endif // SUPPORT_SPI_SENSORS

#endif /* SRC_HEATING_SPITEMPERATURESENSOR_H_ */
