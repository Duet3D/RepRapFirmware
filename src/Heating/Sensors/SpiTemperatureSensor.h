/*
 * SpiTemperatureSensor.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SPITEMPERATURESENSOR_H_
#define SRC_HEATING_SPITEMPERATURESENSOR_H_

#include "SensorWithPort.h"
#include "SharedSpi.h"				// for sspi_device

class SpiTemperatureSensor : public SensorWithPort
{
protected:
	SpiTemperatureSensor(unsigned int sensorNum, const char *name, uint8_t spiMode, uint32_t clockFrequency) noexcept;
	bool ConfigurePort(GCodeBuffer& gb, const StringRef& reply, bool& seen);
	void InitSpi() noexcept;
	TemperatureError DoSpiTransaction(const uint8_t dataOut[], size_t nbytes, uint32_t& rslt) const noexcept
		pre(nbytes <= 8);

	sspi_device device;
	uint32_t lastReadingTime;
	float lastTemperature;
	TemperatureError lastResult;
};

#endif /* SRC_HEATING_SPITEMPERATURESENSOR_H_ */
