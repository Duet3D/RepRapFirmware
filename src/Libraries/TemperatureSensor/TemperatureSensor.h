#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include "TemperatureError.h"		// for result codes
#include "Arduino.h"
#include "SharedSpi.h"				// for sspi_device

class TemperatureSensor
{
public:
	TemperatureSensor() {}
	void InitThermocouple(uint8_t cs);
	void InitRtd(uint8_t cs);
	TemperatureError GetThermocoupleTemperature(float *temp);
	TemperatureError GetRtdTemperature(float *temp);

private:
	TemperatureError DoSpiTransaction(const uint8_t dataOut[], size_t nbytes, uint32_t& rslt) const;
	TemperatureError TryInitRtd() const;

	sspi_device device;
	uint32_t lastReadingTime;
	float lastTemperature;
	TemperatureError lastResult;
};

#endif // TEMPERATURESENSOR_H
