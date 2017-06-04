#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include "RepRapFirmware.h"
#include "TemperatureError.h"		// for result codes
#include "SharedSpi.h"				// for sspi_device

class TemperatureSensor
{
public:
	TemperatureSensor() {}
	void InitThermocouple(uint8_t cs);
	void InitRtd(uint8_t cs);
	void InitLinearAdc(uint8_t cs);
	TemperatureError GetThermocoupleTemperature(float& t);
	TemperatureError GetRtdTemperature(float& t);
	TemperatureError GetLinearAdcTemperature(float& t);

private:
	TemperatureError DoSpiTransaction(const uint8_t dataOut[], size_t nbytes, uint32_t& rslt) const;
	TemperatureError TryInitRtd() const;
	void TryGetLinearAdcTemperature();

	sspi_device device;
	uint32_t lastReadingTime;
	float lastTemperature;
	TemperatureError lastResult;

	static constexpr float MinLinearAdcTemp = 385.0 - (1600.0 - 385.0) * (4.0/16.0);
	static constexpr float LinearAdcDegCPerCount = (1600.0 - 385.0)/3200.0;
};

#endif // TEMPERATURESENSOR_H
