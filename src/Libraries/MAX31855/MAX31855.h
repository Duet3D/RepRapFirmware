#ifndef MAX31855_H
#define MAX31855_H

#include "Arduino.h"
#include "SharedSpi.h"		// for gspi_device

enum MAX31855_error
{
	MAX31855_OK      = 0,	// Success
	MAX31855_ERR_SCV = 1,	// Thermocouple is shorted to Vcc
	MAX31855_ERR_SCG = 2,	// Thermocouple is shorted to ground
	MAX31855_ERR_OC  = 3,	// Thermocouple is open
	MAX31855_ERR_TMO = 4,	// Timeout waiting on I/O (bus busy)
	MAX31855_ERR_IO  = 5,   // Chip not sending output?  CS not hooked up?
	MAX31855_GSPI_BUSY		// General SPI bus is busy
};

class MAX31855
{
public:
	MAX31855() {}
	MAX31855_error getTemperature(float *temp) const;
	void Init(uint8_t cs);
	const char* errorStr(MAX31855_error err) const;

private:
	sspi_device device;
};

#endif //MAX31855_H
