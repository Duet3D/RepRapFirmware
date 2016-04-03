#ifndef MAX31855_H
#define MAX31855_H

#include "Arduino.h"
#include "spi_master.h"

enum MAX31855_error {
	MAX31855_OK      = 0,  // Success
	MAX31855_ERR_SCV = 1,  // Thermocouple is shorted to Vcc
	MAX31855_ERR_SCG = 2,  // Thermocouple is shorted to ground
	MAX31855_ERR_OC  = 3,  // Thermocouple is open
	MAX31855_ERR_TMO = 4,  // Timeout waiting on I/O (bus busy)
	MAX31855_ERR_IO  = 5   // Chip not sending output?  CS not hooked up?
};

class MAX31855 {

public:
	MAX31855(uint8_t cs = 0, bool deferInit = true);
	MAX31855_error getTemperature(float *temp) const;
	void Init(uint8_t cs);
	const char* errorStr(MAX31855_error err) const;

private:
	spi_status_t readRaw(uint16_t *r) const;
	bool initialized;
	struct spi_device device;
};

#endif //MAX31855_H
