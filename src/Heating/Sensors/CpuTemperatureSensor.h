/*
 * CpuTemperatureSensor.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_CPUTEMPERATURESENSOR_H_
#define SRC_HEATING_SENSORS_CPUTEMPERATURESENSOR_H_

#include "TemperatureSensor.h"

#if HAS_CPU_TEMP_SENSOR

class CpuTemperatureSensor : public TemperatureSensor
{
public:
	CpuTemperatureSensor(unsigned int sensorNum) noexcept;

	static constexpr const char *TypeName = "mcutemp";

	void Poll() noexcept override;
};

#endif

#endif /* SRC_HEATING_SENSORS_CPUTEMPERATURESENSOR_H_ */
