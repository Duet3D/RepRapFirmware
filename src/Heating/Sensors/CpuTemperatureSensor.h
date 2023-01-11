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
	explicit CpuTemperatureSensor(unsigned int sensorNum) noexcept;

	void Poll() noexcept override;
	const char *_ecv_array GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *_ecv_array TypeName = "mcutemp";
};

#endif

#endif /* SRC_HEATING_SENSORS_CPUTEMPERATURESENSOR_H_ */
