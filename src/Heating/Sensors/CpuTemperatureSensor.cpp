/*
 * CpuTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "CpuTemperatureSensor.h"
#include "Platform.h"
#include "RepRap.h"

#if HAS_CPU_TEMP_SENSOR

CpuTemperatureSensor::CpuTemperatureSensor(unsigned int sensorNum) noexcept : TemperatureSensor(sensorNum, "Microcontroller embedded temperature sensor")
{
}

void CpuTemperatureSensor::Poll() noexcept
{
	const MinMaxCurrent temperatures = reprap.GetPlatform().GetMcuTemperatures();
	SetResult(temperatures.current, TemperatureError::success);
}

#endif

// End
