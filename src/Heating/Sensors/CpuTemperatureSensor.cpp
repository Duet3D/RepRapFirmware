/*
 * CpuTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "CpuTemperatureSensor.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>

#if HAS_CPU_TEMP_SENSOR

CpuTemperatureSensor::CpuTemperatureSensor(unsigned int sensorNum) noexcept : TemperatureSensor(sensorNum, "Microcontroller embedded temperature sensor")
{
}

void CpuTemperatureSensor::Poll() noexcept
{
	const MinCurMax temperatures = reprap.GetPlatform().GetMcuTemperatures();
	SetResult(temperatures.current, TemperatureError::ok);
}

#endif

// End
