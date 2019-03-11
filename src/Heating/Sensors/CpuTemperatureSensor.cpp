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

CpuTemperatureSensor::CpuTemperatureSensor(unsigned int channel) : TemperatureSensor(channel, "microcontroller embedded temperature sensor")
{
}

void CpuTemperatureSensor::Init()
{
}

TemperatureError CpuTemperatureSensor::TryGetTemperature(float& t)
{
	float minT, maxT;
	reprap.GetPlatform().GetMcuTemperatures(minT, t, maxT);
	return TemperatureError::success;
}

#endif

// End
