/*
 * TmcDriverTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "TmcDriverTemperatureSensor.h"
#include "Platform.h"
#include "RepRap.h"

#if HAS_SMART_DRIVERS

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int sensorNum, unsigned int chan) noexcept
	: TemperatureSensor(sensorNum, "Stepper driver temperature warnings"), channel(chan)
{
}

void TmcDriverTemperatureSensor::Poll() noexcept
{
	SetResult(reprap.GetPlatform().GetTmcDriversTemperature(channel), TemperatureError::success);
}

#endif

// End
