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

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int sensorNum, unsigned int chan)
	: TemperatureSensor(sensorNum, "TMC2660 temperature warnings"), channel(chan)
{
}

TemperatureError TmcDriverTemperatureSensor::TryGetTemperature(float& t)
{
	t = reprap.GetPlatform().GetTmcDriversTemperature(channel);
	return TemperatureError::success;
}

#endif

// End
