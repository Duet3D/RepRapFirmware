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

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int channel) : TemperatureSensor(channel, "TMC2660 temperature warnings")
{
}

void TmcDriverTemperatureSensor::Init()
{
}

TemperatureError TmcDriverTemperatureSensor::TryGetTemperature(float& t)
{
	t = reprap.GetPlatform().GetTmcDriversTemperature(GetSensorChannel() - FirstTmcDriversSenseChannel);
	return TemperatureError::success;
}

#endif

// End
