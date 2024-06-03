/*
 * TmcDriverTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "TmcDriverTemperatureSensor.h"
#include <Movement/Move.h>
#include <Platform/RepRap.h>

#if HAS_SMART_DRIVERS

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int sensorNum, unsigned int chan) noexcept
	: TemperatureSensor(sensorNum, "Stepper driver temperature warnings"), channel(chan)
{
}

const char *TmcDriverTemperatureSensor::GetShortSensorType() const noexcept
{
#ifdef DUET_NG
	return (channel == 1) ? DuexTypeShortName : PrimaryTypeName;
#else
	return PrimaryTypeName;
#endif
}

void TmcDriverTemperatureSensor::Poll() noexcept
{
	SetResult(reprap.GetMove().GetTmcDriversTemperature(channel), TemperatureError::ok);
}

#endif

// End
