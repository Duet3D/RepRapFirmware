/*
 * LinearAnalogSensor.cpp
 *
 *  Created on: 16 Apr 2019
 *      Author: David
 */

#include "LinearAnalogSensor.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Pins.h"
#include "RepRap.h"
#include "Platform.h"

LinearAnalogSensor::LinearAnalogSensor(unsigned int sensorNum)
	: SensorWithPort(sensorNum, "Linear analog"), lowTemp(DefaultLowTemp), highTemp(DefaultHighTemp), filtered(true), adcFilterChannel(-1)
{
	CalcDerivedParameters();
}

GCodeResult LinearAnalogSensor::Configure(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	if (!ConfigurePort(gb, reply, PinAccess::readAnalog, seen))
	{
		return GCodeResult::error;
	}

	gb.TryGetFValue('L', lowTemp, seen);
	gb.TryGetFValue('H', highTemp, seen);
	TryConfigureSensorName(gb, seen);
	if (gb.Seen('F'))
	{
		seen = true;
		filtered = gb.GetIValue() >= 1;
	}

	if (seen)
	{
		CalcDerivedParameters();
	}
	else
	{
		CopyBasicDetails(reply);
		reply.catf(", %sfiltered, range %.1f to %.1f", (filtered) ? "" : "un", (double)lowTemp, (double)highTemp);
	}
	return GCodeResult::ok;
}

void LinearAnalogSensor::Init()
{
	if (adcFilterChannel >= 0)
	{
		reprap.GetPlatform().GetAdcFilter(adcFilterChannel).Init(0);
	}
}

TemperatureError LinearAnalogSensor::TryGetTemperature(float& t)
{
	if (filtered && adcFilterChannel >= 0)
	{
		const volatile ThermistorAveragingFilter& tempFilter = reprap.GetPlatform().GetAdcFilter(adcFilterChannel);
		if (tempFilter.IsValid())
		{
			const int32_t averagedTempReading = tempFilter.GetSum()/(ThermistorAverageReadings >> AdcOversampleBits);
			t = (averagedTempReading * linearIncreasePerCount) + lowTemp;
			return TemperatureError::success;
		}
		else
		{
			t = BadErrorTemperature;
			return TemperatureError::notReady;
		}
	}
	else
	{
		t = (port.ReadAnalog() * linearIncreasePerCount) + lowTemp;
		return TemperatureError::success;
	}
}

void LinearAnalogSensor::CalcDerivedParameters()
{
	adcFilterChannel = reprap.GetPlatform().GetAveragingFilterIndex(port);
	linearIncreasePerCount = (highTemp - lowTemp)/((filtered) ? FilteredAdcRange : UnfilteredAdcRange);
}

// End
