/*
 * LinearAnalogSensor.cpp
 *
 *  Created on: 16 Apr 2019
 *      Author: David
 */

#include "LinearAnalogSensor.h"
#include "GCodes/GCodeBuffer.h"
#include "Pins.h"
#include "RepRap.h"
#include "Platform.h"

LinearAnalogSensor::LinearAnalogSensor(unsigned int channel)
	: TemperatureSensor(channel, "Linear analog"), thermistorInputChannel(channel - FirstLinearAnalogChannel), lowTemp(DefaultLowTemp), highTemp(DefaultHighTemp), filtered(true)
{
	CalcDerivedParameters();
}

GCodeResult LinearAnalogSensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	if (mCode == 305)
	{
		bool seen = false;
		gb.TryGetFValue('L', lowTemp, seen);
		gb.TryGetFValue('H', highTemp, seen);
		TryConfigureHeaterName(gb, seen);
		if (gb.Seen('F'))
		{
			seen = true;
			filtered = gb.GetIValue() >= 1;
		}

		if (seen)
		{
			CalcDerivedParameters();
		}
		else if (!gb.Seen('X'))
		{
			CopyBasicHeaterDetails(heater, reply);
			reply.catf(", %sfiltered, range %.1f to %.1f", (filtered) ? "" : "un", (double)lowTemp, (double)highTemp);
		}
	}
	return GCodeResult::ok;
}

void LinearAnalogSensor::Init()
{
	reprap.GetPlatform().GetAdcFilter(thermistorInputChannel).Init(PinToAdcChannel(TEMP_SENSE_PINS[thermistorInputChannel]));
}

TemperatureError LinearAnalogSensor::TryGetTemperature(float& t)
{
	if (filtered)
	{
		const volatile ThermistorAveragingFilter& tempFilter = reprap.GetPlatform().GetAdcFilter(thermistorInputChannel);
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
		t = (AnalogInReadChannel(PinToAdcChannel(TEMP_SENSE_PINS[thermistorInputChannel])) * linearIncreasePerCount) + lowTemp;
		return TemperatureError::success;
	}
}

void LinearAnalogSensor::CalcDerivedParameters()
{
	linearIncreasePerCount = (highTemp - lowTemp)/((filtered) ? FilteredAdcRange : UnfilteredAdcRange);
}

// End
