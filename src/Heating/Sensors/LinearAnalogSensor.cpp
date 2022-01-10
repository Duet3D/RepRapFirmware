/*
 * LinearAnalogSensor.cpp
 *
 *  Created on: 16 Apr 2019
 *      Author: David
 */

#include "LinearAnalogSensor.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>

#include <AnalogIn.h>

#if SAME5x
using AnalogIn::AdcBits;
#else
using LegacyAnalogIn::AdcBits;
#endif

// ADC resolution
// For the theory behind ADC oversampling, see http://www.atmel.com/Images/doc8003.pdf
static constexpr unsigned int AdcOversampleBits = 2;								// we use 2-bit oversampling when using a filtered channel
static constexpr int32_t UnfilteredAdcRange = 1u << AdcBits;						// The readings we pass in should be in range 0..(AdcRange - 1)
static constexpr int32_t FilteredAdcRange = 1u << (AdcBits + AdcOversampleBits);	// The readings we pass in should be in range 0..(AdcRange - 1)

LinearAnalogSensor::LinearAnalogSensor(unsigned int sensorNum) noexcept
	: SensorWithPort(sensorNum, "Linear analog"), lowTemp(DefaultLowTemp), highTemp(DefaultHighTemp), filtered(true), adcFilterChannel(-1)
{
	CalcDerivedParameters();
}

GCodeResult LinearAnalogSensor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	if (!ConfigurePort(gb, reply, PinAccess::readAnalog, changed))
	{
		return GCodeResult::error;
	}

	gb.TryGetFValue('B', lowTemp, changed);
	gb.TryGetFValue('C', highTemp, changed);
	TryConfigureSensorName(gb, changed);
	if (gb.Seen('F'))
	{
		changed = true;
		filtered = gb.GetIValue() >= 1;
	}

	if (changed)
	{
		const bool wasFiltered = filtered;
		CalcDerivedParameters();
		if (adcFilterChannel >= 0)
		{
			reprap.GetPlatform().GetAdcFilter(adcFilterChannel).Init(0);
		}
		else if (wasFiltered)
		{
			reply.copy("filtering not supported on this port");
			return GCodeResult::warning;
		}
	}
	else
	{
		CopyBasicDetails(reply);
		reply.catf(", %sfiltered, range %.1f to %.1f", (filtered) ? "" : "un", (double)lowTemp, (double)highTemp);
	}
	return GCodeResult::ok;
}

void LinearAnalogSensor::Poll() noexcept
{
	if (filtered && adcFilterChannel >= 0)
	{
		const volatile ThermistorAveragingFilter& tempFilter = reprap.GetPlatform().GetAdcFilter(adcFilterChannel);
		if (tempFilter.IsValid())
		{
			const int32_t averagedTempReading = tempFilter.GetSum()/(ThermistorAverageReadings >> AdcOversampleBits);
			SetResult((averagedTempReading * linearIncreasePerCount) + lowTemp, TemperatureError::success);
		}
		else
		{
			SetResult(TemperatureError::notReady);
		}
	}
	else
	{
		SetResult((port.ReadAnalog() * linearIncreasePerCount) + lowTemp, TemperatureError::success);
	}
}

void LinearAnalogSensor::CalcDerivedParameters() noexcept
{
	adcFilterChannel = reprap.GetPlatform().GetAveragingFilterIndex(port);
	if (adcFilterChannel < 0)
	{
		filtered = false;
	}
	linearIncreasePerCount = (highTemp - lowTemp)/((filtered) ? FilteredAdcRange : UnfilteredAdcRange);
}

// End
