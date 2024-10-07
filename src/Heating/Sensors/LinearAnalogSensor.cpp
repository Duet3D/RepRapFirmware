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

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#if SAME5x
constexpr unsigned int AdcBits = AnalogIn::AdcBits;
#else
constexpr unsigned int AdcBits = LegacyAnalogIn::AdcBits;
#endif

// ADC resolution
// For the theory behind ADC oversampling, see http://www.atmel.com/Images/doc8003.pdf
static constexpr unsigned int AdcOversampleBits = 2;								// we use 2-bit oversampling when using a filtered channel
static constexpr int32_t UnfilteredAdcRange = 1u << AdcBits;						// The readings we pass in should be in range 0..(AdcRange - 1)
static constexpr int32_t FilteredAdcRange = 1u << (AdcBits + AdcOversampleBits);	// The readings we pass in should be in range 0..(AdcRange - 1)

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor LinearAnalogSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new LinearAnalogSensor(sensorNum); } );

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
	if (gb.Seen('F'))
	{
		changed = true;
		filtered = gb.GetIValue() >= 1;
	}
	const bool portOrFilterChanged = changed;

	gb.TryGetFValue('B', lowTemp, changed);
	gb.TryGetFValue('C', highTemp, changed);
	ConfigureCommonParameters(gb, changed);

	if (changed)
	{
		const bool wasFiltered = filtered;
		CalcDerivedParameters();
		if (portOrFilterChanged)
		{
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
	}
	else
	{
		CopyBasicDetails(reply);
		reply.catf(", %sfiltered, range %.1f to %.1f", (filtered) ? "" : "un", (double)lowTemp, (double)highTemp);
	}
	return GCodeResult::ok;
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult LinearAnalogSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	if (!ConfigurePort(parser, reply, PinAccess::readAnalog, seen))
	{
		return GCodeResult::error;
	}

	ConfigureCommonParameters(parser, seen);
	if (parser.GetFloatParam('B', lowTemp))
	{
		seen = true;
	}
	if (parser.GetFloatParam('C', highTemp))
	{
		seen = true;
	}
	if (parser.GetBoolParam('F', filtered))
	{
		seen = true;
	}
	if (seen)
	{
		const bool wasFiltered = filtered;
		CalcDerivedParameters();
		if (adcFilterChannel < 0 && wasFiltered)
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

#endif

void LinearAnalogSensor::Poll() noexcept
{
	if (filtered && adcFilterChannel >= 0)
	{
		const volatile ThermistorAveragingFilter& tempFilter = reprap.GetPlatform().GetAdcFilter(adcFilterChannel);
		if (tempFilter.IsValid())
		{
			const int32_t averagedTempReading = tempFilter.GetSum()/(ThermistorAverageReadings >> AdcOversampleBits);
			SetResult((averagedTempReading * linearIncreasePerCount) + lowTemp, TemperatureError::ok);
		}
		else
		{
			SetResult(TemperatureError::notReady);
		}
	}
	else
	{
		SetResult((port.ReadAnalog() * linearIncreasePerCount) + lowTemp, TemperatureError::ok);
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
