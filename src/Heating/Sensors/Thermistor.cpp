/*
 * Thermistor.cpp
 *
 *  Created on: 10 Nov 2016
 *      Author: David
 */

#include "Thermistor.h"
#include "Platform.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"

// The Steinhart-Hart equation for thermistor resistance is:
// 1/T = A + B ln(R) + C [ln(R)]^3
//
// The simplified (beta) equation assumes C=0 and is:
// 1/T = A + (1/Beta) ln(R)
//
// The parameters that can be configured in RRF are R25 (the resistance at 25C), Beta, and optionally C.

// Create an instance with default values
Thermistor::Thermistor(unsigned int channel)
	: TemperatureSensor(channel - FirstThermistorChannel, "Thermistor"), adcLowOffset(0), adcHighOffset(0)
{
	r25 = (channel == FirstThermistorChannel) ? BED_R25 : EXT_R25;
	beta = (channel == FirstThermistorChannel) ? BED_BETA : EXT_BETA;
	shC = (channel == FirstThermistorChannel) ? BED_SHC : EXT_SHC;
	seriesR = THERMISTOR_SERIES_RS;
	CalcDerivedParameters();
}

void Thermistor::Init()
{
	reprap.GetPlatform().GetThermistorFilter(GetSensorChannel() - FirstThermistorChannel).Init((1 << AdcBits) - 1);
}

// Configure the temperature sensor
bool Thermistor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, StringRef& reply, bool& error)
{
	bool seen = false;
	if (mCode == 305)
	{
		gb.TryGetFValue('B', beta, seen);
		if (seen)
		{
			shC = 0.0;						// if user changes B and doesn't define C, assume C=0
		}
		gb.TryGetFValue('C', shC, seen);
		gb.TryGetFValue('T', r25, seen);
		gb.TryGetFValue('R', seriesR, seen);
		if (seen)
		{
			CalcDerivedParameters();
		}

		if (gb.Seen('L'))
		{
			adcLowOffset = (int8_t)constrain<int>(gb.GetIValue(), -100, 100);
			seen = true;
		}
		if (gb.Seen('H'))
		{
			adcHighOffset = (int8_t)constrain<int>(gb.GetIValue(), -100, 100);
			seen = true;
		}

		TryConfigureHeaterName(gb, seen);

		if (!seen && !gb.Seen('X'))
		{
			CopyBasicHeaterDetails(heater, reply);
			reply.catf(", T:%.1f B:%.1f C:%.2e R:%.1f L:%d H:%d",
				(double)r25, (double)beta, (double)shC, (double)seriesR, adcLowOffset, adcHighOffset);
		}
	}

	return seen;
}

// Get the temperature
TemperatureError Thermistor::GetTemperature(float& t)
{
	const volatile ThermistorAveragingFilter& filter = reprap.GetPlatform().GetThermistorFilter(GetSensorChannel() - FirstThermistorChannel);
	if (filter.IsValid())
	{
		const int32_t averagedReading = filter.GetSum()/(ThermistorAverageReadings >> Thermistor::AdcOversampleBits);
		const float temp = CalcTemperature(averagedReading);

		if (temp < MinimumConnectedTemperature)
		{
			// thermistor is disconnected
			t = ABS_ZERO;
			return TemperatureError::openCircuit;
		}

		t = temp;
		return TemperatureError::success;
	}

	// Filter is not ready yet
	t = BAD_ERROR_TEMPERATURE;
	return TemperatureError::busBusy;
}

// Calculate temperature from an ADC reading in the range 0..1
float Thermistor::CalcTemperature(int32_t adcReading) const
{
	const float denom = (float)(AdcRange  + (int)adcHighOffset - adcReading) - 0.5;
	if (denom <= 0.0)
	{
		return ABS_ZERO;
	}
	const float resistance = seriesR * ((float)(adcReading - (int)adcLowOffset) + 0.5)/denom;
	const float logResistance = log(resistance);
	const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;
	return (recipT > 0.0) ? (1.0/recipT) + ABS_ZERO : BAD_ERROR_TEMPERATURE;
}

// Calculate expected ADC reading at a particular temperature, rounded down as the ADC does
int32_t Thermistor::CalcAdcReading(float temperature) const
{
	const double bDFiv3c = shB/(3.0 * shC);
	const double halfY = (shA - 1.0/(temperature - ABS_ZERO))/(2.0 * shC);
	const double x = sqrt((bDFiv3c * bDFiv3c * bDFiv3c) + (halfY * halfY));
	const double oneThird = 1.0/3.0;
	const float resistance = exp(pow(x - halfY, oneThird) - pow(x + halfY, oneThird));
	const float fraction = resistance/(resistance + seriesR);
	const int32_t actualAdcRange = AdcRange  + (int)adcHighOffset - (int)adcLowOffset;
	const int32_t val = (int32_t)(fraction * (float)actualAdcRange) + (int)adcLowOffset;
	return constrain<int>(val, 0, AdcRange - 1);
}

// Calculate shA and shB from the other parameters
void Thermistor::CalcDerivedParameters()
{
	shB = 1.0/beta;
	const float lnR25 = logf(r25);
	shA = 1.0/(25.0 - ABS_ZERO) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;
}

// End
