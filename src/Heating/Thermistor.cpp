/*
 * Thermistor.cpp
 *
 *  Created on: 10 Nov 2016
 *      Author: David
 */

#include "Thermistor.h"
#include "Pins.h"
#include "Configuration.h"

// The Steinhart-Hart equation for thermistor resistance is:
// 1/T = A + B ln(R) + C [ln(R)]^3
//
// The simplified (beta) equation assumes C=0 and is:
// 1/T = A + (1/Beta) ln(R)
//
// The parameters that can be configured in RRF are R25 (the resistance at 25C), Beta, and optionally C.

// Create an instance with default values
Thermistor::Thermistor() : adcLowOffset(0), adcHighOffset(0)
{
	SetParameters(EXT_R25, EXT_BETA, EXT_SHC, THERMISTOR_SERIES_RS);
}

// Initialise the instance
void Thermistor::SetParameters(float p_r25, float p_beta, float p_shC, float p_seriesR)
{
	r25 = p_r25;
	beta = p_beta;
	shC = p_shC;
	seriesR = p_seriesR;
	CalcDerivedParameters();
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
	const double lnR25 = log(r25);
	shA = 1.0/(25.0 - ABS_ZERO) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;
}

// End
