/*
 * Thermistor.h
 *
 *  Created on: 10 Nov 2016
 *      Author: David
 */

#ifndef SRC_HEATING_THERMISTOR_H_
#define SRC_HEATING_THERMISTOR_H_

#include "TemperatureSensor.h"

// The Steinhart-Hart equation for thermistor resistance is:
// 1/T = A + B ln(R) + C [ln(R)]^3
//
// The simplified (beta) equation assumes C=0 and is:
// 1/T = A + (1/Beta) ln(R)
//
// The parameters that can be configured in RRF are R25 (the resistance at 25C), Beta, and optionally C.

class Thermistor : public TemperatureSensor
{
public:
	Thermistor(unsigned int channel);										// create an instance with default values
	bool Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, StringRef& reply, bool& error) override; // configure the sensor from M305 parameters
	void Init() override;
	TemperatureError GetTemperature(float& t) override;

private:
	float CalcTemperature(int32_t adcReading) const;						// calculate temperature from an ADC reading in the range 0..1
	int32_t CalcAdcReading(float temperature) const;						// calculate expected ADC reading at a particular temperature

	float GetR25() const { return r25; }
	float GetBeta() const { return beta; }
	float GetShc() const { return shC; }
	float GetSeriesR() const { return seriesR; }
	int8_t GetLowOffset() const { return adcLowOffset; }
	int8_t GetHighOffset() const { return adcHighOffset; }

	void SetLowOffset(int8_t p_offset) { adcLowOffset = p_offset; }
	void SetHighOffset(int8_t p_offset) { adcHighOffset = p_offset; }

	// For the theory behind ADC oversampling, see http://www.atmel.com/Images/doc8003.pdf
	static const unsigned int AdcOversampleBits = 2;						// we use 2-bit oversampling

	void CalcDerivedParameters();											// calculate shA and shB

	// The following are configurable parameters
	float r25, beta, shC, seriesR;											// parameters declared in the M305 command
	int8_t adcLowOffset, adcHighOffset;										// ADC low and high end offsets

	// The following are derived from the configurable parameters
	float shA, shB;															// derived parameters

	static const unsigned int AdcBits = 12;									// the ADCs in the SAM processors are 12-bit
	static const int32_t AdcRange = 1 << (AdcBits + AdcOversampleBits);		// The readings we pass in should be in range 0..(AdcRange - 1)
};

#endif /* SRC_HEATING_THERMISTOR_H_ */
