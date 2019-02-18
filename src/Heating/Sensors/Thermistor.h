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
	Thermistor(unsigned int channel, bool p_isPT1000);						// create an instance with default values
	GCodeResult Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) override; // configure the sensor from M305 parameters
	void Init() override;

protected:
	TemperatureError TryGetTemperature(float& t) override;

private:
	// For the theory behind ADC oversampling, see http://www.atmel.com/Images/doc8003.pdf
	static constexpr unsigned int AdcOversampleBits = 2;					// we use 2-bit oversampling

	void CalcDerivedParameters();											// calculate shA and shB

	// The following are configurable parameters
	unsigned int thermistorInputChannel;
	float r25, beta, shC, seriesR;											// parameters declared in the M305 command
	bool isPT1000;															// true if it is a PT1000 sensor, not a thermistor

#if !HAS_VREF_MONITOR
	int8_t adcLowOffset, adcHighOffset;										// ADC low and high end offsets
#endif

	// The following are derived from the configurable parameters
	float shA, shB;															// derived parameters

	static constexpr unsigned int AdcBits = 12;								// the ADCs in the SAM processors are 12-bit
	static constexpr int32_t AdcRange = 1 << (AdcBits + AdcOversampleBits);	// The readings we pass in should be in range 0..(AdcRange - 1)
};

#endif /* SRC_HEATING_THERMISTOR_H_ */
