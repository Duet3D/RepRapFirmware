/*
 * Thermistor.h
 *
 *  Created on: 10 Nov 2016
 *      Author: David
 */

#ifndef SRC_HEATING_THERMISTOR_H_
#define SRC_HEATING_THERMISTOR_H_

#include "SensorWithPort.h"

// The Steinhart-Hart equation for thermistor resistance is:
// 1/T = A + B ln(R) + C [ln(R)]^3
//
// The simplified (beta) equation assumes C=0 and is:
// 1/T = A + (1/Beta) ln(R)
//
// The parameters that can be configured in RRF are R25 (the resistance at 25C), Beta, and optionally C.

class Thermistor : public SensorWithPort
{
public:
	Thermistor(unsigned int sensorNum, bool p_isPT1000) noexcept;					// create an instance with default values
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) override THROWS(GCodeException); // configure the sensor from M305 parameters
	void Poll() noexcept override;
	const char *GetShortSensorType() const noexcept override { return (isPT1000) ? TypeNamePT1000 : TypeNameThermistor; }

	static constexpr const char *TypeNameThermistor = "thermistor";
	static constexpr const char *TypeNamePT1000 = "pt1000";

private:
	void CalcDerivedParameters() noexcept;											// calculate shA and shB
	int32_t GetRawReading(bool& valid) const noexcept;								// get the ADC reading

	// The following are configurable parameters
	float r25, beta, shC, seriesR;													// parameters declared in the M305 command
	int8_t adcFilterChannel;
	bool isPT1000;																	// true if it is a PT1000 sensor, not a thermistor
	int8_t adcLowOffset, adcHighOffset;

	// The following are derived from the configurable parameters
	float shA, shB;																	// derived parameters
};

#endif /* SRC_HEATING_THERMISTOR_H_ */
