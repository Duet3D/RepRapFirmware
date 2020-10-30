/*
 * LinearAnalogSensor.h
 *
 *  Created on: 16 Apr 2019
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_LINEARANALOGSENSOR_H_
#define SRC_HEATING_SENSORS_LINEARANALOGSENSOR_H_

#include "SensorWithPort.h"

class LinearAnalogSensor : public SensorWithPort
{
public:
	LinearAnalogSensor(unsigned int sensorNum) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) override THROWS(GCodeException);
	void Poll() noexcept override;
	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "linearanalog";

private:
	void CalcDerivedParameters() noexcept;

	// Configurable parameters
	float lowTemp, highTemp;
	bool filtered;

	// Derived parameters
	int adcFilterChannel;
	float linearIncreasePerCount;

	static constexpr float DefaultLowTemp = 0.0;
	static constexpr float DefaultHighTemp = 100.0;
};

#endif /* SRC_HEATING_SENSORS_LINEARANALOGSENSOR_H_ */
