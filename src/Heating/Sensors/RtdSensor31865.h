/*
 * RtdSensor31865.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_RTDSENSOR31865_H_
#define SRC_HEATING_RTDSENSOR31865_H_

#include "SpiTemperatureSensor.h"

class RtdSensor31865 : public SpiTemperatureSensor
{
public:
	RtdSensor31865(unsigned int channel);
	void Init() override;
	TemperatureError GetTemperature(float& t) override;

private:
	TemperatureError TryInitRtd() const;
};

#endif /* SRC_HEATING_RTDSENSOR31865_H_ */
