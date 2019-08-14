/*
 * RemoteSensor.h
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_REMOTESENSOR_H_
#define SRC_HEATING_SENSORS_REMOTESENSOR_H_

#include "TemperatureSensor.h"

#if SUPPORT_CAN_EXPANSION

class RemoteSensor : public TemperatureSensor
{
public:
	RemoteSensor(unsigned int sensorNum, CanAddress pBoardAddress);

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) override;
	CanAddress GetBoardAddress() const override { return boardAddress; }
	void UpdateRemoteTemperature(const CanTemperatureReport& report) override;

protected:
	// Try to get a temperature reading
	TemperatureError TryGetTemperature(float& t) override;

private:
	float lastTemperature;
	uint32_t whenLastReadingReceived;
	CanAddress boardAddress;
	TemperatureError lastError;
};

#endif

#endif /* SRC_HEATING_SENSORS_REMOTESENSOR_H_ */
