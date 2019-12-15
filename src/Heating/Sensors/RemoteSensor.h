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
	RemoteSensor(unsigned int sensorNum, CanAddress pBoardAddress) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) override;
	CanAddress GetBoardAddress() const noexcept override { return boardAddress; }
	void UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept override;

	// Try to get a temperature reading
	void Poll() noexcept override { }				// nothing to do here because reception of CAN messages update the reading

private:
	CanAddress boardAddress;
};

#endif

#endif /* SRC_HEATING_SENSORS_REMOTESENSOR_H_ */
