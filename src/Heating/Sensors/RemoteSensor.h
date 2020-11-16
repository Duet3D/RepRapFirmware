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
	~RemoteSensor();

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) override THROWS(GCodeException);
	CanAddress GetBoardAddress() const noexcept override { return boardAddress; }
	void Poll() noexcept override { }				// nothing to do here because reception of CAN messages update the reading
	void UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept override;
	const char *GetShortSensorType() const noexcept override { return "remote"; }	// TODO save the actual type

private:
	CanAddress boardAddress;
};

#endif

#endif /* SRC_HEATING_SENSORS_REMOTESENSOR_H_ */
