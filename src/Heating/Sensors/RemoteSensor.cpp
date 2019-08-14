/*
 * RemoteSensor.cpp
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#include "RemoteSensor.h"

#if SUPPORT_CAN_EXPANSION

#include "CAN/CanMessageGenericConstructor.h"
#include "CanMessageBuffer.h"

constexpr uint32_t RemoteTemperatureTimeoutMillis = 1000;

RemoteSensor::RemoteSensor(unsigned int sensorNum, CanAddress pBoardAddress)
	: TemperatureSensor(sensorNum, "remote"),
	  lastTemperature(BadErrorTemperature), whenLastReadingReceived(millis()), boardAddress(pBoardAddress), lastError(TemperatureError::notReady)
{
}

GCodeResult RemoteSensor::Configure(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	TryConfigureSensorName(gb, seen);
	CanMessageGenericConstructor cons(M308Params);
	if (!cons.PopulateFromCommand(gb, reply))
	{
		return GCodeResult::error;
	}
	return cons.SendAndGetResponse(CanMessageType::m308, boardAddress, reply);
}

// Try to get a temperature reading
TemperatureError RemoteSensor::TryGetTemperature(float& t)
{
	if (millis() - whenLastReadingReceived > RemoteTemperatureTimeoutMillis)
	{
		lastTemperature = BadErrorTemperature;
		lastError = TemperatureError::timeout;
	}

	t = lastTemperature;
	return lastError;
}

void RemoteSensor::UpdateRemoteTemperature(const CanTemperatureReport& report)
{
	lastError = (TemperatureError)report.errorCode;
	lastTemperature = report.temperature;
	whenLastReadingReceived = millis();
}

#endif

// End
