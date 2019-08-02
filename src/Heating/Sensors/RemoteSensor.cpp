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

RemoteSensor::RemoteSensor(unsigned int sensorNum, CanAddress pBoardAddress)
	: TemperatureSensor(sensorNum, "remote"), boardAddress(pBoardAddress)
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
	//TODO
	t = BadErrorTemperature;
	return TemperatureError::notReady;
}

#endif

// End
