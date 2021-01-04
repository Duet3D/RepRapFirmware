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
#include <General/Portability.h>

constexpr uint32_t RemoteTemperatureTimeoutMillis = 1000;

RemoteSensor::RemoteSensor(unsigned int sensorNum, CanAddress pBoardAddress) noexcept
	: TemperatureSensor(sensorNum, "remote"), boardAddress(pBoardAddress)
{
}

RemoteSensor::~RemoteSensor()
{
	CanMessageGenericConstructor cons(M308NewParams);
	cons.AddUParam('S', GetSensorNumber());
	cons.AddStringParam('P', NoPinName);
	String<StringLength50> dummy;
	(void)cons.SendAndGetResponse(CanMessageType::m308New, boardAddress, dummy.GetRef());
}

GCodeResult RemoteSensor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) THROWS(GCodeException)
{
	TryConfigureSensorName(gb, changed);
	CanMessageGenericConstructor cons(M308NewParams);
	cons.PopulateFromCommand(gb);
	const GCodeResult ret = cons.SendAndGetResponse(CanMessageType::m308New, boardAddress, reply);
	if ((ret == GCodeResult::ok || ret == GCodeResult::warning) && StringStartsWith(reply.c_str(), "type "))
	{
		// It's just a query for the sensor parameters, so prefix the sensor number and name
		String<StringLength50> temp;
		temp.printf("Sensor %u ", GetSensorNumber());
		if (GetSensorName() != nullptr)
		{
			temp.catf("(%s) ", GetSensorName());
		}
		reply.Insert(0, temp.c_str());
	}
	else
	{
		changed = true;
	}
	return ret;
}

void RemoteSensor::UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept
{
	if (src == boardAddress)
	{
		SetResult(report.GetTemperature(), (TemperatureError)report.errorCode);
	}
}

#endif

// End
