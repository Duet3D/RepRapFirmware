/*
 * RemoteHeater.cpp
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#include "RemoteHeater.h"

#if SUPPORT_CAN_EXPANSION

#include "RepRap.h"
#include "Heat.h"
#include "CAN/CanMessageGenericConstructor.h"
#include "CAN/CanInterface.h"
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>

void RemoteHeater::Spin()
{
	// Nothing needed here unless we want to copy the sensor temperature across. For now we don't store the temperature locally.
}

void RemoteHeater::ResetHeater()
{
	//TODO
}

GCodeResult RemoteHeater::ConfigurePortAndSensor(GCodeBuffer& gb, const StringRef& reply)
{
	CanMessageGenericConstructor cons(M950Params);
	if (!cons.PopulateFromCommand(gb, reply))
	{
		return GCodeResult::error;
	}
	return cons.SendAndGetResponse(CanMessageType::m950, boardAddress, reply);
}

// If it's a local heater, turn it off and release its port. If it is remote, delete the remote heater.
void RemoteHeater::ReleasePort()
{
	//TODO
}

void RemoteHeater::SwitchOff()
{
	//TODO
}

void RemoteHeater::ResetFault()
{
	//TODO
}

float RemoteHeater::GetTemperature() const
{
	TemperatureError err;
	return reprap.GetHeat().GetSensorTemperature(GetSensorNumber(), err);
}

float RemoteHeater::GetAveragePWM() const
{
	//TODO
	return 0.0;		// not yet supported
}

// Return the integral accumulator
float RemoteHeater::GetAccumulator() const
{
	//TODO
	return 0.0;		// not yet supported
}

void RemoteHeater::StartAutoTune(float targetTemp, float maxPwm, const StringRef& reply)
{
	//TODO
}

void RemoteHeater::GetAutoTuneStatus(const StringRef& reply) const
{
	//TODO
}

void RemoteHeater::Suspend(bool sus)
{
	//TODO
}

Heater::HeaterMode RemoteHeater::GetMode() const
{
	return HeaterMode::off;
}

void RemoteHeater::SwitchOn()
{
	if (!GetModel().IsEnabled())
	{
		SetModelDefaults();
	}
	//TODO
}

// This is called when the heater model has been updated
GCodeResult RemoteHeater::UpdateModel(const StringRef& reply)
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		CanMessageUpdateHeaterModel * const msg = buf->SetupRequestMessage<CanMessageUpdateHeaterModel>(CanInterface::GetCanAddress(), boardAddress);
		model.SetupCanMessage(GetHeaterNumber(), *msg);
		return CanInterface::SendRequestAndGetStandardReply(buf, reply);
	}

	reply.copy("No CAN buffer available");
	return GCodeResult::error;
}

#endif

// End
