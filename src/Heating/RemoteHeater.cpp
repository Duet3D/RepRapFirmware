/*
 * RemoteHeater.cpp
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#include "RemoteHeater.h"

#if SUPPORT_CAN_EXPANSION

#include "CAN/CanMessageGenericConstructor.h"

void RemoteHeater::Spin()
{
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

void RemoteHeater::SwitchOff()
{
}

void RemoteHeater::ResetFault()
{
}

float RemoteHeater::GetTemperature() const
{
	return 0.0;
}

float RemoteHeater::GetAveragePWM() const
{
	return 0.0;		// not yet supported
}

// Return the integral accumulator
float RemoteHeater::GetAccumulator() const
{
	return 0.0;		// not yet supported
}

void RemoteHeater::StartAutoTune(float targetTemp, float maxPwm, const StringRef& reply)
{
}

void RemoteHeater::GetAutoTuneStatus(const StringRef& reply) const
{
}

void RemoteHeater::Suspend(bool sus)
{
}

Heater::HeaterMode RemoteHeater::GetMode() const
{
	return HeaterMode::off;
}

void RemoteHeater::SwitchOn()
{
}

// This is called when the heater model has been updated. Returns true if successful.
GCodeResult RemoteHeater::UpdateModel(const StringRef& reply)
{
	CanMessageGenericConstructor cons(M307Params);
	cons.AddUParam('H', GetHeaterNumber());
	cons.AddFParam('A', GetModel().GetGain());
	cons.AddFParam('C', GetModel().GetTimeConstant());
	cons.AddFParam('D', GetModel().GetDeadTime());
	cons.AddFParam('V', GetModel().GetVoltage());
	cons.AddFParam('S', GetModel().GetMaxPwm());
	cons.AddUParam('B', (GetModel().UsePid()) ? 0 : 1);
	cons.AddUParam('I', (GetModel().IsInverted()) ? 1 : 0);

	const char* const err = cons.GetErrorMessage();
	if (err != nullptr)
	{
		reply.copy(err);
		return GCodeResult::error;
	}
	return cons.SendAndGetResponse(CanMessageType::m307, boardAddress, reply);
}

#endif

// End
