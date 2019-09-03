/*
 * RemoteFan.cpp
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#include "RemoteFan.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMessageBuffer.h"
#include "CAN/CanInterface.h"
#include "CAN/CanMessageGenericConstructor.h"

RemoteFan::RemoteFan(unsigned int fanNum, CanAddress boardNum)
	: Fan(fanNum),
	  lastRpm(-1), whenLastRpmReceived(0), boardNumber(boardNum), thermostaticFanRunning(false)
{
}

RemoteFan::~RemoteFan()
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	cons.AddStringParam('C', "nil");
	String<1> dummy;
	(void)cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, dummy.GetRef());
}

bool RemoteFan::Check()
{
	return thermostaticFanRunning;
}

bool RemoteFan::IsEnabled() const
{
	return true;
}

void RemoteFan::SetPwmFrequency(PwmFrequency freq)
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	cons.AddUParam('Q', freq);
	String<1> dummy;
	(void)cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, dummy.GetRef());
}

int32_t RemoteFan::GetRPM()
{
	if (millis() - whenLastRpmReceived > RpmReadingTimeout)
	{
		lastRpm = -1;
	}
	return lastRpm;
}

void RemoteFan::AppendPortDetails(const StringRef& str) const
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	//TODO the following is not quite right, it appends all the fan parameters
	(void)cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, str);
}

bool RemoteFan::UpdateFanConfiguration(const StringRef& reply)
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		reply.copy("No CAN buffer available");
		return false;
	}

	auto msg = buf->SetupRequestMessage<CanMessageFanParameters>(CanId::MasterAddress, boardNumber);
	msg->fanNumber = fanNumber;
	msg->blipTime = blipTime;
	msg->val = val;
	msg->minVal = minVal;
	msg->maxVal = maxVal;
	msg->triggerTemperatures[0] = triggerTemperatures[0];
	msg->triggerTemperatures[1] = triggerTemperatures[1];
	msg->sensorsMonitored = sensorsMonitored;

	return CanInterface::SendRequestAndGetStandardReply(buf, reply) == GCodeResult::ok;
}

// Update the hardware PWM
void RemoteFan::Refresh()
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		//TODO error handling
	}
	else
	{
		auto msg = buf->SetupRequestMessage<CanMessageSetFanSpeed>(CanId::MasterAddress, boardNumber);
		msg->fanNumber = fanNumber;
		msg->pwm = val;
		String<1> dummy;
		const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf, dummy.GetRef());
		if (rslt != GCodeResult::ok)
		{
			//TODO error handling
		}
	}
}

GCodeResult RemoteFan::ConfigurePort(const char* pinNames, PwmFrequency freq, const StringRef& reply)
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	cons.AddUParam('Q', freq);
	cons.AddStringParam('C', pinNames);
	return cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, reply);
}

#endif

// End
