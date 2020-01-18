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

RemoteFan::RemoteFan(unsigned int fanNum, CanAddress boardNum) noexcept
	: Fan(fanNum),
	  boardNumber(boardNum), thermostaticFanRunning(false)
{
}

RemoteFan::~RemoteFan() noexcept
{
	CanMessageGenericConstructor cons(M950FanParams);
	try
	{
		cons.AddUParam('F', fanNumber);
		cons.AddStringParam('C', "nil");
		String<1> dummy;
		(void)cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, dummy.GetRef());
	}
	catch (...)
	{
	}
}

bool RemoteFan::Check() noexcept
{
	return thermostaticFanRunning;
}

bool RemoteFan::IsEnabled() const noexcept
{
	return true;
}

GCodeResult RemoteFan::SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	cons.AddUParam('Q', freq);
	return cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, reply);
}

void RemoteFan::UpdateRpmFromRemote(CanAddress src, int32_t rpm) noexcept
{
	if (src == boardNumber)
	{
		SetLastRpm(rpm);
	}
}

GCodeResult RemoteFan::ReportPortDetails(const StringRef& str) const noexcept
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	return cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, str);
}

bool RemoteFan::UpdateFanConfiguration(const StringRef& reply) noexcept
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer available");
		return false;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardNumber);
	auto msg = buf->SetupRequestMessage<CanMessageFanParameters>(rid, CanId::MasterAddress, boardNumber);
	msg->fanNumber = fanNumber;
	msg->blipTime = blipTime;
	msg->val = val;
	msg->minVal = minVal;
	msg->maxVal = maxVal;
	msg->triggerTemperatures[0] = triggerTemperatures[0];
	msg->triggerTemperatures[1] = triggerTemperatures[1];
	msg->sensorsMonitored = sensorsMonitored.GetRaw();

	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply) == GCodeResult::ok;
}

// Update the hardware PWM
GCodeResult RemoteFan::Refresh(const StringRef& reply) noexcept
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer available");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardNumber);
	auto msg = buf->SetupRequestMessage<CanMessageSetFanSpeed>(rid, CanId::MasterAddress, boardNumber);
	msg->fanNumber = fanNumber;
	msg->pwm = val;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

GCodeResult RemoteFan::ConfigurePort(const char* pinNames, PwmFrequency freq, const StringRef& reply) noexcept
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	cons.AddUParam('Q', freq);
	cons.AddStringParam('C', pinNames);
	return cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, reply);
}

#endif

// End
