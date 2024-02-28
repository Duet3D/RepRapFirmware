/*
 * RemoteFan.cpp
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#include "RemoteFan.h"

#if SUPPORT_CAN_EXPANSION

#include <CanMessageBuffer.h>
#include <CanMessageGenericTables.h>
#include "CAN/CanInterface.h"
#include "CAN/CanMessageGenericConstructor.h"

RemoteFan::RemoteFan(unsigned int fanNum, CanAddress boardNum) noexcept
	: Fan(fanNum),
	  lastRpm(-1), lastPwm(-1.0), whenLastReportReceived(0),
	  boardNumber(boardNum)
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

bool RemoteFan::Check(bool checkSensors) noexcept
{
	if (millis() - whenLastReportReceived > FanReportTimeout)
	{
		lastRpm = -1;
		lastPwm = -1.0;
	}
	return sensorsMonitored.IsNonEmpty() && lastPwm > 0.0;
}

bool RemoteFan::IsEnabled() const noexcept
{
	return true;
}

GCodeResult RemoteFan::SetFanParameters(bool setFreq, PwmFrequency freq, bool setPpr, float ppr, const StringRef& reply) noexcept
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	if (setFreq) { cons.AddUParam('Q', freq); }
	if (setPpr) { cons.AddFParam('K', ppr); }
	const GCodeResult rslt = cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, reply);
	if (rslt <= GCodeResult::warning)
	{
		if (setFreq) { pwmFreq = freq; }
		if (setPpr) { tachoPulsesPerRev = ppr; }
	}
	return rslt;
}

void RemoteFan::UpdateFromRemote(CanAddress src, const FanReport& report) noexcept
{
	if (src == boardNumber)
	{
		lastPwm = (report.actualPwm < 0) ? -1.0 : (float)report.actualPwm * (1.0/65535);
		lastRpm = report.rpm;
		whenLastReportReceived = millis();
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
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer available");
		return false;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardNumber, buf);
	auto msg = buf->SetupRequestMessage<CanMessageFanParameters>(rid, CanInterface::GetCanAddress(), boardNumber);
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
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer available");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardNumber, buf);
	auto msg = buf->SetupRequestMessage<CanMessageSetFanSpeed>(rid, CanInterface::GetCanAddress(), boardNumber);
	msg->fanNumber = fanNumber;
	msg->pwm = val;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

GCodeResult RemoteFan::ConfigurePort(const char* pinNames, PwmFrequency freq, float ppr, const StringRef& reply) noexcept
{
	CanMessageGenericConstructor cons(M950FanParams);
	cons.AddUParam('F', fanNumber);
	cons.AddUParam('Q', freq);
	cons.AddStringParam('C', pinNames);
	cons.AddFParam('K', ppr);
	const GCodeResult rslt = cons.SendAndGetResponse(CanMessageType::m950Fan, boardNumber, reply);
	if (rslt <= GCodeResult::warning)
	{
		pwmFreq = freq;
		tachoPulsesPerRev = ppr;
	}
	return rslt;
}

#endif

// End
