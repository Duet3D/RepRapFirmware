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
#include "Platform.h"
#include "CAN/CanMessageGenericConstructor.h"
#include "CAN/CanInterface.h"
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>

RemoteHeater::RemoteHeater(unsigned int num, CanAddress board)
	: Heater(num), boardAddress(board), lastMode(HeaterMode::offline), averagePwm(0), lastTemperature(0.0), whenLastStatusReceived(0)
{
}

RemoteHeater::~RemoteHeater()
{
	CanMessageGenericConstructor cons(M950HeaterParams);
	cons.AddUParam('H', GetHeaterNumber());
	cons.AddStringParam('C', "nil");
	String<1> dummy;
	(void)cons.SendAndGetResponse(CanMessageType::m950Heater, boardAddress, dummy.GetRef());
}

void RemoteHeater::Spin()
{
	// Nothing needed here unless we want to copy the sensor temperature across. For now we don't store the temperature locally.
}

void RemoteHeater::ResetHeater()
{
	//TODO
}

GCodeResult RemoteHeater::ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sensorNumber, const StringRef& reply)
{
	SetSensorNumber(sensorNumber);
	CanMessageGenericConstructor cons(M950HeaterParams);
	cons.AddUParam('H', GetHeaterNumber());
	cons.AddUParam('Q', freq);
	cons.AddUParam('T', sensorNumber);
	cons.AddStringParam('C', portName);
	return cons.SendAndGetResponse(CanMessageType::m950Heater, boardAddress, reply);
}

GCodeResult RemoteHeater::SetPwmFrequency(PwmFrequency freq, const StringRef& reply)
{
	CanMessageGenericConstructor cons(M950HeaterParams);
	cons.AddUParam('H', GetHeaterNumber());
	cons.AddUParam('Q', freq);
	return cons.SendAndGetResponse(CanMessageType::m950Heater, boardAddress, reply);
}

GCodeResult RemoteHeater::ReportDetails(const StringRef& reply) const
{
	CanMessageGenericConstructor cons(M950HeaterParams);
	cons.AddUParam('H', GetHeaterNumber());
	return cons.SendAndGetResponse(CanMessageType::m950Heater, boardAddress, reply);
}

void RemoteHeater::SwitchOff()
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to switch off remote heater %u: no CAN buffer available\n", GetHeaterNumber());
	}
	else
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
		auto msg = buf->SetupRequestMessage<CanMessageSetHeaterTemperature>(rid, CanId::MasterAddress, boardAddress);
		msg->heaterNumber = GetHeaterNumber();
		msg->setPoint = GetTargetTemperature();
		msg->command = CanMessageSetHeaterTemperature::commandOff;
		String<StringLength50> reply;
		if (CanInterface::SendRequestAndGetStandardReply(buf, rid, reply.GetRef()) != GCodeResult::ok)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to switch off remote heater %u: %s\n", GetHeaterNumber(), reply.c_str());
		}
	}
}

GCodeResult RemoteHeater::ResetFault(const StringRef& reply)
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	auto msg = buf->SetupRequestMessage<CanMessageSetHeaterTemperature>(rid, CanId::MasterAddress, boardAddress);
	msg->heaterNumber = GetHeaterNumber();
	msg->setPoint = GetTargetTemperature();
	msg->command = CanMessageSetHeaterTemperature::commandResetFault;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

float RemoteHeater::GetTemperature() const
{
	if (millis() - whenLastStatusReceived < RemoteStatusTimeout)
	{
		return lastTemperature;
	}

	TemperatureError err;
	return reprap.GetHeat().GetSensorTemperature(GetSensorNumber(), err);
}

float RemoteHeater::GetAveragePWM() const
{
	return (millis() - whenLastStatusReceived < RemoteStatusTimeout) ? (float)averagePwm / 255.0 : 0;
}

// Return the integral accumulator
float RemoteHeater::GetAccumulator() const
{
	return 0.0;		// not supported
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
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
		auto msg = buf->SetupRequestMessage<CanMessageSetHeaterTemperature>(rid, CanId::MasterAddress, boardAddress);
		msg->heaterNumber = GetHeaterNumber();
		msg->setPoint = GetTargetTemperature();
		msg->command = (sus) ? CanMessageSetHeaterTemperature::commandSuspend : CanMessageSetHeaterTemperature::commandUnsuspend;
		String<1> dummy;
		(void) CanInterface::SendRequestAndGetStandardReply(buf, rid, dummy.GetRef());
	}
}

Heater::HeaterMode RemoteHeater::GetMode() const
{
	return (millis() - whenLastStatusReceived < RemoteStatusTimeout) ? lastMode : HeaterMode::offline;
}

// This isn't just called to turn the heater on, it is called when the temperature needs to be updated
GCodeResult RemoteHeater::SwitchOn(const StringRef& reply)
{
	if (!GetModel().IsEnabled())
	{
		SetModelDefaults();
	}

	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	auto msg = buf->SetupRequestMessage<CanMessageSetHeaterTemperature>(rid, CanId::MasterAddress, boardAddress);
	msg->heaterNumber = GetHeaterNumber();
	msg->setPoint = GetTargetTemperature();
	msg->command = CanMessageSetHeaterTemperature::commandOn;
	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

// This is called when the heater model has been updated
GCodeResult RemoteHeater::UpdateModel(const StringRef& reply)
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
		CanMessageUpdateHeaterModel * const msg = buf->SetupRequestMessage<CanMessageUpdateHeaterModel>(rid, CanInterface::GetCanAddress(), boardAddress);
		model.SetupCanMessage(GetHeaterNumber(), *msg);
		return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
	}

	reply.copy("No CAN buffer");
	return GCodeResult::error;
}

GCodeResult RemoteHeater::UpdateFaultDetectionParameters(const StringRef& reply)
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
		CanMessageSetHeaterFaultDetectionParameters * const msg = buf->SetupRequestMessage<CanMessageSetHeaterFaultDetectionParameters>(rid, CanInterface::GetCanAddress(), boardAddress);
		msg->heater = GetHeaterNumber();
		msg->maxFaultTime = GetMaxHeatingFaultTime();
		msg->maxTempExcursion = GetMaxTemperatureExcursion();
		return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
	}

	reply.copy("No CAN buffer");
	return GCodeResult::error;
}

void RemoteHeater::UpdateRemoteStatus(CanAddress src, const CanHeaterReport& report)
{
	if (src == boardAddress)
	{
		lastMode = (HeaterMode)report.mode;
		averagePwm = report.averagePwm;
		lastTemperature = report.temperature;
		whenLastStatusReceived = millis();
	}
}

#endif

// End
