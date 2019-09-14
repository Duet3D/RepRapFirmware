/*
 * RemoteZProbe.cpp
 *
 *  Created on: 14 Sep 2019
 *      Author: David
 */

#include "RemoteZProbe.h"

#if SUPPORT_CAN_EXPANSION

#include <CanMessageBuffer.h>
#include <CAN/CanInterface.h>
#include <CAN/CanMessageGenericConstructor.h>
#include <RepRap.h>
#include <Platform.h>

// Members of class RemoteZProbe
RemoteZProbe::~RemoteZProbe()
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf != nullptr)
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
		auto msg = buf->SetupRequestMessage<CanMessageDestroyZProbe>(rid, CanId::MasterAddress, boardAddress);
		msg->number = number;
		String<StringLength50> reply;
		if (CanInterface::SendRequestAndGetStandardReply(buf, rid, reply.GetRef()) != GCodeResult::ok)
		{
			reprap.GetPlatform().Message(ErrorMessage, reply.c_str());
		}
	}
}

GCodeResult RemoteZProbe::AppendPinNames(const StringRef& str) const
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		str.cat("[not available: no CAN buffer]");
		return GCodeResult::warning;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	auto msg = buf->SetupRequestMessage<CanMessageGetZProbePinNames>(rid, CanId::MasterAddress, boardAddress);
	msg->number = number;
	String<StringLength20> pinNames;
	const GCodeResult rslt = CanInterface::SendRequestAndGetStandardReply(buf, rid, pinNames.GetRef());
	if (rslt == GCodeResult::ok)
	{
		str.cat(pinNames.c_str());
	}
	else
	{
		str.catf("[not available: %s]", pinNames.c_str());
	}
	return rslt;
}

void RemoteZProbe::SetProbing(bool isProbing) const
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reprap.GetPlatform().Message(ErrorMessage, "No CAN buffer");
	}
	else
	{
		const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
		auto msg = buf->SetupRequestMessage<CanMessageSetProbing>(rid, CanId::MasterAddress, boardAddress);
		msg->number = number;
		String<StringLength50> reply;
		if (CanInterface::SendRequestAndGetStandardReply(buf, rid, reply.GetRef()) != GCodeResult::ok)
		{
			reprap.GetPlatform().Message(ErrorMessage, reply.c_str());
		}
	}
}

// Create a remote Z probe
GCodeResult RemoteZProbe::Create(const StringRef& pinNames, const StringRef& reply)
{
	CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		reply.copy("No CAN buffer");
		return GCodeResult::error;
	}

	const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
	auto msg = buf->SetupRequestMessage<CanMessageCreateZProbe>(rid, CanId::MasterAddress, boardAddress);
	msg->probeNumber = number;
	msg->probeType = (uint8_t)type;
	SafeStrncpy(msg->pinNames, pinNames.c_str(), ARRAY_SIZE(msg->pinNames));
	buf->dataLength = msg->GetActualDataLength();

	return CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
}

// Configure an existing remote Z probe
GCodeResult RemoteZProbe::Configure(GCodeBuffer& gb, const StringRef &reply, bool& seen)
{
	GCodeResult rslt = ZProbe::Configure(gb, reply, seen);
	if (seen && rslt == GCodeResult::ok)
	{
		CanMessageBuffer * const buf = CanMessageBuffer::Allocate();
		if (buf == nullptr)
		{
			reply.copy("No CAN buffer");
			rslt = GCodeResult::error;
		}
		else
		{
			const CanRequestId rid = CanInterface::AllocateRequestId(boardAddress);
			auto msg = buf->SetupRequestMessage<CanMessageConfigureZProbe>(rid, CanId::MasterAddress, boardAddress);

			msg->number = number;
			msg->type = (uint8_t)type;
			msg->triggerHeight = triggerHeight;
			msg->calibTemperature = calibTemperature;
			msg->temperatureCoefficient = temperatureCoefficient;
			msg->adcValue = adcValue;
			msg->misc = misc.all;
			msg->sensor = sensor;

			rslt = CanInterface::SendRequestAndGetStandardReply(buf, rid, reply);
		}
	}
	return rslt;
}

GCodeResult RemoteZProbe::SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply)
{
	//TODO
	reply.copy("Programming remote Z probes not yet supported");
	return GCodeResult::error;
}

#endif

// End
