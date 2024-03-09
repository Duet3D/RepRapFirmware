/*
 * SensorWithPort.cpp
 *
 *  Created on: 18 Jul 2019
 *      Author: David
 */

#include "SensorWithPort.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(SensorWithPort, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(SensorWithPort, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry SensorWithPort::objectModelTable[] =
{
	{ "port",	OBJECT_MODEL_FUNC(self->port), 	ObjectModelEntryFlags::none }
};

constexpr uint8_t SensorWithPort::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(SensorWithPort, TemperatureSensor)

SensorWithPort::SensorWithPort(unsigned int sensorNum, const char *type) noexcept
	: TemperatureSensor(sensorNum, type)
{
}

SensorWithPort::~SensorWithPort() noexcept
{
	port.Release();
}

// Try to configure the port. Return true if the port is valid at the end, else return false and set the error message in 'reply'. Set 'seen' if we saw the P parameter.
bool SensorWithPort::ConfigurePort(GCodeBuffer& gb, const StringRef& reply, PinAccess access, bool& seen)
{
	if (gb.Seen('P'))
	{
		seen = true;
		return port.AssignPort(gb, reply, PinUsedBy::sensor, access);
	}
	if (port.IsValid())
	{
		return true;
	}
	reply.copy("Missing port name parameter");
	return false;
}

#if SUPPORT_REMOTE_COMMANDS

// Try to configure the port. Return true if success or the port string is not given but the port is already valid, false if error.
bool SensorWithPort::ConfigurePort(const CanMessageGenericParser& parser, const StringRef& reply, PinAccess access, bool& seen)
{
	String<StringLength20> portName;
	if (parser.GetStringParam('P', portName.GetRef()))
	{
		seen = true;
		return port.AssignPort(portName.c_str(), reply, PinUsedBy::sensor, access);
	}
	if (port.IsValid())
	{
		return true;
	}
	reply.copy("Missing port name parameter");
	return false;
}

#endif

// Copy the pin details to the reply buffer
void SensorWithPort::AppendPinDetails(const StringRef& reply) const noexcept
{
	reply.cat(" using pin ");
	port.AppendPinName(reply);
}

// End
