/*
 * SensorWithPort.cpp
 *
 *  Created on: 18 Jul 2019
 *      Author: David
 */

#include "SensorWithPort.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

SensorWithPort::SensorWithPort(unsigned int sensorNum, const char *type) noexcept
	: TemperatureSensor(sensorNum, type)
{
}

SensorWithPort::~SensorWithPort() noexcept
{
	port.Release();
}

// Try to configure the port. Return true if the port is valid at the end, else return false and set the error message in 'reply'. Set 'seen' if we saw the P parameter.
bool SensorWithPort::ConfigurePort(GCodeBuffer& gb, const StringRef& reply, PinAccess access, PinUsedBy usedBy, bool& seen)
{
	if (gb.Seen('P'))
	{
		seen = true;
		return port.AssignPort(gb, reply, usedBy, access);
	}
	if (port.IsValid())
	{
		return true;
	}
	reply.copy("Missing port name parameter");
	return false;
}

bool SensorWithPort::ConfigurePort(GCodeBuffer& gb, const StringRef& reply, PinAccess access, bool& seen)
{
	return ConfigurePort(gb, reply, access, PinUsedBy::sensor, seen);
}

// Copy the basic details to the reply buffer. This hides the version in the base class.
void SensorWithPort::CopyBasicDetails(const StringRef& reply) const noexcept
{
	reply.printf("Sensor %u", GetSensorNumber());
	if (GetSensorName() != nullptr)
	{
		reply.catf(" (%s)", GetSensorName());
	}
	reply.catf(" type %s using pin ", GetSensorType());
	port.AppendPinName(reply);
	reply.catf(", reading %.1f, last error: %s", (double)GetStoredReading(), TemperatureErrorString(GetLastError()));
}

// End
