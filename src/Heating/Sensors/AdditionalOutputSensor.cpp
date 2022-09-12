/*
 * AdditionalOutputSensor.cpp
 *
 *  Created on: 17 Oct 2019
 *      Author: manuel
 */

#include <ctype.h>
#include "AdditionalOutputSensor.h"
#include <Platform/RepRap.h>
#include <Heating/Heat.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

AdditionalOutputSensor::AdditionalOutputSensor(unsigned int sensorNum, const char *type, bool pEnforcePollOrder) noexcept
	: TemperatureSensor(sensorNum, type), parentSensor(0), outputNumber(0), enforcePollOrder(pEnforcePollOrder)
{
}

AdditionalOutputSensor::~AdditionalOutputSensor() noexcept
{
}

GCodeResult AdditionalOutputSensor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	GCodeResult rslt = GCodeResult::ok;

	if (gb.Seen('P'))
	{
		changed = true;
		String<StringLength20> pParam;
		gb.GetQuotedString(pParam.GetRef());
		rslt = ConfigurePort(pParam.c_str(), reply);
		if (rslt > GCodeResult::warning)
		{
			return rslt;
		}
	}

	TryConfigureSensorName(gb, changed);
	if (!changed && !gb.Seen('Y'))
	{
		// No parameters were provided, so report the current configuration
		CopyBasicDetails(reply);
		reply.catf(", additional output %d of sensor %d", outputNumber, parentSensor);
	}
	return GCodeResult::ok;
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult AdditionalOutputSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	String<StringLength20> pParam;
	if (parser.GetStringParam('P', pParam.GetRef()))
	{
		return ConfigurePort(pParam.c_str(), reply);
	}

	CopyBasicDetails(reply);
	reply.catf(", additional output %d of sensor %d", outputNumber, parentSensor);
	return GCodeResult::ok;
}

#endif

GCodeResult AdditionalOutputSensor::ConfigurePort(const char* portName, const StringRef& reply) noexcept
{
	if (toupper(*portName) != 'S')
	{
		reply.copy("Parent sensor needs to start with S");
		return GCodeResult::error;
	}
	// Advance beyond the leading S
	++portName;

	if (!isDigit(*portName))
	{
		reply.copy("Parent sensor number expected following S");
		return GCodeResult::error;
	}

	// Parse parent sensor number
	parentSensor = StrToU32(portName, &portName);
	if (*portName != '.')
	{
		reply.copy("Missing additional output number of parent");
		return GCodeResult::error;
	}

	// We use this block to have the ReadLockedPointer below go out of scope as early as possible
	{
		const auto parent = reprap.GetHeat().FindSensor(parentSensor);
		if (parent.IsNull())
		{
			reply.printf("Parent sensor %d does not exist", parentSensor);
			return GCodeResult::error;
		}

		if (enforcePollOrder && parentSensor > GetSensorNumber())
		{
			reply.copy("Parent sensor must be a lower sensor number than this one");
			return GCodeResult::error;
		}

		// Advance beyond the dot
		++portName;

		// Parse output number
		outputNumber = StrToU32(portName, &portName);

		if (outputNumber > parent->GetNumAdditionalOutputs())
		{
			reply.printf("Parent sensor only has %d additional outputs", parent->GetNumAdditionalOutputs());
			return GCodeResult::error;
		}
	}

	// Initialize with a value already
	Poll();
	return GCodeResult::ok;
}

void AdditionalOutputSensor::Poll() noexcept
{
	float t;
	const auto parent = reprap.GetHeat().FindSensor(parentSensor);
	if (parent.IsNull())
	{
		SetResult(TemperatureError::notReady);
		return;
	}
	if (this->outputNumber > parent->GetNumAdditionalOutputs())
	{
		SetResult(TemperatureError::invalidOutputNumber);
		return;
	}
	const auto err = parent->GetLatestTemperature(t, this->outputNumber);
	if (err == TemperatureError::success)
	{
		SetResult(t, err);
	}
	else
	{
		SetResult(err);
	}

}
