/*
 * AdditionalOutputSensor.cpp
 *
 *  Created on: 17 Oct 2019
 *      Author: manuel
 */

#include <ctype.h>
#include "AdditionalOutputSensor.h"
#include "RepRap.h"
#include "Heating/Heat.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

AdditionalOutputSensor::AdditionalOutputSensor(unsigned int sensorNum, const char *type, bool enforcePollOrder)
	: TemperatureSensor(sensorNum, type), parentSensor(0), outputNumber(0), enforcePollOrder(enforcePollOrder)
{
}

AdditionalOutputSensor::~AdditionalOutputSensor() {
}

GCodeResult AdditionalOutputSensor::Configure(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	if (gb.Seen('P'))
	{
		seen = true;
		String<StringLength20> pParam;
		StringRef ref = pParam.GetRef();
		if (!gb.GetQuotedString(ref))
		{
			reply.copy("Missing parent sensor and output number");
			return GCodeResult::error;
		}

		size_t pos = 0;
		if (ref[pos] != 'S' && ref[pos] != 's')
		{
			reply.copy("Parent sensor needs to start with S");
			return GCodeResult::error;
		}
		// Advance beyond the leading S
		++pos;

		if (!isDigit(ref[pos]))
		{
			reply.copy("Parent sensor number expected following S");
			return GCodeResult::error;
		}

		// Parse parent sensor number
		while (isdigit(ref[pos]))
		{
			parentSensor = (parentSensor * 10) + (ref[pos] - '0');
			++pos;
		}

		if (parentSensor > MaxSensorsInSystem)
		{
			reply.catf("Parent sensor must be a lower than %d", MaxSensorsInSystem);
			return GCodeResult::error;
		}

		if (enforcePollOrder && parentSensor > GetSensorNumber())
		{
			reply.copy("Parent sensor must be a lower sensor number than this one");
			return GCodeResult::error;
		}

		// Advance beyond the dot
		++pos;

		// Parse output number
		while (isdigit(ref[pos]))
		{
			outputNumber = (outputNumber * 10) + (ref[pos] - '0');
			++pos;
		}

		// When getting here Heat holds a write lock on the sensors so unfortunately we can
		// neither check for the existence of the parent nor if it has the requested output while configuring
	}

	TryConfigureSensorName(gb, seen);
	if (!seen && !gb.Seen('Y'))
	{
		// No parameters were provided, so report the current configuration
		CopyBasicDetails(reply);
		reply.catf(", additional output %d of sensor %d", outputNumber, parentSensor);
	}
	return GCodeResult::ok;
}

void AdditionalOutputSensor::Poll()
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
