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

AdditionalOutputSensor::AdditionalOutputSensor(unsigned int sensorNum, const char *type, bool enforcePollOrder) noexcept
	: TemperatureSensor(sensorNum, type), parentSensor(0), outputNumber(0), enforcePollOrder(enforcePollOrder)
{
}

AdditionalOutputSensor::~AdditionalOutputSensor() noexcept
{
}

GCodeResult AdditionalOutputSensor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	if (gb.Seen('P'))
	{
		changed = true;
		String<StringLength20> pParam;
		gb.GetQuotedString(pParam.GetRef());

		const char *pn = pParam.c_str();
		if (*pn != 'S' && *pn != 's')
		{
			reply.copy("Parent sensor needs to start with S");
			return GCodeResult::error;
		}
		// Advance beyond the leading S
		++pn;

		if (!isDigit(*pn))
		{
			reply.copy("Parent sensor number expected following S");
			return GCodeResult::error;
		}

		// Parse parent sensor number
		parentSensor = StrToU32(pn, &pn);
		if (*pn != '.')
		{
			reply.copy("Missing additional output number of parent");
			return GCodeResult::error;
		}

		// We use this block to have the ReadLockPointer below go out of scope as early as possible
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
			++pn;

			// Parse output number
			outputNumber = StrToU32(pn, &pn);

			if (outputNumber > parent->GetNumAdditionalOutputs())
			{
				reply.printf("Parent sensor only has %d additional outputs", parent->GetNumAdditionalOutputs());
				return GCodeResult::error;
			}
		}

		// Initialize with a value already
		Poll();
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
