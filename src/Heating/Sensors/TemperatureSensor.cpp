#include <Heating/Sensors/CurrentLoopTemperatureSensor.h>
#include "TemperatureSensor.h"
#include "Thermistor.h"
#include "ThermocoupleSensor31855.h"
#include "ThermocoupleSensor31856.h"
#include "RtdSensor31865.h"
#include "GCodes/GCodeBuffer.h"

#ifndef __RADDS__
#include "CpuTemperatureSensor.h"
#endif

#ifdef DUET_NG
#include "TmcDriverTemperatureSensor.h"
#endif

// Constructor
TemperatureSensor::TemperatureSensor(unsigned int chan, const char *t) : sensorChannel(chan), sensorType(t), heaterName(nullptr) {}

// Virtual destructor
TemperatureSensor::~TemperatureSensor()
{
	delete heaterName;
}

// Set the name - normally called only once
void TemperatureSensor::SetHeaterName(const char *newName)
{
	// Change the heater name in a thread-safe manner
	const char *oldName = heaterName;
	heaterName = nullptr;
	delete oldName;

	if (newName != nullptr)
	{
		char *temp = new char[strlen(newName)];
		strcpy(temp, newName);
		heaterName = temp;
	}
}

// Default implementation of Configure, for sensors that have no configurable parameters
bool TemperatureSensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, StringRef& reply, bool& error)
{
	bool seen = false;
	if (mCode == 305)
	{
		TryConfigureHeaterName(gb, seen);
		if (!seen && !gb.Seen('X'))
		{
			// No parameters provided, so report the current configuration
			CopyBasicHeaterDetails(heater, reply);
		}
	}
	return seen;
}

void TemperatureSensor::CopyBasicHeaterDetails(unsigned int heater, StringRef& reply) const
{
	reply.printf("Heater %u", heater);
	if (heaterName != nullptr)
	{
		reply.catf(" (%s)", heaterName);
	}
	reply.catf(" uses %s sensor channel %u", sensorType, sensorChannel);
}

// Configure then heater name, if it is provided
void TemperatureSensor::TryConfigureHeaterName(GCodeBuffer& gb, bool& seen)
{
	char buf[MaxHeaterNameLength + 1];
	bool localSeen = false;
	gb.TryGetQuotedString('S', buf, ARRAY_SIZE(buf), localSeen);
	if (localSeen)
	{
		SetHeaterName(buf);
		seen = true;
	}
}

// Factory method
TemperatureSensor *TemperatureSensor::Create(unsigned int channel)
{
	TemperatureSensor *ts = nullptr;
	if (channel < Heaters)
	{
		ts =  new Thermistor(channel);
	}
	else if (FirstMax31855ThermocoupleChannel <= channel && channel < FirstMax31855ThermocoupleChannel + MaxSpiTempSensors)
	{
		ts =  new ThermocoupleSensor31855(channel);
	}
	else if (FirstMax31856ThermocoupleChannel <= channel && channel < FirstMax31856ThermocoupleChannel + MaxSpiTempSensors)
	{
		ts =  new ThermocoupleSensor31856(channel);
	}
	else if (FirstRtdChannel <= channel && channel < FirstRtdChannel + MaxSpiTempSensors)
	{
		ts =  new RtdSensor31865(channel);
	}
	else if (FirstLinearAdcChannel <= channel && channel < FirstLinearAdcChannel + MaxSpiTempSensors)
	{
		ts =  new CurrentLoopTemperatureSensor(channel);
	}
#ifndef __RADDS__
	else if (channel == CpuTemperatureSenseChannel)
	{
		ts =  new CpuTemperatureSensor(channel);
	}
#endif
#ifdef DUET_NG
	else if (channel >= FirstTmcDriversSenseChannel && channel < FirstTmcDriversSenseChannel + 2)
	{
		ts =  new TmcDriverTemperatureSensor(channel);
	}
#endif

	if (ts != nullptr)
	{
		ts->Init();
	}
	return ts;
}

// End
