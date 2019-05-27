/*
 * DhtSensor.cpp
 *
 *  Created on: 15 Sep 2017
 *      Author: Christian
 */

#include "DhtSensor.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/StepTimer.h"
#include "Hardware/IoPorts.h"

#if SUPPORT_DHT_SENSOR

constexpr uint32_t MinimumReadInterval = 2000;		// ms
constexpr uint32_t MaximumReadTime = 20;			// ms
constexpr uint32_t MinimumOneBitLength = 50;		// microseconds
constexpr uint32_t MinimumOneBitStepClocks = (StepTimer::StepClockRate * MinimumOneBitLength)/1000000;

# include "Tasks.h"

// Static data members of class DhtSensorHardwareInterface
Mutex DhtSensorHardwareInterface::dhtMutex;
Task<DhtSensorHardwareInterface::DhtTaskStackWords> *DhtSensorHardwareInterface::dhtTask = nullptr;
DhtSensorHardwareInterface *DhtSensorHardwareInterface::activeSensors[MaxSpiTempSensors] = { 0 };

extern "C" [[noreturn]] void DhtTaskStart(void * pvParameters)
{
	DhtSensorHardwareInterface::SensorTask();
}

// Pulse ISR
extern "C" void DhtDataTransition(CallbackParameter cp)
{
	static_cast<DhtSensorHardwareInterface*>(cp.vp)->Interrupt();
}

DhtSensorHardwareInterface::DhtSensorHardwareInterface(Pin p_pin)
	: sensorPin(p_pin), type(DhtSensorType::none), lastResult(TemperatureError::notInitialised),
	  lastTemperature(BadErrorTemperature), lastHumidity(BadErrorTemperature), badTemperatureCount(0)
{
	IoPort::SetPinMode(sensorPin, INPUT_PULLUP);
}

TemperatureError DhtSensorHardwareInterface::GetTemperatureOrHumidity(float& t, bool wantHumidity) const
{
	t = (wantHumidity) ? lastHumidity : lastTemperature;
	return lastResult;
}

/*static*/ GCodeResult DhtSensorHardwareInterface::Configure(TemperatureSensor *ts, unsigned int relativeChannel, unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	MutexLocker lock(dhtMutex);

	if (relativeChannel >= MaxSpiTempSensors || activeSensors[relativeChannel] == nullptr)
	{
		reply.copy("invalid channel");
		return GCodeResult::error;
	}

	return activeSensors[relativeChannel]->Configure(ts, mCode, heater, gb, reply);
}

GCodeResult DhtSensorHardwareInterface::Configure(TemperatureSensor *ts, unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	GCodeResult rslt = GCodeResult::ok;
	if (mCode == 305)
	{
		bool seen = false;
		ts->TryConfigureHeaterName(gb, seen);

		if (gb.Seen('T'))
		{
			seen = true;

			const int dhtType = gb.GetIValue();
			switch (dhtType)
			{
			case 11:
				type = DhtSensorType::Dht11;
				break;
			case 21:
				type = DhtSensorType::Dht21;
				break;
			case 22:
				type = DhtSensorType::Dht22;
				break;
			default:
				reply.copy("Invalid DHT sensor type");
				rslt = GCodeResult::error;
				break;
			}
		}

		if (!seen && !gb.Seen('X'))
		{
			ts->CopyBasicHeaterDetails(heater, reply);

			const char *sensorTypeString;
			switch (type)
			{
			case DhtSensorType::Dht11:
				sensorTypeString = "DHT11";
				break;
			case DhtSensorType::Dht21:
				sensorTypeString = "DHT21";
				break;
			case DhtSensorType::Dht22:
				sensorTypeString = "DHT22";
				break;
			default:
				sensorTypeString = "unknown";
				break;
			}
			reply.catf(", sensor type %s", sensorTypeString);
		}
	}
	return rslt;
}

// Create a hardware interface object for the specified channel if there isn't already
DhtSensorHardwareInterface *DhtSensorHardwareInterface::Create(unsigned int relativeChannel)
{
	if (relativeChannel >= MaxSpiTempSensors)
	{
		return nullptr;
	}

	MutexLocker lock(dhtMutex);

	if (activeSensors[relativeChannel] == nullptr)
	{
		activeSensors[relativeChannel] = new DhtSensorHardwareInterface(SpiTempSensorCsPins[relativeChannel]);
	}

	if (dhtTask == nullptr)
	{
		dhtTask = new Task<DhtTaskStackWords>;
		dhtTask->Create(DhtTaskStart, "DHTSENSOR", nullptr, TaskPriority::DhtPriority);
	}

	return activeSensors[relativeChannel];
}

/*static*/ void DhtSensorHardwareInterface::InitStatic()
{
	dhtMutex.Create("DHT");
}

/*static*/ TemperatureError DhtSensorHardwareInterface::GetTemperatureOrHumidity(unsigned int relativeChannel, float& t, bool wantHumidity)
{
	if (relativeChannel >= MaxSpiTempSensors)
	{
		t = BadErrorTemperature;
		return TemperatureError::unknownChannel;
	}

	MutexLocker lock(dhtMutex);

	if (activeSensors[relativeChannel] == nullptr)
	{
		t = BadErrorTemperature;
		return TemperatureError::notInitialised;
	}

	return activeSensors[relativeChannel]->GetTemperatureOrHumidity(t, wantHumidity);
}

void DhtSensorHardwareInterface::Interrupt()
{
	if (numPulses < ARRAY_SIZE(pulses))
	{
		const uint16_t now = StepTimer::GetInterruptClocks16();
		if (IoPort::ReadPin(sensorPin))
		{
			lastPulseTime = now;
		}
		else if (lastPulseTime != 0)
		{
			pulses[numPulses++] = now - lastPulseTime;
		}
	}
}

void DhtSensorHardwareInterface::TakeReading()
{
	if (type != DhtSensorType::none)			// if sensor has been configured
	{
		// Send the start bit. This must be at least 18ms for the DHT11, 0.8ms for the DHT21, and 1ms long for the DHT22.
		IoPort::SetPinMode(sensorPin, OUTPUT_LOW);
		delay(20);

		{
			TaskCriticalSectionLocker lock;		// make sure the Heat task doesn't interrupt the sequence

			// End the start signal by setting data line high. the sensor will respond with the start bit in 20 to 40us.
			// We need only force the data line high long enough to charge the line capacitance, after that the pullup resistor keeps it high.
			IoPort::WriteDigital(sensorPin, HIGH);		// this will generate an interrupt, but we will ignore it
			delayMicroseconds(3);

			// Now start reading the data line to get the value from the DHT sensor
			IoPort::SetPinMode(sensorPin, INPUT_PULLUP);

			// It appears that switching the pin to an output disables the interrupt, so we need to call attachInterrupt here
			// We are likely to get an immediate interrupt at this point corresponding to the low-to-high transition. We must ignore this.
			numPulses = ARRAY_SIZE(pulses);		// tell the ISR not to collect data yet
			attachInterrupt(sensorPin, DhtDataTransition, INTERRUPT_MODE_CHANGE, this);
			lastPulseTime = 0;
			numPulses = 0;						// tell the ISR to collect data
		}

		// Wait for the incoming signal to be read by the ISR (1 start bit + 40 data bits), or until timeout.
		// We don't have the ISR wake the process up, because that would require the priority of the pin change interrupt to be reduced.
		// So we just delay for long enough for the data to have been sent. It takes typically 4 to 5ms.
		delay(MaximumReadTime);

		detachInterrupt(sensorPin);

		// Attempt to convert the signal into temp+RH values
		const TemperatureError rslt = ProcessReadings();
		if (rslt == TemperatureError::success)
		{
			lastResult = rslt;
			badTemperatureCount = 0;
		}
		else if (badTemperatureCount < MaxBadTemperatureCount)
		{
			badTemperatureCount++;
		}
		else
		{
			lastResult = rslt;
			lastTemperature = BadErrorTemperature;
			lastHumidity = BadErrorTemperature;
		}
	}
}

// Code executed by the DHT sensor task.
// This is run at the same priority as the Heat task, so it must not sit in any spin loops.
/*static*/ [[noreturn]] void DhtSensorHardwareInterface::SensorTask()
{
	for (;;)
	{
		for (DhtSensorHardwareInterface *&sensor : activeSensors)
		{
			{
				MutexLocker lock(dhtMutex);

				if (sensor != nullptr)
				{
					sensor->TakeReading();
				}
			}
			delay(MinimumReadInterval/MaxSpiTempSensors);
		}
	}
}

// Process a reading. If success then update the temperature and humidity and return TemperatureError::success.
// Else return the TemperatureError code but do not update the readings.
TemperatureError DhtSensorHardwareInterface::ProcessReadings()
{
	// Check enough bits received and check start bit
	if (numPulses != ARRAY_SIZE(pulses) || pulses[0] < MinimumOneBitStepClocks)
	{
//		debugPrintf("pulses %u p0 %u\n", numPulses, (unsigned int)pulses[0]);
		return TemperatureError::ioError;
	}

	// Reset 40 bits of received data to zero
	uint8_t data[5] = { 0, 0, 0, 0, 0 };

	// Inspect each high pulse and determine which ones are 0 (less than 50us) or 1 (more than 50us). Ignore the start bit.
	for (size_t i = 0; i < 40; ++i)
	{
		data[i / 8] <<= 1;
		if (pulses[i + 1] >= MinimumOneBitStepClocks)
		{
			data[i / 8] |= 1;
		}
	}

//	debugPrintf("Data: %02x %02x %02x %02x %02x\n", data[0], data[1], data[2], data[3], data[4]);
	// Verify checksum
	if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4])
	{
//		debugPrintf("Cks err\n");
		return TemperatureError::ioError;
	}

	// Generate final results
	switch (type)
	{
	case DhtSensorType::Dht11:
		lastHumidity = data[0];
		lastTemperature = data[2];
		return TemperatureError::success;

	case DhtSensorType::Dht21:
	case DhtSensorType::Dht22:
		lastHumidity = ((data[0] * 256) + data[1]) * 0.1;
		lastTemperature = (((data[2] & 0x7F) * 256) + data[3]) * 0.1;
		if (data[2] & 0x80)
		{
			lastTemperature *= -1.0;
		}
		return TemperatureError::success;

	default:
		return TemperatureError::notInitialised;
	}
}

// Class DhtTemperatureSensor members
DhtTemperatureSensor::DhtTemperatureSensor(unsigned int channel)
	: TemperatureSensor(channel, "DHT-temperature")
{
}

void DhtTemperatureSensor::Init()
{
	DhtSensorHardwareInterface::Create(GetSensorChannel() - FirstDhtTemperatureChannel);
}

GCodeResult DhtTemperatureSensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	return DhtSensorHardwareInterface::Configure(this, GetSensorChannel() - FirstDhtTemperatureChannel, mCode, heater, gb, reply);
}

DhtTemperatureSensor::~DhtTemperatureSensor()
{
	// We don't delete the hardware interface object because the humidity channel may still be using it
}

TemperatureError DhtTemperatureSensor::TryGetTemperature(float& t)
{
	return DhtSensorHardwareInterface::GetTemperatureOrHumidity(GetSensorChannel() - FirstDhtTemperatureChannel, t, false);
}

// Class DhtHumiditySensor members
DhtHumiditySensor::DhtHumiditySensor(unsigned int channel)
	: TemperatureSensor(channel, "DHT-humidity")
{
}

void DhtHumiditySensor::Init()
{
	DhtSensorHardwareInterface::Create(GetSensorChannel() - FirstDhtHumidityChannel);
}

GCodeResult DhtHumiditySensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply)
{
	return DhtSensorHardwareInterface::Configure(this, GetSensorChannel() - FirstDhtHumidityChannel, mCode, heater, gb, reply);
}

DhtHumiditySensor::~DhtHumiditySensor()
{
	// We don't delete the hardware interface object because the temperature channel may still be using it
}

TemperatureError DhtHumiditySensor::TryGetTemperature(float& t)
{
	return DhtSensorHardwareInterface::GetTemperatureOrHumidity(GetSensorChannel() - FirstDhtHumidityChannel, t, true);
}

#endif

// End
