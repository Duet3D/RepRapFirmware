/*
 * DhtSensor.cpp
 *
 *  Created on: 15 Sep 2017
 *      Author: Christian
 */

#include "DhtSensor.h"

#if SUPPORT_DHT_SENSOR

#include <Platform/RepRap.h>
#include <Heating/Heat.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/StepTimer.h>

constexpr uint16_t MinimumReadInterval = 2000;		// ms
constexpr uint8_t  MaximumReadTime = 20;			// ms
constexpr uint8_t  MinimumOneBitLength = 50;		// microseconds
constexpr uint32_t MinimumOneBitStepClocks = (StepClockRate * MinimumOneBitLength)/1000000;

// Pulse ISR
void DhtDataTransition(CallbackParameter cp) noexcept
{
	static_cast<DhtTemperatureSensor*>(cp.vp)->Interrupt();
}

// Class DhtTemperatureSensor members
DhtTemperatureSensor::DhtTemperatureSensor(unsigned int sensorNum, DhtSensorType t) noexcept
	: SensorWithPort(sensorNum, "DHT-temperature"), type(t), lastReadTime(0)
{
}

DhtTemperatureSensor::~DhtTemperatureSensor() noexcept
{
}

const char *DhtTemperatureSensor::GetShortSensorType() const noexcept
{
	switch (type)
	{
	case DhtSensorType::Dht21:			return TypeNameDht21;
	case DhtSensorType::Dht22:			return TypeNameDht22;
	default:							return "unknown";
	}
}

GCodeResult DhtTemperatureSensor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
#if SAME5x
	// SAME5x needs two ports because the output ports don't support interrupts
	if (gb.Seen('P'))
	{
		IoPort* const portAddrs[] = { &port, &interruptPort };
		PinAccess const accessModes[] = { PinAccess::write1, PinAccess::read };
		switch (IoPort::AssignPorts(gb, reply, PinUsedBy::sensor, 2, portAddrs, accessModes))
		{
		case 2:
			break;										// success

		case 0:
			return GCodeResult::error;					// error message has already been written to 'reply'

		default:
			reply.copy("on this board, this sensor type needs one output and one input port");
			return GCodeResult::error;
		}

		numPulses = ARRAY_SIZE(pulses);					// tell the ISR not to collect data yet
		if (!interruptPort.AttachInterrupt(DhtDataTransition, InterruptMode::change, CallbackParameter(this)))
		{
			reply.copy("failed to attach interrupt to port ");
			interruptPort.AppendPinName(reply);
			return GCodeResult::error;
		}
	}
#else
	if (!ConfigurePort(gb, reply, PinAccess::write1, changed))
	{
		return GCodeResult::error;
	}
#endif

	TryConfigureSensorName(gb, changed);

	// It's a new sensor
	if (gb.Seen('Y'))
	{
		changed = true;
		TakeReading();
	}

	if (!changed)
	{
#if SAME5x
		reply.printf("Sensor %u", GetSensorNumber());
		if (GetSensorName() != nullptr)
		{
			reply.catf(" (%s)", GetSensorName());
		}
		reply.catf(" type %s using pins ", GetSensorType());
		const IoPort* const portAddrs[] = { &port, &interruptPort };
		IoPort::AppendPinNames(reply, 2, portAddrs);
		reply.catf(", reading %.1f, last error: %s", (double)GetStoredReading(), TemperatureErrorString(GetLastError()));
#else
		CopyBasicDetails(reply);
#endif

		const char *sensorTypeString;
		switch (type)
		{
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

	return GCodeResult::ok;
}

TemperatureError DhtTemperatureSensor::GetLatestTemperature(float &t, uint8_t outputNumber) noexcept
{
	if (outputNumber > 1)
	{
		t = BadErrorTemperature;
		return TemperatureError::invalidOutputNumber;
	}
	const auto result = TemperatureSensor::GetLatestTemperature(t);
	if (outputNumber == 1)
	{
		t = lastHumidity;
	}
	return result;
}

void DhtTemperatureSensor::Interrupt() noexcept
{
	if (numPulses < ARRAY_SIZE(pulses))
	{
		const uint16_t now = StepTimer::GetTimerTicks16();
		if (port.ReadDigital())
		{
			lastPulseTime = now;
		}
		else if (lastPulseTime != 0)
		{
			pulses[numPulses++] = now - lastPulseTime;
		}
	}
}

void DhtTemperatureSensor::Poll() noexcept
{
	const auto now = millis();
	if ((now - lastReadTime) >= MinimumReadInterval)
	{
		TakeReading();
	}
	SetResult(GetStoredReading(), TemperatureError::success);
}

void DhtTemperatureSensor::TakeReading() noexcept
{
	const IoPort& irqPort =
#if SAME5x
		interruptPort;
#else
		port;
#endif

	// Send the start bit. This must be at least 0.8ms for the DHT21 and 1ms for the DHT22. We no longer support DHT11 because it needed 18ms.
	port.SetMode(PinAccess::write0);
	delay(2);

	{
		TaskCriticalSectionLocker lock;		// make sure the Heat task doesn't interrupt the sequence

		// End the start signal by setting data line high. the sensor will respond with the start bit in 20 to 40us.
		// We need only force the data line high long enough to charge the line capacitance, after that the pullup resistor keeps it high.
		numPulses = ARRAY_SIZE(pulses);		// tell the ISR not to collect data yet
		port.WriteDigital(true);			// this may generate an interrupt, but we will ignore it
		delayMicroseconds(3);

		// Now start reading the data line to get the value from the DHT sensor
		port.SetMode(PinAccess::readWithPullup_InternalUseOnly);

		// It appears that switching the pin to an output disables the interrupt, so we need to call attachInterrupt here
		// We are likely to get an immediate interrupt at this point corresponding to the low-to-high transition. We must ignore this.
		irqPort.AttachInterrupt(DhtDataTransition, InterruptMode::change, CallbackParameter(this));
		delayMicroseconds(2);				// give the interrupt time to occur
		lastPulseTime = 0;
		numPulses = 0;						// tell the ISR to collect data
	}

	// Wait for the incoming signal to be read by the ISR (1 start bit + 40 data bits), or until timeout.
	// We don't have the ISR wake the process up, because that would require the priority of the pin change interrupt to be reduced.
	// So we just delay for long enough for the data to have been sent. It takes typically 4 to 5ms.
	delay(MaximumReadTime);

	irqPort.DetachInterrupt();

	// Attempt to convert the signal into temp+RH values
	float t;
	const auto rslt = ProcessReadings(t, lastHumidity);
	if (rslt == TemperatureError::success)
	{
		SetResult(t, rslt);
		badTemperatureCount = 0;
	}
	else if (badTemperatureCount < MaxBadTemperatureCount)
	{
		badTemperatureCount++;
	}
	else
	{
		SetResult(rslt);
		lastHumidity = BadErrorTemperature;
	}
	lastReadTime = millis();
}

// Process a reading. If success then update the temperature and humidity and return TemperatureError::success.
// Else return the TemperatureError code but do not update the readings.
TemperatureError DhtTemperatureSensor::ProcessReadings(float& t, float& h) noexcept
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
	case DhtSensorType::Dht21:
	case DhtSensorType::Dht22:
		h = ((data[0] * 256) + data[1]) * 0.1;
		t = (((data[2] & 0x7F) * 256) + data[3]) * 0.1;
		if (data[2] & 0x80)
		{
			t *= -1.0;
		}
		return TemperatureError::success;

	default:
		return TemperatureError::notInitialised;
	}
}


// Class DhtHumiditySensor members
DhtHumiditySensor::DhtHumiditySensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, "DHT-humidity", false)
{
}

DhtHumiditySensor::~DhtHumiditySensor() noexcept
{
}


#endif

// End
