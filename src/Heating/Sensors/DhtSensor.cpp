/*
 * DhtSensor.cpp
 *
 *  Created on: 15 Sep 2017
 *      Author: Christian
 */

#include "DhtSensor.h"
#include "Platform.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"

#if SUPPORT_DHT_SENSOR

const uint32_t MinimumReadInterval = 2000;		// ms
const uint32_t MaximumReadTime = 50;			// ms


size_t DhtSensor::numInstances = 0;
DhtSensorType DhtSensor::type = DhtSensorType::Dht11;

uint32_t DhtSensor::lastReadTime = 0;
TemperatureError DhtSensor::lastResult = TemperatureError::notInitialised;
float DhtSensor::lastTemperature = BAD_ERROR_TEMPERATURE;
float DhtSensor::lastHumidity = BAD_ERROR_TEMPERATURE;

DhtSensor::SensorState DhtSensor::state = Initialising;
uint32_t DhtSensor::lastOperationTime = 0;


DhtSensor::DhtSensor(unsigned int channel) : TemperatureSensor(channel, "DHTxx")
{
	numInstances++;
}

DhtSensor::~DhtSensor()
{
	numInstances--;
}

bool DhtSensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply, bool& error)
{
	if (mCode == 305)
	{
		bool seen = false;
		TryConfigureHeaterName(gb, seen);

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
					error = true;
					reply.copy("Invalid DHT sensor type");
					break;
			}
		}

		if (!seen && !gb.Seen('X'))
		{
			CopyBasicHeaterDetails(heater, reply);

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
	return false;
}

void DhtSensor::Init()
{
	if (numInstances == 1)
	{
		pinMode(DhtDataPin, OUTPUT_LOW);
	}
}

TemperatureError DhtSensor::GetTemperature(float& t)
{
	switch (GetSensorChannel())
	{
		case DhtTemperatureChannel:
			t = lastTemperature;
			return lastResult;

		case DhtHumidityChannel:
			t = lastHumidity;
			return lastResult;

		default:
			return TemperatureError::unknownChannel;
	}
}

// Pulse ISR
uint32_t lastPulseTime;
volatile uint8_t numPulses;
uint32_t pulses[41];			// 1 start bit + 40 data bits

void DhtDataTransition(CallbackParameter)
{
	const uint32_t now = micros();
	if (digitalRead(DhtDataPin) == HIGH)
	{
		lastPulseTime = now;
	}
	else if (lastPulseTime > 0)
	{
		pulses[numPulses++] = now - lastPulseTime;
		if (numPulses == ARRAY_SIZE(pulses))
		{
			detachInterrupt(DhtDataPin);
		}
	}
}

// Keep this sensor running
/*static*/ void DhtSensor::Spin()
{
	if (numInstances == 0 || millis() - lastReadTime < MinimumReadInterval)
	{
		return;
	}

	switch (state)
	{
	case Initialising:
		// Send start signal. See DHT datasheet for full signal diagram:
		// http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf
		digitalWrite(DhtDataPin, HIGH);

		state = Starting;
		lastOperationTime = millis();
		break;

	case Starting:
		// Wait 250ms
		if (millis() - lastOperationTime >= 250)
		{
			pinMode(DhtDataPin, OUTPUT_LOW);
			digitalWrite(DhtDataPin, LOW);

			state = Starting2;
			lastOperationTime = millis();
		}
		break;

	case Starting2:
		// Wait 20ms
		if (millis() - lastOperationTime >= 20)
		{
			// End the start signal by setting data line high for 40 microseconds
			digitalWrite(DhtDataPin, HIGH);
			delayMicroseconds(40);

			// Now start reading the data line to get the value from the DHT sensor
			pinMode(DhtDataPin, INPUT_PULLUP);
			delayMicroseconds(10);

			// Read from the DHT sensor using an ISR, because we cannot use delays
			// due to the fact that this messes with the stepping ISR
			numPulses = 0;
			lastPulseTime = 0;
			attachInterrupt(DhtDataPin, DhtDataTransition, INTERRUPT_MODE_CHANGE, nullptr);

			// Wait for the next operation to complete
			lastOperationTime = millis();
			state = Reading;
		}
		break;

	case Reading:
		// Make sure we don't time out
		if (millis() - lastOperationTime > MaximumReadTime)
		{
			detachInterrupt(DhtDataPin);
			state = Initialising;
			lastReadTime = millis();
			lastResult = TemperatureError::timeout;
			break;
		}

		// Wait for the reading to complete (1 start bit + 40 data bits)
		if (numPulses != 41)
		{
			break;
		}

		// We're reading now - reset the state
		state = Initialising;
		lastReadTime = millis();

		// Check start bit
		if (pulses[0] < 40)
		{
			lastResult = TemperatureError::ioError;
			break;
		}

		// Reset 40 bits of received data to zero
		uint8_t data[5] = { 0, 0, 0, 0, 0 };

		// Inspect each high pulse and determine which ones
		// are 0 (less than 40us) or 1 (more than 40us)
		for (size_t i = 0; i < 40; ++i)
		{
			data[i / 8] <<= 1;
			if (pulses[i + 1] > 40)
			{
				data[i / 8] |= 1;
			}
		}

		// Verify checksum
		if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4])
		{
			lastResult = TemperatureError::ioError;
			break;
		}

		// Generate final results
		switch (type)
		{
			case DhtSensorType::Dht11:
				lastHumidity = data[0];
				lastTemperature = data[2];
				lastResult = TemperatureError::success;
				break;

			case DhtSensorType::Dht21:
			case DhtSensorType::Dht22:
				lastHumidity = ((data[0] * 256) + data[1]) * 0.1;
				lastTemperature = (((data[2] & 0x7F) * 256) + data[3]) * 0.1;
				if (data[2] & 0x80)
				{
					lastTemperature *= -1.0;
				}
				lastResult = TemperatureError::success;
				break;

			default:
				lastHumidity = BAD_ERROR_TEMPERATURE;
				lastTemperature = BAD_ERROR_TEMPERATURE;
				lastResult = TemperatureError::notInitialised;
				break;
		}
		break;
	}
}

#endif

// End
