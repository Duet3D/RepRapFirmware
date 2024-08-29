/*
 * DataCollection.cpp
 *
 *  Created on: 26 Aug 2024
 *      Author: Andy Everitt
 */

#include "DataCollection.h"

#if SUPPORT_DATA_COLLECTION

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Movement/Move.h>
#include <Heating/Heat.h>
#include <Heating/Sensors/LinearAnalogSensor.h>

namespace DataCollection
{
	static uint32_t lastTransmissionTime = 0;

	static struct Buffer
	{
		uint8_t buffer[MaxBufferLen];
		size_t len;

		Buffer()
		{
			Clear();
		}

		void Clear()
		{
			memset(buffer, 0, MaxBufferLen);
			len = 0;


		}

		bool AddData(uint8_t data)
		{
			if (len >= MaxBufferLen)
			{
				return false;
			}

			buffer[len] = data;		// first 3 bytes are STX & num bytes
			len++;
			return true;
		}
	} buffer;

	// Convert a float to an ASCII array of length 5, implied decimal place before last 2 characters
	static bool SerialiseFloat(float input, uint8_t* output, size_t &len)
	{
		// Do not support number that take more than 5 digits (including 2 decimal)
		if (unlikely(abs(input) > 999.99))
		{
			input = input < 0 ? -999.99 : 999.99;
			return false;
		}

		char charBuf[8];		// sign, decimal, 5 digits, end character
		SafeSnprintf(charBuf, 8, "%.2f", (double)input);
		len = strlen(charBuf);

		for (size_t i = 0; i < len; i++)
		{
			output[i] = static_cast<uint8_t>(charBuf[i]);
		}
		return true;
	}

	bool SendDataToUart()
	{
		Platform& platform = reprap.GetPlatform();

		platform.SendUartData(DefaultAuxChannel, buffer.buffer, buffer.len);
		return true;
	}

	void ClearBuffer()
	{
		buffer.Clear();
	}

	inline bool AddDataToBuffer(uint32_t val)
	{
		bool failed = false;
		char buf[11];
		SafeSnprintf(buf, 11, "%lu", val);
		for (size_t i = 0; i < strlen(buf); i++)
		{
			failed |= AddDataToBuffer((uint8_t)buf[i]);
		}
		return !failed;
	}

	inline bool AddDataToBuffer(uint8_t val)
	{
		return buffer.AddData(val);
	}

	bool AddDataToBuffer(uint8_t *data, size_t len)
	{
		bool failed = false;
		for (size_t i = 0; i < len; i++)
		{
			failed |= AddDataToBuffer(data[i]);
		}
		return !failed;
	}

	static void AddAxisPosition(size_t axisOrExtruder)
	{
		Move& move = reprap.GetMove();

		int32_t currentMicrosteps = move.GetLiveMotorPosition(axisOrExtruder);
		if (currentMicrosteps < 0)
		{
			AddDataToBuffer((uint8_t)'-');
		}
		float pos = currentMicrosteps / move.DriveStepsPerMm(axisOrExtruder);
		uint8_t asciiPos[5] = {0};
		size_t len = 5;
		SerialiseFloat(pos, asciiPos, len);
		AddDataToBuffer(asciiPos, len);
	}

	void CollectAndSendData()
	{
		ClearBuffer();

		// Add timestamp
		lastTransmissionTime = millis();
		AddDataToBuffer(lastTransmissionTime);
		AddDataToBuffer((uint8_t)',');

		// Add X,Y,Z position to buffer
		for (size_t axis = 0; axis <= 2; axis++)
		{
			AddAxisPosition(axis);
			AddDataToBuffer((uint8_t)',');
		}

		// Add E0 position to buffer
		AddAxisPosition(ExtruderToLogicalDrive(0));
		AddDataToBuffer((uint8_t)',');

		// Add analog sensor
		// TODO will need to poll this faster
		const auto sensor = reprap.GetHeat().FindSensor(AnalogSensorNum);
		uint16_t reading = 0;
		if (sensor.IsNotNull())
		{
			float fReading;
			sensor->GetLatestTemperature(fReading);
			reading = static_cast<uint16_t>(fReading);
		}

		AddDataToBuffer((uint32_t)reading);
		AddDataToBuffer((uint8_t)'\n');

		SendDataToUart();
	}

	uint32_t GetLastTransmissionTime()
	{
		return lastTransmissionTime;
	}
}
#endif


