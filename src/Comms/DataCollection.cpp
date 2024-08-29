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
			len = 3;


		}

		bool AddData(uint8_t data)
		{
			if (len >= MaxBufferLen - 3)
			{
				return false;
			}

			buffer[len] = data;		// first 3 bytes are STX & num bytes
			len++;
			return true;
		}

		void Finalise()
		{
			buffer[0] = 0x02;	// STX

			char numBytesBuf[3];
			SafeSnprintf(numBytesBuf, 3, "%.2u", len - 3);
			buffer[1] = static_cast<uint8_t>(numBytesBuf[0]);
			buffer[2] = static_cast<uint8_t>(numBytesBuf[1]);

			uint8_t checksum[2];
			CalculateUartCheckSum(buffer + 1, len - 1, checksum);
			buffer[len++] = checksum[0];
			buffer[len++] = checksum[1];

			buffer[len++] = '\n';
			buffer[len++] = 0x03;
		}
	} buffer;

	// Convert a float to an ASCII array of length 5, implied decimal place before last 2 characters
	static bool SerialiseFloat(float input, uint8_t* output)
	{
		uint32_t rounded = (uint32_t)(100 * abs(input) + 0.5);	// remove decimal place and sign

		// Do not support number that take more than 5 digits (including 2 decimal)
		if (rounded > 99999)
		{
			output[0] = '9';
			output[1] = '9';
			output[2] = '9';
			output[3] = '9';
			output[4] = '9';
			return false;
		}

		char charBuf[6];
		SafeSnprintf(charBuf, 6, "%.5lu", rounded);

		for (size_t i = 0; i < 5; i++)
		{
			output[i] = static_cast<uint8_t>(charBuf[i]);
		}
		return true;
	}

	bool SendDataToUart()
	{
		Platform& platform = reprap.GetPlatform();

		buffer.Finalise();

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
		SafeSnprintf(buf, 11, "%.10lu", val);
		for (size_t i = 0; i < 10; i++)
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
		else
		{
			AddDataToBuffer((uint8_t)'+');
		}
		float pos = currentMicrosteps / move.DriveStepsPerMm(axisOrExtruder);
		debugPrintf("Pos: %f (%ld) ", (double)pos, currentMicrosteps);
		uint8_t asciiPos[5] = {0};
		SerialiseFloat(pos, asciiPos);
		AddDataToBuffer(asciiPos, 5);
	}

	void CollectAndSendData()
	{
		ClearBuffer();

		// Add timestamp
		lastTransmissionTime = millis();
		AddDataToBuffer(lastTransmissionTime);

		// Add X,Y,Z position to buffer
		for (size_t axis = 0; axis <= 2; axis++)
		{
			AddAxisPosition(axis);
		}

		// Add E0 position to buffer
		AddAxisPosition(ExtruderToLogicalDrive(0));

		// Add analog sensor TODO
		const auto sensor = reprap.GetHeat().FindSensor(AnalogSensorNum);
		uint16_t reading = 0;
		if (sensor.IsNotNull())
		{
			float fReading;
			sensor->GetLatestTemperature(fReading);
			reading = static_cast<uint16_t>(fReading);
		}

		uint8_t asciiReading[4];
		ConvertHexToAsciiHex(reading >> 8, asciiReading);
		ConvertHexToAsciiHex(reading & 0xFF, asciiReading + 2);
		AddDataToBuffer(asciiReading, 4);

		SendDataToUart();
	}

	uint32_t GetLastTransmissionTime()
	{
		return lastTransmissionTime;
	}
}
#endif


