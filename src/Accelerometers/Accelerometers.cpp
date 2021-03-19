/*
 * Accelerometers.cpp
 *
 *  Created on: 19 Mar 2021
 *      Author: David
 */

#include "Accelerometers.h"

#if SUPPORT_ACCELEROMETERS

#include <CanMessageFormats.h>
#include <Storage/MassStorage.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>

static FileStore *f = nullptr;
static unsigned int expectedSampleNumber;
static CanAddress currentBoard = CanId::NoAddress;
static uint8_t axes;

void Accelerometers::ProcessReceivedData(CanAddress src, const CanMessageAccelerometerData& msg, size_t msgLen) noexcept
{
	if (msg.firstSampleNumber == 0)
	{
		// Close any existing file
		if (f != nullptr)
		{
			f->Write("Data incomplete\n");
			f->Close();
			f = nullptr;
		}

		Platform& p = reprap.GetPlatform();
		const time_t time = p.GetDateTime();
		tm timeInfo;
		gmtime_r(&time, &timeInfo);
		String<StringLength50> temp;
		temp.printf("0:/sys/accelerometer/%u_%04u-%02u-%02u_%02u.%02u.%02u.csv",
						(unsigned int)src, timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		f = MassStorage::OpenFile(temp.c_str(), OpenMode::write, 0);
		if (f != nullptr)
		{
			currentBoard = src;
			axes = msg.axes;
			expectedSampleNumber = 0;
			temp.printf("Sample,Rate,Overflowed");
			if (axes & 1u) { temp.cat(",X"); }
			if (axes & 2u) { temp.cat(",Y"); }
			if (axes & 4u) { temp.cat(",Z"); }
			temp.cat('\n');
			f->Write(temp.c_str());
		}
	}

	if (f != nullptr)
	{
		if (msgLen < msg.GetActualDataLength())
		{
			f->Write("Received bad data\n");
			f->Close();
			f = nullptr;
		}
		else if (msg.axes != axes || msg.firstSampleNumber != expectedSampleNumber || src != currentBoard)
		{
			f->Write("Received mismatched data\n");
			f->Close();
			f = nullptr;
		}
		else
		{
			unsigned int numSamples = msg.numSamples;
			const unsigned int numAxes = (axes & 1u) + ((axes >> 1) & 1u) + ((axes >> 2) & 1u);
			size_t dataIndex = 0;
			uint16_t currentBits = 0;
			unsigned int bitsLeft = 0;
			const unsigned int resolution = msg.bitsPerSampleMinusOne + 1;
			const uint16_t mask = (1u << resolution) - 1;
			const unsigned int bitsAfterPoint = resolution - 2;					// assumes the range is +/- 2g
			const int decimalPlaces = (bitsAfterPoint >= 11) ? 4 : (bitsAfterPoint >= 8) ? 3 : 2;
			while (numSamples != 0)
			{
				String<StringLength50> temp;
				temp.printf("%u,%u,%u", expectedSampleNumber, msg.actualSampleRate, msg.overflowed);
				++expectedSampleNumber;

				for (unsigned int axis = 0; axis < numAxes; ++axis)
				{
					// Extract one value from the message. A value spans at most two words in the buffer.
					uint16_t val = currentBits;
					if (bitsLeft >= resolution)
					{
						bitsLeft -= resolution;
						currentBits >>= resolution;
					}
					else
					{
						currentBits = msg.data[dataIndex++];
						val |= currentBits << bitsLeft;
						currentBits >>= resolution - bitsLeft;
						bitsLeft += 16 - resolution;
					}
					val &= mask;

					// Sign-extend it
					if (val & (1u << (resolution - 1)))
					{
						val |= ~mask;
					}

					// Convert it to a float number of g
					const float fVal = (float)(int16_t)val/(float)(1u << bitsAfterPoint);

					// Append it to the buffer
					temp.catf(",%.*f", decimalPlaces, (double)fVal);
				}

				temp.cat('\n');
				f->Write(temp.c_str());
				--numSamples;
			}
		}
		if (msg.lastPacket)
		{
			f->Close();
			f = nullptr;
		}
	}
}

#endif

// End
