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

void Accelerometers::ProcessReceivedData(CanAddress src, const CanMessageAccelerometerData& msg) noexcept
{
	if (msg.firstSampleNumber == 0)
	{
		// Close any existing file
		if (f != nullptr)
		{
			f->Write("Data incomplete\n");
			f->Close();
		}

		Platform& p = reprap.GetPlatform();
		const time_t time = p.GetDateTime();
		tm timeInfo;
		gmtime_r(&time, &timeInfo);
		String<StringLength50> temp;
		temp.printf("accelerometer/%u_%04u-%02u-%02u_%02u.%02u.%02u.csv",
						(unsigned int)src, timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
		f = p.OpenSysFile(temp.c_str(), OpenMode::write);

		if (f != nullptr)
		{
			currentBoard = src;
			axes = msg.axes;
			expectedSampleNumber = 0;
			temp.printf("Sample,");
			if (axes & 1u) { temp.cat("X,"); }
			if (axes & 2u) { temp.cat("Y,"); }
			if (axes & 4u) { temp.cat("Z,"); }
			temp.cat("Overflowed\n");
			f->Write(temp.c_str());
		}
	}

	if (f != nullptr)
	{
		if (msg.axes != axes || msg.firstSampleNumber != expectedSampleNumber || src != currentBoard)
		{
			f->Write("Received mismatched data\n");
			f->Close();
		}
		else
		{
			unsigned int numSamples = msg.numSamples;
			//TODO check that the message is long enough to hold this number of samples
			unsigned int numAxes = (axes & 1u) + ((axes & 2u) >> 1) + ((axes & 4u) >> 2);
			size_t dataIndex = 0;
			uint16_t currentBits = 0;
			unsigned int bitsLeft = 0;
			unsigned int resolution = msg.bitsPerSampleMinusOne + 1;
			uint16_t mask = (1u << resolution) - 1;
			while (numSamples != 0)
			{
				String<StringLength50> temp;
				temp.printf("%u,", expectedSampleNumber);
				++expectedSampleNumber;

				for (unsigned int axis = 0; axis < numAxes; ++axis)
				{
					// Extract one value from the message
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

					// Append it to the buffer
					temp.catf("%d,", (int16_t)val);
				}

				temp.cat((msg.overflowed) ? "1\n" : "0\n");
				f->Write(temp.c_str());
				--numSamples;
			}
		}
		if (msg.lastPacket)
		{
			f->Close();
		}
	}
}

#endif

// End
