/*
 * RtdSensor31865.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "RtdSensor31865.h"
#include "RepRap.h"
#include "Platform.h"
#include "Core.h"

const uint32_t MAX31865_Frequency = 4000000;	// maximum for MAX31865 is also 5MHz

// SPI modes:
// If the inactive state of SCL is LOW (CPOL = 0) (in the case of the MAX31865, this is sampled on the falling edge of CS):
// The MAX31865 changes data after the rising edge of CLK, and samples input data on the falling edge.
// This requires NCPHA = 0.
const uint8_t MAX31865_SpiMode = SPI_MODE_1;

// Define the minimum interval between readings. The MAX31865 needs 62.5ms in 50Hz filter mode.
const uint32_t MinimumReadInterval = 100;		// minimum interval between reads, in milliseconds

// Table of temperature vs. MAX31865 result for PT100 thermistor, from the MAX31865 datasheet
struct TempTableEntry
{
	int16_t temperature;
	uint16_t adcReading;
};

static const TempTableEntry tempTable[] =
{
	{-30,	7227},
	{-20,	7550},
	{-10,	7871},
	{0,		8192},
	{10,	8512},
	{20,	8830},
	{30,	9148},
	{40,	9465},
	{50,	9781},
	{60,	10096},
	{70,	10410},
	{80,	10723},
	{90,	11035},
	{100,	11346},
	{110,	11657},
	{120,	11966},
	{130,	12274},
	{140,	12582},
	{150,	12888},
	{160,	13194},
	{170,	13498},
	{180,	13802},
	{190,	14104},
	{200,	14406},
	{225,	15156},
	{250,	15901},
	{275,	16639},
	{300,	17371},
	{325,	18098},
	{350,	18818},
	{375,	19533},
	{400,	20242},
	{425,	20945},
	{450,	21642},
	{475,	22333},
	{500,	23018},
	{525,	23697},
	{550,	24370}
};

const size_t NumTempTableEntries = sizeof(tempTable)/sizeof(tempTable[0]);

RtdSensor31865::RtdSensor31865(unsigned int channel)
	: SpiTemperatureSensor(channel, "PT100 (MAX31865)", channel - FirstRtdChannel, MAX31865_SpiMode, MAX31865_Frequency)
{
}

// Perform the actual hardware initialization for attaching and using this device on the SPI hardware bus.
void RtdSensor31865::Init()
{
	InitSpi();

	TemperatureError rslt;
	for (unsigned int i = 0; i < 3; ++i)		// try 3 times
	{
		rslt = TryInitRtd();
		if (rslt == TemperatureError::success)
		{
			break;
		}
		delay(MinimumReadInterval);
	}

	lastReadingTime = millis();
	lastResult = rslt;
	lastTemperature = 0.0;

	if (rslt != TemperatureError::success)
	{
		reprap.GetPlatform().MessageF(GENERIC_MESSAGE, "Error: failed to initialise RTD: %s\n", TemperatureErrorString(rslt));
	}
}

// Try to initialise the RTD
TemperatureError RtdSensor31865::TryInitRtd() const
{
	// Note that to get the MAX31865 to do continuous conversions, we need to set the bias bit as well as the continuous-conversion bit
	static const uint8_t modeData[2] = { 0x80, 0xC3 };		// write register 0, bias on, auto conversion, clear errors, 50Hz
	uint32_t rawVal;
	TemperatureError sts = DoSpiTransaction(modeData, 2, rawVal);

	if (sts == TemperatureError::success)
	{
		static const uint8_t readData[2] = { 0x00, 0x00 };		// read register 0
		sts = DoSpiTransaction(readData, 2, rawVal);
	}

	//debugPrintf("Status %d data %04x\n", (int)sts, rawVal);
	return (sts == TemperatureError::success && (uint8_t)rawVal != 0xC1)
				? TemperatureError::badResponse
				: sts;
}

TemperatureError RtdSensor31865::GetTemperature(float& t)
{
	if (inInterrupt() || millis() - lastReadingTime < MinimumReadInterval)
	{
		t = lastTemperature;
	}
	else
	{
		static const uint8_t dataOut[4] = {0, 55, 55, 55};		// read registers 0 (control), 1 (MSB) and 2 (LSB)
		uint32_t rawVal;
		TemperatureError sts = DoSpiTransaction(dataOut, 4, rawVal);

		if (sts != TemperatureError::success)
		{
			lastResult = sts;
		}
		else
		{
			lastReadingTime = millis();
			if (((rawVal & 0x00C10000) != 0xC10000)
#if 0
					// We no longer check the error status bit, because it seems to be impossible to clear it once it has been set.
					// Perhaps we would need to exit continuous reading mode to do so, and then re-enable it afterwards. But this would
					// take to long.
#else
					|| (rawVal & 1) != 0
#endif
					)
			{
				// Either the continuous conversion bit has got cleared, or the fault bit has been set
				TryInitRtd();
				lastResult = TemperatureError::hardwareError;
			}
			else
			{
				uint16_t adcVal = (rawVal >> 1) & 0x7FFF;

				// Formally-verified binary search routine, adapted from one of the eCv examples
				size_t low = 0u, high = NumTempTableEntries;
				while (high > low)
				keep(low <= high; high <= NumTempTableEntries)
				keep(low == 0u || tempTable[low - 1u].adcReading < adcVal)
				keep(high == NumTempTableEntries || adcVal <= tempTable[high].adcReading)
				decrease(high - low)
				{
					size_t mid = (high - low)/2u + low;			// get the mid point, avoiding arithmetic overflow
					if (adcVal <= tempTable[mid].adcReading)
					{
						high = mid;
					}
					else
					{
						low = mid + 1u;
					}
				}
				assert(low <= NumTempTableEntries);
				assert(low == 0 || tempTable[low - 1] < adcVal);
				assert(low == NumTempTableEntries || adcVal <= tempTable[low]);

				if (low == 0)									// if off the bottom of the table
				{
					lastResult = TemperatureError::shortCircuit;
				}
				else  if (low >= NumTempTableEntries)					// if off the top of the table
				{
					lastResult = TemperatureError::openCircuit;
				}
				else
				{
					const float interpolationFraction = (float)(adcVal - tempTable[low - 1].adcReading)/(float)(tempTable[low].adcReading - tempTable[low - 1].adcReading);
					t = lastTemperature = ((float)(tempTable[low].temperature - tempTable[low - 1].temperature) * interpolationFraction)
							+ (float)tempTable[low - 1].temperature;
					//debugPrintf("raw %u low %u interp %f temp %f\n", adcVal, low, interpolationFraction, *t);
					lastResult = TemperatureError::success;
				}
			}
		}
	}
	return lastResult;
}

// End
