/*
 * RtdSensor31865.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "RtdSensor31865.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"

const uint32_t MAX31865_Frequency = 4000000;	// maximum for MAX31865 is also 5MHz

// SPI modes:
// If the inactive state of SCL is LOW (CPOL = 0) (in the case of the MAX31865, this is sampled on the falling edge of CS):
// The MAX31865 changes data after the rising edge of CLK, and samples input data on the falling edge.
// This requires NCPHA = 0.
const uint8_t MAX31865_SpiMode = SPI_MODE_1;

// Define the minimum interval between readings. The MAX31865 needs 62.5ms in 50Hz filter mode.
const uint32_t MinimumReadInterval = 100;		// minimum interval between reads, in milliseconds

// Default configuration register
// Note that to get the MAX31865 to do continuous conversions, we need to set the bias bit as well as the continuous-conversion bit
//  Vbias=1
//  Conversion mode=1
//	1shot = 0
//	3wire=0
//	Fault detection=00 no action
//	Fault status=1 clear any existing fault
//	50/60Hz reject=1 for 50Hz (0 for 60Hz)
const uint8_t DefaultCr0 = 0b11000011;
const uint8_t Cr0ReadMask = 0b11011101;		// bits 1 and 5 auto clear, so ignore the value read

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
	: SpiTemperatureSensor(channel, "PT100 (MAX31865)", channel - FirstRtdChannel, MAX31865_SpiMode, MAX31865_Frequency),
	  cr0(DefaultCr0)
{
}

// Configure this temperature sensor
bool RtdSensor31865::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, StringRef& reply, bool& error)
{
	if (mCode == 305)
	{
		bool seen = false;
		TryConfigureHeaterName(gb, seen);
		if (gb.Seen('F'))
		{
			seen = true;
			if (gb.GetIValue() == 60)
			{
				cr0 &= ~0x01;		// set 60Hz rejection
			}
			else
			{
				cr0 |= 0x01;		// default to 50Hz rejection
			}
		}

		if (!seen && !gb.Seen('X'))
		{
			CopyBasicHeaterDetails(heater, reply);
			reply.catf(", reject %dHz", (cr0 & 0x01) ? 50 : 60);
		}
	}
	return false;
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
	const uint8_t modeData[2] = { 0x80, cr0 };			// write register 0
	uint32_t rawVal;
	TemperatureError sts = DoSpiTransaction(modeData, ARRAY_SIZE(modeData), rawVal);

	if (sts == TemperatureError::success)
	{
		static const uint8_t readData[2] = { 0x00, 0x00 };	// read register 0
		sts = DoSpiTransaction(readData, ARRAY_SIZE(readData), rawVal);
	}

	//debugPrintf("Status %d data %04x\n", (int)sts, rawVal);
	if (sts == TemperatureError::success && (rawVal & Cr0ReadMask) != (cr0 & Cr0ReadMask))
	{
		sts = TemperatureError::badResponse;
	}

	return sts;
}

TemperatureError RtdSensor31865::GetTemperature(float& t)
{
	if (inInterrupt() || millis() - lastReadingTime < MinimumReadInterval)
	{
		t = lastTemperature;
	}
	else
	{
		static const uint8_t dataOut[4] = {0, 0x55, 0x55, 0x55};			// read registers 0 (control), 1 (MSB) and 2 (LSB)
		uint32_t rawVal;
		const TemperatureError sts = DoSpiTransaction(dataOut, ARRAY_SIZE(dataOut), rawVal);

		if (sts != TemperatureError::success)
		{
			lastResult = sts;
		}
		else
		{
			lastReadingTime = millis();
			if (   (((rawVal >> 16) & Cr0ReadMask) != (cr0 & Cr0ReadMask))	// if control register not as expected
				|| (rawVal & 1) != 0										// or fault bit set
			   )
			{
				static const uint8_t faultDataOut[2] = {0x07, 0x55};
				if (DoSpiTransaction(faultDataOut, ARRAY_SIZE(faultDataOut), rawVal)== TemperatureError::success)	// read the fault register
				{
					lastResult = (rawVal & 0x04) ? TemperatureError::overOrUnderVoltage
								: (rawVal & 0x18) ? TemperatureError::openCircuit
									: TemperatureError::hardwareError;
				}
				else
				{
					lastResult = TemperatureError::hardwareError;
				}
				delayMicroseconds(1);										// MAX31865 requires CS to be high for 400ns minimum
				TryInitRtd();												// clear the fault and hope for better luck next time
			}
			else
			{
				const uint16_t adcVal = (rawVal >> 1) & 0x7FFF;

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
