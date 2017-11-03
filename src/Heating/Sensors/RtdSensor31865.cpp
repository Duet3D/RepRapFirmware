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

const uint16_t DefaultRef = 400;

//	 pt100rtd list of resistances v temperature
//
//	DIN 43760 / IEC 751 resistance values (ohms) were multiplied by 100 and
//	converted to 16 bit unsigned integers with no loss of accuracy.
//
//	Examples:
//	1852 represents 18.52 ohms and corresponds to a temperature of -200C.
//	10000 ==> 100.00 ohms @   0C
//	13851 ==> 138.51 ohms @ 100C

const float CelsiusMin = -100.0;					// starting temperature of the temp table below
const float CelsiusInterval = 10.0;

static const uint16_t tempTable[] =
{
	6026,  6430,  6833,  7233,  7633,  8031,  8427,  8822,  9216,  9609,
	10000, 10390, 10779, 11167, 11554, 11940, 12324, 12708, 13090, 13471,
	13851, 14229, 14607, 14983, 15358, 15733, 16105, 16477, 16848, 17217,
	17586, 17953, 18319, 18684, 19047, 19410, 19771, 20131, 20490, 20848,
	21205, 21561, 21915, 22268, 22621, 22972, 23321, 23670, 24018, 24364,
	24709, 25053, 25396, 25738, 26078, 26418, 26756, 27093, 27429, 27764,
	28098, 28430, 28762, 29092, 29421, 29749, 30075, 30401, 30725, 31048,
	31371, 31692, 32012, 32330, 32648, 32964, 33279, 33593, 33906, 34218,
	34528, 34838, 35146, 35453, 35759, 36064, 36367, 36670, 36971, 37271,
	37570, 37868, 38165, 38460, 38755, 39048
};

const size_t NumTempTableEntries = sizeof(tempTable)/sizeof(tempTable[0]);

RtdSensor31865::RtdSensor31865(unsigned int channel)
	: SpiTemperatureSensor(channel, "PT100 (MAX31865)", channel - FirstRtdChannel, MAX31865_SpiMode, MAX31865_Frequency),
	  rref(DefaultRef), cr0(DefaultCr0)
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

		if (gb.Seen('W'))
		{
			seen = true;
			if (gb.GetUIValue() == 3) 
			{
				cr0 |= 0x10;		// 3 wire configuration
			} 
			else 
			{
				cr0 &= ~0x10;		// 2 or 4 wire configuration
			}
		}

		if (gb.Seen('R'))
		{
			seen = true;
			rref = (uint16_t)gb.GetUIValue();
		}

		if (!seen && !gb.Seen('X'))
		{
			CopyBasicHeaterDetails(heater, reply);
			reply.catf(", %s wires, reject %dHz, reference resistor %u ohms", (cr0 & 0x10) ? "3" : "2/4", (cr0 & 0x01) ? 50 : 60, (unsigned int)rref);
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
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to initialise RTD: %s\n", TemperatureErrorString(rslt));
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
				const uint16_t ohmsx100 = (uint16_t)((((rawVal >> 1) & 0x7FFF) * rref * 100) >> 15);

				// Formally-verified binary search routine, adapted from one of the eCv examples
				size_t low = 0u, high = NumTempTableEntries;
				while (high > low)
				keep(low <= high; high <= NumTempTableEntries)
				keep(low == 0u || tempTable[low - 1u] < ohmsx100)
				keep(high == NumTempTableEntries || ohmsx100 <= tempTable[high])
				decrease(high - low)
				{
					size_t mid = (high - low)/2u + low;			// get the mid point, avoiding arithmetic overflow
					if (ohmsx100 <= tempTable[mid])
					{
						high = mid;
					}
					else
					{
						low = mid + 1u;
					}
				}
				assert(low <= NumTempTableEntries);
				assert(low == 0 || tempTable[low - 1] < ohmsx100);
				assert(low == NumTempTableEntries || ohmsx100 <= tempTable[low]);

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
					const float temperatureFraction = (float)(ohmsx100 - tempTable[low - 1])/(float)(tempTable[low] - tempTable[low - 1]);

					t = lastTemperature = CelsiusInterval * (low - 1 + temperatureFraction) + CelsiusMin;

					//debugPrintf("raw %f low %u temp %f\n", ohmsx100, low, t);
					lastResult = TemperatureError::success;
				}
			}
		}
	}
	return lastResult;
}

// End
