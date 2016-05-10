#include "TemperatureSensor.h"
#include "RepRapFirmware.h"

// MAX31855 thermocouple chip
//
// The MAX31855 continuously samples a Type K thermocouple.  When the MAX31855
// is selected via its chip select (CS) pin, it unconditionally writes a 32 bit
// sequence onto the bus.  This sequence is designed such that we need only
// interpret the high 16 bits in order to ascertain the temperature reading and
// whether there a fault condition exists.  The low 16 bits provide the
// MAX31855's cold junction temperature (and thus the board temperature) as
// well as finer detail on any existing fault condition.
//
// The temperature read from the chip is a signed, two's-complement integer.
// As it is a 14 bit value (in units of one-quarter degrees Celsius), we
// convert it to a proper signed 16 bit value by adding two high bits.  The
// high bits added should both be zero if the value is positive (highest bit
// of the 14 bit value is zero), or both be one if the value is negative
// (highest bit of the 14 bit value is one).
//
// Note Bene: there's a Arduino Due sketch floating about the internet which
// gets this wrong and for negative temperatures generates incorrect values.
// E.g, -2047C for what should be -1C; -1798C for what should be -250C.  The
// values of -1C and -250C are shown as examples in Table 4 of the datasheet
// for the MAX21855.)  The incorrect Arduino Due sketch appears in, and may be
// from, the book Arduino Sketches: Tools and Techniques for Programming
// Wizardry, James A. Langbridge, January 12, 2015, John Wiley & Sons.

// Bits  -- Interpretation
// -----    -----------------------------------------------------------------
// 31:18 -- 14 bit, signed thermocouple temperature data.  Units of 0.25 C
//    17 -- Reserved
//    16 -- Fault indicator (1 if fault detected; 0 otherwise)
// 15:04 -- 12 bit, signed cold junction temperature data.  Units of 0.0625 C
//    03 -- Reserved
//    02 -- SCV fault; reads 1 if the thermocouple is shorted to Vcc
//    01 -- SCG fault; reads 1 if the thermocouple is shorted to ground
//    00 --  OC fault; reads 1 if the thermocouple is not connected (open)

// For purposes of setting bit transfer widths and timings, we need to use a
// Peripheral Channel Select (PCS).  Use channel #3 as it is unlikely to be
// used by anything else as the Arduino Due leaves pin 78 unconnected.
//
// No warranty given or implied, use at your own risk.
// dan.newman@mtbaldy.us
// GPL v3

const uint32_t MAX31855_Frequency = 4000000;	// maximum for MAX31855 is 5MHz
const uint32_t MAX31865_Frequency = 4000000;	// maximum for MAX31865 is also 5MHz

// SPI modes:
// If the inactive state of SCL is LOW (CPOL = 0) (in the case of the MAX31865, this is sampled on the falling edge of CS):
// The MAX31855 sets up the first data bit after the falling edge of CS, and changes the data on each falling clock edge.
// So the SAM needs to sample data on the rising clock edge. This requires NCPHA = 1.
// The MAX31865 changes data after the rising edge of CS, and samples input data on the falling edge.
// This requires NCPHA = 0.

const uint8_t MAX31855_SpiMode = SPI_MODE_0;
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

// Perform the actual hardware initialization for attaching and using this device on the SPI hardware bus.
void TemperatureSensor::InitThermocouple(uint8_t cs)
{
	device.csPin = cs;
	device.spiMode = MAX31855_SpiMode;
	device.clockFrequency = MAX31855_Frequency;
	sspi_master_init(&device, 8);

	lastReadingTime = millis();
	lastResult = TemperatureError::success;
	lastTemperature = 0.0;
}

// Perform the actual hardware initialization for attaching and using this device on the SPI hardware bus.
void TemperatureSensor::InitRtd(uint8_t cs)
{
	device.csPin = cs;
	device.spiMode = MAX31865_SpiMode;
	device.clockFrequency = MAX31865_Frequency;
	sspi_master_init(&device, 8);

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
		reprap.GetPlatform()->MessageF(GENERIC_MESSAGE, "Error: failed to initialise RTD: %s\n", TemperatureErrorString(rslt));
	}
}

// Try to initialise the RTD
TemperatureError TemperatureSensor::TryInitRtd() const
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

TemperatureError TemperatureSensor::GetThermocoupleTemperature(float *t)
{
	if (inInterrupt() || millis() - lastReadingTime < MinimumReadInterval)
	{
		*t = lastTemperature;
	}
	else
	{
		uint32_t rawVal;
		TemperatureError sts = DoSpiTransaction(nullptr, 4, rawVal);
		if (sts != TemperatureError::success)
		{
			lastResult = sts;
		}
		else
		{
			lastReadingTime = millis();

			if ((rawVal & 0x00020008) != 0)
			{
				// These two bits should always read 0. Likely the entire read was 0xFF 0xFF which is not uncommon when first powering up
				lastResult = TemperatureError::ioError;
			}
			else if ((rawVal & 0x00010007) != 0)		// check the fault bits
			{
				// Check for three more types of bad reads as we set the response code:
				//   1. A read in which the fault indicator bit (16) is set but the fault reason bits (0:2) are all clear;
				//   2. A read in which the fault indicator bit (16) is clear, but one or more of the fault reason bits (0:2) are set; and,
				//   3. A read in which more than one of the fault reason bits (0:1) are set.
				if ((rawVal & 0x00010000) == 0)
				{
					// One or more fault reason bits are set but the fault indicator bit is clear
					lastResult = TemperatureError::ioError;
				}
				else
				{
					// At this point we are assured that bit 16 (fault indicator) is set and that at least one of the fault reason bits (0:2) are set.
					// We now need to ensure that only one fault reason bit is set.
					uint8_t nbits = 0;
					if (rawVal & 0x01)
					{
						// Open Circuit
						++nbits;
						lastResult = TemperatureError::openCircuit;
					}
					if (rawVal & 0x02)
					{
						// Short to ground;
						++nbits;
						lastResult = TemperatureError::shortToGround;
					}
					if (rawVal && 0x04)
					{
						// Short to Vcc
						++nbits;
						lastResult = TemperatureError::shortToVcc;
					}

					if (nbits != 1)
					{
						// Fault indicator was set but a fault reason was not set (nbits == 0) or too many fault reason bits were set (nbits > 1).
						// Assume that a communication error with the MAX31855 has occurred.
						lastResult = TemperatureError::ioError;
					}
				}
			}
			else
			{
				rawVal >>= 18;							// shift the 14-bit temperature data to the bottom of the word
				rawVal |= (0 - (rawVal & 0x2000));		// sign-extend the sign bit

				// And convert to from units of 1/4C to 1C
				*t = lastTemperature = (float)(0.25 * (float)(int32_t)rawVal);
				lastResult = TemperatureError::success;
			}
		}
	}
	return lastResult;
}

TemperatureError TemperatureSensor::GetRtdTemperature(float *t)
{
	if (inInterrupt() || millis() - lastReadingTime < MinimumReadInterval)
	{
		*t = lastTemperature;
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
				//keep(low <= high; high <= NumTempTableEntries)
				//keep(low == 0u || tempTable[low - 1u].adcReading < adcVal)
				//keep(high == NumTempTableEntries || adcVal <= tempTable[high].adcReading)
				//decrease(high - low)
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
				//assert(low <= NumTempTableEntries)
				//assert(low == 0 || table[low - 1] < adcVal)
				//assert(low == NumTempTableEntries || adcVal <= table[low])

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
					*t = lastTemperature = ((float)(tempTable[low].temperature - tempTable[low - 1].temperature) * interpolationFraction)
							+ (float)tempTable[low - 1].temperature;
					//debugPrintf("raw %u low %u interp %f temp %f\n", adcVal, low, interpolationFraction, *t);
					lastResult = TemperatureError::success;
				}
			}
		}
	}
	return lastResult;
}

// Send and receive 1 to 4 bytes of data and return the result as a single 32-bit word
TemperatureError TemperatureSensor::DoSpiTransaction(const uint8_t dataOut[], size_t nbytes, uint32_t& rslt) const
{
	if (!sspi_acquire())
	{
		return TemperatureError::busBusy;
	}

	sspi_master_setup_device(&device);
	delayMicroseconds(1);
	sspi_select_device(&device);
	delayMicroseconds(1);

	uint8_t rawBytes[4];
	spi_status_t sts = sspi_transceive_packet(dataOut, rawBytes, nbytes);

	delayMicroseconds(1);
	sspi_deselect_device(&device);
	delayMicroseconds(1);

	sspi_release();

	if (sts != SPI_OK)
	{
		return TemperatureError::timeout;
	}

	rslt = rawBytes[0];
	for (size_t i = 1; i < nbytes; ++i)
	{
		rslt <<= 8;
		rslt |= rawBytes[i];
	}

	return TemperatureError::success;
}

// End
