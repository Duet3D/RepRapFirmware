/*
 * ThermocoupleSensor31855.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

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

#include "ThermocoupleSensor31855.h"
#include "RepRap.h"
#include "Platform.h"
#include "Core.h"

const uint32_t MAX31855_Frequency = 4000000;	// maximum for MAX31855 is 5MHz

// SPI modes:
// If the inactive state of SCL is LOW (CPOL = 0) (in the case of the MAX31855, this is sampled on the falling edge of CS):
// The MAX31855 sets up the first data bit after the falling edge of CLK, and changes the data on each falling clock edge.
// So the SAM needs to sample data on the rising clock edge. This requires NCPHA = 1.
const uint8_t MAX31855_SpiMode = SPI_MODE_0;

// Define the minimum interval between readings
const uint32_t MinimumReadInterval = 100;		// minimum interval between reads, in milliseconds

ThermocoupleSensor31855::ThermocoupleSensor31855(unsigned int channel)
	: SpiTemperatureSensor(channel, "Thermocouple (MAX31855)", channel - FirstMax31855ThermocoupleChannel, MAX31855_SpiMode, MAX31855_Frequency)
{
}

// Perform the actual hardware initialization for attaching and using this device on the SPI hardware bus.
void ThermocoupleSensor31855::Init()
{
	InitSpi();
	lastReadingTime = millis();
}

TemperatureError ThermocoupleSensor31855::TryGetTemperature(float& t)
{
	if (inInterrupt() || millis() - lastReadingTime < MinimumReadInterval)
	{
		t = lastTemperature;
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
				t = lastTemperature = (float)(0.25 * (float)(int32_t)rawVal);
				lastResult = TemperatureError::success;
			}
		}
	}
	return lastResult;
}

// End
