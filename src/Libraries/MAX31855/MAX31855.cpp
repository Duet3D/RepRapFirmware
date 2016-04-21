#include "MAX31855.h"

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

#define PERIPHERAL_CHANNEL_ID       3
#define PERIPHERAL_CHANNEL_CS_PIN  78  // NPCS3

// 5.0 MHz Max clock frequency when clocking data out of a MAX31855 
#define MAX31855_MAX_FREQ 5000000u

// Perform the actual hardware initialization for attaching and using this
// device on the SPI hardware bus.
void MAX31855::Init(uint8_t cs)
{
	device.csPin = cs;
	pinMode(cs, OUTPUT);
	digitalWrite(cs, HIGH);

#ifdef DUET_NG
	sspi_master_init(&device, 8);
#else
	device.id   = PERIPHERAL_CHANNEL_ID;		// Peripheral channel
	sspi_master_init(&device, 16);
#endif
}

MAX31855_error MAX31855::getTemperature(float *t) const
{
	if (!sspi_acquire())
	{
		return MAX31855_GSPI_BUSY;
	}

	// Assume properly initialized
	// Ensure that the configuration is as needed; another GSPI consumer
	// may have changed the bus speed and/or timing delays.
	sspi_master_setup_device(&device, SPI_MODE_0, MAX31855_MAX_FREQ);

	// Select the device; enable CS (set it LOW)
	sspi_select_device(&device);
	delayMicroseconds(1);		// TODO shorten this (MAX31855 needs 100ns minimum)

	// Read in 32 bits

#ifdef DUET_NG
	uint8_t dataOut[4] = {0, 0, 0, 0};
	uint8_t rawBytes[4];
	spi_status_t sts = sspi_transceive_packet(dataOut, rawBytes, 4);
	uint16_t raw[2];
	raw[0] = ((uint16_t)rawBytes[0] << 8) | (uint16_t)rawBytes[1];
	raw[1] = ((uint16_t)rawBytes[2] << 8) | (uint16_t)rawBytes[3];
#else
	uint16_t dataOut[2] = {0, 0};
	uint16_t raw[2];
	spi_status_t sts = sspi_transceive_packet16(dataOut, raw, 2);
#endif

	// Deselect the device; disable CS (set it HIGH)
	sspi_deselect_device(&device);

	sspi_release();

	if (sts != SPI_OK)
	{
		return MAX31855_ERR_TMO;
	}

	if ((raw[0] & 0x02) || (raw[1] & 0x08))
	{
		// These two bits should always read 0
		// Likely the entire read was 0xFF 0xFF which is not uncommon when first powering up
		return MAX31855_ERR_IO;
	}

	// The MAX31855 response is designed such that we can look at
	// just the high 16 bits and know the temperature and whether
	// an error occurred.  The low 16 bits contain the cold junction
	// temperature and finer detail on the nature of any error.

	// We also want to check for three more types of bad reads:
	//
	//   1. A read in which the fault indicator bit (16) is set but the fault reason bits (0:2)
	//      are all clear;
	//   2. A read in which the fault indicator bit (16) is clear, but one or more of the fault
	//      reason bits (0:2) are set; and,
	//   3. A read in which more than one of the fault reason bits (0:1) are set.
	//
	// We will perform those three sanity checks as we set the error response code.

	if ((raw[0] & 0x01) || (raw[1] & 0x07))
	{
		if (!(raw[0] & 0x01))
		{
			// One or more fault reason bits are set but the fault indicator bit is clear?
			return MAX31855_ERR_IO;
		}

		// At this point we are assured that bit 16 (fault indicator) is set
		// and that at least one of the fault reason bits (0:2) are set.  We
		// now need to ensure that only one fault reason bit is set.
		uint8_t nbits = 0;
		MAX31855_error err;

		if (raw[1] & 0x01)
		{
			// Open Circuit
			++nbits;
			err = MAX31855_ERR_OC;
		}
		if (raw[1] & 0x02)
		{
			// Short to ground;
			++nbits;
			err = MAX31855_ERR_SCG;
		}
		if (raw[1] && 0x04)
		{
			// Short to Vcc
			++nbits;
			err = MAX31855_ERR_SCV;
		}

		if (nbits == 1)
		{
			// Looks like a legitimate error response: bit 16 was set and only one of bits 0:2 was set
			return err;
		}

		// Fault indicator was set but a fault reason was not set (nbits == 0) or too
		// many fault reason bits were set (nbits > 1): assume that a communication error
		// with the MAX31855 has occurred.
		return MAX31855_ERR_IO;
	}

	// Note, the reading is negative if raw[0] & 0x8000 is nonzero

	// Shift the data
	raw[0] >>= 2;
	int16_t temp = (int16_t)(raw[0] & 0x3fff);

	// Handle negative temperatures
	if (raw[0] & 0x2000)
	{
		temp |= 0xc000;
	}

	// And convert to from units of 1/4C to 1C
	*t = (float)(0.25 * (float)temp);

	// Success!
	return MAX31855_OK;
}

const char* MAX31855::errorStr(MAX31855_error err) const
{
	switch (err)
	{
	default                : return "unknown MAX31855 error";
	case MAX31855_OK       : return "successful temperature read";
	case MAX31855_ERR_SCV  : return "thermocouple is shorted to +Vcc";
	case MAX31855_ERR_SCG  : return "thermocouple is shorted to ground";
	case MAX31855_ERR_OC   : return "thermocouple is broken (open)";
	case MAX31855_ERR_TMO  : return "error communicating with MAX31855; read timed out";
	case MAX31855_ERR_IO   : return "error communicating with MAX31855; disconnected?";
	case MAX31855_GSPI_BUSY: return "general SPI bus is busy";
	}
}

// End
