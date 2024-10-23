/*
 * ADS131A02.cpp
 *
 *  Created on: 7 Oct 2024
 *      Author: David
 */

#include "AdcSensorADS131A02.h"

#if SUPPORT_SPI_SENSORS && SUPPORT_ADS131A02

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

/*
 * The ADS131A02 is a 2-channel 24-bit signal-delta ADC.
 * The SPI interface can be configured in the following modes depending on the connections of the M1 and M2 pins (see Table 10 in the datasheet)
 * 1. 16-bit data words. In this a case only the most significant 16 bits of data are returned for each channel.
 * 2. 24-bit data words. We can choose to have all 24 bits of data, or 16 bits of data and a 8-bit Hamming code.
 * 3. 32-bit data words. All 24 bits of data are returned and optionally a 8-bit Hamming code.
 * The device operates in fixed frame or dynamic frame mode.
 * The first word returned is always the status word, with the status is the most significant 16 bits of the word.
 * After that the data for each channel is returned. In dynamic frame mode, data for disabled channels is omitted.
 * Finally, if CRC is enabled in the control register then a CRC word is returned containing the CRC in the most significant 16 bits.
 *
 * This driver doesn't use the CRC or Hamming codes, and uses fixed frame mode.
 * To use 16-bit data words, leave pin M1 floating and connect pin M2 to ground.
 * To use 24-bit data words, connect pin M1 to ground and M2 to ground.
 *
 * We use the SPI interface of the device in Synchronous Slave Mode.
 */

const uint32_t ADS131_Frequency = 15000000;				// maximum for ADS131A02 is 25MHz for a single device, using 1:1 mark-space ratio

// The ADS131 samples input data on the falling edge and changes the output data on the rising edge. The clock is low when inactive.
const SpiMode ADS131_SpiMode = SPI_MODE_1;

// Define the minimum interval between readings
const uint32_t MinimumReadInterval = 3;					// minimum interval between reads, in milliseconds

// Define the values we set the configuration registers to
constexpr uint8_t A_SYS_CFG_val =  (1u << 7)			// negative charge pump enable
								 | (1u << 6)			// high resolution mode
								 | (1u << 5)			// reserved bit 5, always write 1
								 | (0u << 4)			// Vref =2.442V
								 | (0u << 3)			// enable internal reference
								 | (0u << 0);			// fault detection threshold

constexpr uint8_t D_SYS_CFG_val =  (0u << 7)			// WDT disable
								 | (0u << 6)			// CRC mode, not relevant because we disable the CRC
								 | (3u << 4)			// maximum DONE delay (default) - we don't use the DONE output so it shouldn't matter
								 | (3u << 2)			// maximum HiZ delay on Dout (default)
								 | (1u << 1)			// fixed 4 words per frame
								 | (0u << 0);			// disable CRC

constexpr uint8_t CLK1_val = 	   (0u << 7)			// use crystal as clock source
								 | (0u << 4)			// reserved bits 4..6, write 0
								 | (4u << 1)			// clock divider 8
								 | (0u << 0);			// reserved bit 0, write 0

constexpr uint8_t CLK2_val =	   (4u << 5)			// fmod = fclk/8 (default)
								 | (0u << 4)			// reserved bit 4, write 0
								 | (6u << 0);			// oversampling ratio, fdata = fmod/400 (default)

constexpr uint8_t  ADC_ENA_val =   (0u << 4)			// reserved bits 4..7, write 0
#if FOUR_CHANNELS
								 | (0x0fu << 0);		// enable channels 0-3
#else
								 | (3u << 0);			// enable channels 0 and 1
#endif

constexpr uint8_t ADC_GAIN_VAL = 0u;					// gain is 2^ADC_GAIN_VAL, max 2^4

// Table of initialisation data written to ADS131 registers
const AdcSensorADS131A02Chan0::InitTableEntry AdcSensorADS131A02Chan0::initTable[] =
{
	{ ADS131Register::A_SYS_CFG,	A_SYS_CFG_val	},
	{ ADS131Register::D_SYS_CFG,	D_SYS_CFG_val	},
	{ ADS131Register::CLK1,			CLK1_val		},
	{ ADS131Register::CLK2,			CLK2_val		},
	{ ADS131Register::ADC_ENA,		ADC_ENA_val		},
	{ ADS131Register::ADC1_GAIN,	ADC_GAIN_VAL	},
	{ ADS131Register::ADC2_GAIN,	ADC_GAIN_VAL	},
#if FOUR_CHANNELS
	{ ADS131Register::ADC3_GAIN,	ADC_GAIN_VAL	},
	{ ADS131Register::ADC4_GAIN,	ADC_GAIN_VAL	},
#endif
};

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor AdcSensorADS131A02Chan0::typeDescriptor_chan0_16bit(TypeName_chan0_16bit, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new AdcSensorADS131A02Chan0(sensorNum, false); } );
TemperatureSensor::SensorTypeDescriptor AdcSensorADS131A02Chan0::typeDescriptor_chan0_24bit(TypeName_chan0_24bit, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new AdcSensorADS131A02Chan0(sensorNum, true); } );
TemperatureSensor::SensorTypeDescriptor AdcSensorADS131A02Chan1::typeDescriptor_chan1(TypeName_chan1, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new AdcSensorADS131A02Chan1(sensorNum); } );

AdcSensorADS131A02Chan0::AdcSensorADS131A02Chan0(unsigned int sensorNum, bool p_24bit) noexcept
	: SpiTemperatureSensor(sensorNum, (p_24bit) ? TypeName_chan0_24bit : TypeName_chan0_16bit, ADS131_SpiMode, ADS131_Frequency),
	  use24bitFrames(p_24bit)
{
	for (float& f : readingAtMin) { f = DefaultReadingAtMin; }
	for (float& f : readingAtMax) { f = DefaultReadingAtMax; }
}

// Configure this temperature sensor
GCodeResult AdcSensorADS131A02Chan0::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	size_t numValues = NumChannels;
	gb.TryGetFloatArray('L', numValues, readingAtMin, changed, true);
	gb.TryGetFloatArray('H', numValues, readingAtMax, changed, true);

	if (!ConfigurePort(gb, reply, changed))
	{
		return GCodeResult::error;
	}

	ConfigureCommonParameters(gb, changed);
	return FinishConfiguring(changed, reply);
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult AdcSensorADS131A02Chan0::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	size_t numValues = NumChannels;
	if (parser.GetFloatArrayParam('L', numValues, readingAtMin))
	{
		seen = true;
	}

	numValues = NumChannels;
	if (parser.GetFloatArrayParam('H', numValues, readingAtMax))
	{
		seen = true;
	}

	if (!ConfigurePort(parser, reply, seen))
	{
		return GCodeResult::error;
	}

	return FinishConfiguring(seen, reply);
}

#endif

GCodeResult AdcSensorADS131A02Chan0::FinishConfiguring(bool changed, const StringRef& reply) noexcept
{
	if (changed)
	{
		CalcDerivedParameters();

		// Initialise the sensor
		InitSpi();
		TemperatureError rslt = TryInitAdc();
		if (rslt == TemperatureError::ok)
		{
			for (unsigned int i = 0; i < 3; ++i)		// try 3 times
			{
				rslt = TakeReading();
				if (rslt == TemperatureError::ok)
				{
					break;
				}
				delay(MinimumReadInterval);
			}
		}
		SetResult(lastReadings[0], rslt);

		if (rslt != TemperatureError::ok)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to initialise daughter board ADC: %s\n", rslt.ToString());
		}
	}
	else
	{
		CopyBasicDetails(reply);
		for (unsigned int chan = 0; chan < NumChannels; ++chan)
		{
			reply.catf(", channel %u reading range %.1f to %.1fC", chan, (double)readingAtMin[chan], (double)readingAtMax[chan]);
		}
	}
	return GCodeResult::ok;
}

TemperatureError AdcSensorADS131A02Chan0::GetAdditionalOutput(float &t, uint8_t outputNumber) noexcept
{
	if (outputNumber > 0 && outputNumber < NumChannels)
	{
		t = (lastResult == TemperatureError::ok) ? lastReadings[outputNumber] : BadErrorTemperature;
		return lastResult;
	}

	t = BadErrorTemperature;
	return TemperatureError::invalidOutputNumber;
}

void AdcSensorADS131A02Chan0::Poll() noexcept
{
	const TemperatureError rslt = TakeReading();
	SetResult(lastReadings[0], rslt);
}

void AdcSensorADS131A02Chan0::CalcDerivedParameters() noexcept
{
//TODO	linearAdcDegCPerCount = (tempAt20mA - minLinearAdcTemp) / 4096.0;
}

// Wait for the device to become ready after a reset returning true if successful
TemperatureError AdcSensorADS131A02Chan0::WaitReady() noexcept
{
	/* From the datasheet:
	 * When powering up the device or coming out of a power-on reset (POR) state, the ADC does not accept any commands.
	 * During this time, the host can poll the ADC until the command status response reads back FFDDh (DD denotes the channel count defined by the NU_CH[3:0] bits in the ID_MSB register),
	 * indicating that the ADC power-on reset cycle is complete and that the ADC is ready to accept commands.
	 * Use the UNLOCK command to enable the SPI interface and begin communication with the device.
	 * The command status response associated with the UNLOCK command is 0655h.
	 */
	uint16_t status;
	uint32_t readings[NumChannels];
	delay(10);
	TemperatureError ret = DoTransaction(ADS131Command::nullcmd, ADS131Register::none, 0, status, readings, false);
	if (ret == TemperatureError::ok)
	{
		for (unsigned int retry = 0; retry < 5; ++retry)
		{
			delay(10);
			ret = DoTransaction(ADS131Command::nullcmd, ADS131Register::none, 0, status, readings, false);
			if (ret != TemperatureError::ok)
			{
				break;
			}

			// The documentation says that the status word returned after a null command is 0xFFdd where dd is the hardware device ID.
			// It turns out that the ID is just the number of channels, so 02 for the ADS131A02 and 04 for the ADS131A04.
			expectedResponse = 0xFF00 | NumChannels;				// set up the expected response for the next SPI transaction
			if (status == expectedResponse)
			{
				return ret;
			}
		}
		ret = TemperatureError::notReady;
	}
	return ret;
}

TemperatureError AdcSensorADS131A02Chan0::TryInitAdc() noexcept
{
	// If the device has already been initialised then WaitReady won't work.
	// So send a Reset command first. This will fail if the device hasn't previously been initialised, but that doesn't matter.
	uint16_t status;
	uint32_t readings[2];
	TemperatureError ret = DoTransaction(ADS131Command::reset, ADS131Register::none, 0, status, readings, false);
	if (ret == TemperatureError::ok)
	{
		ret == WaitReady();
		if (ret == TemperatureError::ok)
		{
			ret = DoTransaction(ADS131Command::unlock, ADS131Register::none, 0, status, readings, true);
		}
		if (ret == TemperatureError::ok)
		{
			for (const InitTableEntry& x : initTable)
			{
				ret = DoTransaction(ADS131Command::wreg, x.regNum, x.val, status, readings, true);
				if (ret != TemperatureError::ok)
				{
					break;
				}
			}
		}
		if (ret == TemperatureError::ok)
		{
			ret = DoTransaction(ADS131Command::wakeup, ADS131Register::none, 0, status, readings, true);
		}

		// Send a null command so that the next register returned is the status register
		if (ret == TemperatureError::ok)
		{
			ret = DoTransaction(ADS131Command::nullcmd, ADS131Register::none, 0, status, readings, true);
		}
	}
	lastResult = ret;
	return ret;
}

// Try to get a temperature reading from the linear ADC by doing an SPI transaction
TemperatureError AdcSensorADS131A02Chan0::TakeReading() noexcept
{
	uint16_t status;
	uint32_t readings[NumChannels];
	TemperatureError ret = DoTransaction(ADS131Command::nullcmd, ADS131Register::none, 0, status, readings, false);
	if (ret == TemperatureError::ok)
	{
		for (size_t i = 0; i < NumChannels; ++i)
		{
			lastReadings[i] = readingAtMin[i] + ldexpf((float)(int32_t)readings[i], -32) * (readingAtMax[i] - readingAtMin[i]);
		}
		if ((status & 0x01101111b) != 0) { debugPrintf("Reading response %04x\n", status); }

		return ((status & 0x01101111b) == 0) ? ret : TemperatureError::badResponse;		// return badResponse if an error other than ADC input over-range has occurred
	}
	return ret;
}

// Send a command and receive the response
TemperatureError AdcSensorADS131A02Chan0::DoTransaction(ADS131Command command, ADS131Register regNum, uint8_t data, uint16_t &status, uint32_t readings[NumChannels], bool checkResponse) noexcept
{
	const uint16_t fullCommand = command | ((uint16_t)regNum << 8) | (uint16_t)data;
	TemperatureError rslt(TemperatureError::ok);
	if (use24bitFrames)
	{
		uint8_t sendBuffer[6 + NumChannels * 3];
		sendBuffer[0] = (fullCommand >> 8) & 0xFF;
		sendBuffer[1] = fullCommand & 0xFF;
		memset(sendBuffer + 2, 0, sizeof(sendBuffer) - 2);

		uint8_t receiveBuffer[6 + NumChannels * 3];
		rslt = DoSpiTransaction(sendBuffer, receiveBuffer, sizeof(sendBuffer));

		status = ((uint16_t)receiveBuffer[0] << 8) | receiveBuffer[1];
		for (size_t i = 0; i < NumChannels; ++i)
		{
			const size_t offset = i * 3;
			readings[i] =  ((uint32_t)receiveBuffer[3 + offset] << 24) | ((uint32_t)receiveBuffer[4 + offset] << 16) | ((uint32_t)receiveBuffer[5 + offset] << 8);
		}
	}
	else
	{
		uint16_t sendBuffer[2 + NumChannels];
		sendBuffer[0] = __builtin_bswap16(fullCommand);
		sendBuffer[1] = sendBuffer[2] = 0;

		uint16_t receiveBuffer[2 + NumChannels];
		rslt = DoSpiTransaction(reinterpret_cast<const uint8_t *_ecv_array>(sendBuffer), reinterpret_cast<uint8_t *_ecv_array>(receiveBuffer), sizeof(sendBuffer));

		status = __builtin_bswap16(receiveBuffer[0]);
		for (size_t i = 0; i < NumChannels; ++i)
		{
			readings[i] = (uint32_t)__builtin_bswap16(receiveBuffer[1 + i]) << 16;
		}
	}

	// If requested, check that the received response matches the previous command
	if (rslt == TemperatureError::ok && checkResponse && status != expectedResponse)
	{
		rslt = TemperatureError::badResponse;
		debugPrintf("Expected response %04x got %04x, current command %04x\n", expectedResponse, status, fullCommand);
	}

	// Set up the expected response for the next command
	expectedResponse = (command == ADS131Command::wreg) ? fullCommand ^ (ADS131Command::wreg ^ ADS131Command::rreg)
						: command;
	return rslt;
}

// Methods of second channel sensor object
AdcSensorADS131A02Chan1::AdcSensorADS131A02Chan1(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, TypeName_chan1, false)
{
}

#endif

// End
