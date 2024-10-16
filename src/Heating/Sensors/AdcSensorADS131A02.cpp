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
								 | (3u << 0);			// enable channels 0 and 1

const AdcSensorADS131A02::InitTableEntry AdcSensorADS131A02::initTable[] =
{
	{ ADS131Register::A_SYS_CFG,	A_SYS_CFG_val},
	{ ADS131Register::D_SYS_CFG,	D_SYS_CFG_val},
	{ ADS131Register::CLK1,			CLK1_val},
	{ ADS131Register::CLK2,			CLK2_val},
	{ ADS131Register::ADC_ENA,		ADC_ENA_val}
};

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor AdcSensorADS131A02::typeDescriptor_16bit(TypeName_16bit, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new AdcSensorADS131A02(sensorNum, false); } );
TemperatureSensor::SensorTypeDescriptor AdcSensorADS131A02::typeDescriptor_24bit(TypeName_24bit, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new AdcSensorADS131A02(sensorNum, true); } );

AdcSensorADS131A02::AdcSensorADS131A02(unsigned int sensorNum, bool p_24bit) noexcept
	: SpiTemperatureSensor(sensorNum, (p_24bit) ? TypeName_24bit : TypeName_16bit, ADS131_SpiMode, ADS131_Frequency),
	  use24bitFrames(p_24bit)
{
}

// Configure this temperature sensor
GCodeResult AdcSensorADS131A02::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	gb.TryGetFValue('L', readingAtMin, changed);
	gb.TryGetFValue('H', readingAtMax, changed);

	if (!ConfigurePort(gb, reply, changed))
	{
		return GCodeResult::error;
	}

	ConfigureCommonParameters(gb, changed);
	return FinishConfiguring(changed, reply);
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult AdcSensorADS131A02::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = parser.GetFloatParam('L', readingAtMin);
	seen = parser.GetFloatParam('H', readingAtMax) || seen;

	if (!ConfigurePort(parser, reply, seen))
	{
		return GCodeResult::error;
	}

	return FinishConfiguring(seen, reply);
}

#endif

GCodeResult AdcSensorADS131A02::FinishConfiguring(bool changed, const StringRef& reply) noexcept
{
	if (changed)
	{
		CalcDerivedParameters();

		// Initialise the sensor
		InitSpi();
		TryInitAdc();

		TemperatureError rslt(TemperatureError::unknownError);
		float t;
		for (unsigned int i = 0; i < 3; ++i)		// try 3 times
		{
			rslt = TryGetLinearAdcTemperature(t);
			if (rslt == TemperatureError::ok)
			{
				break;
			}
			delay(MinimumReadInterval);
		}
		SetResult(t, rslt);

		if (rslt != TemperatureError::ok)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to initialise daughter board ADC: %s\n", rslt.ToString());
		}
	}
	else
	{
		CopyBasicDetails(reply);
		reply.catf(", reading range %.1f to %.1fC", (double)readingAtMin, (double)readingAtMax);
	}
	return GCodeResult::ok;
}

void AdcSensorADS131A02::Poll() noexcept
{
	float t;
	const TemperatureError rslt = TryGetLinearAdcTemperature(t);
	SetResult(t, rslt);
}

void AdcSensorADS131A02::CalcDerivedParameters() noexcept
{
//TODO	linearAdcDegCPerCount = (tempAt20mA - minLinearAdcTemp) / 4096.0;
}

// Wait for the device to become ready after a reset returning true if successful
TemperatureError AdcSensorADS131A02::WaitReady() const noexcept
{
	/* From the datasheet:
	 * When powering up the device or coming out of a power-on reset (POR) state, the ADC does not accept any commands.
	 * During this time, the host can poll the ADC until the command status response reads back FFDDh (DD denotes the channel count defined by the NU_CH[3:0] bits in the ID_MSB register),
	 * indicating that the ADC power-on reset cycle is complete and that the ADC is ready to accept commands.
	 * Use the UNLOCK command to enable the SPI interface and begin communication with the device.
	 * The command status response associated with the UNLOCK command is 0655h.
	 */
	uint16_t status;
	uint32_t readings[2];
	TemperatureError ret = DoTransaction(ADS131Command::nullcmd, ADS131Register::none, 0, status, readings);
	if (ret == TemperatureError::ok)
	{
		delay(10);
		for (unsigned int retry = 0; retry < 5; ++retry)
		{
			ret = DoTransaction(ADS131Command::nullcmd, ADS131Register::none, 0, status, readings);
			if (ret != TemperatureError::ok) { break; }
			if ((status & 0xFF00) == 0xFF00) { return TemperatureError::ok; }
		}
			ret = TemperatureError::notReady;
	}
	return ret;
}

TemperatureError AdcSensorADS131A02::TryInitAdc() const noexcept
{
	TemperatureError ret = WaitReady();
	if (ret == TemperatureError::ok)
	{
		uint16_t status;
		uint32_t readings[2];
		ret = DoTransaction(ADS131Command::reset, ADS131Register::none, 0, status, readings);
		if (ret == TemperatureError::ok)
		{
			ret == WaitReady();
			if (ret == TemperatureError::ok)
			{
				for (const InitTableEntry& x : initTable)
				{
					ret = DoTransaction(ADS131Command::wreg, x.regNum, x.val, status, readings);
					if (ret != TemperatureError::ok)
					{
						return ret;
					}
				}
			}
		}
	}
	return ret;
}

// Try to get a temperature reading from the linear ADC by doing an SPI transaction
TemperatureError AdcSensorADS131A02::TryGetLinearAdcTemperature(float& t) noexcept
{
#if 1
	t = BadErrorTemperature;
	return TemperatureError::unknownError;
#else
	/*
	 * The MCP3204 waits for a high input input bit before it does anything. Call this clock 1.
	 * The next input bit it high for single-ended operation, low for differential. This is clock 2.
	 * The next 3 input bits are the channel selection bits. These are clocks 3..5.
	 * Clock 6 produces a null bit on its trailing edge, which is read by the processor on clock 7.
	 * Clocks 7..18 produce data bits B11..B0 on their trailing edges, which are read by the MCU on the leading edges of clocks 8-19.
	 * If we supply further clocks, then clocks 18..29 are the same data but LSB first, omitting bit 0.
	 * Clocks 30 onwards will be zeros.
	 * So we need to use at least 19 clocks. We round this up to 24 clocks, and we check that the extra 5 bits we receive are the 5 least significant data bits in reverse order.
	 *
	 * MCP3204 & MCP3208
	 * Single CH0 "C0" - Differential CH0-CH1 "80"
	 * Single CH1 "C8" - Differential CH1-CH0 "88"
	 * Single CH2 "D0" - Differential CH2-CH3 "90"
	 * Single CH3 "D8" - Differential CH3-CH2 "98"
	 * MCP3208 Only
	 * Single CH4 "E0" - Differential CH4-CH5 "A0"
	 * Single CH5 "E8" - Differential CH5-CH4 "A8"
	 * Single CH6 "F0" - Differential CH6-CH7 "B0"
	 * Single CH7 "F8" - Differential CH7-CH6 "B8"
	 *
	 * These values represent clocks 1 to 5.
	 */

	const uint8_t adcData[] = { channelByte, 0x00, 0x00 };
	uint32_t rawVal;
	TemperatureError rslt = DoSpiTransaction(adcData, 3, rawVal);
	//debugPrintf("ADC data %u\n", rawVal);

	if (rslt == TemperatureError::ok)
	{
		const uint32_t adcVal1 = (rawVal >> 5) & ((1u << 13) - 1u);
		const uint32_t adcVal2 = ((rawVal & 1) << 5) | ((rawVal & 2) << 3) | ((rawVal & 4) << 1) | ((rawVal & 8) >> 1) | ((rawVal & 16) >> 3) | ((rawVal & 32) >> 5);
		if (adcVal1 >= 4096 || adcVal2 != (adcVal1 & ((1u << 6) - 1u)))
		{
			rslt = TemperatureError::badResponse;
		}
		else
		{
			t = minLinearAdcTemp + (linearAdcDegCPerCount * (float)adcVal1);
		}
	}
	return rslt;
#endif
}

// Send a command and receive the response
TemperatureError AdcSensorADS131A02::DoTransaction(ADS131Command command, ADS131Register regNum, uint8_t data, uint16_t &status, uint32_t readings[2]) const noexcept
{
	const uint16_t fullCommand = command | ((uint16_t)regNum << 8) | (uint16_t)data;
	TemperatureError rslt(TemperatureError::ok);
	if (use24bitFrames)
	{
		uint8_t sendBuffer[9];
		sendBuffer[0] = (fullCommand >> 8) & 0xFF;
		sendBuffer[1] = fullCommand & 0xFF;
		memset(sendBuffer + 2, 0, sizeof(sendBuffer) - 2);

		uint8_t receiveBuffer[9];
		rslt = DoSpiTransaction(sendBuffer, receiveBuffer, sizeof(sendBuffer));

		status = ((uint16_t)receiveBuffer[0] << 8) | receiveBuffer[1];
		readings[0] = ((uint32_t)receiveBuffer[3] << 16) | ((uint32_t)receiveBuffer[4] << 8) | receiveBuffer[5];
		readings[1] = ((uint32_t)receiveBuffer[6] << 16) | ((uint32_t)receiveBuffer[7] << 8) | receiveBuffer[8];
	}
	else
	{
		uint16_t sendBuffer[3];
		sendBuffer[0] = __builtin_bswap16(fullCommand);
		sendBuffer[1] = sendBuffer[2] = 0;

		uint16_t receiveBuffer[3];
		rslt = DoSpiTransaction(reinterpret_cast<const uint8_t *_ecv_array>(sendBuffer), reinterpret_cast<uint8_t *_ecv_array>(receiveBuffer), sizeof(sendBuffer));

		status = __builtin_bswap16(receiveBuffer[0]);
		readings[0] = (uint32_t)__builtin_bswap16(receiveBuffer[1]);
		readings[1] = (uint32_t)__builtin_bswap16(receiveBuffer[2]);
	}
	return rslt;
}

#endif

// End
