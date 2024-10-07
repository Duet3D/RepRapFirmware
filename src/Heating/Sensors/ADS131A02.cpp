/*
 * ADS131A02.cpp
 *
 *  Created on: 7 Oct 2024
 *      Author: David
 */

#include "ADS131A02.h"

#if SUPPORT_SPI_SENSORS && SUPPORT_ADS131A02

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

const uint32_t ADS131_Frequency = 15000000;		// maximum for ADS131A02 is 25MHz for a single device, using 1:1 mark-space ratio

// The ADS131 samples input data on the falling edge and changes the output data on the rising edge.
const SpiMode ADS131_SpiMode = SPI_MODE_1;

// Define the minimum interval between readings
const uint32_t MinimumReadInterval = 3;			// minimum interval between reads, in milliseconds

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor ADS131A02::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new ADS131A02(sensorNum); } );

ADS131A02::ADS131A02(unsigned int sensorNum) noexcept
	: SpiTemperatureSensor(sensorNum, TypeName, ADS131_SpiMode, ADS131_Frequency)
{
	// TODO Auto-generated constructor stub
}

// Configure this temperature sensor
GCodeResult ADS131A02::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
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

GCodeResult ADS131A02::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
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

GCodeResult ADS131A02::FinishConfiguring(bool changed, const StringRef& reply) noexcept
{
	if (changed)
	{
		CalcDerivedParameters();

		// Initialise the sensor
		InitSpi();

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

void ADS131A02::Poll() noexcept
{
	float t;
	const TemperatureError rslt = TryGetLinearAdcTemperature(t);
	SetResult(t, rslt);
}

void ADS131A02::CalcDerivedParameters() noexcept
{
//TODO	linearAdcDegCPerCount = (tempAt20mA - minLinearAdcTemp) / 4096.0;
}

// Try to get a temperature reading from the linear ADC by doing an SPI transaction
TemperatureError ADS131A02::TryGetLinearAdcTemperature(float& t) noexcept
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

#endif

// End
