/*
 * LinearAdcTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "CurrentLoopTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

const uint32_t MCP3204_Frequency = 1000000;		// maximum for MCP3204 is 1MHz @ 2.7V, will be slightly higher at 3.3V

// The MCP3204 samples input data on the rising edge and changes the output data on the rising edge.
const SpiMode MCP3204_SpiMode = SPI_MODE_0;

// Define the minimum interval between readings
const uint32_t MinimumReadInterval = 100;		// minimum interval between reads, in milliseconds

CurrentLoopTemperatureSensor::CurrentLoopTemperatureSensor(unsigned int sensorNum) noexcept
	: SpiTemperatureSensor(sensorNum, "Current Loop", MCP3204_SpiMode, MCP3204_Frequency),
	  tempAt4mA(DefaultTempAt4mA), tempAt20mA(DefaultTempAt20mA), chipChannel(DefaultChipChannel), isDifferential(false)
{
	CalcDerivedParameters();
}

// Configure this temperature sensor
GCodeResult CurrentLoopTemperatureSensor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	if (!ConfigurePort(gb, reply, changed))
	{
		return GCodeResult::error;
	}

	gb.TryGetFValue('L', tempAt4mA, changed);
	gb.TryGetFValue('H', tempAt20mA, changed);
	gb.TryGetUIValue('C', chipChannel, changed);
	gb.TryGetBValue('D', isDifferential, changed);
	ConfigureCommonParameters(gb, changed);
	return FinishConfiguring(changed, reply);
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult CurrentLoopTemperatureSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	if (!ConfigurePort(parser, reply, seen))
	{
		return GCodeResult::error;
	}

	return FinishConfiguring(seen, reply);
}

#endif

GCodeResult CurrentLoopTemperatureSensor::FinishConfiguring(bool changed, const StringRef& reply) noexcept
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
		reply.catf(", temperature range %.1f to %.1fC", (double)tempAt4mA, (double)tempAt20mA);
	}
	return GCodeResult::ok;
}

void CurrentLoopTemperatureSensor::Poll() noexcept
{
	float t;
	const TemperatureError rslt = TryGetLinearAdcTemperature(t);
	SetResult(t, rslt);
}

void CurrentLoopTemperatureSensor::CalcDerivedParameters() noexcept
{
	minLinearAdcTemp = tempAt4mA - 0.25 * (tempAt20mA - tempAt4mA);
	linearAdcDegCPerCount = (tempAt20mA - minLinearAdcTemp) / 4096.0;
}

// Try to get a temperature reading from the linear ADC by doing an SPI transaction
TemperatureError CurrentLoopTemperatureSensor::TryGetLinearAdcTemperature(float& t) noexcept
{
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

	const uint8_t channelByte = ((isDifferential) ? 0x80u : 0xC0u) | (chipChannel * 0x08u);
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
}

#endif // SUPPORT_SPI_SENSORS

// End
