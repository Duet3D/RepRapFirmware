/*
 * Thermistor.cpp
 * Reads temperature from a thermistor or a PT1000 sensor connected to a thermistor port
 *
 *  Created on: 10 Nov 2016
 *      Author: David
 */

#include "Thermistor.h"
#include "Platform.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

// The Steinhart-Hart equation for thermistor resistance is:
// 1/T = A + B ln(R) + C [ln(R)]^3
//
// The simplified (beta) equation assumes C=0 and is:
// 1/T = A + (1/Beta) ln(R)
//
// The parameters that can be configured in RRF are R25 (the resistance at 25C), Beta, and optionally C.

// Create an instance with default values
Thermistor::Thermistor(unsigned int sensorNum, bool p_isPT1000) noexcept
	: SensorWithPort(sensorNum, (p_isPT1000) ? "PT1000" : "Thermistor"),
	  r25(DefaultR25), beta(DefaultBeta), shC(DefaultShc), seriesR(DefaultThermistorSeriesR), adcFilterChannel(-1), isPT1000(p_isPT1000)
#if !HAS_VREF_MONITOR || defined(DUET3)
	  , adcLowOffset(0), adcHighOffset(0)
#endif
{
	CalcDerivedParameters();
}

// Configure the temperature sensor
GCodeResult Thermistor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	if (!ConfigurePort(gb, reply, PinAccess::readAnalog, changed))
	{
		return GCodeResult::error;
	}

	gb.TryGetFValue('R', seriesR, changed);
	if (!isPT1000)
	{
		bool seenB = false;
		gb.TryGetFValue('B', beta, seenB);
		if (seenB)
		{
			shC = 0.0;						// if user changes B and doesn't define C, assume C=0
			changed = true;
		}
		gb.TryGetFValue('C', shC, changed);
		gb.TryGetFValue('T', r25, changed);
		if (changed)
		{
			CalcDerivedParameters();
		}
	}

#if !HAS_VREF_MONITOR || defined(DUET3)
	if (gb.Seen('L'))
	{
		adcLowOffset = (int8_t)constrain<int>(gb.GetIValue(), std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max());
		changed = true;
	}
	if (gb.Seen('H'))
	{
		adcHighOffset = (int8_t)constrain<int>(gb.GetIValue(), std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max());
		changed = true;
	}
#endif

	TryConfigureSensorName(gb, changed);

	if (changed)
	{
		adcFilterChannel = reprap.GetPlatform().GetAveragingFilterIndex(port);
		if (adcFilterChannel >= 0)
		{
			reprap.GetPlatform().GetAdcFilter(adcFilterChannel).Init((1u << AdcBits) - 1);
		}
	}
	else
	{
		CopyBasicDetails(reply);
		if (isPT1000)
		{
			// For a PT1000 sensor, only the series resistor is configurable
			reply.catf(", R:%.1f", (double)seriesR);
		}
		else
		{
			reply.catf(", T:%.1f B:%.1f C:%.2e R:%.1f", (double)r25, (double)beta, (double)shC, (double)seriesR);
		}
#if !HAS_VREF_MONITOR || defined(DUET3)
		reply.catf(" L:%d H:%d", adcLowOffset, adcHighOffset);
#endif
	}

	return GCodeResult::ok;
}

// Get the temperature
void Thermistor::Poll() noexcept
{
	int32_t averagedTempReading;
	bool tempFilterValid;
	if (adcFilterChannel >= 0)
	{
		const volatile ThermistorAveragingFilter& tempFilter = reprap.GetPlatform().GetAdcFilter(adcFilterChannel);
		averagedTempReading = tempFilter.GetSum()/(tempFilter.NumAveraged() >> Thermistor::AdcOversampleBits);
		tempFilterValid = tempFilter.IsValid();
	}
	else
	{
		averagedTempReading = (uint32_t)port.ReadAnalog() << Thermistor::AdcOversampleBits;
		tempFilterValid = true;
	}

#if HAS_VREF_MONITOR
	// Use the actual VSSA and VREF values read by the ADC
	const volatile ThermistorAveragingFilter& vrefFilter = reprap.GetPlatform().GetAdcFilter(VrefFilterIndex);
	const volatile ThermistorAveragingFilter& vssaFilter = reprap.GetPlatform().GetAdcFilter(VssaFilterIndex);
	if (tempFilterValid && vrefFilter.IsValid() && vssaFilter.IsValid())
	{
# ifdef DUET3
		// Duet 3 MB6HC board revisions 0.6 and 1.0 have the series resistor connected to VrefP, not VrefMon.
		// So Extrapolate the reading for VrefP. We also allow offsets to be added.
		// Version 1.01 and later boards have the series resistors connected to VrefMon.
		const int32_t rawAveragedVssaReading = vssaFilter.GetSum()/(vssaFilter.NumAveraged() >> Thermistor::AdcOversampleBits);
		const int32_t rawAveragedVrefReading = vrefFilter.GetSum()/(vrefFilter.NumAveraged() >> Thermistor::AdcOversampleBits);
		const int32_t averagedVssaReading = rawAveragedVssaReading + (adcLowOffset * (1 << (AdcBits - 12 + Thermistor::AdcOversampleBits - 1)));
		const int32_t averagedVrefReading = ((reprap.GetPlatform().GetBoardType() == BoardType::Duet3_v06_100)
											? ((rawAveragedVrefReading - rawAveragedVssaReading) * (4715.0/4700.0)) + rawAveragedVssaReading
												: rawAveragedVrefReading
											) + (adcHighOffset * (1 << (AdcBits - 12 + Thermistor::AdcOversampleBits - 1)));
# else
		const int32_t averagedVssaReading = vssaFilter.GetSum()/(vssaFilter.NumAveraged() >> Thermistor::AdcOversampleBits);
		const int32_t averagedVrefReading = vrefFilter.GetSum()/(vrefFilter.NumAveraged() >> Thermistor::AdcOversampleBits);
# endif

		// VREF is the measured voltage at VREF less the drop of a 15 ohm resistor.
		// VSSA is the voltage measured across the VSSA fuse. We assume the same 15 ohms maximum resistance for the fuse.
		// Assume a maximum ADC reading offset of 100.
		constexpr int32_t maxDrop = (OversampledAdcRange * 15)/MinVrefLoadR + (100 << Thermistor::AdcOversampleBits);

		if (averagedVrefReading < OversampledAdcRange - maxDrop)
		{
			SetResult(TemperatureError::badVref);
		}
		else if (averagedVssaReading > maxDrop)
		{
			SetResult(TemperatureError::badVssa);
		}
		else
		{
#else
	if (tempFilterValid)
	{
		{
#endif

			// Calculate the resistance
#if HAS_VREF_MONITOR
			if (averagedVrefReading <= averagedTempReading)
			{
				SetResult((isPT1000) ? BadErrorTemperature : ABS_ZERO, TemperatureError::openCircuit);
			}
			else if (averagedTempReading <= averagedVssaReading)
			{
				SetResult(BadErrorTemperature, TemperatureError::shortCircuit);
			}
			else
			{
				const float resistance = seriesR * (float)(averagedTempReading - averagedVssaReading)/(float)(averagedVrefReading - averagedTempReading);
#else
			const int32_t averagedVrefReading = OversampledAdcRange + 2 * adcHighOffset;	// double the offset because we increased AdcOversampleBits from 1 to 2
			if (averagedVrefReading <= averagedTempReading)
			{
				SetResult((isPT1000) ? BadErrorTemperature : ABS_ZERO, TemperatureError::openCircuit);
			}
			else
			{
				const float denom = (float)(averagedVrefReading - averagedTempReading) - 0.5;
				const int32_t averagedVssaReading = 2 * adcLowOffset;					// double the offset because we increased AdcOversampleBits from 1 to 2
				float resistance = seriesR * ((float)(averagedTempReading - averagedVssaReading) + 0.5)/denom;
# ifdef DUET_NG
				// The VSSA PTC fuse on the later Duets has a resistance of a few ohms. I measured 1.0 ohms on two revision 1.04 Duet WiFi boards.
				resistance -= 1.0;														// assume 1.0 ohms and only one PT1000 sensor
# endif
#endif
				if (isPT1000)
				{
					// We want 100 * the equivalent PT100 resistance, which is 10 * the actual PT1000 resistance
					const uint16_t ohmsx100 = (uint16_t)lrintf(constrain<float>(resistance * 10, 0.0, 65535.0));
					float t;
					const TemperatureError sts = GetPT100Temperature(t, ohmsx100);
					SetResult(t, sts);
				}
				else
				{
					// Else it's a thermistor
					const float logResistance = log(resistance);
					const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;
					const float temp =  (recipT > 0.0) ? (1.0/recipT) + ABS_ZERO : BadErrorTemperature;

					if (temp < MinimumConnectedTemperature)
					{
						// Assume thermistor is disconnected
						SetResult(ABS_ZERO, TemperatureError::openCircuit);
					}
					else
					{
						SetResult(temp, TemperatureError::success);
					}
				}
			}
		}
	}
	else
	{
		// Filter is not ready yet
		SetResult(TemperatureError::notReady);
	}
}

// Calculate shA and shB from the other parameters
void Thermistor::CalcDerivedParameters() noexcept
{
	shB = 1.0/beta;
	const float lnR25 = logf(r25);
	shA = 1.0/(25.0 - ABS_ZERO) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;
}

// End
