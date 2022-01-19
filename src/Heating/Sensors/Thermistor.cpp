/*
 * Thermistor.cpp
 * Reads temperature from a thermistor or a PT1000 sensor connected to a thermistor port
 *
 *  Created on: 10 Nov 2016
 *      Author: David
 */

#include "Thermistor.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if HAS_VREF_MONITOR
# include <GCodes/GCodes.h>
# include <Hardware/NonVolatileMemory.h>
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#include <AnalogIn.h>
using
#if SAME5x
	AnalogIn
#else
	LegacyAnalogIn
#endif
	::AdcBits;

// For the theory behind ADC oversampling, see http://www.atmel.com/Images/doc8003.pdf
static constexpr unsigned int AdcOversampleBits = 2;							// we use 2-bit oversampling
static constexpr int32_t OversampledAdcRange = 1u << (AdcBits + AdcOversampleBits);	// The readings we pass in should be in range 0..(AdcRange - 1)

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
	  r25(DefaultThermistorR25), beta(DefaultThermistorBeta), shC(DefaultThermistorC), seriesR(DefaultThermistorSeriesR), adcFilterChannel(-1),
	  isPT1000(p_isPT1000), adcLowOffset(0), adcHighOffset(0)
{
	CalcDerivedParameters();
}

// Get the ADC reading
int32_t Thermistor::GetRawReading(bool& valid) const noexcept
{
	if (adcFilterChannel >= 0)
	{
		// Filtered ADC channel
		const volatile ThermistorAveragingFilter& tempFilter = reprap.GetPlatform().GetAdcFilter(adcFilterChannel);
		valid = tempFilter.IsValid();
		return tempFilter.GetSum()/(tempFilter.NumAveraged() >> AdcOversampleBits);
	}

	// Raw ADC channel
	valid = true;
	return (uint32_t)port.ReadAnalog() << AdcOversampleBits;
}

// Configure the H parameter returning true if successful, false if error
bool Thermistor::ConfigureHParam(int hVal, const StringRef& reply) noexcept
{
	if (hVal == 999)
	{
#if HAS_VREF_MONITOR
		bool valid;
		const int32_t val = GetRawReading(valid);
		if (valid)
		{
			int32_t vrefReading = reprap.GetPlatform().GetAdcFilter(VrefFilterIndex).GetSum();
# ifdef DUET3_MB6HC
			// Duet 3 MB6HC board revisions 0.6 and 1.0 have the series resistor connected to VrefP, not VrefMon, so extrapolate the VrefMon reading to estimate VrefP.
			// Version 1.01 and later boards have the series resistors connected to VrefMon.
			if (reprap.GetPlatform().GetBoardType() == BoardType::Duet3_6HC_v06_100)
			{
				const int32_t vssaReading = reprap.GetPlatform().GetAdcFilter(VssaFilterIndex).GetSum();
				vrefReading = vssaReading + lrintf((vrefReading - vssaReading) * (4715.0/4700.0));
			}
# endif
			const int32_t computedCorrection =
							(val - (int32_t)(vrefReading/(ThermistorAveragingFilter::NumAveraged() >> AdcOversampleBits)))
								/(1 << (AdcBits + AdcOversampleBits - 13));
			if (computedCorrection >= -127 && computedCorrection <= 127)
			{
				adcHighOffset = (int8_t)computedCorrection;
				reply.copy("Measured H correction for port \"");
				port.AppendPinName(reply);
				reply.catf("\" is %d", adcHighOffset);

				// Store the value in NVM
				if (!reprap.GetGCodes().IsRunningConfigFile())
				{
					NonVolatileMemory mem;
					mem.SetThermistorHighCalibration(adcFilterChannel, adcHighOffset);
					mem.EnsureWritten();
				}
			}
			else
			{
				reply.printf("Computed correction H%" PRIi32 " is out of range. Check that you have disconnected the thermistor.", computedCorrection);
				return false;
			}
		}
		else
		{
			reply.copy("Temperature reading is not valid");
			return false;
		}
#else
		reply.copy("Thermistor input auto calibration is not supported by this hardware");
		return false;
#endif
	}
	else
	{
		adcHighOffset = (int8_t)constrain<int>(hVal, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max());
	}
	return true;
}

// Configure the L parameter returning true if successful, false if error
bool Thermistor::ConfigureLParam(int lVal, const StringRef& reply) noexcept
{
	if (lVal == 999)
	{
#if HAS_VREF_MONITOR
		bool valid;
		const int32_t val = GetRawReading(valid);
		if (valid)
		{
			const int32_t computedCorrection =
							(val - (int32_t)(reprap.GetPlatform().GetAdcFilter(VssaFilterIndex).GetSum()/(ThermistorAveragingFilter::NumAveraged() >> AdcOversampleBits)))
								/(1 << (AdcBits + AdcOversampleBits - 13));
			if (computedCorrection >= -127 && computedCorrection <= 127)
			{
				adcLowOffset = (int8_t)computedCorrection;
				reply.copy("Measured L correction for port \"");
				port.AppendPinName(reply);
				reply.catf("\" is %d", adcLowOffset);

				// Store the value in NVM
				if (!reprap.GetGCodes().IsRunningConfigFile())
				{
					NonVolatileMemory mem;
					mem.SetThermistorLowCalibration(adcFilterChannel, adcLowOffset);
					mem.EnsureWritten();
				}
			}
			else
			{
				reply.printf("Computed correction L%" PRIi32 " is out of range. Check that you have placed a jumper across the thermistor input.", computedCorrection);
				return false;
			}
		}
		else
		{
			reply.copy("Temperature reading is not valid");
			return false;
		}
#else
		reply.copy("Thermistor input auto calibration is not supported by this hardware");
		return false;
#endif
	}
	else
	{
		adcLowOffset = (int8_t)constrain<int>(lVal, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max());
	}
	return true;
}

// Initialise parameters after changing the port
void Thermistor::InitPort() noexcept
{
	adcLowOffset = adcHighOffset = 0;
	Platform& p = reprap.GetPlatform();
	adcFilterChannel = p.GetAveragingFilterIndex(port);
	if (adcFilterChannel >= 0)
	{
		p.GetAdcFilter(adcFilterChannel).Init((1u << AdcBits) - 1);
#ifdef DUET_NG
		seriesR = p.GetDefaultThermistorSeriesR(adcFilterChannel);
#endif
#if HAS_VREF_MONITOR
		// Default the H and L parameters to the values from nonvolatile memory
		NonVolatileMemory mem;
		adcLowOffset = mem.GetThermistorLowCalibration(adcFilterChannel);
		adcHighOffset = mem.GetThermistorHighCalibration(adcFilterChannel);
#endif
	}
}

// Configure the temperature sensor
GCodeResult Thermistor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	if (!ConfigurePort(gb, reply, PinAccess::readAnalog, changed))
	{
		return GCodeResult::error;
	}

	if (changed)
	{
		InitPort();							// we changed the port, so clear the ADC corrections and set up the ADC filter if there is one
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

	if (gb.Seen('L'))
	{
		if (!ConfigureLParam(gb.GetIValue(), reply))
		{
			return GCodeResult::error;
		}
		changed = true;
	}

	if (gb.Seen('H'))
	{
		if (!ConfigureHParam(gb.GetIValue(), reply))
		{
			return GCodeResult::error;
		}
		changed = true;
	}

	TryConfigureSensorName(gb, changed);

	if (!changed)
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
		reply.catf(" L:%d H:%d", adcLowOffset, adcHighOffset);

		if (reprap.Debug(moduleHeat) && adcFilterChannel >= 0)
		{
#if HAS_VREF_MONITOR
			reply.catf(", Vref %" PRIu32 " Vssa %" PRIu32 " Th %" PRIu32,
				reprap.GetPlatform().GetAdcFilter(VrefFilterIndex).GetSum(),
				reprap.GetPlatform().GetAdcFilter(VssaFilterIndex).GetSum(),
				reprap.GetPlatform().GetAdcFilter(adcFilterChannel).GetSum());
#else
			reply.catf(", Th %" PRIu32, reprap.GetPlatform().GetAdcFilter(adcFilterChannel).GetSum());
#endif
		}
	}

	return GCodeResult::ok;
}

#if SUPPORT_REMOTE_COMMANDS

// Configure the temperature sensor
GCodeResult Thermistor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool changed = false;
	if (!ConfigurePort(parser, reply, PinAccess::readAnalog, changed))
	{
		return GCodeResult::error;
	}

	if (changed)
	{
		InitPort();							// we changed the port, so clear the ADC corrections and set up the ADC filter if there is one
	}

	changed = parser.GetFloatParam('R', seriesR) || changed;
	if (!isPT1000)
	{
		if (parser.GetFloatParam('B', beta))
		{
			shC = 0.0;						// if user changes B and doesn't define C, assume C=0
			changed = true;
		}
		changed = parser.GetFloatParam('C', shC) || changed;
		changed = parser.GetFloatParam('T', r25) || changed;
		if (changed)
		{
			CalcDerivedParameters();
		}
	}

	int16_t lVal;
	if (parser.GetIntParam('L', lVal))
	{
		if (!ConfigureLParam(lVal, reply))
		{
			return GCodeResult::error;
		}
		changed = true;
	}

	int16_t hVal;
	if (parser.GetIntParam('H', hVal))
	{
		if (!ConfigureHParam(hVal, reply))
		{
			return GCodeResult::error;
		}
		changed = true;
	}

	if (!changed)
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
		reply.catf(" L:%d H:%d", adcLowOffset, adcHighOffset);
	}

	return GCodeResult::ok;
}

#endif

// Get the temperature
void Thermistor::Poll() noexcept
{
	bool tempFilterValid;
	const int32_t averagedTempReading = GetRawReading(tempFilterValid);

#if HAS_VREF_MONITOR
	// Use the actual VSSA and VREF values read by the ADC
	const volatile ThermistorAveragingFilter& vrefFilter = reprap.GetPlatform().GetAdcFilter(VrefFilterIndex);
	const volatile ThermistorAveragingFilter& vssaFilter = reprap.GetPlatform().GetAdcFilter(VssaFilterIndex);
	if (tempFilterValid && vrefFilter.IsValid() && vssaFilter.IsValid())
	{
		const int32_t rawAveragedVssaReading = vssaFilter.GetSum()/(vssaFilter.NumAveraged() >> AdcOversampleBits);
		const int32_t rawAveragedVrefReading = vrefFilter.GetSum()/(vrefFilter.NumAveraged() >> AdcOversampleBits);
		const int32_t averagedVssaReading = rawAveragedVssaReading + (adcLowOffset * (1 << (AdcBits + AdcOversampleBits - 13)));
# ifdef DUET3_MB6HC
		// Duet 3 MB6HC board revisions 0.6 and 1.0 have the series resistor connected to VrefP, not VrefMon, so extrapolate the VrefMon reading to estimate VrefP.
		// Version 1.01 and later boards have the series resistors connected to VrefMon.
		const int32_t correctedVrefReading = (reprap.GetPlatform().GetBoardType() == BoardType::Duet3_6HC_v06_100)
											? rawAveragedVssaReading + lrintf((rawAveragedVrefReading - rawAveragedVssaReading) * (4715.0/4700.0))
											: rawAveragedVrefReading;
		const int32_t averagedVrefReading = correctedVrefReading + (adcHighOffset * (1 << (AdcBits + AdcOversampleBits - 13)));
# else
		const int32_t averagedVrefReading = rawAveragedVrefReading + (adcHighOffset * (1 << (AdcBits + AdcOversampleBits - 13)));
# endif

		// VREF is the measured voltage at VREF less the drop of a 15 ohm resistor.
		// VSSA is the voltage measured across the VSSA fuse. We assume the same maximum resistance for the fuse.
		// Assume a maximum ADC reading offset of 100.
		constexpr int32_t maxDrop = (OversampledAdcRange * VrefSeriesR)/(MinVrefLoadR + VrefSeriesR) + (100 << AdcOversampleBits);

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
			const int32_t averagedVrefReading = OversampledAdcRange + (adcHighOffset * (1 << (AdcBits + AdcOversampleBits - 13)));
			const int32_t averagedVssaReading = adcLowOffset * (1 << (AdcBits + AdcOversampleBits - 13));
#endif
			// Calculate the resistance
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
				float resistance = seriesR * (float)(averagedTempReading - averagedVssaReading)/(float)(averagedVrefReading - averagedTempReading);
#ifdef DUET_NG
				// The VSSA PTC fuse on the later Duets has a resistance of a few ohms. I measured 1.0 ohms on two revision 1.04 Duet WiFi boards.
				resistance -= 1.0;														// assume 1.0 ohms and only one PT1000 sensor
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
					const float logResistance = logf(resistance);
					const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;
					const float temp =  (recipT > 0.0) ? (1.0/recipT) + ABS_ZERO : BadErrorTemperature;

					// It's hard to distinguish between an open circuit and a cold high-resistance thermistor.
					// So we treat a temperature below -5C as an open circuit, unless we are using a low-resistance thermistor. The E3D thermistor has a resistance of about 470k @ -5C.
					if (temp < MinimumConnectedTemperature && resistance > seriesR * 100)
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
