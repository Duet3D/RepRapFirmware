/*
 * Pid.cpp
 *
 *  Created on: 21 Jul 2016
 *      Author: David
 */

#include "Pid.h"
#include "GCodes/GCodes.h"
#include "Heat.h"
#include "HeaterProtection.h"
#include "Platform.h"
#include "RepRap.h"

// Private constants
const uint32_t InitialTuningReadingInterval = 250;	// the initial reading interval in milliseconds
const uint32_t TempSettleTimeout = 20000;	// how long we allow the initial temperature to settle

// Static class variables

float *PID::tuningTempReadings = nullptr;	// the readings from the heater being tuned
float PID::tuningStartTemp;					// the temperature when we turned on the heater
float PID::tuningPwm;						// the PWM to use
float PID::tuningTargetTemp;				// the maximum temperature we are allowed to reach
uint32_t PID::tuningBeginTime;				// when we started the tuning process
uint32_t PID::tuningPhaseStartTime;			// when we started the current tuning phase
uint32_t PID::tuningReadingInterval;		// how often we are sampling
size_t PID::tuningReadingsTaken;			// how many samples we have taken

float PID::tuningHeaterOffTemp;				// the temperature when we turned the heater off
float PID::tuningPeakTemperature;			// the peak temperature reached, averaged over 3 readings (so slightly less than the true peak)
uint32_t PID::tuningHeatingTime;			// how long we had the heating on for
uint32_t PID::tuningPeakDelay;				// how many milliseconds the temperature continues to rise after turning the heater off

#if HAS_VOLTAGE_MONITOR
unsigned int voltageSamplesTaken;			// how many readings we accumulated
float tuningVoltageAccumulator;				// sum of the voltage readings we take during the heating phase
#endif

// Member functions and constructors

PID::PID(Platform& p, int8_t h) : platform(p), heaterProtection(nullptr), heater(h), mode(HeaterMode::off), invertPwmSignal(false)
{
}

inline void PID::SetHeater(float power) const
{
	platform.SetHeater(heater, invertPwmSignal ? (1.0 - power) : power, model.GetPwmFrequency());
}

void PID::Init(float pGain, float pTc, float pTd, bool usePid, bool inverted)
{
	maxTempExcursion = DefaultMaxTempExcursion;
	maxHeatingFaultTime = DefaultMaxHeatingFaultTime;
	model.SetParameters(pGain, pTc, pTd, 1.0, GetHighestTemperatureLimit(), 0.0, usePid, inverted, 0);
	Reset();

	SetHeater(0.0);							// set up the pin even if the heater is not enabled (for PCCB)

	// Time the sensor was last sampled.  During startup, we use the current
	// time as the initial value so as to not trigger an immediate warning from the Tick ISR.
	lastSampleTime = millis();
}

void PID::Reset()
{
	mode = HeaterMode::off;
	previousTemperaturesGood = 0;
	previousTemperatureIndex = 0;
	activeTemperature = 0.0;
	standbyTemperature = 0.0;
	iAccumulator = 0.0;
	badTemperatureCount = 0;
	active = false; 						// default to standby temperature
	tuned = false;
	averagePWM = lastPwm = 0.0;
	heatingFaultCount = 0;
	temperature = BadErrorTemperature;
}

// Set the process model
bool PID::SetModel(float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, PwmFrequency pwmFreq)
{
	const float temperatureLimit = GetHighestTemperatureLimit();
	const bool rslt = model.SetParameters(gain, tc, td, maxPwm, temperatureLimit, voltage, usePid, inverted, pwmFreq);
	if (rslt)
	{
#if defined(DUET_06_085)
		if (heater == NumHeaters - 1)
		{
			// The last heater on the Duet 0.8.5 + DueX4 shares its pin with Fan1
			platform.EnableSharedFan(!model.IsEnabled());
		}
#endif
		if (model.IsEnabled())
		{
			const float predictedMaxTemp = gain + NormalAmbientTemperature;
			const float noWarnTemp = (temperatureLimit - NormalAmbientTemperature) * 1.5 + 50.0;		// allow 50% extra power plus enough for an extra 50C
			if (predictedMaxTemp > noWarnTemp)
			{
				platform.MessageF(WarningMessage,
						"Heater %u appears to be over-powered. If left on at full power, its temperature is predicted to reach %dC.\n",
						heater, (int)predictedMaxTemp);
			}
		}
		else
		{
			Reset();
		}
	}
	return rslt;
}

// Get the highest temperature limit
float PID::GetHighestTemperatureLimit() const
{
	return reprap.GetHeat().GetHighestTemperatureLimit(heater);
}

// Get the lowest temperature limit
float PID::GetLowestTemperatureLimit() const
{
	return reprap.GetHeat().GetLowestTemperatureLimit(heater);
}

// Read and store the temperature of this heater and returns the error code.
TemperatureError PID::ReadTemperature()
{
	TemperatureError err = TemperatureError::success;				// assume no error
	temperature = reprap.GetHeat().GetTemperature(heater, err);		// in the event of an error, err is set and BAD_ERROR_TEMPERATURE is returned
	return err;
}

// This must be called whenever the heater is turned on, and any time the heater is active and the target temperature is changed
void PID::SwitchOn()
{
	if (model.IsEnabled())
	{
		if (mode == HeaterMode::fault)
		{
			if (reprap.Debug(Module::moduleHeat))
			{
				platform.MessageF(WarningMessage, "Heater %d not switched on due to temperature fault\n", heater);
			}
		}
		else if (model.IsEnabled())
		{
			//debugPrintf("Heater %d on, temp %.1f\n", heater, temperature);
			const float target = (active) ? activeTemperature : standbyTemperature;
			const HeaterMode oldMode = mode;
			mode = (temperature + TEMPERATURE_CLOSE_ENOUGH < target) ? HeaterMode::heating
					: (temperature > target + TEMPERATURE_CLOSE_ENOUGH) ? HeaterMode::cooling
						: HeaterMode::stable;
			if (mode != oldMode)
			{
				heatingFaultCount = 0;
				if (mode == HeaterMode::heating)
				{
					timeSetHeating = millis();
				}
				if (reprap.Debug(Module::moduleHeat) && oldMode == HeaterMode::off)
				{
					platform.MessageF(GenericMessage, "Heater %d switched on\n", heater);
				}
			}
		}
	}
}

// Switch off the specified heater. If in tuning mode, delete the array used to store tuning temperature readings.
void PID::SwitchOff()
{
	lastPwm = 0.0;
	if (model.IsEnabled())
	{
		SetHeater(0.0);
		if (IsTuning())
		{
			delete tuningTempReadings;
			tuningTempReadings = nullptr;
		}
		if (mode > HeaterMode::off)
		{
			mode = HeaterMode::off;
			if (reprap.Debug(Module::moduleHeat))
			{
				platform.MessageF(GenericMessage, "Heater %d switched off\n", heater);
			}
		}
	}
}

// This is the main heater control loop function
void PID::Spin()
{
	if (model.IsEnabled())
	{
		// Read the temperature even if the heater is suspended
		const TemperatureError err = ReadTemperature();

		// Handle any temperature reading error and calculate the temperature rate of change, if possible
		if (err != TemperatureError::success)
		{
			previousTemperaturesGood <<= 1;				// this reading isn't a good one
			if (mode > HeaterMode::suspended)			// don't worry about errors when reading heaters that are switched off or flagged as having faults
			{
				// Error may be a temporary error and may correct itself after a few additional reads
				badTemperatureCount++;
				if (badTemperatureCount > MaxBadTemperatureCount)
				{
					lastPwm = 0.0;
					SetHeater(0.0);						// do this here just to be sure, in case the call to platform.Message causes a delay
					if (IsTuning())
					{
						delete tuningTempReadings;
						tuningTempReadings = nullptr;
					}
					mode = HeaterMode::fault;
					reprap.GetGCodes().HandleHeaterFault(heater);
					platform.MessageF(ErrorMessage, "Temperature reading fault on heater %d: %s\n", heater, TemperatureErrorString(err));
					reprap.FlagTemperatureFault(heater);
				}
			}
			// We leave lastPWM alone if we have a temporary temperature reading error
		}
		else
		{
			// We have an apparently-good temperature reading. Calculate the derivative, if possible.
			float derivative = 0.0;
			bool gotDerivative = false;
			badTemperatureCount = 0;
			if ((previousTemperaturesGood & (1 << (NumPreviousTemperatures - 1))) != 0)
			{
				const float tentativeDerivative = ((float)SecondsToMillis/HeatSampleIntervalMillis) * (temperature - previousTemperatures[previousTemperatureIndex])
								/ (float)(NumPreviousTemperatures);
				// Some sensors give occasional temperature spikes. We don't expect the temperature to increase by more than 10C/second.
				if (fabsf(tentativeDerivative) <= 10.0)
				{
					derivative = tentativeDerivative;
					gotDerivative = true;
				}
			}
			previousTemperatures[previousTemperatureIndex] = temperature;
			previousTemperaturesGood = (previousTemperaturesGood << 1) | 1;

			// Get the target temperature and the error
			const float targetTemperature = (active) ? activeTemperature : standbyTemperature;
			const float error = targetTemperature - temperature;

			// Do the heating checks
			switch(mode)
			{
			case HeaterMode::heating:
				{
					if (error <= TEMPERATURE_CLOSE_ENOUGH)
					{
						mode = HeaterMode::stable;
						heatingFaultCount = 0;
					}
					else if (gotDerivative)
					{
						const float expectedRate = GetExpectedHeatingRate();
						if (derivative + AllowedTemperatureDerivativeNoise < expectedRate
							&& (float)(millis() - timeSetHeating) > model.GetDeadTime() * SecondsToMillis * 2)
						{
							++heatingFaultCount;
							if (heatingFaultCount * HeatSampleIntervalMillis > maxHeatingFaultTime * SecondsToMillis)
							{
								SetHeater(0.0);					// do this here just to be sure
								mode = HeaterMode::fault;
								reprap.GetGCodes().HandleHeaterFault(heater);
								platform.MessageF(ErrorMessage, "Heating fault on heater %d, temperature rising much more slowly than the expected %.1f" DEGREE_SYMBOL "C/sec\n",
											heater, (double)expectedRate);
								reprap.FlagTemperatureFault(heater);
							}
						}
						else if (heatingFaultCount != 0)
						{
							--heatingFaultCount;
						}
					}
					else
					{
						// Leave the heating fault count alone
					}
				}
				break;

			case HeaterMode::stable:
				if (fabsf(error) > maxTempExcursion && temperature > MaxAmbientTemperature)
				{
					++heatingFaultCount;
					if (heatingFaultCount * HeatSampleIntervalMillis > maxHeatingFaultTime * SecondsToMillis)
					{
						SetHeater(0.0);					// do this here just to be sure
						mode = HeaterMode::fault;
						reprap.GetGCodes().HandleHeaterFault(heater);
						platform.MessageF(ErrorMessage, "Heating fault on heater %d, temperature excursion exceeded %.1f" DEGREE_SYMBOL "C\n",
											heater, (double)maxTempExcursion);
					}
				}
				else if (heatingFaultCount != 0)
				{
					--heatingFaultCount;
				}
				break;

			case HeaterMode::cooling:
				if (-error <= TEMPERATURE_CLOSE_ENOUGH && targetTemperature > MaxAmbientTemperature)
				{
					// We have cooled to close to the target temperature, so we should now maintain that temperature
					mode = HeaterMode::stable;
					heatingFaultCount = 0;
				}
				else
				{
					// We could check for temperature excessive or not falling here, but without an alarm or a power-off mechanism, there is not much we can do
					// TODO emergency stop?
				}
				break;

			default:		// this covers off, fault, suspended, and the auto tuning states
				break;
			}

			// Calculate the PWM
			if (mode <= HeaterMode::suspended)
			{
				lastPwm = 0.0;
			}
			else if (mode < HeaterMode::tuning0)
			{
				// Performing normal temperature control
				if (model.UsePid())
				{
					// Using PID mode. Determine the PID parameters to use.
					const bool inLoadMode = (mode == HeaterMode::stable) || fabsf(error) < 3.0;		// use standard PID when maintaining temperature
					const PidParameters& params = model.GetPidParameters(inLoadMode);

					// If the P and D terms together demand that the heater is full on or full off, disregard the I term
					const float errorMinusDterm = error - (params.tD * derivative);
					const float pPlusD = params.kP * errorMinusDterm;
					const float expectedPwm = constrain<float>((temperature - NormalAmbientTemperature)/model.GetGain(), 0.0, model.GetMaxPwm());
					if (pPlusD + expectedPwm > model.GetMaxPwm())
					{
						lastPwm = model.GetMaxPwm();
						// If we are heating up, preset the I term to the expected PWM at this temperature, ready for the switch over to PID
						if (mode == HeaterMode::heating && error > 0.0 && derivative > 0.0)
						{
							iAccumulator = expectedPwm;
						}
					}
					else if (pPlusD + expectedPwm < 0.0)
					{
						lastPwm = 0.0;
					}
					else
					{
						const float errorToUse = error;
						iAccumulator = constrain<float>
										(iAccumulator + (errorToUse * params.kP * params.recipTi * HeatSampleIntervalMillis * MillisToSeconds),
											0.0, model.GetMaxPwm());
						lastPwm = constrain<float>(pPlusD + iAccumulator, 0.0, model.GetMaxPwm());
					}
#if HAS_VOLTAGE_MONITOR
					// Scale the PID based on the current voltage vs. the calibration voltage
					if (lastPwm < 1.0 && model.GetVoltage() >= 10.0)				// if heater is not fully on and we know the voltage we tuned the heater at
					{
						if (!reprap.GetHeat().IsBedOrChamberHeater(heater))
						{
							const float currentVoltage = platform.GetCurrentPowerVoltage();
							if (currentVoltage >= 10.0)				// if we have a sensible reading
							{
								lastPwm = min<float>(lastPwm * fsquare(model.GetVoltage()/currentVoltage), 1.0);	// adjust the PWM by the square of the voltage ratio
							}
						}
					}
#endif
				}
				else
				{
					// Using bang-bang mode
					lastPwm = (error > 0.0) ? model.GetMaxPwm() : 0.0;
				}

				// Check if the generated PWM signal needs to be inverted for inverse temperature control
				if (model.IsInverted())
				{
					lastPwm = model.GetMaxPwm() - lastPwm;
				}

				// Verify that everything is operating in the required temperature range
				for (HeaterProtection *prot = heaterProtection; prot != nullptr; prot = prot->Next())
				{
					if (!prot->Check())
					{
						lastPwm = 0.0;
						switch (prot->GetAction())
						{
						case HeaterProtectionAction::GenerateFault:
							mode = HeaterMode::fault;
							reprap.GetGCodes().HandleHeaterFault(heater);
							platform.MessageF(ErrorMessage, "Heating fault on heater %d\n", heater);
							break;

						case HeaterProtectionAction::TemporarySwitchOff:
							// Do nothing, the PWM value has already been set above
							break;

						case HeaterProtectionAction::PermanentSwitchOff:
							SwitchOff();
							break;
						}
					}
				}
			}
			else
			{
				DoTuningStep();
			}
		}

		// Set the heater power and update the average PWM
		SetHeater(lastPwm);
		averagePWM = averagePWM * (1.0 - HeatSampleIntervalMillis/(HeatPwmAverageTime * SecondsToMillis)) + lastPwm;
		previousTemperatureIndex = (previousTemperatureIndex + 1) % NumPreviousTemperatures;

		// For temperature sensors which do not require frequent sampling and averaging,
		// their temperature is read here and error/safety handling performed.  However,
		// unlike the Tick ISR, this code is not executed at interrupt level and consequently
		// runs the risk of having undesirable delays between calls.  To guard against this,
		// we record for each PID object when it was last sampled and have the Tick ISR
		// take action if there is a significant delay since the time of last sampling.
		lastSampleTime = millis();

//  	debugPrintf("Heater %d: e=%f, P=%f, I=%f, d=%f, r=%f\n", heater, error, pp.kP*error, temp_iState, temp_dState, result);
	}
}

void PID::SetActiveTemperature(float t)
{
	if (t > GetHighestTemperatureLimit())
	{
		platform.MessageF(ErrorMessage, "Temperature %.1f" DEGREE_SYMBOL "C too high for heater %d\n", (double)t, heater);
	}
	else if (t < GetLowestTemperatureLimit())
	{
		platform.MessageF(ErrorMessage, "Temperature %.1f" DEGREE_SYMBOL "C too low for heater %d\n", (double)t, heater);
	}
	else
	{
		activeTemperature = t;
		if (mode > HeaterMode::suspended && active)
		{
			SwitchOn();
		}
	}
}

void PID::SetStandbyTemperature(float t)
{
	if (t > GetHighestTemperatureLimit())
	{
		platform.MessageF(ErrorMessage, "Temperature %.1f" DEGREE_SYMBOL "C too high for heater %d\n", (double)t, heater);
	}
	else if (t < GetLowestTemperatureLimit())
	{
		platform.MessageF(ErrorMessage, "Temperature %.1f" DEGREE_SYMBOL "C too low for heater %d\n", (double)t, heater);
	}
	else
	{
		standbyTemperature = t;
		if (mode > HeaterMode::suspended && !active)
		{
			SwitchOn();
		}
	}
}

void PID::SetHeaterProtection(HeaterProtection *h)
{
	heaterProtection = h;
}

void PID::Activate()
{
	if (mode != HeaterMode::fault)
	{
		active = true;
		SwitchOn();
	}
}

void PID::Standby()
{
	if (mode != HeaterMode::fault)
	{
		active = false;
		SwitchOn();
	}
}

// Check heater protection elements and return true if everything is good
bool PID::CheckProtection() const
{
	for (HeaterProtection *prot = heaterProtection; prot != nullptr; prot = prot->Next())
	{
		if (!prot->Check())
		{
			// Something is not right
			return false;
		}
	}
	return true;
}

void PID::ResetFault()
{
	badTemperatureCount = 0;
	if (mode == HeaterMode::fault)
	{
		mode = HeaterMode::off;
		SwitchOff();
	}
}

float PID::GetAveragePWM() const
{
	return averagePWM * HeatSampleIntervalMillis/(HeatPwmAverageTime * SecondsToMillis);
}

// Get a conservative estimate of the expected heating rate at the current temperature and average PWM. The result may be negative.
float PID::GetExpectedHeatingRate() const
{
	// In the following we allow for the gain being only 75% of what we think it should be, to avoid false alarms
	const float maxTemperatureRise = 0.75 * model.GetGain() * GetAveragePWM();		// this is the highest temperature above ambient we expect the heater can reach at this PWM
	const float initialHeatingRate = maxTemperatureRise/model.GetTimeConstant();	// this is the expected heating rate at ambient temperature
	return (maxTemperatureRise >= 20.0)
			? (maxTemperatureRise + NormalAmbientTemperature - temperature) * initialHeatingRate/maxTemperatureRise
			: 0.0;
}

// Auto tune this PID
void PID::StartAutoTune(float targetTemp, float maxPwm, const StringRef& reply)
{
	// Starting an auto tune
	if (!model.IsEnabled())
	{
		reply.printf("Error: heater %d cannot be auto tuned while it is disabled", heater);
	}
	else if (lastPwm > 0.0 || GetAveragePWM() > 0.02)
	{
		reply.printf("Error: heater %d must be off and cold before auto tuning it", heater);
	}
	else
	{
		const TemperatureError err = ReadTemperature();
		if (err != TemperatureError::success)
		{
			reply.printf("Error: heater %d reported error '%s' at start of auto tuning", heater, TemperatureErrorString(err));
		}
		else
		{
			mode = HeaterMode::tuning0;
			tuningReadingsTaken = 0;
			tuned = false;					// assume failure

			// We don't normally allow dynamic memory allocation when running. However, auto tuning is rarely done and it
			// would be wasteful to allocate a permanent array just in case we are going to run it, so we make an exception here.
			tuningTempReadings = new float[MaxTuningTempReadings];
			tuningTempReadings[0] = temperature;
			tuningReadingInterval = HeatSampleIntervalMillis;
			tuningPwm = maxPwm;
			tuningTargetTemp = targetTemp;
			reply.printf("Auto tuning heater %d using target temperature %.1f" DEGREE_SYMBOL "C and PWM %.2f - do not leave printer unattended", heater, (double)targetTemp, (double)maxPwm);
		}
	}
}

void PID::GetAutoTuneStatus(const StringRef& reply)	// Get the auto tune status or last result
{
	if (mode >= HeaterMode::tuning0)
	{
		reply.printf("Heater %d is being tuned, phase %u of %u",
						heater,
						(unsigned int)mode - (unsigned int)HeaterMode::tuning0 + 1,
						(unsigned int)HeaterMode::lastTuningMode - (unsigned int)HeaterMode::tuning0 + 1);
	}
	else if (tuned)
	{
		reply.printf("Heater %d tuning succeeded, use M307 H%d to see result", heater, heater);
	}
	else
	{
		reply.printf("Heater %d tuning failed", heater);
	}
}

/* Notes on the auto tune algorithm
 *
 * Most 3D printer firmwares use the Åström-Hägglund relay tuning method (sometimes called Ziegler-Nichols + relay).
 * This gives results  of variable quality, but they seem to be generally satisfactory.
 *
 * We use Cohen-Coon tuning instead. This models the heating process as a first-order process (i.e. one that with constant heating
 * power approaches the equilibrium temperature exponentially) with dead time. This process is defined by three constants:
 *
 *  G is the gain of the system, i.e. the increase in ultimate temperature increase per unit of additional PWM
 *  td is the dead time, i.e. the time between increasing the heater PWM and the temperature following an exponential curve
 *  tc is the time constant of the exponential curve
 *
 * If the temperature is stable at T0 to begin with, the temperature at time t after increasing heater PWM by p is:
 *  T = T0 when t <= td
 *  T = T0 + G * p * (1 - exp((t - td)/tc)) when t >= td
 * In practice the transition from no change to the exponential curve is not instant, however this model is a reasonable approximation.
 *
 * Having a process model allows us to preset the I accumulator to a suitable value when switching between heater full on/off and using PID.
 * It will also make it easier to include feedforward terms in future.
 *
 * The auto tune procedure follows the following steps:
 * 1. Turn on any thermostatically-controlled fans that are triggered by the heater being tuned. This is done by code in the Platform module
 *    when it sees that a heater is being auto tuned.
 * 2. Accumulate temperature readings and wait for the starting temperature to stabilise. Abandon auto tuning if the starting temperature
 *    is not stable.
 * 3. Apply a known power to the heater and take temperature readings.
 * 4. Wait until the temperature vs time curve has flattened off, such that the temperature rise over the last 1/3 of the readings is less than the
 *    total temperature rise - which means we have been heating for about 3 time constants. Abandon auto tuning if we don't see a temperature rise
 *    after 30 seconds, or we exceed the target temperature plus 10C.
 * 5. Calculate the G, td and tc values that best fit the model to the temperature readings.
 * 6. Calculate the P, I and D parameters from G, td and tc using the modified Cohen-Coon tuning rules, or the Ho et al tuning rules.
 *    Cohen-Coon (modified to use half the original Kc value):
 *     Kc = (0.67/G) * (tc/td + 0.185)
 *     Ti = 2.5 * td * (tc + 0.185 * td)/(tc + 0.611 * td)
 *     Td = 0.37 * td * tc/(tc + 0.185 * td)
 *    Ho et al, best response to load changes:
 *     Kc = (1.435/G) * (td/tc)^-0.921
 *     Ti = 1.14 * (td/tc)^0.749
 *     Td = 0.482 * tc * (td/tc)^1.137
 *    Ho et al, best response to setpoint changes:
 *     Kc = (1.086/G) * (td/tc)^-0.869
 *     Ti = tc/(0.74 - 0.13 * td/tc)
 *     Td = 0.348 * tc * (td/tc)^0.914
 */

// This is called on each temperature sample when auto tuning
// It must set lastPWM to the required PWM, unless it is the same as last time.
void PID::DoTuningStep()
{
	// See if another sample is due
	if (tuningReadingsTaken == 0)
	{
		tuningPhaseStartTime = millis();
		if (mode == HeaterMode::tuning0)
		{
			tuningBeginTime = tuningPhaseStartTime;
		}
	}
	else if (millis() - tuningPhaseStartTime < tuningReadingsTaken * tuningReadingInterval)
	{
		return;		// not due yet
	}

	// See if we have room to store the new reading, and if not, double the sample interval
	if (tuningReadingsTaken == MaxTuningTempReadings)
	{
		// Double the sample interval
		tuningReadingsTaken /= 2;
		for (size_t i = 1; i < tuningReadingsTaken; ++i)
		{
			tuningTempReadings[i] = tuningTempReadings[i * 2];
		}
		tuningReadingInterval *= 2;
	}

	tuningTempReadings[tuningReadingsTaken] = temperature;
	++tuningReadingsTaken;

	switch(mode)
	{
	case HeaterMode::tuning0:
		// Waiting for initial temperature to settle after any thermostatic fans have turned on
		if (ReadingsStable(6000/HeatSampleIntervalMillis, 2.0))			// expect temperature to be stable within a 2C band for 6 seconds
		{
			// Starting temperature is stable, so move on
			tuningReadingsTaken = 1;
#if HAS_VOLTAGE_MONITOR
			tuningVoltageAccumulator = 0.0;
			voltageSamplesTaken = 0;
#endif
			tuningTempReadings[0] = tuningStartTemp = temperature;
			timeSetHeating = tuningPhaseStartTime = millis();
			lastPwm = tuningPwm;										// turn on heater at specified power
			tuningReadingInterval = HeatSampleIntervalMillis;			// reset sampling interval
			mode = HeaterMode::tuning1;
			platform.Message(GenericMessage, "Auto tune phase 1, heater on\n");
			return;
		}
		if (millis() - tuningPhaseStartTime < 20000)
		{
			// Allow up to 20 seconds for starting temperature to settle
			return;
		}
		platform.Message(GenericMessage, "Auto tune cancelled because starting temperature is not stable\n");
		break;

	case HeaterMode::tuning1:
		// Heating up
		{
			const bool isBedOrChamberHeater = reprap.GetHeat().IsBedOrChamberHeater(heater);
			const uint32_t heatingTime = millis() - tuningPhaseStartTime;
			const float extraTimeAllowed = (isBedOrChamberHeater) ? 60.0 : 30.0;
			if (heatingTime > (uint32_t)((model.GetDeadTime() + extraTimeAllowed) * SecondsToMillis) && (temperature - tuningStartTemp) < 3.0)
			{
				platform.Message(GenericMessage, "Auto tune cancelled because temperature is not increasing\n");
				break;
			}

			const uint32_t timeoutMinutes = (isBedOrChamberHeater) ? 30 : 7;
			if (heatingTime >= timeoutMinutes * 60 * (uint32_t)SecondsToMillis)
			{
				platform.Message(GenericMessage, "Auto tune cancelled because target temperature was not reached\n");
				break;
			}

#if HAS_VOLTAGE_MONITOR
			tuningVoltageAccumulator += platform.GetCurrentPowerVoltage();
			++voltageSamplesTaken;
#endif
			if (temperature >= tuningTargetTemp)							// if reached target
			{
				tuningHeatingTime = heatingTime;

				// Move on to next phase
				tuningReadingsTaken = 1;
				tuningHeaterOffTemp = tuningTempReadings[0] = temperature;
				tuningPhaseStartTime = millis();
				tuningReadingInterval = HeatSampleIntervalMillis;			// reset sampling interval
				mode = HeaterMode::tuning2;
				lastPwm = 0.0;
				SetHeater(0.0);
				platform.Message(GenericMessage, "Auto tune phase 2, heater off\n");
			}
		}
		return;

	case HeaterMode::tuning2:
		// Heater turned off, looking for peak temperature
		{
			const int peakIndex = GetPeakTempIndex();
			if (peakIndex < 0)
			{
				if (millis() - tuningPhaseStartTime < 60 * 1000)			// allow 1 minute for the bed temperature reach peak temperature
				{
					return;			// still waiting for peak temperature
				}
				platform.Message(GenericMessage, "Auto tune cancelled because temperature is not falling\n");
			}
			else if (peakIndex == 0)
			{
				if (reprap.Debug(moduleHeat))
				{
					DisplayBuffer("At no peak found");
				}
				platform.Message(GenericMessage, "Auto tune cancelled because temperature peak was not identified\n");
			}
			else
			{
				tuningPeakTemperature = tuningTempReadings[peakIndex];
				tuningPeakDelay = peakIndex * tuningReadingInterval;

				// Move on to next phase
				tuningReadingsTaken = 1;
				tuningTempReadings[0] = temperature;
				tuningPhaseStartTime = millis();
				tuningReadingInterval = HeatSampleIntervalMillis;			// reset sampling interval
				mode = HeaterMode::tuning3;
				platform.MessageF(GenericMessage, "Auto tune phase 3, peak temperature was %.1f\n", (double)tuningPeakTemperature);
				return;
			}
		}
		break;

	case HeaterMode::tuning3:
		{
			// Heater is past the peak temperature and cooling down. Wait until it is part way back to the starting temperature so we can measure the cooling rate.
			// In the case of a bed that shows a reservoir effect, the choice of how far we wait for it to cool down will effect the result.
			// If we wait for it to cool down by 50% then we get a short time constant and a low gain, which causes overshoot. So try a bit more.
			const float coolDownProportion = 0.6;
			if (temperature > (tuningTempReadings[0] * (1.0 - coolDownProportion)) + (tuningStartTemp * coolDownProportion))
			{
				return;
			}
			CalculateModel();
		}
		break;

	default:
		// Should not happen, but if it does then quit
		break;
	}

	// If we get here, we have finished
	SwitchOff();								// sets mode and lastPWM, also deletes tuningTempReadings
}

// Return true if the last 'numReadings' readings are stable
/*static*/ bool PID::ReadingsStable(size_t numReadings, float maxDiff)
{
	if (tuningTempReadings == nullptr || tuningReadingsTaken < numReadings)
	{
		return false;
	}

	float minReading = tuningTempReadings[tuningReadingsTaken - numReadings];
	float maxReading = minReading;
	for (size_t i = tuningReadingsTaken - numReadings + 1; i < tuningReadingsTaken; ++i)
	{
		const float t = tuningTempReadings[i];
		if (t < minReading) { minReading = t; }
		if (t > maxReading) { maxReading = t; }
	}

	return maxReading - minReading <= maxDiff;
}

// Calculate which reading gave us the peak temperature.
// Return -1 if peak not identified yet, 0 if we are never going to find a peak, else the index of the peak
// If the readings show a continuous decrease then we return 1, because zero dead time would lead to infinities
/*static*/ int PID::GetPeakTempIndex()
{
	// Check we have enough readings to look for the peak
	if (tuningReadingsTaken < 15)
	{
		return -1;							// too few readings
	}

	// Look for the peak
	int peakIndex = IdentifyPeak(1);
	if (peakIndex < 0)
	{
		peakIndex = IdentifyPeak(3);
		if (peakIndex < 0)
		{
			peakIndex = IdentifyPeak(5);
			if (peakIndex < 0)
			{
				peakIndex = IdentifyPeak(7);
				if (peakIndex < 0)
				{
					return 0;					// more than one peak
				}
			}
		}
	}

	// If we have found one peak and it's not too near the end of the readings, return it
	return ((size_t)peakIndex + 3 < tuningReadingsTaken) ? max<int>(peakIndex, 1) : -1;
}

// See if there is exactly one peak in the readings.
// Return -1 if more than one peak, else the index of the peak. The so-called peak may be right at the end, in which case it isn't really a peak.
// With a well-insulated bed heater the temperature may not start dropping appreciably within the 120 second time limit allowed.
/*static*/ int PID::IdentifyPeak(size_t numToAverage)
{
	int firstPeakIndex = -1, lastSameIndex = -1;
	float peakTempTimesN = -999.0;
	for (size_t i = 0; i + numToAverage <= tuningReadingsTaken; ++i)
	{
		float peak = 0.0;
		for (size_t j = 0; j < numToAverage; ++j)
		{
			peak += tuningTempReadings[i + j];
		}
		if (peak > peakTempTimesN)
		{
			if ((int)i == lastSameIndex + 1)
			{
				firstPeakIndex = lastSameIndex = (int)i;	// readings still going up or staying the same, so advance the first peak index
				peakTempTimesN = peak;
			}
			else
			{
				return -1;						// error, more than one peak
			}
		}
		else if (peak == peakTempTimesN)		// exact equality can occur because the floating point value is computed from an integral value
		{
			lastSameIndex = (int)i;
		}
	}
	return firstPeakIndex + (numToAverage - 1)/2;
}

// Calculate the heater model from the accumulated heater parameters
void PID::CalculateModel()
{
	if (reprap.Debug(moduleHeat))
	{
		DisplayBuffer("At completion");
	}
	const float tc = (float)((tuningReadingsTaken - 1) * tuningReadingInterval)
						/(1000.0 * logf((tuningTempReadings[0] - tuningStartTemp)/(tuningTempReadings[tuningReadingsTaken - 1] - tuningStartTemp)));
	const float heatingTime = (tuningHeatingTime - tuningPeakDelay) * 0.001;
	const float gain = (tuningHeaterOffTemp - tuningStartTemp)/(1.0 - expf(-heatingTime/tc));

	// There are two ways of calculating the dead time:
	// 1. Based on the delay to peak temperature after we turned the heater off. Adding 0.5sec and then taking 65% of the result is about right.
	// 2. Based on the peak temperature compared to the temperature at which we turned the heater off.
	// Try #2 because it is easier to identify the peak temperature than the delay to peak temperature. It can be slightly to aggressive, so add 30%.
	//const float td = (float)(tuningPeakDelay + 500) * 0.00065;		// take the dead time as 65% of the delay to peak rounded up to a half second
	const float td = tc * logf((gain + tuningStartTemp - tuningHeaterOffTemp)/(gain + tuningStartTemp - tuningPeakTemperature)) * 1.3;

	tuned = SetModel(gain, tc, td, tuningPwm,
#if HAS_VOLTAGE_MONITOR
						tuningVoltageAccumulator/voltageSamplesTaken,
#else
						0.0,
#endif
		true, false, model.GetPwmFrequency());
	if (tuned)
	{
		platform.MessageF(LoggedGenericMessage,
				"Auto tune heater %d completed in %" PRIu32 " sec\n"
				"Use M307 H%d to see the result, or M500 to save the result in config-override.g\n",
				heater, (millis() - tuningBeginTime)/(uint32_t)SecondsToMillis, heater);
	}
	else
	{
		platform.MessageF(WarningMessage, "Auto tune of heater %u failed due to bad curve fit (A=%.1f, C=%.1f, D=%.1f)\n", heater, (double)gain, (double)tc, (double)td);
	}
}

void PID::DisplayBuffer(const char *intro)
{
	OutputBuffer *buf;
	if (OutputBuffer::Allocate(buf))
	{
		buf->catf("%s: interval %.1f sec, readings", intro, (double)(tuningReadingInterval * MillisToSeconds));
		for (size_t i = 0; i < tuningReadingsTaken; ++i)
		{
			buf->catf(" %.1f", (double)tuningTempReadings[i]);
		}
		buf->cat('\n');
		platform.Message(UsbMessage, buf);
	}
}

// Suspend the heater, or resume it
void PID::Suspend(bool sus)
{
	if (sus)
	{
		if (mode == HeaterMode::stable || mode == HeaterMode::heating || mode == HeaterMode::cooling)
		{
			mode = HeaterMode::suspended;
			SetHeater(0.0);
			lastPwm = 0.0;
		}
	}
	else if (mode == HeaterMode::suspended)
	{
		SwitchOn();
	}
}

// End
