/*
 * Pid.cpp
 *
 *  Created on: 21 Jul 2016
 *      Author: David
 */

#include "LocalHeater.h"
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include "Heat.h"
#include "HeaterMonitor.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Platform/Event.h>
#include <Tools/Tool.h>

#if SUPPORT_REMOTE_COMMANDS

# include <CAN/CanInterface.h>

namespace ExpansionMode
{

// Variables used during heater tuning
static float tuningHighTemp;							// the target upper temperature
static float tuningLowTemp;								// the target lower temperature
static float tuningPeakTempDrop;						// must be well below TuningHysteresis

static uint32_t dHigh;
static uint32_t dLow;
static uint32_t tOn;
static uint32_t tOff;
static float heatingRate;
static float coolingRate;
static float tuningVoltage;								// the VIN voltage with the heater on

static uint16_t cyclesDone;
static bool tuningCycleComplete;

}

#endif

// Member functions and constructors

LocalHeater::LocalHeater(unsigned int heaterNum) noexcept : Heater(heaterNum), mode(HeaterMode::off)
{
	LocalHeater::ResetHeater();
	SetHeater(0.0);							// set up the pin even if the heater is not enabled (for PCCB)

	// Time the sensor was last sampled.  During startup, we use the current
	// time as the initial value so as to not trigger an immediate warning from the Tick ISR.
	lastSampleTime = millis();
}

LocalHeater::~LocalHeater() noexcept
{
	LocalHeater::SwitchOff();
	for (auto& port : ports)
	{
		port.Release();
	}
}

float LocalHeater::GetTemperature() const noexcept
{
	return temperature;
}

float LocalHeater::GetAccumulator() const noexcept
{
	return iAccumulator;
}

inline void LocalHeater::SetHeater(float power) const noexcept
{
	for (auto& port : ports)
	{
		port.WriteAnalog(power);
	}
}

void LocalHeater::ResetHeater() noexcept
{
	mode = HeaterMode::off;
	previousTemperaturesGood = 0;
	previousTemperatureIndex = 0;
	iAccumulator = extrusionBoost = 0.0;
	badTemperatureCount = 0;
	averagePWM = lastPwm = 0.0;
	heatingFaultCount = 0;
	temperature = BadErrorTemperature;
}

// Configure the heater port and the sensor number
GCodeResult LocalHeater::ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply)
{
	if constexpr (MaxPortsPerHeater == 1)
	{
		if (!ports[0].AssignPort(portName, reply, PinUsedBy::heater, PinAccess::pwm))
		{
			return GCodeResult::error;
		}
	}
	else
	{
		PinAccess access[MaxPortsPerHeater];
		IoPort* portAddrs[MaxPortsPerHeater];
		for (size_t i = 0; i < MaxPortsPerHeater; ++i)
		{
			access[i] = PinAccess::pwm;
			portAddrs[i] = &ports[i];
		}
		if (IoPort::AssignPorts(portName, reply, PinUsedBy::heater, MaxPortsPerHeater, portAddrs, access) == 0)
		{
			return GCodeResult::error;
		}
	}

	for (auto& port : ports)
	{
		port.SetFrequency(freq);
	}
	SetSensorNumber(sn);
	if (reprap.GetHeat().FindSensor(sn).IsNull())
	{
		reply.printf("Sensor number %u has not been defined", sn);
		return GCodeResult::warning;
	}
	return GCodeResult::ok;
}

GCodeResult LocalHeater::SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept
{
	for (auto& port : ports)
	{
		port.SetFrequency(freq);
	}
	return GCodeResult::ok;
}

GCodeResult LocalHeater::ReportDetails(const StringRef& reply) const noexcept
{
	reply.printf("Heater %u pins(s) ", GetHeaterNumber());
	ports[0].AppendPinName(reply);
	if constexpr (MaxPortsPerHeater > 1)
	{
		for (size_t i = 1; i < MaxPortsPerHeater && ports[i].IsValid(); ++i)
		{
			reply.cat('+');
			ports[i].AppendPinName(reply);
		}
	}

	ports[0].AppendFrequency(reply);

	if (GetSensorNumber() >= 0)
	{
		reply.catf(", sensor %d", GetSensorNumber());
	}
	else
	{
		reply.cat(", no sensor");
	}
	return GCodeResult::ok;
}

// Read and store the temperature of this heater and returns the error code.
TemperatureError LocalHeater::ReadTemperature() noexcept
{
	TemperatureError err;
	temperature = reprap.GetHeat().GetSensorTemperature(GetSensorNumber(), err);		// in the event of an error, err is set and BAD_ERROR_TEMPERATURE is returned
	return err;
}

// This must be called whenever the heater is turned on, and any time the heater is active and the target temperature is changed
GCodeResult LocalHeater::SwitchOn(const StringRef& reply) noexcept
{
	if (!GetModel().IsEnabled())
	{
		reply.printf("Heater %u not switched on due to bad model", GetHeaterNumber());
		return GCodeResult::error;
	}

	if (mode == HeaterMode::fault)
	{
		reply.printf("Heater %u not switched on due to temperature fault", GetHeaterNumber());
		return GCodeResult::error;
	}

	const float target = GetTargetTemperature();
	const HeaterMode newMode = (temperature + TemperatureCloseEnough < target) ? HeaterMode::heating
								: (temperature > target + TemperatureCloseEnough) ? HeaterMode::cooling
									: HeaterMode::stable;
	if (newMode != mode)
	{
		if (reprap.Debug(Module::moduleHeat) && mode == HeaterMode::off)
		{
			reprap.GetPlatform().MessageF(GenericMessage, "Heater %u switched on\n", GetHeaterNumber());
		}

		// The Heat task can preempt the GCodes task that calls this, so lock out the Heat task while we update multiple variables
		TaskCriticalSectionLocker lock;
		if (newMode == HeaterMode::heating)
		{
			lastTemperatureValue = temperature;
			lastTemperatureMillis = timeSetHeating = millis();
		}
		heatingFaultCount = 0;
		mode = newMode;
	}
	return GCodeResult::ok;
}

// Switch off the specified heater. If in tuning mode, delete the array used to store tuning temperature readings.
void LocalHeater::SwitchOff() noexcept
{
	lastPwm = 0.0;
	if (GetModel().IsEnabled())
	{
		SetHeater(0.0);
		if (mode > HeaterMode::off)
		{
			mode = HeaterMode::off;
			if (reprap.Debug(Module::moduleHeat))
			{
				reprap.GetPlatform().MessageF(GenericMessage, "Heater %u switched off\n", GetHeaterNumber());
			}
		}
	}
}

// This is called when the heater model has been updated. Returns true if successful.
GCodeResult LocalHeater::UpdateModel(const StringRef& reply) noexcept
{
	return GCodeResult::ok;
}

// This is the main heater control loop function
void LocalHeater::Spin() noexcept
{
	// Read the temperature even if the heater is suspended or the model is not enabled
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
				RaiseHeaterFault(HeaterFaultType::failedToReadSensor, "%s", TemperatureErrorString(err));
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
		if ((previousTemperaturesGood & (1u << (NumPreviousTemperatures - 1))) != 0)
		{
			const float tentativeDerivative = (SecondsToMillis/(float)HeatSampleIntervalMillis) * (temperature - previousTemperatures[previousTemperatureIndex])
							/ (float)(NumPreviousTemperatures);
			// Some sensors give occasional temperature spikes. We don't expect the temperature to increase by more than 10C/second.
			if (fabsf(tentativeDerivative) <= 10.0)
			{
				derivative = tentativeDerivative;
				gotDerivative = true;
			}
		}
		previousTemperatures[previousTemperatureIndex] = temperature;
		previousTemperaturesGood = (previousTemperaturesGood << 1) | 1u;
		previousTemperatureIndex = (previousTemperatureIndex + 1) % NumPreviousTemperatures;

		if (GetModel().IsEnabled())
		{
			// Get the target temperature and the error
			const float targetTemperature = GetTargetTemperature();
			const float error = targetTemperature - temperature;

			// Do the heating checks
			switch(mode)
			{
			case HeaterMode::heating:
				{
					if (error <= TemperatureCloseEnough)
					{
						mode = HeaterMode::stable;
						heatingFaultCount = 0;
					}
					else
					{
						const uint32_t now = millis();
						if ((float)(now - timeSetHeating) < GetModel().GetDeadTime() * SecondsToMillis * 2)		// wait for twice the dead time before we start looking at the temperature rise
						{
							// Record the temperature for when we are past the dead time
							lastTemperatureValue = temperature;
							lastTemperatureMillis = now;
						}
						else if (gotDerivative)												// this is a check in case we just had a temperature spike
						{
							const float expectedRate = GetExpectedHeatingRate();
							const float minSamplingInterval = 3.0/expectedRate;				// only check the temperature when we expect at least 3C rise since last time
							const float actualInterval = (float)(now - lastTemperatureMillis) * MillisToSeconds;
							if (actualInterval >= minSamplingInterval)
							{
								// Check that we are heating fast enough, and if so, take another sample
								const float expectedTemperatureRise = expectedRate * actualInterval;
								const float actualTemperatureRise = temperature - lastTemperatureValue;
								// Bed heaters sometimes have much slower long term heating rates than their short term heating rates, so allow them a lower measured heating rate
								if (actualTemperatureRise < expectedTemperatureRise * ((IsBedOrChamber()) ? MinBedTemperatureRiseFactor : MinToolTemperatureRiseFactor))
								{
									++heatingFaultCount;
									if (heatingFaultCount * HeatSampleIntervalMillis > GetMaxHeatingFaultTime() * SecondsToMillis)
									{
										RaiseHeaterFault(HeaterFaultType::temperatureRisingTooSlowly,
															"expected %.2f" DEGREE_SYMBOL "C/sec measured %.2f" DEGREE_SYMBOL "C/sec",
																(double)expectedRate, (double)(actualTemperatureRise/actualInterval));
									}
								}
								else
								{
									lastTemperatureValue = temperature;
									lastTemperatureMillis = now;
									if (heatingFaultCount != 0)
									{
										--heatingFaultCount;
									}
								}
							}
						}
					}
				}
				break;

			case HeaterMode::stable:
				if (fabsf(error) > GetMaxTemperatureExcursion() && temperature > MaxAmbientTemperature)
				{
					++heatingFaultCount;
					if (heatingFaultCount * HeatSampleIntervalMillis > GetMaxHeatingFaultTime() * SecondsToMillis)
					{
						RaiseHeaterFault(HeaterFaultType::exceededAllowedExcursion,
											"target %.1f" DEGREE_SYMBOL "C actual %.1f" DEGREE_SYMBOL "C",
												(double)targetTemperature, (double)temperature);
					}
				}
				else if (heatingFaultCount != 0)
				{
					--heatingFaultCount;
				}
				break;

			case HeaterMode::cooling:
				if (-error <= TemperatureCloseEnough && targetTemperature > MaxAmbientTemperature)
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
			if (mode >= HeaterMode::tuning0)
			{
				DoTuningStep();
			}
			else if (mode <= HeaterMode::suspended)
			{
				lastPwm = 0.0;
			}
			else
			{
				// Performing normal temperature control
				if (GetModel().UsePid())
				{
					// Using PID mode. Determine the PID parameters to use.
					const bool inLoadMode = (mode == HeaterMode::stable) || fabsf(error) < 3.0;		// use standard PID when maintaining temperature
					const PidParameters& params = GetModel().GetPidParameters(inLoadMode);

					// If the P and D terms together demand that the heater is full on or full off, disregard the I term
					const float errorMinusDterm = error - (params.tD * derivative);
					const float pPlusD = params.kP * errorMinusDterm;
					const float expectedPwm = GetModel().EstimateRequiredPwm(temperature - NormalAmbientTemperature, 0.0);
					if (pPlusD + expectedPwm > GetModel().GetMaxPwm())
					{
						lastPwm = GetModel().GetMaxPwm();
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
										(iAccumulator + (errorToUse * params.kP * params.recipTi * (HeatSampleIntervalMillis * MillisToSeconds)),
											0.0, GetModel().GetMaxPwm());
						lastPwm = constrain<float>(pPlusD + iAccumulator + extrusionBoost, 0.0, GetModel().GetMaxPwm());
					}
#if HAS_VOLTAGE_MONITOR
					// Scale the PID based on the current voltage vs. the calibration voltage
					if (!reprap.GetHeat().IsBedOrChamberHeater(GetHeaterNumber()))
					{
						lastPwm = GetModel().CorrectPwmForVoltage(lastPwm, reprap.GetPlatform().GetCurrentPowerVoltage());
					}
#endif
				}
				else
				{
					// Using bang-bang mode
					lastPwm = (error > 0.0) ? GetModel().GetMaxPwm() : 0.0;
				}

				// Check if the generated PWM signal needs to be inverted for inverse temperature control
				if (GetModel().IsInverted())
				{
					lastPwm = GetModel().GetMaxPwm() - lastPwm;
				}
			}

			// Verify that everything is operating in the required temperature range
			for (size_t i = 0; i < ARRAY_SIZE(monitors); ++i)
			{
				HeaterMonitor& prot = monitors[i];
				if (!prot.Check())
				{
					if (reprap.Debug(moduleHeat))
					{
						reprap.GetPlatform().MessageF(GenericMessage, "Heater %u protection kicked in\n", GetHeaterNumber());
					}
					lastPwm = 0.0;
					switch (prot.GetAction())
					{
					case HeaterMonitorAction::ShutDown:
						reprap.GetHeat().SwitchOffAll(true);
						reprap.GetPlatform().AtxPowerOff();
						break;

					case HeaterMonitorAction::GenerateFault:
						RaiseHeaterFault(HeaterFaultType::monitorTriggered, "monitor %u was triggered", i);
						break;

					case HeaterMonitorAction::TemporarySwitchOff:
						// Do nothing, the PWM value has already been set above
						break;

					case HeaterMonitorAction::PermanentSwitchOff:
						SwitchOff();
						break;
					}
				}
			}
		}
		else
		{
			lastPwm = 0.0;
		}

		// Set the heater power and update the average PWM
		SetHeater(lastPwm);
		constexpr float avgFactor = HeatSampleIntervalMillis/(HeatPwmAverageTime * SecondsToMillis);
		averagePWM = (averagePWM * (1.0 - avgFactor)) + (lastPwm * avgFactor);

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

GCodeResult LocalHeater::ResetFault(const StringRef& reply) noexcept
{
	badTemperatureCount = 0;
	if (mode == HeaterMode::fault)
	{
		mode = HeaterMode::off;
		SwitchOff();
	}
	return GCodeResult::ok;
}

float LocalHeater::GetAveragePWM() const noexcept
{
	return averagePWM;
}

// Get a conservative estimate of the expected heating rate at the current temperature and average PWM. The result may be negative.
float LocalHeater::GetExpectedHeatingRate() const noexcept
{
	const float temperatureRise = max<float>(temperature - LowAmbientTemperature, 0.0);
	const float pwm = min<float>(GetAveragePWM(), lastPwm);
	return GetModel().GetNetHeatingRate(temperatureRise, 1.0, pwm);
}

// Auto tune this heater. The caller has already checked that no other heater is being tuned and has set up tuningTargetTemp, tuningPwm, tuningFans, tuningHysteresis and tuningFanPwm.
GCodeResult LocalHeater::StartAutoTune(const StringRef& reply, bool seenA, float ambientTemp) noexcept
{
	if (lastPwm > 0.0 || GetAveragePWM() > 0.02)
	{
		reply.printf("heater %u must be off and cold before auto tuning it", GetHeaterNumber());
		return GCodeResult::error;
	}

	reprap.GetFansManager().SetFansValue(tuningFans, 0.0);

	tuningStartTemp.Clear();
	tuningBeginTime = millis();
	tuned = false;					// assume failure

	if (seenA)
	{
		tuningStartTemp.Add(ambientTemp);
		ClearCounters();
		timeSetHeating = millis();
		lastPwm = tuningPwm;										// turn on heater at specified power
		tuningPhase = 1;
		mode = HeaterMode::tuning1;
		ReportTuningUpdate();
	}
	else
	{
		tuningPhase = 0;
		mode = HeaterMode::tuning0;
	}

	return GCodeResult::ok;
}

// Call this when the PWM of a cooling fan has changed. If there are multiple fans, caller must divide pwmChange by the number of fans.
void LocalHeater::FeedForwardAdjustment(float fanPwmChange, float extrusionChange) noexcept
{
	if (mode == HeaterMode::stable)
	{
		const float boost = GetModel().GetPwmCorrectionForFan(GetTargetTemperature() - NormalAmbientTemperature, fanPwmChange) * FanFeedForwardMultiplier;
#if 0
		if (reprap.Debug(moduleHeat))
		{
			debugPrintf("iacc=%.3f, applying boost %.3f\n", (double)iAccumulator, (double)boost);
		}
#endif
		InterruptCriticalSectionLocker lock;
		iAccumulator += boost;
	}
}

// Set extrusion feedforward. This is called from an ISR.
void LocalHeater::SetExtrusionFeedForward(float pwm) noexcept
{
	extrusionBoost = pwm;
}

/* Notes on the auto tune algorithm
 *
 * Most 3D printer firmwares use the �str�m-H�gglund relay tuning method (sometimes called Ziegler-Nichols + relay).
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
 * We can calculate the P, I and D parameters from G, td and tc using the modified Cohen-Coon tuning rules, or the Ho et al tuning rules.
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
// It must set lastPWM to the required PWM before returning, unless it is the same as last time.
void LocalHeater::DoTuningStep() noexcept
{
	const uint32_t now = millis();
	switch (mode)
	{
	case HeaterMode::tuning0:		// Waiting for initial temperature to settle after any thermostatic fans have turned on
		if (tuningStartTemp.GetNumSamples() < 5000/HeatSampleIntervalMillis)
		{
			tuningStartTemp.Add(temperature);							// take another reading until we have samples temperatures for 5 seconds
			return;
		}

		if (tuningStartTemp.GetDeviation() <= 2.0)
		{
			timeSetHeating = now;
			lastPwm = tuningPwm;										// turn on heater at specified power
			mode = HeaterMode::tuning1;

			tuningPhase = 1;
			ReportTuningUpdate();
			return;
		}

		if (now - tuningBeginTime < 20000)
		{
			// Allow up to 20 seconds for starting temperature to settle
			return;
		}

		reprap.GetPlatform().Message(GenericMessage, "Auto tune cancelled because starting temperature is not stable\n");
		break;

	case HeaterMode::tuning1:		// Heating up
#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			if (temperature >= ExpansionMode::tuningHighTemp)							// if reached target
			{
				// Move on to next phase
				lastPwm = 0.0;
				SetHeater(0.0);
				peakTemp = afterPeakTemp = temperature;
				lastOffTime = peakTime = afterPeakTime = now;
				mode = HeaterMode::tuning2;
			}
			else
			{
				lastPwm = tuningPwm;
			}
			return;
		}
#endif
		{
			const bool isBedOrChamberHeater = reprap.GetHeat().IsBedOrChamberHeater(GetHeaterNumber());
			const uint32_t heatingTime = now - timeSetHeating;
			const float extraTimeAllowed = (isBedOrChamberHeater) ? 120.0 : 30.0;
			if (heatingTime > (uint32_t)((GetModel().GetDeadTime() + extraTimeAllowed) * SecondsToMillis) && (temperature - tuningStartTemp.GetMean()) < 3.0)
			{
				reprap.GetPlatform().Message(GenericMessage, "Auto tune cancelled because temperature is not increasing\n");
				break;
			}

			const uint32_t timeoutMinutes = (isBedOrChamberHeater) ? 30 : 7;
			if (heatingTime >= timeoutMinutes * 60 * (uint32_t)SecondsToMillis)
			{
				reprap.GetPlatform().Message(GenericMessage, "Auto tune cancelled because target temperature was not reached\n");
				break;
			}
		}

		if (temperature >= tuningTargetTemp)							// if reached target
		{
			// Move on to next phase
			lastPwm = 0.0;
			SetHeater(0.0);
			peakTemp = afterPeakTemp = temperature;
			lastOffTime = peakTime = afterPeakTime = now;
			tuningVoltage.Clear();
			idleCyclesDone = 0;
			mode = HeaterMode::tuning2;
			tuningPhase = 2;
			ReportTuningUpdate();
		}
		return;

	case HeaterMode::tuning2:		// Heater is off, record the peak temperature and time
#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			if (temperature >= peakTemp)
			{
				peakTemp = afterPeakTemp = temperature;
				peakTime = afterPeakTime = now;
			}
			else if (temperature < ExpansionMode::tuningLowTemp)
			{
				// Temperature has dropped below the low limit.
				// If we have been doing idle cycles, see whether we can switch to collecting data, and turn the heater on.
				// If we have been collecting data, see if we have enough, and either turn the heater on to start another cycle or finish tuning.

				// Save the data (don't know whether we need it yet)
				ExpansionMode::dHigh = peakTime - lastOffTime;
				ExpansionMode::tOff = now - lastOffTime;
				ExpansionMode::coolingRate = (afterPeakTemp - temperature) * SecondsToMillis/(now - afterPeakTime);
				lastOnTime = peakTime = afterPeakTime = now;
				peakTemp = afterPeakTemp = temperature;
				lastPwm = tuningPwm;						// turn on heater at specified power
				mode = HeaterMode::tuning3;
			}
			else if (afterPeakTime == peakTime && ExpansionMode::tuningHighTemp - temperature >= ExpansionMode::tuningPeakTempDrop)
			{
				afterPeakTime = now;
				afterPeakTemp = temperature;
			}
			return;
		}
#endif
		if (temperature >= peakTemp)
		{
			peakTemp = afterPeakTemp = temperature;
			peakTime = afterPeakTime = now;
		}
		else if (temperature < tuningTargetTemp - tuningHysteresis)
		{
			// Temperature has dropped below the low limit.
			// If we have been doing idle cycles, see whether we can switch to collecting data, and turn the heater on.
			// If we have been collecting data, see if we have enough, and either turn the heater on to start another cycle or finish tuning.

			// Save the data (don't know whether we need it yet)
			dHigh.Add((float)(peakTime - lastOffTime));
			tOff.Add((float)(now - lastOffTime));
			const float currentCoolingRate = (afterPeakTemp - temperature) * SecondsToMillis/(now - afterPeakTime);
			coolingRate.Add(currentCoolingRate);

			// Decide whether to finish this phase
			if (tuningPhase == 2)				// if we are doing idle cycles
			{
				// To allow for heat reservoirs, we do idle cycles until the cooling rate decreases by no more than a certain amount in a single cycle
				if (idleCyclesDone == TuningHeaterMaxIdleCycles || (idleCyclesDone >= TuningHeaterMinIdleCycles && currentCoolingRate >= lastCoolingRate * HeaterSettledCoolingTimeRatio))
				{
					tuningPhase = 3;
					ReportTuningUpdate();
				}
				else
				{
					lastCoolingRate = currentCoolingRate;
					ClearCounters();
					++idleCyclesDone;
				}
			}
			else if (coolingRate.GetNumSamples() >= MinTuningHeaterCycles)
			{
				const bool isConsistent = dLow.DeviationFractionWithin(0.2)
										&& dHigh.DeviationFractionWithin(0.2)
										&& heatingRate.DeviationFractionWithin(0.1)
										&& coolingRate.DeviationFractionWithin(0.1);
				if (isConsistent || coolingRate.GetNumSamples() == MaxTuningHeaterCycles)
				{
					if (!isConsistent)
					{
						reprap.GetPlatform().Message(WarningMessage, "heater behaviour was not consistent during tuning\n");
					}

					if (tuningPhase == 3)
					{
						CalculateModel(fanOffParams);
						if (tuningFans.IsEmpty())
						{
							SetAndReportModelAfterTuning(false);
							break;
						}
						else
						{
							tuningPhase = 4;
							ClearCounters();
#if TUNE_WITH_HALF_FAN
							reprap.GetFansManager().SetFansValue(tuningFans, tuningFanPwm * 0.5);	// turn fans on at half PWM
#else
							reprap.GetFansManager().SetFansValue(tuningFans, tuningFanPwm);		// turn fans on at full PWM
#endif
							ReportTuningUpdate();
						}
					}
#if TUNE_WITH_HALF_FAN
					else if (tuningPhase == 4)
					{
						CalculateModel(fanOnParams);
						tuningPhase = 5;
						ClearCounters();
						reprap.GetFansManager().SetFansValue(tuningFans, tuningFanPwm);			// turn fans fully on
						ReportTuningUpdate();
					}
#endif
					else
					{
						reprap.GetFansManager().SetFansValue(tuningFans, 0.0);					// turn fans off
						CalculateModel(fanOnParams);
						SetAndReportModelAfterTuning(true);
						break;
					}
				}
			}
			lastOnTime = peakTime = afterPeakTime = now;
			peakTemp = afterPeakTemp = temperature;
			lastPwm = tuningPwm;						// turn on heater at specified power
			mode = HeaterMode::tuning3;
		}
		else if (afterPeakTime == peakTime && tuningTargetTemp - temperature >= TuningPeakTempDrop)
		{
			afterPeakTime = now;
			afterPeakTemp = temperature;
		}
		return;

	case HeaterMode::tuning3:	// Heater is turned on, record the lowest temperature and time
#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			if (temperature <= peakTemp)
			{
				peakTemp = afterPeakTemp = temperature;
				peakTime = afterPeakTime = now;
			}
			else if (temperature >= ExpansionMode::tuningHighTemp)
			{
				// We have reached the target temperature, so record a data point and turn the heater off
# if HAS_VOLTAGE_MONITOR
				ExpansionMode::tuningVoltage = reprap.GetPlatform().GetCurrentPowerVoltage();	// save this while the heater is on
# else
				tuningVoltage = 0.0;
# endif
				ExpansionMode::dLow = peakTime - lastOnTime;
				ExpansionMode::tOn = now - lastOnTime;
				ExpansionMode::heatingRate = (temperature - afterPeakTemp) * SecondsToMillis/(now - afterPeakTime);
				lastOffTime = peakTime = afterPeakTime = now;
				peakTemp = afterPeakTemp = temperature;
				lastPwm = 0.0;										// turn heater off
				mode = HeaterMode::tuning2;
				++ExpansionMode::cyclesDone;
				ExpansionMode::tuningCycleComplete = true;
			}
			else if (afterPeakTime == peakTime && temperature - ExpansionMode::tuningLowTemp >= ExpansionMode::tuningPeakTempDrop)
			{
				afterPeakTime = now;
				afterPeakTemp = temperature;
			}
			return;
		}
#endif
#if HAS_VOLTAGE_MONITOR
		tuningVoltage.Add(reprap.GetPlatform().GetCurrentPowerVoltage());
#endif
		if (temperature <= peakTemp)
		{
			peakTemp = afterPeakTemp = temperature;
			peakTime = afterPeakTime = now;
		}
		else if (temperature >= tuningTargetTemp)
		{
			// We have reached the target temperature, so record a data point and turn the heater off
			dLow.Add((float)(peakTime - lastOnTime));
			tOn.Add((float)(now - lastOnTime));
			heatingRate.Add((temperature - afterPeakTemp) * SecondsToMillis/(now - afterPeakTime));
			lastOffTime = peakTime = afterPeakTime = now;
			peakTemp = afterPeakTemp = temperature;
			lastPwm = 0.0;								// turn heater off
			mode = HeaterMode::tuning2;
		}
		else if (afterPeakTime == peakTime && temperature - tuningTargetTemp >= TuningPeakTempDrop - tuningHysteresis)
		{
			afterPeakTime = now;
			afterPeakTemp = temperature;
		}
		return;

	default:
		// Should not happen, but if it does then quit
		break;
	}

	// If we get here, we have finished
	SwitchOff();										// sets mode and lastPWM, also deletes tuningTempReadings
}

// Calculate the heater model from the accumulated heater parameters
// Suspend the heater, or resume it
void LocalHeater::Suspend(bool sus) noexcept
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
		String<1> dummy;
		(void)SwitchOn(dummy.GetRef());
	}
}

void LocalHeater::RaiseHeaterFault(HeaterFaultType type, const char *_ecv_array format, ...) noexcept
{
	lastPwm = 0.0;
	SetHeater(0.0);
	if (mode != HeaterMode::fault)
	{
		mode = HeaterMode::fault;
		reprap.FlagTemperatureFault(GetHeaterNumber());

		va_list vargs;
		va_start(vargs, format);

#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			CanInterface::RaiseEvent(EventType::heater_fault, (uint16_t)type, GetHeaterNumber(), format, vargs);
		}
		else
#endif
		{
			Event::AddEventV(EventType::heater_fault, (uint16_t)type, CanInterface::GetCanAddress(), GetHeaterNumber(), format, vargs);
		}
		va_end(vargs);
	}
}

#if SUPPORT_REMOTE_COMMANDS

// Start or stop running heater tuning cycles
GCodeResult LocalHeater::TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept
{
	if (msg.on)
	{
		if (lastPwm > 0.0 || GetAveragePWM() > 0.02)
		{
			reply.printf("heater %u must be off and cold before auto tuning it", GetHeaterNumber());
			return GCodeResult::error;
		}

		// We could do some more checks here but the main board should have done all the checks needed already
		ExpansionMode::tuningHighTemp = msg.highTemp;
		ExpansionMode::tuningLowTemp = msg.lowTemp;
		tuningPwm = msg.pwm;
		ExpansionMode::tuningPeakTempDrop = msg.peakTempDrop;
		timeSetHeating = millis();
		ExpansionMode::tuningCycleComplete = false;
		ExpansionMode::cyclesDone = 0;
		mode = HeaterMode::tuning1;
	}
	else
	{
		SwitchOff();
	}
	return GCodeResult::ok;
}

// Get a heater tuning cycle report, if we have one. Caller must fill in the heater number.
/*static*/ bool LocalHeater::GetTuningCycleData(CanMessageHeaterTuningReport& msg) noexcept
{
	if (ExpansionMode::tuningCycleComplete)
	{
		msg.cyclesDone = ExpansionMode::cyclesDone;
		msg.dhigh = ExpansionMode::dHigh;
		msg.dlow = ExpansionMode::dLow;
		msg.ton = ExpansionMode::tOn;
		msg.toff = ExpansionMode::tOff;
		msg.heatingRate = ExpansionMode::heatingRate;
		msg.coolingRate = ExpansionMode::coolingRate;
		msg.voltage = ExpansionMode::tuningVoltage;
		ExpansionMode::tuningCycleComplete = false;
		return true;
	}

	return false;
}

#endif

// End
