/*
 * Pid.cpp
 *
 *  Created on: 21 Jul 2016
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "Pid.h"

//******************************************************************************************************

PID::PID(Platform* p, int8_t h) : platform(p), heater(h), mode(HeaterMode::off)
{
}

void PID::Init()
{
	SetHeater(0.0);
	mode = HeaterMode::off;
	previousTemperaturesGood = 0;
	previousTemperatureIndex = 0;
	activeTemperature = ABS_ZERO;
	standbyTemperature = ABS_ZERO;
	temp_iState = 0.0;
	badTemperatureCount = 0;
	active = false; 		// Default to standby temperature
	averagePWM = lastPWM = 0.0;

	// Time the sensor was last sampled.  During startup, we use the current
	// time as the initial value so as to not trigger an immediate warning from the Tick ISR.
	lastSampleTime = millis();
}

// Read and store the temperature of this heater and returns the error code.
TemperatureError PID::ReadTemperature()
{
	TemperatureError err = TemperatureError::success;				// assume no error
	temperature = platform->GetTemperature(heater, err);			// in the event of an error, err is set and BAD_ERROR_TEMPERATURE is returned
	if (err == TemperatureError::success)
	{
		if (temperature < BAD_LOW_TEMPERATURE)
		{
			err = TemperatureError::openCircuit;
		}
		else if (temperature > platform->GetTemperatureLimit())
		{
			err = TemperatureError::tooHigh;
		}
	}
	return err;
}

// This must be called whenever the heater is turned on, and any time the heater is active and the target temperature is changed
void PID::SwitchOn()
{
	if (mode == HeaterMode::fault)
	{
		if (reprap.Debug(Module::moduleHeat))
		{
			platform->MessageF(GENERIC_MESSAGE, "Heater %d not switched on due to temperature fault\n", heater);
		}
	}
	else
	{
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
				timeSetHeating = platform->Time();
			}
			if (reprap.Debug(Module::moduleHeat) && oldMode == HeaterMode::off)
			{
				platform->MessageF(GENERIC_MESSAGE, "Heater %d switched on\n", heater);
			}
		}
	}
}

void PID::Spin()
{
	// For temperature sensors which do not require frequent sampling and averaging,
	// their temperature is read here and error/safety handling performed.  However,
	// unlike the Tick ISR, this code is not executed at interrupt level and consequently
	// runs the risk of having undesirable delays between calls.  To guard against this,
	// we record for each PID object when it was last sampled and have the Tick ISR
	// take action if there is a significant delay since the time of last sampling.
	lastSampleTime = millis();

	// Set up the default PWM value
	float heaterPwm = 0.0;

	// Read the temperature
	TemperatureError err = ReadTemperature();
	const PidParameters& pp = platform->GetPidParameters(heater);

	// Handle any temperature reading error and calculate the temperature rate of change, if possible
	if (err != TemperatureError::success)
	{
		previousTemperaturesGood <<= 1;				// this reading isn't a good one
		if (mode > HeaterMode::off)					// don't worry about errors when reading heaters that are switched off or flagged as having faults
		{
			// Error may be a temporary error and may correct itself after a few additional reads
			badTemperatureCount++;
			if (badTemperatureCount > MAX_BAD_TEMPERATURE_COUNT)
			{
				SetHeater(0.0);						// do this here just to be sure, in case the call to platform->Message causes a delay
				mode = HeaterMode::fault;
				platform->MessageF(GENERIC_MESSAGE, "Error: Temperature reading fault on heater %d: %s\n", heater, TemperatureErrorString(err));
				reprap.FlagTemperatureFault(heater);
			}
		}

		// We have already initialised heaterPwm to 0.0 so no need to do that here
	}
	else
	{
		// We have a good temperature reading. Calculate the derivative, if possible.
		float derivative = 0.0;
		bool gotDerivative = false;
		badTemperatureCount = 0;
		if ((previousTemperaturesGood & (1 << (NumPreviousTemperatures - 1))) != 0)
		{
			derivative = (temperature - previousTemperatures[previousTemperatureIndex]) / (platform->HeatSampleTime() * NumPreviousTemperatures);
			gotDerivative = true;
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
					float startupTime;
					if (derivative < GetExpectedHeatingRate(temperature, lastPWM, startupTime) && platform->Time() - timeSetHeating > startupTime)
					{
						++heatingFaultCount;
						if (heatingFaultCount * platform->HeatSampleTime() > MaxHeatingFaultTime)
						{
							SetHeater(0.0);			// do this here just to be sure
							mode = HeaterMode::fault;
							platform->MessageF(GENERIC_MESSAGE, "Error: heating fault on heater %d, temperature rising too slowly\n", heater);
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
			if (fabs(error) > MaxStableTemperatureError)
			{
				mode = HeaterMode::fault;
				platform->MessageF(GENERIC_MESSAGE, "Error: heating fault on heater %d, temperature excursion too large\n", heater);
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
				// We could check for temperature excessive or not falling here, but without an alarm there is not much we can do
			}
			break;

		case HeaterMode::off:
		case HeaterMode::fault:
			break;
		}

		// If we are still switched on, calculate the PWM
		if (mode > HeaterMode::off)
		{
			if (!pp.UsePID())
			{
				heaterPwm = (error > 0.0) ? 1.0 : 0.0;
			}
			else if (error < -pp.fullBand)
			{
				// actual temperature is well above target
				temp_iState = (targetTemperature + pp.fullBand - 25.0) * pp.kT;	// set the I term to our estimate of what will be needed ready for the switch to PID
				heaterPwm = 0.0;
			}
			else if (error > pp.fullBand)
			{
				// actual temperature is well below target
				temp_iState = (targetTemperature - pp.fullBand - 25.0) * pp.kT;	// set the I term to our estimate of what will be needed ready for the switch to PID
				heaterPwm = 1.0;
			}
			else
			{
				temp_iState = constrain<float>(temp_iState + (error * pp.kI * platform->HeatSampleTime()), pp.pidMin, pp.pidMax);
				heaterPwm = ((pp.kP * error) + temp_iState - (pp.kD * derivative)) * (1.0/255.0);
			}
		}
	}

	// Scale the heater power by kS, constrain it, set the heater power, and update the average PWM
	lastPWM = heaterPwm;
	heaterPwm = constrain<float>(heaterPwm * pp.kS, 0.0, 1.0);
	SetHeater(heaterPwm);
	averagePWM = averagePWM * (1.0 - platform->HeatSampleTime()/HEAT_PWM_AVERAGE_TIME) + heaterPwm;
	previousTemperatureIndex = (previousTemperatureIndex + 1) % NumPreviousTemperatures;

//  debugPrintf("Heater %d: e=%f, P=%f, I=%f, d=%f, r=%f\n", heater, error, pp.kP*error, temp_iState, temp_dState, result);
}

void PID::SetActiveTemperature(float t)
{
	if (t > platform->GetTemperatureLimit())
	{
		platform->MessageF(GENERIC_MESSAGE, "Error: Temperature %.1f too high for heater %d!\n", t, heater);
	}
	else
	{
		activeTemperature = t;
		if (mode > HeaterMode::off && active)
		{
			SwitchOn();
		}
	}
}

void PID::SetStandbyTemperature(float t)
{
	if (t > platform->GetTemperatureLimit())
	{
		platform->MessageF(GENERIC_MESSAGE, "Error: Temperature %.1f too high for heater %d!\n", t, heater);
	}
	else
	{
		standbyTemperature = t;
		if (mode > HeaterMode::off && !active)
		{
			SwitchOn();
		}
	}
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

void PID::ResetFault()
{
	mode = HeaterMode::off;
	SwitchOff();
	badTemperatureCount = 0;
}

void PID::SwitchOff()
{
	SetHeater(0.0);
	if (mode > HeaterMode::off)
	{
		mode = HeaterMode::off;
	}
	if (reprap.Debug(Module::moduleHeat))
	{
		platform->MessageF(GENERIC_MESSAGE, "Heater %d switched off", heater);
	}
}

float PID::GetAveragePWM() const
{
	return averagePWM * platform->HeatSampleTime()/HEAT_PWM_AVERAGE_TIME;
}

// Get the minimum heating rate we expect at the specified temperature and pwm, and the heater startup time
float PID::GetExpectedHeatingRate(float temp, float pwm, float& startupTime) const
{
	float initialHeatingRate;							// this will be the minimum heating rate @ 20C
	float maxTemperature;								// this will be the highest temperature that we are certain the heater can reach

	if (heater == reprap.GetHeat()->GetBedHeater())
	{
		initialHeatingRate = BedHeatingRate;
		maxTemperature = BedHeaterAchievableTemp;
		startupTime = BedHeaterStartupTime;
	}
	else if (heater == reprap.GetHeat()->GetChamberHeater())
	{
		initialHeatingRate = ChamberHeatingRate;
		maxTemperature = ChamberHeaterAchievableTemp;
		startupTime = ChamberHeaterStartupTime;
	}
	else
	{
		initialHeatingRate = ExtruderHeatingRate;
		maxTemperature = ExtruderHeaterAchievableTemp;
		startupTime = ExtruderHeaterStartupTime;
	}

	if (pwm < 0.1)
	{
		return 0.0;		// avoid overflow etc. in the following calculation because a low or silly PWM value was passed
	}

	const float adjustedMaxTemp = pwm * (maxTemperature - NormalAmbientTemperature) + NormalAmbientTemperature;
	return (adjustedMaxTemp - temp) * initialHeatingRate * pwm/(adjustedMaxTemp - NormalAmbientTemperature);
}

// End
