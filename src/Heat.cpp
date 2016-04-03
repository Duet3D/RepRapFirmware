/****************************************************************************************************

RepRapFirmware - Heat

This is all the code to deal with heat and temperature.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"

const float invHeatPwmAverageCount = HEAT_SAMPLE_TIME/HEAT_PWM_AVERAGE_TIME;

Heat::Heat(Platform* p) : platform(p), active(false), coldExtrude(false), bedHeater(BED_HEATER), chamberHeater(-1)
{
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		pids[heater] = new PID(platform, heater);
	}
}

void Heat::Init()
{
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		pids[heater]->Init();
	}
	lastTime = platform->Time();
	longWait = lastTime;
	coldExtrude = false;
	active = true;
}

void Heat::Exit()
{
	for (size_t heater = 0; heater < HEATERS; heater++)
	{
		pids[heater]->SwitchOff();
	}
	platform->Message(HOST_MESSAGE, "Heat class exited.\n");
	active = false;
}

void Heat::Spin()
{
	if (active)
	{
		float t = platform->Time();
		if (t - lastTime >= platform->HeatSampleTime())
		{
			lastTime = t;
			for (size_t heater=0; heater < HEATERS; heater++)
			{
				pids[heater]->Spin();
			}
		}
	}
	platform->ClassReport(longWait);
}

void Heat::Diagnostics() 
{
	platform->Message(GENERIC_MESSAGE, "Heat Diagnostics:\n");
	for (size_t heater=0; heater < HEATERS; heater++)
	{
		if (pids[heater]->Active())
		{
			platform->MessageF(GENERIC_MESSAGE, "Heater %d: I-accumulator = %.1f\n", heater, pids[heater]->GetAccumulator());
		}
	}
}

bool Heat::AllHeatersAtSetTemperatures(bool includingBed) const
{
	size_t firstHeater = 	(bedHeater == -1) ? E0_HEATER :
							(includingBed) ? min<int8_t>(bedHeater, E0_HEATER) : E0_HEATER;

	for(size_t heater = firstHeater; heater < HEATERS; heater++)
	{
		if (!HeaterAtSetTemperature(heater))
		{
			return false;
		}
	}
	return true;
}

//query an individual heater
bool Heat::HeaterAtSetTemperature(int8_t heater) const
{
	// If it hasn't anything to do, it must be right wherever it is...
	if (heater < 0 || pids[heater]->SwitchedOff() || pids[heater]->FaultOccurred())
	{
		return true;
	}

	float dt = GetTemperature(heater);
	float target = (pids[heater]->Active()) ? GetActiveTemperature(heater) : GetStandbyTemperature(heater);
	return (target < TEMPERATURE_LOW_SO_DONT_CARE) || (fabs(dt - target) <= TEMPERATURE_CLOSE_ENOUGH);
}

//******************************************************************************************************

PID::PID(Platform* p, int8_t h) : platform(p), heater(h)
{
}

void PID::Init()
{
	SetHeater(0.0);
	temperature = platform->GetTemperature(heater);
	activeTemperature = ABS_ZERO;
	standbyTemperature = ABS_ZERO;
	lastTemperature = temperature;
	temp_iState = 0.0;
	badTemperatureCount = 0;
	temperatureFault = false;
	active = false; 		// Default to standby temperature
	switchedOff = true;
	heatingUp = false;
	averagePWM = 0.0;

	// Time the sensor was last sampled.  During startup, we use the current
	// time as the initial value so as to not trigger an immediate warning from
	// the Tick ISR.
	lastSampleTime = millis();
}

void PID::SwitchOn()
{
	if (reprap.Debug(Module::moduleHeat))
	{
		platform->MessageF(GENERIC_MESSAGE, "Heater %d switched on.\n", heater);
	}
	switchedOff = temperatureFault;
}

void PID::SetHeater(float power) const
{
	platform->SetHeater(heater, power);
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

	// Always know our temperature, regardless of whether we have been switched on or not

	Platform::TempError err = Platform::TempError::errOk;		// assume no error
	temperature = platform->GetTemperature(heater, &err);		// in the event of an error, err is set and BAD_ERROR_TEMPERATURE is returned

	// If we're not switched on, or there's a fault, turn the power off and go home.
	// If we're not switched on, then nothing is using us.  This probably means that
	// we don't even have a thermistor connected.  So don't even check for faults if we
	// are not switched on.  This is safe, as the next bit of code always turns our
	// heater off in that case anyway.
	if (temperatureFault || switchedOff)
	{
		SetHeater(0.0); // Make sure to turn it off...
		averagePWM *= (1.0 - invHeatPwmAverageCount);
		return;
	}

	// We are switched on.  Check for faults.  Temperature silly-low or silly-high mean open-circuit
	// or shorted thermistor respectively.
	if (temperature < BAD_LOW_TEMPERATURE)
	{
		err = Platform::TempError::errOpen;
	}
	else if (temperature > platform->GetTemperatureLimit())
	{
		err = Platform::TempError::errTooHigh;
	}

	if (err != Platform::TempError::errOk)
	{
		if (platform->DoThermistorAdc(heater) || !(Platform::TempErrorPermanent(err)))
		{
			// Error may be a temporary error and may correct itself after a few additional reads
			badTemperatureCount++;
		}
		else
		{
			// Error condition is not expected to correct itself (e.g., digital device such
			// as a MAX31855 reporting that the temp sensor is shorted -- even a temporary
			// short in such a situation warrants manual attention and correction).
			badTemperatureCount = MAX_BAD_TEMPERATURE_COUNT + 1;
		}

		if (badTemperatureCount > MAX_BAD_TEMPERATURE_COUNT)
		{
			SetHeater(0.0);
			temperatureFault = true;
			platform->MessageF(GENERIC_MESSAGE, "Temperature fault on heater %d%s%s, T = %.1f\n",
							 heater,
							 (err != Platform::TempError::errOk) ? ", " : "",
							 (err != Platform::TempError::errOk) ? Platform::TempErrorStr(err) : "",
							 temperature);
			reprap.FlagTemperatureFault(heater);
		}
	}
	else
	{
		badTemperatureCount = 0;
	}

	// Now check how long it takes to warm up.  If too long, maybe the thermistor is not in contact with the heater
	if (heatingUp && heater != reprap.GetHeat()->GetBedHeater()) // FIXME - also check bed warmup time?
	{
		float tmp = (active) ? activeTemperature : standbyTemperature;
		if (temperature < tmp - TEMPERATURE_CLOSE_ENOUGH)
		{
			float tim = platform->Time() - timeSetHeating;
			float limit = platform->TimeToHot();
			if (tim > limit && limit > 0.0)
			{
				SetHeater(0.0);
				temperatureFault = true;
				platform->MessageF(GENERIC_MESSAGE, "Heating fault on heater %d, T = %.1f C; still not at temperature %.1f after %f seconds.\n", heater, temperature, tmp, tim);
				reprap.FlagTemperatureFault(heater);
			}
		}
		else
		{
			heatingUp = false;
		}
	}

	float targetTemperature = (active) ? activeTemperature : standbyTemperature;
	float error = targetTemperature - temperature;
	const PidParameters& pp = platform->GetPidParameters(heater);

	if (!pp.UsePID())
	{
		float heaterValue = (error > 0.0) ? min<float>(pp.kS, 1.0) : 0.0;
		SetHeater(heaterValue);
		averagePWM = averagePWM * (1.0 - invHeatPwmAverageCount) + heaterValue;
		return;
	}

	if (error < -pp.fullBand)
	{
		// actual temperature is well above target
		temp_iState = (targetTemperature + pp.fullBand - 25.0) * pp.kT;	// set the I term to our estimate of what will be needed ready for the switch to PID
		SetHeater(0.0);
		averagePWM *= (1.0 - invHeatPwmAverageCount);
		lastTemperature = temperature;
		return;
	}

	if (error > pp.fullBand)
	{
		// actual temperature is well below target
		temp_iState = (targetTemperature - pp.fullBand - 25.0) * pp.kT;	// set the I term to our estimate of what will be needed ready for the switch to PID
		float heaterValue = min<float>(pp.kS, 1.0);
		SetHeater(heaterValue);
		averagePWM = averagePWM * (1.0 - invHeatPwmAverageCount) + heaterValue;
		lastTemperature = temperature;
		return;
	}

	float sampleInterval = platform->HeatSampleTime();
	temp_iState += error * pp.kI * sampleInterval;

	if (temp_iState < pp.pidMin)
	{
		temp_iState = pp.pidMin;
	}
	else if (temp_iState > pp.pidMax)
	{
		temp_iState = pp.pidMax;
	}

	float temp_dState = pp.kD * (temperature - lastTemperature) / sampleInterval;
	float result = (pp.kP * error + temp_iState - temp_dState) * pp.kS / 255.0;

	lastTemperature = temperature;

	if (result < 0.0)
	{
		result = 0.0;
	}
	else if (result > 1.0)
	{
		result = 1.0;
	}

	if (!temperatureFault)
	{
		SetHeater(result);
	}

	averagePWM = averagePWM * (1.0 - invHeatPwmAverageCount) + result;

//  debugPrintf("Heater %d: e=%f, P=%f, I=%f, d=%f, r=%f\n", heater, error, pp.kP*error, temp_iState, temp_dState, result);
}

void PID::SetActiveTemperature(float t)
{
	if (t > platform->GetTemperatureLimit())
	{
		platform->MessageF(GENERIC_MESSAGE, "Error: Temperature %.1f too high for heater %d!\n", t, heater);
	}

	SwitchOn();
	activeTemperature = t;
}

void PID::SetStandbyTemperature(float t)
{
	if (t > platform->GetTemperatureLimit())
	{
		platform->MessageF(GENERIC_MESSAGE, "Error: Temperature %.1f too high for heater %d!\n", t, heater);
	}

	SwitchOn();
	standbyTemperature = t;
}

void PID::Activate()
{
	if (temperatureFault)
	{
		return;
	}

	SwitchOn();
	active = true;
	if (!heatingUp)
	{
		timeSetHeating = platform->Time();
	}
	heatingUp = activeTemperature > temperature;
}

void PID::Standby()
{
	if (temperatureFault)
	{
		return;
	}

	SwitchOn();
	active = false;
	if (!heatingUp)
	{
		timeSetHeating = platform->Time();
	}
	heatingUp = standbyTemperature > temperature;
}

void PID::ResetFault()
{
	temperatureFault = false;
    timeSetHeating = platform->Time();		// otherwise we will get another timeout immediately
	badTemperatureCount = 0;
}

void PID::SwitchOff()
{
	SetHeater(0.0);
	active = false;
	switchedOff = true;
	heatingUp = false;
}

float PID::GetAveragePWM() const
{
	return averagePWM * invHeatPwmAverageCount;
}

// End
