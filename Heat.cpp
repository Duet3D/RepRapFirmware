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

Heat::Heat(Platform* p, GCodes* g)
{
  platform = p;
  gCodes = g;
  for(int8_t heater=0; heater < HEATERS; heater++)
    pids[heater] = new PID(platform, heater);
  active = false;
}

void Heat::Init()
{
  for(int8_t heater=0; heater < HEATERS; heater++)
  {
    pids[heater]->Init();
  }
  lastTime = platform->Time();
  longWait = lastTime;
  active = true;
}

void Heat::Exit()
{
  for(int8_t heater=0; heater < HEATERS; heater++)
  {
	 pids[heater]->SwitchOff();
  }
  platform->Message(HOST_MESSAGE, "Heat class exited.\n");
  active = false;
}

void Heat::Spin()
{
  if(!active)
    return;
    
  float t = platform->Time();
  if(t - lastTime < platform->HeatSampleTime())
    return;
  lastTime = t;
  for(int8_t heater=0; heater < HEATERS; heater++)
  {
    pids[heater]->Spin();
  }
  platform->ClassReport(longWait);
}

void Heat::Diagnostics() 
{
  platform->AppendMessage(BOTH_MESSAGE, "Heat Diagnostics:\n");
  for(int8_t heater=0; heater < HEATERS; heater++)
  {
	  if (pids[heater]->active)
	  {
		  platform->AppendMessage(BOTH_MESSAGE, "Heater %d: I-accumulator = %.1f\n", heater, pids[heater]->temp_iState);
	  }
  }
}

bool Heat::AllHeatersAtSetTemperatures(bool includingBed) const
{
	for(int8_t heater = (includingBed) ? 0 : 1; heater < HEATERS; heater++)
	{
		if(!HeaterAtSetTemperature(heater))
			return false;
	}
	return true;
}

//query an individual heater
bool Heat::HeaterAtSetTemperature(int8_t heater) const
{
	if(pids[heater]->SwitchedOff())  // If it hasn't anything to do, it must be right wherever it is...
		return true;

	float dt = GetTemperature(heater);
	float target = (pids[heater]->Active()) ? GetActiveTemperature(heater) : GetStandbyTemperature(heater);
	return (target < TEMPERATURE_LOW_SO_DONT_CARE) || (fabs(dt - target) <= TEMPERATURE_CLOSE_ENOUGH);
}

//******************************************************************************************************

PID::PID(Platform* p, int8_t h)
{
  platform = p;
  heater = h;
}

void PID::Init()
{
  platform->SetHeater(heater, 0.0);
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
}

void PID::SwitchOn()
{
//	if(reprap.Debug())
//	{
//		snprintf(scratchString, STRING_LENGTH, "Heater %d switched on.\n", heater);
//		platform->Message(BOTH_MESSAGE, scratchString);
//	}
	switchedOff = false;
}


void PID::Spin()
{
  // Always know our temperature, regardless of whether we have been switched on or not

  temperature = platform->GetTemperature(heater);

  // If we're not switched on, or there's a fault, turn the power off and go home.
  // If we're not switched on, then nothing is using us.  This probably means that
  // we don't even have a thermistor connected.  So don't even check for faults if we
  // are not switched on.  This is safe, as the next bit of code always turns our
  // heater off in that case anyway.

  if (temperatureFault || switchedOff)
  {
	  platform->SetHeater(heater, 0.0); // Make sure...
	  averagePWM *= (1.0 - invHeatPwmAverageCount);
	  return;
  }

  // We are switched on.  Check for faults.  Temperature silly-low or silly-high mean open-circuit
  // or shorted thermistor respectively.

  if (temperature < BAD_LOW_TEMPERATURE || temperature > BAD_HIGH_TEMPERATURE)
  {
	  badTemperatureCount++;
	  if (badTemperatureCount > MAX_BAD_TEMPERATURE_COUNT)
	  {
		  platform->SetHeater(heater, 0.0);
		  temperatureFault = true;
		  //switchedOff = true;
		  platform->Message(BOTH_MESSAGE, "Temperature fault on heater %d, T = %.1f\n", heater, temperature);
		  reprap.FlagTemperatureFault(heater);
	  }
  }
  else
  {
	  badTemperatureCount = 0;
  }

  // Now check how long it takes to warm up.  If too long, maybe the thermistor is not in contact with the heater

  if (heatingUp && heater != HOT_BED) // FIXME - also check bed warmup time?
  {
	  float tmp = (active) ? activeTemperature : standbyTemperature;
	  if (temperature < tmp - TEMPERATURE_CLOSE_ENOUGH)
	  {
		  float tim = platform->Time() - timeSetHeating;
		  float limit = platform->TimeToHot();
		  if (tim > limit && limit > 0.0)
		  {
			  platform->SetHeater(heater, 0.0);
			  temperatureFault = true;
			  //switchedOff = true;
			  platform->Message(BOTH_MESSAGE, "Heating fault on heater %d, T = %.1f C; still not at temperature %.1f after %f seconds.\n", heater, temperature, tmp, tim);
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
	platform->SetHeater(heater, heaterValue);
	averagePWM = averagePWM * (1.0 - invHeatPwmAverageCount) + heaterValue;
	return;
  }
  
  if(error < -pp.fullBand)
  {
	  // actual temperature is well above target
	  temp_iState = (targetTemperature + pp.fullBand - 25.0) * pp.kT;	// set the I term to our estimate of what will be needed ready for the switch to PID
	  platform->SetHeater(heater, 0.0);
	  averagePWM *= (1.0 - invHeatPwmAverageCount);
	  lastTemperature = temperature;
	  return;
  }
  if(error > pp.fullBand)
  {
	  // actual temperature is well below target
	  temp_iState = (targetTemperature - pp.fullBand - 25.0) * pp.kT;	// set the I term to our estimate of what will be needed ready for the switch to PID
	  float heaterValue = min<float>(pp.kS, 1.0);
	  platform->SetHeater(heater, heaterValue);
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
	  platform->SetHeater(heater, result);
  }

  averagePWM = averagePWM * (1.0 - invHeatPwmAverageCount) + result;

//  debugPrintf("Heater %d: e=%f, P=%f, I=%f, d=%f, r=%f\n", heater, error, pp.kP*error, temp_iState, temp_dState, result);
}

float PID::GetAveragePWM() const
{
	return averagePWM * invHeatPwmAverageCount;
}

// End

