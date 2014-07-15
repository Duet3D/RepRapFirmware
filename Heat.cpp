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
    pids[heater]->Init();
  lastTime = platform->Time();
  longWait = lastTime;
  active = true;
}

void Heat::Exit()
{
  for(int8_t heater=0; heater < HEATERS; heater++)
	 pids[heater]->SwitchOff();
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
  platform->ClassReport("Heat", longWait);
}

void Heat::Diagnostics() 
{
  platform->AppendMessage(BOTH_MESSAGE, "Heat Diagnostics:\n");
  // TODO - Put something useful in here
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

  if(temperatureFault || switchedOff)
  {
	  platform->SetHeater(heater, 0.0); // Make sure...
	  return;
  }

  // We are switched on.  Check for faults.

  if(temperature < BAD_LOW_TEMPERATURE || temperature > BAD_HIGH_TEMPERATURE)
  {
	  badTemperatureCount++;
	  if(badTemperatureCount > MAX_BAD_TEMPERATURE_COUNT)
	  {
		  platform->SetHeater(heater, 0.0);
		  temperatureFault = true;
		  switchedOff = true;
		  snprintf(scratchString, STRING_LENGTH, "Temperature measurement fault on heater %d, T = %.1f\n", heater, temperature);
		  platform->Message(HOST_MESSAGE, scratchString);
	  }
  }
  else
  {
	  badTemperatureCount = 0;
  }

  float targetTemperature = (active) ? activeTemperature : standbyTemperature;
  float error = targetTemperature - temperature;
  const PidParameters& pp = platform->GetPidParameters(heater);
  
  if(!pp.UsePID())
  {
	  platform->SetHeater(heater, (error > 0.0) ? 1.0 : 0.0);
	  return;
  }
  
  if(error < -pp.fullBand)
  {
	  // actual temperature is well above target
	  temp_iState = (targetTemperature - 25.0 + pp.fullBand) * pp.kT;	// set the I term to our estimate of what will be needed ready for the switch to PID
	  platform->SetHeater(heater, 0.0);
	  lastTemperature = temperature;
	  return;
  }
  if(error > pp.fullBand)
  {
	  // actual temperature is well below target
	  temp_iState = (targetTemperature - 25.0 - pp.fullBand) * pp.kT;	// set the I term to our estimate of what will be needed ready for the switch to PID
	  platform->SetHeater(heater, 1.0);
	  lastTemperature = temperature;
	  return;
  }  
   
  temp_iState += error * pp.kI;
  
  if (temp_iState < pp.pidMin) temp_iState = pp.pidMin;
  else if (temp_iState > pp.pidMax) temp_iState = pp.pidMax;
   
  float temp_dState =  pp.kD * (temperature - lastTemperature);
  float result = pp.kP * error + temp_iState - temp_dState;

  lastTemperature = temperature;

  // Legacy - old RepRap PID parameters were set to give values in [0, 255] for 1 byte PWM control
  // TODO - maybe change them to give [0.0, 1.0]?

  if (result < 0.0) result = 0.0;
  else if (result > 255.0) result = 255.0;
  result = result/255.0;

  if(!temperatureFault)
  {
	  platform->SetHeater(heater, result);
  }

  //debugPrintf("Heat: e=%f, P=%f, I=%f, d=%f, r=%f\n", error, platform->PidKp(heater)*error, temp_iState, temp_dState, result);
}
