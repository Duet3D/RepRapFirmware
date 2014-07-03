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
    pids[heater]->Spin();
  platform->ClassReport("Heat", longWait);
}

void Heat::Diagnostics() 
{
  platform->Message(HOST_MESSAGE, "Heat Diagnostics:\n");
  // TODO - Put something useful in here
}

bool Heat::AllHeatersAtSetTemperatures()
{
	for(int8_t heater = 0; heater < HEATERS; heater++)
	{
		if(!HeaterAtSetTemperature(heater))
			return false;
	}
	return true;
}

//query an individual heater

bool Heat::HeaterAtSetTemperature(int8_t heater)
{
	if(pids[heater]->SwitchedOff())  // If it hasn't anything to do, it must be right wherever it is...
		return true;

	float dt = GetTemperature(heater);
	if(pids[heater]->Active())
	{
		if(GetActiveTemperature(heater) < TEMPERATURE_LOW_SO_DONT_CARE)
			dt = 0.0;
		else
			dt = fabs(dt - GetActiveTemperature(heater));
	} else
	{
		if(GetStandbyTemperature(heater) < TEMPERATURE_LOW_SO_DONT_CARE)
			dt = 0.0;
		else
			dt = fabs(dt - GetStandbyTemperature(heater));
	}
	if(dt > TEMPERATURE_CLOSE_ENOUGH)
		return false;
	return true;
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
  temp_dState = 0.0;
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
		  platform->Message(HOST_MESSAGE, "Temperature measurement fault on heater ");
		  snprintf(scratchString, STRING_LENGTH, "%d", heater);
		  platform->Message(HOST_MESSAGE, scratchString);
		  platform->Message(HOST_MESSAGE, ", T = ");
  		  snprintf(scratchString, STRING_LENGTH, "%f", temperature);
		  platform->Message(HOST_MESSAGE, scratchString);
		  platform->Message(HOST_MESSAGE, "\n");
	  }
  } else
  {
	  badTemperatureCount = 0;
  }

  float error = ((active) ? activeTemperature : standbyTemperature) - temperature;
  
  if(!platform->UsePID(heater))
  {
    platform->SetHeater(heater, (error > 0.0) ? 1.0 : 0.0);
    return; 
  }
  
  if(error < -platform->FullPidBand(heater))
  {
     temp_iState = 0.0;
     platform->SetHeater(heater, 0.0);
     lastTemperature = temperature;
     return;
  }
  if(error > platform->FullPidBand(heater))
  {
     temp_iState = 0.0;
     platform->SetHeater(heater, 1.0);
     lastTemperature = temperature;
     return;
  }  
   
  temp_iState += error * platform->PidKi(heater);
  
  if (temp_iState < platform->PidMin(heater)) temp_iState = platform->PidMin(heater);
  else if (temp_iState > platform->PidMax(heater)) temp_iState = platform->PidMax(heater);
   
  temp_dState =  platform->PidKd(heater)*(temperature - lastTemperature)*(1.0 - platform->DMix(heater)) + platform->DMix(heater)*temp_dState; 

  float result = platform->PidKp(heater)*error + temp_iState - temp_dState;

  lastTemperature = temperature;

  // Legacy - old RepRap PID parameters were set to give values in [0, 255] for 1 byte PWM control
  // TODO - maybe change them to give [0.0, 1.0]?

  if (result < 0.0) result = 0.0;
  else if (result > 255.0) result = 255.0;
  result = result/255.0;

  if(!temperatureFault)
	  platform->SetHeater(heater, result);

//  char buffer[100];
//  snprintf(buffer, ARRAY_SIZE(buffer), "Heat: e=%f, P=%f, I=%f, d=%f, r=%f\n", error, platform->PidKp(heater)*error, temp_iState, temp_dState, result);
//  platform->Message(HOST_MESSAGE, buffer);

}
