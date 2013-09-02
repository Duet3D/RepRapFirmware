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
  active = true; 
}

void Heat::Exit()
{
  active = false;
}

void Heat::Spin()
{
  if(!active)
    return;
    
  unsigned long t = platform->Time();
  if(t - lastTime < platform->HeatSampleTime())
    return;
  lastTime = t;
  for(int8_t heater=0; heater < HEATERS; heater++)
    pids[heater]->Spin();
}

void Heat::Diagnostics() 
{
  platform->Message(HOST_MESSAGE, "Heat Diagnostics:\n"); 
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
  active = false;
}


void PID::Spin()
{
  temperature = platform->GetTemperature(heater);
  
  float error;
  if(active)
    error = activeTemperature - temperature;
  else
    error = standbyTemperature - temperature;
  
  if(!platform->UsePID(heater))
  {
    if(error > 0.0)
      platform->SetHeater(heater, 1.0);
    else
      platform->SetHeater(heater, 0.0);
    return; 
  }
  
  if(error < -platform->FullPidBand(heater))
  {
     temp_iState = 0.0;
     platform->SetHeater(heater, 0.0);
     return;
  }
  if(error > platform->FullPidBand(heater))
  {
     temp_iState = 0.0;
     platform->SetHeater(heater, 1.0);
     return;
  }  
   
  temp_iState += error;
  
  if (temp_iState < platform->PidMin(heater)) temp_iState = platform->PidMin(heater);
  if (temp_iState > platform->PidMax(heater)) temp_iState = platform->PidMax(heater);
   
  temp_dState =  platform->PidKd(heater)*(temperature - lastTemperature)*(1.0 - platform->DMix(heater)) + platform->DMix(heater)*temp_dState; 

  float result = platform->PidKp(heater)*error + platform->PidKi(heater)*temp_iState - temp_dState;

  lastTemperature = temperature;

  if (result < 0.0) result = 0.0;
  if (result > 255.0) result = 255.0;
  result = result/255.0;
  platform->SetHeater(heater, result);
}
