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
  {
    platform->SetHeater(heater, -1);
    pids[heater]->Init();
  }
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

//******************************************************************************************************

PID::PID(Platform* p, int8_t h)
{
  platform = p;
  heater = h;
}

void PID::Init()
{
  temperature = platform->GetTemperature(heater);
  setTemperature = 0.0;
  platform->SetHeater(heater, -1.0);
}

void PID::Spin()
{
  temperature = platform->GetTemperature(heater);
  if(!platform->UsePID(heater))
  {
    if(temperature < setTemperature)
      platform->SetHeater(heater, 1.0);
    else
      platform->SetHeater(heater, 0.0);
    return; 
  }
}
