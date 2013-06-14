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
    pids[heater] = new PID(heater);
  active = false;
}

void Heat::Init(float st)
{
  sampleTime = st;

  for(int8_t heater=0; heater < HEATERS; heater++)
  {
    platform->SetHeater(heater, -1);
//    pids[heater]->Init();
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
   if(t - lastTime < sampleTime)
     return;
   lastTime = t;
}

//******************************************************************************************************

PID::PID(int8_t h)
{
  heater = h;
}
 /* 
    kp = p;
  ki = i;
  kd = d;
  kw = w;*/
