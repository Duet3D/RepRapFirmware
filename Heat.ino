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

Heat::Heat(Platform* p)
{
  //Serial.println("Heat constructor"); 
  platform = p;
}

void Heat::Init()
{
  lastTime = platform->Time();
  //frac = 0;
  //inc = 0.01;  
}

void Heat::Exit()
{
  
}

void Heat::Spin()
{
   unsigned long t = platform->Time();
   if(t - lastTime < 3000)
     return;
   lastTime = t;
/*   if(frac > 1 || frac < 0)
   {
     inc = -inc;
     //Serial.print("Temps: ");
     //Serial.print(platform->getTemperature(0));
    // Serial.print(", ");
     //Serial.println(platform->getTemperature(1));
   }
   platform->setHeater(0, frac);
   platform->setHeater(1, 1 - frac);
   frac += inc;*/
}
