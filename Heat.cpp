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
  Serial.println("Heat constructor"); 
  platform = p;
  lastTime = platform->time();
}

void Heat::spin()
{
  
}
