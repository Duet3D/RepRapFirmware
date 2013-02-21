/****************************************************************************************************

RepRapFirmware - G Codes

This class interprets G Codes from one or more sources, and calls the functions in Move, Heat etc
that drive the machine to do what the G Codes command.

-----------------------------------------------------------------------------------------------------

Version 0.1

13 February 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"

GCodes::GCodes(Platform* p, Move* m, Heat* h, Webserver* w)
{
  Serial.println("GCodes constructor"); 
  platform = p;
  move = m;
  heat = h;
  webserver = w;
  lastTime = platform->time();
  gcodePointer = 0;
}

void GCodes::ActOnGcode()
{
  platform->Message(HOST_MESSAGE, "GCode: ");
  platform->Message(HOST_MESSAGE, gcodeBuffer);
  platform->Message(HOST_MESSAGE, "\n\n");
}

void GCodes::spin()
{
  if(webserver->Available())
  {
    gcodeBuffer[gcodePointer] = webserver->Read();
    if(!gcodeBuffer[gcodePointer])
    {
      gcodePointer = 0;
      ActOnGcode();
      gcodeBuffer[0] = 0;
    } else
      gcodePointer++;
  }
}

