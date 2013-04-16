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

GCodes::GCodes(Platform* p, Webserver* w)
{
  active = false;
  platform = p;
  webserver = w;
}

void GCodes::Exit()
{
   active = false;
}

void GCodes::Init()
{
  lastTime = platform->Time();
  gcodePointer = 0;
  active = true;
  moveAvailable = false;
  heatAvailable = false;
  readPointer = -1;
  drivesRelative = false;
  axesRelative = false;
  gCodeLetters = GCODE_LETTERS;
}

void GCodes::SetUpMove()
{
  reprap.GetMove()->GetCurrentState(moveBuffer);
  for(char i = 0; i < DRIVES; i++)
  {
    if(i < AXES)
    {
      if(Seen(gCodeLetters[i]))
      {
        if(axesRelative)
          moveBuffer[i] += GetFValue();
        else
          moveBuffer[i] = GetFValue();
      }
    } else
    {
      if(Seen(gCodeLetters[i]))
      {
        if(drivesRelative)
          moveBuffer[i] = GetFValue();
        else
          moveBuffer[i] = GetFValue(); //***TODO need to store old values and subtract them here
      }
    }
  }
  if(Seen(gCodeLetters[DRIVES]))
    moveBuffer[DRIVES] = GetFValue();
  moveAvailable = true;  
}

void GCodes::ActOnGcode()
{
  int code;
  
  if(Seen('G'))
  {
    code = GetIValue();
    switch(code)
    {
    case 1:
      SetUpMove();
      break;
      
    case 90:
      drivesRelative = false;
      axesRelative = false;
      break;
      
    case 91:
      drivesRelative = true;
      axesRelative = true;
      break;
      
    default:
      platform->Message(HOST_MESSAGE, "GCodes - invalid G Code: ");
      platform->Message(HOST_MESSAGE, gcodeBuffer);
      platform->Message(HOST_MESSAGE, "<br>\n");
    }
    return;
  }
  
  if(Seen('M'))
  {
    code = GetIValue();
    switch(code)
    {
    case 82:
      drivesRelative = false;
      break;
      
    case 83:
      drivesRelative = true;
      break;    
     
    default:
      platform->Message(HOST_MESSAGE, "GCodes - invalid M Code: ");
      platform->Message(HOST_MESSAGE, gcodeBuffer);
      platform->Message(HOST_MESSAGE, "<br>\n");
    }
    return;
  }
  
  if(Seen('T'))
  {
    code = GetIValue();
    switch(code)
    {
    
     
    default:
      platform->Message(HOST_MESSAGE, "GCodes - invalid T Code: ");
      platform->Message(HOST_MESSAGE, gcodeBuffer);
      platform->Message(HOST_MESSAGE, "<br>\n");
    }
  }
  
}


void GCodes::Spin()
{
  if(!active)
    return;
    
  if(webserver->Available())
  {
    gcodeBuffer[gcodePointer] = webserver->Read();
    if(gcodeBuffer[gcodePointer] == '\n' || !gcodeBuffer[gcodePointer])
    {
      gcodeBuffer[gcodePointer] = 0;
      gcodePointer = 0;
      ActOnGcode();
      gcodeBuffer[0] = 0;
    } else
      gcodePointer++;
    
    if(gcodePointer >= GCODE_LENGTH)
    {
      platform->Message(HOST_MESSAGE, "GCodes: G Code buffer length overflow.<br>\n");
      gcodePointer = 0;
      gcodeBuffer[0] = 0;
    }
  }
}


boolean GCodes::ReadMove(float* m)
{
    if(!moveAvailable)
      return false; 
    for(char i = 0; i <= DRIVES; i++)
      m[i] = moveBuffer[i];
    moveAvailable = false;
}


boolean GCodes::ReadHeat(float* h)
{
    
}


boolean GCodes::Seen(char c)
{
  readPointer = 0;
  while(gcodeBuffer[readPointer])
  {
    if(gcodeBuffer[readPointer] == c)
      return true;
    readPointer++;
  }
  readPointer = -1;
  return false;
}


float GCodes::GetFValue()
{
  if(readPointer < 0)
  {
     platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode float before a search.<br>\n");
     return 0.0;
  }
  float result = (float)strtod(&gcodeBuffer[readPointer + 1], NULL);
  readPointer = -1;
  return result; 
}


int GCodes::GetIValue()
{
  if(readPointer < 0)
  {
    platform->Message(HOST_MESSAGE, "GCodes: Attempt to read a GCode int before a search.<br>\n");
    return 0;
  }
  int result = (int)strtol(&gcodeBuffer[readPointer + 1], NULL, 0);
  readPointer = -1;
  return result;  
}


