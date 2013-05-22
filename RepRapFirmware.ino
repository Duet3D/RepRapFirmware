/****************************************************************************************************

RepRapFirmware - Main Program

This firmware is intended to be a fully object-oriented highly modular control program for 
RepRap self-replicating 3D printers.

It owes a lot to Marlin and to the original RepRap FiveD_GCode.

General design principles:

  * Control by RepRap G Codes.  These are taken to be machine independent, though some may be unsupported.
  * Full use of C++ OO techniques,
  * Make classes hide their data,
  * Make everything as stateless as possible,
  * No use of conditional compilation except for #include guards - if you need that, you should be
       forking the repository to make a new branch - let the repository take the strain,
  * Concentration of all machine-dependent defintions and code in Platform.h and Platform.cpp,
  * No specials for (X,Y) or (Z) - all movement is 3-dimensional,
  * Try to be efficient in memory use, but this is not critical,
  * Labour hard to be efficient in time use, and this is  critical,
  * Don't abhor floats - they work fast enough if you're clever,
  * Don't avoid arrays and structs/classes,
  * Don't avoid pointers,
  * Use operator and function overloading where appropriate, particularly for vector algebra.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

// If this goes in the right place (Platform.h) the compile fails. Why? - AB

#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>

#include "RepRapFirmware.h"

// We just need one instance of RepRap; everything else is contaied within it and hidden

RepRap reprap;

//*************************************************************************************************

// RepRap member functions.

// Do nothing more in the constructor; put what you want in RepRap:Init()

RepRap::RepRap() 
{
  active = false;
  platform = new Platform(this);
  webserver = new Webserver(platform);
  gCodes = new GCodes(platform, webserver);
  move = new Move(platform, gCodes);
  heat = new Heat(platform, gCodes);
}

void RepRap::Init()
{
  platform->Init();
  gCodes->Init();
  webserver->Init();
  move->Init();
  heat->Init();
  dbg = true;
  platform->Message(HOST_MESSAGE, "RepRapPro RepRap Firmware (Re)Started\n");
  active = true;
}

void RepRap::Exit()
{
  active = false;
  heat->Exit();
  move->Exit();
  gCodes->Exit();
  webserver->Exit();
  platform->Exit();  
}

void RepRap::Spin()
{
  if(!active)
    return;
    
  platform->Spin();
  webserver->Spin();
  gCodes->Spin();
  move->Spin();
  heat->Spin();
}

void RepRap::InterruptTime()
{
  char buffer[50];
  DDA* dda = new DDA(move, platform);
  float a[] = {1.0, 2.0, 3.0, 4.0, 5.0};
  float b[] = {2.0, 3.0, 4.0, 5.0, 6.0};
  float u = 50;
  float v = 50;
  dda->Init(a, b, u, v);
  dda->Start(false);
  unsigned long t = platform->Time();
  for(long i = 0; i < 100000; i++) 
    dda->Step(false);
  t = platform->Time() - t;
  platform->Message(HOST_MESSAGE, "Time for 100000 calls of the interrupt function: ");
  sprintf(buffer, "%ld", t);
  platform->Message(HOST_MESSAGE, buffer);
  platform->Message(HOST_MESSAGE, " microseconds.\n");
}


//*************************************************************************************************

// Utilities and storage not part of any class

static long precision[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

char* ftoa(char *a, const float& f, int prec)
{
  char *ret = a;
  long whole = (long)f;
  sprintf(a,"%d",whole);
  while (*a != '\0') a++;
  *a++ = '.';
  long decimal = abs((long)((f - (float)whole) * precision[prec]));
  sprintf(a,"%d",decimal);
  return ret;
}

// String testing

boolean StringEndsWith(char* string, char* ending)
{
  int j = strlen(string);
  int k = strlen(ending);
  if(k > j)
    return false;
  
  return(StringEquals(&string[j - k], ending));
}

boolean StringEquals(char* s1, char* s2)
{
  int i = 0;
  while(s1[i] && s2[i])
  {
     if(tolower(s1[i]) != tolower(s2[i]))
       return false;
     i++;
  }
  
  return !(s1[i] || s2[i]);
}

boolean StringStartsWith(char* string, char* starting)
{ 
  int j = strlen(string);
  int k = strlen(starting);
  if(k > j)
    return false;
  
  for(int i = 0; i < k; i++)
    if(string[i] != starting[i])
      return false;
      
  return true;
}

int StringContains(char* string, char* match)
{ 
  int i = 0;
  int count = 0;
  
  while(string[i])
  {
    if(string[i++] == match[count])
    {
      count++;
      if(!match[count])
        return i;
    } else
      count = 0;
  }
      
  return -1;
}








  


