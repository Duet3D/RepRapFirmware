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

inline Platform* RepRap::GetPlatform() { return platform; }
inline Move* RepRap::GetMove() { return move; }
inline Heat* RepRap::GetHeat() { return heat; }
inline GCodes* RepRap::GetGCodes() { return gCodes; }
inline Webserver* RepRap::GetWebserver() { return webserver; }

void RepRap::Init()
{
  platform->Init();
  gCodes->Init();
  webserver->Init();
  move->Init();
  heat->Init();
  platform->Message(HOST_MESSAGE, "RepRapPro RepRap Firmware (Re)Started<br>\n");
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


void RepRap::Interrupt()
{
  
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









  


