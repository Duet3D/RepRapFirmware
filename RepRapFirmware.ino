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


void RepRap::Init()
{
  platform = new Platform(this);
  move = new Move(platform);
  heat = new Heat(platform);
  webserver = new Webserver(platform);
  gcodes = new GCodes(platform, move, heat, webserver);
  platform->Message(HOST_MESSAGE, "RepRapPro RepRap Firmware Started\n\n");
}

void RepRap::Spin()
{
  platform->Spin();
  move->Spin();
  heat->Spin();
  gcodes->Spin();
  webserver->Spin();
}

void RepRap::Interrupt()
{
  
}


  


