
/****************************************************************************************************

RepRapFirmware - Main Program

This firmware is intended to be a fully object-oriented highly modular control program for
RepRap self-replicating 3D printers.

It owes a lot to Marlin and to the original RepRap FiveD_GCode.


General design principles:

  * Control by RepRap G Codes.  These are taken to be machine independent, though some may be unsupported.
  * Full use of C++ OO techniques,
  * Make classes hide their data,
  * Make everything except the Platform class (see below) as stateless as possible,
  * No use of conditional compilation except for #include guards - if you need that, you should be
       forking the repository to make a new branch - let the repository take the strain,
  * Concentration of all machine-dependent defintions and code in Platform.h and Platform.cpp,
  * No specials for (X,Y) or (Z) - all movement is 3-dimensional,
  * Except in Platform.h, use real units (mm, seconds etc) throughout the rest of the code wherever possible,
  * Try to be efficient in memory use, but this is not critical,
  * Labour hard to be efficient in time use, and this is  critical,
  * Don't abhor floats - they work fast enough if you're clever,
  * Don't avoid arrays and structs/classes,
  * Don't avoid pointers,
  * Use operator and function overloading where appropriate.


Naming conventions:

  * #defines are all CAPITALS_WITH_OPTIONAL_UNDERSCORES_BETWEEN_WORDS
  * No underscores in other names - MakeReadableWithCapitalisation
  * Class names and functions start with a CapitalLetter
  * Variables start with a lowerCaseLetter
  * Use veryLongDescriptiveNames


Structure:

There are six main classes:

  * RepRap
  * GCodes
  * Heat
  * Move
  * Platform, and
  * Webserver

RepRap:

This is just a container class for the single instances of all the others, and otherwise does very little.

GCodes:

This class is fed GCodes, either from the web interface, or from GCode files, or from a serial interface,
Interprets them, and requests actions from the RepRap machine via the other classes.

Heat:

This class implements all heating and temperature control in the RepRap machine.

Move:

This class controls all movement of the RepRap machine, both along its axes, and in its extruder drives.

Platform:

This is the only class that knows anything about the physical setup of the RepRap machine and its
controlling electronics.  It implements the interface between all the other classes and the RepRap machine.
All the other classes are completely machine-independent (though they may declare arrays dimensioned
to values #defined in Platform.h).

Webserver:

This class talks to the network (via Platform) and implements a simple webserver to give an interactive
interface to the RepRap machine.  It uses the Knockout and Jquery Javascript libraries to achieve this.



When the software is running there is one single instance of each main class, and all the memory allocation is
done on initialization.  new/malloc should not be used in the general running code, and delete is never
used.  Each class has an Init() function that resets it to its boot-up state; the constructors merely handle
that memory allocation on startup.  Calling RepRap.Init() calls all the other Init()s in the right sequence.

There are other ancillary classes that are declared in the .h files for the master classes that use them.  For
example, Move has a DDA class that implements a Bresenham/digital differential analyser.


Timing:

There is a single interrupt chain entered via Platform.Interrupt().  This controls movement step timing, and
this chain of code should be the only place that volatile declarations and structure/variable-locking are
required.  All the rest of the code is called sequentially and repeatedly as follows:

All the main classes have a Spin() function.  These are called in a loop by the RepRap.Spin() function and implement
simple timesharing.  No class does, or ever should, wait inside one of its functions for anything to happen or call
any sort of delay() function.  The general rule is:

  Can I do a thing?
    Yes - do it
    No - set a flag/timer to remind me to do it next-time-I'm-called/at-a-future-time and return.

The restriction this strategy places on almost all the code in the firmware (that it must execute quickly and
never cause waits or delays) is balanced by the fact that none of that code needs to worry about synchronization,
locking, or other areas of code accessing items upon which it is working.  As mentioned, only the interrupt
chain needs to concern itself with such problems.  Unlike movement, heating (including PID controllers) does
not need the fast precision of timing that interrupts alone can offer.  Indeed, most heating code only needs
to execute a couple of times a second.

Most data is transferred bytewise, with classes' Spin() functions typically containing code like this:

  Is a byte available for me?
    Yes
      read it and add it to my buffer
      Is my buffer complete?
         Yes
           Act on the contents of my buffer
         No
           Return
  No
    Return

Note that it is simple to raise the "priority" of any class's activities relative to the others by calling its
Spin() function more than once from RepRap.Spin().

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

// If this goes in the right place (Platform.h) the compile fails. Why? - AB

//#include <SPI.h>
//#include <Ethernet.h>
//#include <SD.h>

#include "RepRapFirmware.h"

// We just need one instance of RepRap; everything else is contained within it and hidden

RepRap reprap;

//*************************************************************************************************

// RepRap member functions.

// Do nothing more in the constructor; put what you want in RepRap:Init()

RepRap::RepRap()
{
  active = false;
  platform = new Platform();
  webserver = new Webserver(platform);
  gCodes = new GCodes(platform, webserver);
  move = new Move(platform, gCodes);
  heat = new Heat(platform, gCodes);
}

void RepRap::Init()
{
  debug = false;
  platform->Init();
  gCodes->Init();
  webserver->Init();
  move->Init();
  heat->Init();
  active = true;

  platform->Message(HOST_MESSAGE, NAME);
  platform->Message(HOST_MESSAGE, " Version ");
  platform->Message(HOST_MESSAGE, VERSION);
  platform->Message(HOST_MESSAGE, ", dated ");
  platform->Message(HOST_MESSAGE, DATE);
  platform->Message(HOST_MESSAGE, ".\n\nExecuting ");
  platform->Message(HOST_MESSAGE, platform->GetConfigFile());
  platform->Message(HOST_MESSAGE, "...\n\n");

  while(gCodes->RunConfigurationGCodes()); // Wait till the file is finished

  platform->Message(HOST_MESSAGE, "\nStarting network...\n");
  platform->StartNetwork(); // Need to do this here, as the configuration GCodes may set IP address etc.

  platform->Message(HOST_MESSAGE, "\n");
  platform->Message(HOST_MESSAGE, NAME);
  platform->Message(HOST_MESSAGE, " is up and running.\n");
}

void RepRap::Exit()
{
  active = false;
  heat->Exit();
  move->Exit();
  gCodes->Exit();
  webserver->Exit();
  platform->Message(HOST_MESSAGE, "RepRap class exited.\n");
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

void RepRap::Diagnostics()
{
  platform->Diagnostics();
  move->Diagnostics();
  heat->Diagnostics();
  gCodes->Diagnostics();
  webserver->Diagnostics();
}

// Turn off the heaters, disable the motors, and
// deactivate the Heat and Move classes.  Leave everything else
// working.

void RepRap::EmergencyStop()
{
	int8_t i;

	//platform->DisableInterrupts();

	heat->Exit();
	for(i = 0; i < HEATERS; i++)
		platform->SetHeater(i, 0.0);


	// We do this twice, to avoid an interrupt switching
	// a drive back on.  move->Exit() should prevent
	// interrupts doing this.

	for(int8_t i = 0; i < 2; i++)
	{
		move->Exit();
		for(i = 0; i < DRIVES; i++)
		{
			platform->SetMotorCurrent(i, 0.0);
			platform->Disable(i);
		}
	}
	platform->Message(HOST_MESSAGE, "Emergency Stop! Reset the controller to continue.");
}



//*************************************************************************************************

// Utilities and storage not part of any class


// Float to a string.

static long precision[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
char scratchString[STRING_LENGTH];

char* ftoa(char *a, const float& f, int prec)
{
  if(a == NULL)
    a = scratchString;
  char *ret = a;
  long whole = (long)f;
  if(!whole && f < 0.0)
  {
	  a[0] = '-';
	  a++;
  }
  snprintf(a, STRING_LENGTH, "%d", whole);
  while (*a != '\0') a++;
  *a++ = '.';
  long decimal = abs((long)((f - (float)whole) * precision[prec]));
  snprintf(a, STRING_LENGTH, "%0*d", prec, decimal);
  return ret;
}

// String testing

bool StringEndsWith(char* string, char* ending)
{
  int j = strlen(string);
  int k = strlen(ending);
  if(k > j)
    return false;

  return(StringEquals(&string[j - k], ending));
}

bool StringEquals(char* s1, char* s2)
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

bool StringStartsWith(char* string, char* starting)
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











