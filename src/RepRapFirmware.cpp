
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
  * Concentration of all machine-dependent definitions and code in Platform.h and Platform.cpp,
  * No specials for (X,Y) or (Z) - all movement is 3-dimensional,
  * Except in Platform.h, use real units (mm, seconds etc) throughout the rest of the code wherever possible,
  * Try to be efficient in memory use, but this is not critical,
  * Labour hard to be efficient in time use, and this is critical,
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

There are ten main classes:

  * RepRap
  * GCodes
  * Heat
  * Move
  * Platform
  * Network
  * Webserver
  * Roland
  * Scanner, and
  * PrintMonitor

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

Network:

This class implements a basic TCP interface for the Webserver classes using lwip.

Webserver:

This class talks to the network (via Platform) and implements a simple webserver to give an interactive
interface to the RepRap machine.  It uses the Knockout and Jquery Javascript libraries to achieve this.
In addition, FTP and Telnet servers are provided for easier SD card file management and G-Code handling.

Roland:

This class can interface with a Roland mill (e.g. Roland MDX-20/15) and allows the underlying hardware
to act as a G-Code proxy, which translates G-Codes to internal Roland commands.

Scanner:
This is an extension meant for 3D scanner boards. Refer to M750 ff. for the exact usage of this module.

PrintMonitor:

This class provides methods to obtain statistics (height, filament usage etc.) from generated G-Code
files and to calculate estimated print end-times for a live print.


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

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "Platform.h"
#include "RepRap.h"

#ifdef RTOS
# include "FreeRTOS.h"
# include "task.h"
#endif

// We just need one instance of RepRap; everything else is contained within it and hidden

RepRap reprap;

const char * const moduleName[] =
{
	"Platform",
	"Network",
	"Webserver",
	"GCodes",
	"Move",
	"Heat",
	"DDA",
	"Roland",
	"Scanner",
	"PrintMonitor",
	"Storage",
	"PortControl",
	"DuetExpansion",
	"FilamentSensors",
	"WiFi",
	"Display",
	"none"
};

// class MillisTimer members

// Start or restart the timer
void MillisTimer::Start()
{
	whenStarted = millis();
	running = true;
}

// Check whether the timer is running and a timeout has expired, but don't stop it
bool MillisTimer::Check(uint32_t timeoutMillis) const
{
	return running && millis() - whenStarted >= timeoutMillis;
}

// Check whether a timeout has expired and stop the timer if it has, else leave it running if it was running
bool MillisTimer::CheckAndStop(uint32_t timeoutMillis)
{
	const bool ret = Check(timeoutMillis);
	if (ret)
	{
		running = false;
	}
	return ret;
}

//*************************************************************************************************

// Utilities and storage not part of any class

// For debug use
void debugPrintf(const char* fmt, ...)
{
	// Calls to debugPrintf() from with ISRs are unsafe, both because of timing issues and because the call to Platform::MessageF tries to acquire a mutex.
	// So ignore the call if we are coming from within an ISR.
	if (!inInterrupt())
	{
		va_list vargs;
		va_start(vargs, fmt);
		reprap.GetPlatform().MessageF(DebugMessage, fmt, vargs);
		va_end(vargs);
	}
}

#ifdef RTOS

void delay(uint32_t ms)
{
	vTaskDelay(ms);
}

#endif

// Convert a float to double for passing to printf etc. If it is a NaN or infinity, convert it to 9999.9 to avoid getting JSON parse errors.
double HideNan(float val)
{
	return (double)((std::isnan(val) || std::isinf(val)) ? 9999.9 : val);
}

// Append a list of driver numbers to a string, with a space before each one
void ListDrivers(const StringRef& str, DriversBitmap drivers)
{
	for (unsigned int d = 0; drivers != 0; ++d)
	{
		if ((drivers & 1) != 0)
		{
			str.catf(" %u", d);
		}
		drivers >>= 1;
	}
}

// Convert a PWM that is possibly in the old style 0..255 to be in the range 0.0..1.0
float ConvertOldStylePwm(float v)
{
	if (v > 1.0)
	{
		v = v/255.0;
	}
	return constrain<float>(v, 0.0, 1.0);
}

// End
