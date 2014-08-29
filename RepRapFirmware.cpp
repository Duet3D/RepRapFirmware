
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

There are seven main classes:

  * RepRap
  * GCodes
  * Heat
  * Move
  * Platform
  * Network, and
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

Network:

This class implements a basic TCP interface for the Webserver classes using lwip.

Webserver:

This class talks to the network (via Platform) and implements a simple webserver to give an interactive
interface to the RepRap machine.  It uses the Knockout and Jquery Javascript libraries to achieve this.
In addition, FTP and Telnet servers are provided for easier SD card file management and G-Code handling.



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

RepRap::RepRap() : active(false), debug(false), stopped(false), spinState(0), ticksInSpinState(0), resetting(false)
{
  platform = new Platform();
  network = new Network();
  webserver = new Webserver(platform);
  gCodes = new GCodes(platform, webserver);
  move = new Move(platform, gCodes);
  heat = new Heat(platform, gCodes);
  toolList = NULL;
}

void RepRap::Init()
{
  debug = false;
  activeExtruders = 1;		// we always report at least 1 extruder to the web interface
  activeHeaters = 2;		// we always report the bed heater + 1 extruder heater to the web interface

  // All of the following init functions must execute reasonably quickly before the watchdog times us out
  platform->Init();
  gCodes->Init();
  webserver->Init();
  move->Init();
  heat->Init();
  currentTool = NULL;
  const uint32_t wdtTicks = 256;	// number of watchdog ticks @ 32768Hz/128 before the watchdog times out (max 4095)
  WDT_Enable(WDT, (wdtTicks << WDT_MR_WDV_Pos) | (wdtTicks << WDT_MR_WDD_Pos) | WDT_MR_WDRSTEN);	// enable watchdog, reset the mcu if it times out
  coldExtrude = true;		// DC42 changed default to true for compatibility because for now we are aiming for compatibility with RRP 0.78
  active = true;			// must do this before we start the network, else the watchdog may time out

  platform->Message(HOST_MESSAGE, "%s Version %s dated %s\n", NAME, VERSION, DATE);

  platform->Message(HOST_MESSAGE, ".\n\nExecuting ");
  platform->Message(HOST_MESSAGE, platform->GetConfigFile());
  platform->Message(HOST_MESSAGE, "...\n\n");

  // We inject an M98 into the serial input stream to run the start-up macro

  scratchString.printf("M98 P%s\n", platform->GetConfigFile());
  platform->GetLine()->InjectString(scratchString.Pointer());

  bool runningTheFile = false;
  bool initialisingInProgress = true;
  while(initialisingInProgress)
  {
	  Spin();
	  if(gCodes->PrintingAFile())
	  {
		  runningTheFile = true;
	  }
	  if(runningTheFile)
	  {
		  if(!gCodes->PrintingAFile())
		  {
			  initialisingInProgress = false;
		  }
	  }
  }

  platform->Message(HOST_MESSAGE, "\nStarting network...\n");
  network->Init(); // Need to do this here, as the configuration GCodes may set IP address etc.

  platform->Message(HOST_MESSAGE, "\n%s is up and running.\n", NAME);
  fastLoop = FLT_MAX;
  slowLoop = 0.0;
  lastTime = platform->Time();
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

	spinState = 1;
	ticksInSpinState = 0;
	platform->Spin();

	++spinState;
	ticksInSpinState = 0;
	network->Spin();

	++spinState;
	ticksInSpinState = 0;
	webserver->Spin();

	++spinState;
	ticksInSpinState = 0;
	gCodes->Spin();

	++spinState;
	ticksInSpinState = 0;
	move->Spin();

	++spinState;
	ticksInSpinState = 0;
	heat->Spin();

	spinState = 0;
	ticksInSpinState = 0;

	// Keep track of the loop time

	double t = platform->Time();
	double dt = t - lastTime;
	if(dt < fastLoop)
	{
		fastLoop = dt;
	}
	if(dt > slowLoop)
	{
		slowLoop = dt;
	}
	lastTime = t;
}

void RepRap::Timing()
{
	platform->AppendMessage(BOTH_MESSAGE, "Slowest main loop (seconds): %f; fastest: %f\n", slowLoop, fastLoop);
	fastLoop = FLT_MAX;
	slowLoop = 0.0;
}

void RepRap::Diagnostics()
{
  platform->Diagnostics();				// this includes a call to our Timing() function
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
	stopped = true;
	platform->SetAtxPower(false);		// turn off the ATX power if we can

	//platform->DisableInterrupts();

	Tool* tool = toolList;
	while(tool)
	{
		tool->Standby();
		tool = tool->Next();
	}

	heat->Exit();
	for(int8_t heater = 0; heater < HEATERS; heater++)
	{
		platform->SetHeater(heater, 0.0);
	}

	// We do this twice, to avoid an interrupt switching
	// a drive back on.  move->Exit() should prevent
	// interrupts doing this.

	for(int8_t i = 0; i < 2; i++)
	{
		move->Exit();
		for(int8_t drive = 0; drive < DRIVES; drive++)
		{
			platform->SetMotorCurrent(drive, 0.0);
			platform->Disable(drive);
		}
	}

	platform->Message(BOTH_MESSAGE, "Emergency Stop! Reset the controller to continue.");
}

/*
 * The first tool added becomes the one selected.  This will not happen in future releases.
 */

void RepRap::AddTool(Tool* tool)
{
	if(toolList == NULL)
	{
		toolList = tool;
		currentTool = tool;
		tool->Activate(currentTool);
		return;
	}

	toolList->AddTool(tool);
	tool->UpdateExtruderAndHeaterCount(activeExtruders, activeHeaters);
}

void RepRap::SelectTool(int toolNumber)
{
	Tool* tool = toolList;

	while(tool)
	{
		if(tool->Number() == toolNumber)
		{
			tool->Activate(currentTool);
			currentTool = tool;
			return;
		}
		tool = tool->Next();
	}

	// Selecting a non-existent tool is valid.  It sets them all to standby.

	if(currentTool != NULL)
	{
		StandbyTool(currentTool->Number());
	}
	currentTool = NULL;

}

void RepRap::PrintTool(int toolNumber, StringRef& reply) const
{
	for(Tool *tool = toolList; tool != NULL; tool = tool->next)
	{
		if(tool->Number() == toolNumber)
		{
			tool->Print(reply);
			return;
		}
	}
	reply.copy("Attempt to print details of non-existent tool.");
}

void RepRap::StandbyTool(int toolNumber)
{
	Tool* tool = toolList;

	while(tool)
	{
		if(tool->Number() == toolNumber)
		{
			tool->Standby();
			if(currentTool == tool)
			{
				currentTool = NULL;
			}
			return;
		}
		tool = tool->Next();
	}

	platform->Message(HOST_MESSAGE, "Attempt to standby a non-existent tool: %d.\n", toolNumber);
}

Tool* RepRap::GetTool(int toolNumber)
{
	Tool* tool = toolList;

	while(tool)
	{
		if(tool->Number() == toolNumber)
			return tool;
		tool = tool->Next();
	}
	return NULL; // Not an error
}

void RepRap::SetToolVariables(int toolNumber, float* standbyTemperatures, float* activeTemperatures)
{
	Tool* tool = toolList;

	while(tool)
	{
		if(tool->Number() == toolNumber)
		{
			tool->SetVariables(standbyTemperatures, activeTemperatures);
			return;
		}
		tool = tool->Next();
	}

	platform->Message(HOST_MESSAGE, "Attempt to set variables for a non-existent tool: %d.\n", toolNumber);
}


void RepRap::Tick()
{
	if (active)
	{
		WDT_Restart(WDT);			// kick the watchdog
		if (!resetting)
		{
			platform->Tick();
			++ticksInSpinState;
			if (ticksInSpinState >= 20000)	// if we stall for 20 seconds, save diagnostic data and reset
			{
				resetting = true;
				for(uint8_t i = 0; i < HEATERS; i++)
				{
					platform->SetHeater(i, 0.0);
				}
				for(uint8_t i = 0; i < DRIVES; i++)
				{
					platform->Disable(i);
					// We can't set motor currents to 0 here because that requires interrupts to be working, and we are in an ISR
				}

				platform->SoftwareReset(SoftwareResetReason::stuckInSpin + spinState);
			}
		}
	}
}


//*************************************************************************************************
// StringRef class member implementations

size_t StringRef::strlen() const
{
	return strnlen(p, len - 1);
}

int StringRef::printf(const char *fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	int ret = vsnprintf(p, len, fmt, vargs);
	va_end(vargs);
	return ret;
}

int StringRef::vprintf(const char *fmt, va_list vargs)
{
	return vsnprintf(p, len, fmt, vargs);
}

int StringRef::catf(const char *fmt, ...)
{
	size_t n = strlen();
	if (n + 1 < len)		// if room for at least 1 more character and a null
	{
		va_list vargs;
		va_start(vargs, fmt);
		int ret = vsnprintf(p + n, len - n, fmt, vargs);
		va_end(vargs);
		return ret + n;
	}
	return 0;
}

// This is quicker than printf for printing constant strings
size_t StringRef::copy(const char* src)
{
	size_t length = strnlen(src, len - 1);
	memcpy(p, src, length);
	p[length] = 0;
	return length;
}

// This is quicker than catf for printing constant strings
size_t StringRef::cat(const char* src)
{
	size_t length = strlen();
	size_t toCopy = strnlen(src, len - length - 1);
	memcpy(p + length, src, toCopy);
	length += toCopy;
	p[length] = 0;
	return length;
}

//*************************************************************************************************

// Utilities and storage not part of any class

static char scratchStringBuffer[200];		// this is now used only for short messages
StringRef scratchString(scratchStringBuffer, ARRAY_SIZE(scratchStringBuffer));

// For debug use
void debugPrintf(const char* fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	scratchString.vprintf(fmt, vargs);
	va_end(vargs);
	reprap.GetPlatform()->Message(DEBUG_MESSAGE, scratchString);
}

// String testing

bool StringEndsWith(const char* string, const char* ending)
{
  int j = strlen(string);
  int k = strlen(ending);
  if(k > j)
    return false;

  return(StringEquals(&string[j - k], ending));
}

bool StringEquals(const char* s1, const char* s2)
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

bool StringStartsWith(const char* string, const char* starting)
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

int StringContains(const char* string, const char* match)
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
    {
      count = 0;
    }
  }

  return -1;
}











