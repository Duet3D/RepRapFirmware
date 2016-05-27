/****************************************************************************************************

RepRapFirmware - Main Include

This includes all the other include files in the right order and defines some globals.
No other definitions or information should be in here.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef REPRAPFIRMWARE_H
#define REPRAPFIRMWARE_H

#include <cstddef>		// for size_t
#include <cfloat>
#include <cstdarg>

#include "Core.h"
#include "Configuration.h"
#include "StringRef.h"

// Module numbers and names, used for diagnostics and debug
enum Module : uint8_t
{
	modulePlatform = 0,
	moduleNetwork = 1,
	moduleWebserver = 2,
	moduleGcodes = 3,
	moduleMove = 4,
	moduleHeat = 5,
	moduleDda = 6,
	moduleRoland = 7,
	modulePrintMonitor = 8,
	numModules = 9,				// make this one greater than the last module number
	noModule = 15
};

extern const char *moduleName[];

// Warn of what's to come, so we can use pointers to classes...

class Network;
class Platform;
class Webserver;
class GCodes;
class Move;
class Heat;
class Tool;
class Roland;
class PrintMonitor;
class RepRap;
class FileStore;

// A single instance of the RepRap class contains all the others

extern RepRap reprap;

// Functions and globals not part of any class

extern "C" void debugPrintf(const char* fmt, ...);

bool StringEndsWith(const char* string, const char* ending);
bool StringStartsWith(const char* string, const char* starting);
bool StringEquals(const char* s1, const char* s2);
int StringContains(const char* string, const char* match);
  
// Macro to assign an array from an initializer list
#define ARRAY_INIT(_dest, _init) static_assert(sizeof(_dest) == sizeof(_init), "Incompatible array types"); memcpy(_dest, _init, sizeof(_init));

extern StringRef scratchString;

#include "OutputMemory.h"
#include "Network.h"
#include "Platform.h"
#include "Webserver.h"
#include "GCodes.h"
#include "Move.h"
#include "Heat.h"
#include "Tool.h"
#include "Roland.h"
#include "PrintMonitor.h"
#include "Reprap.h"

extern uint32_t isqrt64(uint64_t num);		// This is defined in its own file, Isqrt.cpp or Isqrt.asm

#endif


