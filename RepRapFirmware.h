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

#include "Arduino.h"
#include "Configuration.h"
#include "StringRef.h"

// Module numbers and names, used for diagnostics and debug
enum Module
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
  
// Macro to give us the number of elements in an array
#define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))
// Macro to give us the highest valid index into an array i.e. one less than the size
#define ARRAY_UPB(_x)	(ARRAY_SIZE(_x) - 1)
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

// std::min and std::max don't seem to work with this variant of gcc, so define our own ones here
// We use these only with primitive types, so pass them directly instead of by const reference
#undef min
#undef max

template<class X> inline X min(X _a, X _b)
{
	return (_a < _b) ? _a : _b;
}

template<class X> inline X max(X _a, X _b)
{
	return (_a > _b) ? _a : _b;
}

// Specialisations for float and double to handle NANs properly
template<> inline float min(float _a, float _b)
{
	return (isnan(_a) || _a < _b) ? _a : _b;
}

template<> inline float max(float _a, float _b)
{
	return (isnan(_a) || _a > _b) ? _a : _b;
}

template<> inline double min(double _a, double _b)
{
	return (isnan(_a) || _a < _b) ? _a : _b;
}

template<> inline double max(double _a, double _b)
{
	return (isnan(_a) || _a > _b) ? _a : _b;
}

inline float fsquare(float arg)
{
	return arg * arg;
}

inline uint64_t isquare64(int32_t arg)
{
	return (uint64_t)((int64_t)arg * arg);
}

inline uint64_t isquare64(uint32_t arg)
{
	return (uint64_t)arg * arg;
}

inline void swap(float& a, float& b)
{
	float temp = a;
	a = b;
	b = temp;
}

#undef constrain
template<class T> inline float constrain(T val, T vmin, T vmax)
{
	return max<T>(vmin, min<T>(val, vmax));
}

extern uint32_t isqrt64(uint64_t num);		// This is defined in its own file, Isqrt.cpp or Isqrt.asm

#endif


