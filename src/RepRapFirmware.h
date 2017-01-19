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

#include "ecv.h"
#include "Core.h"
#include "Configuration.h"
#include "Pins.h"
#include "Libraries/General/StringRef.h"

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
	moduleStorage = 9,
	numModules = 10,				// make this one greater than the last module number
	noModule = 15
};

extern const char *moduleName[];

// Warn of what's to come, so we can use pointers to classes without including the entire header files
class Network;
class Platform;
class Webserver;
class GCodes;
class Move;
class DDA;
class Heat;
class Tool;
class Roland;
class PrintMonitor;
class RepRap;
class FileStore;
class OutputBuffer;
class OutputStack;

// A single instance of the RepRap class contains all the others
extern RepRap reprap;

// Functions and globals not part of any class
extern "C" void debugPrintf(const char* fmt, ...);

bool StringEndsWith(const char* string, const char* ending);
bool StringStartsWith(const char* string, const char* starting);
bool StringEquals(const char* s1, const char* s2);
int StringContains(const char* string, const char* match);

// Macro to assign an array from an initialiser list
#define ARRAY_INIT(_dest, _init) static_assert(sizeof(_dest) == sizeof(_init), "Incompatible array types"); memcpy(_dest, _init, sizeof(_init));

// A string buffer used for temporary purposes
extern StringRef scratchString;

// Common definitions used by more than one module
const size_t CART_AXES = 3;										// The number of Cartesian axes
const size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E0_AXIS = 3;	// The indices of the Cartesian axes in drive arrays

// Common conversion factors
const float MinutesToSeconds = 60.0;
const float SecondsToMinutes = 1.0/MinutesToSeconds;
const float SecondsToMillis = 1000.0;
const float MillisToSeconds = 0.001;
const float InchToMm = 25.4;
const float DegreesToRadians = PI/180.0;
const float RadiansToDegrees = 180.0/PI;

// Type of an offset in a file
typedef uint32_t FilePosition;
const FilePosition noFilePosition = 0xFFFFFFFF;

#endif
