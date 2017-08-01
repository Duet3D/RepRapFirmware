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
#include <climits>		// for CHAR_BIT

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
	moduleScanner = 8,
	modulePrintMonitor = 9,
	moduleStorage = 10,
	modulePortControl = 11,
	moduleDuetExpansion = 12,
	numModules = 13,				// make this one greater than the last module number
	noModule = 15
};

extern const char *moduleName[];

// Warn of what's to come, so we can use pointers to classes without including the entire header files
class Network;
class Platform;
class GCodes;
class Move;
class DDA;
class Heat;
class Tool;
class Roland;
class Scanner;
class PrintMonitor;
class RepRap;
class FileStore;
class OutputBuffer;
class OutputStack;
class GCodeBuffer;
class GCodeQueue;
class FilamentSensor;
class RandomProbePointSet;

#if SUPPORT_IOBITS
class PortControl;
#endif

// Define floating point type to use for calculations where we would like high precision in matrix calculations
#ifdef DUET_NG
typedef double floatc_t;					// type of matrix element used for calibration
#else
// We are more memory-constrained on the SAM3X
typedef float floatc_t;						// type of matrix element used for calibration
#endif

typedef uint32_t AxesBitmap;				// Type of a bitmap representing a set of axes
typedef uint32_t FansBitmap;				// Type of a bitmap representing a set of fan numbers

// A single instance of the RepRap class contains all the others
extern RepRap reprap;

// Functions and globals not part of any class
extern "C" void debugPrintf(const char* fmt, ...);

bool StringEndsWith(const char* string, const char* ending);
bool StringStartsWith(const char* string, const char* starting);
bool StringEquals(const char* s1, const char* s2);
int StringContains(const char* string, const char* match);
void SafeStrncpy(char *dst, const char *src, size_t length) pre(length != 0);
void SafeStrncat(char *dst, const char *src, size_t length) pre(length != 0);

// Macro to assign an array from an initialiser list
#define ARRAY_INIT(_dest, _init) static_assert(sizeof(_dest) == sizeof(_init), "Incompatible array types"); memcpy(_dest, _init, sizeof(_init));

// Classes to facilitate range-based for loops that iterate from 0 up to just below a limit
template<class T> class SimpleRangeIterator
{
public:
	SimpleRangeIterator(T value_) : val(value_) {}
    bool operator != (SimpleRangeIterator<T> const& other) const { return val != other.val;     }
    T const& operator*() const { return val; }
    SimpleRangeIterator& operator++() { ++val; return *this; }

private:
    T val;
};

template<class T> class SimpleRange
{
public:
	SimpleRange(T limit) : _end(limit) {}
	SimpleRangeIterator<T> begin() const { return SimpleRangeIterator<T>(0); }
	SimpleRangeIterator<T> end() const { return SimpleRangeIterator<T>(_end); 	}

private:
	const T _end;
};

// Helper functions to work on bitmaps of various lengths.
// The primary purpose of these is to allow us to switch between 16, 32 and 64-bit bitmaps.

// Convert an unsigned integer to a bit in a bitmap
template<typename BitmapType> inline constexpr BitmapType MakeBitmap(unsigned int n)
{
	return (BitmapType)1u << n;
}

// Make a bitmap with the lowest n bits set
template<typename BitmapType> inline constexpr BitmapType LowestNBits(unsigned int n)
{
	return ((BitmapType)1u << n) - 1;
}

// Check if a particular bit is set in a bitmap
template<typename BitmapType> inline constexpr bool IsBitSet(BitmapType b, unsigned int n)
{
	return (b & ((BitmapType)1u << n)) != 0;
}

// Set a bit in a bitmap
template<typename BitmapType> inline void SetBit(BitmapType &b, unsigned int n)
{
	b |= ((BitmapType)1u << n);
}

// Clear a bit in a bitmap
template<typename BitmapType> inline void ClearBit(BitmapType &b, unsigned int n)
{
	b &= ~((BitmapType)1u << n);
}

// Convert an array of longs to a bit map with overflow checking
template<typename BitmapType> BitmapType LongArrayToBitMap(const long *arr, size_t numEntries)
{
	BitmapType res = 0;
	for (size_t i = 0; i < numEntries; ++i)
	{
		const long f = arr[i];
		if (f >= 0 && f < sizeof(BitmapType) * CHAR_BIT)
		{
			SetBit(res, (unsigned int)f);
		}
	}
	return res;
}

// Macro to create a SimpleRange from an array
#define ARRAY_INDICES(_arr) (SimpleRange<size_t>(ARRAY_SIZE(_arr)))

// A string buffer used for temporary purposes
extern StringRef scratchString;

// Common definitions used by more than one module
const size_t XYZ_AXES = 3;										// The number of Cartesian axes
const size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E0_AXIS = 3;	// The indices of the Cartesian axes in drive arrays

// Common conversion factors
const float MinutesToSeconds = 60.0;
const float SecondsToMinutes = 1.0/MinutesToSeconds;
const float SecondsToMillis = 1000.0;
const float MillisToSeconds = 0.001;
const float InchToMm = 25.4;
const float DegreesToRadians = PI/180.0;
const float RadiansToDegrees = 180.0/PI;

#define DEGREE_SYMBOL	"\xC2\xB0"								// degree-symbol encoding in UTF8

// Type of an offset in a file
typedef uint32_t FilePosition;
const FilePosition noFilePosition = 0xFFFFFFFF;

// Interrupt priorities - must be chosen with care! 0 is the highest priority, 15 is the lowest.
#if SAM4S || SAM4E
const uint32_t NvicPriorityWatchdog = 0;		// watchdog has highest priority (SAM4 only)
#endif

const uint32_t NvicPriorityUart = 1;			// UART is next to avoid character loss
const uint32_t NvicPrioritySystick = 2;			// systick kicks the watchdog and starts the ADC conversions, so must be quite high
const uint32_t NvicPriorityStep = 3;			// step interrupt is next highest, it can preempt most other interrupts

#if !defined(DUET_NG) && !defined(__RADDS__)
const uint32_t NvicPriorityNetworkTick = 4;		// priority for network tick interrupt
const uint32_t NvicPriorityEthernet = 4;		// priority for Ethernet interface
#endif

const uint32_t NvicPrioritySpi = 5;				// SPI used for network transfers on Duet WiFi/Duet vEthernet
const uint32_t NvicPriorityPins = 6;			// priority for GPIO pin interrupts
const uint32_t NvicPriorityTwi = 7;				// TWI used to read endstop and other inputs on the DueXn

#endif
