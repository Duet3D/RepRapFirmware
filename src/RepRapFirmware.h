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
#undef value			// needed because we include <optional>

#include "Core.h"

typedef uint16_t PwmFrequency;		// type used to represent a PWM frequency. 0 sometimes means "default".

#include "Configuration.h"
#include "Pins.h"

#include "General/SafeStrtod.h"
#include "General/SafeVsnprintf.h"
#include "General/StringRef.h"

// Module numbers and names, used for diagnostics and debug
// All of these including noModule must be <= 15 because we 'or' the module number into the software reset code
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
	moduleFilamentSensors = 13,
	moduleWiFi = 14,
	moduleDisplay = 15,
	numModules = 16,				// make this one greater than the last module number
	noModule = 16
};

extern const char * const moduleName[];

// Warn of what's to come, so we can use pointers to classes without including the entire header files
class Network;
class Platform;
class GCodes;
class Move;
class DDA;
class Kinematics;
class Heat;
class PID;
class TemperatureSensor;
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
class FilamentMonitor;
class RandomProbePointSet;
class Logger;

#if SUPPORT_IOBITS
class PortControl;
#endif

#if SUPPORT_12864_LCD
class Display;
#endif

// Define floating point type to use for calculations where we would like high precision in matrix calculations
#if SAM4E || SAM4S || SAME70
typedef double floatc_t;					// type of matrix element used for calibration
#else
// We are more memory-constrained on the SAM3X
typedef float floatc_t;						// type of matrix element used for calibration
#endif

typedef uint32_t AxesBitmap;				// Type of a bitmap representing a set of axes
typedef uint32_t DriversBitmap;				// Type of a bitmap representing a set of driver numbers
typedef uint32_t FansBitmap;				// Type of a bitmap representing a set of fan numbers
typedef uint16_t Pwm_t;						// Type of a PWM value when we don't want to use floats

// Logical pins used for general output, servos, CCN and laser control
typedef uint16_t LogicalPin;				// Type used to represent logical pin numbers
constexpr LogicalPin NoLogicalPin = 0xFFFFu;

#if SUPPORT_IOBITS
typedef uint16_t IoBits_t;					// Type of the port control bitmap (G1 P parameter)
#endif

#if SUPPORT_LASER || SUPPORT_IOBITS
union LaserPwmOrIoBits
{
#if SUPPORT_LASER
	Pwm_t laserPwm;							// the laser PWM to use for this move
#endif
#if SUPPORT_IOBITS
	IoBits_t ioBits;						// I/O bits to set/clear at the start of this move
#endif

	void Clear()							// set to zero, whichever one it is
	{
#if SUPPORT_LASER
		laserPwm = 0;
#else
		ioBits = 0;
#endif
	}
};
#endif

// A single instance of the RepRap class contains all the others
extern RepRap reprap;

// Debugging support
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
//#define DEBUG_HERE do { } while (false)
#define DEBUG_HERE do { debugPrintf("At " __FILE__ " line %d\n", __LINE__); delay(50); } while (false)

// Functions and globals not part of any class

#ifdef RTOS

void delay(uint32_t ms);

#else

inline void delay(uint32_t ms)
{
	coreDelay(ms);
}

#endif

bool StringEndsWithIgnoreCase(const char* string, const char* ending);
bool StringStartsWith(const char* string, const char* starting);
bool StringStartsWithIgnoreCase(const char* string, const char* starting);
bool StringEqualsIgnoreCase(const char* s1, const char* s2);
int StringContains(const char* string, const char* match);
void SafeStrncpy(char *dst, const char *src, size_t length) pre(length != 0);
void SafeStrncat(char *dst, const char *src, size_t length) pre(length != 0);

double HideNan(float val);

void ListDrivers(const StringRef& str, DriversBitmap drivers);

// Macro to assign an array from an initialiser list
#define ARRAY_INIT(_dest, _init) static_assert(sizeof(_dest) == sizeof(_init), "Incompatible array types"); memcpy(_dest, _init, sizeof(_init));

// UTF8 code for the degree-symbol
#define DEGREE_SYMBOL	"\xC2\xB0"	// Unicode degree-symbol as UTF8

// Functions to change the base priority, to shut out interrupts up to a priority level

// From section 3.12.7 of http://infocenter.arm.com/help/topic/com.arm.doc.dui0553b/DUI0553.pdf:
// When you write to BASEPRI_MAX, the instruction writes to BASEPRI only if either:
// • Rn is non-zero and the current BASEPRI value is 0
// • Rn is non-zero and less than the current BASEPRI value
__attribute__( ( always_inline ) ) __STATIC_INLINE void __set_BASEPRI_MAX(uint32_t value)
{
  __ASM volatile ("MSR basepri_max, %0" : : "r" (value) : "memory");
}

// Get the base priority and shut out interrupts lower than or equal to a specified priority
inline uint32_t ChangeBasePriority(uint32_t prio)
{
	const uint32_t oldPrio = __get_BASEPRI();
	__set_BASEPRI_MAX(prio << (8 - __NVIC_PRIO_BITS));
	return oldPrio;
}

// Restore the base priority following a call to ChangeBasePriority
inline void RestoreBasePriority(uint32_t prio)
{
	__set_BASEPRI(prio);
}

// Set the base priority when we are not interested in the existing value i.e. definitely in non-interrupt code
inline void SetBasePriority(uint32_t prio)
{
	__set_BASEPRI(prio << (8 - __NVIC_PRIO_BITS));
}

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

// Macro to create a SimpleRange from an array
#define ARRAY_INDICES(_arr) (SimpleRange<size_t>(ARRAY_SIZE(_arr)))

// A simple milliseconds timer class
class MillisTimer
{
public:
	MillisTimer() { running = false; }
	void Start();
	void Stop() { running = false; }
	bool Check(uint32_t timeoutMillis) const;
	bool CheckAndStop(uint32_t timeoutMillis);
	bool IsRunning() const { return running; }

private:
	uint32_t whenStarted;
	bool running;
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
template<typename BitmapType> BitmapType UnsignedArrayToBitMap(const uint32_t *arr, size_t numEntries)
{
	BitmapType res = 0;
	for (size_t i = 0; i < numEntries; ++i)
	{
		const uint32_t f = arr[i];
		if (f < sizeof(BitmapType) * CHAR_BIT)
		{
			SetBit(res, f);
		}
	}
	return res;
}

// Convert a PWM that is possibly in the old style 0..255 to be in the range 0.0..1.0
float ConvertOldStylePwm(float v);

// Common definitions used by more than one module

constexpr size_t ScratchStringLength = 220;							// standard length of a scratch string, enough to print delta parameters to
constexpr size_t ShortScratchStringLength = 50;

constexpr size_t XYZ_AXES = 3;										// The number of Cartesian axes
constexpr size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, E0_AXIS = 3;	// The indices of the Cartesian axes in drive arrays
constexpr size_t CoreXYU_AXES = 5;									// The number of axes in a CoreXYU machine (there is a hidden V axis)
constexpr size_t CoreXYUV_AXES = 5;									// The number of axes in a CoreXYUV machine
constexpr size_t U_AXIS = 3, V_AXIS = 4;							// The indices of the U and V motors in a CoreXYU machine (needed by Platform)

// Common conversion factors
constexpr float MinutesToSeconds = 60.0;
constexpr float SecondsToMinutes = 1.0/MinutesToSeconds;
constexpr float SecondsToMillis = 1000.0;
constexpr float MillisToSeconds = 0.001;
constexpr float InchToMm = 25.4;
constexpr float Pi = 3.141592653589793;
constexpr float TwoPi = 3.141592653589793 * 2;
constexpr float DegreesToRadians = 3.141592653589793/180.0;
constexpr float RadiansToDegrees = 180.0/3.141592653589793;

#define DEGREE_SYMBOL	"\xC2\xB0"									// degree-symbol encoding in UTF8

// Type of an offset in a file
typedef uint32_t FilePosition;
const FilePosition noFilePosition = 0xFFFFFFFF;

//-------------------------------------------------------------------------------------------------
// Interrupt priorities - must be chosen with care! 0 is the highest priority, 15 is the lowest.
// This interacts with FreeRTOS config constant configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY which is currently defined as 3 for the SAME70 and 5 for the SAM4x.
// ISRs with better (numerically lower) priorities than this value cannot make FreeRTOS calls, but those interrupts wont be disabled even in FreeRTOS critical sections.

#if __NVIC_PRIO_BITS == 3
// We have only 8 interrupt priority levels on the SAME70
// Use priority 2 or lower for interrupts where low latency is critical and FreeRTOS calls are not needed.

const uint32_t NvicPriorityWatchdog = 0;		// the secondary watchdog has the highest priority
const uint32_t NvicPriorityPanelDueUart = 1;	// UART is highest to avoid character loss (it has only a 1-character receive buffer)
const uint32_t NvicPriorityWiFiUart = 2;		// UART used to receive debug data from the WiFi module
const uint32_t NvicPriorityMCan = 2;			// CAN interface

# ifndef RTOS
const uint32_t NvicPrioritySystick = 3;			// systick kicks the watchdog and starts the ADC conversions, so must be quite high
# endif

const uint32_t NvicPriorityPins = 3;			// priority for GPIO pin interrupts - filament sensors must be higher than step
const uint32_t NvicPriorityStep = 4;			// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityUSB = 5;				// USB interrupt
const uint32_t NvicPriorityHSMCI = 5;			// HSMCI command complete interrupt

# if HAS_LWIP_NETWORKING
const uint32_t NvicPriorityNetworkTick = 6;		// priority for network tick interrupt (to be replaced by a FreeRTOS task)
const uint32_t NvicPriorityEthernet = 6;		// priority for Ethernet interface
# endif

const uint32_t NvicPriorityDMA = 6;				// end-of-DMA interrupt used by TMC drivers and HSMCI
const uint32_t NvicPrioritySpi = 6;				// SPI is used for network transfers on Duet WiFi/Duet vEthernet

#elif __NVIC_PRIO_BITS == 4
// We have 16 priority levels
// Use priority 4 or lower for interrupts where low latency is critical and FreeRTOS calls are not needed.

# if SAM4E || __LPC17xx__
const uint32_t NvicPriorityWatchdog = 0;		// the secondary watchdog has the highest priority
# endif

const uint32_t NvicPriorityPanelDueUart = 1;	// UART is highest to avoid character loss (it has only a 1-character receive buffer)
const uint32_t NvicPriorityDriversSerialTMC = 2; // USART or UART used to control and monitor the smart drivers

# ifndef RTOS
const uint32_t NvicPrioritySystick = 3;			// systick kicks the watchdog and starts the ADC conversions, so must be quite high
# endif

const uint32_t NvicPriorityPins = 5;			// priority for GPIO pin interrupts - filament sensors must be higher than step
const uint32_t NvicPriorityStep = 6;			// step interrupt is next highest, it can preempt most other interrupts
const uint32_t NvicPriorityWiFiUart = 7;		// UART used to receive debug data from the WiFi module
const uint32_t NvicPriorityUSB = 7;				// USB interrupt
const uint32_t NvicPriorityHSMCI = 7;			// HSMCI command complete interrupt

# if HAS_LWIP_NETWORKING
const uint32_t NvicPriorityNetworkTick = 8;		// priority for network tick interrupt (to be replaced by a FreeRTOS task)
const uint32_t NvicPriorityEthernet = 8;		// priority for Ethernet interface
# endif

const uint32_t NvicPrioritySpi = 8;				// SPI is used for network transfers on Duet WiFi/Duet vEthernet
const uint32_t NvicPriorityTwi = 9;				// TWI is used to read endstop and other inputs on the DueXn

#endif

#endif
