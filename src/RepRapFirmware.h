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

#include <ecv_duet3d.h>

#include <cstddef>		// for size_t
#include <cfloat>
#include <cstdarg>
#include <climits>		// for CHAR_BIT

#include <ctime>
[[deprecated("use gmtime_r instead for thread-safety")]] tm *_ecv_null gmtime(const time_t* t);
[[deprecated("use SafeStrptime instead")]] char *_ecv_array strptime (const char *_ecv_array buf, const char *_ecv_array format, struct tm *timeptr);
const char *_ecv_array SafeStrptime(const char *_ecv_array buf, const char *_ecv_array format, struct tm *timeptr) noexcept;

#include <Core.h>

#ifndef SAMC21
# error SAMC21 should be defined as 0 or 1
#endif

#ifndef SAME5x
# error SAME5X should be defined as 0 or 1
#endif

#if SAME70
# define __nocache		__attribute__((section(".ram_nocache")))
#else
# define __nocache		// nothing
#endif

#include <CoreIO.h>
#include <Devices.h>

// The following are needed by many other files, so include them here
#include <Platform/MessageType.h>
#include <GCodeResult.h>

// Convert an error or warning result into a suitable generic message type. Should only be called with GCodeResult::warning or GCodeResult::error.
inline MessageType GetGenericMessageType(GCodeResult rslt)
{
	return (rslt == GCodeResult::warning) ? WarningMessage : ErrorMessage;
}

#define SPEED_CRITICAL	__attribute__((optimize("O2")))

// API level definition.
// ApiLevel 1 is the first level that supports rr_model.
constexpr unsigned int ApiLevel = 1;

// Definitions needed by Pins.h and/or Configuration.h
// Logical pins used for general output, servos, CCN and laser control
typedef uint8_t LogicalPin;				// type used to represent logical pin numbers
constexpr LogicalPin NoLogicalPin = 0xFF;
constexpr const char * _ecv_array NoPinName = "nil";

// Enumeration to describe what we want to do with a pin
enum class PinAccess : int
{
	read,
	readWithPullup_InternalUseOnly,
	readAnalog,
	write0,
	write1,
	pwm,
	servo
};

enum class PinUsedBy : uint8_t
{
	unused = 0,
	heater,
	fan,
	endstop,
	zprobe,
	tacho,
	spindle,
	laser,
	gpin,
	gpout,
	filamentMonitor,
	temporaryInput,
	sensor,
	sdCard
};

#include <Config/Pins.h>
#include <Config/Configuration.h>

static_assert(MinVisibleAxes <= MinAxes);
static_assert(NumNamedPins <= 255 || sizeof(LogicalPin) > 1, "Need 16-bit logical pin numbers");

#if SUPPORT_CAN_EXPANSION

# include <CanId.h>

// We have to declare CanInterface::GetCanAddress here because CanInterface.h needs to include this file for the declaration of DriverId
namespace CanInterface
{
	CanAddress GetCanAddress() noexcept;
}

#else

namespace CanInterface
{
	inline CanAddress GetCanAddress() noexcept { return 0; }
}

#endif

#include <General/String.h>
#include <General/StringFunctions.h>
#include <General/Bitmap.h>
#include <General/SafeStrtod.h>
#include <General/SafeVsnprintf.h>
#include <RRF3Common.h>

#define THROWS(...)				// expands to nothing, for providing exception specifications
#define THROW_INTERNAL_ERROR	throw GCodeException(-1, -1, "internal error at file " __FILE__ "(%d)", (int32_t)__LINE__)

// Assertion mechanism
extern "C" [[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept __attribute__((naked));
#define RRF_ASSERT(_expr) do { if (!(_expr)) { vAssertCalled(__LINE__, __FILE__); } } while (false)

// Type of a driver identifier
struct DriverId
{
	uint8_t localDriver;

#if SUPPORT_CAN_EXPANSION

	CanAddress boardAddress;

	DriverId() noexcept : localDriver(0), boardAddress(CanInterface::GetCanAddress())  { }

	// Constructor used by ATE configurations and object model
	DriverId(CanAddress addr, uint8_t drv) noexcept : localDriver(drv), boardAddress(addr) { }

	void SetFromBinary(uint32_t val) noexcept
	{
		localDriver = val & 0x000000FF;
		const uint32_t brdNum = val >> 16;
		boardAddress = (brdNum <= CanId::MaxCanAddress) ? (CanAddress)brdNum : CanId::NoAddress;
	}

	void SetLocal(unsigned int driver) noexcept
	{
		localDriver = (uint8_t)driver;
		boardAddress = CanInterface::GetCanAddress();
	}

	bool IsLocal() const noexcept { return boardAddress == CanInterface::GetCanAddress(); }
	bool IsRemote() const noexcept { return boardAddress != CanInterface::GetCanAddress(); }

	bool operator<(const DriverId other) const noexcept
	{
		return boardAddress < other.boardAddress || (boardAddress == other.boardAddress && localDriver < other.localDriver);
	}

	bool operator==(const DriverId other) const noexcept
	{
		return boardAddress == other.boardAddress && localDriver == other.localDriver;
	}

	bool operator!=(const DriverId other) const noexcept
	{
		return boardAddress != other.boardAddress || localDriver != other.localDriver;
	}

	uint32_t AsU32() const noexcept
	{
		return (boardAddress << 8) | localDriver;
	}

#else

	DriverId() noexcept : localDriver(0)  { }

	// Constructor used by object model
	explicit DriverId(uint8_t drv) noexcept : localDriver(drv) { }

	// Set the driver ID from the binary value, returning true if there was a nonzero board number so that the caller knows the address is not valid
	bool SetFromBinary(uint32_t val) noexcept
	{
		localDriver = val & 0x000000FFu;
		const uint32_t brdNum = val >> 16;
		return (brdNum != 0);
	}

	void SetLocal(unsigned int driver) noexcept
	{
		localDriver = (uint8_t)driver;
	}

	bool operator==(const DriverId other) const noexcept
	{
		return localDriver == other.localDriver;
	}

	bool operator!=(const DriverId other) const noexcept
	{
		return localDriver != other.localDriver;
	}

	bool IsLocal() const noexcept { return true; }
	bool IsRemote() const noexcept { return false; }

	uint32_t AsU32() const noexcept
	{
		return localDriver;
	}

#endif
};

#if SUPPORT_CAN_EXPANSION
# define PRIdriverId				"%u.%u"
# define DRIVER_ID_PRINT_ARGS(_d)	_d.boardAddress,_d.localDriver
#else
# define PRIdriverId				"%u"
# define DRIVER_ID_PRINT_ARGS(_d)	_d.localDriver
#endif

// Module numbers and names, used for diagnostics and debug
// All of these including noModule must be <= 31 because we 'or' the module number into the software reset code
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
	moduleSbcInterface = 16,
	moduleCan = 17,
	numModules = 18,				// make this one greater than the last real module number
	noModule = numModules
};

const char *_ecv_array GetModuleName(uint8_t module) noexcept;

// Warn of what's to come, so we can use pointers and references to classes without including the entire header files
class Network;
class Platform;
class GCodes;
class Move;
class DDA;
class Kinematics;
class Heat;
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
class FansManager;

#if SUPPORT_IOBITS
class PortControl;
#endif

#if SUPPORT_12864_LCD
class Display;
#endif

#if HAS_SBC_INTERFACE
class SbcInterface;
#endif

#if SUPPORT_CAN_EXPANSION
class ExpansionManager;
#endif

// Define floating point type to use for calculations where we would like high precision in matrix calculations
#if SAME70
typedef double floatc_t;						// type of matrix element used for calibration
#else
// We are more memory-constrained on the older processors
typedef float floatc_t;							// type of matrix element used for calibration
#endif

#ifdef DUET3
typedef Bitmap<uint32_t> AxesBitmap;			// Type of a bitmap representing a set of axes, and sometimes extruders too
#else
typedef Bitmap<uint16_t> AxesBitmap;			// Type of a bitmap representing a set of axes, and sometimes extruders too
#endif
typedef Bitmap<uint32_t> ExtrudersBitmap;		// Type of a bitmap representing a set of extruder drive numbers
typedef Bitmap<uint32_t> DriversBitmap;			// Type of a bitmap representing a set of local driver numbers
typedef Bitmap<uint32_t> FansBitmap;			// Type of a bitmap representing a set of fan numbers
typedef Bitmap<uint32_t> HeatersBitmap;			// Type of a bitmap representing a set of heater numbers
typedef Bitmap<uint16_t> DriverChannelsBitmap;	// Type of a bitmap representing a set of drivers that typically have a common cooling fan
typedef Bitmap<uint32_t> InputPortsBitmap;		// Type of a bitmap representing a set of input ports
typedef Bitmap<uint32_t> TriggerNumbersBitmap;	// Type of a bitmap representing a set of trigger numbers

#if defined(DUET3) || defined(DUET3MINI)
typedef Bitmap<uint64_t> SensorsBitmap;
#else
typedef Bitmap<uint32_t> SensorsBitmap;
#endif

static_assert(MaxAxesPlusExtruders <= AxesBitmap::MaxBits());
static_assert(MaxExtruders <= ExtrudersBitmap::MaxBits());
static_assert(MaxFans <= FansBitmap::MaxBits());
static_assert(MaxHeaters <= HeatersBitmap::MaxBits());
static_assert(NumDirectDrivers <= DriversBitmap::MaxBits());
static_assert(MaxSensors <= SensorsBitmap::MaxBits());
static_assert(MaxGpInPorts <= InputPortsBitmap::MaxBits());
static_assert(MaxTriggers <= TriggerNumbersBitmap::MaxBits());

typedef uint16_t Pwm_t;							// Type of a PWM value when we don't want to use floats

#if SUPPORT_IOBITS
typedef uint16_t IoBits_t;						// Type of the port control bitmap (G1 P parameter)
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

// Debugging support
extern "C" void debugPrintf(const char* fmt, ...) noexcept __attribute__ ((format (printf, 1, 2)));
#define DEBUG_HERE do { debugPrintf("At " __FILE__ " line %d\n", __LINE__); delay(50); } while (false)

// Functions and globals not part of any class

float HideNan(float val) noexcept;

void ListDrivers(const StringRef& str, DriversBitmap drivers) noexcept;

// Macro to assign an array from an initialiser list
#define ARRAY_INIT(_dest, _init) static_assert(sizeof(_dest) == sizeof(_init), "Incompatible array types"); memcpy(_dest, _init, sizeof(_init));

// UTF8 code for the degree-symbol
#define DEGREE_SYMBOL	"\xC2\xB0"	// Unicode degree-symbol as UTF8

// Classes to facilitate range-based for loops that iterate from 0 up to just below a limit
template<class T> class SimpleRangeIterator
{
public:
	explicit SimpleRangeIterator(T value_) noexcept : val(value_) {}
    bool operator != (SimpleRangeIterator<T> const& other) const noexcept { return val != other.val;     }
    T const& operator*() const noexcept { return val; }
    SimpleRangeIterator<T>& operator++() noexcept { ++val; return *this; }

private:
    T val;
};

template<class T> class SimpleRange
{
public:
	explicit SimpleRange(T limit) noexcept : _end(limit) {}
	SimpleRangeIterator<T> begin() const noexcept { return SimpleRangeIterator<T>(0); }
	SimpleRangeIterator<T> end() const noexcept { return SimpleRangeIterator<T>(_end); 	}

private:
	const T _end;
};

// Macro to create a SimpleRange from an array
#define ARRAY_INDICES(_arr) (SimpleRange<size_t>(ARRAY_SIZE(_arr)))

// A simple milliseconds timer class
class MillisTimer
{
public:
	MillisTimer() noexcept { running = false; }
	void Start() noexcept;
	void Stop() noexcept { running = false; }
	bool Check(uint32_t timeoutMillis) const noexcept;
	bool CheckAndStop(uint32_t timeoutMillis) noexcept;
	bool IsRunning() const noexcept { return running; }

private:
	uint32_t whenStarted;
	bool running;
};

// Function to delete an object and clear the pointer. Safe to call even if the pointer is already null.
template <typename T> void DeleteObject(T *null & ptr) noexcept
{
	T *null p2 = nullptr;
	std::swap(ptr, p2);
	delete p2;
}

// Function to make a pointer point to a new object and delete the existing object, if any. T2 must be the same as T or derived from it.
template <typename T, typename T2> void ReplaceObject(T *null & ptr, T2* pNew) noexcept
{
	T *null p2 = static_cast<T *null>(pNew);
	std::swap(ptr, p2);
	delete p2;
}

// Common definitions used by more than one module

constexpr size_t XY_AXES = 2;										// The number of Cartesian axes
constexpr size_t XYZ_AXES = 3;										// The number of Cartesian axes
constexpr size_t X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2;				// The indices of the Cartesian axes in drive arrays
constexpr size_t U_AXIS = 3;										// The assumed index of the U axis when executing M673
constexpr size_t NO_AXIS = 0x3F;									// A value to represent no axis, must fit in 6 bits (see EndstopHitDetails and RemoteInputHandle) and not be a valid axis number

static_assert(MaxAxesPlusExtruders <= MaxAxes + MaxExtruders);

#if SUPPORT_CAN_EXPANSION
constexpr size_t MaxTotalDrivers = NumDirectDrivers + MaxCanDrivers;
#else
constexpr size_t MaxTotalDrivers = NumDirectDrivers;
#endif

// Convert between extruder drive numbers and logical drive numbers.
// In order to save memory when MaxAxesPlusExtruders < MaxAxes + MaxExtruders, the logical drive number of an axis is the same as the axis number,
// but the logical drive number of an extruder is MaxAxesPlusExtruders - 1 - extruder_number.
inline size_t ExtruderToLogicalDrive(size_t extruder) noexcept { return MaxAxesPlusExtruders - 1 - extruder; }
inline size_t LogicalDriveToExtruder(size_t drive) noexcept { return MaxAxesPlusExtruders - 1 - drive; }

const AxesBitmap DefaultXAxisMapping = AxesBitmap::MakeFromBits(X_AXIS);	// by default, X is mapped to X
const AxesBitmap DefaultYAxisMapping = AxesBitmap::MakeFromBits(Y_AXIS);	// by default, Y is mapped to Y
const AxesBitmap XyzAxes = AxesBitmap::MakeLowestNBits(XYZ_AXES);
const AxesBitmap XyAxes = AxesBitmap::MakeLowestNBits(XY_AXES);

// Common conversion factors
constexpr float MinutesToSeconds = 60.0;
constexpr uint32_t iMinutesToSeconds = 60;
constexpr float SecondsToMinutes = 1.0/MinutesToSeconds;
constexpr float SecondsToMillis = 1000.0;
constexpr float MillisToSeconds = 0.001;
constexpr float InchToMm = 25.4;
constexpr float Pi = 3.141592653589793;
constexpr float TwoPi = 3.141592653589793 * 2.0;
constexpr float DegreesToRadians = 3.141592653589793/180.0;
constexpr float RadiansToDegrees = 180.0/3.141592653589793;

// The step clock is used for timing step pulses and oyther fine-resolution timer purposes

#if SAME70 || SAME5x
// All Duet 3 boards use a common step clock rate of 750kHz so that we can sync the clocks over CAN
constexpr uint32_t StepClockRate = 48000000/64;								// 750kHz
#elif defined(__LPC17xx__)
constexpr uint32_t StepClockRate = 1000000;									// 1MHz
#else
constexpr uint32_t StepClockRate = SystemCoreClockFreq/128;					// Duet 2 and Maestro: use just under 1MHz
#endif

constexpr uint64_t StepClockRateSquared = (uint64_t)StepClockRate * StepClockRate;
constexpr float StepClocksToMillis = 1000.0/(float)StepClockRate;

// Functions to convert speeds and accelerations between seconds and step clocks
static inline constexpr float ConvertSpeedFromMmPerSec(float speed) noexcept
{
	return speed * 1.0/(float)StepClockRate;
}

static inline constexpr float ConvertSpeedFromMmPerMin(float speed) noexcept
{
	return speed * (1.0/(float)(StepClockRate * iMinutesToSeconds));
}

static inline constexpr float ConvertSpeedFromMm(float speed, bool useSeconds) noexcept
{
	return speed * ((useSeconds) ? 1.0/(float)StepClockRate : 1.0/(float)(StepClockRate * iMinutesToSeconds));
}

static inline constexpr float InverseConvertSpeedToMmPerSec(float speed) noexcept
{
	return speed * (float)StepClockRate;
}

static inline constexpr float InverseConvertSpeedToMmPerMin(float speed) noexcept
{
	return speed * (float)(StepClockRate * iMinutesToSeconds);
}

static inline constexpr float InverseConvertSpeedToMm(float speed, bool useSeconds) noexcept
{
	return speed * (float)((useSeconds) ? StepClockRate : StepClockRate * iMinutesToSeconds);
}

static inline constexpr float ConvertAcceleration(float accel) noexcept
{
	return accel * (1.0/(float)StepClockRateSquared);
}

static inline constexpr float InverseConvertAcceleration(float accel) noexcept
{
	return accel * (float)StepClockRateSquared;
}

constexpr unsigned int MaxFloatDigitsDisplayedAfterPoint = 7;
const char *_ecv_array GetFloatFormatString(float val, unsigned int numDigitsAfterPoint) noexcept;

#if SUPPORT_WORKPLACE_COORDINATES
constexpr size_t NumCoordinateSystems = 9;							// G54 up to G59.3
#else
constexpr size_t NumCoordinateSystems = 1;
#endif

#define DEGREE_SYMBOL	"\xC2\xB0"									// degree-symbol encoding in UTF8

#if HAS_SBC_INTERFACE
typedef uint32_t FileHandle;
const FileHandle noFileHandle = 0;
#endif

// Type of an offset in a file
typedef uint32_t FilePosition;
const FilePosition noFilePosition = 0xFFFFFFFFu;

//-------------------------------------------------------------------------------------------------
// Interrupt priorities - must be chosen with care! 0 is the highest priority, 7 or 15 is the lowest.
// This interacts with FreeRTOS config constant configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY which is currently defined as 3 for the SAME70 and 5 for the SAM4x.
// ISRs with better (numerically lower) priorities than this value cannot make FreeRTOS calls, but those interrupts wont be disabled even in FreeRTOS critical sections.

#if __NVIC_PRIO_BITS == 3
// We have only 8 interrupt priority levels on the SAME70 and SAME5x
// Use priority 2 or lower for interrupts where low latency is critical and FreeRTOS calls are not needed.

const uint32_t NvicPriorityWatchdog = 0;			// the secondary watchdog has the highest priority

#if SAME5x
const NvicPriority NvicPriorityAuxUartRx = 1;		// UART used to receive data from PanelDue or other serial input
const NvicPriority NvicPriorityAuxUartTx = 3;		// the SAME5x driver makes FreeRTOS calls during transmission, so use a lower priority
const NvicPriority NvicPriorityWiFiUartRx = 2;		// UART used to receive debug data from the WiFi module
const NvicPriority NvicPriorityWiFiUartTx = 3;		// the SAME5x driver makes FreeRTOS calls during transmission, so use a lower priority
const NvicPriority NvicPriorityDriverDiag = 4;
const NvicPriority NvicPriorityAdc = 4;
#else
const NvicPriority NvicPriorityAuxUart = 3;			// UART is highest to avoid character loss (it has only a 1-character receive buffer)
const NvicPriority NvicPriorityWiFiUart = 3;		// UART used to receive debug data from the WiFi module
#endif

const NvicPriority NvicPriorityCan = 4;				// CAN interface
const NvicPriority NvicPriorityPins = 4;			// priority for GPIO pin interrupts - filament sensors must be higher than step
const NvicPriority NvicPriorityDriversSerialTMC = 4; // USART or UART used to control and monitor the smart drivers
const NvicPriority NvicPriorityStep = 5;			// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUSB = 6;				// USB interrupt
const NvicPriority NvicPriorityHSMCI = 6;			// HSMCI command complete interrupt

# if HAS_LWIP_NETWORKING
const NvicPriority NvicPriorityNetworkTick = 7;		// priority for network tick interrupt (to be replaced by a FreeRTOS task)
const NvicPriority NvicPriorityEthernet = 7;		// priority for Ethernet interface
# endif

const NvicPriority NvicPriorityDMA = 7;				// end-of-DMA interrupt used by TMC drivers and HSMCI
const NvicPriority NvicPrioritySpi = 7;				// SPI is used for network transfers on Duet WiFi/Duet vEthernet

#elif __NVIC_PRIO_BITS >= 4
// We have at least 16 priority levels
// Use priority 2 or lower for interrupts where low latency is critical and FreeRTOS calls are not needed.

# if SAM4E || defined(__LPC17xx__)
const NvicPriority NvicPriorityWatchdog = 0;		// the secondary watchdog has the highest priority
# endif

const NvicPriority NvicPriorityAuxUart = 3;			// UART is highest to avoid character loss (it has only a 1-character receive buffer)

# if defined(__LPC17xx__)
constexpr NvicPriority NvicPriorityTimerPWM = 4;
constexpr NvicPriority NvicPriorityTimerServo = 5;
# endif

const NvicPriority NvicPriorityDriversSerialTMC = 5; // USART or UART used to control and monitor the smart drivers
const NvicPriority NvicPriorityPins = 5;			// priority for GPIO pin interrupts - filament sensors must be higher than step
const NvicPriority NvicPriorityStep = 6;			// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityWiFiUart = 7;		// UART used to receive debug data from the WiFi module
const NvicPriority NvicPriorityUSB = 7;				// USB interrupt
const NvicPriority NvicPriorityHSMCI = 7;			// HSMCI command complete interrupt

# if HAS_LWIP_NETWORKING
const NvicPriority NvicPriorityNetworkTick = 8;		// priority for network tick interrupt (to be replaced by a FreeRTOS task)
const NvicPriority NvicPriorityEthernet = 8;		// priority for Ethernet interface
# endif

const NvicPriority NvicPrioritySpi = 8;				// SPI is used for network transfers on Duet WiFi/Duet vEthernet
const NvicPriority NvicPriorityTwi = 9;				// TWI is used to read endstop and other inputs on the DueXn

#endif

#endif
