/****************************************************************************************************

RepRapFirmware - Configuration

This is where all machine-independent configuration and other definitions are set up. Nothing that
depends on any particular RepRap, RepRap component, or RepRap controller should go in here. Define
machine-dependent things in Platform.h

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <cstddef>			// for size_t

// Other firmware that we might switch to be compatible with.

enum Compatibility
{
	me = 0,
	reprapFirmware = 1,
	marlin = 2,
	teacup = 3,
	sprinter = 4,
	repetier = 5
};

// Generic constants

const float ABS_ZERO = -273.15;						// Celsius
const float NEARLY_ABS_ZERO = -273.0;				// Celsius
const float ROOM_TEMPERATURE = 21.0;				// Celsius

// Timeouts
const float LONG_TIME = 300.0;						// Seconds
const uint32_t MinimumWarningInterval = 4000;		// Milliseconds
const uint32_t FanCheckInterval = 500;				// Milliseconds
const uint32_t DriverCoolingTimeout = 4000;			// Milliseconds
const float DefaultMessageTimeout = 10.0;			// How long a message is displayed by default, in seconds


// FanCheckInterval must be lower than MinimumWarningInterval to avoid giving driver over temperature warnings too soon when thermostatic control of electronics cooling fans is used
static_assert(FanCheckInterval < MinimumWarningInterval, "FanCheckInterval too large");

// Comms defaults

const unsigned int MAIN_BAUD_RATE = 115200;			// Default communication speed of the USB if needed
const unsigned int AUX_BAUD_RATE = 57600;			// Ditto - for auxiliary UART device
const unsigned int AUX2_BAUD_RATE = 115200;			// Ditto - for second auxiliary UART device

const uint32_t SERIAL_MAIN_TIMEOUT = 1000;			// timeout in ms for sending data to the main serial/USB port

// Heater values

const float HEAT_SAMPLE_TIME = 0.5;					// Seconds
const float HEAT_PWM_AVERAGE_TIME = 5.0;			// Seconds

const float TEMPERATURE_CLOSE_ENOUGH = 1.0;			// Celsius
const float TEMPERATURE_LOW_SO_DONT_CARE = 40.0;	// Celsius
const float HOT_ENOUGH_TO_EXTRUDE = 160.0;			// Celsius
const float HOT_ENOUGH_TO_RETRACT = 90.0;			// Celsius

const uint8_t MAX_BAD_TEMPERATURE_COUNT = 4;		// Number of bad temperature samples permitted before a heater fault is reported
const float BAD_LOW_TEMPERATURE = -10.0;			// Celsius
const float DefaultExtruderTemperatureLimit = 288.0; // Celsius - E3D say to tighten the hot end at 285C
const float DefaultBedTemperatureLimit = 125.0;		// Celsius
const float HOT_END_FAN_TEMPERATURE = 45.0;			// Temperature at which a thermostatic hot end fan comes on
const float ThermostatHysteresis = 1.0;				// How much hysteresis we use to prevent noise turning fans on/off too often
const float BAD_ERROR_TEMPERATURE = 2000.0;			// Must exceed any reasonable 5temperature limit including DEFAULT_TEMPERATURE_LIMIT

// Heating model default parameters. For the chamber heater, we use the same values as for the bed heater.
// These parameters are about right for an E3Dv6 hot end with 30W heater.
const float DefaultHotEndHeaterGain = 340.0;
const float DefaultHotEndHeaterTimeConstant = 140.0;
const float DefaultHotEndHeaterDeadTime = 5.5;

const int8_t DefaultBedHeater = 0;
const int8_t DefaultChamberHeater = -1;
const int8_t DefaultE0Heater = 1;					// Index of the default first extruder heater

const unsigned int FirstVirtualHeater = 100;		// the heater number at which virtual heaters start
const unsigned int MaxVirtualHeaters = 10;			// the number of virtual heaters supported

// These parameters are about right for a typical PCB bed heater that maxes out at 110C
const float DefaultBedHeaterGain = 90.0;
const float DefaultBedHeaterTimeConstant = 700.0;
const float DefaultBedHeaterDeadTime = 10.0;

// Parameters used to detect heating errors
const float DefaultMaxHeatingFaultTime = 5.0;		// How many seconds we allow a heating fault to persist
const float AllowedTemperatureDerivativeNoise = 0.25;	// How much fluctuation in the averaged temperature derivative we allow
const float MaxAmbientTemperature = 45.0;			// We expect heaters to cool to this temperature or lower when switched off
const float NormalAmbientTemperature = 25.0;		// The ambient temperature we assume - allow for the printer heating its surroundings a little
const float DefaultMaxTempExcursion = 15.0;			// How much error we tolerate when maintaining temperature before deciding that a heater fault has occurred
const float MinimumConnectedTemperature = -5.0;		// Temperatures below this we treat as a disconnected thermistor

static_assert(DefaultMaxTempExcursion > TEMPERATURE_CLOSE_ENOUGH, "DefaultMaxTempExcursion is too low");

// Temperature sense channels
const unsigned int FirstThermistorChannel = 0;		// Temperature sensor channels 0... are thermistors
const unsigned int FirstMax31855ThermocoupleChannel = 100;	// Temperature sensor channels 100... are MAX31855 thermocouples
const unsigned int FirstMax31856ThermocoupleChannel = 150;	// Temperature sensor channels 150... are MAX31856 thermocouples
const unsigned int FirstRtdChannel = 200;			// Temperature sensor channels 200... are RTDs
const unsigned int FirstLinearAdcChannel = 300;		// Temperature sensor channels 300... use an ADC that provides a linear output over a temperature range
const unsigned int CpuTemperatureSenseChannel = 1000;  // Sensor 1000 is the MCJU's own temperature sensor
const unsigned int FirstTmcDriversSenseChannel = 1001; // Sensors 1001..1002 are the TMC2660 driver temperature sense
const unsigned int NumTmcDriversSenseChannels = 2;	// Sensors 1001..1002 are the TMC2660 driver temperature sense

// PWM frequencies
const unsigned int SlowHeaterPwmFreq = 10;			// slow PWM frequency for bed and chamber heaters, compatible with DC/AC SSRs
const unsigned int NormalHeaterPwmFreq = 250;		// normal PWM frequency used for hot ends
const unsigned int DefaultFanPwmFreq = 250;			// increase to 25kHz using M106 command to meet Intel 4-wire PWM fan specification
const unsigned int DefaultPinWritePwmFreq = 500;	// default PWM frequency for M42 pin writes and extrusion ancillary PWM

// Default Z probe values

// The maximum number of probe points is constrained by RAM usage:
// - Each probe point uses 12 bytes of static RAM. So 16 points use 192 bytes
// - The delta calibration points use the same static ram, but when auto-calibrating we temporarily need more to hold the matrices etc. as follows:
//     Using single-precision maths and up to 9-factor calibration: (9 + 5) * 4 bytes per point
//     Using double-precision maths and up to 9-factor calibration: (9 + 5) * 8 bytes per point
//   So 32 points using double precision arithmetic need 3584 bytes of stack space.
#ifdef DUET_NG
const size_t MaxGridProbePoints = 441;				// 441 allows us to probe e.g. 400x400 at 20mm intervals
const size_t MaxXGridPoints = 41;					// Maximum number of grid points in one X row
const size_t MaxProbePoints = 32;					// Maximum number of G30 probe points
const size_t MaxDeltaCalibrationPoints = 32;		// Should a power of 2 for speed
#else
const size_t MaxGridProbePoints = 121;				// 121 allows us to probe 200x200 at 20mm intervals
const size_t MaxXGridPoints = 21;					// Maximum number of grid points in one X row
const size_t MaxProbePoints = 32;					// Maximum number of G30 probe points
const size_t MaxDeltaCalibrationPoints = 32;		// Should a power of 2 for speed
#endif

const float DefaultGridSpacing = 20.0;				// Default bed probing grid spacing in mm

static_assert(MaxProbePoints <= MaxGridProbePoints, "MaxProbePoints must be <= MaxGridProbePoints");
static_assert(MaxDeltaCalibrationPoints <= MaxProbePoints, "MaxDeltaCalibrationPoints must be <= MaxProbePoints");

const float DEFAULT_Z_DIVE = 5.0;					// Millimetres
const float DEFAULT_PROBE_SPEED = 2.0;				// Default Z probing speed mm/sec
const float DEFAULT_TRAVEL_SPEED = 100.0;			// Default speed for travel to probe points
const float ZProbeMaxAcceleration = 250.0;			// Maximum Z acceleration to use at the start of a probing move
const size_t MaxZProbeProgramBytes = 8;				// Maximum number of bytes in a Z probe program

const float TRIANGLE_ZERO = -0.001;					// Millimetres
const float SILLY_Z_VALUE = -9999.0;				// Millimetres

// String lengths

const size_t FORMAT_STRING_LENGTH = 256;
const size_t MACHINE_NAME_LENGTH = 40;
const size_t PASSWORD_LENGTH = 20;

const size_t GCODE_LENGTH = 100;
const size_t GCODE_REPLY_LENGTH = 2048;
const size_t MESSAGE_LENGTH = 256;

const size_t FILENAME_LENGTH = 100;
const size_t MaxHeaterNameLength = 20;				// Maximum number of characters in a heater name

// Output buffer lengths

#ifdef DUET_NG
const uint16_t OUTPUT_BUFFER_SIZE = 256;			// How many bytes does each OutputBuffer hold?
const size_t OUTPUT_BUFFER_COUNT = 32;				// How many OutputBuffer instances do we have?
const size_t RESERVED_OUTPUT_BUFFERS = 1;			// Number of reserved output buffers after long responses. Must be enough for an HTTP header
#else
const uint16_t OUTPUT_BUFFER_SIZE = 128;			// How many bytes does each OutputBuffer hold?
const size_t OUTPUT_BUFFER_COUNT = 32;				// How many OutputBuffer instances do we have?
const size_t RESERVED_OUTPUT_BUFFERS = 2;			// Number of reserved output buffers after long responses. Must be enough for an HTTP header
#endif

// Move system

const float DefaultFeedrate = 3000.0;				// The initial requested feed rate after resetting the printer, in mm/min
const float DefaultRetractSpeed = 1000.0;			// The default firmware retraction and un-retraction speed, in mm
const float DefaultRetractLength = 2.0;

const float DefaultArcSegmentLength = 0.2;			// G2 and G3 arc movement commands get split into segments this long

const float DEFAULT_IDLE_TIMEOUT = 30.0;			// Seconds
const float DEFAULT_IDLE_CURRENT_FACTOR = 0.3;		// Proportion of normal motor current that we use for idle hold

// Triggers

const unsigned int MaxTriggers = 10;				// Must be <= 32 because we store a bitmap of pending triggers in a uint32_t

// Default nozzle and filament values

const float NOZZLE_DIAMETER = 0.5;					// Millimetres
const float FILAMENT_WIDTH = 1.75;					// Millimetres

const unsigned int MaxStackDepth = 5;				// Maximum depth of stack

// Webserver stuff

#define DEFAULT_PASSWORD "reprap"					// Default machine password
#define DEFAULT_NAME "My Duet"						// Default machine name
#define HOSTNAME "duet"								// Default netbios name

#define INDEX_PAGE_FILE "reprap.htm"
#define FOUR04_PAGE_FILE "html404.htm"

// Filesystem and upload defaults

#define FS_PREFIX "0:"
#define WEB_DIR "0:/www/"							// Place to find web files on the SD card
#define GCODE_DIR "0:/gcodes/"						// Ditto - G-Codes
#define SYS_DIR "0:/sys/"							// Ditto - System files
#define MACRO_DIR "0:/macros/"						// Ditto - Macro files
#define SCANS_DIRECTORY "0:/scans/"					// Directory for uploaded 3D scans
#define FILAMENTS_DIRECTORY "0:/filaments/"			// Directory for filament configurations

#define CONFIG_FILE "config.g"
#define DEFAULT_FILE "default.g"

#define EOF_STRING "<!-- **EoF** -->"

// Firmware update file names are now defined in the Pins file

// List defaults

const char LIST_SEPARATOR = ':';
const char FILE_LIST_SEPARATOR = ',';
const char FILE_LIST_BRACKET = '"';

#endif
