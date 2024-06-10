/****************************************************************************************************

RepRapFirmware - Configuration

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
#include <cstring>			// for strlen

// Motion systems
#if SUPPORT_ASYNC_MOVES
constexpr unsigned int NumMovementSystems = 2;			// for now we support only two motion systems
#else
constexpr unsigned int NumMovementSystems = 1;
#endif

// Axes
constexpr float DefaultAxisMaxFeedrate = 100.0;			// mm/sec
constexpr float DefaultZMaxFeedrate = 20.0;
constexpr float DefaultEMaxFeedrate = 100.0;

constexpr float DefaultAxisAcceleration = 1000.0;		// mm/sec^2
constexpr float DefaultZAcceleration = 200.0;
constexpr float DefaultEAcceleration = 500.0;

constexpr float DefaultAxisDriveStepsPerUnit = 80.0;	// steps/mm
constexpr float DefaultZDriveStepsPerUnit = 800.0;
constexpr float DefaultEDriveStepsPerUnit = 420.0;

constexpr float DefaultAxisInstantDv = 15.0;			// mm/sec
constexpr float DefaultZInstantDv = 10.0;
constexpr float DefaultEInstantDv = 5.0;

constexpr float DefaultMinFeedrate = 0.5;				// the default minimum movement speed in mm/sec (extruding moves will go slower than this if the extrusion rate demands it)
constexpr float AbsoluteMinFeedrate = 0.01;				// the absolute minimum movement speed in mm/sec
constexpr float ImpossiblyHighFeedRate = 10000.0;		// a feedrate higher than any that are likely to be achieved

constexpr float MinimumJerk = 0.1;						// the minimum jerk in mm/sec
constexpr float MinimumAcceleration = 0.1;				// the minimum acceleration in mm/sec^2
constexpr float DefaultPrintingAcceleration = 50000.0;	// higher than any likely max acceleration defined by M201
constexpr float DefaultTravelAcceleration = 50000.0;	// higher than any likely max acceleration defined by M201

constexpr float DefaultAxisMinimum = 0.0;
constexpr float DefaultAxisMaximum = 200.0;

constexpr float DefaultFilamentDiameter = 1.75;			// the default filament diameter assumed

constexpr unsigned int MaxTools = 50;					// this limit is to stop the serialised object model getting too large
constexpr unsigned int MinVisibleAxes = 2;				// the minimum number of axes that we allow to be visible

constexpr float DefaultBacklashCorrectionDistanceFactor = 10.0;	// backlash correction is spread over (backlash amount * this) mm

// Timeouts
constexpr uint32_t LogFlushInterval = 15000;			// Milliseconds
constexpr float DefaultMessageTimeout = 10.0;			// How long a message is displayed by default, in seconds
constexpr uint16_t MinimumGpinReportInterval = 30;		// Minimum interval in milliseconds between input change reports sent over CAN bus

// Comms defaults
constexpr unsigned int MAIN_BAUD_RATE = 115200;			// Default communication speed of the USB if needed
constexpr unsigned int AUX_BAUD_RATE = 57600;			// Ditto - for auxiliary UART device
constexpr unsigned int AUX2_BAUD_RATE = 115200;			// Ditto - for second auxiliary UART device
constexpr uint32_t SERIAL_MAIN_TIMEOUT = 2000;			// timeout in ms for sending data to the main serial/USB port
constexpr uint32_t AuxTimeout = 2000;					// timeout in ms for PanelDue replies

constexpr uint32_t UnsolicitedStatusReportInterval = 2000;	// Interval between sending unsolicited status reports, in milliseconds

// Message boxes
constexpr unsigned int MaxMessageBoxes = 8;				// the maximum number of message boxes that can be queued

#define PANEL_DUE_FIRMWARE_FILE "PanelDueFirmware.bin"

// Conditional GCode support
constexpr unsigned int MaxBlockIndent = 10;				// maximum indentation of GCode. Each level of indentation introduces a new block.

// Default Z probe values

// The maximum number of probe points is constrained by RAM usage:
// - Each probe point uses 12 bytes of static RAM. So 16 points use 192 bytes
// - The delta calibration points use the same static ram, but when auto-calibrating we temporarily need more to hold the matrices etc. as follows:
//     Using single-precision maths and up to 9-factor calibration: (9 + 5) * 4 bytes per point
//     Using double-precision maths and up to 9-factor calibration: (9 + 5) * 8 bytes per point
//   So 32 points using double precision arithmetic need 3584 bytes of stack space.
#if SAME70
constexpr size_t MaxGridProbePoints = 961;				// 961 allows us to probe e.g. 300x300 at 10mm intervals
#else
constexpr size_t MaxGridProbePoints = 441;				// 441 allows us to probe e.g. 400x400 at 20mm intervals
#endif
constexpr size_t MaxAxis0GridPoints = 41;				// Maximum number of grid points in one X row
constexpr size_t MaxProbePoints = 32;					// Maximum number of G30 probe points
constexpr size_t MaxCalibrationPoints = 32;				// Should a power of 2 for speed

constexpr float DefaultGridSpacing = 20.0;				// Default bed probing grid spacing in mm

static_assert(MaxCalibrationPoints <= MaxProbePoints, "MaxCalibrationPoints must be <= MaxProbePoints");

constexpr size_t MaxScanningProbeCalibrationPoints = 33;	// The maximum number of heights we measure when calibrating a scanning probe. Use an odd number.

// SD card
constexpr uint32_t SdCardDetectDebounceMillis = 200;	// How long we give the SD card to settle in the socket
constexpr unsigned int MaxSdCardTries = 5;				// Number of read or write attempts before giving up
constexpr uint32_t SdCardRetryDelay = 20;				// Number of milliseconds delay between SD transfer retries. We now double for each retry.

// Z probing
constexpr float DefaultZProbeTriggerHeight = 0.7;		// Millimetres
constexpr float DefaultZProbeTemperature = 25.0;
constexpr float DefaultZDive = 5.0;						// Millimetres
constexpr float DefaultProbingSpeed = 2.0;				// Default Z probing speed mm/sec
constexpr float DefaultZProbeTravelSpeed = 100.0;		// Default speed for travel to probe points
constexpr float DefaultZProbeTolerance = 0.03;			// How close the Z probe trigger height from consecutive taps must be
constexpr uint8_t DefaultZProbeTaps = 1;				// The maximum number of times we probe each point
constexpr int32_t DefaultZProbeADValue = 500;			// Default trigger threshold

constexpr float SILLY_Z_VALUE = -9999.0;				// Millimetres

// String lengths. Try not to have too many different ones, because each one causes an instantiation of the String template
constexpr size_t MaxMessageLength = 256;
constexpr size_t MaxTitleLength = 61;
constexpr size_t MaxMultiplePinNamesLength = 61;		// 50 is not quite enough for 4 endstops such as !duex.e2Stop+!duex.e3Stop+!duex.e4Stop+!duex.e5Stop

constexpr size_t MaxFilenameLength = 120;				// Maximum length of a filename including the path
constexpr size_t MaxVariableNameLength = 120;

// Standard string lengths, to avoid having too many different instantiations of the String<n> template
constexpr size_t StringLength20 = 20;
constexpr size_t StringLength50 = 50;					// Used for pin names
constexpr size_t StringLength100 = 100;					// Used for error messages
constexpr size_t StringLength500 = 500;					// Used when writing the height map
constexpr size_t StringLength256 = 256;					// Used for various things

constexpr size_t MaxHeaterNameLength = StringLength20;	// Maximum number of characters in a heater name
constexpr size_t MaxFanNameLength = StringLength20;		// Maximum number of characters in a fan name
#ifdef DUET3_ATE
constexpr size_t GCodeReplyLength = StringLength500;	// Maximum number of characters in a GCode reply that doesn't use an OutputBuffer (ATE codes can generate long replies)
constexpr size_t FormatStringLength = StringLength500;	// GCode replies are processed by Platform::MessageF which uses an intermediate buffer of this length
#else
constexpr size_t GCodeReplyLength = StringLength256;	// Maximum number of characters in a GCode reply that doesn't use an OutputBuffer
constexpr size_t FormatStringLength = StringLength256;
#endif
constexpr size_t MachineNameLength = StringLength50;
constexpr size_t RepRapPasswordLength = StringLength20;
constexpr size_t MediumStringLength = MaxFilenameLength;
constexpr size_t M117StringLength = MediumStringLength;
constexpr size_t StringLengthLoggedCommand = StringLength100;	// Length of a string buffer for a command to be logged
constexpr size_t MaxStringExpressionLength = StringLength256;

// Increased GCODE_LENGTH because M587 and M589 commands on the Duet WiFi can get very long and GCode meta commands can get even longer
// Also if HAS_SBC_INTERFACE is enabled then it needs to be large enough to hold SBC commands sent in binary mode, see GCodeBuffer.h
constexpr size_t MaxGCodeLength = 256;					// maximum number of non-comment characters in a line of GCode including the null terminator

// Define the maximum length of a GCode that we can queue to synchronise it to a move. Long enough for M150 R255 U255 B255 P255 S255 F1 encoded in binary mode (64 bytes).
constexpr size_t ShortGCodeLength = 64;

// Output buffer length and number of buffers
// When using RTOS, it is best if it is possible to fit an HTTP response header in a single buffer. Our headers are currently about 230 bytes long.
// A note on reserved buffers: the worst case is when a GCode with a long response is processed. After string the response, there must be enough buffer space
// for the HTTP responder to return a status response. Otherwise DWC never gets to know that it needs to make a rr_reply call and the system deadlocks.
#if SAME70 || SAME5x
constexpr size_t OUTPUT_BUFFER_SIZE = 256;				// How many bytes does each OutputBuffer hold?
constexpr size_t OUTPUT_BUFFER_COUNT = 40;				// How many OutputBuffer instances do we have?
constexpr size_t RESERVED_OUTPUT_BUFFERS = 4;			// Number of reserved output buffers after long responses, enough to hold a status response
constexpr size_t MinimumBuffersForObjectModel = 20;		// Minimum number of free buffers we want before we start assembling a request for the object model
#elif SAM4E || SAM4S
constexpr size_t OUTPUT_BUFFER_SIZE = 256;				// How many bytes does each OutputBuffer hold?
constexpr size_t OUTPUT_BUFFER_COUNT = 26;				// How many OutputBuffer instances do we have?
constexpr size_t RESERVED_OUTPUT_BUFFERS = 4;			// Number of reserved output buffers after long responses, enough to hold a status response
constexpr size_t MinimumBuffersForObjectModel = 20;		// Minimum number of free buffers we want before we start assembling a request for the object model
#else
# error Unsupported processor
#endif

constexpr size_t maxQueuedCodes = 16;					// How many codes can be queued?

#if SAME70 || SAME5x
constexpr size_t MaxTrackedObjects = 40;				// How many build plate objects we track. Each one needs 16 bytes of storage, in addition to the string space.
#else
constexpr size_t MaxTrackedObjects = 20;				// How many build plate objects we track. Each one needs 16 bytes of storage, in addition to the string space.
#endif

// Expression evaluation in GCode meta commands etc.
constexpr size_t MaxExpressionArrayIndices = 5;

// How many filaments we can return in the file information. Each one uses 4 bytes of statically-allocated RAM.
#if SAME70 || SAME5x
constexpr unsigned int MaxFilaments = 20;
#else
constexpr unsigned int MaxFilaments = 8;
#endif

constexpr size_t MaxLaserPixelsPerMove = 8;				// How many S parameters you can use on a single G1 command when laser engraving

// Move system
constexpr float DefaultFeedRate = 3000.0;				// The initial requested feed rate after resetting the printer, in mm/min
constexpr float MaximumG0FeedRate = 60000.0;			// The maximum feed rate for G0 commands in mm/min, if the M203 settings permit
constexpr float MinRetractSpeed = 60.0;					// The minimum firmware retraction/un-retraction speed in mm/min
constexpr float DefaultRetractSpeed = 1000.0;			// The default firmware retraction and un-retraction speed, in mm/min
constexpr float DefaultRetractLength = 2.0;

constexpr float MaxArcDeviation = 0.005;				// maximum deviation from ideal arc due to segmentation
constexpr float MinArcSegmentLength = 0.02;				// G2 and G3 arc movement commands get split into segments at least this long
constexpr float MaxArcSegmentLength = 1.0;				// G2 and G3 arc movement commands get split into segments at most this long
constexpr float MaxArcSegmentsPerSec = 200.0;
constexpr float SegmentsPerFulArcCalculation = 8.0;		// we do the full sine/cosine calculation every this number of segments

constexpr uint32_t DefaultIdleTimeout = 30000;			// Milliseconds
constexpr float DefaultIdleCurrentFactor = 0.3;			// Proportion of normal motor current that we use for idle hold

constexpr uint32_t DefaultGracePeriod = 10;				// how long we wait for more moves to become available before starting movement

constexpr float DefaultNonlinearExtrusionLimit = 0.2;	// Maximum additional commanded extrusion to compensate for nonlinearity
constexpr size_t NumVisibleRestorePoints = 6;					// Number of restore points, must be at least 3

constexpr float AxisRoundingError = 0.02;				// Maximum possible error when we round trip a machine position to motor coordinates and back

// Default nozzle and filament values
constexpr float NOZZLE_DIAMETER = 0.5;					// Millimetres
constexpr float FILAMENT_WIDTH = 1.75;					// Millimetres

constexpr unsigned int MaxStackDepth = 10;				// Maximum depth of stack (was 5 in 3.01-RC2, increased to 7 for 3.01-RC3, 10 for 3.4.0beta6)

// CNC and laser support
constexpr float DefaultMinSpindlePwm = 0.0;				// Default minimum PWM level for spindle control
constexpr float DefaultMaxSpindlePwm = 1.0;				// Default maximum PWM level for spindle control
constexpr float DefaultIdleSpindlePwm = 0.0;			// Default idle PWM level for spindle control
constexpr int32_t DefaultMinSpindleRpm = 60;			// Default minimum available spindle RPM
constexpr int32_t DefaultMaxSpindleRpm = 10000;			// Default spindle RPM at full PWM
constexpr float DefaultMaxLaserPower = 255.0;			// Power setting in M3 command for full power
constexpr uint32_t LaserPwmIntervalMillis = 5;			// Interval (ms) between adjusting the laser PWM during acceleration or deceleration

// I2C
// A note on the i2C clock frequency.
// On a Duet WiFi attached to a DueX5 through 160mm of ribbon cable, the cable capacitance in combination with the 4K7 pullup resistors slows down the
// I2C signal rise time to about 500ns. This is above the 300ns maximum specified for both the ATSAM4E and the SX1509B.
// It appears that the slow rise time interferes with the watchdog timer resets, because at 200MHz clock frequency the system gets stuck
// in a boot loop caused by the watchdog timer going off.
// At 100kHz I2C clock frequency, these issues are rare.
constexpr uint32_t I2cClockFreq = 100000;				// clock frequency in Hz. 100kHz is 10us per bit, so about 90us per byte if there is no clock stretching
constexpr size_t MaxI2cBytes = 34;						// max bytes in M260 or M261 command. Increased to 34 for NeoDriver.

// File handling
#if defined(DUET3) || defined(DUET3MINI)
constexpr size_t MAX_FILES = 20;						// Must be large enough to handle the max number of concurrent web requests + file being printed + macros being executed + log file
#else
constexpr size_t MAX_FILES = 10;						// Must be large enough to handle the max number of concurrent web requests + file being printed + macros being executed + log file
#endif

constexpr size_t MaxLiteralArrayElements = 40;			// Maximum number of array elements we are allowed in a literal array
constexpr size_t MaxFileReadArrayElements = 40;			// Maximum number of array elements we are allowed to read in a read() function

constexpr size_t FILE_BUFFER_SIZE = 128;

constexpr size_t MaxThumbnails = 4;						// Maximum number of thumbnail images read from the job file that we store and report

// Webserver stuff
#define DEFAULT_PASSWORD		"reprap"				// Default machine password
#define DEFAULT_MACHINE_NAME	"My Duet"				// Default machine name
#define DEFAULT_HOSTNAME 		"duet"					// Default netbios name

#define INDEX_PAGE_FILE			"index.html"
#define OLD_INDEX_PAGE_FILE		"reprap.htm"
#define FOUR04_PAGE_FILE		"html404.htm"

// Filesystem and upload defaults
#define FS_PREFIX				"0:"
#define WEB_DIR					"0:/www/"				// Place to find web files on the SD card
#define GCODE_DIR				"0:/gcodes/"			// Ditto - G-Codes
#define DEFAULT_SYS_DIR			"0:/sys/"				// Ditto - System files (can be changed using M505)
#define MACRO_DIR				"0:/macros/"			// Ditto - Macro files
#define FILAMENTS_DIRECTORY		"0:/filaments/"			// Directory for filament configurations
#define FIRMWARE_DIRECTORY		"0:/firmware/"			// Directory for firmware and IAP files
#define MENU_DIR				"0:/menu/"				// Directory for menu files

// MaxExpectedWebDirFilenameLength is the maximum length of a filename that we can accept in a HTTP request without rejecting it out of hand
// and perhaps warning the user of a possible virus attack.
// It must be at least as long as any web file request from DWC, which is the file path excluding the initial "0:/www" and the trailing ".gz, possibly with "/" prepended.
// As at 2020-05-02 the longest filename requested by DWC is "/fonts/materialdesignicons-webfont.3e2c1c79.eot" which is 48 characters long
// It must be small enough that a filename within this length doesn't cause an overflow in MassStorage::CombineName. This is checked by the static_assert below.
constexpr size_t MaxExpectedWebDirFilenameLength = MaxFilenameLength - 20;
static_assert(MaxExpectedWebDirFilenameLength + strlen(WEB_DIR) + strlen(".gz") <= MaxFilenameLength);

#define UPLOAD_EXTENSION ".part"					// Extension to a filename for a file being uploaded

#define DEFAULT_LOG_FILE "eventlog.txt"

#define EOF_STRING "<!-- **EoF** -->"

// List defaults
constexpr char EXPRESSION_LIST_SEPARATOR = ',';
constexpr char LIST_SEPARATOR = ':';

#endif
