#ifndef PINS_SAME70_H__
#define PINS_SAME70_H__

# define FIRMWARE_NAME		"RepRapFirmware for SAME70"
# define DEFAULT_BOARD_TYPE BoardType::SamE70TestBoard
const size_t NumFirmwareUpdateModules = 4;		// 3 modules, plus one for manual upload to WiFi module (module 2 not used)
# define IAP_FIRMWARE_FILE	"SAME70Firmware.bin"
# define WIFI_FIRMWARE_FILE	"DuetWiFiServer.bin"
# define WIFI_WEB_FILE		"DuetWebControl.bin"

// Features definition
#define HAS_LWIP_NETWORKING		1
#define HAS_WIFI_NETWORKING		1
#define HAS_CPU_TEMP_SENSOR		0
#define HAS_HIGH_SPEED_SD		1
#define HAS_SMART_DRIVERS		0		// TBD
#define HAS_STALL_DETECT		0		// TBD
#define HAS_VOLTAGE_MONITOR		0		// TBD
#define HAS_VREF_MONITOR		0		// TBD
#define ACTIVE_LOW_HEAT_ON		1

#define IAP_UPDATE_FILE		"iape70.bin"		// need special build for SAME70

#define SUPPORT_INKJET		0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND		0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER		0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_IOBITS		1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR	0					// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES	1		// set nonzero to support G10 L2 and G53..59

#define USE_CACHE			0					// Cache controller has some problems on the SAME70

// The physical capabilities of the machine

const size_t DRIVES = 12;						// The maximum number of drives supported by the electronics
const size_t MaxSmartDrivers = 10;				// The maximum number of smart drivers
#define DRIVES_(a,b,c,d,e,f,g,h,i,j,k,l) { a,b,c,d,e,f,g,h,i,j,k,l }

constexpr size_t Heaters = 8;						// The number of heaters in the machine; 0 is the heated bed even if there isn't one
#define HEATERS_(a,b,c,d,e,f,g,h) { a,b,c,d,e,f,g,h }

constexpr size_t NumExtraHeaterProtections = 8;		// The number of extra heater protection instances

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 9;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
// Initialization macro used in statements needing to initialize values in arrays of size MAX_AXES
#define AXES_(a,b,c,d,e,f,g,h,i) { a,b,c,d,e,f,g,h,i }

constexpr size_t MaxExtruders = DRIVES - MinAxes;	// The maximum number of extruders
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial

//TWI is disabled for now on the SAM7E until we rewrite the driver
//#define I2C_IFACE	Wire							// Which TWI interface we use

constexpr Pin DueXnExpansionStart = 200;			// Pin numbers 200-215 are on the I/O expander
constexpr Pin AdditionalIoExpansionStart = 220;		// Pin numbers 220-235 are on the additional I/O expander

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// DRIVES
constexpr Pin GlobalTmcEnablePin = NoPin;				// The pin that drives ENN of all TMC2660 drivers on production boards (on pre-production boards they are grounded)
constexpr Pin ENABLE_PINS[DRIVES] = { NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin };
constexpr Pin STEP_PINS[DRIVES] = { NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin };
constexpr Pin DIRECTION_PINS[DRIVES] = { NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin };

constexpr Pin DueX_SG = NoPin;							// DueX stallguard detect pin (TBD)
constexpr Pin DueX_INT = NoPin;						// DueX interrupt pin (TBD)

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
constexpr Pin END_STOP_PINS[DRIVES] = { NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin };

// HEATERS
constexpr Pin TEMP_SENSE_PINS[Heaters] = { NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin }; // Thermistor pin numbers
constexpr Pin HEAT_ON_PINS[Heaters] = { NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin };	// Heater pin numbers (TBD)

// Default thermistor parameters
constexpr float BED_R25 = 100000.0;
constexpr float BED_BETA = 3988.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4388.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float THERMISTOR_SERIES_RS = 4700.0;

// Number of SPI temperature sensors to support

constexpr size_t MaxSpiTempSensors = 0;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { };

// DHTxx data pin
constexpr Pin DhtDataPin = NoPin;											// TBD

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = NoPin;

// Analogue pin numbers
constexpr Pin Z_PROBE_PIN = NoPin;											// TBD
constexpr Pin PowerMonitorVinDetectPin = NoPin;								// TBD
constexpr Pin PowerMonitor5vDetectPin = NoPin;								// TBD

//constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider (TBD)

constexpr Pin VssaSensePin = NoPin;

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin Z_PROBE_MOD_PIN = NoPin;
constexpr Pin DiagPin = NoPin;												// TBD

// Cooling fans
constexpr size_t NUM_FANS = 1;
constexpr Pin COOLING_FAN_PINS[NUM_FANS] = { NoPin };
constexpr Pin COOLING_FAN_RPM_PIN = NoPin;									// TBD

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { 32, NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { NoPin };
constexpr uint32_t ExpectedSdCardSpeed = 25000000;

#if SUPPORT_INKJET
// Inkjet control pins
constexpr Pin INKJET_SERIAL_OUT = xx;										// Serial bitpattern into the shift register
constexpr Pin INKJET_SHIFT_CLOCK = xx;										// Shift the register
constexpr Pin INKJET_STORAGE_CLOCK = xx;									// Put the pattern in the output register
constexpr Pin INKJET_OUTPUT_ENABLE = xx;									// Make the output visible
constexpr Pin INKJET_CLEAR = xx;											// Clear the register to 0

#endif

#if SUPPORT_ROLAND
// Roland mill
constexpr Pin ROLAND_CTS_PIN = xx;											// Expansion pin 11, PA12_TXD1
constexpr Pin ROLAND_RTS_PIN = xx;											// Expansion pin 12, PA13_RXD1

#endif

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This next definition defines the highest one.
// This is the mapping from logical pins 60+ to firmware pin numbers
constexpr Pin SpecialPinMap[] =
{
};
constexpr Pin DueX5GpioPinMap[] = {};					// TBD
constexpr int HighestLogicalPin = 50;										// highest logical pin number on this electronics

// SAME70 Flash locations (may be expanded in the future) [TBD]
constexpr uint32_t IAP_FLASH_START = 0x00470000;
constexpr uint32_t IAP_FLASH_END = 0x0047FFFF;		// we allow a full 64K on the SAM4

// Duet pin numbers to control the WiFi interface
constexpr Pin EspResetPin = 19;					// Low on this in holds the WiFi module in reset (ESP_RESET)
constexpr Pin EspEnablePin = 48;				// High to enable the WiFi module, low to power it down (ESP_CH_PD)
constexpr Pin EspDataReadyPin = 12;				// Input from the WiFi module indicating that it wants to transfer data (ESP GPIO0)
constexpr Pin SamTfrReadyPin = 36;				// Output from the SAM to the WiFi module indicating we can accept a data transfer (ESP GPIO4 via 7474)
constexpr Pin SamCsPin = 20;					// SPI NPCS pin, input from WiFi module

// Timer allocation
#define NETWORK_TC			(TC0)
#define NETWORK_TC_CHAN		(0)
#define NETWORK_TC_IRQN		TC0_IRQn
#define NETWORK_TC_HANDLER	TC0_Handler
#define NETWORK_TC_ID		ID_TC0

#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(1)
#define STEP_TC_IRQN		TC1_IRQn
#define STEP_TC_HANDLER		TC1_Handler
#define STEP_TC_ID			ID_TC1

#endif
