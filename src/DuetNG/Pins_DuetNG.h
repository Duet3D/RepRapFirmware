#ifndef PINS_DUETNG_H__
#define PINS_DUETNG_H__

#if defined(DUET_WIFI)

# define FIRMWARE_NAME "RepRapFirmware for Duet WiFi"
# define DEFAULT_BOARD_TYPE BoardType::DuetWiFi_10
const size_t NumFirmwareUpdateModules = 4;		// 3 modules, plus one for manual upload to WiFi module
# define IAP_FIRMWARE_FILE	"DuetWiFiFirmware.bin"
# define WIFI_FIRMWARE_FILE	"DuetWiFiServer.bin"
# define WIFI_WEB_FILE		"DuetWebControl.bin"

#elif defined(DUET_ETHERNET)

# define FIRMWARE_NAME "RepRapFirmware for Duet Ethernet"
# define DEFAULT_BOARD_TYPE BoardType::DuetEthernet_10
# define IAP_FIRMWARE_FILE	"DuetEthernetFirmware.bin"
const size_t NumFirmwareUpdateModules = 1;		// 1 module

#else
# error Firmware name not defined
#endif

#define IAP_UPDATE_FILE		"iap4e.bin"			// hoping eventually to use the same IAP file for both Duet WiFi and Duet Ethernet

#define SUPPORT_INKJET		0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND		0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER		1					// set zero to disable support for FreeLSS scanners
#define SUPPORT_IOBITS		1					// set to support P parameter in G0/G1 commands

// The physical capabilities of the machine

const size_t DRIVES = 10;						// The number of drives in the machine, including X, Y and Z plus extruder drives
#define DRIVES_(a,b,c,d,e,f,g,h,i,j) { a,b,c,d,e,f,g,h,i,j }

const size_t Heaters = 8;						// The number of heaters in the machine; 0 is the heated bed even if there isn't one
#define HEATERS_(a,b,c,d,e,f,g,h) { a,b,c,d,e,f,g,h }

const size_t MinAxes = 3;						// The minimum and default number of axes
const size_t MaxAxes = 9;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
// Initialization macro used in statements needing to initialize values in arrays of size MAX_AXES
#define AXES_(a,b,c,d,e,f,g,h,i) { a,b,c,d,e,f,g,h,i }

const size_t MaxExtruders = DRIVES - MinAxes;	// The maximum number of extruders
const size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

const size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial

const Pin ExpansionStart = 200;					// Pin numbers at/above this are on the I/O expander

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// DRIVES

const Pin GlobalTmcEnablePin = 38;				// The pin that drives ENN of all TMC2660 drivers on production boards (on pre-production boards they are grounded)
const Pin ENABLE_PINS[DRIVES] = { 78, 41, 42, 49, 57, 87, 88, 89, 90, 31 };
const Pin STEP_PINS[DRIVES] = { 70, 71, 72, 69, 68, 66, 65, 64, 67, 91 };
const Pin DIRECTION_PINS[DRIVES] = { 75, 76, 77, 01, 73, 92, 86, 80, 81, 32 };

const Pin DueX_SG = 96;				// DueX stallguard detect pin = PE0 (was E2_STOP)
const Pin DueX_INT = 17;			// DueX interrupt pin = PA17 (was E6_STOP)

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
const Pin END_STOP_PINS[DRIVES] = { 46, 02, 93, 74, 48, 200, 203, 202, 201, 213 };

// HEATERS

const bool HEAT_ON = false;												// false for inverted heater (e.g. Duet v0.6), true for not (e.g. Duet v0.4)
const Pin TEMP_SENSE_PINS[Heaters] = { 45, 47, 44, 61, 62, 63, 59, 18 }; // Thermistor pin numbers
const Pin HEAT_ON_PINS[Heaters] = { 19, 20, 16, 35, 37, 40, 43, 15 };	// Heater pin numbers (heater 7 pin TBC)

// Default thermistor parameters
const float BED_R25 = 100000.0;
const float BED_BETA = 3988.0;
const float BED_SHC = 0.0;
const float EXT_R25 = 100000.0;
const float EXT_BETA = 4388.0;
const float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
const float THERMISTOR_SERIES_RS = 4700.0;

// Number of SPI temperature sensors to support

#if SUPPORT_ROLAND

// chrishamm's pin assignments
const size_t MaxSpiTempSensors = 2;

// Digital pins the 31855s have their select lines tied to
const Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 56, 27 };

#else

const size_t MaxSpiTempSensors = 8;

// Digital pins the 31855s have their select lines tied to
const Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 28, 50, 51, 52, 24, 97, 98, 99 };	// SPI0_CS1, SPI0_CS2, CS3, CS4, CS5, CS6, CS7, CS8

#endif

// Arduino Due pin number that controls the ATX power on/off
const Pin ATX_POWER_PIN = 79;											// Arduino Due pin number that controls the ATX power on/off

// Analogue pin numbers
const Pin Z_PROBE_PIN = 33;												// AFE1_AD4/PC1 Z probe analog input
const Pin PowerMonitorVinDetectPin = 36;								// AFE1_AD7/PC4 Vin monitor
const Pin PowerMonitor5vDetectPin = 29;									// AFE1_AD1/PB3 5V regulator input monitor

const float PowerFailVoltageRange = 11.0 * 3.3;							// we use an 11:1 voltage divider

const Pin VssaSensePin = 103;

// Digital pin number to turn the IR LED on (high) or off (low)
const Pin Z_PROBE_MOD_PIN = 34;											// Digital pin number to turn the IR LED on (high) or off (low) on Duet v0.6 and v1.0 (PB21)

// Cooling fans
const size_t NUM_FANS = 9;
const Pin COOLING_FAN_PINS[NUM_FANS] = { 55, 58, 00, 212, 207, 206, 205, 204, 215 };
const Pin COOLING_FAN_RPM_PIN = 102;									// PB6 on expansion connector

// SD cards
const size_t NumSdCards = 2;
const Pin SdCardDetectPins[NumSdCards] = {53, NoPin};
const Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
const Pin SdSpiCSPins[1] = {56};

#if SUPPORT_INKJET
// Inkjet control pins
const Pin INKJET_SERIAL_OUT = xx;										// Serial bitpattern into the shift register
const Pin INKJET_SHIFT_CLOCK = xx;										// Shift the register
const Pin INKJET_STORAGE_CLOCK = xx;									// Put the pattern in the output register
const Pin INKJET_OUTPUT_ENABLE = xx;									// Make the output visible
const Pin INKJET_CLEAR = xx;											// Clear the register to 0

#endif

#if SUPPORT_ROLAND
// Roland mill
const Pin ROLAND_CTS_PIN = xx;											// Expansion pin 11, PA12_TXD1
const Pin ROLAND_RTS_PIN = xx;											// Expansion pin 12, PA13_RXD1

#endif

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This next definition defines the highest one.
// This is the mapping from logical pins 60+ to firmware pin numbers
const Pin SpecialPinMap[] =
{
	24, 97, 98, 99														// We allow CS5-CS8 to be used because few users need >4 thermocouples or RTDs
};
const Pin DueX5GpioPinMap[] = { 211, 210, 209, 208 };					// GPIO 1-4 on DueX5
const int HighestLogicalPin = 100 + ARRAY_SIZE(DueX5GpioPinMap) - 1;	// highest logical pin number on this electronics

// SAM4E Flash locations (may be expanded in the future)
const uint32_t IAP_FLASH_START = 0x00470000;
const uint32_t IAP_FLASH_END = 0x0047FFFF;		// we allow a full 64K on the SAM4

// Duet pin numbers to control the WiFi interface
const Pin EspResetPin = 100;			// Low on this in holds the WiFi module in reset (ESP_RESET)
const Pin EspEnablePin = 101;			// High to enable the WiFi module, low to power it down (ESP_CH_PD)
const Pin EspTransferRequestPin = 95;	// Input from the WiFi module indicating that it wants to transfer data (ESP GPIO0)
const Pin SamTfrReadyPin = 94;			// Output from the SAM to the WiFi module indicating we can accept a data transfer (ESP GPIO4 via 7474)
const Pin SamCsPin = 11;				// SPI NPCS pin, input from WiFi module

// Timer allocation (no network timer on DuetNG)
// TC0 channel 0 is used for FAN2
// TC0 channel 1 is currently unused (may use ift for a heater or a fan)
// TC0 channel 2 is available for us to use
#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(2)
#define STEP_TC_IRQN		TC2_IRQn
#define STEP_TC_HANDLER		TC2_Handler

#endif
