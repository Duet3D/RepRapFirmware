#ifndef PINS_DUETNG_H__
#define PINS_DUETNG_H__

#define NAME "RepRapFirmware for Duet WiFi"

const size_t NumFirmwareUpdateModules = 4;			// 3 modules, plus one for manual upload to WiFi module
#define IAP_UPDATE_FILE		"iap4e.bin"
#define IAP_FIRMWARE_FILE	"DuetWiFiFirmware.bin"
#define WIFI_FIRMWARE_FILE	"DuetWiFiServer.bin"
#define WIFI_WEB_FILE		"DuetWebControl.bin"

// Default board type
#ifdef PROTOTYPE_1
#define DEFAULT_BOARD_TYPE BoardType::DuetWiFi_06
#else
#define DEFAULT_BOARD_TYPE BoardType::DuetWiFi_10
#endif

#define SUPPORT_ETHERNET	0					// set nonzero to support embedded web interface over Ethernet
#define SUPPORT_INKJET		0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND		0					// set nonzero to support Roland mill

// The physical capabilities of the machine

const size_t DRIVES = 10;						// The number of drives in the machine, including X, Y, and Z plus extruder drives
#define DRIVES_(a,b,c,d,e,f,g,h,i,j) { a,b,c,d,e,f,g,h,i,j }

const int8_t HEATERS = 8;						// The number of heaters in the machine; 0 is the heated bed even if there isn't one
#define HEATERS_(a,b,c,d,e,f,g,h) { a,b,c,d,e,f,g,h }
const size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

const size_t AXES = 3;							// The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
const size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to -1 to flag unavailability.

// DRIVES

const Pin ENABLE_PINS[DRIVES] = { 78, 41, 42, 49, 57, 87, 88, 89, 90, 31 };
const bool ENABLE_VALUES[DRIVES] = { false, false, false, false, false, false, false, false, false, false };	// What to send to enable a drive
const Pin STEP_PINS[DRIVES] = { 70, 71, 72, 69, 68, 66, 65, 64, 67, 91 };
const Pin DIRECTION_PINS[DRIVES] = { 75, 76, 77, 01, 73, 92, 86, 80, 81, 32 };
const bool DIRECTIONS[DRIVES] = { FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS };	// What each axis needs to make it go forwards - defaults

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
const Pin END_STOP_PINS[DRIVES] = { 46, 02, 93, 74, 48, 96, 97, 98, 99, 17 };

// Indices for motor current digipots (if any): first 4 are for digipot 1 (on duet), second 4 for digipot 2 (on expansion board)
#ifdef PROTOTYPE_1
const uint8_t POT_WIPES[8] = { 1, 3, 2, 0, 1, 3, 2, 0 };
const float SENSE_RESISTOR = 0.1;										// Stepper motor current sense resistor
const float MAX_STEPPER_DIGIPOT_VOLTAGE = (3.3 * 2.5 / (2.7 + 2.5));	// Stepper motor current reference voltage
const float STEPPER_DAC_VOLTAGE_RANGE = 2.02;							// Stepper motor current reference voltage for E1 if using a DAC
const float STEPPER_DAC_VOLTAGE_OFFSET = -0.11;							// Stepper motor current offset voltage for E1 if using a DAC
#endif

// HEATERS

const bool HEAT_ON = false;												// false for inverted heater (e.g. Duet v0.6), true for not (e.g. Duet v0.4)

const Pin TEMP_SENSE_PINS[HEATERS] = { 45, 47, 44, 61, 62, 63, 59, 18 }; // Thermistor pin numbers

#ifdef PROTOTYPE_1
const Pin HEAT_ON_PINS[HEATERS] = { 19, 20, 16, 15, 37, 40, 43, 38 };	// Heater pin numbers
#else
const Pin HEAT_ON_PINS[HEATERS] = { 19, 20, 16, 35, 37, 40, 43, 38 };	// Heater pin numbers (heater 7 pin TBC)
#endif

// Default thermistor parameters
// Bed thermistor: now assuming 100K
// Hot end thermistor: http://www.digikey.co.uk/product-search/en?x=20&y=11&KeyWords=480-3137-ND
const float BED_R25 = 100000.0;
const float BED_BETA = 3988.0;
const float EXT_R25 = 100000.0;
const float EXT_BETA = 4388.0;

// Thermistor series resistor value in Ohms
const float THERMISTOR_SERIES_RS = 4700.0;

// Number of SPI temperature sensors to support

#if SUPPORT_ROLAND

// chrishamm's pin assignments
const size_t MaxSpiTempSensors = 2;

// Digital pins the 31855s have their select lines tied to
const Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 56, 27 };

#else

const size_t MaxSpiTempSensors = 4;

// Digital pins the 31855s have their select lines tied to
# ifdef PROTOTYPE_1
const Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 24, 25, 50, 51 };	// SPI1_CS0, SPI1_CS1, CS2, CS3
# else
const Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 28, 50, 51, 52 };	// SPI0_CS1, SPI0_CS2, CS3, CS4
# endif

#endif

// Arduino Due pin number that controls the ATX power on/off
const Pin ATX_POWER_PIN = 79;											// Arduino Due pin number that controls the ATX power on/off

// Analogue pin numbers
const Pin Z_PROBE_PIN = 33;												// AFE1_AD4/PC1 Z probe analog input
const Pin PowerMonitorVinDetectPin = 36;								// AFE1_AD7/PC4 Vin monitor
const Pin PowerMonitor5vDetectPin = 29;									// AFE1_AD1/PB3 Buck regulator input monitor

const float PowerFailVoltageRange = 11.0 * 3.3;							// we use an 11:1 voltage divider

// Digital pin number to turn the IR LED on (high) or off (low)
const Pin Z_PROBE_MOD_PIN = 34;											// Digital pin number to turn the IR LED on (high) or off (low) on Duet v0.6 and v1.0 (PB21)

// COOLING FANS

const size_t NUM_FANS = 3;
const Pin COOLING_FAN_PINS[NUM_FANS] = { 55, 58, 00 };
const Pin COOLING_FAN_RPM_PIN = 32;

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

// Definition of which pins we allow to be controlled using M42
//
// The allowed pins are these ones on the DueX4 expansion connector:
//TODO document these

const size_t NUM_PINS_ALLOWED = 96;

#if 1
#define PINS_ALLOWED { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };	//TODO temporary!
#else
#define PINS_ALLOWED {				\
	/* pins 00-07 */	0b00000000,	\
	/* pins 08-15 */	0b00000000,	\
	/* pins 16-23 */	0b00000110,	\
	/* pins 24-31 */	0b10000011,	\
	/* pins 32-39 */	0b00001001,	\
	/* pins 40-47 */	0b00000000,	\
	/* pins 48-55 */	0b00011000,	\
	/* pins 56-63 */	0b00000000,	\
	/* pins 64-71 */	0b00000000,	\
	/* pins 72-79 */	0b00000000,	\
	/* pins 80-87 */	0b00000000,	\
	/* pins 88-95 */	0b00001000	\
}
#endif

// SAM4E Flash locations (may be expanded in the future)
const uint32_t IAP_FLASH_START = 0x00470000;
const uint32_t IAP_FLASH_END = 0x0047FBFF;

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
