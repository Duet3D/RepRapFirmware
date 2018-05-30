/*
 * Pins_DuetM.h
 *
 *  Created on: 29 Nov 2017
 *      Author: David
 */

#ifndef SRC_DUETM_PINS_DUETM_H_
#define SRC_DUETM_PINS_DUETM_H_

# define FIRMWARE_NAME "RepRapFirmware for Duet 2 Maestro"
# define DEFAULT_BOARD_TYPE BoardType::DuetM_10
constexpr size_t NumFirmwareUpdateModules = 1;		// 1 module
# define IAP_FIRMWARE_FILE	"DuetMaestroFirmware.bin"

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		0
#define HAS_W5500_NETWORKING	1

#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		0
#define HAS_VOLTAGE_MONITOR		1
#define HAS_VREF_MONITOR		1
#define ACTIVE_LOW_HEAT_ON		1

#define IAP_UPDATE_FILE		"iap4s.bin"

#define SUPPORT_INKJET		0						// set nonzero to support inkjet control
#define SUPPORT_ROLAND		0						// set nonzero to support Roland mill
#define SUPPORT_SCANNER		0						// set zero to disable support for FreeLSS scanners
#define SUPPORT_IOBITS		0						// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR	1						// set nonzero to support DHT temperature/humidity sensors (requires RTOS)
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_12864_LCD	1						// set nonzero to support 12864 LCD and rotary encoder

// The physical capabilities of the machine

constexpr size_t DRIVES = 7;						// The maximum number of drives supported by the electronics
constexpr size_t MaxSmartDrivers = 7;				// The maximum number of smart drivers
#define DRIVES_(a,b,c,d,e,f,g,h,i,j,k,l) { a,b,c,d,e,f,g }

constexpr size_t Heaters = 4;						// The number of heaters/thermistors in the machine. Duet M has 3 heaters but 4 thermistors.
#define HEATERS_(a,b,c,d,e,f,g,h) { a,b,c }

constexpr size_t NumExtraHeaterProtections = 4;		// The number of extra heater protection instances

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 6;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
// Initialization macro used in statements needing to initialize values in arrays of size MAX_AXES
#define AXES_(a,b,c,d,e,f,g,h,i) { a,b,c,d,e,f }

constexpr size_t MaxExtruders = DRIVES - MinAxes;	// The maximum number of extruders
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial

#define I2C_IFACE	Wire							// First and only I2C interface
#define I2C_IRQn	WIRE_ISR_ID

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drivers
constexpr Pin GlobalTmcEnablePin = 1;				// The pin that drives ENN of all drivers
constexpr Pin ENABLE_PINS[DRIVES] = { NoPin, NoPin, NoPin, NoPin, NoPin, 63, 61 };
constexpr Pin STEP_PINS[DRIVES] = { 56, 38, 64, 40, 41, 67, 60 };
constexpr Pin DIRECTION_PINS[DRIVES] = { 54, 8, 30, 33, 42, 18, 57 };
constexpr Pin DriverMuxPins[3] = { 50, 52, 53 };	// Pins that control the UART multiplexer, LSB first

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
constexpr Pin END_STOP_PINS[DRIVES] = { 24, 32, 46, 25, 43, NoPin, NoPin };

// Heaters and thermistors
constexpr Pin HEAT_ON_PINS[Heaters] = { 36, 37, 16, NoPin };	// Heater pin numbers
constexpr Pin TEMP_SENSE_PINS[Heaters] = { 20, 26, 66, 27 }; 	// Thermistor pin numbers
constexpr Pin VssaSensePin = 19;
constexpr Pin VrefSensePin = 17;

// Default thermistor parameters
constexpr float BED_R25 = 100000.0;
constexpr float BED_BETA = 3988.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4388.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float THERMISTOR_SERIES_RS = 2200.0;

// Number of SPI temperature sensors to support
constexpr size_t MaxSpiTempSensors = 2;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 35, 55 };			// SPI0_CS1, SPI0_CS2

// DHTxx data pin
//constexpr Pin DhtDataPin = 97;											// Pin CS6

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = 0;

// Analogue pin numbers
constexpr Pin Z_PROBE_PIN = 51;												// Z probe analog input
constexpr Pin PowerMonitorVinDetectPin = 48;								// Vin monitor
constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin Z_PROBE_MOD_PIN = 62;
constexpr Pin DiagPin = Z_PROBE_MOD_PIN;

// Cooling fans
constexpr size_t NUM_FANS = 3;
constexpr Pin COOLING_FAN_PINS[NUM_FANS] = { 59, 58, 65 };
constexpr Pin COOLING_FAN_RPM_PIN = 21;

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { 44, NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { 34 };
constexpr uint32_t ExpectedSdCardSpeed = 15000000;

// 12864 LCD
constexpr Pin LcdCSPin = 45;
constexpr Pin LcdBeepPin = 15;
constexpr Pin EncoderPinA = 31;
constexpr Pin EncoderPinB = 39;
constexpr Pin EncoderPinSw = 7;

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This next definition defines the highest one.
// This is the mapping from logical pins 60+ to firmware pin numbers
constexpr Pin SpecialPinMap[] =
{
	21, 22, 3, 4, Z_PROBE_MOD_PIN											// PA21/RXD1/AD8, PA22/TXD1/AD9, PA3/TWD0, PA4/TWC, Z_MOD
};

constexpr int HighestLogicalPin = 64;										// highest logical pin number on this electronics

// SAM4S Flash locations (may be expanded in the future)
constexpr uint32_t IAP_FLASH_START = 0x00470000;
constexpr uint32_t IAP_FLASH_END = 0x0047FFFF;								// we allow a full 64K on the SAM4

// Duet pin numbers to control the W5500 interface
constexpr Pin W5500ResetPin = 100;											// Low on this in holds the W5500 module in reset (ESP_RESET)
constexpr Pin W5500SsPin = 11;												// SPI NPCS pin, input from W5500 module

// Timer allocation
// TC0 channel 0 is used for step pulse generation and software timers
// TC0 channel 1 is used for LCD beep
// TC0 channel 2 is currently unused
#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(0)
#define STEP_TC_ID			ID_TC0
#define STEP_TC_IRQN		TC0_IRQn
#define STEP_TC_HANDLER		TC0_Handler

#endif /* SRC_DUETM_PINS_DUETM_H_ */
