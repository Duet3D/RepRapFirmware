#ifndef PINS_DUETNG_H__
#define PINS_DUETNG_H__

# define FIRMWARE_NAME	"RepRapFirmware for Duet 2 WiFi/Ethernet"
# define DEFAULT_BOARD_TYPE BoardType::DuetWiFi_10
constexpr size_t NumFirmwareUpdateModules = 4;		// 3 modules, plus one for manual upload to WiFi module (module 2 is now unused)
# define IAP_FIRMWARE_FILE	"Duet2CombinedFirmware.bin"
#define IAP_UPDATE_FILE		"iap4e.bin"				// using the same IAP file for both Duet WiFi and Duet Ethernet
# define WIFI_FIRMWARE_FILE	"DuetWiFiServer.bin"

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		1
#define HAS_W5500_NETWORKING	1

#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1
#define SUPPORT_TMC2660			1
#define TMC2660_USES_USART		1
#define HAS_VOLTAGE_MONITOR		1
#define HAS_VREF_MONITOR		0
#define ACTIVE_LOW_HEAT_ON		1

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			1					// set zero to disable support for FreeLSS scanners
#define SUPPORT_LASER			1					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_12864_LCD		0					// set nonzero to support 12864 LCD and rotary encoder
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_FTP				1
#define SUPPORT_TELNET			1

#define USE_CACHE				1					// set nonzero to enable the cache

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 12;				// The maximum number of drives supported directly by the electronics
constexpr size_t MaxTotalDrivers = NumDirectDrivers; // The maximum number of drives including CAN expansion
constexpr size_t MaxSmartDrivers = 10;				// The maximum number of smart drivers

constexpr size_t NumEndstops = 12;					// The number of inputs we have for endstops, filament sensors etc.
constexpr size_t NumHeaters = 8;					// The number of heaters in the machine
constexpr size_t NumExtraHeaterProtections = 8;		// The number of extra heater protection instances
constexpr size_t NumThermistorInputs = 8;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 9;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES

constexpr size_t MaxExtruders = NumDirectDrivers - MinAxes;	// The maximum number of extruders
constexpr size_t MaxDriversPerAxis = 5;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxHeatersPerTool = 8;
constexpr size_t MaxExtrudersPerTool = 8;

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_WIFI_DEVICE Serial1

constexpr Pin UsbVBusPin = 54;						// Pin used to monitor VBUS on USB port

#define I2C_IFACE	Wire							// Which TWI interface we use
#define I2C_IRQn	WIRE_ISR_ID						// The interrupt number it uses

constexpr Pin DueXnExpansionStart = 200;			// Pin numbers 200-215 are on the I/O expander
constexpr Pin AdditionalIoExpansionStart = 220;		// Pin numbers 220-235 are on the additional I/O expander

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// DRIVES
constexpr Pin GlobalTmc2660EnablePin = 38;			// The pin that drives ENN of all TMC2660 drivers on production boards (on pre-production boards they are grounded)
constexpr Pin ENABLE_PINS[NumDirectDrivers] = { 78, 41, 42, 49, 57, 87, 88, 89, 90, 31, 82, 60 };
constexpr Pin STEP_PINS[NumDirectDrivers] = { 70, 71, 72, 69, 68, 66, 65, 64, 67, 91, 84, 85 };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { 75, 76, 77, 01, 73, 92, 86, 80, 81, 32, 83, 25 };

// Pin assignments etc. using USART1 in SPI mode
Usart * const USART_TMC2660 = USART1;
constexpr uint32_t  ID_TMC2660_SPI = ID_USART1;
constexpr IRQn TMC2660_SPI_IRQn = USART1_IRQn;
# define TMC2660_SPI_Handler	USART1_Handler

constexpr Pin TMC2660MosiPin = 22;					// PA13
constexpr Pin TMC2660MisoPin = 21;					// PA22
constexpr Pin TMC2660SclkPin = 23;					// PA23

constexpr Pin DueX_SG = 96;							// DueX stallguard detect pin = PE0 (was E2_STOP)
constexpr Pin DueX_INT = 17;						// DueX interrupt pin = PA17 (was E6_STOP)

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
constexpr Pin END_STOP_PINS[NumEndstops] = { 46, 02, 93, 74, 48, 96, 97, 98, 99, 17, 39, 8 };
constexpr Pin DUEX_END_STOP_PINS[5] = { 200, 203, 202, 201, 213 };				// these replace endstops 5-9 if a DueX is present

// HEATERS
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { 45, 47, 44, 61, 62, 63, 59, 18 }; // Thermistor pin numbers
constexpr Pin HEAT_ON_PINS[NumHeaters] = { 19, 20, 16, 35, 37, 40, 43, 15 };	// Heater pin numbers (heater 7 pin TBC)

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

#if SUPPORT_ROLAND

// chrishamm's pin assignments
constexpr size_t MaxSpiTempSensors = 2;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 56, 27 };

#else

constexpr size_t MaxSpiTempSensors = 8;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 28, 50, 51, 52, 24, 97, 98, 99 };	// SPI0_CS1, SPI0_CS2, CS3, CS4, CS5, CS6, CS7, CS8

#endif

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = 79;

// Analogue pin numbers
constexpr Pin Z_PROBE_PIN = 33;												// AFE1_AD4/PC1 Z probe analog input
constexpr Pin PowerMonitorVinDetectPin = 36;								// AFE1_AD7/PC4 Vin monitor
constexpr Pin PowerMonitor5vDetectPin = 29;									// AFE1_AD1/PB3 5V regulator input monitor

constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

constexpr Pin VssaSensePin = 103;

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin Z_PROBE_MOD_PIN = 34;
constexpr Pin DiagPin = Z_PROBE_MOD_PIN;

// Cooling fans
constexpr size_t NUM_FANS = 9;
constexpr Pin COOLING_FAN_PINS[NUM_FANS] = { 55, 58, 00, 212, 207, 206, 205, 204, 215 };
constexpr size_t NumTachos = 1;
constexpr Pin TachoPins[NumTachos] = { 102 };								// PB6 on expansion connector

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = {53, NoPin};
constexpr Pin SdWriteProtectPins[NumSdCards] = {NoPin, NoPin};
constexpr Pin SdSpiCSPins[1] = {56};
constexpr uint32_t ExpectedSdCardSpeed = 20000000;

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
// This is the mapping from logical pins 60+ to firmware pin numbers
constexpr Pin SpecialPinMap[] =
{
	24, 97, 98, 99,															// We allow CS5-CS8 to be used because few users need >4 thermocouples or RTDs
	7,																		// SW_ENC on CONN_SD
	Z_PROBE_MOD_PIN
};
constexpr Pin DueX5GpioPinMap[] = { 211, 210, 209, 208 };					// Pins 100-103 map to GPIO 1-4 on DueX5
// We also allow pins 120-135 to be used if there is an additional SX1509B expander
constexpr int HighestLogicalPin = 135;										// highest logical pin number on this electronics

// SAM4E Flash locations (may be expanded in the future)
constexpr uint32_t IAP_FLASH_START = 0x00470000;
constexpr uint32_t IAP_FLASH_END = 0x0047FFFF;		// we allow a full 64K on the SAM4

// Duet pin numbers to control the WiFi interface on the Duet WiFi
constexpr Pin EspResetPin = PortEPin(4);			// Low on this in holds the WiFi module in reset (ESP_RESET)
constexpr Pin EspEnablePin = PortEPin(5);			// High to enable the WiFi module, low to power it down (ESP_CH_PD)
constexpr Pin EspDataReadyPin = PortDPin(31);		// Input from the WiFi module indicating that it wants to transfer data (ESP GPIO0)
constexpr Pin SamTfrReadyPin = PortDPin(30);		// Output from the SAM to the WiFi module indicating we can accept a data transfer (ESP GPIO4 via 7474)
constexpr Pin SamCsPin = PortAPin(11);				// SPI NPCS pin, input from WiFi module

// Duet pin numbers to control the W5500 interface on the Duet Ethernet
constexpr Pin W5500ResetPin = PortEPin(4);			// Low on this in holds the W5500 module in reset (ESP_RESET)
constexpr Pin W5500InterruptPin = PortDPin(31);		// W5500 interrupt output, active low
constexpr Pin W5500ModuleSensePin = PortAPin(5);	// URXD1, tied to ground on the Ethernet module
constexpr Pin W5500SsPin = PortAPin(11);			// SPI NPCS pin, input from W5500 module

// Timer allocation (no network timer on DuetNG)
// TC0 channel 0 is used for FAN2
// TC0 channel 1 is currently unused (may use it for a heater or a fan)
// TC0 channel 2 is available for us to use
#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(2)
#define STEP_TC_IRQN		TC2_IRQn
#define STEP_TC_HANDLER		TC2_Handler
#define STEP_TC_ID			ID_TC2

#endif
