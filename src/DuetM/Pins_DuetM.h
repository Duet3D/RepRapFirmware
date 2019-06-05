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
#define IAP_UPDATE_FILE		"iap4s.bin"

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		0
#define HAS_W5500_NETWORKING	1

#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1
#define SUPPORT_TMC22xx			1
#define TMC22xx_HAS_MUX			1
#define HAS_VOLTAGE_MONITOR		1
#define HAS_VREF_MONITOR		1
#define ACTIVE_LOW_HEAT_ON		1

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_LASER			1					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			0					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors (requires RTOS)
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_12864_LCD		1					// set nonzero to support 12864 LCD and rotary encoder
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_FTP				1
#define SUPPORT_TELNET			1

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 7;				// The maximum number of drives supported by the electronics
constexpr size_t MaxTotalDrivers = NumDirectDrivers;
constexpr size_t MaxSmartDrivers = 7;				// The maximum number of smart drivers

constexpr size_t NumEndstops = 5;					// The number of inputs we have for endstops, filament sensors etc.
constexpr size_t NumHeaters = 3;					// The number of heaters/thermistors in the machine. Duet M has 3 heaters but 4 thermistors.
constexpr size_t NumExtraHeaterProtections = 4;		// The number of extra heater protection instances
constexpr size_t NumThermistorInputs = 4;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 6;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES

constexpr size_t MaxExtruders = NumDirectDrivers - MinAxes;	// The maximum number of extruders
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 4;

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial

// SerialUSB
constexpr Pin UsbVBusPin = 47;						// Pin used to monitor VBUS on USB port

#define I2C_IFACE	Wire							// First and only I2C interface
#define I2C_IRQn	WIRE_ISR_ID

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drivers
constexpr Pin GlobalTmc22xxEnablePin = 1;			// The pin that drives ENN of all drivers
constexpr Pin ENABLE_PINS[NumDirectDrivers] = { NoPin, NoPin, NoPin, NoPin, NoPin, 63, 61 };
constexpr Pin STEP_PINS[NumDirectDrivers] = { 56, 38, 64, 40, 41, 67, 57 };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { 54, 8, 30, 33, 42, 18, 60 };

// UART interface to stepper drivers
Uart * const UART_TMC22xx = UART0;
const IRQn TMC22xx_UART_IRQn = UART0_IRQn;
const uint32_t ID_TMC22xx_UART = ID_UART0;
const uint8_t TMC22xx_UART_PINS = APINS_UART0;
#define TMC22xx_UART_Handler	UART0_Handler

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// In testing I found that 500kbaud was not reliable, so now using 200kbaud.
const uint32_t DriversBaudRate = 200000;
const uint32_t TransferTimeout = 10;				// any transfer should complete within 10 ticks @ 1ms/tick

constexpr Pin TMC22xxMuxPins[3] = { 50, 52, 53 };	// Pins that control the UART multiplexer, LSB first

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
constexpr Pin END_STOP_PINS[NumEndstops] = { 24, 32, 46, 25, 43 };

// Heaters and thermistors
constexpr Pin HEAT_ON_PINS[NumHeaters] = { 36, 37, 16 };					// Heater pin numbers
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { 20, 26, 66, 27 }; 	// Thermistor pin numbers
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
constexpr size_t NumTachos = 1;
constexpr Pin TachoPins[NumTachos] = { 21 };

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { 44, NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { 34 };
constexpr uint32_t ExpectedSdCardSpeed = 15000000;

// 12864 LCD
// The ST7920 datasheet specifies minimum clock cycle time 400ns @ Vdd=4.5V, min. clock width 200ns high and 20ns low.
// This assumes that the Vih specification is met, which is 0.7 * Vcc = 3.5V @ Vcc=5V
// The Duet Maestro level shifts all 3 LCD signals to 5V, so we meet the Vih specification and can reliably run at 2MHz.
// For other electronics, there are reports that operation with 3.3V LCD signals may work if you reduce the clock frequency.
constexpr uint32_t LcdSpiClockFrequency = 2000000;		// 2.0MHz
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
constexpr Pin W5500ResetPin = PortCPin(13);									// Low on this in holds the W5500 in reset
constexpr Pin W5500SsPin = PortAPin(11);									// SPI NPCS pin to W5500
constexpr Pin W5500IntPin = PortAPin(23);									// Interrupt from W5500

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
