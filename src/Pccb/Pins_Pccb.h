/*
 * Pins_DuetM.h
 *
 *  Created on: 29 Nov 2017
 *      Author: David
 */

#ifndef SRC_DUETM_PINS_DUETM_H_
#define SRC_DUETM_PINS_DUETM_H_

#ifdef PCCB_X5
# define FIRMWARE_NAME "RepRapFirmware for PCCB+DueX5"
#else
# define FIRMWARE_NAME "RepRapFirmware for PCCB"
#endif

#define DEFAULT_BOARD_TYPE BoardType::PCCB_10
constexpr size_t NumFirmwareUpdateModules = 1;		// 1 module
#define IAP_FIRMWARE_FILE	"PccbFirmware.bin"
#define IAP_UPDATE_FILE		"iap4s.bin"

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		0
#define HAS_W5500_NETWORKING	0

#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1					// SD card socket is optional

#ifdef PCCB_X5
# define SUPPORT_TMC2660		1
# define TMC2660_USES_USART		0
#else
# define SUPPORT_TMC22xx		1
# define TMC22xx_HAS_MUX		0
#endif

#define HAS_VOLTAGE_MONITOR		1
#define HAS_VREF_MONITOR		1
#define ACTIVE_LOW_HEAT_ON		1					// although we have no heaters, this matters because we treat the LEDs as heaters

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_IOBITS			0					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		0					// set nonzero to support DHT temperature/humidity sensors (requires RTOS)
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_12864_LCD		0					// set nonzero to support 12864 LCD and rotary encoder
#define SUPPORT_DOTSTAR_LED		1					// set nonzero to support DotStar LED strips

// The physical capabilities of the machine

#ifdef PCCB_X5

constexpr size_t NumDirectDrivers = 6;				// The maximum number of drives supported by the electronics
constexpr size_t MaxSmartDrivers = 5;				// The maximum number of smart drivers

#else

constexpr size_t NumDirectDrivers = 8;				// The maximum number of drives supported by the electronics
constexpr size_t MaxSmartDrivers = 2;				// The maximum number of smart drivers

#endif

constexpr size_t MaxTotalDrivers = NumDirectDrivers;

constexpr size_t NumEndstops = 4;					// The number of inputs we have for endstops, filament sensors etc.
constexpr size_t NumHeaters = 2;					// The number of heaters in the machine. PCCB has no heaters, but we pretend that the LED pins are heaters.
constexpr size_t NumExtraHeaterProtections = 4;		// The number of extra heater protection instances
constexpr size_t NumThermistorInputs = 2;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 6;						// The maximum number of movement axes in the machine, <= DRIVES

constexpr size_t MaxExtruders = NumDirectDrivers - MinAxes;	// The maximum number of extruders
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t NUM_SERIAL_CHANNELS = 1;			// The number of serial IO channels (USB only)
#define SERIAL_MAIN_DEVICE SerialUSB

// SerialUSB
constexpr Pin UsbVBusPin = 47;						// Pin used to monitor VBUS on USB port

#define I2C_IFACE	Wire							// First and only I2C interface
#define I2C_IRQn	WIRE_ISR_ID

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drivers
constexpr Pin GlobalTmc22xxEnablePin = 1;			// The pin that drives ENN of all internal drivers

#ifdef PCCB_X5

constexpr Pin GlobalTmc2660EnablePin = 52;			// The pin that drives ENN of all drivers on the DueX5
constexpr Pin ENABLE_PINS[NumDirectDrivers] = { 61, 35, 41, 55, 0, 64 };
constexpr Pin STEP_PINS[NumDirectDrivers] = { 60, 38, 58, 56, 46, 50 };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { 17, 57, 54, 34, 1, 53 };

Spi * const SPI_TMC2660 = SPI;
constexpr uint32_t ID_TMC2660_SPI = ID_SPI;
constexpr IRQn TMC2660_SPI_IRQn = SPI_IRQn;
# define TMC2660_SPI_Handler	SPI_Handler

// Pin assignments, using USART1 in SPI mode
constexpr Pin TMC2660MosiPin = 13;					// PA13
constexpr Pin TMC2660MisoPin = 12;					// PA12
constexpr Pin TMC2660SclkPin = 14;					// PA14

#else

constexpr Pin ENABLE_PINS[NumDirectDrivers] = { NoPin, NoPin, 61, 35, 41, 55, 0, 64 };
constexpr Pin STEP_PINS[NumDirectDrivers] = { 40, 43, 60, 38, 58, 56, 46, 50 };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { 8, 11, 17, 57, 54, 34, 1, 53 };

Uart * const TMC22xxUarts[MaxSmartDrivers] = { UART0, UART1 };
constexpr uint32_t TMC22xxUartIds[MaxSmartDrivers] = { ID_UART0, ID_UART1 };
constexpr IRQn TMC22xxUartIRQns[MaxSmartDrivers] = { UART0_IRQn, UART1_IRQn };
constexpr Pin TMC22xxUartPins[MaxSmartDrivers] = { APINS_UART0, APINS_UART1 };

#endif

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// On the PCCB we have only 2 drivers, so we use a lower baud rate to reduce the CPU load

const uint32_t DriversBaudRate = 100000;
const uint32_t TransferTimeout = 10;						// any transfer should complete within 10 ticks @ 1ms/tick

#define UART_TMC_DRV0_Handler	UART0_Handler
#define UART_TMC_DRV1_Handler	UART1_Handler

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
constexpr Pin END_STOP_PINS[NumEndstops] = { 25, 67, 24, 63 };

// Heaters and thermistors
constexpr Pin HEAT_ON_PINS[NumHeaters] = { 36, 59 };					// these are actually the LED control pins
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { 20, 49 }; 	// Thermistor pin numbers
constexpr Pin VssaSensePin = 19;
constexpr Pin VrefSensePin = 27;

// Default thermistor parameters - on PCCB we default both thermistors to the same parameters
constexpr float BED_R25 = 100000.0;
constexpr float BED_BETA = 4388.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4388.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float THERMISTOR_SERIES_RS = 2200.0;

// Number of SPI temperature sensors to support
constexpr size_t MaxSpiTempSensors = 1;		//TODO which SPI channels does PCCB route to the DueX?

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 63 };				// SPI0_CS6 if a DueX5 is connected

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = NoPin;

// Analogue pin numbers
constexpr Pin Z_PROBE_PIN = NoPin;											// Z probe analog input
constexpr Pin PowerMonitorVinDetectPin = 48;								// Vin monitor
constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin Z_PROBE_MOD_PIN = NoPin;
constexpr Pin DiagPin = NoPin;

// Cooling fans
constexpr size_t NUM_FANS = 4;
constexpr size_t NumTachos = 2;
constexpr Pin COOLING_FAN_PINS[NUM_FANS] = { 16, 39, 15, 37 };				// PWML2, PWML3, TIOA1, PWML1
constexpr Pin TachoPins[NumTachos] = { 26, 66 };

// Main LED control
constexpr size_t NumLeds = 2;												// number of main LEDs
constexpr Pin LedOnPins[NumLeds] = { 36, 59 };								// LED control pins

// DotStar LED control (USART0 is SharedSPI,
Usart * const DotStarUsart = USART1;
constexpr uint32_t DotStarUsartId = ID_USART1;
constexpr Pin DotStarMosiPin = 22;
constexpr Pin DotStarSclkPin = 23;
constexpr IRQn DotStarUsartIRQn = USART1_IRQn;
const uint32_t DotStarSpiClockFrequency = 100000;		// try sending at 100kHz

// SD cards
constexpr size_t NumSdCards = 1;
constexpr Pin SdCardDetectPins[NumSdCards] = { 44 };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin };
constexpr Pin SdSpiCSPins[1] = { NoPin };
constexpr uint32_t ExpectedSdCardSpeed = 15000000;

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This next definition defines the highest one.
// This is the mapping from logical pins 60+ to firmware pin numbers
constexpr Pin SpecialPinMap[] =
{
	18, 21, 51, 59, 62, 65													// PA18/AD1, PA21/AD8, PC15/AD11, PC22, PC23, PC29
};

constexpr int HighestLogicalPin = 65;										// highest logical pin number on this electronics

// SAM4S Flash locations (may be expanded in the future)
constexpr uint32_t IAP_FLASH_START = 0x00470000;
constexpr uint32_t IAP_FLASH_END = 0x0047FFFF;								// we allow a full 64K on the SAM4

// Timer allocation
// TC0 channel 0 is used for step pulse generation and software timers
// TC0 channel 1 is currently unused
// TC0 channel 2 is currently unused
#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(0)
#define STEP_TC_ID			ID_TC0
#define STEP_TC_IRQN		TC0_IRQn
#define STEP_TC_HANDLER		TC0_Handler

#endif /* SRC_DUETM_PINS_DUETM_H_ */
