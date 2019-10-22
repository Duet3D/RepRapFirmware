/*
 * Pins_DuetM.h
 *
 *  Created on: 29 Nov 2017
 *      Author: David
 */

#ifndef SRC_DUETM_PINS_DUETM_H_
#define SRC_DUETM_PINS_DUETM_H_

#define FIRMWARE_NAME "RepRapFirmware for Duet 2 Maestro"
#define DEFAULT_BOARD_TYPE BoardType::DuetM_10
constexpr size_t NumFirmwareUpdateModules = 1;		// 1 module
#define IAP_FIRMWARE_FILE	"DuetMaestroFirmware.bin"
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
#define ENFORCE_MAX_VIN			0
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
#define SUPPORT_ASYNC_MOVES		1
#define ALLOCATE_DEFAULT_PORTS	1

#define NO_EXTRUDER_ENDSTOPS	1	// Temporary!!!

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 7;				// The maximum number of drives supported by the electronics
constexpr size_t MaxSmartDrivers = 7;				// The maximum number of smart drivers

constexpr size_t MaxSensorsInSystem = 32;
typedef uint32_t SensorsBitmap;

constexpr size_t MaxHeaters = 4;					// The maximum number of heaters in the machine
constexpr size_t NumExtraHeaterProtections = 4;		// The number of extra heater protection instances
constexpr size_t NumThermistorInputs = 4;
constexpr size_t NumTmcDriversSenseChannels = 2;

constexpr size_t MaxZProbes = 2;
constexpr size_t MaxGpioPorts = 10;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 6;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 4;					// The maximum number of extruders
constexpr size_t NumDefaultExtruders = 4;			// The number of drivers that we configure as extruders by default

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 4;

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial

// SerialUSB
constexpr Pin UsbVBusPin = PortCPin(11);			// Pin used to monitor VBUS on USB port

#define I2C_IFACE	Wire							// First and only I2C interface
#define I2C_IRQn	WIRE_ISR_ID

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drivers
constexpr Pin GlobalTmc22xxEnablePin = 1;			// The pin that drives ENN of all drivers
constexpr Pin ENABLE_PINS[NumDirectDrivers] = { NoPin, NoPin, NoPin, NoPin, NoPin, PortCPin(27), PortCPin(25) };
constexpr Pin STEP_PINS[NumDirectDrivers] = { PortCPin(20), PortCPin(2), PortCPin(28), PortCPin(4), PortCPin(5), PortCPin(31), PortCPin(21) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { PortCPin(18), PortAPin(8), PortBPin(4), PortBPin(7), PortCPin(6), PortAPin(18), PortCPin(24) };

// UART interface to stepper drivers
Uart * const UART_TMC22xx = UART0;
constexpr IRQn TMC22xx_UART_IRQn = UART0_IRQn;
constexpr uint32_t ID_TMC22xx_UART = ID_UART0;
constexpr uint8_t TMC22xx_UART_PINS = APINS_UART0;
#define TMC22xx_UART_Handler	UART0_Handler

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// In testing I found that 500kbaud was not reliable, so now using 200kbaud.
constexpr uint32_t DriversBaudRate = 200000;
constexpr uint32_t TransferTimeout = 10;				// any transfer should complete within 10 ticks @ 1ms/tick

constexpr Pin TMC22xxMuxPins[3] = { PortCPin(14), PortCPin(16), PortCPin(17) };	// Pins that control the UART multiplexer, LSB first

// Thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortAPin(20), PortBPin(0), PortCPin(30), PortBPin(1) }; 	// Thermistor pin numbers
constexpr Pin VssaSensePin = PortAPin(19);
constexpr Pin VrefSensePin = PortAPin(17);

// Default thermistor parameters
constexpr float BED_R25 = 100000.0;
constexpr float BED_BETA = 3988.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4388.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / 4) * 4700.0/((DefaultThermistorSeriesR / 4) + 4700.0);
																			// there are 4 temperature sensing channels and a 4K7 load resistor
// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[] = { PortBPin(14), PortCPin(19) };		// SPI0_CS1, SPI0_CS2

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = PortAPin(0);

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortCPin(12);						// Vin monitor
constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin Z_PROBE_PIN = PortCPin(15);									// Z probe analog input
constexpr Pin Z_PROBE_MOD_PIN = PortCPin(26);
constexpr Pin DiagPin = Z_PROBE_MOD_PIN;

// Cooling fans
constexpr size_t NumTotalFans = 4;

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortCPin(8), NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { PortCPin(2) };
constexpr uint32_t ExpectedSdCardSpeed = 15000000;

// 12864 LCD
// The ST7920 datasheet specifies minimum clock cycle time 400ns @ Vdd=4.5V, min. clock width 200ns high and 20ns low.
// This assumes that the Vih specification is met, which is 0.7 * Vcc = 3.5V @ Vcc=5V
// The Duet Maestro level shifts all 3 LCD signals to 5V, so we meet the Vih specification and can reliably run at 2MHz.
// For other electronics, there are reports that operation with 3.3V LCD signals may work if you reduce the clock frequency.
constexpr uint32_t LcdSpiClockFrequency = 2000000;		// 2.0MHz
constexpr Pin LcdCSPin = PortCPin(9);
constexpr Pin LcdBeepPin = PortAPin(15);
constexpr Pin EncoderPinA = PortBPin(5);
constexpr Pin EncoderPinB = PortCPin(3);
constexpr Pin EncoderPinSw = PortAPin(7);

// Enum to represent allowed types of pin access
// We don't have a separate bit for servo, because Duet PWM-capable ports can be used for servos if they are on the Duet main board
enum class PinCapability: uint8_t
{
	// Individual capabilities
	read = 1,
	ain = 2,
	write = 4,
	pwm = 8,

	// Combinations
	ainr = 1|2,
	rw = 1|4,
	wpwm = 4|8,
	rwpwm = 1|4|8,
	ainrw = 1|2|4,
	ainrwpwm = 1|2|4|8
};

constexpr inline PinCapability operator|(PinCapability a, PinCapability b)
{
	return (PinCapability)((uint8_t)a | (uint8_t)b);
}

// Struct to represent a pin that can be assigned to various functions
// This can be varied to suit the hardware. It is a struct not a class so that it can be direct initialised in read-only memory.
struct PinEntry
{
	Pin GetPin() const { return pin; }
	PinCapability GetCapability() const { return cap; }
	const char* GetNames() const { return names; }

	Pin pin;
	PinCapability cap;
	const char *names;
};

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DueX.
constexpr PinEntry PinTable[] =
{
	// Heater outputs
	{ PortCPin(0),	PinCapability::wpwm,	"!bedheat" },
	{ PortCPin(1),	PinCapability::wpwm,	"!e0heat" },
	{ PortAPin(16), PinCapability::wpwm,	"!e1heat" },

	// Fan outputs
	{ PortCPin(23),	PinCapability::wpwm,	"fan0" },
	{ PortCPin(22),	PinCapability::wpwm,	"fan1" },
	{ PortCPin(29),	PinCapability::wpwm,	"fan2" },

	// Endstop inputs
	{ PortAPin(24),	PinCapability::read,	"xstop" },
	{ PortBPin(6),	PinCapability::read,	"ystop" },
	{ PortCPin(10),	PinCapability::read,	"zstop" },
	{ PortAPin(25),	PinCapability::read,	"e0stop" },
	{ PortCPin(7),	PinCapability::read,	"e1stop" },

	// Thermistor inputs
	{ PortAPin(20),	PinCapability::ainr,	"bedtemp" },
	{ PortBPin(0),	PinCapability::ainr,	"e0temp" },
	{ PortCPin(30), PinCapability::ainr,	"e1temp" },
	{ PortBPin(1),	PinCapability::ainr,	"ctemp" },

	// SPI CS signals on the daughter board connector
	{ PortBPin(14),	PinCapability::rw,		"spi.cs1" },
	{ PortCPin(19),	PinCapability::rw,		"spi.cs2" },

	// Misc
	{ Z_PROBE_PIN,	PinCapability::ainr,	"zprobe.in" },
	{ Z_PROBE_MOD_PIN, PinCapability::write, "zprobe.mod,servo" },
	{ ATX_POWER_PIN, PinCapability::write,	"pson" },
	{ PortAPin(21), PinCapability::ainrw,	"exp.pa21" },
	{ PortAPin(22), PinCapability::ainrw,	"exp.pa22" },
	{ PortAPin(3),	PinCapability::rw,		"exp.pa3,twd0" },
	{ PortAPin(4),	PinCapability::rw,		"exp.pa4,twck0" },
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted);

// Default pin allocations
constexpr const char *DefaultEndstopPinNames[] = { "xstop", "ystop", "zstop" };
constexpr const char *DefaultZProbePinNames = "^zprobe.in+zprobe.mod";
constexpr const char *DefaultFanPinNames[] = { "fan0", "fan1", "fan2" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { DefaultFanPwmFreq };

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

namespace StepPins
{
	// *** These next three functions must use the same bit assignments in the drivers bitmap ***
	// Each stepper driver must be assigned one bit in a 32-bit word, in such a way that multiple drivers can be stepped efficiently
	// and more or less simultaneously by doing parallel writes to several bits in one or more output ports.
	// All our step pins are on port C, so the bitmap is just the map of step bits in port C.

	// Calculate the step bit for a driver. This doesn't need to be fast. It must return 0 if the driver is remote.
	static inline uint32_t CalcDriverBitmap(size_t driver)
	{
		return (driver < NumDirectDrivers)
				? g_APinDescription[STEP_PINS[driver]].ulPin
				: 0;
	}

	// Set the specified step pins high
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversHigh(uint32_t driverMap)
	{
		PIOC->PIO_ODSR = driverMap;				// on Duet Maestro all step pins are on port C
	}

	// Set all step pins low
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversLow()
	{
		PIOC->PIO_ODSR = 0;						// on Duet Maestro all step pins are on port C
	}
}

#endif /* SRC_DUETM_PINS_DUETM_H_ */
