/*
 * Pins_DuetM.h
 *
 *  Created on: 29 Nov 2017
 *      Author: David
 */

#ifndef SRC_PCCB_PINS_PCCB_H_
#define SRC_PCCB_PINS_PCCB_H_

#if defined(PCCB_10)
# define BOARD_NAME				"PC001373"
# define BOARD_SHORT_NAME		"PC001373"
# define FIRMWARE_NAME 			"RepRapFirmware for PC001373"
# define DEFAULT_BOARD_TYPE 	BoardType::PCCB_v10
#elif defined(PCCB_08_X5)
# define FIRMWARE_NAME 			"RepRapFirmware for PCCB 0.8+DueX5"
# define DEFAULT_BOARD_TYPE 	BoardType::PCCB_v08
#elif defined(PCCB_08)
# define FIRMWARE_NAME 			"RepRapFirmware for PCCB 0.8"
# define DEFAULT_BOARD_TYPE 	BoardType::PCCB_v08
#else
# error Unknown board
#endif

constexpr size_t NumFirmwareUpdateModules = 1;		// 1 module
#define IAP_FIRMWARE_FILE		"PccbFirmware.bin"
#define IAP_UPDATE_FILE			"PccbIAP.bin"
constexpr uint32_t IAP_IMAGE_START = 0x20010000;

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		0
#define HAS_W5500_NETWORKING	0

#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1					// SD card socket is optional

#if defined(PCCB_10) || defined(PCCB_08_X5)
# define SUPPORT_TMC2660		1
# define TMC2660_USES_USART		0
#endif
#if defined(PCCB_08)
# define SUPPORT_TMC22xx		1
# define TMC22xx_HAS_MUX		0
#endif

#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			1
#define HAS_VREF_MONITOR		1

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_IOBITS			0					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		0					// set nonzero to support DHT temperature/humidity sensors (requires RTOS)
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_12864_LCD		0					// set nonzero to support 12864 LCD and rotary encoder
#define SUPPORT_DOTSTAR_LED		1					// set nonzero to support DotStar LED strips
#define ALLOCATE_DEFAULT_PORTS	1

// The physical capabilities of the machine

#if defined(PCCB_10)

constexpr size_t NumDirectDrivers = 8;				// The maximum number of drives supported by the electronics (7 TMC2660, 1 dumb)
constexpr size_t MaxSmartDrivers = 7;				// The maximum number of smart drivers

#elif defined(PCCB_08_X5)

constexpr size_t NumDirectDrivers = 6;				// The maximum number of drives supported by the electronics
constexpr size_t MaxSmartDrivers = 5;				// The maximum number of smart drivers

#elif defined(PCCB_08)

constexpr size_t NumDirectDrivers = 8;				// The maximum number of drives supported by the electronics
constexpr size_t MaxSmartDrivers = 2;				// The maximum number of smart drivers

#endif

constexpr size_t MaxSensors = 32;

constexpr size_t MaxHeaters = 1;					// The number of heaters in the machine. PCCB has no heaters.
constexpr size_t MaxMonitorsPerHeater = 3;			// The maximum number of monitors per heater

constexpr size_t MaxBedHeaters = 1;
constexpr size_t MaxChamberHeaters = 1;
constexpr int8_t DefaultBedHeater = -1;
constexpr int8_t DefaultE0Heater = 0;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 2;
constexpr size_t NumTmcDriversSenseChannels = 1;

constexpr size_t MaxGpInPorts = 5;
constexpr size_t MaxGpOutPorts = 5;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 6;						// The maximum number of movement axes in the machine, <= DRIVES
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 3;					// The maximum number of extruders
constexpr size_t NumDefaultExtruders = 0;			// The number of drivers that we configure as extruders by default

constexpr size_t MaxAxesPlusExtruders = NumDirectDrivers;

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 1;

constexpr size_t MaxFans = 7;

constexpr unsigned int MaxTriggers = 16;			// Maximum number of triggers

constexpr size_t MaxSpindles = 2;					// Maximum number of configurable spindles

constexpr size_t NUM_SERIAL_CHANNELS = 1;			// The number of serial IO channels (USB only)
#define SERIAL_MAIN_DEVICE SerialUSB

// SerialUSB
constexpr Pin UsbVBusPin = PortCPin(11);			// Pin used to monitor VBUS on USB port

#define I2C_IFACE	Wire							// First and only I2C interface
#define I2C_IRQn	WIRE_ISR_ID

// Drivers

#if defined(PCCB_10)
constexpr Pin ENABLE_PINS[NumDirectDrivers] =		{ PortAPin(9),  PortAPin(10), PortBPin(14), PortCPin(25), PortCPin( 5), PortCPin(19), PortAPin( 0), PortCPin(28) };
constexpr Pin STEP_PINS[NumDirectDrivers] =			{ PortCPin(4),  PortCPin(7),  PortCPin(24), PortCPin( 2), PortCPin(22), PortCPin(20), PortCPin(10), PortCPin(14) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =	{ PortAPin(8),  PortAPin(11), PortAPin(17), PortCPin(21), PortCPin(18), PortBPin(13), PortAPin( 1), PortCPin(17) };
#endif

#if defined(PCCB_08_X5)
constexpr Pin ENABLE_PINS[NumDirectDrivers] =		{ PortBPin(14), PortCPin(25), PortCPin( 5), PortCPin(19), PortAPin( 0), PortCPin(28) };
constexpr Pin STEP_PINS[NumDirectDrivers] =			{ PortCPin(24), PortCPin( 2), PortCPin(22), PortCPin(20), PortCPin(10), PortCPin(14) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =	{ PortAPin(17), PortCPin(21), PortCPin(18), PortBPin(13), PortAPin( 1), PortCPin(17) };
#endif

#if defined(PCCB_10) || defined(PCCB_08_X5)

Spi * const SPI_TMC2660 = SPI;
constexpr uint32_t ID_TMC2660_SPI = ID_SPI;
constexpr IRQn TMC2660_SPI_IRQn = SPI_IRQn;
# define TMC2660_SPI_Handler	SPI_Handler

// Pin assignments, using USART1 in SPI mode
constexpr Pin TMC2660MosiPin = PortAPin(13);
constexpr Pin TMC2660MisoPin = PortAPin(12);
constexpr Pin TMC2660SclkPin = PortAPin(14);
constexpr Pin GlobalTmc2660EnablePin = PortCPin(16); // The pin that drives ENN of all drivers on the DueX5

constexpr uint32_t DefaultStandstillCurrentPercent = 100;					// it's not adjustable on TMC2660

#elif defined(PCCB_08)

constexpr Pin ENABLE_PINS[NumDirectDrivers] =		{ NoPin,		NoPin,		  PortBPin(14), PortCPin(25), PortCPin( 5), PortCPin(19), PortAPin( 0), PortCPin(28) };
constexpr Pin STEP_PINS[NumDirectDrivers] =			{ PortCPin( 4), PortCPin( 7), PortCPin(24), PortCPin( 2), PortCPin(22), PortCPin(20), PortCPin(10), PortCPin(14) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =	{ PortAPin( 8), PortAPin(11), PortAPin(17), PortCPin(21), PortCPin(18), PortCPin(13), PortAPin( 1), PortCPin(17) };

Uart * const TMC22xxUarts[MaxSmartDrivers] = { UART0, UART1 };
constexpr uint32_t TMC22xxUartIds[MaxSmartDrivers] = { ID_UART0, ID_UART1 };
constexpr IRQn TMC22xxUartIRQns[MaxSmartDrivers] = { UART0_IRQn, UART1_IRQn };
constexpr Pin TMC22xxUartPins[MaxSmartDrivers] = { APINS_UART0, APINS_UART1 };

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// On the PCCB we have only 2 drivers, so we use a lower baud rate to reduce the CPU load

constexpr uint32_t DriversBaudRate = 100000;
constexpr uint32_t TransferTimeout = 10;			// any transfer should complete within 10 ticks @ 1ms/tick

#define UART_TMC_DRV0_Handler	UART0_Handler
#define UART_TMC_DRV1_Handler	UART1_Handler

constexpr uint32_t DefaultStandstillCurrentPercent = 75;

#endif

#if defined(PCCB_08) || defined(PCCB_08_X5)
constexpr Pin GlobalTmc22xxEnablePin = 1;			// The pin that drives ENN of all internal drivers
#endif

// Heaters and thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortAPin(20), PortCPin(13) }; 	// thermistor pin numbers
constexpr Pin VssaSensePin = PortAPin(19);
constexpr Pin VrefSensePin = PortBPin(1);

// Default thermistor parameters - on PCCB we default both thermistors to the same parameters
constexpr float BED_R25 = 100000.0;
constexpr float BED_BETA = 4388.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4388.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float MinVrefLoadR = DefaultThermistorSeriesR / 2;		// there are 2 temperature sensing channels

// Number of SPI temperature sensors to support
constexpr size_t MaxSpiTempSensors = 1;		//TODO which SPI channels does PCCB route to the DueX?

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { PortCPin(27) };	// SPI0_CS6 if a DueX5 is connected

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = NoPin;

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortCPin(12);						// Vin monitor
constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr size_t MaxZProbes = 1;

constexpr Pin DiagPin = NoPin;

// DotStar LED control (USART0 is SharedSPI so we use USART1)
#define DOTSTAR_USES_USART	1

Usart * const DotStarUsart = USART1;
constexpr Pin DotStarMosiPin = PortAPin(22);
constexpr Pin DotStarSclkPin = PortAPin(23);
constexpr uint32_t DotStarClockId = ID_USART1;
constexpr IRQn DotStarIRQn = USART1_IRQn;

// SD cards
constexpr size_t NumSdCards = 1;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortCPin(8) };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin };
constexpr Pin SdSpiCSPins[1] = { NoPin };
constexpr uint32_t ExpectedSdCardSpeed = 15000000;

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

constexpr inline PinCapability operator|(PinCapability a, PinCapability b) noexcept
{
	return (PinCapability)((uint8_t)a | (uint8_t)b);
}

// Struct to represent a pin that can be assigned to various functions
// This can be varied to suit the hardware. It is a struct not a class so that it can be direct initialised in read-only memory.
struct PinEntry
{
	Pin GetPin() const noexcept { return pin; }
	PinCapability GetCapability() const noexcept { return cap; }
	const char* GetNames() const noexcept { return names; }

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
	// LED outputs
	{ PortCPin(0),	PinCapability::wpwm,	"led" },
	{ PortCPin(23),	PinCapability::wpwm,	"!leddim" },

#if defined(PCCB_10)
	// Fan outputs
	{ PortAPin(16),	PinCapability::wpwm,	"fan0" },
	{ PortAPin(15),	PinCapability::wpwm,	"fan1" },
	{ PortCPin(3),	PinCapability::wpwm,	"fan2" },
	{ PortBPin(2),	PinCapability::write,	"fan3" },
	{ PortBPin(3),	PinCapability::write,	"fan4" },
	{ PortCPin(1),	PinCapability::wpwm,	"fan5" },

	// Tacho inputs
	{ PortBPin(0),	PinCapability::read,	"fan5a.tach" },
	{ PortCPin(30),	PinCapability::read,	"fan5b.tach" },
#else
	// Fan outputs
	{ PortAPin(16),	PinCapability::wpwm,	"fan0" },
	{ PortCPin(3),	PinCapability::wpwm,	"fan1" },
	{ PortAPin(15),	PinCapability::wpwm,	"fan2" },
	{ PortCPin(1),	PinCapability::wpwm,	"fan3" },

	// Tacho inputs
	{ PortBPin(0),	PinCapability::read,	"fan3a.tach" },
	{ PortCPin(30),	PinCapability::read,	"fan3b.tach" },
#endif

	// Endstop inputs
#if defined(PCCB_10)
	{ PortAPin(24),	PinCapability::read,	"stop0" },
	{ PortAPin(25),	PinCapability::read,	"stop1" },
	{ PortCPin(6),	PinCapability::read,	"stop2" },
#else
	{ PortAPin(24),	PinCapability::read,	"stop0" },
	{ PortAPin(25),	PinCapability::read,	"stop1" },
	{ PortCPin(31),	PinCapability::read,	"stop2" },
#endif

	// Thermistor inputs
	{ PortAPin(20),	PinCapability::ainr,	"temp0" },
	{ PortCPin(13),	PinCapability::ainr,	"temp1" },

	// Misc expansion
	{ PortAPin(18), PinCapability::ainrw,	"exp.pa18,exp.35" },
	{ PortAPin(21), PinCapability::ainrw,	"exp.pa21,exp.36" },
	{ PortCPin(26),	PinCapability::rwpwm,	"exp.pc26,exp.13,duex.heater4" },
	{ PortCPin(27),	PinCapability::rw,		"exp.pc27,exp.9,spi.cs6,stop3"},
	{ PortCPin(29),	PinCapability::rwpwm,	"exp.pc29,exp.8,duex.heater3" }
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;

#if ALLOCATE_DEFAULT_PORTS

// Default pin allocations
constexpr const char *DefaultEndstopPinNames[] = { "stop1", "nil", "stop0" };	// stop0 is the default Z endstop, stop1 is the X endstop
constexpr const char *DefaultZProbePinNames = "nil";
constexpr const char *DefaultHeaterPinNames[] = { "nil" };
constexpr const char *DefaultGpioPinNames[] = { "led", "leddim" };

#if defined(PCCB_10)
constexpr const char *DefaultFanPinNames[] = { "fan0", "fan1", "fan2", "fan3", "fan4", "!fan5+fan5a.tach", "nil+fan5b.tach" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { DefaultFanPwmFreq, DefaultFanPwmFreq, DefaultFanPwmFreq, DefaultFanPwmFreq, DefaultFanPwmFreq, 25000 };
#else
constexpr const char *DefaultFanPinNames[] = { "fan0", "fan1", "fan2", "!fan3+fan3a.tach", "nil+fan3btach" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { DefaultFanPwmFreq, DefaultFanPwmFreq, DefaultFanPwmFreq, 25000 };
#endif

#endif

// Timer allocation
// TC0 channel 0 is used for step pulse generation and software timers (lower 16 bits)
// TC0 channel 1 is currently unused
// TC0 channel 2 is used for step pulse generation and software timers (upper 16 bits)
#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(0)
#define STEP_TC_CHAN_UPPER	(2)
#define STEP_TC_ID			ID_TC0
#define STEP_TC_ID_UPPER	ID_TC2
#define STEP_TC_IRQN		TC0_IRQn
#define STEP_TC_HANDLER		TC0_Handler

namespace StepPins
{
	// *** These next three functions must use the same bit assignments in the drivers bitmap ***
	// Each stepper driver must be assigned one bit in a 32-bit word, in such a way that multiple drivers can be stepped efficiently
	// and more or less simultaneously by doing parallel writes to several bits in one or more output ports.
	// All our step pins are on port C, so the bitmap is just the map of step bits in port C.

	// Calculate the step bit for a driver. This doesn't need to be fast. It must return 0 if the driver is remote.
	static inline uint32_t CalcDriverBitmap(size_t driver) noexcept
	{
		return (driver < NumDirectDrivers)
				? g_APinDescription[STEP_PINS[driver]].ulPin
				: 0;
	}

	// Set the specified step pins high
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversHigh(uint32_t driverMap) noexcept
	{
		PIOC->PIO_ODSR = driverMap;				// on PCCB all step pins are on port C
	}

	// Set all step pins low
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversLow() noexcept
	{
		PIOC->PIO_ODSR = 0;						// on PCCB all step pins are on port C
	}
}

#endif /* SRC_PCCB_PINS_PCCB_H_ */
