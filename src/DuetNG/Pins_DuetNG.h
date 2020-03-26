#ifndef PINS_DUETNG_H__
#define PINS_DUETNG_H__

// Pins definition file for Duet 2 WiFi/Ethernet
// This file is normally #included by #including RepRapFirmware.h, which includes this file

#define BOARD_NAME_WIFI			"Duet 2 WiFi"
#define BOARD_NAME_ETHERNET		"Duet 2 Ethernet"
#define BOARD_SHORT_NAME_WIFI	"2WiFi"
#define BOARD_SHORT_NAME_ETHERNET	"2Ethernet"

#define FIRMWARE_NAME			"RepRapFirmware for Duet 2 WiFi/Ethernet"
#define DEFAULT_BOARD_TYPE	 	BoardType::DuetWiFi_10
#define IAP_FIRMWARE_FILE		"Duet2CombinedFirmware.bin"

constexpr uint32_t IAP_IMAGE_START = 0x20010000;	// IAP is loaded into the second 64kb of RAM
#define IAP_UPDATE_FILE			"Duet2CombinedIAP.bin"	// using the same IAP file for both Duet WiFi and Duet Ethernet
#define WIFI_FIRMWARE_FILE		"DuetWiFiServer.bin"

constexpr size_t NumFirmwareUpdateModules = 4;		// 3 modules, plus one for manual upload to WiFi module (module 2 is now unused)

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		1
#define HAS_W5500_NETWORKING	1

#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1
#define SUPPORT_TMC2660			1
#define TMC2660_USES_USART		1
#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			1
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
#define SUPPORT_ASYNC_MOVES		1
#define ALLOCATE_DEFAULT_PORTS	0
#define TRACK_OBJECT_NAMES		1

#define USE_CACHE				1					// set nonzero to enable the cache
#define USE_MPU					0					// set nonzero to enable the memory protection unit

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 12;				// The maximum number of drives supported directly by the electronics
constexpr size_t MaxSmartDrivers = 10;				// The maximum number of smart drivers

constexpr size_t MaxSensors = 32;

constexpr size_t MaxHeaters = 10;					// The maximum number of heaters in the machine
constexpr size_t MaxMonitorsPerHeater = 3;			// The maximum number of monitors per heater

constexpr size_t MaxBedHeaters = 4;
constexpr size_t MaxChamberHeaters = 4;
constexpr int8_t DefaultBedHeater = 0;
constexpr int8_t DefaultE0Heater = 1;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 8;
constexpr size_t NumTmcDriversSenseChannels = 2;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxGpInPorts = 10;
constexpr size_t MaxGpOutPorts = 10;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 10;						// The maximum number of movement axes in the machine, usually just X, Y and Z
constexpr size_t MaxDriversPerAxis = 5;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 7;					// The maximum number of extruders
constexpr size_t NumDefaultExtruders = 1;			// The number of drivers that we configure as extruders by default

constexpr size_t MaxAxesPlusExtruders = 12;

constexpr size_t MaxHeatersPerTool = 8;
constexpr size_t MaxExtrudersPerTool = 8;

constexpr size_t MaxFans = 12;

constexpr unsigned int MaxTriggers = 16;			// Must be <= 32 because we store a bitmap of pending triggers in a uint32_t

constexpr size_t MaxSpindles = 4;					// Maximum number of configurable spindles

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_WIFI_DEVICE Serial1

constexpr Pin UsbVBusPin = PortCPin(22);			// Pin used to monitor VBUS on USB port

#define I2C_IFACE	Wire							// Which TWI interface we use
#define I2C_IRQn	WIRE_ISR_ID						// The interrupt number it uses

constexpr Pin DueXnExpansionStart = 200;			// Pin numbers 200-215 are on the I/O expander
constexpr Pin AdditionalIoExpansionStart = 220;		// Pin numbers 220-235 are on the additional I/O expander

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drives
constexpr Pin GlobalTmc2660EnablePin = PortCPin(6);	// The pin that drives ENN of all TMC2660 drivers on production boards (on pre-production boards they are grounded)
constexpr Pin ENABLE_PINS[NumDirectDrivers] =
{
	PortDPin(14), PortCPin(9), PortCPin(10), PortCPin(17), PortCPin(25),	// Duet
	PortDPin(23), PortDPin(24), PortDPin(25), PortDPin(26), PortBPin(14),	// DueX5
	PortDPin(18), PortCPin(28)												// CONN_LCD
};
constexpr Pin STEP_PINS[NumDirectDrivers] =
{
	PortDPin(6), PortDPin(7), PortDPin(8), PortDPin(5), PortDPin(4),		// Duet
	PortDPin(2), PortDPin(1), PortDPin(0), PortDPin(3), PortDPin(27),		// DueX5
	PortDPin(20), PortDPin(21)												// CONN_LCD
};
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =
{	PortDPin(11), PortDPin(12), PortDPin(13), PortAPin(1), PortDPin(9),		// Duet
	PortDPin(28), PortDPin(22), PortDPin(16), PortDPin(17), PortCPin(0),	// DueX5
	PortDPin(19), PortAPin(25)												// CONN_LCD
};

// Pin assignments etc. using USART1 in SPI mode
Usart * const USART_TMC2660 = USART1;
constexpr uint32_t  ID_TMC2660_SPI = ID_USART1;
constexpr IRQn TMC2660_SPI_IRQn = USART1_IRQn;
# define TMC2660_SPI_Handler	USART1_Handler

constexpr Pin TMC2660MosiPin = PortAPin(22);
constexpr Pin TMC2660MisoPin = PortAPin(21);
constexpr Pin TMC2660SclkPin = PortAPin(23);

constexpr uint32_t DefaultStandstillCurrentPercent = 100;					// it's not adjustable on Duet 2

constexpr Pin DueX_SG = PortEPin(0);				// DueX stallguard detect pin = PE0 (was E2_STOP)
constexpr Pin DueX_INT = PortAPin(17);				// DueX interrupt pin = PA17 (was E6_STOP)

// Thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] =
{
	PortCPin(13), PortCPin(15), PortCPin(12),								// Duet
	PortCPin(29), PortCPin(30), PortCPin(31), PortCPin(27), PortAPin(18)	// DueX5
};

// Default thermistor parameters
constexpr float BED_R25 = 100000.0;
constexpr float BED_BETA = 3988.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4388.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 4700.0;
constexpr float MinVrefLoadR = DefaultThermistorSeriesR / 8;		// there are 8 temperature sensing channels

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[] =
	{ PortBPin(2), PortCPin(18), PortCPin(19), PortCPin(20), PortAPin(24), PortEPin(1), PortEPin(2), PortEPin(3) };	// SPI0_CS1, SPI0_CS2, CS3, CS4, CS5, CS6, CS7, CS8

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = PortDPin(15);

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortCPin(4);						// AFE1_AD7/PC4 Vin monitor
constexpr Pin PowerMonitor5vDetectPin = PortBPin(3);						// AFE1_AD1/PB3 5V regulator input monitor

constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

constexpr Pin VssaSensePin = PortBPin(7);

// Z probes
constexpr Pin Z_PROBE_PIN = PortCPin(1);									// AFE1_AD4/PC1 Z probe analog input
constexpr Pin Z_PROBE_MOD_PIN = PortCPin(2);
constexpr Pin DiagPin = Z_PROBE_MOD_PIN;

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortCPin(21), NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { PortCPin(24) };
constexpr uint32_t ExpectedSdCardSpeed = 20000000;

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
	// Duet 2 and DueX heater outputs
	{ PortAPin(19),	PinCapability::wpwm,	"!bedheat" },
	{ PortAPin(20), PinCapability::wpwm,	"!e0heat" },
	{ PortAPin(16), PinCapability::wpwm,	"!e1heat" },
	{ PortCPin(3),	PinCapability::wpwm,	"exp.heater3,exp.8,!duex.e2heat,!duex.pwm1" },
	{ PortCPin(5),	PinCapability::wpwm,	"exp.heater4,exp.13,!duex.e3heat,!duex.pwm2" },
	{ PortCPin(8),	PinCapability::wpwm,	"exp.heater5,exp.18,!duex.e4heat,!duex.pwm3" },
	{ PortCPin(11),	PinCapability::wpwm,	"exp.heater6,exp.23,!duex.e5heat,!duex.pwm4" },
	{ PortAPin(15),	PinCapability::wpwm,	"exp.heater7,exp.31,!duex.e6heat,!duex.pwm5" },

	// Duet 2 and DueX fan outputs
	{ PortCPin(23),	PinCapability::wpwm,	"fan0" },
	{ PortCPin(26),	PinCapability::wpwm,	"fan1" },
	{ PortAPin(0),	PinCapability::wpwm,	"fan2" },
	{ 212,			PinCapability::wpwm,	"duex.fan3" },
	{ 207,			PinCapability::wpwm,	"duex.fan4" },
	{ 206,			PinCapability::wpwm,	"duex.fan5" },
	{ 205,			PinCapability::wpwm,	"duex.fan6" },
	{ 204,			PinCapability::wpwm,	"duex.fan7" },
	{ 215,			PinCapability::wpwm,	"duex.fan8" },

	// Endstop inputs
	{ PortCPin(14),	PinCapability::read,	"xstop" },
	{ PortAPin(02),	PinCapability::read,	"ystop" },
	{ PortDPin(29),	PinCapability::read,	"zstop" },
	{ PortDPin(10),	PinCapability::read,	"e0stop" },
	{ PortCPin(16),	PinCapability::read,	"e1stop" },
	{ PortEPin(0),	PinCapability::rw,		"exp.e2stop,exp.4" },
	{ PortEPin(1),	PinCapability::rw,		"exp.e3stop,exp.9,spi.cs6,duex.cs6" },
	{ PortEPin(2),	PinCapability::rw,		"exp.e4stop,exp.14,spi.cs7,duex.cs7" },
	{ PortEPin(3),	PinCapability::rw,		"exp.e5stop,exp.19,spi.cs8,duex.cs8" },
	{ PortAPin(17),	PinCapability::rw,		"exp.e6stop,exp.24" },
	{ 200,			PinCapability::read,	"duex.e2stop" },
	{ 203,			PinCapability::read,	"duex.e3stop" },
	{ 202,			PinCapability::read,	"duex.e4stop" },
	{ 201,			PinCapability::read,	"duex.e5stop" },
	{ 213,			PinCapability::read,	"duex.e6stop" },

	// Thermistor inputs
	{ PortCPin(13),	PinCapability::ainr,	"bedtemp" },
	{ PortCPin(15),	PinCapability::ainr,	"e0temp" },
	{ PortCPin(12),	PinCapability::ainr,	"e1temp" },
	{ PortCPin(29),	PinCapability::ainr,	"e2temp,duex.e2temp,exp.thermistor3,exp.35" },
	{ PortCPin(30),	PinCapability::ainr,	"e3temp,duex.e3temp,exp.thermistor4,exp.36" },
	{ PortCPin(31),	PinCapability::ainr,	"e4temp,duex.e4temp,exp.thermistor5,exp.37" },
	{ PortCPin(27),	PinCapability::ainr,	"e5temp,duex.e5temp,exp.thermistor6,exp.38" },
	{ PortAPin(18),	PinCapability::ainr,	"e6temp,duex.e6temp,exp.thermistor7,exp.39" },

	// SPI CS pins
	{ PortBPin(2),	PinCapability::rw,		"spi.cs1" },
	{ PortCPin(18),	PinCapability::rw,		"spi.cs2" },
	{ PortCPin(19),	PinCapability::rw,		"spi.cs3" },
	{ PortCPin(20),	PinCapability::rw,		"spi.cs4" },
	{ PortAPin(24),	PinCapability::rw,		"spi.cs5,duex.cs5,exp.50" },

	// Misc
	{ Z_PROBE_PIN,	PinCapability::ainr,	"zprobe.in" },
	{ Z_PROBE_MOD_PIN, PinCapability::write, "zprobe.mod" },
	{ ATX_POWER_PIN, PinCapability::write,	"pson" },
	{ PortCPin(7),	PinCapability::rw,		"connlcd.encb,connlcd.3" },
	{ PortAPin(8),	PinCapability::rw,		"connlcd.enca,connlcd.4" },
	{ PortAPin(7),	PinCapability::rw,		"connsd.encsw,connsd.7" },
	{ PortBPin(6),	PinCapability::rw,		"exp.pb6,exp.29,duex.pb6" },
	{ 211,			PinCapability::rwpwm,	"duex.gp1" },
	{ 210,			PinCapability::rwpwm,	"duex.gp2" },
	{ 209,			PinCapability::rwpwm,	"duex.gp3" },
	{ 208,			PinCapability::rwpwm,	"duex.gp4" },
	{ 220,			PinCapability::rwpwm,	"sx1509b.0" },
	{ 221,			PinCapability::rwpwm,	"sx1509b.1" },
	{ 222,			PinCapability::rwpwm,	"sx1509b.2" },
	{ 223,			PinCapability::rwpwm,	"sx1509b.3" },
	{ 224,			PinCapability::rwpwm,	"sx1509b.4" },
	{ 225,			PinCapability::rwpwm,	"sx1509b.5" },
	{ 226,			PinCapability::rwpwm,	"sx1509b.6" },
	{ 227,			PinCapability::rwpwm,	"sx1509b.7" },
	{ 228,			PinCapability::rwpwm,	"sx1509b.8" },
	{ 229,			PinCapability::rwpwm,	"sx1509b.9" },
	{ 230,			PinCapability::rwpwm,	"sx1509b.10" },
	{ 231,			PinCapability::rwpwm,	"sx1509b.11" },
	{ 232,			PinCapability::rwpwm,	"sx1509b.12" },
	{ 233,			PinCapability::rwpwm,	"sx1509b.13" },
	{ 234,			PinCapability::rwpwm,	"sx1509b.14" },
	{ 235,			PinCapability::rwpwm,	"sx1509b.15" }
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;

#if ALLOCATE_DEFAULT_PORTS

// Default pin allocations
constexpr const char *DefaultEndstopPinNames[] = { "xstop", "ystop", "zstop" };
constexpr const char *DefaultZProbePinNames = "^zprobe.in+zprobe.mod";
constexpr const char *DefaultFanPinNames[] = { "fan0", "fan1", "fan2" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { DefaultFanPwmFreq };

#endif

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

namespace StepPins
{
	// *** These next three functions must use the same bit assignments in the drivers bitmap ***
	// Each stepper driver must be assigned one bit in a 32-bit word, in such a way that multiple drivers can be stepped efficiently
	// and more or less simultaneously by doing parallel writes to several bits in one or more output ports.
	// All our step pins are on port D, so the bitmap is just the map of step bits in port D.

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
		PIOD->PIO_ODSR = driverMap;				// on Duet WiFi/Ethernet all step pins are on port D
	}

	// Set all step pins low
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversLow() noexcept
	{
		PIOD->PIO_ODSR = 0;						// on Duet WiFi/Ethernet all step pins are on port D
	}
}

#endif
