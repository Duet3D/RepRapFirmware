#ifndef PINS_SAME70_H__
#define PINS_SAME70_H__

#define FIRMWARE_NAME		"RepRapFirmware for SAME70-XPLD"
#define DEFAULT_BOARD_TYPE BoardType::SAME70XPLD_0
const size_t NumFirmwareUpdateModules = 4;		// 3 modules, plus one for manual upload to WiFi module (module 2 not used)
#define IAP_FIRMWARE_FILE	"SAME70XPLDFirmware.bin"
#define WIFI_FIRMWARE_FILE	"DuetWiFiServer.bin"
#define IAP_UPDATE_FILE		"iapsame70xpld.bin"

// Features definition
#define HAS_LWIP_NETWORKING		1
#define HAS_WIFI_NETWORKING		0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1

#define SUPPORT_CAN_EXPANSION	0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_VREF_MONITOR		0

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_LASER			1					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		0					// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_FTP				1
#define SUPPORT_TELNET			1
#define SUPPORT_ASYNC_MOVES		1

#define USE_CACHE				0					// Cache controller disabled for now

#define NO_EXTRUDER_ENDSTOPS	1	// Temporary!!!

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 5;				// The maximum number of drives supported by the electronics

constexpr size_t MaxSensorsInSystem = 32;
typedef uint32_t SensorsBitmap;

constexpr size_t MaxHeaters = 6;
constexpr size_t MaxExtraHeaterProtections = 6;		// The number of extra heater protection instances

constexpr size_t MaxBedHeaters = 4;
constexpr size_t MaxChamberHeaters = 4;
constexpr int8_t DefaultBedHeater = 0;
constexpr int8_t DefaultE0Heater = 1;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 4;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxGpioPorts = 12;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 5;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= MaxAxesPlusExtruders
constexpr size_t MaxDriversPerAxis = 5;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 5;					// The maximum number of extruders
constexpr size_t NumDefaultExtruders = 2;			// The number of drivers that we configure as extruders by default

constexpr size_t MaxAxesPlusExtruders = NumDirectDrivers;

constexpr size_t MaxHeatersPerTool = 4;
constexpr size_t MaxExtrudersPerTool = 6;

constexpr size_t MaxFans = 12;

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_WIFI_DEVICE Serial1

constexpr Pin UsbVBusPin = NoPin;					// Pin used to monitor VBUS on USB port

//TWI is disabled for now on the SAM7E until we rewrite the driver
//#define I2C_IFACE	Wire							// Which TWI interface we use

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drivers

constexpr Pin ENABLE_PINS[NumDirectDrivers] = { NoPin, NoPin, NoPin, NoPin, NoPin };
constexpr Pin STEP_PINS[NumDirectDrivers] = { PortCPin(19), PortCPin(19), PortCPin(19), PortCPin(19), PortCPin(19) };	// Do not use NoPin in this list! Code assumes all on port C
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { NoPin, NoPin, NoPin, NoPin, NoPin };

// Heater and thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortCPin(31), PortDPin(30), PortCPin(13), PortCPin(30) };
																			// Thermistor pin numbers (labelled AD1-2 and AD4-5 on XPLD, but AD5 has a 0R resistor missing)
// Default thermistor parameters
constexpr float BED_R25 = 100000.0;
constexpr float BED_BETA = 3988.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4388.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 4700.0;

// Number of SPI temperature sensors to support

constexpr size_t MaxSpiTempSensors = 4;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { NoPin, NoPin, NoPin, NoPin };

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = NoPin;

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = NoPin;								// TBD

constexpr Pin VssaSensePin = NoPin;
constexpr Pin VrefSensePin = NoPin;

// Diagnostic LED pin
constexpr Pin DiagPin = NoPin;												// TBD

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortCPin(16), NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { NoPin };
constexpr uint32_t ExpectedSdCardSpeed = 25000000;

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
	// Sample pin table entry
	{ PortAPin(0),	PinCapability::rw,	"pa0" },
	// Lots more pins need to be defined here...
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted);

// Default pin allocations
constexpr const char *DefaultEndstopPinNames[] = { "nil" };
constexpr const char *DefaultZProbePinNames = "^zprobe.in+zprobe.mod";
constexpr const char *DefaultHeaterPinNames[] = { "nil" };
constexpr const char *DefaultFanPinNames[] = { "nil" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { DefaultFanPwmFreq };

// SAME70 Flash locations
// These are designed to work with 1Mbyte flash processors as well as 2Mbyte
// We can only erase complete 128kb sectors on the SAME70, so we allow 128Kb for IAP
constexpr uint32_t IAP_FLASH_START = 0x004E0000;
constexpr uint32_t IAP_FLASH_END = 0x004FFFFF;

// Duet pin numbers to control the WiFi interface
constexpr Pin EspResetPin = PortBPin(1);					// Low on this in holds the WiFi module in reset (ESP_RESET)
constexpr Pin EspDataReadyPin = PortAPin(19);				// Input from the WiFi module indicating that it wants to transfer data (ESP GPIO0)
constexpr Pin SamTfrReadyPin = PortCPin(31);				// Output from the SAM to the WiFi module indicating we can accept a data transfer (ESP GPIO4 via 7474)
constexpr Pin SamCsPin = PortBPin(2);						// SPI NPCS pin, input from WiFi module

// Timer allocation
#define NETWORK_TC			(TC1)
#define NETWORK_TC_CHAN		(1)
#define NETWORK_TC_IRQN		TC1_IRQn
#define NETWORK_TC_HANDLER	TC1_Handler
#define NETWORK_TC_ID		ID_TC1

#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(0)					// channel for lower 16 bits
#define STEP_TC_CHAN_UPPER	(2)					// channel for upper 16 bits
#define STEP_TC_IRQN		TC0_IRQn
#define STEP_TC_HANDLER		TC0_Handler
#define STEP_TC_ID			ID_TC0
#define STEP_TC_ID_UPPER	ID_TC2

// DMA channel allocation
constexpr uint8_t DmacChanHsmci = 0;			// this is hard coded in the ASF HSMCI driver
constexpr uint8_t DmacChanWiFiTx = 1;
constexpr uint8_t DmacChanWiFiRx = 2;
constexpr uint8_t DmacChanTmcTx = 3;
constexpr uint8_t DmacChanTmcRx = 4;

constexpr size_t NumDmaChannelsUsed = 5;

namespace StepPins
{
	// *** These next three functions must use the same bit assignments in the drivers bitmap ***
	// Each stepper driver must be assigned one bit in a 32-bit word, in such a way that multiple drivers can be stepped efficiently
	// and more or less simultaneously by doing parallel writes to several bits in one or more output ports.
	// Assume that all our step pins are on port C, so the bitmap is just the map of step bits in port C.

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
		PIOC->PIO_ODSR = driverMap;				// on SAME70XPLD all step pins are on port C
	}

	// Set all step pins low
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversLow()
	{
		PIOC->PIO_ODSR = 0;						// on SAME70XPLD all step pins are on port C
	}
}

#endif
