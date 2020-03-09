#ifndef PINS_DUET_H__
#define PINS_DUET_H__

#define FIRMWARE_NAME "RepRapFirmware for RADDS"
#define IAP_FIRMWARE_FILE "RepRapFirmware-RADDS.bin"

#define IAP_IN_RAM				0

#if IAP_IN_RAM

// TODO

#else

constexpr uint32_t IAP_IMAGE_START = 0x000F0000;
constexpr uint32_t IAP_IMAGE_END = 0x000FFBFF;		// don't touch the last 1KB, it's used for NvData
# define IAP_UPDATE_FILE "iapradds.bin"

#endif

const size_t NumFirmwareUpdateModules = 1;

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		0
#define HAS_CPU_TEMP_SENSOR		0				// enabling the CPU temperature sensor disables Due pin 13 due to bug in SAM3X
#define HAS_HIGH_SPEED_SD		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_VREF_MONITOR		0

const size_t NumFirmwareUpdateModules = 1;
#define IAP_FIRMWARE_FILE "RepRapFirmware-RADDS.bin"

#define IAP_IN_RAM				0

#if IAP_IN_RAM

# define IAP_UPDATE_FILE "RaddsIAP.bin"
constexpr uint32_t IAP_IMAGE_START = 0x20008000u;

#else

# define IAP_UPDATE_FILE "iapradds.bin"
constexpr uint32_t IAP_IMAGE_START = 0x000F0000;
constexpr uint32_t IAP_IMAGE_END = 0x000FFBFF;		// don't touch the last 1KB, it's used for NvData

#endif

// Default board type
#define DEFAULT_BOARD_TYPE BoardType::RADDS_15
#define ELECTRONICS "RADDS"

#define SUPPORT_INKJET			0				// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0				// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0				// set nonzero to support FreeLSS scanners
#define SUPPORT_IOBITS			0				// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		0				// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_OBJECT_MODEL	1

// The physical capabilities of the machine

// The number of drives in the machine, including X, Y, and Z plus extruder drives
constexpr size_t NumDirectDrivers = 9;

constexpr size_t MaxSensors = 32;

constexpr size_t MaxHeaters = 3;
constexpr size_t MaxExtraHeaterProtections = 4;		// The number of extra heater protection instances

constexpr size_t MaxBedHeaters = 1;
constexpr size_t MaxChamberHeaters = 2;
constexpr int8_t DefaultBedHeater = 0;
constexpr int8_t DefaultE0Heater = 1;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 4;

constexpr size_t MaxZProbes = 2;
constexpr size_t MaxGpioPorts = 10;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 6;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 5;					// The maximum number of extruders
constexpr size_t NumDefaultExtruders = 2;			// The number of drivers that we configure as extruders by default

constexpr size_t MaxAxesPlusExtruders = 9;

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 5;

constexpr size_t MaxFans = 12;

constexpr unsigned int MaxTriggers = 16;			// Must be <= 32 because we store a bitmap of pending triggers in a uint32_t

constexpr size_t NUM_SERIAL_CHANNELS = 2;
// Use TX0/RX0 for the auxiliary serial line
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial1

// SerialUSB
constexpr Pin UsbVBusPin = NoPin;					// Pin used to monitor VBUS on USB port. Not needed for SAM3X.

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.
// DRIVES
//			                                    	X   Y   Z  E1  E2  E3  E4  E5  E6
constexpr Pin ENABLE_PINS[NumDirectDrivers] =    { 26, 22, 15, 62, 65, 49, 37, 31, 68 };
//			                                   	   A15 A12 A09 A02 B19 C12 C03 D06 B16
constexpr Pin STEP_PINS[NumDirectDrivers] =      { 24, 17,  2, 61, 64, 51, 35, 29, 67 };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { 23, 16,  3, 60, 63, 53, 33, 27, 66 };

// Analogue pin numbers
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { 4, 0, 1, 2 };

#if 0	// the following are no longer used, left here for reference until PinTable is defined

// Endstops
// E Stops not currently used
// Note: RepRapFirmware only has a single endstop per axis
//       gcode defines if it is a max ("high end") or min ("low end")
//       endstop.  gcode also sets if it is active HIGH or LOW
//
// 28 = RADDS X min
// 30 = RADDS Y min
// 32 = RADDS Z min
// 39 = RADDS PWM3
//
// This leaves 34, 36, and 38 as spare pins (X, Y, Z max)

constexpr Pin END_STOP_PINS[NumEndstops] = { 28, 30, 32, 39 };

// Heater outputs
// Bed PMW: D7 has hardware PWM so bed has PWM
// h0, h1 PMW: D13 & D12 are on TIOB0 & B8 which are both TC B channels, so they get PWM
// h2 bang-bang: D11 is on TIOA8 which is a TC A channel shared with h1, it gets bang-bang control

constexpr Pin HEAT_ON_PINS[NumTotalHeaters] = { 7, 13, 12, 11 };	// bed, h0, h1, h2

// Use a PWM capable pin
constexpr Pin COOLING_FAN_PINS[NUM_FANS] = { 9, 8 }; // Fan 0, Fan 1

// Firmware will attach a FALLING interrupt to this pin
// see FanInterrupt() in Platform.cpp
//
// D25 -- Unused GPIO on AUX1
constexpr Pin TachoPins[NumTachos] = { 25 };

// Definition of which pins we allow to be controlled using M42
// Spare pins on the Arduino Due are
//
//  D5 / TIOA6  / C.25
//  D6 / PWML7  / C.24
// ### Removed: now E0_AXIS endstop D39 / PWMH2  / C.7
// D58 / AD3    / A.6
// D59 / AD2    / A.4
// ### Removed: now E6_DIR ExtV3 D66 / DAC0   / B.15
// ### Removed: now E6_STP ExtV3 D67 / DAC1   / B.16
// ### Removed: now E6_EN ExtV3 D68 / CANRX0 / A.1
// ### Removed: now MSi(=3.3V) ExtV3 D69 / CANTX0 / A.0
// D70 / SDA1   / A.17
// D71 / SCL1   / A.18
// D72 / RX LED / C.30
// D73 / TX LED / A.21

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This is the mapping from logical pins 60+ to firmware pin numbers
constexpr Pin SpecialPinMap[] =
{
	5, 6, 58, 59,
	70, 71, 72, 73
};

#endif

// Default thermistor betas
constexpr float BED_R25 = 10000.0;
constexpr float BED_BETA = 4066.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4066.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 4700.0;

constexpr size_t MaxSpiTempSensors = 2;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 38, 36 };

// Digital pin number that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = 40;

// Z Probe pin
// Must be an ADC capable pin. Can be any of the ARM's A/D capable pins even a non-Arduino pin.
constexpr Pin Z_PROBE_PIN = A5;  // RADDS "ADC" pin

// Digital pin number to turn the IR LED on (high) or off (low)
// D34 -- unused X-max on RADDS
constexpr Pin Z_PROBE_MOD_PIN = 34;
constexpr Pin DiagPin = NoPin;

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { 14, 14 };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[2] = { 87, 77 };

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
	// Endstops
	{ 28,	PinCapability::read,	"xmin" },
	{ 30,	PinCapability::read,	"ymin" },
	{ 32,	PinCapability::read,	"zmin" },
	// Lots more pins need to be defined here...
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);

// Default pin allocations
constexpr const char *DefaultEndstopPinNames[] = { "xmin", "ymin", "zmin" };
constexpr const char *DefaultZProbePinNames = "^zprobe.in+zprobe.mod";
constexpr const char *DefaultFanPinNames[] = { "fan0", "fan1" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { DefaultFanPwmFreq };

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted);

// Timer allocation
#define NETWORK_TC			(TC1)
#define NETWORK_TC_CHAN		(1)
#define NETWORK_TC_IRQN		TC4_IRQn
#define NETWORK_TC_HANDLER	TC4_Handler
#define NETWORK_TC_ID		ID_TC4

#define STEP_TC				(TC1)
#define STEP_TC_CHAN		(0)
#define STEP_TC_IRQN		TC3_IRQn
#define STEP_TC_HANDLER		TC3_Handler
#define STEP_TC_ID			ID_TC3

namespace StepPins
{
	// *** These next three functions must use the same bit assignments in the drivers bitmap ***
	// Each stepper driver must be assigned one bit in a 32-bit word, in such a way that multiple drivers can be stepped efficiently
	// and more or less simultaneously by doing parallel writes to several bits in one or more output ports.
	//  Step pins are PA2,9,12,15 PB16,19 PC3,12 PD6
	//	PC12 clashes with PA12 so we shift PC3,12 left one bit

	// Calculate the step bit for a driver. This doesn't need to be fast. It must return 0 if the driver is remote.
	static inline uint32_t CalcDriverBitmap(size_t driver)
	{
		if (driver >= NumDirectDrivers)
		{
			return 0;
		}

		const PinDescription& pinDesc = g_APinDescription[STEP_PINS[driver]];
		return (pinDesc.pPort == PIOC) ? pinDesc.ulPin << 1 : pinDesc.ulPin;
	}

	// Set the specified step pins high
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversHigh(uint32_t driverMap)
	{
		PIOA->PIO_ODSR = driverMap;
		PIOB->PIO_ODSR = driverMap;
		PIOD->PIO_ODSR = driverMap;
		PIOC->PIO_ODSR = driverMap >> 1;		// do this last, it means the processor doesn't need to preserve the register containing driverMap
	}

	// Set all step pins low
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversLow()
	{
		PIOD->PIO_ODSR = 0;
		PIOC->PIO_ODSR = 0;
		PIOB->PIO_ODSR = 0;
		PIOA->PIO_ODSR = 0;
	}
}

#endif
