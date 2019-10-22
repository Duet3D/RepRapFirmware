#ifndef PINS_SAME70_H__
#define PINS_SAME70_H__

#define BOARD_SHORT_NAME	"MBP03"
#define FIRMWARE_NAME		"RepRapFirmware for Duet 3 v0.3"
#define DEFAULT_BOARD_TYPE	BoardType::Duet3_03
const size_t NumFirmwareUpdateModules = 4;			// 3 modules, plus one for manual upload to WiFi module (module 2 not used)

#define IAP_FIRMWARE_FILE	"Duet3Firmware_" BOARD_SHORT_NAME ".bin"
#define WIFI_FIRMWARE_FILE	"DuetWiFiServer.bin"
#define IAP_UPDATE_FILE		"Duet3iap_sd_" BOARD_SHORT_NAME ".bin"

// Features definition
#define HAS_LWIP_NETWORKING		1
#define HAS_WIFI_NETWORKING		1
#define HAS_LINUX_INTERFACE		0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_MASS_STORAGE		1
#define HAS_HIGH_SPEED_SD		1

#define SUPPORT_TMC51xx			1
#define TMC51xx_USES_USART		1

#define SUPPORT_CAN_EXPANSION	1
#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			0
#define HAS_VREF_MONITOR		1

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_LASER			1					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_FTP				1
#define SUPPORT_TELNET			1
#define SUPPORT_ASYNC_MOVES		1
#define ALLOCATE_DEFAULT_PORTS	0

#define USE_CACHE				0					// Cache controller disabled for now

#define NO_EXTRUDER_ENDSTOPS	1	// Temporary!!!

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 6;				// The maximum number of drives supported by the electronics inc. direct expansion
constexpr size_t MaxSmartDrivers = 6;				// The maximum number of direct smart drivers
constexpr size_t MaxCanDrivers = 18;
constexpr size_t MaxCanBoards = 18;

constexpr float MaxTmc5160Current = 3200.0;			// The maximum current we allow the TMC5160/5161 drivers to be set to

constexpr size_t MaxSensorsInSystem = 64;
typedef uint64_t SensorsBitmap;

constexpr size_t MaxHeaters = 12;
constexpr size_t NumExtraHeaterProtections = 8;		// The number of extra heater protection instances
constexpr size_t NumThermistorInputs = 3;			// The TEMP_2 input is unusable, so 3 not 4
constexpr size_t NumTmcDriversSenseChannels = 1;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxGpioPorts = 12;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 9;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
constexpr size_t MaxDriversPerAxis = 5;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 16;					// The maximum number of extruders
constexpr size_t NumDefaultExtruders = 3;			// The number of drivers that we configure as extruders by default

constexpr size_t MaxHeatersPerTool = 4;
constexpr size_t MaxExtrudersPerTool = 6;

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_WIFI_DEVICE Serial1

constexpr Pin UsbVBusPin = PortCPin(21);			// Pin used to monitor VBUS on USB port

// Drivers

constexpr Pin STEP_PINS[NumDirectDrivers] =			{ PortCPin(18), PortCPin(16), PortCPin(28), PortCPin(01), PortCPin(04), PortCPin(9) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =	{ PortBPin(05), PortDPin(10), PortAPin(04), PortAPin(22), PortCPin(03), PortDPin(14) };
constexpr Pin DIAG_PINS[NumDirectDrivers] =			{ PortDPin(19), PortCPin(17), PortDPin(13), PortCPin(02), PortDPin(31), PortCPin(10) };

// Pin assignments etc. using USART1 in SPI mode
constexpr Pin GlobalTmc51xxEnablePin = PortAPin(9);		// The pin that drives ENN of all TMC drivers
constexpr Pin GlobalTmc51xxCSPin = PortDPin(17);		// The pin that drives CS of all TMC drivers
Usart * const USART_TMC51xx = USART1;
constexpr uint32_t  ID_TMC51xx_SPI = ID_USART1;
constexpr IRQn TMC51xx_SPI_IRQn = USART1_IRQn;
#define TMC51xx_SPI_Handler	USART1_Handler

// These next two are #defines to avoid the need to #include DmacManager.h here
#define TMC51xx_DmaTxPerid	((uint32_t)DmaTrigSource::usart1tx)
#define TMC51xx_DmaRxPerid	((uint32_t)DmaTrigSource::usart1rx)

constexpr Pin TMC51xxMosiPin = PortBPin(4);
constexpr Pin TMC51xxMisoPin = PortAPin(21);
constexpr Pin TMC51xxSclkPin = PortAPin(23);

constexpr size_t NumPwmOutputs = 11;				// number of heater/fan/servo outputs
constexpr size_t NumInputOutputs = 9;				// number of connectors we have for endstops, filament sensors, Z probes etc.

// Thermistor/PT1000 inputs. The TEMP2 pin is left out because we had to reassign it to CAN.
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortBPin(3), PortCPin(15), PortCPin(30) };	// Thermistor/PT1000 pins
constexpr Pin VssaSensePin = PortAPin(20);
constexpr Pin VrefSensePin = PortEPin(0);

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / 4) * 4700.0/((DefaultThermistorSeriesR / 4) + 4700.0);
																			// there are 4 temperature sensing channels and a 4K7 load resistor
// Digital pins the SPI temperature sensors have their select lines tied to
constexpr Pin SpiTempSensorCsPins[] = { PortDPin(16), PortDPin(15), PortDPin(27), PortCPin(22) };

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = PortAPin(10);

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortCPin(13);
constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider (TBD)

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin DiagPin = PortCPin(20);

// Cooling fans
constexpr size_t NumTotalFans = 12;

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortAPin(6), NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { PortDPin(24) };
constexpr uint32_t ExpectedSdCardSpeed = 25000000;

// Ethernet
constexpr Pin PhyInterruptPin = PortCPin(6);
constexpr Pin PhyResetPin = PortDPin(11);

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
	// Output connectors
	{ PortAPin(7),	PinCapability::wpwm,	"out0" },
	{ PortAPin(24), PinCapability::wpwm,	"out1" },
	{ PortAPin(16),	PinCapability::wpwm,	"out2" },
	{ PortAPin(11),	PinCapability::wpwm,	"out3" },
	{ PortAPin(15),	PinCapability::wpwm,	"out4" },
	{ PortCPin(5),	PinCapability::wpwm,	"out5" },
	{ PortAPin(8),	PinCapability::wpwm,	"out6" },
	{ PortCPin(11),	PinCapability::wpwm,	"out7" },
	{ PortCPin(8),	PinCapability::wpwm,	"out8" },
	{ PortAPin(0),	PinCapability::wpwm,	"out9" },
	{ PortCPin(23),	PinCapability::wpwm,	"out10,servo" },
	{ PortAPin(10),	PinCapability::write,	"pson" },

	// Tacho inputs associated with outputs 4-6
	{ PortCPin(7),	PinCapability::read,	"out4.tach" },
	{ PortDPin(23),	PinCapability::read,	"out5.tach" },
	{ PortAPin(1),	PinCapability::read,	"out6.tach" },

	// IO connector inputs
	//TODO some have ain capability too
	{ PortDPin(30),	PinCapability::read,	"io0.in" },
	{ PortEPin(4),	PinCapability::read,	"io1.in" },
	{ PortAPin(18),	PinCapability::read,	"io2.in" },
	{ PortEPin(5),	PinCapability::read,	"io3.in" },
	{ PortAPin(17),	PinCapability::read,	"io4.in" },
	{ PortAPin(19),	PinCapability::read,	"io5.in" },
	{ PortCPin(31),	PinCapability::read,	"io6.in" },
	{ PortCPin(0),	PinCapability::read,	"io7.in" },
	{ PortEPin(3),	PinCapability::read,	"io8.in" },

	// IO connector outputs
	//TODO some have PWM capability too
	{ PortBPin(7),	PinCapability::write,	"io0.out" },
	{ PortBPin(6),	PinCapability::write,	"io1.out" },
	{ PortCPin(14),	PinCapability::write,	"io2.out" },
	{ PortAPin(3),	PinCapability::write,	"io3.out" },
	{ PortAPin(2),	PinCapability::write,	"io4.out" },
	{ PortEPin(2),	PinCapability::write,	"io5.out" },
	{ PortAPin(12),	PinCapability::write,	"io6.out" },
	{ PortCPin(29),	PinCapability::write,	"io7.out" },
	{ PortEPin(1),	PinCapability::write,	"io8.out" },

	// Thermistor inputs
	{ PortBPin(3),	PinCapability::ainr,	"temp0" },
	{ PortCPin(15),	PinCapability::ainr,	"temp1" },
	// temp2 is unusable because we had to reassign it to CAN
	{ PortCPin(30),	PinCapability::ainr,	"temp3" },

	// Misc
	{ PortDPin(16),	PinCapability::rw,		"spi.cs0" },
	{ PortDPin(15),	PinCapability::rw,		"spi.cs1" },
	{ PortDPin(27),	PinCapability::rw,		"spi.cs2" },
	{ PortCPin(22),	PinCapability::rw,		"spi.cs3" },
	{ PortDPin(24),	PinCapability::rw,		"spi.cs4" }
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted);

// SAME70 Flash locations
// These are designed to work with 1Mbyte flash processors as well as 2Mbyte
// We can only erase complete 128kb sectors on the SAME70, so we allow 128Kb for IAP
constexpr uint32_t IAP_FLASH_START = 0x004E0000;
constexpr uint32_t IAP_FLASH_END = 0x004FFFFF;

// Duet pin numbers to control the WiFi interface
constexpr Pin EspResetPin = PortAPin(5);					// Low on this in holds the WiFi module in reset (ESP_RESET)
constexpr Pin EspDataReadyPin = PortCPin(19);				// Input from the WiFi module indicating that it wants to transfer data (ESP GPIO0)
constexpr Pin SamTfrReadyPin = PortAPin(29);				// Output from the SAM to the WiFi module indicating we can accept a data transfer (ESP GPIO4 via 7474)
constexpr Pin SamCsPin = PortBPin(2);						// SPI NPCS pin, input from WiFi module
Spi * const EspSpi = SPI0;

// Timer allocation
// Network timer is timer 4 aka TC1 channel1
#define NETWORK_TC			(TC1)
#define NETWORK_TC_CHAN		(1)
#define NETWORK_TC_IRQN		TC4_IRQn
#define NETWORK_TC_HANDLER	TC4_Handler
#define NETWORK_TC_ID		ID_TC4

// Step timer is timer 2 aka TC0 channel 2
#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(2)
#define STEP_TC_IRQN		TC2_IRQn
#define STEP_TC_HANDLER		TC2_Handler
#define STEP_TC_ID			ID_TC2

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
		PIOC->PIO_ODSR = driverMap;				// on Duet 3 all step pins are on port C
	}

	// Set all step pins low
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversLow()
	{
		PIOC->PIO_ODSR = 0;						// on Duet 3 all step pins are on port C
	}
}

#endif
