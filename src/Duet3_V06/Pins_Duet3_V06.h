#ifndef PINS_SAME70_H__
#define PINS_SAME70_H__

#define BOARD_SHORT_NAME		"MB6HC"
#define BOARD_NAME				"Duet 3 MB6HC"
#define FIRMWARE_NAME			"RepRapFirmware for Duet 3 MB6HC"
#define DEFAULT_BOARD_TYPE		BoardType::Auto

const size_t NumFirmwareUpdateModules = 1;

#define IAP_FIRMWARE_FILE		"Duet3Firmware_" BOARD_SHORT_NAME ".bin"
#define IAP_UPDATE_FILE			"Duet3_SDiap_" BOARD_SHORT_NAME ".bin"
#define IAP_UPDATE_FILE_SBC		"Duet3_SBCiap_" BOARD_SHORT_NAME ".bin"
constexpr uint32_t IAP_IMAGE_START = 0x20450000;		// last 64kb of RAM

// Features definition
#define HAS_LWIP_NETWORKING		1
#define HAS_WIFI_NETWORKING		0
#define HAS_CPU_TEMP_SENSOR		1

#define HAS_LINUX_INTERFACE		1
#define HAS_MASS_STORAGE		1
#define HAS_HIGH_SPEED_SD		1

#define SUPPORT_TMC51xx			1
#define TMC51xx_USES_USART		1

#define SUPPORT_CAN_EXPANSION	1
#define SUPPORT_DOTSTAR_LED		1
#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			0
#define HAS_12V_MONITOR			1
#define ENFORCE_MIN_V12			1
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
#define TRACK_OBJECT_NAMES		1

#define USE_MPU					1					// Needed if USE_CACHE is set, so that we can have non-cacheable memory regions
#define USE_CACHE				1

// The physical capabilities of the machine

#include <Duet3Limits.h>							// this file is in the CANlib project because both main and expansion boards need it

constexpr size_t NumDirectDrivers = 6;				// The maximum number of drives supported by the electronics inc. direct expansion
constexpr size_t MaxSmartDrivers = 6;				// The maximum number of direct smart drivers
constexpr size_t MaxCanDrivers = 20;
constexpr size_t MaxCanBoards = 20;

constexpr float MaxTmc5160Current = 6300.0;			// The maximum current we allow the TMC5160/5161 drivers to be set to

constexpr size_t MaxBedHeaters = 12;
constexpr size_t MaxChamberHeaters = 4;
constexpr int8_t DefaultE0Heater = 1;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 4;
constexpr size_t NumTmcDriversSenseChannels = 1;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 10;						// The maximum number of movement axes in the machine
constexpr size_t MaxDriversPerAxis = 5;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 16;					// The maximum number of extruders
constexpr size_t NumDefaultExtruders = 1;			// The number of drivers that we configure as extruders by default

constexpr size_t MaxAxesPlusExtruders = 20;			// May be <= MaxAxes + MaxExtruders

constexpr size_t MaxHeatersPerTool = 8;
constexpr size_t MaxExtrudersPerTool = 8;

constexpr unsigned int MaxTriggers = 32;			// Must be <= 32 because we store a bitmap of pending triggers in a uint32_t

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_WIFI_DEVICE Serial1

constexpr Pin UsbVBusPin = PortCPin(21);			// Pin used to monitor VBUS on USB port

// Drivers
constexpr Pin STEP_PINS[NumDirectDrivers] =			{ PortCPin(18), PortCPin(16), PortCPin(28), PortCPin(01), PortCPin(04), PortCPin(9) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =	{ PortBPin(05), PortDPin(10), PortAPin(04), PortAPin(22), PortCPin(03), PortDPin(14) };
constexpr Pin DIAG_PINS[NumDirectDrivers] =			{ PortDPin(29), PortCPin(17), PortDPin(13), PortCPin(02), PortDPin(31), PortCPin(10) };

// Pin assignments etc. using USART1 in SPI mode
constexpr Pin GlobalTmc51xxEnablePin = PortAPin(9);		// The pin that drives ENN of all TMC drivers
constexpr Pin GlobalTmc51xxCSPin = PortDPin(17);			// The pin that drives CS of all TMC drivers
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

constexpr uint32_t DefaultStandstillCurrentPercent = 71;

// Thermistor/PT1000 inputs
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortCPin(15), PortCPin(29), PortCPin(30), PortCPin(31) };	// Thermistor/PT1000 pins
constexpr Pin VssaSensePin = PortCPin(13);
constexpr Pin VrefSensePin = PortCPin(0);

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / 4) * 4700.0/((DefaultThermistorSeriesR / 4) + 4700.0);
																			// there are 4 temperature sensing channels and a 4K7 load resistor
// Digital pins the SPI temperature sensors have their select lines tied to
constexpr Pin SpiTempSensorCsPins[] = { PortAPin(5), PortAPin(6), PortDPin(20), PortCPin(22) };

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = PortAPin(10);

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortAPin(20);
constexpr Pin PowerMonitorV12DetectPin = PortEPin(4);
constexpr float PowerMonitorVoltageRange = (60.4 + 4.7)/4.7 * 3.3;			// voltage divider ratio times the reference voltage
constexpr float V12MonitorVoltageRange = (60.4 + 4.7)/4.7 * 3.3;			// voltage divider ratio times the reference voltage

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin DiagPin = PortCPin(20);

// SD cards
constexpr size_t NumSdCards = 1;
constexpr Pin SdCardDetectPins[1] = { PortAPin(29) };
constexpr Pin SdWriteProtectPins[1] = { NoPin };
constexpr Pin SdSpiCSPins[1] = { NoPin };
constexpr uint32_t ExpectedSdCardSpeed = 25000000;

// DotStar LED control
#define DOTSTAR_USES_USART	0

constexpr Pin DotStarMosiPin = PortAPin(13);
constexpr Pin DotStarSclkPin = PortAPin(14);
constexpr uint32_t DotStarClockId = ID_QSPI;
constexpr IRQn DotStarIRQn = QSPI_IRQn;
const uint32_t DotStarSpiClockFrequency = 100000;		// try sending at 100kHz

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
//TODO change the table below for the V0.6 board
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
	{ PortAPin(12),	PinCapability::wpwm,	"out9,laser,vfd" },
	{ PortAPin(10),	PinCapability::write,	"pson" },

	// Tacho inputs associated with outputs 4-6
	{ PortCPin(7),	PinCapability::read,	"out4.tach" },
	{ PortDPin(23),	PinCapability::read,	"out5.tach" },
	{ PortAPin(1),	PinCapability::read,	"out6.tach" },

	// IO connector inputs
	{ PortDPin(25),	PinCapability::read,	"io0.in,serial0.rx" },
	{ PortDPin(15),	PinCapability::read,	"io1.in,serial1.rx" },
	{ PortDPin(28),	PinCapability::read,	"io2.in,i2c0.clk" },
	{ PortEPin(5),	PinCapability::ainr,	"io3.in" },
	{ PortDPin(30),	PinCapability::ainr,	"io4.in" },
	{ PortAPin(19),	PinCapability::ainr,	"io5.in" },
	{ PortAPin(18),	PinCapability::ainr,	"io6.in" },
	{ PortAPin(17),	PinCapability::ainr,	"io7.in" },
	{ PortEPin(3),	PinCapability::read,	"io8.in" },		// analog in not usable because it is on the wrong ADC

	// IO connector outputs
	{ PortDPin(26),	PinCapability::rw,		"io0.out,serial0.tx" },
	{ PortDPin(16),	PinCapability::rw,		"io1.out,serial1.tx" },
	{ PortDPin(27),	PinCapability::rw,		"io2.out,i2c0.dat" },
	{ PortAPin(3),	PinCapability::rw,		"io3.out" },
	{ PortEPin(0),	PinCapability::rwpwm,	"io4.out" },
	{ PortDPin(21),	PinCapability::rwpwm,	"io5.out" },
	{ PortAPin(0),	PinCapability::rw,		"io6.out" },
	{ PortCPin(23),	PinCapability::rwpwm,	"io7.out" },
	{ PortEPin(1),	PinCapability::rw,		"io8.out" },	// this pin could be PWM capable but shares the TC with io7.out, so the PWM frequencies are not independent

	// Thermistor inputs
	{ PortCPin(15), PinCapability::ainr,	"temp0" },
	{ PortCPin(29),	PinCapability::ainr,	"temp1" },
	{ PortCPin(30), PinCapability::ainr,	"temp2" },
	{ PortCPin(31), PinCapability::ainr,	"temp3" },

	// Misc
	{ PortAPin(5),	PinCapability::rw,		"spi.cs0,serial3.rx" },
	{ PortAPin(6),	PinCapability::rw,		"spi.cs1,serial3.tx" },
	{ PortDPin(20),	PinCapability::rw,		"spi.cs2" },
	{ PortCPin(22),	PinCapability::rw,		"spi.cs3" }
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;

// Duet pin numbers for the Linux interface
constexpr Pin LinuxTfrReadyPin = PortEPin(2);
Spi * const LinuxSpi = SPI1;

// Timer allocation
// Step timer is timer 0 aka TC0 channel 0. Also used as the CAN timestamp counter.
#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(0)					// channel for lower 16 bits
#define STEP_TC_CHAN_UPPER	(2)					// channel for upper 16 bits
#define STEP_TC_IRQN		TC0_IRQn
#define STEP_TC_HANDLER		TC0_Handler
#define STEP_TC_ID			ID_TC0
#define STEP_TC_ID_UPPER	ID_TC2

// DMA channel allocation
constexpr uint8_t DmacChanHsmci = 0;			// this is hard coded in the ASF HSMCI driver
constexpr uint8_t DmacChanWiFiTx = 1;			// only on v0.3 board
constexpr uint8_t DmacChanWiFiRx = 2;			// only on v0.3 board
constexpr uint8_t DmacChanTmcTx = 3;
constexpr uint8_t DmacChanTmcRx = 4;
constexpr uint8_t DmacChanLinuxTx = 5;
constexpr uint8_t DmacChanLinuxRx = 6;
constexpr uint8_t DmacChanDotStarTx = 7;

constexpr size_t NumDmaChannelsUsed = 8;

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
		PIOC->PIO_ODSR = driverMap;				// on Duet 3 all step pins are on port C
	}

	// Set all step pins low
	// This needs to be as fast as possible, so we do a parallel write to the port(s).
	// We rely on only those port bits that are step pins being set in the PIO_OWSR register of each port
	static inline void StepDriversLow() noexcept
	{
		PIOC->PIO_ODSR = 0;						// on Duet 3 all step pins are on port C
	}
}

#endif
