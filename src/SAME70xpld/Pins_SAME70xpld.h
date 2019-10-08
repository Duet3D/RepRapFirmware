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
#define HAS_WIFI_NETWORKING		1
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1

#define HAS_VOLTAGE_MONITOR		0
#define HAS_VREF_MONITOR		0
#define ACTIVE_LOW_HEAT_ON		0

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_IOBITS			1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59

#define USE_CACHE				0					// Cache controller disabled for now

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 5;				// The maximum number of drives supported by the electronics
constexpr size_t MaxTotalDrivers = 5;				// The maximum number of smart drivers

constexpr size_t NumEndstops = 5;					// The number of inputs we have for endstops, filament sensors etc.
constexpr size_t NumHeaters = 4;					// The number of heaters in the machine; 0 is the heated bed even if there isn't one
constexpr size_t NumExtraHeaterProtections = 8;		// The number of extra heater protection instances
constexpr size_t NumThermistorInputs = 4;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 9;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES

constexpr size_t MaxExtruders = NumDirectDrivers - MinAxes;	// The maximum number of extruders
constexpr size_t MaxDriversPerAxis = 5;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxHeatersPerTool = 4;
constexpr size_t MaxExtrudersPerTool = 6;

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_WIFI_DEVICE Serial1

constexpr Pin UsbVBusPin = NoPin;					// Pin used to monitor VBUS on USB port

//TWI is disabled for now on the SAM7E until we rewrite the driver
//#define I2C_IFACE	Wire							// Which TWI interface we use

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// DRIVES

constexpr Pin ENABLE_PINS[NumDirectDrivers] = { NoPin, NoPin, NoPin, NoPin, NoPin };
constexpr Pin STEP_PINS[NumDirectDrivers] = { PortCPin(19), PortCPin(19), PortCPin(19), PortCPin(19), PortCPin(19) };	// Do not use NoPin in this list! Code assumes all on port C
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { NoPin, NoPin, NoPin, NoPin, NoPin };

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
constexpr Pin END_STOP_PINS[NumEndstops] = { NoPin, NoPin, NoPin, NoPin, NoPin };

// Heater and thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortCPin(31), PortDPin(30), PortCPin(13), PortCPin(30) };
																			// Thermistor pin numbers (labelled AD1-2 and AD4-5 on XPLD, but AD5 has a 0R resistor missing)
constexpr Pin HEAT_ON_PINS[NumHeaters] = { NoPin, NoPin, NoPin, NoPin };	// Heater pin numbers (TBD)

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

constexpr size_t MaxSpiTempSensors = 4;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { NoPin, NoPin, NoPin, NoPin };

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = NoPin;

// Analogue pin numbers
constexpr Pin Z_PROBE_PIN = NoPin;											// TBD
constexpr Pin PowerMonitorVinDetectPin = NoPin;								// TBD

constexpr Pin VssaSensePin = NoPin;
constexpr Pin VrefSensePin = NoPin;

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin Z_PROBE_MOD_PIN = NoPin;
constexpr Pin DiagPin = NoPin;												// TBD

// Cooling fans
constexpr size_t NUM_FANS = 1;
constexpr Pin COOLING_FAN_PINS[NUM_FANS] = { NoPin };
constexpr size_t NumTachos = 1;
constexpr Pin TachoPins[NumTachos] = { NoPin };								// TBD

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortCPin(16), NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { NoPin };
constexpr uint32_t ExpectedSdCardSpeed = 25000000;

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This next definition defines the highest one.
// This is the mapping from logical pins 60+ to firmware pin numbers
constexpr Pin SpecialPinMap[] =
{
};
constexpr int HighestLogicalPin = 50;										// highest logical pin number on this electronics

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
#define NETWORK_TC			(TC0)
#define NETWORK_TC_CHAN		(0)
#define NETWORK_TC_IRQN		TC0_IRQn
#define NETWORK_TC_HANDLER	TC0_Handler
#define NETWORK_TC_ID		ID_TC0

#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(1)
#define STEP_TC_IRQN		TC1_IRQn
#define STEP_TC_HANDLER		TC1_Handler
#define STEP_TC_ID			ID_TC1

// DMA channel allocation
constexpr uint8_t DmacChanHsmci = 0;			// this is hard coded in the ASF HSMCI driver
constexpr uint8_t DmacChanWiFiTx = 1;
constexpr uint8_t DmacChanWiFiRx = 2;
constexpr uint8_t DmacChanTmcTx = 3;
constexpr uint8_t DmacChanTmcRx = 4;

constexpr size_t NumDmaChannelsUsed = 5;

#endif
