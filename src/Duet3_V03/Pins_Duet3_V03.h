#ifndef PINS_SAME70_H__
#define PINS_SAME70_H__

#define FIRMWARE_NAME		"RepRapFirmware for Duet 3"
#define DEFAULT_BOARD_TYPE BoardType::Duet3_03
const size_t NumFirmwareUpdateModules = 4;		// 3 modules, plus one for manual upload to WiFi module (module 2 not used)
#define IAP_FIRMWARE_FILE	"Duet3Firmware.bin"
#define WIFI_FIRMWARE_FILE	"DuetWiFiServer.bin"
#define IAP_UPDATE_FILE		"iapduet3.bin"

// Features definition
#define HAS_LWIP_NETWORKING		1
#define HAS_WIFI_NETWORKING		1
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1

#define SUPPORT_TMC51xx			1
#define TMC51xx_USES_USART		1

#define SUPPORT_CAN_EXPANSION	1
#define HAS_VOLTAGE_MONITOR		1
#define HAS_VREF_MONITOR		1
#define ACTIVE_LOW_HEAT_ON		0

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_LASER			1
#define SUPPORT_IOBITS			1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_FTP				1
#define SUPPORT_TELNET			1
#define SUPPORT_ASYNC_MOVES		1

#define USE_CACHE				0					// Cache controller disabled for now

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 6;				// The maximum number of drives supported by the electronics inc. direct expansion
constexpr size_t MaxSmartDrivers = 6;				// The maximum number of smart drivers

constexpr size_t MaxCanDrivers = 12;				// we need to set a limit until the DDA/DMs are restructured
constexpr size_t MaxTotalDrivers = NumDirectDrivers + MaxCanDrivers;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 9;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES

constexpr size_t MaxExtruders = MaxTotalDrivers - MinAxes;	// The maximum number of extruders
constexpr size_t MaxDriversPerAxis = 5;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxHeatersPerTool = 4;
constexpr size_t MaxExtrudersPerTool = 6;

constexpr size_t NUM_SERIAL_CHANNELS = 2;			// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_WIFI_DEVICE Serial1

constexpr Pin UsbVBusPin = PortCPin(21);			// Pin used to monitor VBUS on USB port

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// DRIVES

constexpr Pin STEP_PINS[NumDirectDrivers] =			{ PortCPin(18), PortCPin(16), PortCPin(28), PortCPin(01), PortCPin(04), PortCPin(9) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =	{ PortBPin(05), PortDPin(10), PortAPin(04), PortAPin(22), PortCPin(03), PortDPin(14) };
constexpr Pin DIAG_PINS[NumDirectDrivers] =			{ PortDPin(19), PortCPin(17), PortDPin(13), PortCPin(02), PortDPin(31), PortCPin(10) };

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

constexpr size_t NumPwmOutputs = 11;				// number of heater/fan/servo outputs
constexpr size_t NumInputOutputs = 9;				// number of connectors we have for endstops, filament sensors, Z probes etc.

#if 0

// Flexible pin assignment
//TODO

#else

// The following are temporary until we implement flexible pin usage
// Assign 4 outputs to heaters and 6 to fans
// Assign 8 I/O connectors as endstop inputs and the last one as the Z probe
constexpr size_t NumHeaters = 4;
constexpr size_t NumFans = 6;
constexpr size_t NumEndstops = 8;

// Endstops
constexpr Pin END_STOP_PINS[NumEndstops] = { PortDPin(30), PortEPin(4), PortAPin(18), PortEPin(5), PortAPin(17), PortAPin(19), PortCPin(31), PortCPin(0) };

// Heater and thermistors
constexpr Pin HEAT_ON_PINS[NumHeaters] = { PortAPin(7), PortAPin(24), PortAPin(16), PortAPin(11) };

// Cooling fans
constexpr size_t NUM_FANS = 7;
constexpr Pin COOLING_FAN_PINS[NUM_FANS] = { PortAPin(15), PortCPin(5), PortAPin(8), PortCPin(11), PortCPin(8), PortAPin(0), PortCPin(23) };

constexpr Pin Z_PROBE_PIN = PortEPin(3);		// IO8
constexpr Pin Z_PROBE_MOD_PIN = PortEPin(1);	// IO8_OUT

#endif

constexpr size_t NumTachos = 3;
constexpr Pin TachoPins[NumTachos] = { PortCPin(7), PortDPin(23), PortAPin(1) };

constexpr size_t NumExtraHeaterProtections = 8;		// The number of extra heater protection instances

// Thermistor/PT1000 inputs
constexpr size_t NumThermistorInputs = 4;
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortBPin(3), PortCPin(15), PortCPin(0), PortCPin(30) };	// Thermistor/PT1000 pins

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
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { PortDPin(16), PortDPin(15), PortDPin(27), PortCPin(22) };

// Pin that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = PortAPin(10);

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortCPin(13);

constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider (TBD)

constexpr Pin VssaSensePin = PortAPin(20);
constexpr Pin VrefSensePin = PortEPin(0);

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin DiagPin = PortCPin(20);

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortAPin(6), NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { PortDPin(24) };
constexpr uint32_t ExpectedSdCardSpeed = 25000000;

// Ethernet
constexpr Pin PhyInterruptPin = PortCPin(6);
constexpr Pin PhyResetPin = PortDPin(11);

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This next definition defines the highest one.
// This is the mapping from logical pins 60+ to firmware pin numbers
constexpr Pin SpecialPinMap[] =
{
};
constexpr Pin DueX5GpioPinMap[] = {};				// TBD
constexpr int HighestLogicalPin = 50;										// highest logical pin number on this electronics

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

#endif
