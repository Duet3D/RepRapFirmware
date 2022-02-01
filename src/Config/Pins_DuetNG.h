#ifndef PINS_DUETNG_H__
#define PINS_DUETNG_H__

#include <PinDescription.h>

// Pins definition file for Duet 2 WiFi/Ethernet
// This file is normally #included by #including RepRapFirmware.h, which includes this file

#define BOARD_NAME_WIFI			"Duet 2 WiFi"
#define BOARD_NAME_ETHERNET		"Duet 2 Ethernet"
#define BOARD_NAME_SBC			"Duet 2 + SBC"
#define BOARD_SHORT_NAME_WIFI		"2WiFi"
#define BOARD_SHORT_NAME_ETHERNET	"2Ethernet"
#define BOARD_SHORT_NAME_SBC		"2SBC"

#if defined(USE_SBC)

#define FIRMWARE_NAME			"RepRapFirmware for Duet 2 SBC"
#define DEFAULT_BOARD_TYPE	 	BoardType::Duet2SBC_10
#define IAP_FIRMWARE_FILE		"Duet2Firmware_SBC.bin"
#define IAP_UPDATE_FILE_SBC		"Duet2_SBCiap32_SBC.bin"

#else

#define FIRMWARE_NAME			"RepRapFirmware for Duet 2 WiFi/Ethernet"
#define DEFAULT_BOARD_TYPE	 	BoardType::DuetWiFi_10
#define IAP_FIRMWARE_FILE		"Duet2CombinedFirmware.bin"
#define IAP_UPDATE_FILE			"Duet2_SDiap32_WiFiEth.bin"	// using the same IAP file for both Duet WiFi and Duet Ethernet
#define WIFI_FIRMWARE_FILE		"DuetWiFiServer.bin"

#endif

constexpr uint32_t IAP_IMAGE_START = 0x20018000;	// IAP is loaded into the last 32kb of RAM

// Features definition
#define HAS_LWIP_NETWORKING		0

#if defined(USE_SBC)
# define HAS_WIFI_NETWORKING	0
# define HAS_W5500_NETWORKING	0
# define HAS_SBC_INTERFACE		1
# define HAS_MASS_STORAGE		0
#else
# define HAS_WIFI_NETWORKING	1
# define HAS_W5500_NETWORKING	1
# define HAS_SBC_INTERFACE		0
#endif

#define HAS_CPU_TEMP_SENSOR		1
#if defined(USE_SBC)
# define HAS_HIGH_SPEED_SD		0
#else
# define HAS_HIGH_SPEED_SD		1
#endif
#define SUPPORT_TMC2660			1
#define TMC2660_USES_USART		1
#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			1
#define HAS_VREF_MONITOR		0
#define ACTIVE_LOW_HEAT_ON		1

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#if defined(USE_SBC)
# define SUPPORT_SCANNER		0
#else
# define SUPPORT_SCANNER		1					// set zero to disable support for FreeLSS scanners
#endif
#define SUPPORT_LASER			1					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_12864_LCD		1					// set nonzero to support 12864 LCD and rotary encoder
#if defined(USE_SBC)
# define SUPPORT_ACCELEROMETERS	0					// temporary until we can pass accelerometer data to the SBC
#else
# define SUPPORT_ACCELEROMETERS	1
#endif
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_LED_STRIPS		1

#define VARIABLE_NUM_DRIVERS	SUPPORT_12864_LCD	// nonzero means that some pins may only support drivers if not used for other purposes e.g. LCD

#if defined(USE_SBC)
# define SUPPORT_HTTP			0
# define SUPPORT_FTP			0
# define SUPPORT_TELNET			0
#else
# define SUPPORT_HTTP			1
# define SUPPORT_FTP			1
# define SUPPORT_TELNET			1
#endif

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
constexpr size_t MaxPortsPerHeater = 2;
constexpr size_t MaxMonitorsPerHeater = 3;			// The maximum number of monitors per heater

constexpr size_t MaxBedHeaters = 4;
constexpr size_t MaxChamberHeaters = 4;
constexpr int8_t DefaultE0Heater = 1;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 8;
constexpr size_t NumTmcDriversSenseChannels = 2;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxGpInPorts = 20;
constexpr size_t MaxGpOutPorts = 20;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 10;						// The maximum number of movement axes in the machine, usually just X, Y and Z
constexpr size_t MaxDriversPerAxis = 6;				// The maximum number of stepper drivers assigned to one axis (increased to 6 for Modix)

constexpr size_t MaxExtruders = 7;					// The maximum number of extruders
constexpr size_t MaxAxesPlusExtruders = 12;

constexpr size_t MaxHeatersPerTool = 8;
constexpr size_t MaxExtrudersPerTool = 8;

constexpr size_t MaxFans = 12;

constexpr unsigned int MaxTriggers = 16;			// Must be <= 32 because we store a bitmap of pending triggers in a uint32_t

constexpr size_t MaxSpindles = 4;					// Maximum number of configurable spindles

constexpr size_t NumSerialChannels = 2;				// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE	SerialUSB
#define SERIAL_AUX_DEVICE	Serial
#define SERIAL_WIFI_DEVICE	Serial1

constexpr Pin UsbVBusPin = PortCPin(22);			// Pin used to monitor VBUS on USB port

#define I2C_IFACE	Wire							// Which TWI interface we use
#define I2C_IRQn	WIRE_ISR_ID						// The interrupt number it uses

constexpr Pin DueXnExpansionStart = 32*4+6;								// Pin numbers 134-149 are on the I/O expander
constexpr Pin AdditionalIoExpansionStart = DueXnExpansionStart+16;		// Pin numbers 150-166 are on the additional I/O expander

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
constexpr GpioPinFunction TMC2660PeriphMode = GpioPinFunction::A;

constexpr uint32_t DefaultStandstillCurrentPercent = 100;					// it's not adjustable on Duet 2

constexpr Pin DueX_SG = PortEPin(0);										// DueX stallguard detect pin on older DueX boards (was E2_STOP)
constexpr Pin DueX_INT = PortAPin(17);										// DueX interrupt pin (was E6_STOP)

// Thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] =
{
	PortCPin(13), PortCPin(15), PortCPin(12),								// Duet
	PortCPin(29), PortCPin(30), PortCPin(31), PortCPin(27), PortAPin(18)	// DueX5
};

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 4700.0;
constexpr float DefaultThermistorSeriesR_DueX_v0_11 = 2200.0;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[] =
	{ PortBPin(2), PortCPin(18), PortCPin(19), PortCPin(20), PortAPin(24), PortEPin(1), PortEPin(2), PortEPin(3) };	// SPI0_CS1, SPI0_CS2, CS3, CS4, CS5, CS6, CS7, CS8

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortCPin(4);						// AFE1_AD7/PC4 Vin monitor
#if 0	// the 5V regulator input monitor pin has never been used and may be removed on future PCB revisions
constexpr Pin PowerMonitor5vDetectPin = PortBPin(3);						// AFE1_AD1/PB3 5V regulator input monitor
#endif

constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

constexpr Pin VssaSensePin = PortBPin(7);

// Z probes
constexpr Pin Z_PROBE_PIN = PortCPin(1);									// AFE1_AD4/PC1 Z probe analog input
constexpr Pin Z_PROBE_MOD_PIN = PortCPin(2);
constexpr Pin DiagPin = Z_PROBE_MOD_PIN;
constexpr bool DiagOnPolarity = true;

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortCPin(21), NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { PortCPin(24) };
constexpr IRQn SdhcIRQn = HSMCI_IRQn;
constexpr uint32_t ExpectedSdCardSpeed = 20000000;

#if SUPPORT_12864_LCD
// The ST7920 datasheet specifies minimum clock cycle time 400ns @ Vdd=4.5V, min. clock width 200ns high and 20ns low.
// This assumes that the Vih specification is met, which is 0.7 * Vcc = 3.5V @ Vcc=5V
// The Duet Maestro level shifts all 3 LCD signals to 5V, so we meet the Vih specification and can reliably run at 2MHz.
// For other electronics, there are reports that operation with 3.3V LCD signals may work if you reduce the clock frequency.
// Displays based on the ST7567 use level shifters because the ST7567 is a 3.3V device.
constexpr uint32_t LcdSpiClockFrequency = 2000000;             // 2.0MHz

constexpr Pin EncoderPinB = PortCPin(7);		// connlcd.3	-> exp2.6
constexpr Pin EncoderPinA = PortAPin(8);		// connlcd.4	-> exp2.8
constexpr Pin LcdNeopixelOutPin = PortDPin(18);	// connlcd.5	-> exp1.5
constexpr Pin LcdResetPin = PortCPin(28);		// connlcd.6	-> exp1.6
constexpr Pin LcdA0Pin = PortDPin(19);			// connlcd.7	-> exp1.7
constexpr Pin LcdCSPin = PortAPin(25);			// connlcd.8	-> exp1.8
constexpr Pin EncoderPinSw = PortDPin(20);		// connlcd.9	-> exp1.9
constexpr Pin LcdBeepPin = PortDPin(21);		// connlcd.10	-> exp1.10

// Additional spi wiring for FYSETC Mini 12864 display:
// connlcd.2 (gnd)	-> exp1.2
// connsd.1 (+5V)	-> exp1.1
// connsd.2 (gnd)	-> exp2.2
// connsd.3 (SD CS)	-> exp2.7
// connsd.4 (sck)	-> exp2.9
// connsd.5 (mosi)	-> exp2.5
// connsd.6 (miso)	-> exp2.10
#endif

// Shared SPI definitions
#define USART_SPI		1
#define USART_SSPI		USART0
#define ID_SSPI			ID_USART0

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DueX.
constexpr PinDescription PinTable[] =
{	//	TC					PWM					ADC				Capability				PinNames
	// Port A
	{ TcOutput::tioa0,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"fan2"										},	// PA00 Fan 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA01 E0_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"ystop"										},	// PA02 Y_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA03 TWD0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA04 TWCK0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA05 URXD1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA06 UTXD1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"connsd.encsw,connsd.7"						},	// PA07 ENC_SW
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"connlcd.enca,connlcd.4"					},	// PA08 Endstop 11 (was LCD ENC_A)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"urxd0"										},	// PA09 URXD0 PanelDue Dout
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"utxd0"										},	// PA10 PanelDue Din
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA11 NPCS0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA12 MISO
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA13 MOSI
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA14 SPCK
	{ TcOutput::tioa1,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"exp.heater7,exp.31,!duex.e6heat,!duex.pwm5"},	// PA15 Heater 7
	{ TcOutput::none,	PwmOutput::pwm0l2_c,AdcInput::none,		PinCapability::wpwm,	"!e1heat"									},	// PA16 Heater 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_0,	PinCapability::rw,		"exp.e6stop,exp.24"							},	// PA17 E6_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_1,	PinCapability::ainr,	"e6temp,duex.e6temp,exp.thermistor7,exp.39"	},	// PA18
	{ TcOutput::none,	PwmOutput::pwm0l0_b,AdcInput::none,		PinCapability::wpwm,	"!bedheat"									},	// PA19 Heater 0
	{ TcOutput::none,	PwmOutput::pwm0l1_b,AdcInput::none,		PinCapability::wpwm,	"!e0heat"									},	// PA20 Heater 1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA21 SPI bus 1 MISO
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA22 SPI bus 1 MOSI
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA23 SPI bus 1 SPCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs5,duex.cs5,exp.50"					},	// PA24 SPI bus 0 CS5
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA25 DIR_11 (was LCD_E)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA26 HSMCI MCDA2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA27 HSMCI MCDA3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA28 HSMCI MCCDA
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA29 HSMCI MCCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA30 HSMCI MCDA0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PA31 HSMCI MCDA1

	// Port B
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB00 SPI0 MISO
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB01 SPI0 MOSI
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs1"									},	// PB02 SPI0 CS1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_1,	PinCapability::none,	nullptr										},	// PB03 PWR_FAIL_DET1 Power fail detect 5V regulator input
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB04 TDI
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB05 TDO
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"exp.pb6,exp.29,duex.pb6"					},	// PB06 (was SWDIO)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB07 VssaSensePin (was SWCLK)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB08 Crystal
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB09 Crystal
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB10 USB DDM
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB11 USB DDP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB12 Erase
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB13 SPI0 SPCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::dac1,		PinCapability::none,	nullptr										},	// PB14 DAC 1 (E1 motor current)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB15 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB16 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB17 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB18 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB19 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB20 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB21 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB22 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB23 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB24 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB25 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB26 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB27 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB28 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB29 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB30 Not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PB31 Not on chip

	// Port C
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_14,	PinCapability::none,	nullptr										},	// PC00 E6_DIR
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_4,	PinCapability::ainr,	"zprobe.in"									},	// PC01 Z probe analog in
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::write,	"zprobe.mod"								},	// PC02 Z_PROBE_MOD Z probe mod and LED
	{ TcOutput::none,	PwmOutput::pwm0l3_b,AdcInput::none,		PinCapability::wpwm,	"exp.heater3,exp.8,!duex.e2heat,!duex.pwm1"	},	// PC03 Heater 3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_7,	PinCapability::none,	nullptr										},	// PC04 PWR_FAIL_DET2 Vin power fail detect
	{ TcOutput::tioa6,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"exp.heater4,exp.13,!duex.e3heat,!duex.pwm2"},	// PC05 Heater 4
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC06 ENN GlobalTmc2660EnablePin
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"connlcd.encb,connlcd.3"					},	// PC07 Endstop 10 (was LCD ENC_B)
	{ TcOutput::tioa7,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"exp.heater5,exp.18,!duex.e4heat,!duex.pwm3"},	// PC08 Heater 5
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC09 Y_EN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC10 Z_EN
	{ TcOutput::tioa8,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"exp.heater6,exp.23,!duex.e5heat,!duex.pwm4"},	// PC11 Heater 6
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_8,	PinCapability::ainr,	"e1temp"									},	// PC12 Thermistor 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_6,	PinCapability::ainr,	"bedtemp"									},	// PC13 Thermistor 0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"xstop"										},	// PC14 X_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_7,	PinCapability::ainr,	"e0temp"									},	// PC15 Thermistor 1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"e1stop"									},	// PC16 E1_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC17 E0_EN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs2"									},	// PC18 SPI0_CS2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs3"									},	// PC19 SPI0 CS3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs4"									},	// PC20 SPI0 CS4
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC21 SD_CD
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC22 USB_PWR_MON
	{ TcOutput::tioa3,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"fan0"										},	// PC23 Fan 0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC24 SPI0_CS0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC25 E1_EN
	{ TcOutput::tioa4,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"fan1"										},	// PC26 Fan 1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_13,	PinCapability::ainr,	"e5temp,duex.e5temp,exp.thermistor6,exp.38"	},	// PC27 Thermistor 6
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PC28 EN_11 (was LCD_RS)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_9,	PinCapability::ainr,	"e2temp,duex.e2temp,exp.thermistor3,exp.35"	},	// PC29 Thermistor 3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_10,	PinCapability::ainr,	"e3temp,duex.e3temp,exp.thermistor4,exp.36"	},	// PC30 Thermistor 4
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_11,	PinCapability::ainr,	"e4temp,duex.e4temp,exp.thermistor5,exp.37"	},	// PC31 Thermistor 5

	// PORT D
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD00 E4_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD01 E3_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD02 E2_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD03 E5_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD04 E1_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD05 E0_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD06 X_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD07 Y_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD08 Z_STEP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD09 E1_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"e0stop"									},	// PD10 E0_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD11 X_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD12 Y_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD13 Z_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD14 X_EN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::write,	"pson"										},	// PD15 PS_ON
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD16 E4_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD17 E5_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD18 EN_10 (was LCD DB7)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD19 DIR_10 (was LCD DB6)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD20 STEP 10 (was LCD DB5)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD21 STEP_11 and LCD buzzer (was LCD DB4)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD22 E3_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD23 E2_EN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD24 E3_EN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD25 E4_EN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD26 E5_EN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD27 E6_STEP Expansion PD27
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD28 E2_Dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"zstop"										},	// PD29 Z_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD30 SAM_TFR_RDY
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PD31 ESP_DATA_RDY

	// Port E
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"exp.e2stop,exp.4"							},	// PE00 E2_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"exp.e3stop,exp.9,spi.cs6,duex.cs6"			},	// PE01 E3_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"exp.e4stop,exp.14,spi.cs7,duex.cs7"		},	// PE02 E4_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"exp.e5stop,exp.19,spi.cs8,duex.cs8"		},	// PE03 E5_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PE04 ESP_RST
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr										},	// PE05 ESP_EN

	// Expansion Header 134-149
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"duex.e2stop" 								}, // E2_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"duex.e5stop"								}, // E5_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"duex.e4stop" 								}, // E4_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"duex.e3stop" 								}, // E3_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"duex.fan7"									}, // Fan 7
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"duex.fan6" 								}, // Fan 6
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"duex.fan5" 								}, // Fan 5
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"duex.fan4" 								}, // Fan 4
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"duex.gp4" 									}, // DueX GP4
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"duex.gp3" 									}, // DueX GP3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"duex.gp2" 									}, // DueX GP2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"duex.gp1" 									}, // DueX GP1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"duex.fan3" 								}, // Fan 3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"duex.e6stop" 								}, // E6_STOP
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr 									}, // DUMMY 214
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"duex.fan8" 								}, // Fan 8

	// SX1509B 150-166
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.0" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.1" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.2" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.3" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.4" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.5" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.6" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.7" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.8" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.9" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.10" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.11" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.12" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.13" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.14" 								},
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"sx1509b.15" 								},
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);
static_assert(NumNamedPins == 32+32+32+32+6+16+16);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;

// USARTs used for SPI
constexpr Pin APIN_USART_SSPI_MOSI = PortBPin(1);
constexpr GpioPinFunction USARTSPIMosiPeriphMode = GpioPinFunction::C;
constexpr Pin APIN_USART_SSPI_MISO = PortBPin(0);
constexpr GpioPinFunction USARTSPIMisoPeriphMode = GpioPinFunction::C;
constexpr Pin APIN_USART_SSPI_SCK = PortBPin(13);
constexpr GpioPinFunction USARTSPISckPeriphMode = GpioPinFunction::C;

// SD Card
constexpr Pin HsmciClockPin = PortAPin(29);
constexpr Pin HsmciOtherPins[] = { PortAPin(28), PortAPin(30), PortAPin(31), PortAPin(26), PortAPin(27) };
constexpr GpioPinFunction HsmciPinsFunction = GpioPinFunction::C;

/*
 * TWI Interfaces
 */
constexpr Pin TWI_Data = PortAPin(3);
constexpr Pin TWI_CK = PortAPin(4);
constexpr GpioPinFunction TWIPeriphMode = GpioPinFunction::A;

#define WIRE_INTERFACE		TWI0
#define WIRE_INTERFACE_ID	ID_TWI0
#define WIRE_ISR_HANDLER	TWI0_Handler
#define WIRE_ISR_ID			TWI0_IRQn

// Serial
constexpr Pin APIN_Serial0_RXD = PortAPin(9);
constexpr Pin APIN_Serial0_TXD = PortAPin(10);
constexpr GpioPinFunction Serial0PeriphMode = GpioPinFunction::A;

// Serial1
constexpr Pin APIN_Serial1_RXD = PortAPin(5);
constexpr Pin APIN_Serial1_TXD = PortAPin(6);
constexpr GpioPinFunction Serial1PeriphMode = GpioPinFunction::C;

// Duet pin numbers to control the WiFi interface on the Duet WiFi
#define ESP_SPI					SPI
#define ESP_SPI_INTERFACE_ID	ID_SPI
#define ESP_SPI_IRQn			SPI_IRQn
#define ESP_SPI_HANDLER			SPI_Handler

// Hardware IDs of the SPI transmit and receive DMA interfaces. See atsam datasheet.
const uint32_t DMA_HW_ID_SPI_TX = 1;
const uint32_t DMA_HW_ID_SPI_RX = 2;

constexpr Pin SPI_MOSI = PortAPin(13);
constexpr Pin SPI_MISO = PortAPin(12);
constexpr Pin SPI_SCK  = PortAPin(14);
constexpr Pin SPI_SS0  = PortAPin(11);
constexpr GpioPinFunction SPIPeriphMode = GpioPinFunction::A;

constexpr Pin APIN_ESP_SPI_MOSI = SPI_MOSI;
constexpr Pin APIN_ESP_SPI_MISO = SPI_MISO;
constexpr Pin APIN_ESP_SPI_SCK  = SPI_SCK;
constexpr Pin APIN_ESP_SPI_SS0  = SPI_SS0;

constexpr Pin EspResetPin = PortEPin(4);			// Low on this in holds the WiFi module in reset (ESP_RESET)
constexpr Pin EspEnablePin = PortEPin(5);			// High to enable the WiFi module, low to power it down (ESP_CH_PD)
constexpr Pin EspDataReadyPin = PortDPin(31);		// Input from the WiFi module indicating that it wants to transfer data (ESP GPIO0)
constexpr Pin SamTfrReadyPin = PortDPin(30);		// Output from the SAM to the WiFi module indicating we can accept a data transfer (ESP GPIO4 via 7474)
constexpr Pin SamCsPin = PortAPin(11);				// SPI NPCS pin, input from WiFi module

// Duet pin numbers to control the W5500 interface on the Duet Ethernet
#define W5500_SPI				SPI
#define W5500_SPI_INTERFACE_ID	ID_SPI
#define W5500_SPI_IRQn			SPI_IRQn
#define W5500_SPI_HANDLER		SPI_Handler

constexpr Pin APIN_W5500_SPI_MOSI = SPI_MOSI;
constexpr Pin APIN_W5500_SPI_MISO = SPI_MISO;
constexpr Pin APIN_W5500_SPI_SCK  = SPI_SCK;
constexpr Pin APIN_W5500_SPI_SS0  = SPI_SS0;

constexpr Pin W5500ResetPin = PortEPin(4);			// Low on this in holds the W5500 module in reset (ESP_RESET)
constexpr Pin W5500InterruptPin = PortDPin(31);		// W5500 interrupt output, active low
constexpr Pin W5500ModuleSensePin = PortAPin(5);	// URXD1, tied to ground on the Ethernet module
constexpr Pin W5500SsPin = PortAPin(11);			// SPI NPCS pin, input from W5500 module

// Duet pin numbers for the SBC interface
#define SBC_SPI					SPI
#define SBC_SPI_INTERFACE_ID	ID_SPI
#define SBC_SPI_IRQn			SPI_IRQn
#define SBC_SPI_HANDLER			SPI_Handler
constexpr Pin APIN_SBC_SPI_MOSI = SPI_MOSI;
constexpr Pin APIN_SBC_SPI_MISO = SPI_MISO;
constexpr Pin APIN_SBC_SPI_SCK  = SPI_SCK;
constexpr Pin APIN_SBC_SPI_SS0  = SPI_SS0;
constexpr GpioPinFunction SBCPinPeriphMode = SPIPeriphMode;

constexpr Pin SbcTfrReadyPin = PortDPin(31);

// Timer allocation (no network timer on DuetNG)
// TC0 channel 0 is used for FAN2
// TC0 channel 1 is currently unused (may use it for a heater or a fan)
// TC0 channel 2 is available for us to use
#define STEP_TC				(TC0)
#define STEP_TC_CHAN		(2)
#define STEP_TC_IRQN		TC2_IRQn
#define STEP_TC_HANDLER		TC2_Handler
#define STEP_TC_ID			ID_TC2

// DMA channel allocation
#if HAS_SBC_INTERFACE
constexpr DmaChannel DmacChanSbcTx = 1;
constexpr DmaChannel DmacChanSbcRx = 2;
#endif

#if HAS_WIFI_NETWORKING
constexpr DmaChannel DmacChanWiFiTx = 1;
constexpr DmaChannel DmacChanWiFiRx = 2;
#endif

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
				? 1u << (STEP_PINS[driver] & 0x1Fu)
				: 0;
	}

	// Set the specified step pins high. This needs to be fast.
	static inline __attribute__((always_inline)) void StepDriversHigh(uint32_t driverMap) noexcept
	{
		PIOD->PIO_SODR = driverMap;				// on Duet WiFi/Ethernet all step pins are on port D
	}

	// Set all step pins low. This needs to be fast.
	static inline __attribute__((always_inline)) void StepDriversLow(uint32_t driverMap) noexcept
	{
		PIOD->PIO_CODR = driverMap;				// on Duet WiFi/Ethernet all step pins are on port D
	}
}

#endif
