#ifndef PINS_DUET3_MB6XD_H__
#define PINS_DUET3_MB6XD_H__

#include <PinDescription.h>

#define BOARD_SHORT_NAME		"MB6XD"
#define BOARD_NAME				"Duet 3 MB6XD"
#define DEFAULT_BOARD_TYPE		BoardType::Auto
#define FIRMWARE_NAME			"RepRapFirmware for Duet 3 MB6XD"
#define IAP_FIRMWARE_FILE		"Duet3Firmware_" BOARD_SHORT_NAME ".bin"

#define IAP_UPDATE_FILE			"Duet3_SDiap32_" BOARD_SHORT_NAME ".bin"
#define IAP_UPDATE_FILE_SBC		"Duet3_SBCiap32_" BOARD_SHORT_NAME ".bin"
constexpr uint32_t IAP_IMAGE_START = 0x20458000;		// last 32kb of RAM

// Features definition
#define HAS_LWIP_NETWORKING		1
#define HAS_WIFI_NETWORKING		0
#define HAS_SBC_INTERFACE		1

#define HAS_MASS_STORAGE		1
#define HAS_HIGH_SPEED_SD		1
#define HAS_CPU_TEMP_SENSOR		1

#define SUPPORT_TMC51xx			0

#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			0
#define HAS_12V_MONITOR			1
#define ENFORCE_MIN_V12			1
#define HAS_VREF_MONITOR		1

#define SUPPORT_CAN_EXPANSION	1
#define DUAL_CAN				1					// support the second CAN interface as simple CAN (not FD)
#define SUPPORT_LED_STRIPS		1
#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_LASER			1					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_ACCELEROMETERS	1
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

#include <Duet3Common.h>							// this file is in the CANlib project because both main and expansion boards need it

constexpr size_t NumDirectDrivers = 6;				// The maximum number of drives supported by the electronics inc. direct expansion
constexpr size_t MaxCanDrivers = 20;
constexpr size_t MaxCanBoards = 20;

constexpr size_t MaxPortsPerHeater = 3;

constexpr size_t MaxBedHeaters = 12;
constexpr size_t MaxChamberHeaters = 4;
constexpr int8_t DefaultE0Heater = 1;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 4;
constexpr size_t NumTmcDriversSenseChannels = 1;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 15;						// The maximum number of movement axes in the machine
constexpr size_t MaxDriversPerAxis = 8;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 16;					// The maximum number of extruders
constexpr size_t MaxAxesPlusExtruders = 25;			// May be <= MaxAxes + MaxExtruders

constexpr size_t MaxHeatersPerTool = 8;
constexpr size_t MaxExtrudersPerTool = 8;

constexpr unsigned int MaxTriggers = 32;			// Must be <= 32 because we store a bitmap of pending triggers in a uint32_t

constexpr size_t NumSerialChannels = 3;				// The number of serial IO channels not counting the WiFi serial connection (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_AUX2_DEVICE Serial1

// Shared SPI (USART 1)
constexpr Pin APIN_USART_SSPI_SCK = PortBPin(13);
constexpr GpioPinFunction USARTSPIMosiPeriphMode = GpioPinFunction::C;
constexpr Pin APIN_USART_SSPI_MOSI = PortBPin(1);
constexpr GpioPinFunction USARTSPIMisoPeriphMode = GpioPinFunction::C;
constexpr Pin APIN_USART_SSPI_MISO = PortBPin(0);
constexpr GpioPinFunction USARTSPISckPeriphMode = GpioPinFunction::C;

constexpr Pin UsbVBusPin = PortCPin(21);			// Pin used to monitor VBUS on USB port

// Drivers
constexpr Pin STEP_PINS[NumDirectDrivers] =			{ PortCPin(18), PortCPin(16), PortCPin(28), PortCPin(01), PortCPin(4),  PortCPin(9)  };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =	{ PortBPin(5),  PortDPin(10), PortAPin(4),  PortAPin(22), PortCPin(3),  PortDPin(14) };
constexpr Pin ENABLE_PINS[NumDirectDrivers] =		{ PortBPin(4),  PortAPin(21), PortAPin(9),  PortAPin(23), PortAPin(2),  PortDPin(17) };
constexpr Pin DRIVER_ERR_PINS[NumDirectDrivers] =	{ PortDPin(29), PortCPin(17), PortDPin(13), PortCPin(02), PortDPin(31), PortCPin(10) };
constexpr Pin StepGatePin = PortDPin(22);
constexpr GpioPinFunction StepGatePinFunction = GpioPinFunction::C;			// TIOB11

#define STEP_GATE_TC		(TC3)					// TC11 is really TC3 channel 2
#define STEP_GATE_TC_ID		(ID_TC11)				// ID for peripheral clock and interrupt
#define STEP_GATE_TC_CHAN	(2)

// Thermistor/PT1000 inputs
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortCPin(15), PortCPin(29), PortCPin(0), PortCPin(31) };	// Thermistor/PT1000 pins
constexpr Pin VssaSensePin = PortCPin(13);
constexpr Pin VrefSensePin = PortCPin(30);

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / NumThermistorInputs) * 4700.0/((DefaultThermistorSeriesR / NumThermistorInputs) + 4700.0);
																			// there are 4 temperature sensing channels and a 4K7 load resistor
constexpr float VrefSeriesR = 15.0;

// Digital pins the SPI temperature sensors have their select lines tied to
constexpr Pin SpiTempSensorCsPins[] = { PortAPin(5), PortAPin(6), PortDPin(20), PortCPin(22) };

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortAPin(20);
constexpr Pin PowerMonitorV12DetectPin = PortEPin(4);
constexpr float PowerMonitorVoltageRange = (60.4 + 4.7)/4.7 * 3.3;			// voltage divider ratio times the reference voltage
constexpr float V12MonitorVoltageRange = (60.4 + 4.7)/4.7 * 3.3;			// voltage divider ratio times the reference voltage

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin DiagPin = PortBPin(6);										// diag/status LED
constexpr Pin ActLedPin = PortBPin(7);										// activityLED
constexpr bool DiagOnPolarity = false;

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortAPin(29), PortDPin(18) };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { PortDPin(19) };
constexpr uint32_t ExpectedSdCardSpeed = 25000000;
constexpr IRQn SdhcIRQn = HSMCI_IRQn;

// DotStar LED control
#define LEDSTRIP_USES_USART	0

constexpr Pin DotStarMosiPin = PortAPin(13);
constexpr Pin DotStarSclkPin = PortAPin(14);
constexpr GpioPinFunction DotStarPinMode = GpioPinFunction::A;
constexpr uint32_t DotStarClockId = ID_QSPI;
constexpr IRQn DotStarIRQn = QSPI_IRQn;

// Ethernet
constexpr Pin EthernetPhyInterruptPin = PortCPin(6);
constexpr Pin EthernetPhyResetPin = PortDPin(11);
constexpr Pin EthernetPhyOtherPins[] = {
		PortDPin(0), PortDPin(1), PortDPin(2), PortDPin(3), PortDPin(4),
		PortDPin(5), PortDPin(6), PortDPin(7), PortDPin(8), PortDPin(9)
};
constexpr auto EthernetPhyOtherPinsFunction = GpioPinFunction::A;

// Shared SPI definitions
#define USART_SPI		1
#define USART_SSPI		USART0
#define ID_SSPI			ID_USART0

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DueX.
//TODO change the table below for the V0.6 board
constexpr PinDescription PinTable[] =
{
	//	TC					PWM					ADC				Capability				PinNames
	// Port A
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"io6.out,!io6.out.iso"	},	// PA00 IO6_OUT
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"out5.tach"				},	// PA01 OUT5_TACH
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA02 driver 4 enable
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"io3.out"				},	// PA03 IO3_OUT
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA04 driver 2 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs1,serial3.rx"	},	// PA05 SPI_CS1 and Serial3 RX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs2,serial3.tx"	},	// PA06 SPI_CS2 and Serial3 TX
	{ TcOutput::none,	PwmOutput::pwm0h3_b,AdcInput::none,		PinCapability::wpwm,	"laser,vfd"				},	// PA07 VFD
	{ TcOutput::none,	PwmOutput::pwm1h3_a,AdcInput::none,		PinCapability::wpwm,	"out5"					},	// PA08 OUT5
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA09 driver 2 enable
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::write,	"pson"					},	// PA10 PS_ON
	{ TcOutput::none,	PwmOutput::pwm0h0_b,AdcInput::none,		PinCapability::wpwm,	"out2"					},	// PA11 OUT2
	{ TcOutput::none,	PwmOutput::pwm1h0_c,AdcInput::none,		PinCapability::wpwm,	"out8"					},	// PA12 OUT8
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA13 DotStarMosi
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA14 DotStarSclk
	{ TcOutput::tioa1,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"out3"					},	// PA15 OUT3
	{ TcOutput::none,	PwmOutput::pwm0l2_c,AdcInput::none,		PinCapability::wpwm,	"out1"					},	// PA16 OUT1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_6,	PinCapability::ainr,	"io7.in,!io7.in.iso"	},	// PA17 IO7_IN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_7,	PinCapability::ainr,	"io6.in,!io6.in.iso"	},	// PA18 IO6_IN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_8,	PinCapability::ainr,	"io5.in,!io5.in.iso"	},	// PA19 IO5_IN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_9,	PinCapability::none,	nullptr					},	// PA20 VIN detect
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA21 driver 1 enable
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA22 driver 3 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA23 driver 3 enable
	{ TcOutput::none,	PwmOutput::pwm0h1_b,AdcInput::none,		PinCapability::wpwm,	"out0"					},	// PA24 OUT0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA25 SDHC MCCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA26 SDHC MCDA2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA27 SDHC MCDA3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA28 SDHC MCCDA
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA29 SdCardDetectPin
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA30 SDHC MCDA0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PA31 SDHC MCDA1

	// Port B
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB00 USART_SSPI_MISO
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB01 USART_SSPI_MOSI
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB02 CAN0_TX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB03 CAN0_RX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB04 driver 0 enable
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB05 driver 0 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB06 SWDIO
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB07 SWCLK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB08	Crystal
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB09 Crystal
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB10 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB11 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB12 Erase
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB13 USART_SSPI_SCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB14 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB15 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB16 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB17 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB18 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB19 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB20 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB21 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB22 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB23 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB24 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB25 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB26 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB27 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB28 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB29 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB30 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PB31 not on chip

	// Port C
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_9,	PinCapability::ainr,	"temp2"					},	// PC00 thermistor 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC01 driver 3 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC02 driver 3 err
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC03 driver 4 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC04 driver 4 step
	{ TcOutput::tioa6,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"out4"					},	// PC05 OUT4
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC06 EthernetPhyInterrupt
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"out3.tach"				},	// PC07 OUT3_TACH
	{ TcOutput::tioa7,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"out7"					},	// PC08 OUT7
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC09 driver 5 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC10 driver 5 err
	{ TcOutput::tioa8,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"out6"					},	// PC11 OUT6
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC12 CAN1_RX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_1,	PinCapability::none,	nullptr					},	// PC13 VssaSensePin
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC14 NC
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_2,	PinCapability::ainr,	"temp0"					},	// PC15 thermistor 0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC16 driver 1 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC17 driver 1 err
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC18 driver 0 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC19 ETH_LED_Y
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC20 unused
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC21 UsbVBusPin
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs4"				},	// PC22 SPI CS4
	{ TcOutput::tioa3,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"io7.out,!io7.out.iso"	},	// PC23 IO7_OUT
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC24 SBC_SPI_SCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC25 SBC_SPI_SS0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC26 SBC_SPI_MISO
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC27 SBC_SPI_MOSI
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PC28 driver 2 step
	{ TcOutput::tioa5,	PwmOutput::none,	AdcInput::adc1_4,	PinCapability::ainrwpwm,"temp1"					},	// PC29 thermistor 1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_5,	PinCapability::none,	nullptr					},	// PC30 vrefmon
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_6,	PinCapability::ainr,	"temp3"					},	// PC31 thermistor 3

	// PORT D
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD00 GTXCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD01 GTXEN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD02 GTX0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD03 GTX1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD04 GRXDV
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD05 GRX0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD06 GRX1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD07 GRXER
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD08 GMDC
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD09 GMDIO
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD10 driver 1 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD11 EthernetPhyReset
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD12 CAN1_TX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD13 driver 2 err
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD14 driver 5 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"io1.in,serial1.rx"		},	// PD15 IO1_IN and Serial1 RX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"io1.out,serial1.tx"	},	// PD16 IO1_OUT and Serial1 TX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD17 driver 5 enable
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD18 External SD card detect
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD19 SPI CS0 (external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs3"				},	// PD20 SPI CS3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"io5.out,!io5.out.iso"	},	// PD21 IO5_OUT (not PWM capable on TIOA11 because TIOB11 is used to generate step pulses)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD22 step gate
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"out4.tach"				},	// PD23 OUT4_TACH
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD24 SWD_EXT_RST
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"io0.in,serial0.rx"		},	// PD25 IO0_IN  Serial0 RX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"io0.out,serial0.tx"	},	// PD26 IO0_OUT Serial0 TX
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"io2.out,i2c0.dat"		},	// PD27 IO2_OUT
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"io2.in,i2c0.clk"		},	// PD28 IO2_IN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD29 driver 0 err
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_0,	PinCapability::ainr,	"io4.in"				},	// PD30 IO4_IN
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PD31 driver 4 err

	// Port E
	{ TcOutput::tioa9,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"io4.out"				},	// PE00 IO4_OUT
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"io8.out,!io8.out.iso"	},	// PE01 IO8_OUT
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr					},	// PE02 SbcTfrReady
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc1_10,	PinCapability::read,	"io8.in,!io8.in.iso"	},	// PE03 IO8_IN analog in not usable because it is on the wrong ADC
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_4,	PinCapability::none,	nullptr					},	// PE04 V12 detect
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_3,	PinCapability::ainr,	"io3.in"				},	// PE05 IO3_IN
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);
static_assert(NumNamedPins == 32+32+32+32+6);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;

// Serial Interfaces
constexpr Pin APIN_Serial0_RXD = PortDPin(25);
constexpr Pin APIN_Serial0_TXD = PortDPin(26);
constexpr auto Serial0PinFunction = GpioPinFunction::C;
constexpr Pin APIN_Serial1_RXD = PortDPin(15);
constexpr Pin APIN_Serial1_TXD = PortDPin(16);
constexpr auto Serial1PinFunction = GpioPinFunction::B;

// SD Card
constexpr Pin HsmciMclkPin = PortAPin(25);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::D;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherPinsFunction = GpioPinFunction::C;

// Duet pin numbers for the SBC interface
#define SBC_SPI					SPI1
#define SBC_SPI_INTERFACE_ID	ID_SPI1
#define SBC_SPI_IRQn			SPI1_IRQn
#define SBC_SPI_HANDLER			SPI1_Handler

constexpr Pin APIN_SBC_SPI_MOSI = PortCPin(27);
constexpr Pin APIN_SBC_SPI_MISO = PortCPin(26);
constexpr Pin APIN_SBC_SPI_SCK = PortCPin(24);
constexpr Pin APIN_SBC_SPI_SS0 = PortCPin(25);
constexpr GpioPinFunction SBCPinPeriphMode = GpioPinFunction::C;

// CAN
constexpr Pin APIN_CAN0_RX = PortBPin(3);
constexpr Pin APIN_CAN0_TX = PortBPin(2);
constexpr GpioPinFunction CAN0PinPeriphMode = GpioPinFunction::A;

constexpr Pin APIN_CAN1_RX = PortCPin(12);
constexpr GpioPinFunction CAN1RXPinPeriphMode = GpioPinFunction::C;
constexpr Pin APIN_CAN1_TX = PortDPin(12);
constexpr GpioPinFunction CAN1TXPinPeriphMode = GpioPinFunction::B;

constexpr unsigned int CanDeviceNumber = 1;				// CAN-FD device number

#if defined(DUAL_CAN) && DUAL_CAN
constexpr unsigned int SecondaryCanDeviceNumber = 0;	// plan CAN device number
#endif

constexpr Pin SbcTfrReadyPin = PortEPin(2);
// Note, the DMAC peripheral IDs are hard-coded in DataTransfer

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
constexpr DmaChannel DmacChanHsmci = 0;			// this is hard coded in the ASF HSMCI driver
//constexpr DmaChannel DmacChanWiFiTx = 1;		// only on v0.3 board
//constexpr DmaChannel DmacChanWiFiRx = 2;		// only on v0.3 board
constexpr DmaChannel DmacChanTmcTx = 3;
constexpr DmaChannel DmacChanTmcRx = 4;
constexpr DmaChannel DmacChanSbcTx = 5;
constexpr DmaChannel DmacChanSbcRx = 6;
constexpr DmaChannel DmacChanDotStarTx = 7;

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
				? 1u << (STEP_PINS[driver] & 0x1Fu)
				: 0;
	}

	// Set the specified step pins high. This needs to be fast.
	static inline __attribute__((always_inline)) void StepDriversHigh(uint32_t driverMap) noexcept
	{
		PIOC->PIO_SODR = driverMap;				// on Duet 3 all step pins are on port C
	}

	// Set the specified step pins low. This needs to be fast.
	static inline __attribute__((always_inline)) void StepDriversLow(uint32_t driverMap) noexcept
	{
		PIOC->PIO_CODR = driverMap;				// on Duet 3 all step pins are on port C
	}

	// Get the bitmap for all our drivers
	constexpr uint32_t AllDriversBitmap =
		[]()->uint32_t
		{
			uint32_t bits = 0;
			for (Pin p : STEP_PINS)
			{
				bits |= 1u << (p & 0x1Fu);
			}
			return bits;
		}
		();
}

#endif	//ifndef PINS_DUET3_MB6XD_H__
