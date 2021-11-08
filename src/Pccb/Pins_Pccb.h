/*
 * Pins_DuetM.h
 *
 *  Created on: 29 Nov 2017
 *      Author: David
 */

#ifndef SRC_PCCB_PINS_PCCB_H_
#define SRC_PCCB_PINS_PCCB_H_

#include <PinDescription.h>

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

// File system options
#define HAS_MASS_STORAGE		0					// SD card socket is optional
#define HAS_HIGH_SPEED_SD		0
#define HAS_EMBEDDED_FILES		1					// A read only file system is appended to the binary

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
#define SUPPORT_LED_STRIPS		1					// set nonzero to support DotStar LED strips
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
constexpr size_t MaxPortsPerHeater = 1;
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
constexpr size_t MaxAxesPlusExtruders = NumDirectDrivers;

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 1;

constexpr size_t MaxFans = 7;

constexpr unsigned int MaxTriggers = 16;			// Maximum number of triggers

constexpr size_t MaxSpindles = 2;					// Maximum number of configurable spindles

constexpr size_t NumSerialChannels = 1;				// The number of serial IO channels (USB only)
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
constexpr GpioPinFunction TMC2660PeriphMode = GpioPinFunction::A;
constexpr Pin GlobalTmc2660EnablePin = PortCPin(16); // The pin that drives ENN of all drivers on the DueX5

constexpr uint32_t DefaultStandstillCurrentPercent = 100;					// it's not adjustable on TMC2660

#elif defined(PCCB_08)

constexpr Pin ENABLE_PINS[NumDirectDrivers] =		{ NoPin,		NoPin,		  PortBPin(14), PortCPin(25), PortCPin( 5), PortCPin(19), PortAPin( 0), PortCPin(28) };
constexpr Pin STEP_PINS[NumDirectDrivers] =			{ PortCPin( 4), PortCPin( 7), PortCPin(24), PortCPin( 2), PortCPin(22), PortCPin(20), PortCPin(10), PortCPin(14) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] =	{ PortAPin( 8), PortAPin(11), PortAPin(17), PortCPin(21), PortCPin(18), PortCPin(13), PortAPin( 1), PortCPin(17) };

constexpr Pin APIN_UART0_RXD = PortAPin(9);
constexpr Pin APIN_UART0_TXD = PortAPin(10);
constexpr GpioPinFunction UART0PeriphMode = GpioPinFunction::A;
constexpr Pin APIN_UART1_RXD = PortBPin(2);
constexpr Pin APIN_UART1_TXD = PortBPin(3);
constexpr GpioPinFunction UART1PeriphMode = GpioPinFunction::A;

Uart * const TMC22xxUarts[MaxSmartDrivers] = { UART0, UART1 };
constexpr uint32_t TMC22xxUartIds[MaxSmartDrivers] = { ID_UART0, ID_UART1 };
constexpr IRQn TMC22xxUartIRQns[MaxSmartDrivers] = { UART0_IRQn, UART1_IRQn };
constexpr Pin TMC22xxUartPins[MaxSmartDrivers] = { APIN_UART0_RXD, APIN_UART0_TXD, APIN_UART1_RXD, APIN_UART1_TXD };

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

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / NumThermistorInputs) * 4700.0/((DefaultThermistorSeriesR / NumThermistorInputs) + 4700.0);
																			// there are 2 temperature sensing channels and a 4K7 load resistor
constexpr float VrefSeriesR = 15.0;

// Number of SPI temperature sensors to support
constexpr size_t MaxSpiTempSensors = 1;		//TODO which SPI channels does PCCB route to the DueX?

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { PortCPin(27) };	// SPI0_CS6 if a DueX5 is connected

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortCPin(12);						// Vin monitor
constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr size_t MaxZProbes = 1;

constexpr Pin DiagPin = PortAPin(7);
constexpr bool DiagOnPolarity = true;

// DotStar LED control (USART0 is SharedSPI so we use USART1)
#define LEDSTRIP_USES_USART	1

Usart * const DotStarUsart = USART1;
constexpr Pin DotStarMosiPin = PortAPin(22);
constexpr Pin DotStarSclkPin = PortAPin(23);
constexpr GpioPinFunction DotStarPinMode = GpioPinFunction::A;
constexpr uint32_t DotStarClockId = ID_USART1;
constexpr IRQn DotStarIRQn = USART1_IRQn;

// SD cards
constexpr size_t NumSdCards = 1;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortCPin(8) };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin };
constexpr Pin SdSpiCSPins[1] = { NoPin };
constexpr IRQn SdhcIRQn = HSMCI_IRQn;
constexpr uint32_t ExpectedSdCardSpeed = 15000000;

// Shared SPI definitions
#define USART_SPI		1
#define USART_SSPI		USART0
#define ID_SSPI			ID_USART0

constexpr PinDescription PinTable[] =
{	//	TC					PWM					ADC				Capability				PinNames
	// Port A
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::write,	nullptr				},	// PA00 PS_ON
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA01 ENN to all stepper drivers
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA02 SCK0 (daughter boards, external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		nullptr				},	// PA03 TWD0 (expansion)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		nullptr				},	// PA04 TWCK0 (expansion)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA05 RXD0 (daughter boards, external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA06 TXD0 (daughter boards, external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA07 DIAG led
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA08 Y dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA09 Stepper drivers UART
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA10 Stepper drivers UART
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA11 NPCS0 (W5500)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA12 MISO (W5500)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA13 MOSI (W5500)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA14 SPCK (W5500)
	{ TcOutput::tioa1,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"fan1"				},	// PA15 Fan 1
	{ TcOutput::none,	PwmOutput::pwm0l2_c,AdcInput::none,		PinCapability::wpwm,	"fan0"				},	// PA16 Heater 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_0,	PinCapability::none,	nullptr				},	// PA17 VREF_MON
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_1,	PinCapability::ainrw,	"exp.pa18,exp.35"	},	// PA18 E2 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_2,	PinCapability::none,	nullptr				},	// PA19 VSSA_MON
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_3,	PinCapability::ainr,	"temp0"				},	// PA20 Thermistor 0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_8,	PinCapability::ainrw,	"exp.pa21,exp.36"	},	// PA21 Analogue, digital or UART expansion
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_9,	PinCapability::ainrw,	nullptr				},	// PA22 Analogue, digital or UART expansion
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA23 USART 1 SCLK (DotStar LED)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"stop0"				},	// PA24 Stop 0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"stop1"				},	// PA25 Stop 1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA26 HSMCI MCDA2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA27 HSMCI MCDA3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA28 HSMCI MCCDA
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA29 HSMCI MCCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA30 HSMCI MCDA0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA31 HSMCI MCDA1

	// Port B
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_4,	PinCapability::read,	"fan5a.tach"		},	// PB00 Fan 5a tach
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_5,	PinCapability::ainr,	nullptr				},	// PB01 Thermistor 3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::write,	"fan3"				},	// PB02 Fan 3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::write,	"fan4"				},	// PB03 Fan 4
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB04 Z dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB05 LCD ENC_A
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	nullptr				},	// PB06 Y stop
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB07 E0 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB08
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB09
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB10
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB11
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB12
	{ TcOutput::none,	PwmOutput::none,	AdcInput::dac0,		PinCapability::none,	nullptr				},	// PB13 SPI0_CS0 (external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		nullptr				},	// PB14 expansion (driver 2 EN/CS)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB15 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB16 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB17 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB18 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB19 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB20 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB21 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB22 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB23 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB24 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB25 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB26 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB27 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB28 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB29 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB30 not on chip
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB31 not on chip

	// Port C
	{ TcOutput::none,	PwmOutput::pwm0l0_b,AdcInput::none,		PinCapability::wpwm,	"led"				},	// PC00 LED
	{ TcOutput::none,	PwmOutput::pwm0l1_b,AdcInput::none,		PinCapability::wpwm,	"fan5"				},	// PC01 Fan 5
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC02 Y step
	{ TcOutput::none,	PwmOutput::pwm0l3_b,AdcInput::none,		PinCapability::wpwm,	"fan2"				},	// PC03 Fan 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC04 E0 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC05 E1 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"stop2"				},	// PC06 Stop 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	nullptr				},	// PC07 E1 stop
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC08 SD card detect
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC09 LCD CS
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	nullptr				},	// PC10 Z stop
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC11 USB Vbus monitor
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_12,	PinCapability::none,	nullptr				},	// PC12 VIN voltage monitor
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_10,	PinCapability::ainr,	"temp1"				},	// PC13 Thermistor 1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC14 MUX0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_11,	PinCapability::ainr,	nullptr				},	// PC15 Z probe input
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC16 MUX1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC17 MUX2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC18 X dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		nullptr				},	// PC19 SPI0_CS2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC20 X step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC21 E3 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"fan1"				},	// PC22 Step 4
	{ TcOutput::tioa3,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"!leddim"			},	// PC23 LED dim
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC24 E3 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC25 E3 en
	{ TcOutput::tioa4,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"exp.pc26,exp.13,duex.heater4"	},	// PC26 Z probe mod/servo/diag LED
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"exp.pc27,exp.9,spi.cs6,stop3"	},	// PC27 E2 en
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC28 Z step
	{ TcOutput::tioa5,	PwmOutput::none,	AdcInput::none,		PinCapability::rwpwm,	"exp.pc29,exp.8,duex.heater3"	},	// PC29 Fan 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_14,	PinCapability::read,	"fan5b.tach"		},	// PC30 Fan 5b tach
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC31 E2 step
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);
static_assert(NumNamedPins == 3*32);

// Function to look up a pin name pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;
// Wire Interfaces
#define WIRE_INTERFACES_COUNT (1)		// SAM4S supports two I2C interfaces but we only have the first one available

#define WIRE_INTERFACE		TWI0
#define WIRE_INTERFACE_ID	ID_TWI0
#define WIRE_ISR_HANDLER	TWI0_Handler
#define WIRE_ISR_ID			TWI0_IRQn

constexpr Pin TWI_Data = PortAPin(3);
constexpr Pin TWI_CK = PortAPin(4);
constexpr GpioPinFunction TWIPeriphMode = GpioPinFunction::A;

// SD Card
constexpr Pin HsmciClockPin = PortAPin(29);
constexpr Pin HsmciOtherPins[] = { PortAPin(28), PortAPin(30), PortAPin(31), PortAPin(26), PortAPin(27) };
constexpr auto HsmciPinsFunction = GpioPinFunction::C;

// Main SPI interface
constexpr Pin APIN_SPI_MOSI = PortAPin(13);
constexpr Pin APIN_SPI_MISO = PortAPin(12);
constexpr Pin APIN_SPI_SCK = PortAPin(14);
constexpr Pin APIN_SPI_SS0 = PortAPin(11);
constexpr GpioPinFunction SPIPeriphMode = GpioPinFunction::A;

// USARTs used for SPI
constexpr Pin APIN_USART_SSPI_MOSI = PortAPin(6);
constexpr GpioPinFunction USARTSPIMosiPeriphMode = GpioPinFunction::A;
constexpr Pin APIN_USART_SSPI_MISO = PortAPin(5);
constexpr GpioPinFunction USARTSPIMisoPeriphMode = GpioPinFunction::A;
constexpr Pin APIN_USART_SSPI_SCK = PortAPin(2);
constexpr GpioPinFunction USARTSPISckPeriphMode = GpioPinFunction::B;

#if ALLOCATE_DEFAULT_PORTS

// Default pin allocations
constexpr const char *DefaultEndstopPinNames[] = { "stop1", "nil", "stop0" };	// stop0 is the default Z endstop, stop1 is the X endstop
constexpr const char *DefaultZProbePinNames = "nil";
constexpr const char *DefaultHeaterPinNames[] = { "nil" };
constexpr const char *DefaultGpioPinNames[] = { "led", "leddim" };

#if defined(PCCB_10)
constexpr const char *DefaultFanPinNames[] = { "fan0", "fan1", "fan2", "fan3", "fan4", "!fan5+fan5a.tach", "nil+fan5b.tach" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { 0, 0, 0, 0, 0, 25000 };
#else
constexpr const char *DefaultFanPinNames[] = { "fan0", "fan1", "fan2", "!fan3+fan3a.tach", "nil+fan3btach" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { 0, 0, 0, 25000 };
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
				? 1u << (STEP_PINS[driver] & 0x1Fu)
				: 0;
	}

	// Set the specified step pins high. This needs to be fast.
	static inline __attribute__((always_inline)) void StepDriversHigh(uint32_t driverMap) noexcept
	{
		PIOC->PIO_SODR = driverMap;				// on PCCB all step pins are on port C
	}

	// Set all step pins low. This needs to be fast.
	static inline __attribute__((always_inline)) void StepDriversLow(uint32_t driverMap) noexcept
	{
		PIOC->PIO_CODR = driverMap;				// on PCCB all step pins are on port C
	}
}

#endif /* SRC_PCCB_PINS_PCCB_H_ */
