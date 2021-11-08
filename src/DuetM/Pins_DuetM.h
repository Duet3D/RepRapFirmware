/*
 * Pins_DuetM.h
 *
 *  Created on: 29 Nov 2017
 *      Author: David
 */

#ifndef SRC_DUETM_PINS_DUETM_H_
#define SRC_DUETM_PINS_DUETM_H_

#include <PinDescription.h>

#define BOARD_NAME				"Duet 2 Maestro"
#define BOARD_SHORT_NAME		"2Maestro"
#define FIRMWARE_NAME			"RepRapFirmware for Duet 2 Maestro"
#define DEFAULT_BOARD_TYPE		 BoardType::DuetM_10

constexpr size_t NumFirmwareUpdateModules = 5;		// 0 = mainboard, 4 = PanelDue, values in between unused
#define IAP_FIRMWARE_FILE		"DuetMaestroFirmware.bin"
#define IAP_UPDATE_FILE			"Duet2_SDiap32_Maestro.bin"
constexpr uint32_t IAP_IMAGE_START = 0x20018000;

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		0
#define HAS_W5500_NETWORKING	1

#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		1
#define SUPPORT_TMC22xx			1
#define TMC22xx_HAS_MUX			1
#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			0
#define HAS_VREF_MONITOR		1

#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			0					// set zero to disable support for FreeLSS scanners
#define SUPPORT_LASER			1					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			0					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors (requires RTOS)
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_12864_LCD		1					// set nonzero to support 12864 LCD and rotary encoder
#define SUPPORT_ACCELEROMETERS	1
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_FTP				1
#define SUPPORT_TELNET			1
#define SUPPORT_ASYNC_MOVES		1
#define ALLOCATE_DEFAULT_PORTS	0
#define TRACK_OBJECT_NAMES		1

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 7;				// The maximum number of drives supported by the electronics
constexpr size_t MaxSmartDrivers = 7;				// The maximum number of smart drivers

constexpr size_t MaxSensors = 32;

constexpr size_t MaxHeaters = 4;					// The maximum number of heaters in the machine
constexpr size_t MaxPortsPerHeater = 2;
constexpr size_t MaxMonitorsPerHeater = 3;			// The maximum number of monitors per heater

constexpr size_t MaxBedHeaters = 2;
constexpr size_t MaxChamberHeaters = 2;
constexpr int8_t DefaultBedHeater = 0;
constexpr int8_t DefaultE0Heater = 1;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 4;
constexpr size_t NumTmcDriversSenseChannels = 2;

constexpr size_t MaxZProbes = 2;
constexpr size_t MaxGpInPorts = 10;
constexpr size_t MaxGpOutPorts = 10;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 6;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 4;					// The maximum number of extruders
constexpr size_t MaxAxesPlusExtruders = 7;

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 4;

constexpr size_t MaxFans = 6;

constexpr unsigned int MaxTriggers = 16;			// Maximum number of triggers

constexpr size_t MaxSpindles = 2;					// Maximum number of configurable spindles

constexpr size_t NumSerialChannels = 2;				// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial

// SerialUSB
constexpr Pin UsbVBusPin = PortCPin(11);			// Pin used to monitor VBUS on USB port

#define I2C_IFACE	Wire							// First and only I2C interface
#define I2C_IRQn	WIRE_ISR_ID

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drivers
constexpr Pin GlobalTmc22xxEnablePin = PortAPin(1);	// The pin that drives ENN of all drivers
constexpr Pin ENABLE_PINS[NumDirectDrivers] = { NoPin, NoPin, NoPin, NoPin, NoPin, PortCPin(27), PortCPin(25) };
constexpr Pin STEP_PINS[NumDirectDrivers] = { PortCPin(20), PortCPin(2), PortCPin(28), PortCPin(4), PortCPin(5), PortCPin(31), PortCPin(21) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { PortCPin(18), PortAPin(8), PortBPin(4), PortBPin(7), PortCPin(6), PortAPin(18), PortCPin(24) };

// Serial
// UART0 used by TMC drivers
constexpr Pin APIN_UART0_RXD = PortAPin(9);
constexpr Pin APIN_UART0_TXD = PortAPin(10);
constexpr GpioPinFunction UART0PeriphMode = GpioPinFunction::A;

// Serial0 uses UART1
constexpr Pin APIN_Serial0_RXD = PortBPin(2);
constexpr Pin APIN_Serial0_TXD = PortBPin(3);
constexpr GpioPinFunction Serial0PeriphMode = GpioPinFunction::A;

// UART interface to stepper drivers
Uart * const UART_TMC22xx = UART0;
constexpr IRQn TMC22xx_UART_IRQn = UART0_IRQn;
constexpr uint32_t ID_TMC22xx_UART = ID_UART0;
constexpr Pin TMC22xxUartRxPin = APIN_UART0_RXD;
constexpr Pin TMC22xxUartTxPin = APIN_UART0_TXD;
constexpr GpioPinFunction TMC22xxUartPeriphMode = UART0PeriphMode;

#define TMC22xx_UART_Handler			UART0_Handler

#define TMC22xx_USES_SERCOM				0
#define TMC22xx_HAS_ENABLE_PINS			1
#define TMC22xx_VARIABLE_NUM_DRIVERS	1
#define TMC22xx_SINGLE_DRIVER			0
#define TMC22xx_HAS_MUX					1
#define TMC22xx_USE_SLAVEADDR			0
#define TMC22xx_DEFAULT_STEALTHCHOP		1

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// In testing I found that 500kbaud was not reliable, so now using 250kbaud.
constexpr uint32_t DriversBaudRate = 250000;
constexpr uint32_t TransferTimeout = 2;										// any transfer should complete within 2 ticks @ 1ms/tick
constexpr uint32_t DefaultStandstillCurrentPercent = 75;

constexpr Pin TMC22xxMuxPins[3] = { PortCPin(14), PortCPin(16), PortCPin(17) };	// Pins that control the UART multiplexer, LSB first

constexpr float DriverSenseResistor = 0.083 + 0.03;							// in ohms
constexpr float DriverVRef = 180.0;											// in mV
constexpr float DriverFullScaleCurrent = DriverVRef/DriverSenseResistor;	// in mA
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;
constexpr float MaximumMotorCurrent = 1600.0;								// we can't go any higher without switching to the low sensitivity range
constexpr float MaximumStandstillCurrent = 1400.0;

// Thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortAPin(20), PortBPin(0), PortCPin(30), PortBPin(1) }; 	// Thermistor pin numbers
constexpr Pin VssaSensePin = PortAPin(19);
constexpr Pin VrefSensePin = PortAPin(17);

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 2200.0;
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / NumThermistorInputs) * 4700.0/((DefaultThermistorSeriesR / NumThermistorInputs) + 4700.0);
																			// there are 4 temperature sensing channels and a 4K7 load resistor
constexpr float VrefSeriesR = 15.0;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[] = { PortBPin(14), PortCPin(19) };		// SPI0_CS1, SPI0_CS2

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortCPin(12);						// Vin monitor
constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

// Digital pin number to turn the IR LED on (high) or off (low), also controls the DIAG LED
constexpr Pin Z_PROBE_PIN = PortCPin(15);									// Z probe analog input
constexpr Pin Z_PROBE_MOD_PIN = PortCPin(26);
constexpr Pin DiagPin = Z_PROBE_MOD_PIN;
constexpr bool DiagOnPolarity = true;

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { PortCPin(8), NoPin };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[1] = { PortBPin(13) };
constexpr IRQn SdhcIRQn = HSMCI_IRQn;
constexpr uint32_t ExpectedSdCardSpeed = 15000000;

// 12864 LCD
// The ST7920 datasheet specifies minimum clock cycle time 400ns @ Vdd=4.5V, min. clock width 200ns high and 20ns low.
// This assumes that the Vih specification is met, which is 0.7 * Vcc = 3.5V @ Vcc=5V
// The Duet Maestro level shifts all 3 LCD signals to 5V, so we meet the Vih specification and can reliably run at 2MHz.
// For other electronics, there are reports that operation with 3.3V LCD signals may work if you reduce the clock frequency.
// The ST7567 specifies minimum clock cycle time 50ns i.e. 20MHz @ Vcc=3.3V
constexpr uint32_t LcdSpiClockFrequency = 2000000;		// 2.0MHz
constexpr Pin LcdCSPin = PortCPin(9);
constexpr Pin LcdA0Pin = PortAPin(21);    // EXP_0
constexpr Pin LcdCSAltPin = PortAPin(22); // EXP_1
constexpr Pin LcdBeepPin = PortAPin(15);
constexpr Pin EncoderPinA = PortBPin(5);
constexpr Pin EncoderPinB = PortCPin(3);
constexpr Pin EncoderPinSw = PortAPin(7);

// Shared SPI definitions
#define USART_SPI		1
#define USART_SSPI		USART0
#define ID_SSPI			ID_USART0

constexpr PinDescription PinTable[] =
{	//	TC					PWM					ADC				Capability				PinNames
	// Port A
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::write,	"pson"				},	// PA00 PS_ON
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA01 ENN to all stepper drivers
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA02 SCK0 (daughter boards, external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"exp.pa3,twd0"		},	// PA03 TWD0 (expansion)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"exp.pa4,twck0"		},	// PA04 TWCK0 (expansion)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA05 RXD0 (daughter boards, external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA06 TXD0 (daughter boards, external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA07 LCD ENC_SW
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA08 Y dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA09 Stepper drivers UART
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA10 Stepper drivers UART
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA11 NPCS0 (W5500)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA12 MISO (W5500)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA13 MOSI (W5500)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA14 SPCK (W5500)
	{ TcOutput::tioa1,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA15 LCD beep
	{ TcOutput::none,	PwmOutput::pwm0l2_c,AdcInput::none,		PinCapability::wpwm,	"!e1heat"			},	// PA16 Heater 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_0,	PinCapability::none,	nullptr				},	// PA17 VREF_MON
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_1,	PinCapability::none,	nullptr				},	// PA18 E2 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_2,	PinCapability::none,	nullptr				},	// PA19 VSSA_MON
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_3,	PinCapability::ainr,	"bedtemp"			},	// PA20 Thermistor 0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_8,	PinCapability::ainrw,	"exp.pa21"			},	// PA21 Analogue, digital or UART expansion
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_9,	PinCapability::ainrw,	"exp.pa22"			},	// PA22 Analogue, digital or UART expansion
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA23 W5500 interrupt
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"xstop"				},	// PA24 X stop
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"e0stop"			},	// PA25 E0 stop
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA26 HSMCI MCDA2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA27 HSMCI MCDA3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA28 HSMCI MCCDA
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA29 HSMCI MCCK
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA30 HSMCI MCDA0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PA31 HSMCI MCDA1

	// Port B
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_4,	PinCapability::ainr,	"e0temp"			},	// PB00 Thermistor 1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_5,	PinCapability::ainr,	"ctemp"				},	// PB01 Thermistor 3
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"urxd"				},	// PB02 URXD0 PanelDue Dout
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"utxd"				},	// PB03 UTXD0 PanelDue Din
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB04 Z dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB05 LCD ENC_A
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"ystop"				},	// PB06 Y stop
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB07 E0 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB08
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB09
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB10
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB11
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PB12
	{ TcOutput::none,	PwmOutput::none,	AdcInput::dac0,		PinCapability::none,	nullptr				},	// PB13 SPI0_CS0 (external SD card)
	{ TcOutput::none,	PwmOutput::none,	AdcInput::dac1,		PinCapability::rw,		"spi.cs1"			},	// PB14 SPI0_CS1 (daughter boards)
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
	{ TcOutput::none,	PwmOutput::pwm0l0_b,AdcInput::none,		PinCapability::wpwm,	"!bedheat"			},	// PC00 Heater 0
	{ TcOutput::none,	PwmOutput::pwm0l1_b,AdcInput::none,		PinCapability::wpwm,	"!e0heat"			},	// PC01 Heater 1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC02 Y step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC03 ENC_B
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC04 E0 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC05 E1 step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC06 E1 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"e1stop"			},	// PC07 E1 stop
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC08 SD card detect
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC09 LCD CS
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::read,	"zstop"				},	// PC10 Z stop
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC11 USB Vbus monitor
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_12,	PinCapability::none,	nullptr				},	// PC12 VIN voltage monitor
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC13 W5500 reset
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC14 MUX0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_11,	PinCapability::ainr,	"zprobe.in"			},	// PC15 Z probe input
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC16 MUX1
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC17 MUX2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC18 X dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::rw,		"spi.cs2"			},	// PC19 SPI0_CS2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC20 X step
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC21 E3 step
	{ TcOutput::none,	PwmOutput::pwm0l3_b,AdcInput::none,		PinCapability::wpwm,	"fan1"				},	// PC22 Fan 1
	{ TcOutput::tioa3,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"fan0"				},	// PC23 Fan 0
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC24 E3 dir
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC25 E3 en
	{ TcOutput::tioa4,	PwmOutput::none,	AdcInput::none,		PinCapability::write,	"zprobe.mod,servo"	},	// PC26 Z probe mod/servo/diag LED
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC27 E2 en
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC28 Z step
	{ TcOutput::tioa5,	PwmOutput::none,	AdcInput::none,		PinCapability::wpwm,	"fan2"				},	// PC29 Fan 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::adc0_14,	PinCapability::ainr,	"e1temp"			},	// PC30 Thermistor 2
	{ TcOutput::none,	PwmOutput::none,	AdcInput::none,		PinCapability::none,	nullptr				},	// PC31 E2 step
};

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);
static_assert(NumNamedPins == 3*32);

// Function to look up a pin name and pass back the corresponding index into the pin table
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

// Duet pin numbers to control the W5500 interface
#define W5500_SPI				SPI
#define W5500_SPI_INTERFACE_ID	ID_SPI
#define W5500_SPI_IRQn			SPI_IRQn
#define W5500_SPI_HANDLER		SPI_Handler

constexpr Pin APIN_W5500_SPI_MOSI = APIN_SPI_MOSI;
constexpr Pin APIN_W5500_SPI_MISO = APIN_SPI_MISO;
constexpr Pin APIN_W5500_SPI_SCK = APIN_SPI_SCK;
constexpr Pin APIN_W5500_SPI_SS0 = APIN_SPI_SS0;

constexpr Pin W5500ResetPin = PortCPin(13);									// Low on this in holds the W5500 in reset
constexpr Pin W5500SsPin = PortAPin(11);									// SPI NPCS pin to W5500
constexpr Pin W5500IntPin = PortAPin(23);									// Interrupt from W5500

// Timer allocation
// TC0 channel 0 is used for step pulse generation and software timers (lower 16 bits)
// TC0 channel 1 is used for LCD beep
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
		PIOC->PIO_SODR = driverMap;				// on Duet Maestro all step pins are on port C
	}

	// Set the specified step pins low. This needs to be fast.
	static inline void __attribute__((always_inline)) StepDriversLow(uint32_t driverMap) noexcept
	{
		PIOC->PIO_CODR = driverMap;				// on Duet Maestro all step pins are on port C
	}
}

#endif /* SRC_DUETM_PINS_DUETM_H_ */
