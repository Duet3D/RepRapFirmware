/*
 * Pins_Duet5LC.h
 *
 *  Created on: 28 May 2020
 *      Author: David
 */

#ifndef SRC_DUET3MINI_PINS_DUET3MINI_H_
#define SRC_DUET3MINI_PINS_DUET3MINI_H_

#include <PinDescription.h>

#define BOARD_SHORT_NAME		"Mini5plus"
#define DEFAULT_BOARD_TYPE		 BoardType::Duet3Mini_Unknown

#ifdef DUET3MINI_V02
#define BOARD_NAME				"Duet 3 Mini 5+ prototype v0.2"
#define FIRMWARE_NAME			"RepRapFirmware for Duet 3 Mini 5+ prototype v0.2"
#define IAP_FIRMWARE_FILE		"Duet3Firmware_" BOARD_SHORT_NAME "_v02.uf2"
#endif

#ifdef DUET3MINI_V04
#define BOARD_NAME				"Duet 3 Mini 5+"
#define FIRMWARE_NAME			"RepRapFirmware for Duet 3 Mini 5+"
#define IAP_FIRMWARE_FILE		"Duet3Firmware_" BOARD_SHORT_NAME "_v04.uf2"
#endif

constexpr size_t NumFirmwareUpdateModules = 2;		// main module and WiFi module

#define IAP_UPDATE_FILE			"Duet3_SDiap_" BOARD_SHORT_NAME ".bin"
#define IAP_UPDATE_FILE_SBC		"Duet3_SBCiap_" BOARD_SHORT_NAME ".bin"
constexpr uint32_t IAP_IMAGE_START = 0x20038000;

#define WIFI_FIRMWARE_FILE		"DuetWiFiServer.bin"

// Features definition
#define HAS_LWIP_NETWORKING		1
#define HAS_WIFI_NETWORKING		1
#define HAS_W5500_NETWORKING	0
#define HAS_LINUX_INTERFACE		1

#define HAS_MASS_STORAGE		1
#define HAS_HIGH_SPEED_SD		1
#define HAS_CPU_TEMP_SENSOR		0					// the temperature sensors don't work in revision A or D chips (revision D is latest as at 2020-06-28)

#define SUPPORT_TMC22xx			1
#define TMC22xx_HAS_MUX			1
#define HAS_STALL_DETECT		1

#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			0
#define HAS_VREF_MONITOR		1

#ifdef DUET3MINI_V04
#define SUPPORT_CAN_EXPANSION	1
#else
#define SUPPORT_CAN_EXPANSION	0
#endif

#define SUPPORT_DOTSTAR_LED		1
#define SUPPORT_INKJET			0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND			0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER			1					// set zero to disable support for FreeLSS scanners
#define SUPPORT_LASER			1					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			1					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		1					// set nonzero to support DHT temperature/humidity sensors (requires RTOS)
#define SUPPORT_WORKPLACE_COORDINATES	1			// set nonzero to support G10 L2 and G53..59
#define SUPPORT_12864_LCD		1					// set nonzero to support 12864 LCD and rotary encoder
#define SUPPORT_OBJECT_MODEL	1
#define SUPPORT_FTP				1
#define SUPPORT_TELNET			1
#define SUPPORT_ASYNC_MOVES		1
#define ALLOCATE_DEFAULT_PORTS	0
#define TRACK_OBJECT_NAMES		1

#define USE_CACHE				1					// set nonzero to enable the cache
#define USE_MPU					0					// set nonzero to enable the memory protection unit

// The physical capabilities of the machine

# include <Duet3Common.h>

#if defined(DUET3MINI_V04)
constexpr size_t NumDirectDrivers = 7;				// The maximum number of drives supported by the electronics
#elif defined(DUET3MINI_V02)
constexpr size_t NumDirectDrivers = 8;				// The maximum number of drives supported by the electronics
#endif

constexpr size_t MaxSmartDrivers = NumDirectDrivers;	// The maximum number of smart drivers

#if SUPPORT_CAN_EXPANSION
constexpr size_t MaxCanDrivers = 7;					// enough to support another Mini5+ as an expansion board
constexpr unsigned int MaxCanBoards = 4;
#endif

constexpr size_t MaxBedHeaters = 2;
constexpr size_t MaxChamberHeaters = 2;
constexpr int8_t DefaultBedHeater = 0;
constexpr int8_t DefaultE0Heater = 1;				// Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 3;
constexpr size_t NumTmcDriversSenseChannels = 2;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 8;						// The maximum number of movement axes in the machine
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 5;					// The maximum number of extruders
constexpr size_t NumDefaultExtruders = 1;			// The number of drivers that we configure as extruders by default

constexpr size_t MaxAxesPlusExtruders = 8;

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 5;

constexpr unsigned int MaxTriggers = 16;			// Maximum number of triggers

constexpr size_t NumSerialChannels = 3;				// The number of serial IO channels (USB and one auxiliary UART)
#define SERIAL_MAIN_DEVICE (serialUSB)
#define SERIAL_AUX_DEVICE (serialUart0)
#define SERIAL_AUX2_DEVICE (serialUart1)

// SerialUSB
constexpr Pin UsbVBusPin = PortBPin(6);				// Pin used to monitor VBUS on USB port

//#define I2C_IFACE	Wire							// First and only I2C interface
//#define I2C_IRQn	WIRE_ISR_ID

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drivers
#ifdef DUET3MINI_V04
constexpr Pin GlobalTmc22xxEnablePin = PortCPin(28);	// The pin that drives ENN of all drivers
PortGroup * const StepPio = &(PORT->Group[2]);		// The PIO that all the step pins are on (port C)

constexpr Pin STEP_PINS[NumDirectDrivers] = { PortCPin(26), PortCPin(25), PortCPin(24), PortCPin(19), PortCPin(16), PortCPin(30), PortCPin(18) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { PortBPin(3), PortBPin(29), PortBPin(28), PortDPin(20), PortDPin(21), PortBPin(0), PortAPin(27) };
constexpr Pin DriverDiagPins[NumDirectDrivers] = { PortAPin(10), PortBPin(8), PortAPin(22), PortAPin(23), PortCPin(21), PortBPin(10), PortCPin(27) };
// CCL inputs that the DIAG inputs use. Bits 0-1 are the CCL LUT number. Bits 8-19 are the value to OR in to the control register for that LUT.
constexpr uint32_t CclDiagInputs[NumDirectDrivers] =
{
	0x01 | CCL_LUTCTRL_INSEL2(0x04),		// CCLIN[5] = 1.2
	0x02 | CCL_LUTCTRL_INSEL2(0x04),		// CCLIN[8]	= 2.2
	0x02 | CCL_LUTCTRL_INSEL0(0x04),		// CCLIN[6] = 2.0
	0x02 | CCL_LUTCTRL_INSEL1(0x04),		// CCLIN[7] = 2.1
	0x03 | CCL_LUTCTRL_INSEL1(0x04),		// CCLIN[10] = 3.1
	0x03 | CCL_LUTCTRL_INSEL2(0x04),		// CCLIN[11] = 3.2
	0x01 | CCL_LUTCTRL_INSEL1(0x04)			// CCLIN[4] = 1.1
};

// UART interface to stepper drivers
constexpr uint8_t TMC22xxSercomNumber = 1;
Sercom * const SERCOM_TMC22xx = SERCOM1;
constexpr IRQn TMC22xx_SERCOM_IRQn = SERCOM1_0_IRQn;
constexpr Pin TMC22xxSercomTxPin = PortAPin(0);
constexpr GpioPinFunction TMC22xxSercomTxPinPeriphMode = GpioPinFunction::D;
constexpr Pin TMC22xxSercomRxPin = PortAPin(1);
constexpr GpioPinFunction TMC22xxSercomRxPinPeriphMode = GpioPinFunction::D;
constexpr uint8_t TMC22xxSercomRxPad = 1;
constexpr Pin TMC22xxMuxPins[1] = { PortDPin(0) };
#endif

#ifdef DUET3MINI_V02
constexpr Pin GlobalTmc22xxEnablePin = PortAPin(1);	// The pin that drives ENN of all drivers
PortGroup * const StepPio = &(PORT->Group[2]);		// The PIO that all the step pins are on (port C)

constexpr Pin STEP_PINS[NumDirectDrivers] = { PortCPin(26), PortCPin(25), PortCPin(24), PortCPin(31), PortCPin(16), PortCPin(30), PortCPin(18), PortCPin(19) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { PortAPin(27), PortBPin(29), PortBPin(28), PortBPin(3), PortBPin(0), PortDPin(21), PortDPin(20), PortCPin(17) };

constexpr Pin DiagMuxPins[] = { PortAPin(23), PortDPin(11), PortBPin(10) };
constexpr Pin DiagMuxOutPin = PortAPin(3);
constexpr Pin DiagIrqPin = PortAPin(0);

// UART interface to stepper drivers
constexpr uint8_t TMC22xxSercomNumber = 1;
Sercom * const SERCOM_TMC22xx = SERCOM1;
constexpr IRQn TMC22xx_SERCOM_IRQn = SERCOM1_0_IRQn;
constexpr Pin TMC22xxSercomTxPin = PortCPin(27);
constexpr GpioPinFunction TMC22xxSercomTxPinPeriphMode = GpioPinFunction::C;
constexpr Pin TMC22xxSercomRxPin = PortCPin(28);
constexpr GpioPinFunction TMC22xxSercomRxPinPeriphMode = GpioPinFunction::C;
constexpr uint8_t TMC22xxSercomRxPad = 1;
constexpr Pin TMC22xxMuxPins[1] = { PortDPin(0) };
#endif

#define TMC22xx_HAS_ENABLE_PINS			0
#define TMC22xx_HAS_MUX					1
#define TMC22xx_USES_SERCOM				1
#define TMC22xx_VARIABLE_NUM_DRIVERS	0
#define TMC22xx_SINGLE_DRIVER			0
#define TMC22xx_USE_SLAVEADDR			1

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// In testing I found that 500kbaud was not reliable, so now using 200kbaud.
constexpr uint32_t DriversBaudRate = 250000;
constexpr uint32_t TransferTimeout = 2;										// any transfer should complete within 2 ticks @ 1ms/tick
constexpr uint32_t DefaultStandstillCurrentPercent = 75;

#if defined(DUET3MINI_V04)
constexpr float DriverSenseResistor = 0.056 + 0.02;							// in ohms
#elif defined(DUET3MINI_V02)
constexpr float DriverSenseResistor = 0.05 + 0.02;							// in ohms
#endif

constexpr float DriverVRef = 180.0;											// in mV
constexpr float DriverFullScaleCurrent = DriverVRef/DriverSenseResistor;	// in mA
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;
constexpr float MaximumMotorCurrent = 2000.0;
constexpr float MaximumStandstillCurrent = 1500.0;

// Thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortCPin(0), PortCPin(1), PortCPin(2) }; 	// Thermistor pin numbers
constexpr Pin VssaSensePin = PortBPin(4);
constexpr Pin VrefSensePin = PortBPin(5);

constexpr float DefaultThermistorSeriesR = 2200.0;							// Thermistor series resistor value in ohms
constexpr float MinVrefLoadR = (DefaultThermistorSeriesR / NumThermistorInputs) * 4700.0/((DefaultThermistorSeriesR / NumThermistorInputs) + 4700.0);
																			// there are 3 temperature sensing channels and a 4K7 load resistor
#if defined(DUET3MINI_V04)
constexpr float VrefSeriesR = 27.0;
#elif defined(DUET3MINI_V02)
constexpr float VrefSeriesR = 15.0;
#endif

// Digital pins that SPi devices have their select lines tied to
#if defined(DUET3MINI_V04)
constexpr Pin SpiTempSensorCsPins[] = { PortDPin(11), PortCPin(7) };		// SPI0_CS1, SPI0_CS2
#elif defined(DUET3MINI_V02)
constexpr Pin SpiTempSensorCsPins[] = { PortCPin(10), PortCPin(7) };		// SPI0_CS1, SPI0_CS2
#endif

// Pin that controls the ATX power on/off
#if defined(DUET3MINI_V04)
constexpr Pin ATX_POWER_PIN = PortDPin(10);									// aliased with io4.out
#elif defined(DUET3MINI_V02)
constexpr Pin ATX_POWER_PIN = PortDPin(10);									// aliased with io5.out
#endif

// Analogue pin numbers
#if defined(DUET3MINI_V04)
constexpr Pin PowerMonitorVinDetectPin = PortCPin(3);						// Vin monitor
#elif defined(DUET3MINI_V02)
constexpr Pin PowerMonitorVinDetectPin = PortBPin(8);						// Vin monitor
#endif

constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider

#ifdef DEBUG
constexpr Pin DiagPin = NoPin;												// Diag/status LED pin is shared with SWD
constexpr Pin ActLedPin = NoPin;											// Activity LED pin is shared with SWCLK
#else
constexpr Pin DiagPin = PortAPin(31);										// Diag/status LED pin
constexpr Pin ActLedPin = PortAPin(30);										// Activity LED pin (Etherne Yellow LED on v0.2 board)
#endif

constexpr bool DiagOnPolarity = false;
constexpr bool ActOnPolarity = false;

// SD cards
constexpr size_t NumSdCards = 2;

#if defined(DUET3MINI_V04)
constexpr Pin SdCardDetectPins[NumSdCards] = { PortBPin(16), PortDPin(12) };
#elif defined(DUET3MINI_V02)
constexpr Pin SdCardDetectPins[NumSdCards] = { PortBPin(16), PortAPin(2) };
#endif

constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[NumSdCards - 1] = { PortCPin(14) };
constexpr Pin SdMciPins[] = { PortAPin(20), PortAPin(21), PortBPin(18), PortBPin(19), PortBPin(20), PortBPin(21) };
constexpr GpioPinFunction SdMciPinsFunction = GpioPinFunction::I;
Sdhc * const SdDevice = SDHC1;
constexpr IRQn_Type SdhcIRQn = SDHC1_IRQn;
constexpr uint32_t ExpectedSdCardSpeed = 15000000;

// 12864 LCD
// The ST7920 datasheet specifies minimum clock cycle time 400ns @ Vdd=4.5V, min. clock width 200ns high and 20ns low.
// This assumes that the Vih specification is met, which is 0.7 * Vcc = 3.5V @ Vcc=5V
// The Duet Maestro level shifts all 3 LCD signals to 5V, so we meet the Vih specification and can reliably run at 2MHz.
// For other electronics, there are reports that operation with 3.3V LCD signals may work if you reduce the clock frequency.
// The ST7567 specifies minimum clock cycle time 50ns i.e. 20MHz @ Vcc=3.3V
constexpr uint32_t LcdSpiClockFrequency = 2000000;		// 2.0MHz
constexpr Pin LcdCSPin = PortCPin(6);
constexpr Pin EncoderPinA = PortCPin(11);
constexpr Pin EncoderPinB = PortDPin(1);
constexpr Pin EncoderPinSw = PortBPin(9);

#if defined(DUET3MINI_V04)
constexpr Pin LcdA0Pin = PortAPin(2);
constexpr Pin LcdBeepPin = PortAPin(9);
constexpr Pin LcdNeopixelOutPin = PortBPin(12);			// shared with io3.out
#elif defined(DUET3MINI_V02)
constexpr Pin LcdA0Pin = PortCPin(3);
constexpr Pin LcdBeepPin = PortAPin(22);
constexpr Pin LcdNeopixelOutPin = PortBPin(14);			// shared with io4.out
#endif

// Neopixel output
constexpr Pin NeopixelOutPin = PortAPin(8);
constexpr GpioPinFunction NeopixelOutPinFunction = GpioPinFunction::H;		// QSPI Data[0]
#define DOTSTAR_USES_USART		(0)

// Shared SPI definitions
constexpr uint8_t SharedSpiSercomNumber = 7;
constexpr Pin SharedSpiMosiPin = PortCPin(12);
constexpr Pin SharedSpiMisoPin = PortCPin(15);
constexpr Pin SharedSpiSclkPin = PortCPin(13);
constexpr GpioPinFunction SharedSpiPinFunction = GpioPinFunction::C;

// Serial on IO0
constexpr uint8_t Serial0SercomNumber = 2;
constexpr uint8_t Sercom0RxPad = 1;
#define SERIAL0_ISR0	SERCOM2_0_Handler
#define SERIAL0_ISR1	SERCOM2_1_Handler
#define SERIAL0_ISR2	SERCOM2_2_Handler
#define SERIAL0_ISR3	SERCOM2_3_Handler

constexpr Pin Serial0TxPin = PortBPin(25);
constexpr Pin Serial0RxPin = PortBPin(24);
constexpr GpioPinFunction Serial0PinFunction = GpioPinFunction::D;

// Serial on IO1
constexpr uint8_t Serial1SercomNumber = 5;
constexpr uint8_t Sercom1RxPad = 1;
#define SERIAL1_ISR0	SERCOM5_0_Handler
#define SERIAL1_ISR1	SERCOM5_1_Handler
#define SERIAL1_ISR2	SERCOM5_2_Handler
#define SERIAL1_ISR3	SERCOM5_3_Handler

constexpr Pin Serial1TxPin = PortBPin(31);
constexpr Pin Serial1RxPin = PortBPin(30);
constexpr GpioPinFunction Serial1PinFunction = GpioPinFunction::D;

// Ethernet pins
constexpr Pin EthernetMacPins[] =
{
	PortAPin(12), PortAPin(13), PortAPin(14), PortAPin(15), PortAPin(17), PortAPin(18), PortAPin(19),
	PortCPin(20), PortCPin(22), PortCPin(23)
};
constexpr GpioPinFunction EthernetMacPinsPinFunction = GpioPinFunction::L;

#if defined(DUET3MINI_V04)
constexpr Pin EthernetPhyResetPin = PortCPin(17);
#elif defined(DUET3MINI_V02)
constexpr Pin EthernetPhyResetPin = PortCPin(21);
#endif

constexpr Pin EthernetClockOutPin = PortAPin(16);
constexpr GpioPinFunction EthernetClockOutPinFunction = GpioPinFunction::M;
constexpr unsigned int EthernetClockOutGclkNumber = 2;

// WiFi pins
constexpr unsigned int WiFiUartSercomNumber = 3;
constexpr uint8_t WiFiUartRxPad = 1;
constexpr Pin WiFiUartSercomPins[] = { PortAPin(16), PortAPin(17) };
constexpr GpioPinFunction WiFiUartSercomPinsMode = GpioPinFunction::D;
constexpr IRQn WiFiUartSercomIRQn = SERCOM3_0_IRQn;			// this is the first of 4 interrupt numbers
#define SERIAL_WIFI_ISR0	SERCOM3_0_Handler
#define SERIAL_WIFI_ISR1	SERCOM3_1_Handler
#define SERIAL_WIFI_ISR2	SERCOM3_2_Handler
#define SERIAL_WIFI_ISR3	SERCOM3_3_Handler

constexpr unsigned int WiFiSpiSercomNumber = 4;
Sercom * const WiFiSpiSercom = SERCOM4;
constexpr Pin EspMosiPin = PortAPin(15);
constexpr Pin EspMisoPin = PortAPin(13);
constexpr Pin EspSclkPin = PortAPin(12);
constexpr Pin EspSSPin = PortAPin(14);
constexpr Pin WiFiSpiSercomPins[] = { EspSclkPin, EspMisoPin, EspSSPin, EspMosiPin };
constexpr GpioPinFunction WiFiSpiSercomPinsMode = GpioPinFunction::D;
constexpr IRQn WiFiSpiSercomIRQn = SERCOM4_3_IRQn;			// this is the SS Low interrupt, the only one we use
#define ESP_SPI_HANDLER		SERCOM4_3_Handler

constexpr Pin EspResetPin = EthernetPhyResetPin;
constexpr Pin EspEnablePin = PortCPin(20);
constexpr Pin EspDataReadyPin = PortAPin(18);
constexpr Pin SamTfrReadyPin = PortAPin(19);
constexpr Pin SamCsPin = PortAPin(14);

// SBC interface
constexpr unsigned int SbcSpiSercomNumber = 0;
Sercom * const SbcSpiSercom = SERCOM0;
constexpr Pin SbcSSPin = PortAPin(6);

#if defined(DUET3MINI_V04)
constexpr Pin SbcTfrReadyPin = PortAPin(3);
#elif defined(DUET3MINI_V02)
constexpr Pin SbcTfrReadyPin = PortBPin(7);
#endif

constexpr Pin SbcSpiSercomPins[] = { PortAPin(4), PortAPin(5), PortAPin(6), PortAPin(7) };
constexpr GpioPinFunction SbcSpiSercomPinsMode = GpioPinFunction::D;
constexpr IRQn SbcSpiSercomIRQn = SERCOM0_3_IRQn;			// this is the SS Low interrupt, the only one we use
#define SBC_SPI_HANDLER		SERCOM0_3_Handler

// CAN
#ifdef DUET3MINI_V04
constexpr unsigned int CanDeviceNumber = 1;					// we use CAN1
constexpr Pin CanTxPin = PortBPin(14);
constexpr Pin CanRxPin = PortBPin(15);
constexpr GpioPinFunction CanPinsMode = GpioPinFunction::H;
#endif

// Function to look up a pin name and pass back the corresponding index into the pin table
bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DueX.

#if defined(DUET3MINI_V04)

constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint Capability				PinNames
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA00 TMC UART TxD
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA01 TMC UART RxD
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, PinCapability::none,	"ate.lcd.a0"	},	// PA02 LCD A0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, PinCapability::none,	"ate.sbc.dr"	},	// PA03 SBC data ready
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA04 SBC SPI MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA05 SBC SPI SCLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA06 SBC SPI SS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA07 SBC SPI MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA08 Neopixel output (QSPI MOSI)
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"are.lcd.buzz"	},	// PA09 LCD buzzer
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d0.diag"	},	// PA10 driver 0 diag
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out4"			},	// PA11 OUT4
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA12 Ethernet/WiFi SCLK (SERCOM4.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA13 Ethernet/WiFi MISO (SERCOM4.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA14 Ethernet/WiFi SS (SERCOM4.2)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA15 Ethernet/WiFi MOSI (SERCOM4.3)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA16 Ethernet GCLK2 out/WiFi RxD (SERCOM3.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA17 Ethernet/WiFi TxD (SERCOM3.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	PinCapability::none,	nullptr			},	// PA18 Ethernet/WiFi ESP_DATA_RDY
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA19 Ethernet/WiFi SAM_TRANSFER_RDY
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA20 SDHC CMD
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA21 SDHC CLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d2.diag"	},	// PA22 driver 2 diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d3.diag"	},	// PA23 driver 3 diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA24 USB
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA25 USB
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d6.dir"	},	// PA27 driver 6 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA30 swclk/ACT LED
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA31 swdio/STATUS LED

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d5.dir"	},	// PB00 driver 5 dir
	{ TcOutput::tc7_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out6,laser,vfd" },	// PB01 OUT6
	{ TcOutput::none,	TccOutput::tcc2_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out5"			},	// PB02 OUT5
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d0.dir"	},	// PB03 driver 0 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_6,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB04 VssaMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_7,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB05 VrefMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_8,	SercomIo::none,		SercomIo::none,		6,	PinCapability::none,	"ate.vbus"		},	// PB06 Vbus
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_9,	SercomIo::none,		SercomIo::none,		7,	PinCapability::ainr,	"io3.in"		},	// PB07 IO3_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d1.diag"	},	// PB08 driver 1 diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	PinCapability::none,	"ate.enc.sw"	},	// PB09 ENC SW
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d5.diag"	},	// PB10 driver 5 diag
	{ TcOutput::tc5_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out3"			},	// PB11 OUT3
	{ TcOutput::none,	TccOutput::tcc3_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"io3.out"		},	// PB12 IO3_OUT
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out2"			},	// PB13 OUT2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	nullptr			},	// PB14 CAN Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::read,	nullptr			},	// PB15 CAN Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.cd"		},	// PB16 SD CD
	{ TcOutput::tc6_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out0"			},	// PB17 OUT0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB18 SD DAT0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB19 SD DAT1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none	,	Nx,	PinCapability::none,	nullptr			},	// PB20 SD DAT2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB21 SD DAT3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB22 crystal XIN1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB23 crystal XOUT1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom2d,	SercomIo::none,		8,	PinCapability::read,	"io0.in"		},	// PB24 IO0_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::sercom2d,	Nx,	PinCapability::write,	"io0.out"		},	// PB25 IO0_OUT
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	PinCapability::read,	"out4.tach"		},	// PB26 OUT4 tacho
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	PinCapability::read,	"out3.tach"		},	// PB27 OUT3 tacho
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d2.dir"	},	// PB28 driver 2 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d1.dir"	},	// PB29 driver 1 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom5d,	SercomIo::none,		14,	PinCapability::read,	"io1.in"		},	// PB30 IO1_IN
	{ TcOutput::none,	TccOutput::tcc0_7G,	AdcInput::none,		SercomIo::none,		SercomIo::sercom5d,	Nx,	PinCapability::wpwm,	"io1.out"		},	// PB31 IO1_OUT

	// Port C
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_10,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::ain,		"temp0"			},	// PC00 thermistor0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_11,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::ain,		"temp1"			},	// PC01 thermistor1
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_4,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::ain,		"temp2"			},	// PC02 thermistor2
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_5,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.vin"		},	// PC03 VIN monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	PinCapability::read,	"io5.in"		},	// PC04 IO5_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	PinCapability::read,	"io4.in"		},	// PC05 IO4_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::write,	"spi.cs3"		},	// PC06 SPI_CS3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::write,	"spi.cs2"		},	// PC07 SPI CS2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC08 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC09 not on chip
	{ TcOutput::none,	TccOutput::tcc1_4G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out1"			},	// PC10 OUT1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	PinCapability::none,	"ate.enc.a"		},	// PC11 ENC A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC12 SPI MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC13 SPI SCK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC14 SPI CS0 (external SD card)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC15 SPI_MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d4.step"	},	// PC16 driver4 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC17 phy reset/ESP reset
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d6.step"	},	// PC18 driver 6 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d3.step"	},	// PC19 driver 3 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC20 Ethernet/WiFi ESP_EN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d6.diag"	},	// PC21 driver 4 diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC22 Ethernet
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC23 Ethernet
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d2.step"	},	// PC24 driver2 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d1.step"	},	// PC25 driver1 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d0.step"	},	// PC26 driver0 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d6.diag"	},	// PC27 driver 6 diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC28 driver ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.d5.step"	},	// PC30 driver 5 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_13,	SercomIo::none,		SercomIo::none,		15,	PinCapability::ainr,	"io6.in"		},	// PC31 IO6_IN

	// Port D
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD00 drivers UART mux
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	PinCapability::none,	"ate.enc.b"		},	// PD01 ENC B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD02 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD03 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom6d,	SercomIo::none,		3,	PinCapability::read,	"io2.in"		},	// PD08 IO2_IN
	{ TcOutput::none,	TccOutput::tcc0_2F,	AdcInput::none,		SercomIo::none,		SercomIo::sercom6d,	Nx,	PinCapability::write,	"io2.out"		},	// PD09 IO2_OUT
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::write,	"io4.out,pson"	},	// PD10 IO4_OUT and PS_ON
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::write,	"spi.cs1"		},	// PD11 SPI2 CS1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	PinCapability::read	,	"ate.spi2.cd"	},	// PD12 SPI2_CD

#if 1
	// Port D 13-19 are not on the chip
	// Port D 20-21 are driver 3 dir and driver 4 dir but those don't need to be in the pin table
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD13 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD14 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD15 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD16 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD17 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD20 driver6 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD21 driver5 dir
#endif
};

#elif defined(DUET3MINI_V02)

constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint Capability				PinNames
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	PinCapability::none,	nullptr			},	// PA00 driver diag interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA01 drivers ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, PinCapability::none,	nullptr			},	// PA02 external SD card CD
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, PinCapability::none,	nullptr			},	// PA03 drivers diag mux out
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA04 SBC SPI MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA05 SBC SPI SCLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA06 SBC SPI SS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA07 SBC SPI MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA08 Neopixel output (QSPI MOSI)
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out3"			},	// PA09 OUT3
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_10,	SercomIo::none,		SercomIo::none,		10,	PinCapability::ainr,	"io3.in"		},	// PA10 IO3_IN
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out4"			},	// PA11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA12 Ethernet/WiFi SCLK (SERCOM4.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA13 Ethernet/WiFi MISO (SERCOM4.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA14 Ethernet/WiFi SS (SERCOM4.2)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA15 Ethernet/WiFi MOSI (SERCOM4.3)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA16 Ethernet GCLK2 out/WiFi RxD (SERCOM3.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA17 Ethernet/WiFi TxD (SERCOM3.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,	PinCapability::none,	nullptr			},	// PA18 Ethernet/WiFi ESP_DATA_RDY
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA19 Ethernet/WiFi SAM_TRANSFER_RDY
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA20 SDHC CMD
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA21 SDHC CLK
	{ TcOutput::none,	TccOutput::tcc1_6F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA22 12864 buzzer
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA23 driver diag mux S0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA24 USB
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA25 USB
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA27 driver0 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA30 swclk/Ethernet LED Y
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA31 swdio/diag LED

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB00 driver4 dir
	{ TcOutput::tc7_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out6,laser,vfd" },	// PB01 OUT6
	{ TcOutput::none,	TccOutput::tcc2_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out5"			},	// PB02 OUT5
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB03 driver3 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_6,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB04 VssaMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_7,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB05 VrefMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_8,	SercomIo::none,		SercomIo::none,		6,	PinCapability::none,	nullptr			},	// PB06 Vbus
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB07 SBC data rdy
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_0,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB08 VinMon
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	PinCapability::none,	nullptr			},	// PB09 ENC SW
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB10 driver diag mux S2
	{ TcOutput::tc5_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out1"			},	// PB11 OUT1
	{ TcOutput::none,	TccOutput::tcc3_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"io3.out"		},	// PB12 IO3_OUT
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out2"			},	// PB13 OUT2
	{ TcOutput::none,	TccOutput::tcc4_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"io4.out"		},	// PB14 IO4_OUT
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		15,	PinCapability::read,	"io4.in"		},	// PB15 IO4_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB16 SD CD
	{ TcOutput::tc6_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::wpwm,	"out0"			},	// PB17 OUT0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB18 SD DAT0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB19 SD DAT1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none	,	Nx,	PinCapability::none,	nullptr			},	// PB20 SD DAT2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB21 SD DAT3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB22 crystal XIN1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB23 crystal XOUT1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom2d,	SercomIo::none,		8,	PinCapability::read,	"io0.in"		},	// PB24 IO0_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::sercom2d,	Nx,	PinCapability::write,	"io0.out"		},	// PB25 IO0_OUT
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,	PinCapability::read,	"out4.tach"		},	// PB26 OUT4 tacho
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,	PinCapability::read,	"out3.tach"		},	// PB27 OUR3 tacho
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB28 driver 2 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB29 driver 1 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom5d,	SercomIo::none,		14,	PinCapability::read,	"io1.in"		},	// PB30 IO1_IN
	{ TcOutput::none,	TccOutput::tcc0_7G,	AdcInput::none,		SercomIo::none,		SercomIo::sercom5d,	Nx,	PinCapability::wpwm,	"io1.out"		},	// PB31 IO1_OUT

	// Port C
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_10,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::ain,		"temp0"			},	// PC00 thermistor0
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_11,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::ain,		"temp1"			},	// PC01 thermistor1
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_4,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::ain,		"temp2"			},	// PC02 thermistor2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC03 12864 A0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	PinCapability::read,	"io6.in"		},	// PC04 IO6_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,	PinCapability::read,	"io5.in"		},	// PC05 IO5_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::write,	"spi.cs3"		},	// PC06 SPI_CS3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::write,	"spi.cs2"		},	// PC07 SPI CS2
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC08 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC09 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::write,	"spi.cs1"		},	// PC10 SPI CS1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	PinCapability::none,	nullptr			},	// PC11 ENC A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC12 SPI MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC13 SPI SCK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC14 SPI CS0 (external SD card)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC15 SPI_MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC16 driver4 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC17 driver7 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC18 driver6 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC19 driver7 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC20 Ethernet/WiFi ESP_RST
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC21 Ethernet PHY_RST/WiFi ESP_EN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC22 Ethernet
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC23 Ethernet
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC24 driver2 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC25 driver1 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC26 driver0 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC27 stepper TxD (SERCOM 1.0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC28 stepper RxD (SERCOM 1.1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC30 driver5 step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PC31 driver3 step

	// Port D
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD00 drivers UART mux
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,	PinCapability::none,	nullptr			},	// PD01 ENC B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD02 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD03 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::sercom6d,	SercomIo::none,		3,	PinCapability::read,	"io2.in"		},	// PD08 IO2_IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::sercom6d,	Nx,	PinCapability::write,	"io2.out"		},	// PD09 IO2_OUT
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::write,	"io5.out,pson"	},	// PD10 IO5_OUT and PS_ON
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PD11 drivers diag mux S1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	PinCapability::read	,	"io7.in"		},	// PD12

#if 1
	// Port D 13-19 are not on the chip
	// Port D 20-21 are driver6 dir and driver5 dir but those don't need to be in the pin table
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD13 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD14 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD15 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD16 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD17 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD20 driver6 dir
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr		},	// PD21 driver5 dir
#endif
};

#endif

constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable);
static_assert(NumNamedPins == 32+32+32+13);

// DMA channel assignments. Channels 0-3 have individual interrupt vectors, channels 4-31 share an interrupt vector.
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel FirstAdcDmaChannel = 2;			// the ADCs use 4 DMA channels
constexpr DmaChannel DmacChanWiFiTx = 6;
constexpr DmaChannel DmacChanWiFiRx = 7;
constexpr DmaChannel DmacChanSbcTx = 8;
constexpr DmaChannel DmacChanSbcRx = 9;
constexpr DmaChannel DmacChanDotStarTx = 10;

constexpr unsigned int NumDmaChannelsUsed = 11;

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 1;				// the baud rate is 250kbps so this is not very critical
constexpr DmaPriority DmacPrioAdcTx = 0;
constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioWiFi = 3;					// high speed SPI in slave mode
constexpr DmaPriority DmacPrioSbc = 3;					// high speed SPI in slave mode
constexpr DmaPriority DmacPrioDotStar = 1;				// QSPI in master mode

// Timer allocation
// TC2 and TC3 are used for step pulse generation and software timers
TcCount32 * const StepTc = &(TC2->COUNT32);
constexpr IRQn StepTcIRQn = TC2_IRQn;
constexpr unsigned int StepTcNumber = 2;
#define STEP_TC_HANDLER		TC2_Handler

// SAME5x event channel allocation, max 32 channels. Only the first 12 provide a synchronous or resynchronised path and can generate interrupts.

#ifdef DUET3MINI_V04
constexpr EventNumber CclLut0Event = 0;					// this uses up 4 channels
constexpr EventNumber NextFreeEvent = CclLut0Event + 4;
#endif

// Step pulse generation
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
		StepPio->OUTSET.reg = driverMap;				// all step pins are on port C
	}

	// Set the specified step pins low. This needs to be fast.
	static inline void __attribute__((always_inline)) StepDriversLow(uint32_t driverMap) noexcept
	{
		StepPio->OUTCLR.reg = driverMap;				// all step pins are on port C
	}
}

#endif /* SRC_DUET3MINI_PINS_DUET3MINI_H_ */
