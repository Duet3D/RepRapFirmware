/*
 * Pins_INDX.h
 *
 *  Created on: 27 Mar 2026
 *      Author: David
 */

#ifndef SRC_CONFIG_PINS_INDX_H_
#define SRC_CONFIG_PINS_INDX_H_

#include <PinDescription.h>
#include <SPI/SpiParameters.h>
#include <UART/UartParameters.h>

#define DEFAULT_BOARD_TYPE		 BoardType::Indx

#define BOARD_SHORT_NAME		"INDX"
#define BOARD_NAME				"INDX"
#define FIRMWARE_NAME			"RepRapFirmware for INDX"

#define IAP_FIRMWARE_FILE		"Duet3Firmware_" BOARD_SHORT_NAME ".uf2"
#define IAP_UPDATE_FILE			"Duet3_SDiap32_" BOARD_SHORT_NAME ".bin"
constexpr uint32_t IAP_IMAGE_START = 0x20028000;

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		0
#define HAS_W5500_NETWORKING	0
#define HAS_SBC_INTERFACE		0
#define SUPPORTS_SBC_OVER_SPI	0
#define SUPPORTS_SBC_OVER_USB	0

#define HAS_MASS_STORAGE		0
#define HAS_HIGH_SPEED_SD		0
#define HAS_EMBEDDED_FILES		1

//#define HAS_CPU_TEMP_SENSOR	0					// according to the SAME5x errata doc, the temperature sensors don't work in revision A or D chips (revision D is latest as at 2020-06-28)
#define HAS_CPU_TEMP_SENSOR		1					// enable this as an experiment - it may be better than nothing

#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2240			0
#define SUPPORT_TMC2240_SPI		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1

#define HAS_VOLTAGE_MONITOR		1
#define ENFORCE_MAX_VIN			0
#define HAS_VREF_MONITOR		0

#define SUPPORT_CAN_EXPANSION	0

#define SUPPORT_LED_STRIPS		1
#define SUPPORT_DMA_NEOPIXEL	1					// using QSPI for Neopixels
#define NEOPIXEL_USES_QSPI		1					// using QSPI for Neopixels
#define SUPPORT_LASER			0					// support laser cutters and engravers using G1 S parameter
#define SUPPORT_IOBITS			0					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR		0					// set nonzero to support DHT temperature/humidity sensors (requires RTOS)
#define SUPPORT_12864_LCD		0					// set nonzero to support 12864 LCD and rotary encoder
#define SUPPORT_ILI9488_LCD		0
#define USE_FONT_CHIP			0
#define SUPPORT_ACCELEROMETERS	0
#define SUPPORT_FTP				0
#define SUPPORT_TELNET			0
#define SUPPORT_ASYNC_MOVES		0
#define SUPPORT_PANELDUE_FLASH	0
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_SLOW_DRIVERS	0

#define USE_CACHE				1					// set nonzero to enable the cache
#define USE_MPU					0					// set nonzero to enable the memory protection unit

// Disable the kinematics we don't need to save flash memory space
#define SUPPORT_LINEAR_DELTA	0
#define SUPPORT_ROTARY_DELTA	0
#define SUPPORT_POLAR			0
#define SUPPORT_SCARA			0
#define SUPPORT_FIVEBARSCARA	0
#define SUPPORT_HANGPRINTER		0

// The physical capabilities of the machine

constexpr size_t NumDirectDrivers = 1;				// The maximum number of drives supported by the electronics
constexpr size_t MaxSmartDrivers = NumDirectDrivers;	// The maximum number of smart drivers

constexpr size_t MaxSensors = 32;

constexpr size_t MaxHeaters = 4;					// The maximum number of heaters in the machine
constexpr size_t MaxPortsPerHeater = 2;
constexpr size_t MaxMonitorsPerHeater = 3;			// The maximum number of monitors per heater

constexpr size_t MaxBedHeaters = 4;
constexpr size_t MaxChamberHeaters = 4;
constexpr size_t MaxHeatersPerBed = 4;
constexpr size_t MaxHeatersPerChamber = 4;

constexpr size_t NumThermistorInputs = 3;
constexpr size_t NumTmcDriversSenseChannels = 1;

constexpr size_t MaxZProbes = 2;
constexpr size_t MaxGpInPorts = 10;
constexpr size_t MaxGpOutPorts = 10;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 4;						// The maximum number of movement axes in the machine
constexpr size_t MaxDriversPerAxis = 2;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 2;					// The maximum number of extruders
constexpr size_t MaxAxesPlusExtruders = 6;

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 2;

constexpr size_t MaxFans = 6;

constexpr unsigned int MaxTriggers = 16;			// Maximum number of triggers

constexpr size_t MaxSpindles = 2;					// Maximum number of configurable spindles
constexpr size_t MaxLedStrips = 2;					// Maximum number of LED strips

#define SERIAL_USB_DEVICE (serialUSB)

#if CORE_USES_TINYUSB
constexpr size_t NumUsbChannels = 2;
# define SERIAL_USB2_DEVICE (serialUSB2)
#else
constexpr size_t NumUsbChannels = 1;
#endif

#define NUM_ASYNC_PORTS			(0)
#define NUM_ASYNC_CHANNELS		(NUM_ASYNC_PORTS)

constexpr size_t NumSerialChannels = NumUsbChannels + NUM_ASYNC_CHANNELS;		// The number of serial IO channels (USB and one auxiliary UART)

// SerialUSB
constexpr Pin UsbVBusPin = NoPin;					// tinyUsb doesn't need to detect Vbus

constexpr size_t NumSdCards = 1;					// not actually an SD card, this is the embedded filestore

// Neopixel output
constexpr Pin NeopixelOutPin = PortAPin(8);
constexpr GpioPinFunction NeopixelOutPinFunction = GpioPinFunction::H;		// QSPI Data[0]
#define LEDSTRIP_USES_USART		(0)

// DMA channel assignments. Channels 0-3 have individual interrupt vectors, channels 4-31 share an interrupt vector.
// When static arbitration within a priority level is selected, lower channel number have higher priority.
// So we use the low channel numbers for the highest priority sources.
constexpr DmaChannel DmacChanWiFiTx = 0;
constexpr DmaChannel DmacChanWiFiRx = 1;
constexpr DmaChannel DmacChanTmcTx = 2;
constexpr DmaChannel DmacChanTmcRx = 3;
constexpr DmaChannel DmacChanSspiTx = 4;
constexpr DmaChannel DmacChanSspiRx = 5;
constexpr DmaChannel DmacChanLedTx = 6;

constexpr unsigned int NumDmaChannelsUsed = 7;

// The DMAC has priority levels 0-3 but on revision A chips it is unsafe to use multiple levels
// Fortunately, all our SAME54P20Achips seem to be revision D
constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 1;				// the baud rate is 100kbps so this is not very critical
constexpr DmaPriority DmacPrioWiFi = 2;					// high speed SPI in slave mode
constexpr DmaPriority DmacPrioSbc = 2;					// high speed SPI in slave mode
constexpr DmaPriority DmacPrioSspiTx = 3;				// high speed SPI in master mode
constexpr DmaPriority DmacPrioSspiRx = 2;				// high speed SPI in master mode
constexpr DmaPriority DmacPrioLed = 3;					// high speed SPI in master mode

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.

// Drivers
constexpr Pin GlobalTmcEnablePin = PortBPin(4);
constexpr Pin GlobalTmcCSPin = PortAPin(10);

#define TMC_USES_SERCOM			1
constexpr uint8_t TmcSercomNumber = 0;
Sercom * const SERCOM_TMC = SERCOM0;

constexpr Pin TMCMosiPin = PortAPin(4);
constexpr GpioPinFunction TMCMosiPinPeriphMode = GpioPinFunction::D;
constexpr Pin TMCMisoPin = PortAPin(7);
constexpr GpioPinFunction TMCMisoPinPeriphMode = GpioPinFunction::D;
constexpr Pin TMCSclkPin = PortAPin(5);
constexpr GpioPinFunction TMCSclkPinPeriphMode = GpioPinFunction::D;

constexpr uint32_t Tmc2240CurrentRange = 0x01;								// which current range we set the TMC2240 to (2A)
constexpr uint32_t Tmc2240SlopeControl = 0x01;								// which slope control we set the TMC2240 to (200V/us)
constexpr float Tmc2240Rref = 24.0;											// TMC2240 reference resistor in Kohms
constexpr float DriverFullScaleCurrent = 24000/Tmc2240Rref;					// in mA, assuming we set the range bits in the DRV_CONF register to 0x01
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;			// with RRef = 15K this works out as 1.6A so this is the maximum current we can ask for

constexpr float MaxMotorCurrent = DriverFullScaleCurrent;

constexpr uint32_t DefaultStandstillCurrentPercent = 75;

PortGroup * const StepPio = &(PORT->Group[1]);								// the PIO that all the step pins are on
constexpr Pin STEP_PINS[NumDirectDrivers] = { PortBPin(12) };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { PortBPin(23) };
constexpr Pin DriverDiagPins[NumDirectDrivers] = { PortBPin(07) };

// Thermistors
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { PortAPin(11), PortBPin(8), PortBPin(9) }; 	// Thermistor pin numbers

constexpr float DefaultThermistorSeriesR = 4700.0;							// Thermistor series resistor value in ohms

// Analogue pin numbers
constexpr Pin PowerMonitorVinDetectPin = PortAPin(2);						// Vin monitor
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;							// to be confirmed
constexpr float PowerMonitorVoltageRange = VinDividerRatio * 3.3;

#ifdef DEBUG
constexpr Pin DiagPin = NoPin;												// Diag/status LED pin is shared with SWD
constexpr Pin ActLedPin = NoPin;											// Activity LED pin
#else
constexpr Pin DiagPin = PortAPin(31);										// Diag/status LED pin
constexpr Pin ActLedPin = PortAPin(30);										// Activity LED pin
#endif

constexpr bool DiagOnPolarity = false;
constexpr bool ActOnPolarity = false;

// Shared SPI definitions
constexpr SpiParameters SharedSpiParams =
{
	.sercomNumber = 7,
	.mosiPin = PortCPin(12),
	.misoPin = PortCPin(15),
	.sclkPin = PortCPin(13),
	.pinFunction = GpioPinFunction::C,
	.dataInPad = 3,
	.dataOutPad = 0,
	.dmaChanTx = DmacChanSspiTx,
	.dmaChanRx = DmacChanSspiRx,
	.dmaPrioTx = DmacPrioSspiTx,
	.dmaPrioRx = DmacPrioSspiRx,
};

// CAN buffer control
constexpr Pin CanBufferDisablePin = PortBPin(31);				// DO NOT drive this pin low, may be jumpered to +3V3 on v0.1 boards

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DueX.
// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	PinCapability::none,	"pcfan.tach"	},	// PA00 print cooling fan tacho
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"pcfan"			},	// PA01 print cooling fan out
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.vin"		},	// PA02 VIN monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	SercomIo::none,		SercomIo::none,		Nx, PinCapability::none,	nullptr			},	// PA03 board type
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA04 SPI0 MOSI (Stepper, sercom0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA05 SPI0 SCK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA06 heater voltage feedback (also on PB05)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA07 SPI0 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"led"			},	// PA08 NP out (QSPI D0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,	PinCapability::none,	"io0.in"		},	// PA09 endstop
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA10 SPI0_CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_11,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.envtemp"	},	// PA11 hot end surround thermistor
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr 		},	// PA12 I2C1 SCL (sercom4)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA13 I2C1 SDA
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA16 SPI1 MOSI (ADC, sercom1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA17 SPI1 SCLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA18 SPI1 CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, PinCapability::none,	nullptr			},	// PA19 SPI1 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	PinCapability::none,	"hsfan.tach"	},	// PA20 heatsink fan tacho
	{ TcOutput::none,	TccOutput::tcc1_5F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"hsfan"			},	// PA21 heatsink fan
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA22 I2C0 SCL (sercom3)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA23 I2C0 SDA
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA24 USB DN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA25 USB DP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	PinCapability::none,	nullptr			},	// PA27 accelerometer interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA30 swclk and LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PA31 swdio and LED1

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB00 SPI2_CS0 (AS5047D,sercom5)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB01 SPI2 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, PinCapability::none,	nullptr			},	// PB02 SPI2 MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, PinCapability::none,	nullptr			},	// PB03 SPI2 SCK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB04 Driver ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_7,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"ate.heaterv"	},	// PB05 Heater voltage feedback (also on PA06)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_8,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB06 Heater current
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	PinCapability::none,	nullptr			},	// PB07 Driver diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"boardtemp"		},	// PB08 board thermistor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"coiltemp"		},	// PB09 LDC coil temperature
	{ TcOutput::none,	TccOutput::tcc0_4F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	PinCapability::none,	nullptr			},	// PB10 LDC interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB11 LDC1612 clock (GCLK5)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB12 Driver step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB13 Driver clock (GCLK7)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB14 CAN1 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB15 CAN1 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB16 ADC clock (GCLK2)
	{ TcOutput::none,	TccOutput::tcc3_1F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB17 CCL_OUT3 for heater FET drive
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB20 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB21 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	PinCapability::none,	nullptr			},	// PB22 ADC !DRDY
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB23 driver DIR and BOOTLOADER_RESET jumper
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB24 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB25 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB27 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB29 not on chip
	{ TcOutput::none,	TccOutput::tcc4_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	"pb30"			},	// PB30 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	PinCapability::none,	nullptr			},	// PB31 USB/!CAN select

	// Virtual pins
};

constexpr size_t NumNamedPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32+32;
constexpr size_t NumVirtualPins = 0;

static_assert(NumNamedPins == NumRealPins + NumVirtualPins);

// Timer/counter used to generate step pulses and other sub-millisecond timings
constexpr unsigned int StepTcNumber = 0;
TcCount32 * const StepTc = &(TC0->COUNT32);
constexpr IRQn StepTcIRQn = TC0_IRQn;
#define STEP_TC_HANDLER			TC0_Handler

// SAME5x event channel allocation, max 32 channels. Only the first 12 provide a synchronous or resynchronised path and can generate interrupts.
constexpr EventNumber CclLut0Event = 0;					// this uses up 4 channels
constexpr EventNumber NextFreeEvent = CclLut0Event + 4;

// Step pulse generation
namespace StepPins
{
	// *** These next three functions must use the same bit assignments in the drivers bitmap ***
	// Each stepper driver must be assigned one bit in a 32-bit word, in such a way that multiple drivers can be stepped efficiently
	// and more or less simultaneously by doing parallel writes to several bits in one or more output ports.
	// All our step pins are on port B, so the bitmap is just the map of step bits in port B.

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
		StepPio->OUTSET.reg = driverMap;				// all step pins are on port B
	}

	// Set the specified step pins low. This needs to be fast.
	static inline void __attribute__((always_inline)) StepDriversLow(uint32_t driverMap) noexcept
	{
		StepPio->OUTCLR.reg = driverMap;				// all step pins are on port B
	}
}

#endif /* SRC_CONFIG_PINS_INDX_H_ */
