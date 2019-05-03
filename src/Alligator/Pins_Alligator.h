#ifndef PINS_DUET_H__
#define PINS_DUET_H__

#define FIRMWARE_NAME "RepRapFirmware for Alligator"

// Features definition
#define HAS_LEGACY_NETWORKING	1
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_HIGH_SPEED_SD		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_VREF_MONITOR		0
#define ACTIVE_LOW_HEAT_ON		0

const size_t NumFirmwareUpdateModules = 1;
#define IAP_UPDATE_FILE "iapalligator.bin"
#define IAP_FIRMWARE_FILE "RepRapFirmware-Alligator.bin"

// Default board type
#define DEFAULT_BOARD_TYPE BoardType::Alligator_2

#define SUPPORT_INKJET		0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND		0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER		0					// set nonzero to support FreeLSS scanners
#define SUPPORT_IOBITS		0					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR	0					// set nonzero to support DHT temperature/humidity sensors

// The physical capabilities of the machine

// Alligator + Piggy module max 7 stepper driver
constexpr size_t NumDirectDrivers = 7;				// The number of drives in the machine, including X, Y, and Z plus extruder drives
constexpr size_t MaxTotalDrivers = NumDirectDrivers;

constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t NumEndstops = 6;					// The number of inputs we have for endstops, filament sensors etc.
// Alligator + Piggy module max 5 heaters
constexpr size_t NumHeaters = 5;					// The number of heaters in the machine; 0 is the heated bed even if there isn't one

constexpr size_t NumExtraHeaterProtections = 4;		// The number of extra heater protection instances
constexpr size_t NumThermistorInputs = 5;

constexpr size_t MaxAxes = 7;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxExtruders = NumDirectDrivers - MinAxes;	// The maximum number of extruders

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 5;

constexpr size_t NUM_SERIAL_CHANNELS = 3;			// The number of serial IO channels (USB and two auxiliary UARTs)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_AUX2_DEVICE Serial1

constexpr Pin UsbVBusPin = NoPin;					// Pin used to monitor VBUS on USB port. Not needed for SAM3X.

// The numbers of entries in each array must correspond with the values of NumDirectDrivers, MaxAxes, or NumHeaters. Set values to NoPin to flag unavailability.
const Pin ENABLE_PINS[NumDirectDrivers] = { 24, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin };
const Pin STEP_PINS[NumDirectDrivers] = { X16, X14, X1, 5, 28, 11, X9 };
const Pin DIRECTION_PINS[NumDirectDrivers] = { 2, 78, X13, 4, 27, 29, 12};

// MICROSTEPPING Pins
const Pin MICROSTEPPING_PINS[NumDirectDrivers - MinAxes] = { X12, X10, 44, 45 };

// Motor FAULT Pin
const Pin MotorFaultDetectPin = 22;

// Endstops
// RepRapFirmware only has a single endstop per axis.
// Gcode defines if it is a max ("high end") or min ("low end") endstop and sets if it is active HIGH or LOW.
// Alligator End-stop pinout mapping for RepRapFirmware:
// 5V SIGN SIGN GND , 5V SIGN SIGN GND, 5V SIGN   SIGN    GND
//     X    E0            Y    E1           Z   E2-Zprobe
const Pin END_STOP_PINS[NumEndstops] = { 33, 35, 38, 34, 37, 39 };

// SPI DAC Motor for Current Vref
const size_t MaxSpiDac = 2;
const Pin SPI_DAC_CS[MaxSpiDac] = { 53, 6 };

// HEATERS
const Pin TEMP_SENSE_PINS[NumThermistorInputs] = { 1, 0, 2, 3, 4 };	// Analogue pin numbers

// h1,h2,h3,h4: X2,8,9,X8 is hardware PWM
// b: X3 is not hardware PWM
const Pin HEAT_ON_PINS[NumHeaters] = { X3, X2, 8, 9, X8 };

// Default thermistor parameters
// Bed thermistor: http://uk.farnell.com/epcos/b57863s103f040/sensor-miniature-ntc-10k/dp/1299930?Ntt=129-9930
// Hot end thermistor: http://www.digikey.co.uk/product-search/en?x=20&y=11&KeyWords=480-3137-ND
const float BED_R25 = 10000.0;
const float BED_BETA = 3988.0;
const float BED_SHC = 0.0;
const float EXT_R25 = 100000.0;
const float EXT_BETA = 4388.0;
const float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
// On Alligator board it is selectable via dip-switch 4700 ohms or 1000 ohms.
// Default value is 4700, If your dip-switch has been moved to 1000 set it wit M305 command
const float THERMISTOR_SERIES_RS = 4700.0;

// Number of SPI temperature sensors to support
const size_t MaxSpiTempSensors = 1;

// Digital pins the 31855s have their select lines tied to
const Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { NoPin };


// Arduino Due pin number that controls the ATX power on/off
const Pin ATX_POWER_PIN = NoPin;										// Arduino Due pin number that controls the ATX power on/off

// Z probe
const Pin Z_PROBE_PIN = 39;	 											// Z min pin ,Last signal of the end-stop connectors

// Digital pin number to turn the IR LED on (high) or off (low)
const Pin Z_PROBE_MOD_PIN = NoPin;										// Digital pin number to turn the IR LED on (high) or off (low) on Duet v0.6 and v1.0 (PB21)
const Pin DiagPin = NoPin;

// Pin number that the DAC that controls the second extruder motor current on the Duet 0.8.5 is connected to
const int Dac0DigitalPin = NoPin;										// Arduino Due pin number corresponding to DAC0 output pin

// COOLING FANS
const size_t NUM_FANS = 4;
// Fan1: X0 PA5 is in A peripheral is in TIOA2 TC A channel, it get hardware PWM
// Fan2: 31 is not hardware PWM
// J5 pin1  black connector ha hardware PWM, attached to FANS
const Pin COOLING_FAN_PINS[NUM_FANS] = { X0, 31, X17};
constexpr size_t NumTachos = 0;
constexpr Pin TachoPins[NumTachos] = { };

// SD cards
const size_t NumSdCards = 1;
const Pin SdCardDetectPins[NumSdCards] = { 87 };
const Pin SdWriteProtectPins[NumSdCards] = { NoPin };
const Pin SdSpiCSPins[NumSdCards] = { 77 };

// SPI EUI48 Mac Address CS Pin
const Pin Eui48csPin  = 26;												// Chip select Eui48 eeprom

// Other SPI CS Pins
const Pin SpiEEPROMcsPin  = 25;											// SPI EEPROM, for now not implemented
const Pin SpiFLASHcsPin  =  23;											// SPI FLASH, for now not implemented

#if SUPPORT_INKJET
// Inkjet control pins
const Pin INKJET_SERIAL_OUT = 21;										// Serial bitpattern into the shift register
const Pin INKJET_SHIFT_CLOCK = 20;										// Shift the register
const Pin INKJET_STORAGE_CLOCK = 67;									// Put the pattern in the output register
const Pin INKJET_OUTPUT_ENABLE = 66;									// Make the output visible
const Pin INKJET_CLEAR = 65;											// Clear the register to 0
#endif

// Expansion Settings
// J14 Expansion connector GPIOs has firmware voltage level selector,  3.3V  or 5V
const Pin ExpansionVoltageLevelPin = 65;								// Translator voltage level pin
const size_t ExpansionVoltageLevel =  3;								// Default expansion voltage level value 3 for 3.3V , 5 for 5V
const Pin ExpansionPiggyDetectPin =  7;									// Piggy modulee Detect Pin

// Reset Pins
const Pin FTDIconverterResetPin = 32;									// FTDI USB converter on Serial reset PIN
const Pin EthernetPhyResetPin = 73;										// Ethernet PHY chip Reset Pin


// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This is the mapping from logical pins 60+ to firmware pin numbers
const Pin SpecialPinMap[] =
{
		// RPI-GEN0 - RPI-GEN6 on J15 RaspberryPI shared GPIO
		// PC11 PC12 PC13 PC14 PC15 PC16 PC17
			X4 , 51 , 50 , 49 , 48 , 47 , 46 ,
		// J5 ,only PC8 and PC9, PC4 configured as PWM
		// PC8  PC9
		    X5 , 41,
		// J14 Expansion
		// PB18  PD5  PD4  PA13  PA12  PA11  PA10
			63 , 15 , 14 ,  16 ,  17 ,  18 ,  19
};

// This next definition defines the highest one.
const int HighestLogicalPin = 60 + ARRAY_SIZE(SpecialPinMap) - 1;		// highest logical pin number on this electronics

// SAM3X Flash locations (may be expanded in the future)
const uint32_t IAP_FLASH_START = 0x000F0000;
const uint32_t IAP_FLASH_END = 0x000FFBFF;								// don't touch the last 1KB, it's used for NvData

// Timer allocation
#define NETWORK_TC			(TC1)
#define NETWORK_TC_CHAN		(1)
#define NETWORK_TC_IRQN		TC4_IRQn
#define NETWORK_TC_HANDLER	TC4_Handler
#define NETWORK_TC_ID		ID_TC4

#define STEP_TC				(TC1)
#define STEP_TC_CHAN		(0)
#define STEP_TC_IRQN		TC3_IRQn
#define STEP_TC_HANDLER		TC3_Handler
#define STEP_TC_ID			ID_TC3

#endif
