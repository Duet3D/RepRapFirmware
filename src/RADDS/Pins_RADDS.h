#ifndef PINS_DUET_H__
#define PINS_DUET_H__

#define FIRMWARE_NAME "RepRapFirmware for RADDS"

// Features definition
#define HAS_LWIP_NETWORKING		0
#define HAS_WIFI_NETWORKING		0
#define HAS_CPU_TEMP_SENSOR		0				// enabling the CPU temperature sensor disables Due pin 13 due to bug in SAM3X
#define HAS_HIGH_SPEED_SD		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_VREF_MONITOR		0
#define ACTIVE_LOW_HEAT_ON		0

const size_t NumFirmwareUpdateModules = 1;
#define IAP_UPDATE_FILE "iapradds.bin"
#define IAP_FIRMWARE_FILE "RepRapFirmware-RADDS.bin"

// Default board type
#define DEFAULT_BOARD_TYPE BoardType::RADDS_15
#define ELECTRONICS "RADDS"

#define SUPPORT_INKJET		0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND		0					// set nonzero to support Roland mill
#define SUPPORT_SCANNER		0					// set nonzero to support FreeLSS scanners
#define SUPPORT_IOBITS		0					// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR	0					// set nonzero to support DHT temperature/humidity sensors

// The physical capabilities of the machine

// The number of drives in the machine, including X, Y, and Z plus extruder drives
constexpr size_t NumDirectDrivers = 9;
constexpr size_t MaxTotalDrivers = NumDirectDrivers;

// The number of heaters in the machine
// 0 is the heated bed even if there isn't one.
constexpr size_t NumEndstops = 4;					// The number of inputs we have for endstops, filament sensors etc.
constexpr size_t NumHeaters = 4;
constexpr size_t NumExtraHeaterProtections = 4;		// The number of extra heater protection instances
constexpr size_t NumThermistorInputs = 4;

constexpr size_t MinAxes = 3;						// The minimum and default number of axes
constexpr size_t MaxAxes = 6;						// The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES

constexpr size_t MaxExtruders = NumDirectDrivers - MinAxes;	// The maximum number of extruders
constexpr size_t MaxDriversPerAxis = 4;				// The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 5;

constexpr size_t NUM_SERIAL_CHANNELS = 2;
// Use TX0/RX0 for the auxiliary serial line
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial1

// SerialUSB
constexpr Pin UsbVBusPin = NoPin;					// Pin used to monitor VBUS on USB port. Not needed for SAM3X.

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to NoPin to flag unavailability.
// DRIVES
//			                                    X   Y   Z  E1  E2  E3  E4  E5  E6
constexpr Pin ENABLE_PINS[NumDirectDrivers] =    { 26, 22, 15, 62, 65, 49, 37, 31, 68 };
//			                                   A15 A12 A09 A02 B19 C12 C03 D06 B16
constexpr Pin STEP_PINS[NumDirectDrivers] =      { 24, 17,  2, 61, 64, 51, 35, 29, 67 };
constexpr Pin DIRECTION_PINS[NumDirectDrivers] = { 23, 16,  3, 60, 63, 53, 33, 27, 66 };

// Endstops
// E Stops not currently used
// Note: RepRapFirmware only as a single endstop per axis
//       gcode defines if it is a max ("high end") or min ("low end")
//       endstop.  gcode also sets if it is active HIGH or LOW
//
// 28 = RADDS X min
// 30 = RADDS Y min
// 32 = RADDS Z min
// 39 = RADDS PWM3
//
// This leaves 34, 36, and 38 as spare pins (X, Y, Z max)

constexpr Pin END_STOP_PINS[NumEndstops] = { 28, 30, 32, 39 };

// HEATERS - The bed is assumed to be the at index 0

// Analogue pin numbers
constexpr Pin TEMP_SENSE_PINS[NumThermistorInputs] = { 4, 0, 1, 2 };

// Heater outputs
// Bed PMW: D7 has hardware PWM so bed has PWM
// h0, h1 PMW: D13 & D12 are on TIOB0 & B8 which are both TC B channels, so they get PWM
// h2 bang-bang: D11 is on TIOA8 which is a TC A channel shared with h1, it gets bang-bang control

constexpr Pin HEAT_ON_PINS[NumHeaters] = { 7, 13, 12, 11 };	// bed, h0, h1, h2

// Default thermistor betas
constexpr float BED_R25 = 10000.0;
constexpr float BED_BETA = 4066.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4066.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float THERMISTOR_SERIES_RS = 4700.0;

constexpr size_t MaxSpiTempSensors = 2;

// Digital pins the 31855s have their select lines tied to
constexpr Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { 38, 36 };

// Digital pin number that controls the ATX power on/off
constexpr Pin ATX_POWER_PIN = 40;

// Z Probe pin
// Must be an ADC capable pin. Can be any of the ARM's A/D capable pins even a non-Arduino pin.
constexpr Pin Z_PROBE_PIN = A5;  // RADDS "ADC" pin

// Digital pin number to turn the IR LED on (high) or off (low)
// D34 -- unused X-max on RADDS
constexpr Pin Z_PROBE_MOD_PIN = 34;
constexpr Pin DiagPin = NoPin;

// Use a PWM capable pin
constexpr size_t NUM_FANS = 2;
constexpr Pin COOLING_FAN_PINS[NUM_FANS] = { 9, 8 }; // Fan 0, Fan 1

// Firmware will attach a FALLING interrupt to this pin
// see FanInterrupt() in Platform.cpp
//
// D25 -- Unused GPIO on AUX1
constexpr size_t NumTachos = 1;
constexpr Pin TachoPins[NumTachos] = { 25 };

// SD cards
constexpr size_t NumSdCards = 2;
constexpr Pin SdCardDetectPins[NumSdCards] = { 14, 14 };
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin };
constexpr Pin SdSpiCSPins[2] = { 87, 77 };

// Definition of which pins we allow to be controlled using M42
// Spare pins on the Arduino Due are
//
//  D5 / TIOA6  / C.25
//  D6 / PWML7  / C.24
// ### Removed: now E0_AXIS endstop D39 / PWMH2  / C.7
// D58 / AD3    / A.6
// D59 / AD2    / A.4
// ### Removed: now E6_DIR ExtV3 D66 / DAC0   / B.15
// ### Removed: now E6_STP ExtV3 D67 / DAC1   / B.16
// ### Removed: now E6_EN ExtV3 D68 / CANRX0 / A.1
// ### Removed: now MSi(=3.3V) ExtV3 D69 / CANTX0 / A.0
// D70 / SDA1   / A.17
// D71 / SCL1   / A.18
// D72 / RX LED / C.30
// D73 / TX LED / A.21

// M42 and M208 commands now use logical pin numbers, not firmware pin numbers.
// This is the mapping from logical pins 60+ to firmware pin numbers
constexpr Pin SpecialPinMap[] =
{
	5, 6, 58, 59,
	70, 71, 72, 73
};

// This next definition defines the highest one.
constexpr int HighestLogicalPin = 60 + ARRAY_SIZE(SpecialPinMap) - 1;		// highest logical pin number on this electronics

// SAM3X Flash locations (may be expanded in the future)
constexpr uint32_t IAP_FLASH_START = 0x000F0000;
constexpr uint32_t IAP_FLASH_END = 0x000FFBFF;		// don't touch the last 1KB, it's used for NvData

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
