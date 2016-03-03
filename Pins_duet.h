#ifndef PINS_DUET_H__
#define PINS_DUET_H__

// What are we supposed to be running on
#define ELECTRONICS "Duet (+ Extension)"

// Default board type
#define DEFAULT_BOARD_TYPE BoardType::Duet_06

#define SUPPORT_INKJET		0					// set nonzero to support inkjet control
#define SUPPORT_ROLAND		0					// set nonzero to support Roland mill

// The physical capabilities of the machine

const size_t DRIVES = 9;						// The number of drives in the machine, including X, Y, and Z plus extruder drives
#define DRIVES_(a,b,c,d,e,f,g,h,i) { a,b,c,d,e,f,g,h,i }

const int8_t HEATERS = 7;						// The number of heaters in the machine; 0 is the heated bed even if there isn't one
#define HEATERS_(a,b,c,d,e,f,g) { a,b,c,d,e,f,g }

const size_t AXES = 3;							// The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
const size_t NUM_SERIAL_CHANNELS = 3;			// The number of serial IO channels (USB and two auxiliary UARTs)
#define SERIAL_MAIN_DEVICE SerialUSB
#define SERIAL_AUX_DEVICE Serial
#define SERIAL_AUX2_DEVICE Serial1

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to -1 to flag unavailability.

// DRIVES

const Pin ENABLE_PINS[DRIVES] = { 29, 27, X1, X0, 37, X8, 50, 47, X13 };
const bool ENABLE_VALUES[DRIVES] = { false, false, false, false, false, false, false, false, false };	// What to send to enable a drive
const Pin STEP_PINS[DRIVES] = { 14, 25, 5, X2, 41, 39, X4, 49, X10 };
const Pin DIRECTION_PINS[DRIVES] = { 15, 26, 4, X3, 35, 53, 51, 48, X11 };
const bool DIRECTIONS[DRIVES] = { BACKWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS };	// What each axis needs to make it go forwards - defaults

// Endstops
// RepRapFirmware only has a single endstop per axis. gcode defines if it is a max ("high end") or min ("low end") endstop. gcode also sets if it is active HIGH or LOW.
const Pin END_STOP_PINS[DRIVES] = { 11, 28, 60, 31, 24, 46, 45, 44, X9 };

// Indices for motor current digipots (if any): first 4 are for digipot 1 (on duet), second 4 for digipot 2 (on expansion board)
const uint8_t POT_WIPES[8] = { 1, 3, 2, 0, 1, 3, 2, 0 };
const float SENSE_RESISTOR = 0.1;										// Stepper motor current sense resistor
const float MAX_STEPPER_DIGIPOT_VOLTAGE = (3.3 * 2.5 / (2.7 + 2.5));	// Stepper motor current reference voltage
const float MAX_STEPPER_DAC_VOLTAGE = 2.12;								// Stepper motor current reference voltage for E1 if using a DAC
const int DAC0_DIGITAL_PIN = 66;										// Arduino Due pin number corresponding to DAC0 output pin

// HEATERS

const bool HEAT_ON = false;												// false for inverted heater (e.g. Duet v0.6), true for not (e.g. Duet v0.4)

const Pin TEMP_SENSE_PINS[HEATERS] = { 5, 4, 0, 7, 8, 9, 11 };			// Analogue pin numbers
const Pin HEAT_ON_PINS[HEATERS] = { 6, X5, X7, 7, 8, 9, -1 };			// Heater Channel 7 (pin X17) is shared with Fan1. Only define 1 or the other

// Default thermistor parameters
// Bed thermistor: http://uk.farnell.com/epcos/b57863s103f040/sensor-miniature-ntc-10k/dp/1299930?Ntt=129-9930
// Hot end thermistor: http://www.digikey.co.uk/product-search/en?x=20&y=11&KeyWords=480-3137-ND
const float BED_R25 = 10000.0;
const float BED_BETA = 3988.0;
const float EXT_R25 = 100000.0;
const float EXT_BETA = 4138.0;

// Thermistor series resistor value in Ohms
const float THERMISTOR_SERIES_RS = 1000.0;

// Number of MAX31855 chips to support
const size_t MAX31855_START_CHANNEL = 100;

#if SUPPORT_ROLAND

// chrishamm's pin assignments
const size_t MAX31855_DEVICES = 2;

// Digital pins the 31855s have their select lines tied to
const Pin MAX31855_CS_PINS[MAX31855_DEVICES] = { 77, 87 };

#else

const size_t MAX31855_DEVICES = 4;

// Digital pins the 31855s have their select lines tied to
const Pin MAX31855_CS_PINS[MAX31855_DEVICES] = { 77, 87, 16, 17 };

#endif

// Arduino Due pin number that controls the ATX power on/off
const Pin ATX_POWER_PIN = 12;											// Arduino Due pin number that controls the ATX power on/off

// Analogue pin numbers
const Pin Z_PROBE_PIN = 10;												// Analogue pin number

// Digital pin number to turn the IR LED on (high) or off (low)
const Pin Z_PROBE_MOD_PIN = 52;											// Digital pin number to turn the IR LED on (high) or off (low) on Duet v0.6 and v1.0 (PB21)
const Pin Z_PROBE_MOD_PIN07 = X12;										// Digital pin number to turn the IR LED on (high) or off (low) on Duet v0.7 and v0.8.5 (PC10)

// COOLING FANS

const size_t NUM_FANS = 2;
const Pin COOLING_FAN_PINS[NUM_FANS] = { X6, X17 };						// Pin D34 is PWM capable but not an Arduino PWM pin - use X6 instead
const Pin COOLING_FAN_RPM_PIN = 23;										// Pin PA15

#if SUPPORT_INKJET
// Inkjet control pins
const Pin INKJET_SERIAL_OUT = 21;										// Serial bitpattern into the shift register
const Pin INKJET_SHIFT_CLOCK = 20;										// Shift the register
const Pin INKJET_STORAGE_CLOCK = 67;									// Put the pattern in the output register
const Pin INKJET_OUTPUT_ENABLE = 66;									// Make the output visible
const Pin INKJET_CLEAR = 36;											// Clear the register to 0

#endif

#if SUPPORT_ROLAND
// Roland mill
const Pin ROLAND_CTS_PIN = 16;											// Expansion pin 11, PA12_TXD1
const Pin ROLAND_RTS_PIN = 17;											// Expansion pin 12, PA13_RXD1

#endif

// Definition of which pins we allow to be controlled using M42
//
// The allowed pins are these ones on the DueX4 expansion connector:
//
// TXD1 aka PA13 aka pin 16
// RXD1 aka PA12 aka pin 17
// TXD0 aka PA11 aka pin 18
// RXD0 aka PA10 aka pin 19
// PC4_PWML1 aka PC4 aka pin 36
// AD13 aka PB20 aka pin 66
// AD14 aka PB21 aka pin 52
// PB16 aka pin 67 (could possibly allow analog output on this one)
// RTS1 aka PA14 aka pin 23
// TWD1 aka PB12 aka pin 20
// TWCK1 aka PB13 aka pin 21

const size_t NUM_PINS_ALLOWED = 72;

#define PINS_ALLOWED {				\
	/* pins 00-07 */	0,			\
	/* pins 08-15 */	0,			\
	/* pins 16-23 */	0b10111111,	\
	/* pins 24-31 */	0,			\
	/* pins 32-39 */	0b00010000,	\
	/* pins 40-47 */	0,			\
	/* pins 48-55 */	0b00010000,	\
	/* pins 56-63 */	0,			\
	/* pins 64-71 */	0b00001100	\
}

#endif
