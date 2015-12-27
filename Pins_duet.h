#ifndef PINS_DUET_H__
#define PINS_DUET_H__

// The physical capabilities of the machine

const size_t DRIVES = 9;						// The number of drives in the machine, including X, Y, and Z plus extruder drives
#define DRIVES_(a,b,c,d,e,f,g,h,i) { a,b,c,d,e,f,g,h,i }

const int8_t HEATERS = 7;						// The number of heaters in the machine; 0 is the heated bed even if there isn't one
#define HEATERS_(a,b,c,d,e,f,g) { a,b,c,d,e,f,g }

const size_t AXES = 3;							// The number of movement axes in the machine, usually just X, Y and Z. <= DRIVES
const size_t NUM_SERIAL_CHANNELS = 3;			// The number of serial IO channels (USB and two auxiliary UARTs)

// The numbers of entries in each array must correspond with the values of DRIVES, AXES, or HEATERS. Set values to -1 to flag unavailability.

// DRIVES

const Pin ENABLE_PINS[DRIVES] = { 29, 27, X1, X0, 37, X8, 50, 47, X13 };
const bool ENABLE_VALUES[DRIVES] = { false, false, false, false, false, false, false, false, false };	// what to send to enable a drive
const Pin STEP_PINS[DRIVES] = { 14, 25, 5, X2, 41, 39, X4, 49, X10 };
const Pin DIRECTION_PINS[DRIVES] = { 15, 26, 4, X3, 35, 53, 51, 48, X11 };
const bool DIRECTIONS[DRIVES] = { BACKWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS, FORWARDS };	// What each axis needs to make it go forwards - defaults

// Endstops
// RepRapFirmware only as a single endstop per axis gcode defines if it is a max ("high end") or min ("low end") endstop. gcode also sets if it is active HIGH or LOW.
const int8_t END_STOP_PINS[DRIVES] = { 11, 28, 60, 31, 24, 46, 45, 44, X9 };

// Indices for motor current digipots (if any): first 4 are for digipot 1,(on duet), second 4 for digipot 2 (on expansion board)
const uint8_t POT_WIPES[DRIVES] = { 1, 3, 2, 0, 1, 3, 2, 0 };			// Only define as many entries as DRIVES are defined
const float SENSE_RESISTOR = 0.1;										// Stepper motor current sense resistor
const float MAX_STEPPER_DIGIPOT_VOLTAGE = (3.3 * 2.5 / (2.7 + 2.5));	// Stepper motor current reference voltage
const float MAX_STEPPER_DAC_VOLTAGE = 2.12;								// Stepper motor current reference voltage for E1 if using a DAC

// HEATERS

const bool HEAT_ON = false;											// false for inverted heater (e.g. Duet v0.6), true for not (e.g. Duet v0.4)

const Pin TEMP_SENSE_PINS[HEATERS] = { 5, 4, 0, 7, 8, 9, 11 };		// Analogue pin numbers
const Pin HEAT_ON_PINS[HEATERS] = { 6, X5, X7, 7, 8, 9, -1 };		// Heater Channel 7 (pin X17) is shared with Fan1. Only define 1 or the other

const Pin MAX31855_SS_PIN = 10;

// Arduino Due pin number that controls the ATX power on/off
const Pin atxPowerPin = 12;											// Arduino Due pin number that controls the ATX power on/off

// Analogue pin numbers
const Pin Z_PROBE_PIN = 10;											// Analogue pin number

// Digital pin number to turn the IR LED on (high) or off (low)
const Pin Z_PROBE_MOD_PIN = 52;										// Digital pin number to turn the IR LED on (high) or off (low) on Duet v0.6 and v1.0 (PB21)
const Pin Z_PROBE_MOD_PIN07 = X12;									// Digital pin number to turn the IR LED on (high) or off (low) on Duet v0.7 and v0.8.5 (PC10)

// COOLING FANS

const size_t NUM_FANS = 2;
const Pin COOLING_FAN_PINS[NUM_FANS] = { X6, X17 };					// Pin D34 is PWM capable but not an Arduino PWM pin - use X6 instead
const Pin COOLING_FAN_RPM_PIN = 23;									// Pin PA15

#endif
