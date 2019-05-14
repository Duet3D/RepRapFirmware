/******************************************************************************
SX1509.h
SX1509 I/O Expander Library Header File
Adapted from Sparkfun SX1509 library (see header below) by dc42
******************************************************************************/

/******************************************************************************
Jim Lindblom @ SparkFun Electronics
Original Creation Date: September 21, 2015
https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library

Here you'll find the Arduino code used to interface with the SX1509 I2C
16 I/O expander. There are functions to take advantage of everything the
SX1509 provides - input/output setting, writing pins high/low, reading 
the input value of pins, LED driver utilities (blink, breath, pwm), and
keypad engine utilites.

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef SX1509_H
#define SX1509_H

#include "Core.h"

const int ReceiveTimeout = 1000;			// Timeout for I2C receive
const uint8_t DefaultOscDivider = 5;		// a clock divider of 2 ^ (5 - 1) = 16 gives a PWM frequency of 2MHz / (16 * 255) = 488Hz

class SX1509
{
private:
	uint8_t deviceAddress;					// I2C Address of SX1509

	// Misc variables:
	uint32_t _clkX;							// clock speed
	uint16_t pwmPins;						// bitmap of pins configured as PWM output pins

	// Read Functions:
	uint8_t readByte(uint8_t registerAddress);
	uint16_t readWord(uint8_t registerAddress);
	uint32_t readDword(uint8_t registerAddress);

	// Write functions:
	void writeByte(uint8_t registerAddress, uint8_t writeValue);
	void writeWord(uint8_t registerAddress, uint16_t writeValue);
	void writeDword(uint8_t registerAddress, uint32_t writeValue);

	void setBitsInWord(uint8_t registerAddress, uint16_t bits);
	void clearBitsInWord(uint8_t registerAddress, uint16_t bits);
	void analogWriteMultiple(uint16_t pins, uint8_t pwm);

	// Helper functions:

	// calculateLEDTRegister - Try to estimate an LED on/off duration register, 
	// given the number of milliseconds and LED clock frequency.
	uint8_t calculateLEDTRegister(int ms);

	// calculateSlopeRegister - Try to estimate an LED rise/fall duration 
	// register, given the number of milliseconds and LED clock frequency.
	uint8_t calculateSlopeRegister(int ms, uint8_t onIntensity, uint8_t offIntensity);


	// -----------------------------------------------------------------------------
	// clock(uint8_t oscDivider)
	//		This function configures the speed of the cloak, which is used to drive LEDs and time debounces.
	//
	//	Inputs:
	//	- oscDivider: Sets the clock divider in REG_MISC. Clock is 2MHz / (1 << (oscDivider - 1). PWM frequency is 1/256 of that.
	// -----------------------------------------------------------------------------
	void clock(uint8_t oscDivider);

public:
	// -----------------------------------------------------------------------------
	// Constructor - SX1509: This function sets up the pins connected to the
	//		SX1509, and sets up the private deviceAddress variable.
	// -----------------------------------------------------------------------------
	SX1509();

	// -----------------------------------------------------------------------------
	// begin(uint8_t address): This function initializes the SX1509.
	//  	It begins the Wire library, resets the IC, and tries to read some
	//  	registers to prove it's connected.
	// Inputs:
	//		- address: should be the 7-bit address of the SX1509. This should be
	//		 one of four values - 0x3E, 0x3F, 0x70, 0x71 - all depending on what the
	//		 ADDR0 and ADDR1 pins are set to. This variable is required.
	// Output: Returns true if communication is successful, false on error.
	// -----------------------------------------------------------------------------
	bool begin(uint8_t address);

	// -----------------------------------------------------------------------------
	// reset(): This function resets the SX1509. A software
	//		reset writes a 0x12 then 0x34 to the REG_RESET as outlined in the
	//		datasheet.
	// -----------------------------------------------------------------------------
	void reset();

	// -----------------------------------------------------------------------------
	// pinMode(uint8_t pin, PinMode inOut): This function sets one of the SX1509's 16
	//		outputs to either an INPUT or OUTPUT.
	//
	//	Inputs:
	//	 	- pin: should be a value between 0 and 15
	//	 	- inOut: The Core INPUT and OUTPUT constants should be used for the
	//		 inOut parameter. They do what they say!
	// -----------------------------------------------------------------------------
	void pinMode(uint8_t pin, PinMode inOut);

	// pinModeMultiple(uint16_t pins, PinMode inOut): This function sets several of the SX1509's 16
	//		outputs to either an INPUT or OUTPUT.
	//
	//	Inputs:
	//	 	- pin: should be a value between 0 and 15
	//	 	- inOut: The Core INPUT and OUTPUT constants should be used for the
	//		 inOut parameter. They do what they say!
	// -----------------------------------------------------------------------------
	void pinModeMultiple(uint16_t pins, PinMode inOut);

	// -----------------------------------------------------------------------------
	// digitalWrite(uint8_t pin, bool highLow): This function writes a pin to either high
	//		or low if it's configured as an OUTPUT. If the pin is configured as an
	//		INPUT, this method will activate either the PULL-UP	or PULL-DOWN
	//		resistor (HIGH or LOW respectively).
	//
	//	Inputs:
	//		- pin: The SX1509 pin number. Should be a value between 0 and 15.
	//		- highLow: true for HIGH, false for LOW.
	// -----------------------------------------------------------------------------
	void digitalWrite(uint8_t pin, bool highLow);

	// -----------------------------------------------------------------------------
	// digitalRead(uint8_t pin): This function reads the HIGH/LOW status of a pin.
	//		The pin should be configured as an INPUT, using the pinDir function.
	//
	//	Inputs:
	//	 	- pin: The SX1509 pin to be read. should be a value between 0 and 15.
	//  Outputs:
	//		This function returns true if HIGH, false if LOW
	// -----------------------------------------------------------------------------
	bool digitalRead(uint8_t pin);

	// -----------------------------------------------------------------------------
	// digitalReadAll(): This function reads all 16 pins.
	// -----------------------------------------------------------------------------
	uint16_t digitalReadAll();

#if 0	// unused
	// -----------------------------------------------------------------------------
	// ledDriverInit(uint8_t pin, bool log): This function initializes LED
	//		driving on a pin. It must be called if you want to use the pwm or blink
	//		functions on that pin.
	//
	//	Inputs:
	//		- pin: The SX1509 pin connected to an LED. Should be 0-15.
	//		- log: selects either linear or logarithmic mode on the LED drivers
	//			- log defaults to 0, linear mode
	//			- currently log sets both bank A and B to the same mode
	// -----------------------------------------------------------------------------
	void ledDriverInit(uint8_t pin, bool log, bool openDrain);
#endif

	// -----------------------------------------------------------------------------
	// ledDriverInitMultiple(uint16_t pins, bool log): This function initializes LED
	//		driving on a pin. It must be called if you want to use the pwm or blink
	//		functions on that pin.
	//
	//	Inputs:
	//		- pins: The SX1509 pins connected to an LED.
	//		- log: selects either linear or logarithmic mode on the LED drivers
	//			- log defaults to 0, linear mode
	//			- currently log sets both bank A and B to the same mode
	// -----------------------------------------------------------------------------
	void ledDriverInitMultiple(uint16_t pins, bool log, bool openDrain);

	// -----------------------------------------------------------------------------
	// analogWrite(uint8_t pin, uint8_t iOn):	This function can be used to control the intensity
	//		of an output pin connected to an LED.
	//
	//	Inputs:
	//		- pin: The SX1509 pin connecte to an LED.Should be 0-15.
	//		- iOn: should be a 0.0-1.0 value setting the intensity of the LED
	//			- 0.0 is completely off, 1.0 is 100% on.
	//
	//	Note: ledDriverInit should be called on the pin before calling this.
	// -----------------------------------------------------------------------------
	void analogWrite(uint8_t pin, uint8_t iOn);

	// -----------------------------------------------------------------------------
	// enableInterrupt(uint8_t pin, uint8_t riseFall): This function sets up an interrupt
	//		on a pin. Interrupts can occur on all SX1509 pins, and can be generated
	//		on rising, falling, or both.
	//
	//	Inputs:
	//		-pin: SX1509 input pin that will generate an interrupt. Should be 0-15.
	//		-riseFall: Configures if you want an interrupt generated on rise fall or
	//			both. For this param, send the pin-change values previously defined
	//			by Arduino:
	//			#define CHANGE 1	<- Both
	//			#define FALLING 2	<- Falling
	//			#define RISING 3	<- Rising
	//
	//	Note: This function does not set up a pin as an input, or configure	its
	//		pull-up/down resistors! Do that before (or after).
	// -----------------------------------------------------------------------------
	void enableInterrupt(uint8_t pin, uint8_t riseFall);

	// -----------------------------------------------------------------------------
	// enableInterruptMultiple(uint16_t pins, uint8_t riseFall): This function sets up an interrupt
	//		on a pin. Interrupts can occur on all SX1509 pins, and can be generated
	//		on rising, falling, or both.
	//
	//	Inputs:
	//		-pins: SX1509 input pins that will generate an interrupt.
	//		-riseFall: Configures if you want an interrupt generated on rise fall or
	//			both. For this param, send the pin-change values previously defined
	//			by Arduino:
	//			#define CHANGE 1	<- Both
	//			#define FALLING 2	<- Falling
	//			#define RISING 3	<- Rising
	//
	//	Note: This function does not set up a pin as an input, or configure	its
	//		pull-up/down resistors! Do that before (or after).
	// -----------------------------------------------------------------------------
	void enableInterruptMultiple(uint16_t pins, uint8_t riseFall);

	// -----------------------------------------------------------------------------
	// interruptSource(void): Returns an unsigned int representing which pin caused
	//		an interrupt.
	//
	//	Output: 16-bit value, with a single bit set representing the pin(s) that
	//		generated an interrupt. E.g. a return value of	0x0104 would mean pins 8
	//		and 3 (bits 8 and 3) have generated an interrupt.
	// -----------------------------------------------------------------------------
	uint16_t interruptSource();
	uint16_t interruptSourceAndClear();

	// -----------------------------------------------------------------------------
	// checkInterrupt(void): Checks if a single pin generated an interrupt.
	//
	//	Output: Boolean value. True if the requested pin has triggered an interrupt/
	//  Input:
	//  	- pin: Pin to be checked for generating an input.
	// -----------------------------------------------------------------------------
	bool checkInterrupt(uint8_t pin);
};

#endif	// SX1509_H
