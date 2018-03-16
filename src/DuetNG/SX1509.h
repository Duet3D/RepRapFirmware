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
	uint32_t errorCount;

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

public:
	// -----------------------------------------------------------------------------
	// Constructor - SX1509: This function sets up the pins connected to the
	//		SX1509, and sets up the private deviceAddress variable.
	// -----------------------------------------------------------------------------
	SX1509();

	uint32_t GetErrorCount() const { return errorCount; }

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

#if 0	// these functions are not used

	// -----------------------------------------------------------------------------
	// setupBlink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t offIntensity, uint8_t tRise, uint8_t
	//		tFall):  blink performs both the blink and breath LED driver functions.
	//
	// 	Inputs:
	//  	- pin: the SX1509 pin (0-15) you want to set blinking/breathing.
	//		- tOn: the amount of time the pin is HIGH
	//			- This value should be between 1 and 31. 0 is off.
	//		- tOff: the amount of time the pin is at offIntensity
	//			- This value should be between 1 and 31. 0 is off.
	//		- offIntensity: How dim the LED is during the off period.
	//			- This value should be between 0 and 7. 0 is completely off.
	//		- onIntensity: How bright the LED will be when completely on.
	//			- This value can be between 0 (0%) and 255 (100%).
	//		- tRise: This sets the time the LED takes to fade in.
	//			- This value should be between 1 and 31. 0 is off.
	//			- This value is used with tFall to make the LED breath.
	//		- tFall: This sets the time the LED takes to fade out.
	//			- This value should be between 1 and 31. 0 is off.
	// 	 Notes:
	//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and
	//			tFall are set on 0-3 or 8-11 those pins will still only blink.
	// 		- ledDriverInit should be called on the pin to be blinked before this.
	// -----------------------------------------------------------------------------
	void setupBlink(uint8_t pin, uint8_t tOn, uint8_t toff, uint8_t onIntensity = 255, uint8_t offIntensity = 0,
					uint8_t tRise = 0, uint8_t tFall = 0, bool log = false, bool openDrain = false);

	// -----------------------------------------------------------------------------
	// blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity);
	//  	Set a pin to blink output for estimated on/off millisecond durations.
	//
	// 	Inputs:
	//  	- pin: the SX1509 pin (0-15) you want to set blinking
	//   	- tOn: estimated number of milliseconds the pin is LOW (LED sinking current will be on)
	//   	- tOff: estimated number of milliseconds the pin is HIGH (LED sinking current will be off)
	//   	- onIntensity: 0-255 value determining LED on brightness
	//   	- offIntensity: 0-255 value determining LED off brightness
	// 	 Notes:
	//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and
	//			tFall are set on 0-3 or 8-11 those pins will still only blink.
	// 		- ledDriverInit should be called on the pin to be blinked before this.
	// -----------------------------------------------------------------------------
	void blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity = 255, uint8_t offIntensity = 0);
	
	// -----------------------------------------------------------------------------
	// breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt, bool log);
	//  	Set a pin to breathe output for estimated on/off millisecond durations, with
	//  	estimated rise and fall durations.
	//
	// 	Inputs:
	//  	- pin: the SX1509 pin (0-15) you want to set blinking
	//   	- tOn: estimated number of milliseconds the pin is LOW (LED sinking current will be on)
	//   	- tOff: estimated number of milliseconds the pin is HIGH (LED sinking current will be off)
	//   	- rise: estimated number of milliseconds the pin rises from LOW to HIGH
	//   	- falll: estimated number of milliseconds the pin falls from HIGH to LOW
	//   	- onIntensity: 0-255 value determining LED on brightness
	//   	- offIntensity: 0-255 value determining LED off brightness
	// 	 Notes:
	//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and
	//			tFall are set on 0-3 or 8-11 those pins will still only blink.
	// 		- ledDriverInit should be called on the pin to be blinked before this,
	//  	  Or call pinMode(<pin>, ANALOG_OUTPUT);
	// -----------------------------------------------------------------------------
	void breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall,
			uint8_t onInt = 255, uint8_t offInt = 0, bool log = false, bool openDrain = false);

	// -----------------------------------------------------------------------------
	// keypad(uint8_t rows, uint8_t columns, uint8_t sleepTime, uint8_t scanTime, uint8_t debounceTime)
	//		Initializes the keypad function on the SX1509. Millisecond durations for sleep,
	//		scan, and debounce can be set.
	//
	//	Inputs:
	//		- rows: The number of rows in the button matrix.
	//			- This value must be between 1 and 7. 0 will turn it off.
	//			- eg: 1 = 2 rows, 2 = 3 rows, 7 = 8 rows, etc.
	//		- columns: The number of columns in the button matrix
	//			- This value should be between 0 and 7.
	//			- 0 = 1 column, 7 = 8 columns, etc.
	//		- sleepTime: Sets the auto-sleep time of the keypad engine.
	//  	  Should be a millisecond duration between 0 (OFF) and 8000 (8 seconds).
	//   	  Possible values are 0, 128, 256, 512, 1000, 2000, 4000, 8000
	//		- scanTime: Sets the scan time per row. Must be set above debounce.
	//  	  Should be a millisecond duration between 1 and 128.
	//   	  Possible values are 1, 2, 4, 8, 16, 32, 64, 128.
	//		- debounceTime: Sets the debounc time per button. Must be set below scan.
	//  	  Should be a millisecond duration between 0 and 64.
	//   	  Possible values are 0 (0.5), 1, 2, 4, 8, 16, 32, 64.
	// -----------------------------------------------------------------------------
	void keypad(uint8_t rows, uint8_t columns, unsigned int sleepTime = 0, uint8_t scanTime = 1, uint8_t debounceTime = 0);

	// -----------------------------------------------------------------------------
	// readKeypad(): This function returns a 16-bit value containing the status of
	//		keypad engine.
	//
	//	Output:
	//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
	//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
	//		button in that row or column is being pressed. As such, at least two
	//		bits should be set.
	// -----------------------------------------------------------------------------
	uint16_t readKeypad();
	
	// -----------------------------------------------------------------------------
	// getRow(): This function returns the first active row from the return value of
	//  	readKeypad().
	//
	//	Input:
	//      - keyData: Should be the unsigned int value returned from readKeypad().
	//	Output:
	//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
	//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
	//		button in that row or column is being pressed. As such, at least two
	//		bits should be set.
	// -----------------------------------------------------------------------------
	uint8_t getRow(uint16_t keyData);
	
	// -----------------------------------------------------------------------------
	// getCol(): This function returns the first active column from the return value of
	//  	readKeypad().
	//
	//	Input:
	//      - keyData: Should be the unsigned int value returned from readKeypad().
	//	Output:
	//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
	//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
	//		button in that row or column is being pressed. As such, at least two
	//		bits should be set.
	// -----------------------------------------------------------------------------
	uint8_t getCol(uint16_t keyData);
	
	// -----------------------------------------------------------------------------
	// debounceConfig(uint8_t configValue): This method configures the debounce time of
	//		every input.
	//
	//	Input:
	//		- configValue: A 3-bit value configuring the debounce time.
	//			000: 0.5ms * 2MHz/fOSC
	//			001: 1ms * 2MHz/fOSC
	//			010: 2ms * 2MHz/fOSC
	//			011: 4ms * 2MHz/fOSC
	//			100: 8ms * 2MHz/fOSC
	//			101: 16ms * 2MHz/fOSC
	//			110: 32ms * 2MHz/fOSC
	//			111: 64ms * 2MHz/fOSC
	// -----------------------------------------------------------------------------
	void debounceConfig(uint8_t configVaule);
	
	// -----------------------------------------------------------------------------
	// debounceTime(uint8_t configValue): This method configures the debounce time of
	//		every input to an estimated millisecond time duration.
	//
	//	Input:
	//		- time: A millisecond duration estimating the debounce time. Actual
	//		  debounce time will depend on fOSC. Assuming it's 2MHz, debounce will
	//		  be set to the 0.5, 1, 2, 4, 8, 16, 32, or 64 ms (whatever's closest)
	// -----------------------------------------------------------------------------
	void debounceTime(uint8_t time);

	// -----------------------------------------------------------------------------
	// debouncePin(uint8_t pin): This method enables debounce on SX1509 input pin.
	//
	//	Input:
	//		- pin: The SX1509 pin to be debounced. Should be between 0 and 15.
	// -----------------------------------------------------------------------------
	void debouncePin(uint8_t pin);

	// -----------------------------------------------------------------------------
	// debounceKeypad(uint8_t pin): This method enables debounce on all pins connected
	//  to a row/column keypad matrix.
	//
	//	Input:
	//		- time: Millisecond time estimate for debounce (see debounceTime()).
	//		- numRows: The number of rows in the keypad matrix.
	//		- numCols: The number of columns in the keypad matrix.
	// -----------------------------------------------------------------------------
	void debounceKeypad(uint8_t time, uint8_t numRows, uint8_t numCols);

#endif

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
	//  Input:
	//  	- clear: boolean commanding whether the interrupt should be cleared
	//  	  after reading or not.
	// -----------------------------------------------------------------------------
	uint16_t interruptSource(bool clear);

	// -----------------------------------------------------------------------------
	// checkInterrupt(void): Checks if a single pin generated an interrupt.
	//
	//	Output: Boolean value. True if the requested pin has triggered an interrupt/
	//  Input:
	//  	- pin: Pin to be checked for generating an input.
	// -----------------------------------------------------------------------------
	bool checkInterrupt(uint8_t pin);

	// -----------------------------------------------------------------------------
	// clock(uint8_t oscDivider)
	//		This function configures the speed of the cloak, which is used to drive LEDs and time debounces.
	//
	//	Inputs:
	//	- oscDivider: Sets the clock divider in REG_MISC. Clock is 2MHz / (1 << (oscDivider - 1). PWM frequency is 1/256 of that.
	// -----------------------------------------------------------------------------
	void clock(uint8_t oscDivider);
};

#endif	// SX1509_H
