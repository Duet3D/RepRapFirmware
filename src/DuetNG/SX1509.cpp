/******************************************************************************
SparkFunSX1509.cpp
SparkFun SX1509 I/O Expander Library Source File
Adapted from Sparkfun SX1509 library by dc42, see header below.
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

#include "RepRapFirmware.h"
#include "Tasks.h"
#include "SX1509.h"
#include "SX1509Registers.h"
#include "Hardware/I2C.h"

SX1509::SX1509() : _clkX(0)
{
}

// Public functions
// Any operation requiring multiple I2C transactions must acquire and release the I2C mutex around them.

// Test for the presence of a SX1509B. The I2C subsystem must be initialised before calling this.
bool SX1509::begin(uint8_t address)
{
	// Store the received parameters into member variables
	deviceAddress =  address;
	
	reset();
	delay(2);														// not sure this is needed, but it may help

	pwmPins = 0;

	// Communication test. We'll read from two registers with different default values to verify communication.
	MutexLocker lock(Tasks::GetI2CMutex());

	const uint16_t testRegisters = readWord(REG_INTERRUPT_MASK_A);	// this should return 0xFF00
	const bool ok = (testRegisters == 0xFF00);
	if (ok)
	{
		clock(DefaultOscDivider);
		writeWord(REG_HIGH_INPUT_B, 0xFFFF);						// set all inputs to be 5V-tolerant
	}

	return ok;
}

// Reset the SX1509B
void SX1509::reset()
{
	MutexLocker lock(Tasks::GetI2CMutex());

	// Software reset command sequence:
	writeByte(REG_RESET, 0x12);
	writeByte(REG_RESET, 0x34);
}

void SX1509::pinMode(uint8_t pin, PinMode inOut)
{
	pinModeMultiple(1u << pin, inOut);
}

// Set the pin mode for multiple pins.
// Once we have enabled LED driver mode, disabling it doesn't seem to work.
// So we track which pins are in PWM (i.e. LED driver) mode, and we never try to switch them back to being ordinary outputs.
void SX1509::pinModeMultiple(uint16_t pins, PinMode inOut)
{
	MutexLocker lock(Tasks::GetI2CMutex());

	switch (inOut)
	{
	case INPUT:
		clearBitsInWord(REG_INPUT_DISABLE_B, pins);
		setBitsInWord(REG_DIR_B, pins);
		clearBitsInWord(REG_PULL_UP_B, pins);
		clearBitsInWord(REG_PULL_DOWN_B, pins);
		break;

	case INPUT_PULLUP:
		clearBitsInWord(REG_INPUT_DISABLE_B, pins);
		setBitsInWord(REG_DIR_B, pins);
		clearBitsInWord(REG_PULL_DOWN_B, pins);
		setBitsInWord(REG_PULL_UP_B, pins);
		break;

	case INPUT_PULLDOWN:
		clearBitsInWord(REG_INPUT_DISABLE_B, pins);
		setBitsInWord(REG_DIR_B, pins);
		clearBitsInWord(REG_PULL_UP_B, pins);
		setBitsInWord(REG_PULL_DOWN_B, pins);
		break;

	case OUTPUT_LOW:
		clearBitsInWord(REG_PULL_UP_B, pins);
		clearBitsInWord(REG_PULL_DOWN_B, pins);
		clearBitsInWord(REG_DATA_B, pins & ~pwmPins);
		clearBitsInWord(REG_OPEN_DRAIN_B, pins);
		clearBitsInWord(REG_DIR_B, pins);
		analogWriteMultiple(pins & pwmPins, 0);
		break;

	case OUTPUT_HIGH:
		clearBitsInWord(REG_PULL_UP_B, pins);
		clearBitsInWord(REG_PULL_DOWN_B, pins);
		setBitsInWord(REG_DATA_B, pins & ~pwmPins);
		clearBitsInWord(REG_OPEN_DRAIN_B, pins);
		clearBitsInWord(REG_DIR_B, pins);
		analogWriteMultiple(pins & pwmPins, 255);
		break;

	case OUTPUT_LOW_OPEN_DRAIN:
		clearBitsInWord(REG_PULL_UP_B, pins);
		clearBitsInWord(REG_PULL_DOWN_B, pins);
		clearBitsInWord(REG_DATA_B, pins & ~pwmPins);
		setBitsInWord(REG_OPEN_DRAIN_B, pins);
		clearBitsInWord(REG_DIR_B, pins);
		analogWriteMultiple(pins & pwmPins, 0);
		break;

	case OUTPUT_HIGH_OPEN_DRAIN:
		clearBitsInWord(REG_PULL_UP_B, pins);
		clearBitsInWord(REG_PULL_DOWN_B, pins);
		setBitsInWord(REG_DATA_B, pins & ~pwmPins);
		setBitsInWord(REG_OPEN_DRAIN_B, pins);
		clearBitsInWord(REG_DIR_B, pins);
		analogWriteMultiple(pins & pwmPins, 255);
		break;

	case OUTPUT_PWM_LOW:
	case OUTPUT_PWM_HIGH:
		ledDriverInitMultiple(pins, false, false);
		break;

	case OUTPUT_PWM_OPEN_DRAIN:
		ledDriverInitMultiple(pins, false, true);
		break;

	default:
		break;
	}
}

void SX1509::digitalWrite(uint8_t pin, bool highLow)
{
	if (((1u << pin) & pwmPins) != 0)
	{
		analogWrite(pin, (highLow) ? 255 : 0);
	}
	else if (highLow)
	{
		MutexLocker lock(Tasks::GetI2CMutex());
		setBitsInWord(REG_DATA_B, 1u << pin);
	}
	else
	{
		MutexLocker lock(Tasks::GetI2CMutex());
		clearBitsInWord(REG_DATA_B, 1u << pin);
	}
}

bool SX1509::digitalRead(uint8_t pin)
{
	if (pin >= 8)
	{
		return (readByte(REG_DATA_B) & (1u << (pin - 8))) != 0;
	}
	else
	{
		return (readByte(REG_DATA_A) & (1u << pin)) != 0;
	}
}

uint16_t SX1509::digitalReadAll()
{
	return readWord(REG_DATA_B);
}

#if 0	// unused

void SX1509::ledDriverInit(uint8_t pin, bool log, bool openDrain)
{
	ledDriverInitMultiple(1u << pin, log, openDrain);
}

#endif

void SX1509::ledDriverInitMultiple(uint16_t pins, bool log, bool openDrain)
{
	if (openDrain)
	{
		setBitsInWord(REG_OPEN_DRAIN_B, pins);
	}
	else
	{
		clearBitsInWord(REG_OPEN_DRAIN_B, pins);
	}
	setBitsInWord(REG_INPUT_DISABLE_B, pins);	// disable input buffer
	clearBitsInWord(REG_PULL_UP_B, pins);		// disable pullup
	clearBitsInWord(REG_PULL_DOWN_B, pins);		// disable pulldown
	clearBitsInWord(REG_DIR_B, pins);			// set as an output

	// Configure LED driver clock and mode (REG_MISC)
	uint8_t tempByte = readByte(REG_MISC);
	if (log)
	{
		tempByte |= (1u << 7) | (1u << 3);		// set logarithmic mode bank B and A
	}
	else
	{
		tempByte &= ~((1u << 7) | (1u << 3));	// set linear mode bank B and A
	}
	writeByte(REG_MISC, tempByte);
	
	// Enable LED driver operation (REG_LED_DRIVER_ENABLE)
	setBitsInWord(REG_LED_DRIVER_ENABLE_B, pins);
	
	// Set REG_DATA bit low ~ LED driver started
	clearBitsInWord(REG_DATA_B, pins);

	pwmPins |= pins;							// record which pins are in LED driver mode
}

void SX1509::analogWrite(uint8_t pin, uint8_t iOn)
{
	// Write the on intensity of pin
	// Linear mode: Ion = iOn
	// Log mode: Ion = f(iOn)
	// We use the pins as active-high outputs to drive the fan mosfets, not to sink LED current directly.
	// This means that we need to invert the intensity, and log mode doesn't make sense.
	writeByte(REG_I_ON[pin], ~iOn);
}

void SX1509::enableInterrupt(uint8_t pin, uint8_t riseFall)
{
	enableInterruptMultiple(1u << pin, riseFall);
}

void SX1509::enableInterruptMultiple(uint16_t pins, uint8_t riseFall)
{
	// Set REG_SENSE_XXX
	// Sensitivity is set as follows:
	// 00: None
	// 01: Rising
	// 10: Falling
	// 11: Both
	uint8_t sensitivity;
	switch (riseFall)
	{
	case INTERRUPT_MODE_CHANGE:
		sensitivity = 0b11;
		break;
	case INTERRUPT_MODE_FALLING:
		sensitivity = 0b10;
		break;
	case INTERRUPT_MODE_RISING:
		sensitivity = 0b01;
		break;
	default:
		sensitivity = 0;
	}

	MutexLocker lock(Tasks::GetI2CMutex());

	uint32_t pinMask = readDword(REG_SENSE_HIGH_B);
	for (unsigned int i = 0; i < 16; ++i)
	{
		if ((pins & (1 << i)) != 0)
		{
			pinMask &= ~(0x03 << (2 * i));
			pinMask |= (sensitivity << (2 * i));
		}
	}

	writeDword(REG_SENSE_HIGH_B, pinMask);
	clearBitsInWord(REG_INTERRUPT_MASK_B, pins);
}

uint16_t SX1509::interruptSource()
{
	return readWord(REG_INTERRUPT_SOURCE_B);
}

uint16_t SX1509::interruptSourceAndClear()
{
	MutexLocker lock(Tasks::GetI2CMutex());

	const uint16_t intSource = interruptSource();
	writeWord(REG_INTERRUPT_SOURCE_B, intSource);	// Clear interrupts
	return intSource;
}

bool SX1509::checkInterrupt(uint8_t pin)
{
	return (interruptSource() & (1u << pin)) != 0;
}

//********* Private functions. The I2C mutex must be owned by the caller. ***********

void SX1509::setBitsInWord(uint8_t registerAddress, uint16_t bits)
{
	if (bits != 0)
	{
		const uint16_t regVal = readWord(registerAddress);
		writeWord(registerAddress, regVal | bits);
	}
}

void SX1509::clearBitsInWord(uint8_t registerAddress, uint16_t bits)
{
	if (bits != 0)
	{
		const uint16_t regVal = readWord(registerAddress);
		writeWord(registerAddress, regVal & (~bits));
	}
}

void SX1509::analogWriteMultiple(uint16_t pins, uint8_t pwm)
{
	for (uint8_t pin = 0; pins != 0; ++pin)
	{
		if ((pins & 1u) != 0)
		{
			analogWrite(pin, pwm);
		}
		pins >>= 1;
	}
}

void SX1509::clock(uint8_t oscDivider)
{
	// RegClock constructed as follows:
	//	6:5 - Oscillator frequency source
	//		00: off, 01: external input, 10: internal 2MHz, 1: reserved
	//	4 - OSCIO pin function
	//		0: input, 1 output
	//	3:0 - Frequency of oscout pin
	//		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
	writeByte(REG_CLOCK, (2u << 5) | (1u << 4) | (1u << 0));	// internal 2MHz oscillator, OSCIO outputs at that frequency
	
	// Config RegMisc[6:4] with oscDivider
	// 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] - 1))
	oscDivider = constrain<uint8_t>(oscDivider, 1, 7);
	_clkX = 2000000.0 / (1u << (oscDivider - 1u));			// update private clock variable
	uint8_t regMisc = readByte(REG_MISC);
	regMisc &= ~((0b111 << 4) | (1u << 1) | (1u << 0));		// clear clock divider bits, auto-increment is enabled, clear interrupt on register read
	regMisc |= oscDivider << 4;
	writeByte(REG_MISC, regMisc);
}

uint8_t SX1509::calculateLEDTRegister(int ms)
{
	if (_clkX == 0)
	{
		return 0;
	}
	
	int regOn1 = (int)(((float)ms / 1000.0) / (64.0 * 255.0 / (float) _clkX));
	int regOn2 = regOn1 / 8;
	regOn1 = constrain<int>(regOn1, 1, 15);
	regOn2 = constrain<int>(regOn2, 16, 31);
	
	const float timeOn1 = 64.0 * regOn1 * 255.0 / _clkX * 1000.0;
	const float timeOn2 = 512.0 * regOn2 * 255.0 / _clkX * 1000.0;

	return (abs(timeOn1 - ms) < abs(timeOn2 - ms)) ? regOn1 : regOn2;
}

uint8_t SX1509::calculateSlopeRegister(int ms, uint8_t onIntensity, uint8_t offIntensity)
{
	if (_clkX == 0)
	{
		return 0;
	}
	
	const float tFactor = ((float) onIntensity - (4.0 * (float)offIntensity)) * 255.0 / (float) _clkX;
	const float timeS = float(ms) / 1000.0;
	
	int regSlope1 = timeS / tFactor;
	int regSlope2 = regSlope1 / 16;
	
	regSlope1 = constrain<int>(regSlope1, 1, 15);
	regSlope2 = constrain<int>(regSlope2, 16, 31);

	const float regTime1 = regSlope1 * tFactor * 1000.0;
	const float regTime2 = 16 * regTime1;

	return (abs(regTime1 - ms) < abs(regTime2 - ms)) ? regSlope1 : regSlope2;
}

// readByte(uint8_t registerAddress)
//	This function reads a single uint8_t located at the registerAddress register.
//	- deviceAddress should already be set by the constructor.
//	- Return value is the uint8_t read from registerAddress
//		- Currently returns 0 if communication has timed out
uint8_t SX1509::readByte(uint8_t registerAddress)
{
	uint8_t data[2];
	data[0] = registerAddress;
	if (I2C::Transfer(deviceAddress, data, 1, 1) == 2)
	{
		return data[1];
	}
	return 0;
}

// readWord(uint8_t registerAddress)
//	This function will read a two-uint8_t word beginning at registerAddress
//	- A 16-bit unsigned int will be returned.
//		- The msb of the return value will contain the value read from registerAddress
//		- The lsb of the return value will contain the value read from registerAddress + 1
uint16_t SX1509::readWord(uint8_t registerAddress)
{
	uint8_t data[3];
	data[0] = registerAddress;
	if (I2C::Transfer(deviceAddress, data, 1, 2) == 3)
	{
		return (data[1] << 8) | data[2];
	}
	return 0;
}

// readDword(uint8_t registerAddress)
//	This function will read a four-uint8_t word beginning at registerAddress
//	- A 32-bit unsigned int will be returned.
//		- The msb of the return value will contain the value read from registerAddress
//		- The lsb of the return value will contain the value read from registerAddress + 1
uint32_t SX1509::readDword(uint8_t registerAddress)
{
	uint8_t data[5];
	data[0] = registerAddress;
	if (I2C::Transfer(deviceAddress, data, 1, 4) == 5)
	{
		return (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
	}
	return 0;
}

// writeByte(uint8_t registerAddress, uint8_t writeValue)
//	This function writes a single uint8_t to a single register on the SX509.
//	- writeValue is written to registerAddress
//	- deviceAddress should already be set from the constructor
//	- No return value.
void SX1509::writeByte(uint8_t registerAddress, uint8_t writeValue)
{
	uint8_t data[2] = { registerAddress, writeValue };
	(void)I2C::Transfer(deviceAddress, data, 2, 0);
}

// writeWord(uint8_t registerAddress, uint16_t writeValue)
//	This function writes a two-uint8_t word to registerAddress and registerAddress + 1
//	- the upper uint8_t of writeValue is written to registerAddress
//		- the lower uint8_t of writeValue is written to registerAddress + 1
//	- No return value.
void SX1509::writeWord(uint8_t registerAddress, uint16_t writeValue)
{
	uint8_t data[3] = { registerAddress, (uint8_t)(writeValue >> 8), (uint8_t)writeValue };
	(void)I2C::Transfer(deviceAddress, data, 3, 0);
}

// writeDword(uint8_t registerAddress, uint32_t writeValue)
//	This function writes a four-uint8_t word to registerAddress .. registerAddress + 3, msb first
//	- No return value.
void SX1509::writeDword(uint8_t registerAddress, uint32_t writeValue)
{
	uint8_t data[5] = { registerAddress, (uint8_t)(writeValue >> 24), (uint8_t)(writeValue >> 16), (uint8_t)(writeValue >> 8), (uint8_t)writeValue };
	(void)I2C::Transfer(deviceAddress, data, 5, 0);
}

// End
