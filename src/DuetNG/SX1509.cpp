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

#include "Core.h"
#include "Wire.h"
#include "SX1509.h"
#include "SX1509Registers.h"
#include "Pins.h"

SX1509::SX1509() : _clkX(0), errorCount(0)
{
}

// Test for the presence of a SX1509B. The I2C subsystem must be initialised before calling this.
bool SX1509::begin(uint8_t address)
{
	// Store the received parameters into member variables
	deviceAddress =  address;
	
	reset();

	pwmPins = 0;

	// Communication test. We'll read from two registers with different default values to verify communication.
	const uint16_t testRegisters = readWord(REG_INTERRUPT_MASK_A);	// this should return 0xFF00
	const bool ok = (testRegisters == 0xFF00);
	if (ok)
	{
		clock(DefaultOscDivider);
		writeWord(REG_HIGH_INPUT_B, 0xFFFF);						// set all inputs to be 5V-tolerant
	}

	return ok;
}

void SX1509::reset()
{
	// Software reset command sequence:
	writeByte(REG_RESET, 0x12);
	writeByte(REG_RESET, 0x34);
}

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

void SX1509::pinMode(uint8_t pin, PinMode inOut)
{
	pinModeMultiple(1u << pin, inOut);
}

// Set the pin mode for multiple pins.
// Once we have enabled LED driver mode, disabling it doesn't seem to work.
// So we track which pins are in PWM (i.e. LED driver) mode, and we never try to switch them back to being ordinary outputs.
void SX1509::pinModeMultiple(uint16_t pins, PinMode inOut)
{
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
		setBitsInWord(REG_DATA_B, 1u << pin);
	}
	else
	{
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

#if 0	// these functions are not used

void SX1509::blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity)
{
	const uint8_t onReg = calculateLEDTRegister(tOn);
	const uint8_t offReg = calculateLEDTRegister(tOff);
	
	setupBlink(pin, onReg, offReg, onIntensity, offIntensity, 0, 0, false, false);
}

void SX1509::breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt, bool log, bool openDrain)
{
	offInt = constrain<uint8_t>(offInt, 0, 7);
	
	const uint8_t onReg = calculateLEDTRegister(tOn);
	const uint8_t offReg = calculateLEDTRegister(tOff);
	
	const uint8_t riseTime = calculateSlopeRegister(rise, onInt, offInt);
	const uint8_t fallTime = calculateSlopeRegister(fall, onInt, offInt);
	
	setupBlink(pin, onReg, offReg, onInt, offInt, riseTime, fallTime, log, openDrain);
}

void SX1509::setupBlink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t onIntensity, uint8_t offIntensity, uint8_t tRise, uint8_t tFall, bool log, bool openDrain)
{
	ledDriverInit(pin, log, openDrain);
	
	// Keep parameters within their limits:
	tOn &= 0x1F;	// tOn should be a 5-bit value
	tOff &= 0x1F;	// tOff should be a 5-bit value
	offIntensity &= 0x07;
	// Write the time on
	// 1-15:  TON = 64 * tOn * (255/ClkX)
	// 16-31: TON = 512 * tOn * (255/ClkX)
	writeByte(REG_T_ON[pin], tOn);
	
	// Write the time/intensity off register
	// 1-15:  TOFF = 64 * tOff * (255/ClkX)
	// 16-31: TOFF = 512 * tOff * (255/ClkX)
	// linear Mode - IOff = 4 * offIntensity
	// log mode - Ioff = f(4 * offIntensity)
	writeByte(REG_OFF[pin], (tOff << 3) | offIntensity);
	
	// Write the on intensity:
	writeByte(REG_I_ON[pin], onIntensity);
	
	// Prepare tRise and tFall
	tRise &= 0x1F;	// tRise is a 5-bit value
	tFall &= 0x1F;	// tFall is a 5-bit value
	
	// Write regTRise
	// 0: Off
	// 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	// 16-31: TRise = 16 * (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	if (REG_T_RISE[pin] != 0xFF)
	{
		writeByte(REG_T_RISE[pin], tRise);
	}

	// Write regTFall
	// 0: off
	// 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	// 16-31: TFall = 16 * (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	if (REG_T_FALL[pin] != 0xFF)
	{
		writeByte(REG_T_FALL[pin], tFall);
	}
}

void SX1509::keypad(uint8_t rows, uint8_t columns, unsigned int sleepTime, uint8_t scanTime, uint8_t debounceTime)
{
	// Set regDir 0:7 outputs, 8:15 inputs:
	uint16_t tempWord = readWord(REG_DIR_B);
	for (uint8_t i = 0; i<rows; i++)
	{
		tempWord &= ~(1 << i);
	}
	for (uint8_t i = 8; i < (columns * 2); i++)
	{
		tempWord |= (1u << i);
	}
	writeWord(REG_DIR_B, tempWord);
	
	// Set regOpenDrain on 0:7:
	uint8_t tempByte = readByte(REG_OPEN_DRAIN_A);
	for (uint8_t i = 0; i < rows; i++)
	{
		tempByte |= (1u << i);
	}
	writeByte(REG_OPEN_DRAIN_A, tempByte);
	
	// Set regPullUp on 8:15:
	tempByte = readByte(REG_PULL_UP_B);
	for (uint8_t i=0; i < columns; i++)
	{
		tempByte |= (1u << i);
	}
	writeByte(REG_PULL_UP_B, tempByte);
	
	// Debounce Time must be less than scan time
	debounceTime = constrain<uint8_t>(debounceTime, 1, 64);
	scanTime = constrain<uint8_t>(scanTime, 1, 128);
	if (debounceTime >= scanTime)
	{
		debounceTime = scanTime >> 1; // Force debounceTime to be less than scanTime
	}
	debounceKeypad(debounceTime, rows, columns);
	
	// Calculate scanTimeBits, based on scanTime
	uint8_t scanTimeBits = 0;
	for (uint8_t i = 7; i > 0; i--)
	{
		if ((scanTime & (1u << i)) != 0)
		{
			scanTimeBits = i;
			break;
		}
	}
	
	// Calculate sleepTimeBits, based on sleepTime
	uint8_t sleepTimeBits = 0;
	if (sleepTime != 0)
	{
		for (uint8_t i = 7; i != 0; i--)
		{
			if ((sleepTime & (1u << (i + 6))) != 0)
			{
				sleepTimeBits = i;
				break;
			}
		}
		// If sleepTime was non-zero, but less than 128, 
		// assume we wanted to turn sleep on, set it to minimum:
		if (sleepTimeBits == 0)
		{
			sleepTimeBits = 1;
		}
	}
	
	// RegKeyConfig1 sets the auto sleep time and scan time per row
	sleepTimeBits = (sleepTimeBits & 0b111)<<4;	
	scanTimeBits &= 0b111;	// Scan time is bits 2:0
	tempByte = sleepTime | scanTimeBits;
	writeByte(REG_KEY_CONFIG_1, tempByte);
	
	// RegKeyConfig2 tells the SX1509 how many rows and columns we've got going
	rows = (rows - 1) & 0b111;	// 0 = off, 0b001 = 2 rows, 0b111 = 8 rows, etc.
	columns = (columns - 1) & 0b111;	// 0b000 = 1 column, ob111 = 8 columns, etc.
	writeByte(REG_KEY_CONFIG_2, (rows << 3) | columns);
}

uint16_t SX1509::readKeypad()
{
	return ~readWord(REG_KEY_DATA_1);
}

uint8_t SX1509::getRow(uint16_t keyData)
{
	const uint8_t rowData = uint8_t(keyData & 0x00FF);
	
	for (uint8_t i = 0; i < 8; i++)
	{
		if (rowData & (1u << i))
		{
			return i;
		}
	}
	return 0;
}

uint8_t SX1509::getCol(uint16_t keyData)
{
	const uint8_t colData = uint8_t((keyData & 0xFF00) >> 8);
	
	for (uint8_t i = 0; i < 8; i++)
	{
		if (colData & (1u << i))
		{
			return i;
		}
	}
	return 0;
	
}

void SX1509::debounceConfig(uint8_t configValue)
{
	// First make sure clock is configured
	uint8_t tempByte = readByte(REG_MISC);
	if ((tempByte & 0x70) == 0)
	{
		tempByte |= (1<<4);	// Just default to no divider if not set
		writeByte(REG_MISC, tempByte);
	}
	tempByte = readByte(REG_CLOCK);
	if ((tempByte & 0x60) == 0)
	{
		tempByte |= (1<<6);	// default to internal osc.
		writeByte(REG_CLOCK, tempByte);
	}
	
	configValue &= 0b111;	// 3-bit value
	writeByte(REG_DEBOUNCE_CONFIG, configValue);
}

void SX1509::debounceTime(uint8_t time)
{
	// Debounce time-to-uint8_t map: (assuming fOsc = 2MHz)
	// 0: 0.5ms		1: 1ms
	// 2: 2ms		3: 4ms
	// 4: 8ms		5: 16ms
	// 6: 32ms		7: 64ms
	// 2^(n-1)
	uint8_t configValue = 0;
	// We'll check for the highest set bit position, 
	// and use that for debounceConfig
	for (int i = 7; i >= 0; i--)
	{
		if ((time & (1u << i)) != 0)
		{
			configValue = i + 1;
			break;
		}
	}
	configValue = constrain<uint8_t>(configValue, 0, 7);
	
	debounceConfig(configValue);
}

void SX1509::debouncePin(uint8_t pin)
{
	unsigned int debounceEnable = readWord(REG_DEBOUNCE_ENABLE_B);
	debounceEnable |= (1<<pin);
	writeWord(REG_DEBOUNCE_ENABLE_B, debounceEnable);
}

void SX1509::debounceKeypad(uint8_t time, uint8_t numRows, uint8_t numCols)
{
	// Set up debounce time:
	debounceTime(time);
	
	// Set up debounce pins:
	for (uint8_t i = 0; i < numRows; i++)
	{
		debouncePin(i);
	}
	for (uint8_t i = 0; i < (8 + numCols); i++)
	{
		debouncePin(i);
	}
}

#endif

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

uint16_t SX1509::interruptSource(bool clear)
{
	const uint16_t intSource = readWord(REG_INTERRUPT_SOURCE_B);
	if (clear)
	{
		writeWord(REG_INTERRUPT_SOURCE_B, 0xFFFF);	// Clear interrupts
	}
	return intSource;
}

bool SX1509::checkInterrupt(uint8_t pin)
{
	return (interruptSource(false) & (1u << pin)) != 0;
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
	unsigned int timeout = ReceiveTimeout;

	I2C_IFACE.beginTransmission(deviceAddress);
	I2C_IFACE.write(registerAddress);
	if (I2C_IFACE.endTransmission() != 0)
	{
		++errorCount;
		return 0;
	}
	I2C_IFACE.requestFrom(deviceAddress, (uint8_t) 1);

	while ((I2C_IFACE.available() < 1) && (timeout != 0))
	{
		timeout--;
	}
		
	if (timeout == 0)
	{
		++errorCount;
		return 0;
	}

	return I2C_IFACE.read();
}

// readWord(uint8_t registerAddress)
//	This function will read a two-uint8_t word beginning at registerAddress
//	- A 16-bit unsigned int will be returned.
//		- The msb of the return value will contain the value read from registerAddress
//		- The lsb of the return value will contain the value read from registerAddress + 1
uint16_t SX1509::readWord(uint8_t registerAddress)
{
	unsigned int timeout = ReceiveTimeout * 2;

	I2C_IFACE.beginTransmission(deviceAddress);
	I2C_IFACE.write(registerAddress);
	if (I2C_IFACE.endTransmission() != 0)
	{
		++errorCount;
		return 0;
	}
	I2C_IFACE.requestFrom(deviceAddress, (uint8_t) 2);

	while (I2C_IFACE.available() < 2 && timeout != 0)
	{
		timeout--;
	}
		
	if (timeout == 0)
	{
		++errorCount;
		return 0;
	}
	
	const uint16_t msb = (I2C_IFACE.read() & 0x00FF) << 8;
	const uint16_t lsb = (I2C_IFACE.read() & 0x00FF);
	return msb | lsb;
}

// readDword(uint8_t registerAddress)
//	This function will read a two-uint8_t word beginning at registerAddress
//	- A 32-bit unsigned int will be returned.
//		- The msb of the return value will contain the value read from registerAddress
//		- The lsb of the return value will contain the value read from registerAddress + 1
uint32_t SX1509::readDword(uint8_t registerAddress)
{
	unsigned int timeout = ReceiveTimeout * 2;

	I2C_IFACE.beginTransmission(deviceAddress);
	I2C_IFACE.write(registerAddress);
	if (I2C_IFACE.endTransmission() != 0)
	{
		++errorCount;
		return 0;
	}
	I2C_IFACE.requestFrom(deviceAddress, (uint8_t) 4);

	while ((I2C_IFACE.available() < 4) && (timeout != 0))
	{
		timeout--;
	}

	if (timeout == 0)
	{
		++errorCount;
		return 0;
	}

	uint32_t rslt = I2C_IFACE.read() & 0x00FF;
	rslt <<= 8;
	rslt |= (I2C_IFACE.read() & 0x00FF);
	rslt <<= 8;
	rslt |= (I2C_IFACE.read() & 0x00FF);
	rslt <<= 8;
	rslt |= (I2C_IFACE.read() & 0x00FF);
	return rslt;
}

// writeByte(uint8_t registerAddress, uint8_t writeValue)
//	This function writes a single uint8_t to a single register on the SX509.
//	- writeValue is written to registerAddress
//	- deviceAddres should already be set from the constructor
//	- No return value.
void SX1509::writeByte(uint8_t registerAddress, uint8_t writeValue)
{
	I2C_IFACE.beginTransmission(deviceAddress);
	I2C_IFACE.write(registerAddress);
	I2C_IFACE.write(writeValue);
	if (I2C_IFACE.endTransmission() != 0)
	{
		++errorCount;
	}
}

// writeWord(uint8_t registerAddress, uint16_t writeValue)
//	This function writes a two-uint8_t word to registerAddress and registerAddress + 1
//	- the upper uint8_t of writeValue is written to registerAddress
//		- the lower uint8_t of writeValue is written to registerAddress + 1
//	- No return value.
void SX1509::writeWord(uint8_t registerAddress, uint16_t writeValue)
{
	I2C_IFACE.beginTransmission(deviceAddress);
	I2C_IFACE.write(registerAddress);
	I2C_IFACE.write((uint8_t)(writeValue >> 8));
	I2C_IFACE.write((uint8_t)writeValue);
	if (I2C_IFACE.endTransmission() != 0)
	{
		++errorCount;
	}
}

// writeDword(uint8_t registerAddress, uint32_t writeValue)
//	This function writes a four-uint8_t word to registerAddress .. registerAddress + 3, msb first
//	- No return value.
void SX1509::writeDword(uint8_t registerAddress, uint32_t writeValue)
{
	I2C_IFACE.beginTransmission(deviceAddress);
	I2C_IFACE.write(registerAddress);
	I2C_IFACE.write((uint8_t)(writeValue >> 24));
	I2C_IFACE.write((uint8_t)(writeValue >> 16));
	I2C_IFACE.write((uint8_t)(writeValue >> 8));
	I2C_IFACE.write((uint8_t)writeValue);
	if (I2C_IFACE.endTransmission() != 0)
	{
		++errorCount;
	}
}

// End
