/*
 * DueXn.cpp
 *
 *  Created on: 19 Oct 2016
 *      Author: David
 */

#include "DueXn.h"
#include "SX1509.h"
#include "Platform.h"

namespace DuetExpansion
{
	static SX1509 expander;
	static uint16_t inputMask;
	static ExpansionBoardType boardType = ExpansionBoardType::none;
	static uint16_t inputBits;

	const uint8_t DueXnAddress = 0x3E;				// address of the SX1509B on the DueX0/DueX2/DueX5

	// The original DueX2 and DueX5 boards had 2 board ID pins, bits 14 an 15.
	// The new ones use bit 15 for fan 8, so not we just have bit 14.
	// If we want any more variants, they will have to use a different I2C address.
	const uint16_t BoardTypePins = (1u << 14);
	const unsigned int BoardTypeShift = 14;
	const ExpansionBoardType boardTypes[] = { ExpansionBoardType::DueX5, ExpansionBoardType::DueX2 };

	const unsigned int Fan3Bit = 12;
	const unsigned int Fan4Bit = 7;
	const unsigned int Fan5Bit = 6;
	const unsigned int Fan6Bit = 5;
	const unsigned int Fan7Bit = 4;
	const unsigned int Fan8Bit = 15;
	const uint16_t AllFanBits = (1u << Fan3Bit) | (1u << Fan4Bit) | (1u << Fan5Bit) | (1u << Fan6Bit) | (1u << Fan7Bit) | (1 << Fan8Bit);

	const unsigned int E2StopBit = 0;
	const unsigned int E3StopBit = 3;
	const unsigned int E4StopBit = 2;
	const unsigned int E5StopBit = 1;
	const unsigned int E6StopBit = 13;
	const uint16_t AllStopBitsX2 = (1u << E2StopBit) | (1u << E3StopBit);
	const uint16_t AllStopBitsX5 = AllStopBitsX2 | (1u << E4StopBit) | (1u << E5StopBit) | (1u << E6StopBit);

	const unsigned int Gpio1Bit = 11;
	const unsigned int Gpio2Bit = 10;
	const unsigned int Gpio3Bit = 9;
	const unsigned int Gpio4Bit = 8;
	const uint16_t AllGpioBits = (1u << Gpio1Bit) | (1u << Gpio2Bit) | (1u << Gpio3Bit) | (1u <<Gpio4Bit);

	// Identify which expansion board (if any) is attached and initialise it
	ExpansionBoardType Init()
	{
		uint8_t ret = expander.begin(DueXnAddress);
		if (ret != 1)
		{
			delay(100);								// wait a little while
			ret = expander.begin(DueXnAddress);		// do 1 retry
		}

		if (ret != 1)
		{
			boardType = ExpansionBoardType::none;	// no device found at that address, or a serious error
		}
		else
		{
			expander.pinModeMultiple(BoardTypePins, INPUT_PULLUP);
			const uint16_t data = expander.digitalReadAll();
			boardType = boardTypes[(data & BoardTypePins) >> BoardTypeShift];
		}

		if (boardType != ExpansionBoardType::none)
		{
			pinMode(DueX_INT, INPUT_PULLUP);
			pinMode(DueX_SG, INPUT_PULLUP);

			expander.pinModeMultiple(AllFanBits, OUTPUT_PWM_LOW);		// Initialise the PWM pins
			const uint16_t stopBits = (boardType == ExpansionBoardType::DueX5) ? AllStopBitsX5 : AllStopBitsX2;	// I am assuming that the X0 has 2 endstop inputs
			expander.pinModeMultiple(stopBits | AllGpioBits, INPUT);	// Initialise the endstop inputs and GPIO pins (no pullups because 5V-tolerant)

			// Set up the interrupt on any input change
			inputMask = stopBits | AllGpioBits;
			expander.enableInterruptMultiple(inputMask, CHANGE);

			// Clear any initial interrupts
			(void)expander.interruptSource(true);
			inputBits = expander.digitalReadAll();
		}

		return boardType;
	}

	// Return the name of the expansion board, or nullptr if no expansion board
	const char* array null GetExpansionBoardName()
	{
		switch(boardType)
		{
		case ExpansionBoardType::DueX5:
			return "DueX5";
		case ExpansionBoardType::DueX2:
			return "DueX2";
		case ExpansionBoardType::DueX0:
			return "DueX0";
		default:
			return nullptr;
		}
	}

	// Set the I/O mode of a pin
	void SetPinMode(Pin pin, PinMode mode)
	{
		if (boardType != ExpansionBoardType::none)
		{
			if (((1 << pin) & AllGpioBits) != 0)
			{
				// The GPIO pins have pullup resistors to +5V, therefore we need to configure them as open drain outputs
				switch(mode)
				{
				case OUTPUT_LOW:
					mode = OUTPUT_LOW_OPEN_DRAIN;
					break;
				case OUTPUT_HIGH:
					mode = OUTPUT_HIGH_OPEN_DRAIN;
					break;
				case OUTPUT_PWM_LOW:
				case OUTPUT_PWM_HIGH:
					mode = OUTPUT_PWM_OPEN_DRAIN;
					break;
				case INPUT_PULLUP:
				case INPUT_PULLDOWN:
					mode = INPUT;			// we are using 5rV-tolerant input with external pullup resistors, so don't enable internal pullup/pulldown resistors
					break;
				default:
					break;
				}
			}
			expander.pinMode(pin, mode);
		}
	}

	// Read a pin
	// TODO this is called from the Step ISR if the 6th axis is used, therefore if should be made more efficient.
	// We need to use the SX15089 interrupt to read the data register using interrupts, and just retrieve that value here.
	bool DigitalRead(Pin pin)
	{
		if (boardType == ExpansionBoardType::none || pin >= 16)
		{
			return false;
		}

		if (!digitalRead(DueX_INT))
		{
			// Interrupt is active, so input data may have changed
			inputBits = expander.digitalReadAll();
		}

		return (inputBits & (1 << pin)) != 0;
	}

	// Write a pin
	void DigitalWrite(Pin pin, bool high)
	{
		if (boardType != ExpansionBoardType::none)
		{
			expander.digitalWrite(pin, high);
		}
	}

	// Set the PWM value on this pin
	void AnalogOut(Pin pin, float pwm)
	{
		if (boardType != ExpansionBoardType::none)
		{
			expander.analogWrite(pin, (uint8_t)(constrain<float>(pwm, 0.0, 1.0) * 255));
		}
	}

	// Diagnose the SX1509 by setting all pins as inputs and reading them
	uint16_t DiagnosticRead()
	{
		expander.pinModeMultiple(AllStopBitsX5 | AllGpioBits | AllFanBits, INPUT);	// Initialise the endstop inputs and GPIO pins (no pullups because 5V-tolerant)
		delay(1);
		const uint16_t retval = expander.digitalReadAll();	// read all inputs with pullup resistors on fans
		Init();															// back to normal
		return retval;
	}
}			// end namespace

// End
