/*
 * DueXn.cpp
 *
 *  Created on: 19 Oct 2016
 *      Author: David
 */

#include "DueXn.h"
#include "SX1509.h"

namespace DuetExpansion
{
	static SX1509 expander;
	static uint16_t inputMask;
	static ExpansionBoardType boardType = ExpansionBoardType::none;

	const uint8_t DueXnAddress = 0x3E;				// address of the SX1509B on the DueX0/DueX2/DueX5

	const uint16_t BoardTypePins = (1u << 14) | (1u << 15);
	const unsigned int BoardTypeShift = 14;
	const ExpansionBoardType boardTypes[] =
		{ ExpansionBoardType::DueX5, ExpansionBoardType::DueX2, ExpansionBoardType::DueX0, ExpansionBoardType::none };

	const unsigned int Fan3Bit = 12;
	const unsigned int Fan4Bit = 7;
	const unsigned int Fan5Bit = 6;
	const unsigned int Fan6Bit = 5;
	const unsigned int Fan7Bit = 4;
	const uint16_t AllFanBits = (1u << Fan3Bit) | (1u << Fan4Bit) | (1u << Fan5Bit) | (1u << Fan6Bit) | (1u << Fan7Bit);

	const unsigned int E2StopBit = 0;
	const unsigned int E3StopBit = 3;
	const unsigned int E4StopBit = 2;
	const unsigned int E5StopBit = 1;
	const unsigned int E6StopBit = 13;
	const uint16_t AllStopBits = (1u << E2StopBit) | (1u << E3StopBit) | (1u << E4StopBit) | (1u << E5StopBit) | (1u << E6StopBit);

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
			expander.pinModeMultiple(AllFanBits, OUTPUT_PWM);			// Initialise the PWM pins
			expander.pinModeMultiple(AllStopBits, INPUT);				// Initialise the endstop inputs (no pullups because 5V-tolerant)
			expander.pinModeMultiple(AllGpioBits, INPUT);				// Initialise the GPIO pins as inputs

			// Set up the interrupt on any input change
			inputMask = AllStopBits | AllGpioBits;
			expander.enableInterruptMultiple(inputMask, CHANGE);

			// Clear any initial interrupts
			expander.interruptSource(true);
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
				case OUTPUT_PWM:
					mode = OUTPUT_PWM_OPEN_DRAIN;
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
		return (boardType != ExpansionBoardType::none)
				? expander.digitalRead(pin)
				: false;
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
}			// end namespace

// End
