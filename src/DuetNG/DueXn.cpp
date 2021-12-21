/*
 * DueXn.cpp
 *
 *  Created on: 19 Oct 2016
 *      Author: David
 */

#include "DueXn.h"
#include "SX1509.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Wire.h>
#include <Hardware/I2C.h>
#include <Platform/TaskPriorities.h>
#include <Interrupts.h>

namespace DuetExpansion
{
	constexpr uint8_t DueXnAddress = 0x3E;						// address of the SX1509B on the DueX2/DueX5
	static constexpr unsigned int DueXTaskStackWords = 100;		// task stack size in dwords

	static SX1509 dueXnExpander;
	static uint16_t dueXnInputMask;
	static volatile uint16_t dueXnInputBits = 0;
	static ExpansionBoardType dueXnBoardType = ExpansionBoardType::none;

	const uint8_t AdditionalIoExpanderAddress = 0x71;	// address of the SX1509B we allow for general I/O expansion

	static SX1509 additionalIoExpander;
	static bool additionalIoExpanderPresent = false;
	static uint16_t additionalIoInputBits = 0;

	static volatile uint32_t dueXnReadCount = 0;
	static uint32_t dueXnReadCountResetMillis = 0;
	static volatile bool taskWaiting = false;

	Task<DueXTaskStackWords> *dueXTask = nullptr;

	// The original DueX2 and DueX5 boards had 2 board ID pins, bits 14 and 15.
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

	// ISR for when the SX1509B on the DueX indicates that the state of an input has changed.
	// Note, we must only wake up the DueX task if it is waiting on this specific interrupt.
	// Otherwise we might wake it prematurely when it is waiting for an I2C transaction to be completed.
	static void DueXIrq(CallbackParameter p) noexcept
	{
		if (taskWaiting)
		{
			taskWaiting = false;
			dueXTask->GiveFromISR();
		}
	}

	// This is the loop executed by the DueX task
	extern "C" [[noreturn]] void DueXTask(void * pvParameters) noexcept
	{
		for (;;)
		{
			taskWaiting = false;						// make sure we are not notified while we do the I2C transaction
			TaskBase::ClearNotifyCount();
			dueXnInputBits = dueXnExpander.digitalReadAll();
			taskWaiting = true;
			++dueXnReadCount;
			if (digitalRead(DueX_INT))
			{
				(void)TaskBase::Take();
			}
		}
	}
}

// Identify which expansion board (if any) is attached and initialise it
ExpansionBoardType DuetExpansion::DueXnInit() noexcept
{
	I2C::Init();										// initialise I2C
	delay(200);											// the SX1509B has an independent power on reset, so give it some time

	// DC 2018-07-12: occasionally the SX1509B isn't found after doing a software reset, so try a few more attempts
	bool ret;
	unsigned int attempts = 0;
	do
	{
		++attempts;
		delay(50);
		ret = dueXnExpander.begin(DueXnAddress);
	} while (!ret && attempts < 5);
	(void)I2C_IFACE.GetErrorCounts(true);				// clear the error counts in case there wasn't a device there or we didn't find it first time

	if (ret)
	{
		dueXnExpander.pinModeMultiple(BoardTypePins, INPUT_PULLUP);
		const uint16_t data = dueXnExpander.digitalReadAll();
		dueXnBoardType = boardTypes[(data & BoardTypePins) >> BoardTypeShift];
		// Check whether it is a version 0.11 board
		pinMode(DIRECTION_PINS[5], INPUT_PULLUP);
		delayMicroseconds(10);
		if (!digitalRead(DIRECTION_PINS[5]))
		{
			// There is a pulldown on driver 5 direction, so this is a version 0.11 board
			dueXnBoardType = (ExpansionBoardType)((uint8_t)dueXnBoardType + 1);
		}
	}
	else
	{
		dueXnBoardType = ExpansionBoardType::none;		// no device found at that address, or a serious error
	}

	if (dueXnBoardType != ExpansionBoardType::none)
	{
		pinMode(DueX_INT, INPUT_PULLUP);
		pinMode(DueX_SG, INPUT_PULLUP);

		dueXnExpander.pinModeMultiple(AllFanBits, OUTPUT_PWM_LOW);		// Initialise the PWM pins
		const uint16_t stopBits = (dueXnBoardType == ExpansionBoardType::DueX2) ? AllStopBitsX2 : AllStopBitsX5;	// I am assuming that the X0 and original X2 have 2 endstop inputs
		dueXnExpander.pinModeMultiple(stopBits | AllGpioBits, INPUT);	// Initialise the endstop inputs and GPIO pins (no pullups because 5V-tolerant)
		dueXnInputMask = stopBits | AllGpioBits;
		dueXnExpander.enableInterruptMultiple(dueXnInputMask, InterruptMode::change);
	}

	return dueXnBoardType;
}

// Create the DueXn task and enable the associated interrupt from the DueXn.
// This must be called after interrupt priorities have been configured, to comply with FreeRTOS constraints.
void DuetExpansion::DueXnTaskInit() noexcept
{
	if (dueXnBoardType != ExpansionBoardType::none)
	{
		// Set up the interrupt on any input change
		attachInterrupt(DueX_INT, DueXIrq, InterruptMode::falling, CallbackParameter(nullptr));

		// Clear any initial interrupts
		(void)dueXnExpander.interruptSourceAndClear();

		dueXTask = new Task<DueXTaskStackWords>();
		dueXTask->Create(DueXTask, "DUEX", nullptr, TaskPriority::DueXPriority);
	}
}

// Look for an additional output pin expander
void DuetExpansion::AdditionalOutputInit() noexcept
{
	I2C::Init();										// initialise I2C

	bool ret;
	unsigned int attempts = 0;
	do
	{
		++attempts;
		delay(50);
		ret = additionalIoExpander.begin(AdditionalIoExpanderAddress);
	} while (!ret && attempts < 5);
	(void)I2C_IFACE.GetErrorCounts(true);				// clear the error counts in case there wasn't a device there or we didn't find it first time

	if (ret)
	{
		additionalIoExpander.pinModeMultiple((1u << 16) - 1, INPUT_PULLDOWN);
		additionalIoInputBits = additionalIoExpander.digitalReadAll();
		additionalIoExpanderPresent = true;
	}
}

// Return the name of the expansion board, or nullptr if no expansion board
const char* _ecv_array null DuetExpansion::GetExpansionBoardName() noexcept
{
	switch (dueXnBoardType)
	{
	case ExpansionBoardType::DueX5:
		return "DueX5";
	case ExpansionBoardType::DueX5_v0_11:
		return "DueX5v0.11";
	case ExpansionBoardType::DueX2:
		return "DueX2";
	case ExpansionBoardType::DueX2_v0_11:
		return "DueX2v0.11";
	case ExpansionBoardType::DueX0:
		return "DueX0";
	default:
		return nullptr;
	}
}

// Return the name of the additional expansion board, or nullptr if no expansion board
const char* _ecv_array null DuetExpansion::GetAdditionalExpansionBoardName() noexcept
{
	return (additionalIoExpanderPresent) ? "SX1509B expander" : nullptr;
}

// Set the I/O mode of a pin
void DuetExpansion::SetPinMode(Pin pin, PinMode mode) noexcept
{
	if (pin >= DueXnExpansionStart && pin < DueXnExpansionStart + 16)
	{
		if (dueXnBoardType != ExpansionBoardType::none)
		{
			pin -= DueXnExpansionStart;
			if (((1u << pin) & AllGpioBits) != 0)
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
					mode = INPUT;			// we are using 5V-tolerant input with external pullup resistors, so don't enable internal pullup/pulldown resistors
					break;
				default:
					break;
				}
			}
			dueXnExpander.pinMode(pin, mode);
		}
	}
	else if (pin >= AdditionalIoExpansionStart && pin < AdditionalIoExpansionStart + 16)
	{
		if (additionalIoExpanderPresent)
		{
			additionalIoExpander.pinMode(pin - AdditionalIoExpansionStart, mode);
		}
	}
}

// Read a pin
// We need to use the SX1509 interrupt to read the data register using interrupts, and just retrieve that value here.
bool DuetExpansion::DigitalRead(Pin pin) noexcept
{
	if (pin >= DueXnExpansionStart && pin < DueXnExpansionStart + 16)
	{
		if (dueXnBoardType != ExpansionBoardType::none)
		{
			return (dueXnInputBits & (1u << (pin - DueXnExpansionStart))) != 0;
		}
	}
	else if (pin >= AdditionalIoExpansionStart && pin < AdditionalIoExpansionStart + 16)
	{
		if (additionalIoExpanderPresent)
		{
			// We don't have an interrupt from the additional I/O expander, so always read fresh data.
			// If this is called from inside an ISR, we will get stale data.
			if (!inInterrupt() && __get_BASEPRI() == 0)								// we must not call expander.digitalRead() from within an ISR
			{
				additionalIoInputBits = additionalIoExpander.digitalReadAll();
			}

			return (additionalIoInputBits & (1u << (pin - AdditionalIoExpansionStart))) != 0;
		}
	}

	return false;
}

// Write a pin
void DuetExpansion::DigitalWrite(Pin pin, bool high) noexcept
{
	if (!inInterrupt() && __get_BASEPRI() == 0)										// we must not call expander.digitalWrite() from within an ISR
	{
		if (pin >= DueXnExpansionStart && pin < DueXnExpansionStart + 16)
		{
			if (dueXnBoardType != ExpansionBoardType::none)
			{
				dueXnExpander.digitalWrite(pin - DueXnExpansionStart, high);
			}
		}
		else if (pin >= AdditionalIoExpansionStart && pin < AdditionalIoExpansionStart + 16)
		{
			if (additionalIoExpanderPresent)
			{
				additionalIoExpander.digitalWrite(pin - AdditionalIoExpansionStart, high);
			}
		}
	}
}

// Set the PWM value on this pin
void DuetExpansion::AnalogOut(Pin pin, float pwm) noexcept
{
	if (!inInterrupt() && __get_BASEPRI() == 0)										// we must not call expander.analogWrite() from within an ISR
	{
		if (pin >= DueXnExpansionStart && pin < DueXnExpansionStart + 16)
		{
			if (dueXnBoardType != ExpansionBoardType::none)
			{
				dueXnExpander.analogWrite(pin - DueXnExpansionStart, (uint8_t)(constrain<float>(pwm, 0.0, 1.0) * 255));
			}
		}
		else if (pin >= AdditionalIoExpansionStart && pin < AdditionalIoExpansionStart + 16)
		{
			if (additionalIoExpanderPresent)
			{
				additionalIoExpander.analogWrite(pin - AdditionalIoExpansionStart, (uint8_t)(constrain<float>(pwm, 0.0, 1.0) * 255));
			}
		}
	}
}

// Print diagnostic data. I2C error counts are now reported by Platform.
void DuetExpansion::Diagnostics(MessageType mtype) noexcept
{
	if (dueXnBoardType != ExpansionBoardType::none)
	{
		const uint32_t now = millis();
		const uint32_t readCount = dueXnReadCount;
		dueXnReadCount = 0;

		reprap.GetPlatform().MessageF(mtype,
										"=== DueX ===\nRead count %" PRIu32 ", %.02f reads/min\n",
										readCount,
										(double)(((float)readCount * (SecondsToMillis * MinutesToSeconds))/(now - dueXnReadCountResetMillis))
									 );
		dueXnReadCountResetMillis = now;
	}
}

// Diagnose the SX1509 by setting all pins as inputs and reading them
uint16_t DuetExpansion::DiagnosticRead() noexcept
{
	dueXnExpander.pinModeMultiple(AllStopBitsX5 | AllGpioBits | AllFanBits, INPUT);	// Initialise the endstop inputs and GPIO pins (no pullups because 5V-tolerant)
	delay(1);
	const uint16_t retval = dueXnExpander.digitalReadAll();		// read all inputs with pullup resistors on fans
	DueXnInit();												// back to normal
	return retval;
}

void DuetExpansion::Exit()
{
	detachInterrupt(DueX_INT);
	if (dueXTask != nullptr)
	{
		dueXTask->TerminateAndUnlink();
	}
}

// End
