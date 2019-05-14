/*
 * DueXn.cpp
 *
 *  Created on: 19 Oct 2016
 *      Author: David
 */

#include "DueXn.h"
#include "SX1509.h"
#include "Platform.h"
#include "RepRap.h"
#include "Wire.h"
#include "Hardware/I2C.h"

namespace DuetExpansion
{
	constexpr uint8_t DueXnAddress = 0x3E;						// address of the SX1509B on the DueX2/DueX5
	static constexpr unsigned int DueXTaskStackWords = 100;		// task stack size in dwords

	static SX1509 dueXnExpander;
	static uint16_t dueXnInputMask;
	static uint16_t dueXnInputBits = 0;
	static ExpansionBoardType dueXnBoardType = ExpansionBoardType::none;

	const uint8_t AdditionalIoExpanderAddress = 0x71;	// address of the SX1509B we allow for general I/O expansion

	static SX1509 additionalIoExpander;
	static bool additionalIoExpanderPresent = false;
	static uint16_t additionalIoInputBits = 0;

	static volatile bool taskWaiting = false;
	static volatile bool inputsChanged = false;

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
	static void DueXIrq(CallbackParameter p)
	{
		inputsChanged = true;
		if (taskWaiting)
		{
			taskWaiting = false;
			dueXTask->GiveFromISR();
		}
	}

	extern "C" [[noreturn]] void DueXTask(void * pvParameters)
	{
		for (;;)
		{
			inputsChanged = false;
			dueXnInputBits = dueXnExpander.digitalReadAll();

			cpu_irq_disable();
			if (!inputsChanged)
			{
				taskWaiting = true;
				cpu_irq_enable();
				TaskBase::Take();
			}
			else
			{
				cpu_irq_enable();
			}
		}
	}

	// Identify which expansion board (if any) is attached and initialise it
	ExpansionBoardType DueXnInit()
	{
		I2C::Init();										// initialise I2C

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
			const uint16_t stopBits = (dueXnBoardType == ExpansionBoardType::DueX5) ? AllStopBitsX5 : AllStopBitsX2;	// I am assuming that the X0 has 2 endstop inputs
			dueXnExpander.pinModeMultiple(stopBits | AllGpioBits, INPUT);	// Initialise the endstop inputs and GPIO pins (no pullups because 5V-tolerant)

			// Set up the interrupt on any input change
			dueXnInputMask = stopBits | AllGpioBits;
			dueXnExpander.enableInterruptMultiple(dueXnInputMask, INTERRUPT_MODE_CHANGE);
			attachInterrupt(DueX_INT, DueXIrq, InterruptMode::INTERRUPT_MODE_FALLING, nullptr);

			// Clear any initial interrupts
			(void)dueXnExpander.interruptSourceAndClear();

			dueXTask = new Task<DueXTaskStackWords>();
			dueXTask->Create(DueXTask, "DUEX", nullptr, TaskPriority::DueXPriority);
		}

		return dueXnBoardType;
	}

	// Look for an additional output pin expander
	void AdditionalOutputInit()
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
	const char* array null GetExpansionBoardName()
	{
		switch(dueXnBoardType)
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

	// Return the name of the additional expansion board, or nullptr if no expansion board
	const char* array null GetAdditionalExpansionBoardName()
	{
		return (additionalIoExpanderPresent) ? "SX1509B expander" : nullptr;
	}

	// Set the I/O mode of a pin
	void SetPinMode(Pin pin, PinMode mode)
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
	bool DigitalRead(Pin pin)
	{
		if (pin >= DueXnExpansionStart && pin < DueXnExpansionStart + 16)
		{
			if (dueXnBoardType != ExpansionBoardType::none)
			{
				if (!digitalRead(DueX_INT) && !inInterrupt() && __get_BASEPRI() == 0)	// we must not call expander.digitalRead() from within an ISR or if the tick interrupt is disabled
				{
					// Interrupt is active, so input data may have changed
					dueXnInputBits = dueXnExpander.digitalReadAll();
				}

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
	void DigitalWrite(Pin pin, bool high)
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

	// Set the PWM value on this pin
	void AnalogOut(Pin pin, float pwm)
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

	// Print diagnostic data
	// I2C error counts are now reported by Platform, so nothing to report here.
	void Diagnostics(MessageType mtype)
	{
	}

	// Diagnose the SX1509 by setting all pins as inputs and reading them
	uint16_t DiagnosticRead()
	{
		dueXnExpander.pinModeMultiple(AllStopBitsX5 | AllGpioBits | AllFanBits, INPUT);	// Initialise the endstop inputs and GPIO pins (no pullups because 5V-tolerant)
		delay(1);
		const uint16_t retval = dueXnExpander.digitalReadAll();		// read all inputs with pullup resistors on fans
		DueXnInit();												// back to normal
		return retval;
	}
}			// end namespace

// End
