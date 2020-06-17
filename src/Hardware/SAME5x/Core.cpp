/*
 * Core_C.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 *
 *  Glue to allow some of our C++ functions to be called from C
 */

#include "Core.h"
#include <Hardware/IoPorts.h>

// IoPort::SetPinMode calls this
extern "C" void pinMode(Pin pin, enum PinMode mode) noexcept
{
	if (pin != NoPin)
	{
		switch (mode)
		{
		case INPUT:
			gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
			break;

		case INPUT_PULLUP:
			gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
			break;

		case INPUT_PULLDOWN:
			gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
			break;

		case OUTPUT_LOW:
			gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
			gpio_set_pin_level(pin, false);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
			break;

		case OUTPUT_HIGH:
			gpio_set_pin_function(pin,GPIO_PIN_FUNCTION_OFF);
			gpio_set_pin_level(pin, true);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
			break;

		case AIN:
			// The SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OFF);		// disable the data input buffer
			gpio_set_pin_function(pin, GPIO_PIN_FUNCTION_B);		// ADC is always on peripheral B
			break;

		default:
			break;
		}
	}
}

// IoPort::ReadPin calls this
extern "C" bool digitalRead(Pin pin) noexcept
{
	const uint8_t port = GPIO_PORT(pin);
	const uint32_t pinMask = 1U << GPIO_PIN(pin);
	return (hri_port_read_IN_reg(PORT, port) & pinMask) != 0;
}

// IoPort::WriteDigital calls this
extern "C" void digitalWrite(Pin pin, bool high) noexcept
{
	const uint8_t port = GPIO_PORT(pin);
	const uint32_t pinMask = 1U << GPIO_PIN(pin);
	if (high)
	{
		hri_port_set_OUT_reg(PORT, port, pinMask);
	}
	else
	{
		hri_port_clear_OUT_reg(PORT, port, pinMask);
	}
}

// End
