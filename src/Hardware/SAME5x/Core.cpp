/*
 * Core_C.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 *
 *  Glue to allow some of our C++ functions to be called from C
 */

#include "Core_C.h"
#include <Hardware/IoPorts.h>

extern "C" void pinMode(Pin pin, enum PinMode mode) noexcept
{
	IoPort::SetPinMode(pin, mode);
}

extern "C" bool digitalRead(Pin pin) noexcept
{
	return IoPort::ReadPin(pin);
}

// End
