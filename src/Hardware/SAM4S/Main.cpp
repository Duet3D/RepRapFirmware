/*
 * Main.cpp
 *  Program entry point
 *  Created on: 11 Jul 2020
 *      Author: David
 *  License: GNU GPL version 3
 */

#include <CoreIO.h>
#include <RepRapFirmware.h>

// Program initialisation
void AppInit() noexcept
{
#if defined(DUET_M)
	// The prototype boards don't have a pulldown on LCD_BEEP, which causes a hissing sound from the beeper on the 12864 display until the pin is initialised
	pinMode(LcdBeepPin, OUTPUT_LOW);

	// Set the 12864 display CS pin low to prevent it from receiving garbage due to other SPI traffic
	pinMode(LcdCSPin, OUTPUT_LOW);

	// On the prototype boards the stepper driver expansion ports don't have external pullup resistors on their enable pins
	pinMode(ENABLE_PINS[5], OUTPUT_HIGH);
	pinMode(ENABLE_PINS[6], OUTPUT_HIGH);
#endif
}

// End
