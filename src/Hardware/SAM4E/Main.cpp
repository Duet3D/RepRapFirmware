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
	// When the reset button is pressed on pre-production Duet WiFi boards, if the TMC2660 drivers were previously enabled then we get
	// uncommanded motor movements if the STEP lines pick up any noise. Try to reduce that by initialising the pins that control the drivers early here.
	// On the production boards the ENN line is pulled high by an external pullup resistor and that prevents motor movements.
	// We no longer do the direction pins because we use some of those as board version indicators.
	for (size_t drive = 0; drive < MaxSmartDrivers; ++drive)
	{
		pinMode(STEP_PINS[drive], OUTPUT_LOW);
		pinMode(ENABLE_PINS[drive], OUTPUT_HIGH);
	}
}

// End
