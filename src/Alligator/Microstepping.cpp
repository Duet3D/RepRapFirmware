/*
 * Alligator Microstepping Settings
 *
 *  Created on: 15 May 2017
 *      Author: Marco Antonini
 */

#include "Microstepping.h"


//***************************************************************************************************
// Alligator Microstepping static class

uint8_t microsteppingPins[ DRIVES - MinAxes ];

void Microstepping::Init() {


	static_assert(sizeof(microsteppingPins) == sizeof(MICROSTEPPING_PINS), "Incompatible array types");
	memcpy(microsteppingPins, MICROSTEPPING_PINS, sizeof(MICROSTEPPING_PINS));

	for(uint8_t pin=0; pin < (DRIVES-MinAxes); pin++ ) {
		pinMode( microsteppingPins[pin], OUTPUT_HIGH);
	}
}

bool Microstepping::Set(uint8_t drive, uint8_t value) {

	// Do not exceed the maximum value
	if ( drive >= (DRIVES - MinAxes) )
	{
		return false;
	}
	if ( value != 16 && value != 32 )
	{
		return false;
	}

	// Write Value and check status
	digitalWrite( microsteppingPins[drive], value == 16 ? LOW : HIGH );

	return true;
}

uint8_t Microstepping::Read( uint8_t drive ) {

	// Do not exceed the maximum value
	if ( drive < (DRIVES - MinAxes) )
	{
		return ( digitalRead(microsteppingPins[drive]) == LOW ? 16 : 32 );
	}
	else
	{
		return 16;
	}
}

// End
