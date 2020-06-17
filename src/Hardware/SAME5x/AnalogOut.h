/*
 * AnalogOut.h
 *
 *  Created on: 9 Jul 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_ANALOGOUT_H_
#define SRC_HARDWARE_ANALOGOUT_H_

#include "RepRapFirmware.h"

namespace AnalogOut
{
	// Initialise this module
	extern void Init();

	// Write a PWM value to the specified pin. 'val' will be constrained to be between 0.0 and 1.0 in this module.
	extern void Write(Pin pin, float val, PwmFrequency freq = 500);
}

#endif /* SRC_HARDWARE_ANALOGOUT_H_ */
