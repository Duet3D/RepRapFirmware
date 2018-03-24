/*
 * Alligator Microstepping Settings
 *
 *  Created on: 15 May 2017
 *      Author: Marco Antonini
 */

#ifndef ALLIGATOR_MICROSTEPPING_H_
#define ALLIGATOR_MICROSTEPPING_H_

#include <inttypes.h>
#include "Core.h"
#include "Pins_Alligator.h"

class Microstepping {

	public:

		static void Init();
		static bool Set(uint8_t drive, uint8_t value);
		static uint8_t Read( uint8_t drive );

	private:

		Microstepping() { };

};
#endif /* ALLIGATOR_MICROSTEPPING_H_ */
