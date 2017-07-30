/*
 * Texas Instruments SPI DAC084S085
 *
 *  Created on: 14 May 2017
 *      Author: Marco Antonini
 */

#ifndef DAC084S085_H_
#define DAC084S085_H_

#include <inttypes.h>
#include "Core.h"
#include "SharedSpi.h"

// 10.0 MHz standard clock frequency for DAC084S085
#define DAC084S085_MAX_FREQ 10000000u


class DAC084S085{

public:

	DAC084S085();

	void Init(uint8_t cs);
	void setChannel(uint8_t channel, unsigned short value);

private:

	bool initialized;
    struct sspi_device device; // Alligator onboard DAC
};


#endif /* DAC084S_ */
