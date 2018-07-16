/*
 * Tacho.h
 *
 *  Created on: 16 Jul 2018
 *      Author: David
 */

#ifndef SRC_FANS_TACHO_H_
#define SRC_FANS_TACHO_H_

#include "RepRapFirmware.h"

class Tacho
{
public:
	Tacho();
	void Init(Pin p_pin);
	uint32_t GetRPM() const;

	void Interrupt();

private:
	static constexpr uint32_t fanMaxInterruptCount = 32;	// number of fan interrupts that we average over

	uint32_t fanInterruptCount;						// accessed only in ISR, so no need to declare it volatile
	volatile uint32_t fanLastResetTime;				// time (microseconds) at which we last reset the interrupt count, accessed inside and outside ISR
	volatile uint32_t fanInterval;					// written by ISR, read outside the ISR

	Pin pin;
};

#endif /* SRC_FANS_TACHO_H_ */
