/*
 * PinDescription.h
 *
 *  Created on: 10 Jul 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAM4S_PINDESCRIPTION_H_
#define SRC_HARDWARE_SAM4S_PINDESCRIPTION_H_

#include <CoreIO.h>

// Enum to represent allowed types of pin access
// We don't have a separate bit for servo, because Duet PWM-capable ports can be used for servos if they are on the Duet main board
enum class PinCapability: uint8_t
{
	// Individual capabilities
	none = 0,
	read = 1,
	ain = 2,
	write = 4,
	pwm = 8,

	// Combinations
	ainr = 1|2,
	rw = 1|4,
	wpwm = 4|8,
	rwpwm = 1|4|8,
	ainrw = 1|2|4,
	ainrwpwm = 1|2|4|8
};

constexpr inline PinCapability operator|(PinCapability a, PinCapability b) noexcept
{
	return (PinCapability)((uint8_t)a | (uint8_t)b);
}

// The pin description says what functions are available on each pin, filtered to avoid allocating the same function to more than one pin..
// It is a struct not a class so that it can be direct initialised in read-only memory.
struct PinDescription : public PinDescriptionBase
{
	PinCapability cap;
	const char* pinNames;

	PinCapability GetCapability() const noexcept { return cap; }
	const char* GetNames() const noexcept { return pinNames; }
};

#endif /* SRC_HARDWARE_SAM4S_PINDESCRIPTION_H_ */
