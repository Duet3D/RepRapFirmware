/*
 * Core.h
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 *
 * This file contains basic CPU and I/O pin support.
 * Use it where we can't include the full Core.h file.
 */

#ifndef SRC_HARDWARE_SAME5X_CORE_H_
#define SRC_HARDWARE_SAME5X_CORE_H_

// Basic CPU and I/O pin support

#ifdef __SAME54P20A__
# include <same54.h>
# define __ARM_ARCH_7M__	1
# define SAME5x		1
# define SAMC21		0
#else
# error unsupported processor
#endif

#define SAM4E	0
#define SAM4S	0
#define SAM3XA	0
#define SAME70	0

#include <inttypes.h>				// for PRIu32 etc.
#include <ctype.h>

typedef uint8_t DmaChannel;
typedef uint8_t DmaPriority;
typedef uint8_t Pin;

static const Pin NoPin = 0xFF;

// Pin mode enumeration. Would ideally be a C++ scoped enum, but we need to use it from C library functions.
enum PinMode
{
	PIN_MODE_NOT_CONFIGURED = -1,	// used in Platform class to record that the mode for a pin has not been set yet
	INPUT = 0,						// pin is a digital input
	INPUT_PULLUP,					// pin is a digital input with pullup enabled
	INPUT_PULLDOWN,					// pin is a digital input with pulldown enabled
	OUTPUT_LOW,						// pin is an output with initial state LOW
	OUTPUT_HIGH,					// pin is an output with initial state HIGH
	AIN,							// pin is an analog input, digital input buffer is disabled if possible
	SPECIAL,						// pin is used for the special function defined for it in the variant.cpp file
	OUTPUT_PWM_LOW,					// PWM output mode, initially low
	OUTPUT_PWM_HIGH,				// PWM output mode, initially high
};

#ifndef __cplusplus
# include <stdbool.h>
#endif

#define UNUSED(_x)	(void)_x
#define Assert(expr) ((void) 0)

// Macro to give us the number of elements in an array
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof((_x)[0]))
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;
extern uint32_t SystemPeripheralClock;

uint32_t millis() noexcept;
uint64_t millis64() noexcept;
uint32_t trueRandom() noexcept;

void pinMode(Pin pin, enum PinMode mode) noexcept;
bool digitalRead(Pin pin) noexcept;
void digitalWrite(Pin pin, bool high) noexcept;

static inline void delayMicroseconds(uint32_t) noexcept __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t usec) noexcept
{
    // Based on Paul Stoffregen's implementation for Teensy 3.0 (http://www.pjrc.com/)
    if (usec != 0)
    {
		uint32_t n = usec * (SystemCoreClock / 3000000);
		asm volatile
		(
			".syntax unified"				"\n\t"
			"L_%=_delayMicroseconds:"       "\n\t"
			"subs   %0, #1"   				"\n\t"
			"bne    L_%=_delayMicroseconds" "\n"
			: "+r" (n) :
		);
    }
}

// Functions and macros to enable/disable interrupts
static inline void cpu_irq_enable() noexcept
{
	__DMB();
	__enable_irq();
}

static inline void cpu_irq_disable() noexcept
{
	__disable_irq();
	__DMB();
}

typedef bool irqflags_t;

static inline bool cpu_irq_is_enabled() noexcept
{
	return __get_PRIMASK() == 0;
}

static inline irqflags_t cpu_irq_save() noexcept
{
	const irqflags_t flags = cpu_irq_is_enabled();
	cpu_irq_disable();
	return flags;
}

static inline bool cpu_irq_is_enabled_flags(irqflags_t flags) noexcept
{
	return flags;
}

static inline void cpu_irq_restore(irqflags_t flags) noexcept
{
	if (cpu_irq_is_enabled_flags(flags))
	{
		cpu_irq_enable();
	}
}

static inline bool isDigit(char c) noexcept
{
	return isdigit(c) != 0;
}

// Return true if we are in any interrupt service routine
static inline bool inInterrupt() noexcept
{
	return (__get_IPSR() & 0x01FF) != 0;
}

#ifdef __cplusplus
}		// end extern "C"
#endif

#endif /* SRC_HARDWARE_SAME5X_CORE_H_ */
