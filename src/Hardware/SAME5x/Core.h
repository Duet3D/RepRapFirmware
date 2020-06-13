/*
 * Core.h
 *
 *  Created on: 28 May 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_CORE_H_
#define SRC_HARDWARE_SAME5X_CORE_H_


#ifdef __SAME54P20A__
# include <same54.h>
# define __ARM_ARCH_7M__	1
# define SAME5x		1
# define SAMC21		0
#else
# error unsupported processor
#endif

#include <cinttypes>				// for PRIu32 etc.
#include <General/SimpleMath.h>
#include <cctype>

#include "AtmelStart_SAME5x/atmel_start_pins.h"

#define SAM4E	0
#define SAM4S	0
#define SAM3XA	0
#define SAME70	0

typedef uint8_t DmaChannel;
typedef uint8_t Pin;
constexpr Pin NoPin = 0xFF;

inline constexpr Pin PortAPin(unsigned int n) noexcept { return n; }
inline constexpr Pin PortBPin(unsigned int n) noexcept { return 32+n; }
inline constexpr Pin PortCPin(unsigned int n) noexcept { return 64+n; }
inline constexpr Pin PortDPin(unsigned int n) noexcept { return 96+n; }

uint32_t millis() noexcept;
extern "C" uint32_t trueRandom() noexcept;

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
//	OUTPUT_LOW_OPEN_DRAIN,			// used in SX1509B expansion driver to put the pin in open drain output mode
//	OUTPUT_HIGH_OPEN_DRAIN,			// used in SX1509B expansion driver to put the pin in open drain output mode
//	OUTPUT_PWM_OPEN_DRAIN			// used in SX1509B expansion driver to put the pin in PWM output mode
};

typedef int8_t AnalogChannelNumber;	//TOD may need to change this
constexpr AnalogChannelNumber NO_ADC = -1;

AnalogChannelNumber PinToAdcChannel(Pin p) noexcept;

enum InterruptMode
{
	INTERRUPT_MODE_NONE = 0,
	INTERRUPT_MODE_LOW,
	INTERRUPT_MODE_HIGH,
	INTERRUPT_MODE_CHANGE,
	INTERRUPT_MODE_FALLING,
	INTERRUPT_MODE_RISING
};

union CallbackParameter
{
	void *vp;
	uint32_t u32;
	int32_t i32;

	CallbackParameter(void *pp) noexcept : vp(pp) { }
	CallbackParameter(uint32_t pp) noexcept : u32(pp) { }
	CallbackParameter(int32_t pp) noexcept : i32(pp) { }
	CallbackParameter() noexcept : u32(0) { }
};

typedef void (*StandardCallbackFunction)(CallbackParameter) noexcept;

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

static inline int32_t random(uint32_t howbig) noexcept
{
	return trueRandom() % howbig;
}

static inline uint32_t random(uint32_t howsmall, uint32_t howbig) noexcept
{
	return random(howbig - howsmall) + howsmall;
}

#endif /* SRC_HARDWARE_SAME5X_CORE_H_ */
