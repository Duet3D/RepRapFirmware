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

extern uint32_t millis() noexcept;

#endif /* SRC_HARDWARE_SAME5X_CORE_H_ */
