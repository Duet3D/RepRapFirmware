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

#define SAM4E	0
#define SAM4S	0
#define SAM3XA	0
#define SAME70	0

#include <inttypes.h>				// for PRIu32 etc.
#include <ctype.h>
#include "AtmelStart_SAME5x/atmel_start_pins.h"

typedef uint8_t DmaChannel;
typedef uint8_t Pin;

const Pin NoPin = 0xFF;

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

// The rest is available only when compiling in C++ mode

#include <General/SimpleMath.h>

inline constexpr Pin PortAPin(unsigned int n) noexcept { return n; }
inline constexpr Pin PortBPin(unsigned int n) noexcept { return 32+n; }
inline constexpr Pin PortCPin(unsigned int n) noexcept { return 64+n; }
inline constexpr Pin PortDPin(unsigned int n) noexcept { return 96+n; }

// Pin function numbers for calls to gpio_set_pin_function
enum class GpioPinFunction : uint32_t { A = 0, B, C, D, E, F, G, H, I, J, K, L, M, N };

inline void SetPinFunction(Pin p, GpioPinFunction f)
{
	gpio_set_pin_function(p, (uint32_t)f);
}

inline void ClearPinFunction(Pin p)
{
	gpio_set_pin_function(p, GPIO_PIN_FUNCTION_OFF);
}

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

extern "C" uint32_t SystemCoreClock;			// in system_samxxx.c
extern "C" uint32_t SystemPeripheralClock;		// in system_samxxx.c

void watchdogReset() noexcept;
void CoreSysTick() noexcept;

static inline int32_t random(uint32_t howbig) noexcept
{
	return trueRandom() % howbig;
}

static inline uint32_t random(uint32_t howsmall, uint32_t howbig) noexcept
{
	return random(howbig - howsmall) + howsmall;
}

// Set a pin high with no error checking
inline void fastDigitalWriteHigh(uint32_t pin) noexcept
{
	hri_port_set_OUT_reg(PORT, GPIO_PORT(pin), 1U << GPIO_PIN(pin));
}

// Set a pin low with no error checking
inline void fastDigitalWriteLow(uint32_t pin) noexcept
{
	hri_port_clear_OUT_reg(PORT, GPIO_PORT(pin), 1U << GPIO_PIN(pin));
}

[[noreturn]] void Reset();

// Timer identifiers used in assigning PWM control devices
enum class TcOutput : uint8_t
{
	// TC devices, on peripheral E for both SAME51 and SAMC21
	tc0_0 = 0, tc0_1,
	tc1_0, tc1_1,
	tc2_0, tc2_1,
	tc3_0, tc3_1,
	tc4_0, tc4_1,
	tc5_0, tc5_1,
	tc6_0, tc6_1,
	tc7_0, tc7_1,

	none = 0xFF,
};

static inline constexpr unsigned int GetDeviceNumber(TcOutput tc) { return (uint8_t)tc >> 1; }
static inline constexpr unsigned int GetOutputNumber(TcOutput tc) { return (uint8_t)tc & 1; }

// Initialise a TC clock
void EnableTcClock(unsigned int tcNumber, uint32_t gclkVal);
void EnableTccClock(unsigned int tccNumber, uint32_t gclkVal);

enum class TccOutput : uint8_t
{
	// TCC devices on peripheral F
	tcc0_0F = 0x00, tcc0_1F, tcc0_2F, tcc0_3F, tcc0_4F, tcc0_5F,
	tcc1_0F = 0x08, tcc1_1F, tcc1_2F, tcc1_3F, tcc1_4F, tcc1_5F, tcc1_6F,
	tcc2_0F = 0x10, tcc2_1F, tcc2_2F, tcc2_3F, tcc2_4F, tcc2_5F,
	tcc3_0F = 0x18, tcc3_1F, tcc3_2F, tcc3_3F, tcc3_4F, tcc3_5F,
	tcc4_0F = 0x20, tcc4_1F, tcc4_2F, tcc4_3F, tcc4_4F, tcc4_5F,
	tcc5_0F = 0x28, tcc5_1F, tcc5_2F, tcc5_3F, tcc5_4F, tcc5_5F,

	// TCC devices on peripheral G
	tcc0_0G = 0x80, tcc0_1G, tcc0_2G, tcc0_3G, tcc0_4G, tcc0_5G, tcc0_6G, tcc0_7G,
	tcc1_0G = 0x88, tcc1_1G, tcc1_2G, tcc1_3G, tcc1_4G, tcc1_5G,
	tcc2_0G = 0x90, tcc2_1G, tcc2_2G, tcc2_3G, tcc2_4G, tcc2_5G,
	tcc3_0G = 0x98, tcc3_1G, tcc3_2G, tcc3_3G, tcc3_4G, tcc3_5G,
	tcc4_0G = 0xA0, tcc4_1G, tcc4_2G, tcc4_3G, tcc4_4G, tcc4_5G,
	tcc5_0G = 0xA8, tcc5_1G, tcc5_2G, tcc5_3G, tcc5_4G, tcc5_5G,

	none = 0xFF
};

static inline constexpr unsigned int GetDeviceNumber(TccOutput tcc) { return ((uint8_t)tcc & 0x7F) >> 3; }
static inline constexpr unsigned int GetOutputNumber(TccOutput tcc) { return (uint8_t)tcc & 7; }

static inline constexpr GpioPinFunction GetPeriNumber(TccOutput tcc)
{
	return ((uint8_t)tcc >= 0x80) ? GpioPinFunction::G : GpioPinFunction::F;		// peripheral G or F
}

// ADC input identifiers
enum class AdcInput : uint8_t
{
	adc0_0 = 0x00, adc0_1, adc0_2, adc0_3, adc0_4, adc0_5, adc0_6, adc0_7, adc0_8, adc0_9, adc0_10, adc0_11,
	adc0_12, adc0_13, adc0_14, adc0_15,
	adc1_0 = 0x10, adc1_1, adc1_2, adc1_3, adc1_4, adc1_5, adc1_6, adc1_7, adc1_8, adc1_9, adc1_10, adc1_11,
	none = 0xFF
};

static inline constexpr unsigned int GetDeviceNumber(AdcInput ain) { return (uint8_t)ain >> 4; }
static inline constexpr unsigned int GetInputNumber(AdcInput ain) { return (uint8_t)ain & 0x0F; }

// SERCOM identifiers
enum class SercomIo : uint8_t
{
	// SERCOM pins on peripheral C
	sercom0c = 0x00,
	sercom1c, sercom2c, sercom3c, sercom4c, sercom5c,
	sercom6c, sercom7c,

	// SERCOM pins on peripheral D
	sercom0d = 0x80,
	sercom1d, sercom2d, sercom3d, sercom4d, sercom5d,
	sercom6d, sercom7d,

	none = 0xFF
};

static inline constexpr unsigned int GetDeviceNumber(SercomIo sercom) { return (uint8_t)sercom & 7; }
static inline constexpr unsigned int GetPeriNumber(SercomIo sercom) { return ((uint8_t)sercom >= 0x80) ? 3 : 2; }	// peripheral D or C

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

#if 0
// Struct to represent a pin that can be assigned to various functions
// This can be varied to suit the hardware. It is a struct not a class so that it can be direct initialised in read-only memory.
struct PinEntry
{
	Pin GetPin() const noexcept { return pin; }
	PinCapability GetCapability() const noexcept { return cap; }
	const char* GetNames() const noexcept { return names; }

	Pin pin;
	PinCapability cap;
	const char *names;
};
#endif

// The pin description says what functions are available on each pin, filtered to avoid allocating the same function to more than one pin..
struct PinDescription
{
	TcOutput tc;
	TccOutput tcc;
	AdcInput adc;
	SercomIo sercomIn;
	SercomIo sercomOut;
	uint8_t exintNumber;
	PinCapability cap;
	const char* pinNames;
};

constexpr uint32_t SerialNumberAddresses[4] = { 0x008061FC, 0x00806010, 0x00806014, 0x00806018 };

#endif	// #if __cplusplus

#endif /* SRC_HARDWARE_SAME5X_CORE_H_ */
