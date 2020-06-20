/*
 * Core.h
 *
 *  Created on: 28 May 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_COREIO_H_
#define SRC_HARDWARE_SAME5X_COREIO_H_

#include "Core.h"

#include <General/SimpleMath.h>
#include <hal_gpio.h>

inline constexpr Pin PortAPin(unsigned int n) noexcept { return n; }
inline constexpr Pin PortBPin(unsigned int n) noexcept { return 32+n; }
inline constexpr Pin PortCPin(unsigned int n) noexcept { return 64+n; }
inline constexpr Pin PortDPin(unsigned int n) noexcept { return 96+n; }

// Pin function numbers for calls to gpio_set_pin_function
enum class GpioPinFunction : uint32_t { A = 0, B, C, D, E, F, G, H, I, J, K, L, M, N };

inline void SetPinFunction(Pin p, GpioPinFunction f) noexcept
{
	gpio_set_pin_function(p, (uint32_t)f);
}

inline void ClearPinFunction(Pin p) noexcept
{
	gpio_set_pin_function(p, GPIO_PIN_FUNCTION_OFF);
}

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

void WatchdogInit() noexcept;
void watchdogReset() noexcept;
void CoreSysTick() noexcept;
void CoreInit() noexcept;

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

[[noreturn]] void Reset() noexcept;

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

static inline constexpr unsigned int GetDeviceNumber(TcOutput tc) noexcept { return (uint8_t)tc >> 1; }
static inline constexpr unsigned int GetOutputNumber(TcOutput tc) noexcept { return (uint8_t)tc & 1; }

// Initialise a TC clock
void EnableTcClock(unsigned int tcNumber, uint32_t gclkVal) noexcept;
void EnableTccClock(unsigned int tccNumber, uint32_t gclkVal) noexcept;

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

static inline constexpr unsigned int GetDeviceNumber(TccOutput tcc) noexcept { return ((uint8_t)tcc & 0x7F) >> 3; }
static inline constexpr unsigned int GetOutputNumber(TccOutput tcc) noexcept { return (uint8_t)tcc & 7; }

static inline constexpr GpioPinFunction GetPeriNumber(TccOutput tcc) noexcept
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

typedef AdcInput AnalogChannelNumber;			// for backwards compatibility
constexpr AnalogChannelNumber NO_ADC = AdcInput::none;

static inline constexpr unsigned int GetDeviceNumber(AdcInput ain) noexcept { return (uint8_t)ain >> 4; }
static inline constexpr unsigned int GetInputNumber(AdcInput ain) noexcept { return (uint8_t)ain & 0x0F; }

AnalogChannelNumber PinToAdcChannel(Pin p) noexcept;

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

static inline constexpr unsigned int GetDeviceNumber(SercomIo sercom) noexcept { return (uint8_t)sercom & 7; }
static inline constexpr unsigned int GetPeriNumber(SercomIo sercom) noexcept { return ((uint8_t)sercom >= 0x80) ? 3 : 2; }	// peripheral D or C

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

	PinCapability GetCapability() const noexcept { return cap; }
	const char* GetNames() const noexcept { return pinNames; }
};

// Addresses of unique ID dwords for SAME5x
constexpr uint32_t SerialNumberAddresses[4] = { 0x008061FC, 0x00806010, 0x00806014, 0x00806018 };

#include "Uart.h"
typedef Uart UARTClass;

extern Uart *serialUart0;

#include "SerialCDC.h"

extern SerialCDC *serialUSB;

#endif /* SRC_HARDWARE_SAME5X_COREIO_H_ */
