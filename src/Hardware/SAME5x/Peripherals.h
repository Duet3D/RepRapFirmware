/*
 * SAME5x.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_H_
#define SRC_HARDWARE_SAME5X_H_

#include <cstdint>
//#include <atmel_start_pins.h>					// for GPIO_PIN_FUNCTION_x

extern "C" uint32_t SystemCoreClock;			// in system_samxxx.c
extern "C" uint32_t SystemPeripheralClock;		// in system_samxxx.c

#if defined(__SAME51N19A__)

# define SAME51		1
constexpr uint32_t SerialNumberAddresses[4] = { 0x008061FC, 0x00806010, 0x00806014, 0x00806018 };

#elif defined(__SAMC21G18A__)

# define SAMC21		1
constexpr uint32_t SerialNumberAddresses[4] = { 0x0080A00C, 0x0080A040, 0x0080A044, 0x0080A048 };

#endif

// Timer identifiers used in assigning PWM control devices
enum class TcOutput : uint8_t
{
	// TC devices, on peripheral E for both SAME51 and SAMC21
	tc0_0 = 0, tc0_1,
	tc1_0, tc1_1,
	tc2_0, tc2_1,
	tc3_0, tc3_1,
	tc4_0, tc4_1,
#ifdef SAME51
	tc5_0, tc5_1,
	tc6_0, tc6_1,
	tc7_0, tc7_1,
#endif

	none = 0xFF,
};

static inline constexpr unsigned int GetDeviceNumber(TcOutput tc) { return (uint8_t)tc >> 1; }
static inline constexpr unsigned int GetOutputNumber(TcOutput tc) { return (uint8_t)tc & 1; }

// Initialise a TC clock
void EnableTcClock(unsigned int tcNumber, uint32_t gclkVal);
void EnableTccClock(unsigned int tccNumber, uint32_t gclkVal);

enum class TccOutput : uint8_t
{
#ifdef SAME51
	// TCC devices on peripheral F
	tcc0_0F = 0x00, tcc0_1F, tcc0_2F, tcc0_3F, tcc0_4F, tcc0_5F,
	tcc1_0F = 0x08, tcc1_1F, tcc1_2F, tcc1_3F, tcc1_4F, tcc1_5F,
	tcc2_0F = 0x10, tcc2_1F, tcc2_2F, tcc2_3F, tcc2_4F, tcc2_5F,
	tcc3_0F = 0x18, tcc3_1F, tcc3_2F, tcc3_3F, tcc3_4F, tcc3_5F,
	tcc4_0F = 0x20, tcc4_1F, tcc4_2F, tcc4_3F, tcc4_4F, tcc4_5F,
	tcc5_0F = 0x28, tcc5_1F, tcc5_2F, tcc5_3F, tcc5_4F, tcc5_5F,

	// TCC devices on peripheral G
	tcc0_0G = 0x80, tcc0_1G, tcc0_2G, tcc0_3G, tcc0_4G, tcc0_5G,
	tcc1_0G = 0x88, tcc1_1G, tcc1_2G, tcc1_3G, tcc1_4G, tcc1_5G,
	tcc2_0G = 0x90, tcc2_1G, tcc2_2G, tcc2_3G, tcc2_4G, tcc2_5G,
	tcc3_0G = 0x98, tcc3_1G, tcc3_2G, tcc3_3G, tcc3_4G, tcc3_5G,
	tcc4_0G = 0xA0, tcc4_1G, tcc4_2G, tcc4_3G, tcc4_4G, tcc4_5G,
	tcc5_0G = 0xA8, tcc5_1G, tcc5_2G, tcc5_3G, tcc5_4G, tcc5_5G,
#endif

#ifdef SAMC21
	// TCC devices on peripheral E
	tcc0_0E = 0x00, tcc0_1E, tcc0_2E, tcc0_3E, tcc0_4E, tcc0_5E,
	tcc1_0E = 0x08, tcc1_1E, tcc1_2E, tcc1_3E, tcc1_4E, tcc1_5E,
	tcc2_0E = 0x10, tcc2_1E, tcc2_2E, tcc2_3E, tcc2_4E, tcc2_5E,
	// TCC devices on peripheral F
	tcc0_0F = 0x80, tcc0_1F, tcc0_2F, tcc0_3F, tcc0_4F, tcc0_5F, tcc0_6F, tcc0_7F,
	tcc1_0F = 0x88, tcc1_1F, tcc1_2F, tcc1_3F, tcc1_4F, tcc1_5F,
#endif

	none = 0xFF
};

static inline constexpr unsigned int GetDeviceNumber(TccOutput tcc) { return ((uint8_t)tcc & 0x7F) >> 3; }
static inline constexpr unsigned int GetOutputNumber(TccOutput tcc) { return (uint8_t)tcc & 7; }

static inline constexpr unsigned int GetPeriNumber(TccOutput tcc)
{
#if defined(SAME51)
	return ((uint8_t)tcc >= 0x80) ? GPIO_PIN_FUNCTION_G : GPIO_PIN_FUNCTION_F;		// peripheral G or F
#elif defined(SAMC21)
	return ((uint8_t)tcc >= 0x80) ? GPIO_PIN_FUNCTION_F : GPIO_PIN_FUNCTION_E;		// peripheral F or E
#else
# error Unsupported processor
#endif
}

// ADC input identifiers
enum class AdcInput : uint8_t
{
	adc0_0 = 0x00, adc0_1, adc0_2, adc0_3, adc0_4, adc0_5, adc0_6, adc0_7, adc0_8, adc0_9, adc0_10, adc0_11,
#if defined(SAME51)
	adc0_12, adc0_13, adc0_14, adc0_15,
	adc1_0 = 0x10, adc1_1, adc1_2, adc1_3, adc1_4, adc1_5, adc1_6, adc1_7, adc1_8, adc1_9, adc1_10, adc1_11,
#elif defined(SAMC21)
	sdadc_0 = 0x10, sdadc_1,
#endif
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
#ifdef SAME51
	sercom6c, sercom7c,
#endif

	// SERCOM pins on peripheral D
	sercom0d = 0x80,
	sercom1d, sercom2d, sercom3d, sercom4d, sercom5d,
#ifdef SAME51
	sercom6d, sercom7d,
#endif

	none = 0xFF
};

static inline constexpr unsigned int GetDeviceNumber(SercomIo sercom) { return (uint8_t)sercom & 7; }
static inline constexpr unsigned int GetPeriNumber(SercomIo sercom) { return ((uint8_t)sercom >= 0x80) ? 3 : 2; }	// peripheral D or C

// The pin description says what functions are available on each pin, filtered to avoid allocating the same function to more than one pin..
struct PinDescription
{
	TcOutput tc;
	TccOutput tcc;
	AdcInput adc;
#ifdef SAMC21
	AdcInput sdadc;
#endif
	SercomIo sercomIn;
	SercomIo sercomOut;
	uint8_t exintNumber;
	const char* pinNames;
};

#endif /* SRC_HARDWARE_SAME5X_H_ */
