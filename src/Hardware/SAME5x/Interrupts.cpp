/*
 * PinInterrupts.cpp
 *
 *  Created on: 6 Jul 2019
 *      Author: David
 */

#include "Interrupts.h"

struct InterruptCallback
{
	StandardCallbackFunction func;
	CallbackParameter param;

	InterruptCallback() : func(nullptr) { }
};

// On the SAME5x we have 16 external interrupts shared between multiple pins. Only one of those pins may be programmed to generate an interrupt.
// Therefore we will have a clash if we try to attach an interrupt to two pins that use the same EXINT.
// The pin table ensures that only one pin is flagged as able to use each EXINT.
static InterruptCallback exintCallbacks[16];

void InitialisePinChangeInterrupts()
{
	hri_gclk_write_PCHCTRL_reg(GCLK, EIC_GCLK_ID, GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBAMASK_EIC_bit(MCLK);

	if (!hri_eic_is_syncing(EIC, EIC_SYNCBUSY_SWRST)) {
		if (hri_eic_get_CTRLA_reg(EIC, EIC_CTRLA_ENABLE)) {
			hri_eic_clear_CTRLA_ENABLE_bit(EIC);
			hri_eic_wait_for_sync(EIC, EIC_SYNCBUSY_ENABLE);
		}
		hri_eic_write_CTRLA_reg(EIC, EIC_CTRLA_SWRST);
	}
	hri_eic_wait_for_sync(EIC, EIC_SYNCBUSY_SWRST);

	hri_eic_write_CTRLA_CKSEL_bit(EIC, 0);				// clocked by GCLK

	// Leave NMI disabled (hri_eic_write_NMICTRL_reg)
	// Leave event control disabled (hri_eic_write_EVCTRL_reg)

	hri_eic_write_ASYNCH_reg(EIC, 0);					// all channels synchronous (needed to have debouncing or filtering)
	hri_eic_write_DEBOUNCEN_reg(EIC, 0);				// debouncing disabled

#if 0
	hri_eic_write_DPRESCALER_reg(
	    EIC,
	    (EIC_DPRESCALER_PRESCALER0(CONF_EIC_DPRESCALER0)) | (CONF_EIC_STATES0 << EIC_DPRESCALER_STATES0_Pos)
	        | (EIC_DPRESCALER_PRESCALER1(CONF_EIC_DPRESCALER1)) | (CONF_EIC_STATES1 << EIC_DPRESCALER_STATES1_Pos)
	        | CONF_EIC_TICKON << EIC_DPRESCALER_TICKON_Pos | 0);
#endif

	hri_eic_write_CONFIG_reg(EIC, 0, 0);
	hri_eic_write_CONFIG_reg(EIC, 1, 0);

	hri_eic_set_CTRLA_ENABLE_bit(EIC);
}

// Attach an interrupt to the specified pin returning true if successful
bool AttachInterrupt(Pin pin, StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param)
{
	if (pin >= ARRAY_SIZE(PinTable))
	{
		return false;			// pin number out of range
	}

	const unsigned int exintNumber = PinTable[pin].exintNumber;
	if (exintNumber >= 16)
	{
		return false;			// no EXINT available on this pin (only occurs for PA8 which is NMI)
	}

	// Configure the interrupt mode
	uint32_t modeWord;
	switch (mode)
	{
	case InterruptMode::INTERRUPT_MODE_LOW:		modeWord = EIC_CONFIG_SENSE0_LOW_Val  | EIC_CONFIG_FILTEN0; break;
	case InterruptMode::INTERRUPT_MODE_HIGH:	modeWord = EIC_CONFIG_SENSE0_HIGH_Val | EIC_CONFIG_FILTEN0; break;
	case InterruptMode::INTERRUPT_MODE_FALLING:	modeWord = EIC_CONFIG_SENSE0_FALL_Val | EIC_CONFIG_FILTEN0; break;
	case InterruptMode::INTERRUPT_MODE_RISING:	modeWord = EIC_CONFIG_SENSE0_RISE_Val | EIC_CONFIG_FILTEN0; break;
	case InterruptMode::INTERRUPT_MODE_CHANGE:	modeWord = EIC_CONFIG_SENSE0_BOTH_Val | EIC_CONFIG_FILTEN0; break;
	default:									modeWord = EIC_CONFIG_SENSE0_NONE_Val; break;
	}

	const irqflags_t flags = cpu_irq_save();
	exintCallbacks[exintNumber].func = callback;
	exintCallbacks[exintNumber].param = param;

	// Switch the pin into EIC mode
	SetPinFunction(pin, GpioPinFunction::A);		// EIC is always on peripheral A

	const unsigned int shift = (exintNumber & 7u) << 2u;
	const uint32_t mask = ~(0x0000000F << shift);

	hri_eic_clear_CTRLA_ENABLE_bit(EIC);
	hri_eic_wait_for_sync(EIC, EIC_SYNCBUSY_ENABLE);

	if (exintNumber < 8)
	{
		EIC->CONFIG[0].reg = (EIC->CONFIG[0].reg & mask) | (modeWord << shift);
	}
	else
	{
		EIC->CONFIG[1].reg = (EIC->CONFIG[1].reg & mask) | (modeWord << shift);
	}

	hri_eic_set_CTRLA_ENABLE_bit(EIC);

	// Enable interrupt
	hri_eic_set_INTEN_reg(EIC, 1ul << exintNumber);
	cpu_irq_restore(flags);

	NVIC_EnableIRQ((IRQn)(EIC_0_IRQn + exintNumber));

	return true;
}

void DetachInterrupt(Pin pin)
{
	if (pin <= ARRAY_SIZE(PinTable))
	{
		const unsigned int exintNumber = PinTable[pin].exintNumber;
		if (exintNumber < 16)
		{
			const unsigned int shift = (exintNumber & 7u) << 2u;
			const uint32_t mask = ~(0x0000000F << shift);

			hri_eic_clear_CTRLA_ENABLE_bit(EIC);
			hri_eic_wait_for_sync(EIC, EIC_SYNCBUSY_ENABLE);

			if (exintNumber < 8)
			{
				EIC->CONFIG[0].reg &= mask;
			}
			else
			{
				EIC->CONFIG[1].reg &= mask;
			}

			hri_eic_set_CTRLA_ENABLE_bit(EIC);

			// Disable the interrupt
			hri_eic_clear_INTEN_reg(EIC, 1ul << exintNumber);
			hri_eic_clear_INTFLAG_reg(EIC, 1ul << exintNumber);

			// Switch the pin out of EIC mode
			ClearPinFunction(pin);

			exintCallbacks[exintNumber].func = nullptr;
		}
	}
}

// Common EXINT handler
static inline void CommonExintHandler(size_t exintNumber)
{
	EIC->INTFLAG.reg = 1ul << exintNumber;				// clear the interrupt
	const InterruptCallback& cb = exintCallbacks[exintNumber];
	if (cb.func != nullptr)
	{
		cb.func(cb.param);
	}
}

extern "C" void EIC_0_Handler(void)
{
	CommonExintHandler(0);
}

extern "C" void EIC_1_Handler(void)
{
	CommonExintHandler(1);
}

extern "C" void EIC_2_Handler(void)
{
	CommonExintHandler(2);
}

extern "C" void EIC_3_Handler(void)
{
	CommonExintHandler(3);
}

extern "C" void EIC_4_Handler(void)
{
	CommonExintHandler(4);
}

extern "C" void EIC_5_Handler(void)
{
	CommonExintHandler(5);
}

extern "C" void EIC_6_Handler(void)
{
	CommonExintHandler(6);
}

extern "C" void EIC_7_Handler(void)
{
	CommonExintHandler(7);
}

extern "C" void EIC_8_Handler(void)
{
	CommonExintHandler(8);
}

extern "C" void EIC_9_Handler(void)
{
	CommonExintHandler(9);
}

extern "C" void EIC_10_Handler(void)
{
	CommonExintHandler(10);
}

extern "C" void EIC_11_Handler(void)
{
	CommonExintHandler(11);
}

extern "C" void EIC_12_Handler(void)
{
	CommonExintHandler(12);
}

extern "C" void EIC_13_Handler(void)
{
	CommonExintHandler(13);
}

extern "C" void EIC_14_Handler(void)
{
	CommonExintHandler(14);
}

extern "C" void EIC_15_Handler(void)
{
	CommonExintHandler(15);
}

// End
