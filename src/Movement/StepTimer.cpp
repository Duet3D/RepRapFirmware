/*
 * StepTimer.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "StepTimer.h"
#include <RTOSIface/RTOSIface.h>
#include "Move.h"

#ifndef __LPC17xx__
# include "sam/drivers/tc/tc.h"
#endif

StepTimer * volatile StepTimer::pendingList = nullptr;

void StepTimer::Init() noexcept
{
	// Timer interrupt for stepper motors
	// The clock rate we use is a compromise. Too fast and the 64-bit square roots take a long time to execute. Too slow and we lose resolution.
	// On Duet WiFi/Ethernet, Duet Maestro and legacy Duets we use a clock prescaler of 128 which gives
	// 1.524us resolution on the Duet 085 (84MHz clock)
	// 1.067us resolution on the Duet WiFi/Ethernet/Maestro (120MHz clock)
	// On Duet 3 we need a step clock rate that can be programmed on SAME70, SAME5x and SAMC21 processors. We choose 750kHz (1.333us resolution)

#ifdef __LPC17xx__
	//LPC has 32bit timers with 32bit prescalers
	//Start a free running Timer using Match Registers to generate interrupts

	// Setup the Prescaler such that every TC increment is equal to 1/StepClockRate
	// The Prescale counter is incremented every Timer PCLK. When the Prescale counter reaches the value in PR+1, TC is then incremented
	// Timer PCLK defaults to SystemCoreClock/4 on boot.
	// Using a StepClockRate of 1MHz gives PR values of exactly 29 and 24 for the 1769 and 1786 respectively
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_TIMER0);              // Enable power and clocking
	STEP_TC->MCR = 0;											    // Disable all MRx interrupts
	STEP_TC->PR = (getPclk(PCLK_TIMER0) / StepClockRate) - 1;	    // Set the Prescaler
	STEP_TC->TC = 0x00;  										    // Restart the Timer Count
	NVIC_SetPriority(STEP_TC_IRQN, NvicPriorityStep);			    // Set the priority for this IRQ
	NVIC_EnableIRQ(STEP_TC_IRQN);
	STEP_TC->TCR = (1 <<SBIT_CNTEN);							    // Start Timer
#else
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk(STEP_TC_ID);

# if SAME70 || SAM4S
	// These processors have 16-bit TCs but we can chain 2 of them together
	pmc_enable_periph_clk(STEP_TC_ID_UPPER);

#  if SAME70
	// Step clock runs at 48MHz/64 for compatibility with the Tool board
	constexpr uint32_t divisor = (64ull * VARIANT_MCK)/(48000000u);
	static_assert(divisor <= 256 && divisor >= 100);

	// TC0 can use either PCLK6 or PCLK7 depending on the setting in the bus matrix Peripheral Clock Configuration Register. Default is PCLK6.
	pmc_disable_pck(PMC_PCK_6);
	pmc_switch_pck_to_mck(PMC_PCK_6, PMC_PCK_PRES(divisor - 1));
	pmc_enable_pck(PMC_PCK_6);

	// Chain TC0 and TC2 together. TC0 provides the lower 16 bits, TC2 the upper 16 bits. CLOCK1 is PCLK6 or PCLK7.
	tc_init(STEP_TC, STEP_TC_CHAN, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR | TC_CMR_EEVT_XC0);	// must set TC_CMR_EEVT nonzero to get RB compare interrupts
	tc_init(STEP_TC, STEP_TC_CHAN_UPPER, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_BURST_XC2);
	tc_set_block_mode(STEP_TC, TC_BMR_TC2XC2S_TIOA0);
#  elif SAM4S
	// Chain TC0 and TC2 together. TC0 provides the lower 16 bits, TC2 the upper 16 bits. CLOCK4 is MCLK/128.
	tc_init(STEP_TC, STEP_TC_CHAN, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR | TC_CMR_EEVT_XC0);	// must set TC_CMR_EEVT nonzero to get RB compare interrupts
	tc_init(STEP_TC, STEP_TC_CHAN_UPPER, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_BURST_XC2);
	tc_set_block_mode(STEP_TC, TC_BMR_TC2XC2S_TIOA0);
#  endif

	// Multiple sources claim there is a bug in both SAM4E and SAME70: the first time that the lower counter wraps round, the upper counter doesn't increment.
	// Workaround from https://www.at91.com/viewtopic.php?t=24000: set up TC0 to generate an output pulse almost immediately
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_RA = 0x0001;
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_RC = 0x0002;

	cpu_irq_disable();
	tc_start(STEP_TC, STEP_TC_CHAN_UPPER);
	tc_start(STEP_TC, STEP_TC_CHAN);

	// Wait until first (lost) pulse is generated, then reset compare trip to TC0 wrap
	while (STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV < 0x0002) { }

	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_RA = 0xFFFF;
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_RC = 0;
	cpu_irq_enable();

# else
	// Use a single 32-bit timer. CLOCK4 is MCLK/128.
	tc_init(STEP_TC, STEP_TC_CHAN, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_EEVT_XC0);	// must set TC_CMR_EEVT nonzero to get RB compare interrupts
	tc_start(STEP_TC, STEP_TC_CHAN);
# endif

	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = ~(uint32_t)0;	// interrupts disabled for now
	tc_get_status(STEP_TC, STEP_TC_CHAN);						// clear any pending interrupt
	NVIC_SetPriority(STEP_TC_IRQN, NvicPriorityStep);			// set priority for this IRQ
	NVIC_EnableIRQ(STEP_TC_IRQN);
#endif
}

#if SAM4S || SAME70

// Get the interrupt clock count. The TCs on the SAM4S and SAME70 are only 16 bits wide, so we maintain the upper 16 bits in a chained counter.
/*static*/ uint32_t StepTimer::GetTimerTicks() noexcept
{
	uint16_t highWord = STEP_TC->TC_CHANNEL[STEP_TC_CHAN_UPPER].TC_CV;		// get the timer high word
	do
	{
		const uint16_t lowWord = STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;	// get the timer low word
		const uint16_t highWordAgain = STEP_TC->TC_CHANNEL[STEP_TC_CHAN_UPPER].TC_CV;
		if (highWordAgain == highWord)
		{
			return ((uint32_t)highWord << 16) | lowWord;
		}
		highWord = highWordAgain;
	} while (true);
}

#endif

// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
// On entry, interrupts must be disabled or the base priority must be <= step interrupt priority.
bool StepTimer::ScheduleTimerInterrupt(uint32_t tim) noexcept
{
	// We need to disable all interrupts, because once we read the current step clock we have only 6us to set up the interrupt, or we will miss it
	const irqflags_t flags = cpu_irq_save();
	const int32_t diff = (int32_t)(tim - GetTimerTicks());			// see how long we have to go
	if (diff < (int32_t)MinInterruptInterval)						// if less than about 6us or already passed
	{
		cpu_irq_restore(flags);
		return true;												// tell the caller to simulate an interrupt instead
	}

#ifdef __LPC17xx__
	STEP_TC->MR[0] = tim;											// set MR0 compare register
	STEP_TC->MCR |= (1u<<SBIT_MR0I);									// enable interrupt on MR0 match
#else
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_RB = tim;					// set up the compare register
	(void)STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_SR;					// read the status register, which clears the status bits and any pending interrupt
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IER = TC_IER_CPBS;			// enable the interrupt
#endif

	cpu_irq_restore(flags);
	return false;
}

// Make sure we get no timer interrupts
void StepTimer::DisableTimerInterrupt() noexcept
{
#ifdef __LPC17xx__
	STEP_TC->MCR &= ~(1u<<SBIT_MR0I);								 // disable Int on MR1
#else
	STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPBS;
#endif
}

// The guts of the ISR
/*static*/ void StepTimer::Interrupt() noexcept
{
	StepTimer * tmr = pendingList;
	if (tmr != nullptr)
	{
		for (;;)
		{
			pendingList = tmr->next;									// remove it from the pending list
			tmr->active = false;
			tmr->callback(tmr->cbParam);								// execute its callback. This may schedule another callback and hence change the pending list.

			tmr = pendingList;
			if (tmr == nullptr)
			{
				break;
			}

			if (!StepTimer::ScheduleTimerInterrupt(tmr->whenDue))
			{
				break;													// interrupt isn't due yet and a new one has been scheduled
			}
		}
	}
}

// Step pulse timer interrupt
extern "C" void STEP_TC_HANDLER() noexcept __attribute__ ((hot));

void STEP_TC_HANDLER() noexcept
{
#if defined(__LPC17xx__)
	uint32_t regval = STEP_TC->IR;
	//find which Match Register triggered the interrupt
	if (regval & (1u << SBIT_MRI0_IFM))								// Interrupt flag for match channel 1.
	{
		STEP_TC->IR |= (1u<<SBIT_MRI0_IFM);							// clear interrupt
		STEP_TC->MCR  &= ~(1u<<SBIT_MR0I);							// Disable Int on MR0
#else
	// ATSAM processor code
	uint32_t tcsr = STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_SR;		// read the status register, which clears the status bits
	tcsr &= STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IMR;				// select only enabled interrupts

	if ((tcsr & TC_SR_CPBS) != 0)									// the timer interrupt uses RB compare
	{
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPBS;		// disable the interrupt
#endif

#ifdef TIMER_DEBUG
		++numTimerInterruptsExecuted;
#endif
		StepTimer::Interrupt();
	}
}

StepTimer::StepTimer() noexcept : next(nullptr), callback(nullptr), active(false)
{
}

// Set up the callback function and parameter
void StepTimer::SetCallback(TimerCallbackFunction cb, CallbackParameter param) noexcept
{
	callback = cb;
	cbParam = param;
}

// Schedule a callback at a particular tick count, returning true if it was not scheduled because it is already due or imminent.
bool StepTimer::ScheduleCallbackFromIsr(Ticks when) noexcept
{
	whenDue = when;
	return ScheduleCallbackFromIsr();
}

bool StepTimer::ScheduleCallbackFromIsr() noexcept
{

	// Optimise the common case i.e. no other timer is pending
	if (pendingList == nullptr)
	{
		if (ScheduleTimerInterrupt(whenDue))
		{
			return true;
		}
		next = nullptr;
		pendingList = this;
		active = true;
		return false;
	}

	// Another timer (or possibly this one) is already pending
	if (active)
	{
		CancelCallbackFromIsr();
	}

	const Ticks now = GetTimerTicks();
	const int32_t howSoon = (int32_t)(whenDue - now);
	StepTimer** ppst = const_cast<StepTimer**>(&pendingList);
	if (howSoon < (int32_t)((*ppst)->whenDue - now))
	{
		// This one is due earlier than the first existing one
		if (ScheduleTimerInterrupt(whenDue))
		{
			return true;
		}
	}
	else
	{
		while (*ppst != nullptr && (int32_t)((*ppst)->whenDue - now) < howSoon)
		{
			ppst = &((*ppst)->next);
		}
	}

	next = *ppst;
	*ppst = this;
	active = true;
	return false;
}

bool StepTimer::ScheduleCallback(Ticks when) noexcept
{
	const uint32_t baseprio = ChangeBasePriority(NvicPriorityStep);
	const bool rslt = ScheduleCallbackFromIsr(when);
	RestoreBasePriority(baseprio);
	return rslt;
}

// Cancel any scheduled callback for this timer. Harmless if there is no callback scheduled.
void StepTimer::CancelCallbackFromIsr() noexcept
{
	for (StepTimer** ppst = const_cast<StepTimer**>(&pendingList); *ppst != nullptr; ppst = &((*ppst)->next))
	{
		if (*ppst == this)
		{
			*ppst = this->next;		// unlink this from the pending list
			break;
		}
	}
	active = false;
}

void StepTimer::CancelCallback() noexcept
{
	const uint32_t baseprio = ChangeBasePriority(NvicPriorityStep);
	CancelCallbackFromIsr();
	RestoreBasePriority(baseprio);
}

// End
