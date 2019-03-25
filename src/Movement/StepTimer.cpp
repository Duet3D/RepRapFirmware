/*
 * StepTimer.cpp
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#include "StepTimer.h"
#include <RTOSIface/RTOSIface.h>
#include <SoftTimer.h>
#include "Move.h"

#ifndef __LPC17xx__
# include "sam/drivers/tc/tc.h"
#endif

#if SAM4S || SAME70
// Static data used by step ISR
static volatile uint32_t stepTimerPendingStatus = 0;	// for holding status bits that we have read (and therefore cleared) but haven't serviced yet
static volatile uint32_t stepTimerHighWord = 0;			// upper 16 bits of step timer
#endif

static uint32_t nextStepInterruptScheduledAt;			// when the next interrupt is scheduled
static bool stepInterruptIsScheduled = false;			// true if an interrupt is scheduled

namespace StepTimer
{
	void Init()
	{
		// Timer interrupt for stepper motors
		// The clock rate we use is a compromise. Too fast and the 64-bit square roots take a long time to execute. Too slow and we lose resolution.
		// We choose a clock divisor of 128 which gives
		// 1.524us resolution on the Duet 085 (84MHz clock)
		// 1.067us resolution on the Duet WiFi (120MHz clock)
		// 0.853us resolution on the SAM E70 (150MHz peripheral clock)

#if __LPC17xx__
		//LPC has 32bit timers
		//Using the same 128 divisor (as also specified in DDA)
		//LPC Timers default to /4 -->  (SystemCoreClock/4)
		const uint32_t res = (VARIANT_MCK/128);						// 1.28us for 100MHz (LPC1768) and 1.067us for 120MHz (LPC1769)

		//Start a free running Timer using Match Registers 0 and 1 to generate interrupts
		LPC_SC->PCONP |= ((uint32_t) 1<<SBIT_PCTIM0);				// Ensure the Power bit is set for the Timer
		STEP_TC->MCR = 0;											// disable all MRx interrupts
		STEP_TC->PR   =  (getPclk(PCLK_TIMER0) / res) - 1;			// Set the LPC Prescaler (i.e. TC increment every 32 TimerClock Ticks)
		STEP_TC->TC  = 0x00;  										// Restart the Timer Count
		NVIC_SetPriority(STEP_TC_IRQN, NvicPriorityStep);			// set high priority for this IRQ; it's time-critical
		NVIC_EnableIRQ(STEP_TC_IRQN);
		STEP_TC->TCR  = (1 <<SBIT_CNTEN);							// Start Timer
#else
		pmc_set_writeprotect(false);
		pmc_enable_periph_clk(STEP_TC_ID);
		tc_init(STEP_TC, STEP_TC_CHAN, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_EEVT_XC0);	// must set TC_CMR_EEVT nonzero to get RB compare interrupts
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = ~(uint32_t)0;	// interrupts disabled for now
#if SAM4S || SAME70													// if 16-bit TCs
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IER = TC_IER_COVFS;	// enable the overflow interrupt so that we can use it to extend the count to 32-bits
#endif
		tc_start(STEP_TC, STEP_TC_CHAN);
		tc_get_status(STEP_TC, STEP_TC_CHAN);						// clear any pending interrupt
		NVIC_SetPriority(STEP_TC_IRQN, NvicPriorityStep);			// set priority for this IRQ
		NVIC_EnableIRQ(STEP_TC_IRQN);
#endif
	}

#if SAM4S || SAME70

	// Get the interrupt clock count, when we know that interrupts are already disabled
	// The TCs on the SAM4S and SAME70 are only 16 bits wide, so we maintain the upper 16 bits in software
	uint32_t GetInterruptClocksInterruptsDisabled()
	{
		uint32_t lowWord = STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;	// get the timer low word
		uint32_t tcsr = STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_SR;	// get the status to see whether there is an overflow
		while ((tcsr & TC_SR_COVFS) != 0)							// if the timer has overflowed
		{
			lowWord = STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;		// get the timer low word
			stepTimerHighWord += (1u << 16);
			tcsr = (tcsr & ~TC_SR_COVFS) | STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_SR;
		}

		tcsr &= STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IMR;			// clear any bits that don't generate interrupts
		if (tcsr != 0)												// if there were any other pending status bits that generate interrupts
		{
			stepTimerPendingStatus |= tcsr;							// save the other pending bits
			NVIC_SetPendingIRQ(STEP_TC_IRQN);						// set step timer interrupt pending
		}
		return (lowWord & 0x0000FFFF) | stepTimerHighWord;
	}

#else

	// Get the interrupt clock count. Despite the name, on these processors we don't need to disable interrupts before calling this.
	uint32_t GetInterruptClocksInterruptsDisabled()
	{
#if __LPC17xx__
        return STEP_TC->TC;
#else
        return STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_CV;
#endif
	}

#endif

	// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
	// On entry, interrupts must be disabled or the base priority must be <= step interrupt priority.
	/*static*/ bool ScheduleStepInterrupt(uint32_t tim)
	{
		if (stepInterruptIsScheduled && (int32_t)(tim - nextStepInterruptScheduledAt) > 0)
		{
			return false;											// an interrupt is already scheduled
		}

		// We need to disable all interrupts, because once we read the current step clock we have only 6us to set up the interrupt, or we will miss it
		const irqflags_t flags = cpu_irq_save();
		const int32_t diff = (int32_t)(tim - GetInterruptClocksInterruptsDisabled());	// see how long we have to go
		if (diff < (int32_t)DDA::MinInterruptInterval)				// if less than about 6us or already passed
		{
			cpu_irq_restore(flags);
			return true;											// tell the caller to execute the ISR instead
		}

		nextStepInterruptScheduledAt = tim;
		stepInterruptIsScheduled = true;

#ifdef __LPC17xx__
		STEP_TC->MR0 = tim;
		STEP_TC->MCR  |= (1 << SBIT_MR0I);     						// enable Int on MR0 match
#else
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_RA = tim;				// set up the compare register

		// We would like to clear any pending step interrupt. To do this, we must read the TC status register.
		// Unfortunately, this would clear any other pending interrupts from the same TC.
		// So we don't, and the step ISR must allow for getting called prematurely.
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IER = TC_IER_CPAS;		// enable the interrupt
#endif

#ifdef MOVE_DEBUG
			++numInterruptsScheduled;
			nextInterruptTime = tim;
			nextInterruptScheduledAt = GetInterruptClocksInterruptsDisabled();
#endif
		cpu_irq_restore(flags);
		return false;
	}

	// Make sure we get no step interrupts
	void DisableStepInterrupt()
	{
#ifdef __LPC17xx__
	    STEP_TC->MCR  &= ~(1<<SBIT_MR0I);								// disable Int on MR0
#else
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPAS;
# if SAM4S || SAME70
		stepTimerPendingStatus &= ~TC_SR_CPAS;
# endif
#endif
		stepInterruptIsScheduled = false;
	}

	// Schedule an interrupt at the specified clock count, or return true if that time is imminent or has passed already.
	// On entry, interrupts must be disabled or the base priority must be <= step interrupt priority.
	bool ScheduleSoftTimerInterrupt(uint32_t tim)
	{
		// We need to disable all interrupts, because once we read the current step clock we have only 6us to set up the interrupt, or we will miss it
		const irqflags_t flags = cpu_irq_save();
		const int32_t diff = (int32_t)(tim - GetInterruptClocksInterruptsDisabled());	// see how long we have to go
		if (diff < (int32_t)DDA::MinInterruptInterval)					// if less than about 6us or already passed
		{
			cpu_irq_restore(flags);
			return true;												// tell the caller to simulate an interrupt instead
		}

#ifdef __LPC17xx__
		STEP_TC->MR1 = tim; //set MR1 compare register
		STEP_TC->MCR  |= (1<<SBIT_MR1I);     // Int on MR1 match
#else
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_RB = tim;					// set up the compare register

		// We would like to clear any pending step interrupt. To do this, we must read the TC status register.
		// Unfortunately, this would clear any other pending interrupts from the same TC.
		// So we don't, and the timer ISR must allow for getting called prematurely.
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IER = TC_IER_CPBS;			// enable the interrupt
#endif

#ifdef SOFT_TIMER_DEBUG
		lastSoftTimerInterruptScheduledAt = GetInterruptClocksInterruptsDisabled();
#endif
		cpu_irq_restore(flags);
		return false;
	}

	// Make sure we get no step interrupts
	void DisableSoftTimerInterrupt()
	{
#ifdef __LPC17xx__
	    STEP_TC->MCR  &= ~(1<<SBIT_MR1I);								 // disable Int on MR1
#else
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPBS;
# if SAM4S || SAME70
		stepTimerPendingStatus &= ~TC_SR_CPBS;
# endif
#endif
	}
}

// Step pulse timer interrupt
extern "C" void STEP_TC_HANDLER() __attribute__ ((hot));

void STEP_TC_HANDLER()
{
#if SAM4S || SAME70
	// On the SAM4 we need to check for overflow whenever we read the step clock counter, and that clears the status flags.
	// So we store the un-serviced status flags.
	for (;;)
	{
		uint32_t tcsr = STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_SR | stepTimerPendingStatus;	// read the status register, which clears the status bits, and or-in any pending status bits
		tcsr &= STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IMR;			// select only enabled interrupts
		if (tcsr == 0)
		{
			break;
		}

		if ((tcsr & TC_SR_COVFS) != 0)
		{
			stepTimerHighWord += (1u << 16);
			stepTimerPendingStatus &= ~TC_SR_COVFS;
		}

		if ((tcsr & TC_SR_CPAS) != 0)								// the step interrupt uses RA compare
		{
			STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPAS;	// disable the interrupt
			stepTimerPendingStatus &= ~TC_SR_CPAS;
#ifdef MOVE_DEBUG
			++numInterruptsExecuted;
			lastInterruptTime = GetInterruptClocks();
#endif
			stepInterruptIsScheduled = false;
			reprap.GetMove().Interrupt();							// execute the step interrupt
		}

		if ((tcsr & TC_SR_CPBS) != 0)
		{
			STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPBS;	// disable the interrupt
			stepTimerPendingStatus &= ~TC_SR_CPBS;
#ifdef SOFT_TIMER_DEBUG
			++numSoftTimerInterruptsExecuted;
#endif
			SoftTimer::Interrupt();
		}
	}
#elif __LPC17xx__
	uint32_t regval = STEP_TC->IR;
	//find which Match Register triggered the interrupt
	if (regval & (1 << SBIT_MRI0_IFM)) //Interrupt flag for match channel 0.
	{
		STEP_TC->IR |= (1<<SBIT_MRI0_IFM); //clear interrupt on MR0 (setting bit will clear int)
		STEP_TC->MCR  &= ~(1<<SBIT_MR0I); //Disable Int on MR0

# ifdef MOVE_DEBUG
        ++numInterruptsExecuted;
        lastInterruptTime = GetInterruptClocks();
# endif
		stepInterruptIsScheduled = false;
		reprap.GetMove().Interrupt();                                // execute the step interrupt
	}

	if (regval & (1 << SBIT_MRI1_IFM)) //Interrupt flag for match channel 1.
	{
		STEP_TC->IR |= (1<<SBIT_MRI1_IFM); //clear interrupt
		STEP_TC->MCR  &= ~(1<<SBIT_MR1I); //Disable Int on MR1
# ifdef SOFT_TIMER_DEBUG
        ++numSoftTimerInterruptsExecuted;
# endif
		SoftTimer::Interrupt();
	}
//end __LPC17xx__

#else
	// SAM4E and SAM3X code
	uint32_t tcsr = STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_SR;		// read the status register, which clears the status bits
	tcsr &= STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IMR;				// select only enabled interrupts

	if ((tcsr & TC_SR_CPAS) != 0)									// the step interrupt uses RA compare
	{
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPAS;		// disable the interrupt
#ifdef MOVE_DEBUG
		++numInterruptsExecuted;
		lastInterruptTime = GetInterruptClocks();
#endif
		stepInterruptIsScheduled = false;
		reprap.GetMove().Interrupt();								// execute the step interrupt
	}

	if ((tcsr & TC_SR_CPBS) != 0)
	{
		STEP_TC->TC_CHANNEL[STEP_TC_CHAN].TC_IDR = TC_IER_CPBS;		// disable the interrupt
#ifdef SOFT_TIMER_DEBUG
		++numSoftTimerInterruptsExecuted;
#endif
		SoftTimer::Interrupt();
	}
#endif
}

// End
