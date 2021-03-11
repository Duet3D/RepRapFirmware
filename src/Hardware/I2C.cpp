/*
 * I2C.cpp
 *
 *  Created on: 13 May 2019
 *      Author: David
 */

#include "I2C.h"
#include <Platform/Tasks.h>

#if defined(I2C_IFACE)
static bool i2cInitialised = false;
#endif

// Initialise the I2C interface, if not already done
void I2C::Init() noexcept
{
#if defined(I2C_IFACE)
	if (!i2cInitialised)
	{
		MutexLocker lock(Tasks::GetI2CMutex());
		if (!i2cInitialised)					// test it again, now that we own the mutex
		{
			NVIC_SetPriority(I2C_IRQn, NvicPriorityTwi);	// we use I2C to talk to the DueX before Platform::InitialiseInterrupts is called, so need to do this here
			I2C_IFACE.BeginMaster(I2cClockFreq);
			i2cInitialised = true;
		}
	}
#endif
}

#if defined(I2C_IFACE)

#include "RTOSIface/RTOSIface.h"

static TaskHandle twiTask = nullptr;			// the task that is waiting for a TWI command to complete

extern "C" void WIRE_ISR_HANDLER() noexcept
{
	WIRE_INTERFACE->TWI_IDR = 0xFFFFFFFF;
	TaskBase::GiveFromISR(twiTask);				// wake up the task
	twiTask = nullptr;
}

uint32_t I2C::statusWaitFunc(Twi *twi, uint32_t bitsToWaitFor) noexcept
{
	bool ok = true;
	uint32_t sr = twi->TWI_SR;
	while (ok && (sr & bitsToWaitFor) == 0)
	{
		// Suspend this task until we get an interrupt indicating that a status bit that we are interested in has been set
		twiTask = TaskBase::GetCallerTaskHandle();
		twi->TWI_IDR = 0xFFFFFFFF;
		twi->TWI_IER = bitsToWaitFor;
		NVIC_EnableIRQ(I2C_IRQn);
		ok = TaskBase::Take(2);
		twiTask = nullptr;
		twi->TWI_IDR = 0xFFFFFFFF;
		sr = twi->TWI_SR;
	}
	return sr;
}

#endif

// End
