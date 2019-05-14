/*
 * I2C.cpp
 *
 *  Created on: 13 May 2019
 *      Author: David
 */

#include "I2C.h"
#include "Tasks.h"

#if defined(I2C_IFACE)
static bool i2cInitialised = false;
#endif

// Initialise the I2C interface, if not already done
void I2C::Init()
{
#if defined(I2C_IFACE)
	if (!i2cInitialised)
	{
		MutexLocker lock(Tasks::GetI2CMutex());
		if (!i2cInitialised)			// test it again, now that we own the mutex
		{
			I2C_IFACE.BeginMaster(I2cClockFreq);
			i2cInitialised = true;
		}
	}
#endif
}

#if defined(I2C_IFACE) && defined(RTOS)

#include "RTOSIface/RTOSIface.h"

static TaskHandle_t twiTask = nullptr;				// the task that is waiting for a TWI command to complete

extern "C" void WIRE_ISR_HANDLER()
{
	WIRE_INTERFACE->TWI_IDR = 0xFFFFFFFF;
	if (twiTask != nullptr)
	{
		BaseType_t higherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(twiTask, &higherPriorityTaskWoken);	// wake up the task
		twiTask = nullptr;
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

uint32_t I2C::statusWaitFunc(Twi *twi, uint32_t bitsToWaitFor)
{
	uint32_t sr = twi->TWI_SR;
	if ((sr & bitsToWaitFor) == 0)
	{
		// Suspend this task until we get an interrupt indicating that a status bit that we are interested in has been set
		twiTask = xTaskGetCurrentTaskHandle();
		twi->TWI_IER = bitsToWaitFor;
		(void)TaskBase::Take(2);
		sr = twi->TWI_SR;
		twi->TWI_IDR = 0xFFFFFFFF;
	}
	return sr;
}

#endif

// End
