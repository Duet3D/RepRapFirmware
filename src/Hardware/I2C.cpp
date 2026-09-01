/*
 * I2C.cpp
 *
 *  Created on: 13 May 2019
 *      Author: David
 */

#include "I2C.h"
#include <Hardware/IoPorts.h>
#include <Platform/Tasks.h>
#include <AppNotifyIndices.h>

#if defined(I2C_IFACE) && (SAM4S || SAM4E)

static bool i2cInitialised = false;

#elif defined(I2C_IFACE)

SharedI2CMaster *_ecv_null I2C::sharedI2C = nullptr;

// The bus runs on a SERCOM, described by the same I2cParameters struct that the expansion boards use
static const I2cParameters I2C0Params =
{
	.sercomNumber = I2CSercomNumber,
	.sclPin = I2CSclPin,
	.sdaPin = I2CSdaPin,
	.pinFunction = I2CPinFunction,
	.irqPriority = NvicPriorityI2C,
	.rxDmaChannel = NoDmaChannel				// M261 reads at most 34 bytes, so receive them under interrupt instead of allocating a DMA channel
};

// Ports used to reserve the bus pins so that they can't also be allocated as GPIO. These are claimed for the lifetime of the run on first use
static IoPort i2cSclPort, i2cSdaPort;

#endif

// Set up the I2C bus if not already done. Returns false and sets reply if the bus pins are not available
bool I2C::Init(const StringRef& reply) noexcept
{
#if defined(I2C_IFACE)
# if SAM4S || SAM4E
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
# else
	if (sharedI2C == nullptr)
	{
		MutexLocker lock(Tasks::GetI2CMutex());
		if (sharedI2C == nullptr)				// test it again, now that we own the mutex
		{
			// Claim the bus pins so that they can't also be used as GPIO, and fail if something else already owns them.
			// We must do this before configuring the SERCOM, so that we don't steal the pins from their current owner if they are in use
			IoPort *_ecv_from const ports[2] = { &i2cSclPort, &i2cSdaPort };
			const PinAccess access[2] = { PinAccess::read, PinAccess::read };
			if (IoPort::AssignPorts(I2CBusPinNames, reply, PinUsedBy::i2c, 2, ports, access) != 2)
			{
				return false;
			}

			SharedI2CMaster *const i2c = new SharedI2CMaster(I2C0Params);
			i2c->SetClockFrequency(I2cClockFreq);
			sharedI2C = i2c;
		}
	}
# endif
#endif
	return true;
}

// Initialise the I2C interface for callers that can't report or handle a failure (the DueX expansion on Duet 2)
void I2C::Init() noexcept
{
	String<1> dummyReply;
	(void)Init(dummyReply.GetRef());
}

#if defined(I2C_IFACE) && (SAM4S || SAM4E)

#include "RTOSIface/RTOSIface.h"

static TaskHandle _ecv_null twiTask = nullptr;			// the task that is waiting for a TWI command to complete

extern "C" void WIRE_ISR_HANDLER() noexcept
{
	WIRE_INTERFACE->TWI_IDR = 0xFFFFFFFFu;
	TaskBase::GiveFromISR(twiTask, NotifyIndices::I2C);		// wake up the task
	twiTask = nullptr;
}

uint32_t I2C::statusWaitFunc(Twi *twi, uint32_t bitsToWaitFor) noexcept
{
	bool ok = true;
	uint32_t sr;
	while (ok && ((sr = twi->TWI_SR) & bitsToWaitFor) == 0)
	{
		// Suspend this task until we get an interrupt indicating that a status bit that we are interested in has been set
		twiTask = TaskBase::GetCallerTaskHandle();
		twi->TWI_IDR = 0xFFFFFFFFu;
		twi->TWI_IER = bitsToWaitFor;
		NVIC_EnableIRQ(I2C_IRQn);
		ok = TaskBase::TakeIndexed(NotifyIndices::I2C, 2);
		twiTask = nullptr;
		twi->TWI_IDR = 0xFFFFFFFFu;
	}
	return sr;
}

#endif

// End
