/*
 * Uart.cpp
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#include <Uart.h>

Uart::Uart(uint8_t sercomNum, uint8_t rxp, size_t numTxSlots, size_t numRxSlots) noexcept
	: sercom(Serial::GetSercom(sercomNum)), txWaitingTask(nullptr), sercomNumber(sercomNum), rxPad(rxp)
{
	txBuffer.Init(numTxSlots);
	rxBuffer.Init(numRxSlots);
}

// Initialise the UART. numRxSlots may be zero if we don't wish to receive.
void Uart::begin(uint32_t baudRate) noexcept
{
	Serial::InitUart(sercomNumber, baudRate, rxPad);
	errors.all = 0;
	sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC | SERCOM_USART_INTENSET_ERROR;

	const IRQn irqNumber = Serial::GetSercomIRQn(sercomNumber);
	NVIC_EnableIRQ(irqNumber);
	NVIC_EnableIRQ((IRQn)(irqNumber + 1));
	NVIC_EnableIRQ((IRQn)(irqNumber + 2));
	NVIC_EnableIRQ((IRQn)(irqNumber + 3));
}

void Uart::end() noexcept
{
	// Clear any received data
	rxBuffer.Clear();

	// Wait for any outstanding data to be sent
	flush();

	// Disable UART interrupt in NVIC
	const IRQn irqNumber = Serial::GetSercomIRQn(sercomNumber);
	NVIC_DisableIRQ(irqNumber);
	NVIC_DisableIRQ((IRQn)(irqNumber + 1));
	NVIC_DisableIRQ((IRQn)(irqNumber + 2));
	NVIC_DisableIRQ((IRQn)(irqNumber + 3));
}

// Non-blocking read, return 0 if no character available
int Uart::read() noexcept
{
	uint8_t c;
	return (rxBuffer.GetItem(c)) ? c : -1;
}

// Write single character, blocking
size_t Uart::write(uint8_t c) noexcept
{
	if (txBuffer.IsEmpty() && sercom->USART.INTFLAG.bit.DRE)
	{
		sercom->USART.DATA.reg = c;
	}
	else
	{
		for (;;)
		{
			if (txBuffer.PutItem(c))
			{
				sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
				break;
			}
			txWaitingTask = RTOSIface::GetCurrentTask();
			sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
			TaskBase::Take(50);
		}
	}
	return 1;
}

// Nonblocking write block
size_t Uart::TryPutBlock(const uint8_t* buffer, size_t buflen) noexcept
{
	const size_t written = txBuffer.PutBlock(buffer, buflen);
	sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
	return written;
}

// Blocking write block
size_t Uart::write(const uint8_t* buffer, size_t buflen) noexcept
{
	const size_t ret = buflen;
	for (;;)
	{
		buflen -= txBuffer.PutBlock(buffer, buflen);
		if (buflen == 0)
		{
			sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
			break;
		}
		txWaitingTask = RTOSIface::GetCurrentTask();
		sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
		TaskBase::Take(50);
	}
	return ret;
}

// Get and clear the errors
Uart::ErrorFlags Uart::GetAndClearErrors() noexcept
{
	__disable_irq();
	const ErrorFlags errs = errors;
	errors.all = 0;
	__enable_irq();
	return errs;
}

// Interrupts from the SERCOM arrive here
void Uart::Interrupt() noexcept
{
	const uint8_t status = sercom->USART.INTFLAG.reg;
	if (status & SERCOM_USART_INTFLAG_RXC)
	{
		const char c = sercom->USART.DATA.reg;
		if (rxBuffer.PutItem(c))
		{
			errors.overrun = true;
		}
	}
	if (status & SERCOM_USART_INTFLAG_ERROR)
	{
		const uint16_t stat2 = sercom->USART.STATUS.reg;
		if (stat2 & SERCOM_USART_STATUS_BUFOVF)
		{
			errors.overrun = true;
		}
		if (stat2 & SERCOM_USART_STATUS_FERR)
		{
			errors.framing = true;
		}
		sercom->USART.STATUS.reg = stat2;
		sercom->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_ERROR;			// clear the error
	}
	if (status & SERCOM_USART_INTFLAG_DRE)
	{
		uint8_t c;
		if (txBuffer.GetItem(c))
		{
			sercom->USART.DATA.reg = c;
			if (txWaitingTask != nullptr && txBuffer.SpaceLeft() >= txBuffer.GetCapacity()/2)
			{
				TaskBase::GiveFromISR(txWaitingTask);
				txWaitingTask = nullptr;
			}
		}
		else
		{
			sercom->USART.INTENCLR.reg = SERCOM_USART_INTENSET_DRE;
			if (txWaitingTask != nullptr)
			{
				TaskBase::GiveFromISR(txWaitingTask);
				txWaitingTask = nullptr;
			}
		}
	}
}

Uart::InterruptCallbackFn Uart::SetInterruptCallback(InterruptCallbackFn f) noexcept
{
	InterruptCallbackFn ret = interruptCallback;
	interruptCallback = f;
	return ret;
}

void Uart::setInterruptPriority(uint32_t priority) const noexcept
{
	const IRQn irqNumber = Serial::GetSercomIRQn(sercomNumber);
	NVIC_SetPriority(irqNumber, priority);
	NVIC_SetPriority((IRQn)(irqNumber + 1), priority);
	NVIC_SetPriority((IRQn)(irqNumber + 2), priority);
	NVIC_SetPriority((IRQn)(irqNumber + 3), priority);
}

// End
