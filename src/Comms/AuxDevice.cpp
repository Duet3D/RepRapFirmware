/*
 * AuxDevice.cpp
 *
 *  Created on: 3 Sep 2020
 *      Author: David
 */

#include "AuxDevice.h"

#if HAS_AUX_DEVICES
#include <Platform/RepRap.h>
#include <Platform/Platform.h>

AuxDevice::AuxDevice() noexcept : uart(nullptr), seq(0), mode(AuxMode::disabled)
{
}

void AuxDevice::Init(AsyncSerial *p_uart) noexcept
{
	uart = p_uart;
#if SAME5x
	uart->setInterruptPriority(NvicPriorityAuxUartRx, NvicPriorityAuxUartTx);
#else
	uart->setInterruptPriority(NvicPriorityAuxUart);
#endif
	mutex.Create("Aux");
}

void AuxDevice::SetMode(AuxMode p_mode, uint32_t baudRate) noexcept
{
	mode = p_mode;
	if (uart != nullptr)
	{
		if (mode == AuxMode::disabled)
		{
			Disable();
		}
		else
		{
			uart->begin(baudRate);
		}
	}
}

void AuxDevice::Disable() noexcept
{
	if (mode != AuxMode::disabled)
	{
		uart->end();
		outStack.ReleaseAll();
		mode = AuxMode::disabled;
	}
}

void AuxDevice::SendPanelDueMessage(const char* msg) noexcept
{
	if (mode == AuxMode::panelDue)
	{
		OutputBuffer *buf;
		if (OutputBuffer::Allocate(buf))
		{
			buf->printf("{\"message\":\"%.s\"}\n", msg);
			outStack.Push(buf);
			Flush();
		}
	}
}

void AuxDevice::AppendAuxReply(const char *msg, bool rawMessage) noexcept
{
	// Discard this response if either no aux device is attached or if the response is empty
	if (msg[0] != 0 && IsEnabledForGCodeIo())
	{
		MutexLocker lock(mutex);
		OutputBuffer *buf;
		if (OutputBuffer::Allocate(buf))
		{
			if (rawMessage || mode == AuxMode::raw)
			{
				buf->copy(msg);
			}
			else
			{
				seq++;
				buf->printf("{\"seq\":%" PRIu32 ",\"resp\":\"%.s\"}\n", seq, msg);
			}
			outStack.Push(buf);
		}
	}
}

void AuxDevice::AppendAuxReply(OutputBuffer *reply, bool rawMessage) noexcept
{
	// Discard this response if either no aux device is attached or if the response is empty
	if (reply == nullptr || reply->Length() == 0 || !IsEnabledForGCodeIo())
	{
		OutputBuffer::ReleaseAll(reply);
	}
	else
	{
		MutexLocker lock(mutex);
		if (rawMessage || mode == AuxMode::raw)
		{
			outStack.Push(reply);
		}
		else
		{
			OutputBuffer *buf;
			if (OutputBuffer::Allocate(buf))
			{
				seq++;
				buf->printf("{\"seq\":%" PRIu32 ",\"resp\":", seq);
				buf->EncodeReply(reply);
				buf->cat("}\n");
				outStack.Push(buf);
			}
			else
			{
				OutputBuffer::ReleaseAll(reply);
			}
		}
	}
}

bool AuxDevice::Flush() noexcept
{
	bool hasMore = !outStack.IsEmpty();
	if (hasMore)
	{
		MutexLocker lock(mutex);
		OutputBuffer *auxOutputBuffer = outStack.GetFirstItem();
		if (auxOutputBuffer == nullptr)
		{
			(void)outStack.Pop();
		}
		else if (!IsEnabledForGCodeIo())
		{
			OutputBuffer::ReleaseAll(auxOutputBuffer);
			(void)outStack.Pop();
		}
		else
		{
			const size_t bytesToWrite = min<size_t>(uart->canWrite(), auxOutputBuffer->BytesLeft());
			if (bytesToWrite > 0)
			{
				uart->print(auxOutputBuffer->Read(bytesToWrite), bytesToWrite);
			}

			if (auxOutputBuffer->BytesLeft() == 0)
			{
				outStack.ReleaseFirstItem();
			}
		}
		hasMore = !outStack.IsEmpty();
	}
	return hasMore;
}

void AuxDevice::Diagnostics(MessageType mt, unsigned int index) noexcept
{
	if (mode != AuxMode::disabled)
	{
		const AsyncSerial::Errors errs = uart->GetAndClearErrors();
		reprap.GetPlatform().MessageF(mt, "Aux%u errors %u,%u,%u\n", index, (unsigned int)errs.uartOverrun, (unsigned int)errs.bufferOverrun, (unsigned int)errs.framing);
	}
}

#if SUPPORT_MODBUS_RTU

// Send some Modbus registers. May return GCodeResult notFinished if the buffer had insufficient room  but may have enough later, or the port was busy.
GCodeResult AuxDevice::SendModbusRegisters(uint8_t slaveAddress, uint16_t startRegister, uint16_t numRegisters, const uint16_t *data) noexcept
{
	//TODO
	return GCodeResult::errorNotSupported;
}

// Read some Modbus registers. May return GCodeResult notFinished if the buffer had insufficient room  but may have enough later, or the port was busy.
GCodeResult AuxDevice::ReadModbusRegisters(uint8_t slaveAddress, uint16_t startRegister, uint16_t numRegisters, uint16_t *data) noexcept
{
	//TODO
	return GCodeResult::errorNotSupported;
}

// Check whether the Modbus operation completed.
GCodeResult AuxDevice::CheckModbusResult(bool& success) noexcept
{
	//TODO
	return GCodeResult::errorNotSupported;
}

#endif

#endif

// End
