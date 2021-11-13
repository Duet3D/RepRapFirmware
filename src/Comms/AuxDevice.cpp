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

AuxDevice::AuxDevice() noexcept : uart(nullptr), seq(0), enabled(false), raw(true)
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

void AuxDevice::Enable(uint32_t baudRate) noexcept
{
	if (uart != nullptr)
	{
		uart->begin(baudRate);
		enabled = true;
	}
}

void AuxDevice::Disable() noexcept
{
	if (enabled)
	{
		uart->end();
		outStack.ReleaseAll();
		enabled = false;
	}
}

void AuxDevice::SendPanelDueMessage(const char* msg) noexcept
{
	if (enabled)
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
	if (msg[0] != 0 && enabled)
	{
		MutexLocker lock(mutex);
		OutputBuffer *buf;
		if (OutputBuffer::Allocate(buf))
		{
			if (rawMessage || raw)
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
	if (reply == nullptr || reply->Length() == 0 || !enabled)
	{
		OutputBuffer::ReleaseAll(reply);
	}
	else
	{
		MutexLocker lock(mutex);
		if (rawMessage || raw)
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
		else if (!enabled)
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
	if (enabled)
	{
		const AsyncSerial::Errors errs = uart->GetAndClearErrors();
		reprap.GetPlatform().MessageF(mt, "Aux%u errors %u,%u,%u\n", index, (unsigned int)errs.uartOverrun, (unsigned int)errs.bufferOverrun, (unsigned int)errs.framing);
	}
}

#endif

// End
