/*
 * AuxDevice.cpp
 *
 *  Created on: 3 Sep 2020
 *      Author: David
 */

#include <Comms/AuxDevice.h>

AuxDevice::AuxDevice() noexcept : uart(nullptr), seq(0), enabled(false), raw(true)
{
}

void AuxDevice::Init(UARTClass *p_uart) noexcept
{
	uart = p_uart;
# if SAME5x
	uart->setInterruptPriority(NvicPriorityPanelDueUartRx, NvicPriorityPanelDueUartTx);
# else
	uart->setInterruptPriority(NvicPriorityPanelDueUart);
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
			buf->copy("{\"message\":");
			buf->EncodeString(msg, false);
			buf->cat("}\n");
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
				buf->printf("{\"seq\":%" PRIu32 ",\"resp\":", seq);
				buf->EncodeString(msg, true, false);
				buf->cat("}\n");
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
				uart->write(auxOutputBuffer->Read(bytesToWrite), bytesToWrite);
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

// End
