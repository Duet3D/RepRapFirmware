/*
 * UsbDevice.cpp
 *
 *  Created on: 13 Mar 2026
 *      Author: Christian
 *
 */

#include "UsbDevice.h"

#include <Comms/AuxDevice.h>
#include <Devices.h>
#include <Platform/RepRap.h>

void UsbDevice::Init(SerialCDC *p_device, Pin vBusPin, const char *mutexName) noexcept
{
	device = p_device;
	mutex.Create(mutexName);
#if SAME5x && !CORE_USES_TINYUSB
	device->Start();
#else
	device->Start(vBusPin);
#endif
}

void UsbDevice::Shutdown() noexcept
{
	device->end();
	output.ReleaseAll();
}

void UsbDevice::Reset(Pin vBusPin) noexcept
{
	device->end();
#if SAME5x && !CORE_USES_TINYUSB
	device->Start();
#else
	device->Start(vBusPin);
#endif
}

bool UsbDevice::IsConnected() const noexcept
{
	return device != nullptr && device->IsConnected();
}

bool UsbDevice::Flush() noexcept
{
	bool hasMore = !output.IsEmpty();
	if (hasMore)
	{
		MutexLocker lock(mutex);
		OutputBuffer *_ecv_null buf = output.GetFirstItem();
		if (buf == nullptr)
		{
			(void)output.Pop();
		}
		else if (!device->IsConnected())
		{
			OutputBuffer::ReleaseAll(buf);
			(void)output.Pop();
		}
		else
		{
			const size_t bytesToWrite = min<size_t>(device->canWrite(), buf->BytesLeft());
			if (bytesToWrite != 0)
			{
				device->print(buf->Read(bytesToWrite), bytesToWrite);
			}

			if (buf->BytesLeft() == 0)
			{
				output.ReleaseFirstItem();
			}
			else
			{
				output.ApplyTimeout(UsbTimeout);
			}
		}
		hasMore = !output.IsEmpty();
	}
	return hasMore;
}

void UsbDevice::AppendReply(size_t channel, AuxMode channelMode, const GCodeBuffer *_ecv_null gb, OutputBuffer *buffer, bool rawMessage) noexcept
{
	if (!device->IsConnected())
	{
		OutputBuffer::ReleaseAll(buffer);
		seq = 0;
	}
	else
	{
		MutexLocker lock(mutex);
		if (rawMessage || channelMode == AuxMode::raw)
		{
			output.Push(buffer);
		}
		else
		{
			OutputBuffer *buf;
			if (OutputBuffer::Allocate(buf))
			{
				seq++;
				RepRap::StartJsonResponse(gb, buf);
				buf->catf("\"seq\":%" PRIu32 ",\"resp\":", seq);
				buf->EncodeReply(buffer);
				buf->cat("}\n");
				output.Push(buf);
			}
			else
			{
				OutputBuffer::ReleaseAll(buffer);
			}
		}
	}
}

void UsbDevice::SendRawMessage(size_t channel, AuxMode channelMode, const GCodeBuffer *_ecv_null gb, const char *_ecv_array message, bool rawMessage) noexcept
{
	MutexLocker lock(mutex);

	if (channelMode == AuxMode::raw || message[0] == '{' || rawMessage)
	{
		OutputBuffer *_ecv_null buf = output.GetLastItem();
		if (buf == nullptr || buf->IsReferenced())
		{
			if (OutputBuffer::Allocate(buf))
			{
				if (output.Push(buf))
				{
					buf->cat(message);
				}
			}
		}
		else
		{
			buf->cat(message);
		}
	}
	else
	{
		OutputBuffer *buf;
		if (OutputBuffer::Allocate(buf))
		{
			seq++;
			RepRap::StartJsonResponse(gb, buf);
			buf->catf("\"seq\":%" PRIu32 ",\"resp\":\"%.s\"}\n", seq, message);
			output.Push(buf);
		}
	}
}

void UsbDevice::SendBlockingMessage(const char *_ecv_array message) noexcept
{
	MutexLocker lock(mutex);
	const char *_ecv_array p = message;
	size_t len = strlen(p);
	while (device->IsConnected() && len != 0 && !reprap.SpinTimeoutImminent())
	{
		const size_t written = device->print(p, len);
		len -= written;
		p += written;
	}
}
