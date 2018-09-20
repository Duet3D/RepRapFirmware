/*
 * CanSender.cpp
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#include "CanSender.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMessageBuffer.h"
#include "Movement/StepTimer.h"
#include "RTOSIface/RTOSIface.h"

// TMC51xx management task
constexpr size_t CanSenderTaskStackWords = 400;
static Task<CanSenderTaskStackWords> canSenderTask;

static CanMessageBuffer *pendingBuffers;
static CanMessageBuffer *lastBuffer;			// only valid when pendingBuffers != nullptr

extern "C" void CanSenderLoop(void *)
{
	for (;;)
	{
		TaskBase::Take(Mutex::TimeoutUnlimited);
		while (pendingBuffers != nullptr)
		{
			CanMessageBuffer *buf;
			{
				TaskCriticalSectionLocker lock;
				buf = pendingBuffers;
				pendingBuffers = buf->next;
			}

			// TODO actually send the message
			buf->msg.timeNow = StepTimer::GetInterruptClocks();
			buf->msg.DebugPrint();
			CanMessageBuffer::Free(buf);
		}
	}
}

void CanSender::Init()
{
	pendingBuffers = nullptr;
	canSenderTask.Create(CanSenderLoop, "CanSender", nullptr, TaskBase::CanSenderPriority);
}

// Add a buffer to the end of the send queue
void CanSender::Send(CanMessageBuffer *buf)
{
	buf->next = nullptr;
	TaskCriticalSectionLocker lock;

	if (pendingBuffers == nullptr)
	{
		pendingBuffers = lastBuffer = buf;
	}
	else
	{
		lastBuffer->next = buf;
	}
	canSenderTask.Give();
}

#endif

// End
