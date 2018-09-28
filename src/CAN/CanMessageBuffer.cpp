/*
 * CanMessageBuffer.cpp
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#include "CanMessageBuffer.h"

#if SUPPORT_CAN_EXPANSION

#include "RTOSIface/RTOSIface.h"

CanMessageBuffer *CanMessageBuffer::freelist = nullptr;
unsigned int CanMessageBuffer::numFree = 0;

void CanMessageBuffer::Init(unsigned int numCanBuffers)
{
	freelist = nullptr;
	while (numCanBuffers != 0)
	{
		freelist = new CanMessageBuffer(freelist);
		--numCanBuffers;
		++numFree;
	}
}

CanMessageBuffer *CanMessageBuffer::Allocate()
{
	TaskCriticalSectionLocker lock;

	CanMessageBuffer *ret = freelist;
	if (ret != nullptr)
	{
		freelist = ret->next;
		--numFree;
	}
	return ret;
}

void CanMessageBuffer::Free(CanMessageBuffer*& buf)
{
	if (buf != nullptr)
	{
		TaskCriticalSectionLocker lock;
		buf->next = freelist;
		freelist = buf;
		buf = nullptr;
		++numFree;
	}
}

#endif

// End
