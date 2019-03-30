/*
 * LinuxInterface.cpp
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */


#include "LinuxInterface.h"
#include "DataTransfer.h"
#include "MessageFormats.h"

#include "RepRapFirmware.h"
#include "MessageFormats.h"
#include "Platform.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer/BinaryGCodeBuffer.h"

#if HAS_LINUX_INTERFACE

LinuxInterface::LinuxInterface() : transfer(new DataTransfer()), gcodeReply(new OutputStack())
{
}

void LinuxInterface::Init()
{
	transfer->Init();

	// TODO enqueue first packet: ask for macro = config.g
}

GCodeBuffer *LinuxInterface::InitGCodeBuffer()
{
	spiGCodeBuffer = new BinaryGCodeBuffer("spi", MessageType::SpiMessage, true);
	return spiGCodeBuffer;
}

void LinuxInterface::Spin()
{
	if (transfer->IsReady())
	{
		// TODO Read and write packets here
		// TODO pump code packets (header+payload) directly into spiGCodeBuffer
	}
}

void LinuxInterface::HandleGCodeReply(const char *reply)
{
	OutputBuffer *buffer = gcodeReply->GetLastItem();
	if (buffer == nullptr || buffer->IsReferenced())
	{
		if (!OutputBuffer::Allocate(buffer))
		{
			// No more space available, stop here
			return;
		}
		gcodeReply->Push(buffer);
	}

	buffer->cat(reply);
}

void LinuxInterface::HandleGCodeReply(OutputBuffer *buffer)
{
	gcodeReply->Push(buffer);
}

#endif
