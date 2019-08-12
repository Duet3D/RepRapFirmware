/*
 * CommandProcessor.cpp
 *
 *  Created on: 12 Aug 2019
 *      Author: David
 */

#include "CommandProcessor.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMessageBuffer.h"

// Process a received broadcast or request message amd free the message buffer
void CommandProcessor::ProcessReceivedMessage(CanMessageBuffer *buf)
{
	//TODO
	buf->DebugPrint("Rec: ");
	CanMessageBuffer::Free(buf);
}

#endif

// End
