/*
 * CommandProcessor.h
 *
 *  Created on: 12 Aug 2019
 *      Author: David
 */

#ifndef SRC_CAN_COMMANDPROCESSOR_H_
#define SRC_CAN_COMMANDPROCESSOR_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

class CanMessageBuffer;

namespace CommandProcessor
{
	void ProcessReceivedMessage(CanMessageBuffer *buf) noexcept;	// Process a received broadcast or request message and free the message buffer
	void AppendBadMotionStats(const StringRef& reply) noexcept;		// Append diagnostics relating to bad motion messages
}

#endif

#endif /* SRC_CAN_COMMANDPROCESSOR_H_ */
