/*
 * CanMessageBuffer.h
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEBUFFER_H_
#define SRC_CAN_CANMESSAGEBUFFER_H_

#include "ReprapFirmware.h"

#if SUPPORT_CAN_EXPANSION

#include "CanMessageFormats.h"

// Can message buffer management
class CanMessageBuffer
{
public:
	CanMessageBuffer(CanMessageBuffer *prev) : next(prev) { }

	static void Init(unsigned int numCanBuffers);
	static CanMessageBuffer *Allocate();
	static void Free(CanMessageBuffer*& buf);
	static unsigned int FreeBuffers() { return numFree; }

	CanMessageBuffer *next;
	CanMovementMessage msg;

private:
	static CanMessageBuffer *freelist;
	static unsigned int numFree;
};

#endif

#endif /* SRC_CAN_CANMESSAGEBUFFER_H_ */
