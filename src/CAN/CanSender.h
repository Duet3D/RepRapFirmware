/*
 * CanSender.h
 *
 *  Created on: 20 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANSENDER_H_
#define SRC_CAN_CANSENDER_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

class CanMessageBuffer;

namespace CanSender
{
	void Init();
	void Send(CanMessageBuffer *buf);
}

#endif

#endif /* SRC_CAN_CANSENDER_H_ */
