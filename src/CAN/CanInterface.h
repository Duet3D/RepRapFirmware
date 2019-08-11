/*
 * CanInterface.h
 *
 *  Created on: 19 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANINTERFACE_H_
#define SRC_CAN_CANINTERFACE_H_

#include "RepRapFirmware.h"
#include <CanId.h>

class CanMessageBuffer;

#if SUPPORT_CAN_EXPANSION

class DDA;
class DriveMovement;
struct PrepParams;

namespace CanInterface
{
	void Init(CanAddress pBoardAddress);
	CanAddress GetCanAddress();
	void Send(CanMessageBuffer *buf);
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
