/*
 * CanInterface.h
 *
 *  Created on: 19 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANINTERFACE_H_
#define SRC_CAN_CANINTERFACE_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

#include <CanId.h>

class CanMessageBuffer;
class DDA;
class DriveMovement;
struct PrepParams;

namespace CanInterface
{
	void Init();
	CanAddress GetCanAddress();
	void SendMotion(CanMessageBuffer *buf);
	void SendRequest(CanMessageBuffer *buf);
	void SendResponse(CanMessageBuffer *buf);
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
