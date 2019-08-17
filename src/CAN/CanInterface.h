/*
 * CanInterface.h
 *
 *  Created on: 19 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANINTERFACE_H_
#define SRC_CAN_CANINTERFACE_H_

#include "RepRapFirmware.h"
#include "GCodes/GCodeResult.h"

#if SUPPORT_CAN_EXPANSION

#include <CanId.h>

class CanMessageBuffer;
class DDA;
class DriveMovement;
struct PrepParams;

namespace CanInterface
{
	static constexpr uint32_t CanResponseTimeout = 300;

	void Init();
	CanAddress GetCanAddress();
	void SendMotion(CanMessageBuffer *buf);
	GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer *buf, const StringRef& reply);
	void SendResponse(CanMessageBuffer *buf);
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
