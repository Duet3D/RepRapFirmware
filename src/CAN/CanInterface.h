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
	GCodeResult SendRequestAndGetStandardReply(CanMessageBuffer *buf, const StringRef& reply);
	void SendResponse(CanMessageBuffer *buf);

	// Motor control functions
	void SendMotion(CanMessageBuffer *buf);
	void DisableRemoteDriver(DriverId driver);
	void SetRemoteDriverIdle(DriverId driver);
	void SetRemoteStandstillCurrentPercent(DriverId driver, float standstillCurrentFraction);
	void UpdateRemoteDriverCurrent(DriverId driver, float motorCurrent);
	bool SetRemoteDriverMicrostepping(DriverId driver, int microsteps, bool interp);
	GCodeResult ConfigureRemoteDriver(DriverId driver, GCodeBuffer& gb, const StringRef& reply);
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
