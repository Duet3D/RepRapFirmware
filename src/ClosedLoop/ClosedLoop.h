/*
 * ClosedLoop.h
 *
 *  Created on: 19 Mar 2021
 *      Author: Louis
 */

#ifndef SRC_CLOSEDLOOP_CLOSEDLOOP_H
#define SRC_CLOSEDLOOP_CLOSEDLOOP_H

#include <RepRapFirmware.h>

#if SUPPORT_CAN_EXPANSION
# include <GCodes/GCodeException.h>

class CanMessageClosedLoopData;

namespace ClosedLoop
{
	GCodeResult StartDataCollection(DriverId id, GCodeBuffer&, const StringRef&) THROWS(GCodeException) pre(id.IsRemote());
	void ProcessReceivedData(CanAddress src, const CanMessageClosedLoopData& msg, size_t msgLen) noexcept;
}

#endif

#endif	// SRC_CLOSEDLOOP_CLOSEDLOOP_H
