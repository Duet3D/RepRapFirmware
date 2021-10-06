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
	GCodeResult StartDataCollection(DriverId, GCodeBuffer&, const StringRef&) THROWS(GCodeException);
	void ProcessReceivedData(CanAddress, const CanMessageClosedLoopData&, size_t) noexcept;
}

#endif

#endif	// SRC_CLOSEDLOOP_CLOSEDLOOP_H
