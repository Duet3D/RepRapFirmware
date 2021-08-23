/*
 * ClosedLoop.h
 *
 *  Created on: 19 Mar 2021
 *      Author: Louis
 */

#include <RepRapFirmware.h>
#include <GCodes/GCodeException.h>

class CanMessageClosedLoopData;

namespace ClosedLoop
{
#if SUPPORT_CAN_EXPANSION
	GCodeResult StartDataCollection(DriverId, GCodeBuffer&, const StringRef&) THROWS(GCodeException);
	void ProcessReceivedData(CanAddress, const CanMessageClosedLoopData&, size_t) noexcept;
#endif
}
