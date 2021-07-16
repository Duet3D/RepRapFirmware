/*
 * ClosedLoop.h
 *
 *  Created on: 19 Mar 2021
 *      Author: Louis
 */

#include <RepRapFirmware.h>
#include <GCodes/GCodeException.h>

//#if SUPPORT_CAN_EXPANSION
//# include <CanId.h>
//#endif

class CanMessageClosedLoopData;

namespace ClosedLoop
{
//	GCodeResult ConfigureAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
//	GCodeResult StartAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
# if SUPPORT_CAN_EXPANSION
	GCodeResult StartDataCollection(DriverId, GCodeBuffer&, const StringRef&) THROWS(GCodeException);
	void ProcessReceivedData(CanAddress, const CanMessageClosedLoopData&, size_t, uint8_t[64]) noexcept;
# endif
}
