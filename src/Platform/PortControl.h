/*
 * PortControl.h
 *
 *  Created on: 15 Jun 2017
 *      Author: David
 */

#ifndef SRC_PORTCONTROL_H_
#define SRC_PORTCONTROL_H_

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"

class GCodeBuffer;

#if SUPPORT_IOBITS

class PortControl
{
public:
	PortControl() noexcept;
	void Init() noexcept;
	void Exit() noexcept;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	// Functions called by DDARing
	void UpdatePorts(IoBits_t newPortState) noexcept;
	bool IsConfigured() const noexcept { return numConfiguredPorts != 0; }
	uint32_t GetAdvanceClocks() const noexcept { return advanceClocks; }

private:

	static const size_t MaxIoBitsPorts = 16;		// the port bitmap is currently a 16-bit word

	uint8_t gpOutPortNumbers[MaxIoBitsPorts];
	size_t numConfiguredPorts;
	uint32_t advanceMillis;
	uint32_t advanceClocks;
	IoBits_t currentPortState;
};

#endif

#endif /* SRC_PORTCONTROL_H_ */
