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
	bool Configure(GCodeBuffer& gb, const StringRef& reply);

	// Functions called by DDARing
	void UpdatePorts(IoBits_t newPortState) noexcept;
	bool IsConfigured() const noexcept { return numConfiguredPorts != 0; }
	uint32_t GetAdvanceClocks() const noexcept { return advanceClocks; }

private:

	static const size_t MaxPorts = 16;		// the port bitmap is currently a 16-bit word

	IoPort portMap[MaxPorts];
	size_t numConfiguredPorts;
	unsigned int advanceMillis;
	uint32_t advanceClocks;
	IoBits_t currentPortState;
};

#endif

#endif /* SRC_PORTCONTROL_H_ */
