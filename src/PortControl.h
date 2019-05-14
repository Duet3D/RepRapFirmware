/*
 * PortControl.h
 *
 *  Created on: 15 Jun 2017
 *      Author: David
 */

#ifndef SRC_PORTCONTROL_H_
#define SRC_PORTCONTROL_H_

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"			// for PinConfiguration

class GCodeBuffer;

#if SUPPORT_IOBITS

class PortControl
{
public:
	PortControl();
	void Init();
	void Exit();
	void Spin(bool full);
	bool Configure(GCodeBuffer& gb, const StringRef& reply);

private:
	void UpdatePorts(IoBits_t newPortState);

	static const size_t MaxPorts = 16;		// the port bitmap is currently a 16-bit word

	IoPort portMap[MaxPorts];
	size_t numConfiguredPorts;
	unsigned int advanceMillis;
	uint32_t advanceClocks;
	IoBits_t currentPortState;
};

#endif

#endif /* SRC_PORTCONTROL_H_ */
