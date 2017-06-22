/*
 * PortControl.h
 *
 *  Created on: 15 Jun 2017
 *      Author: David
 */

#ifndef SRC_PORTCONTROL_H_
#define SRC_PORTCONTROL_H_

#include "RepRapFirmware.h"

class GCodeBuffer;

#if SUPPORT_IOBITS

typedef uint16_t IoBits_t;

class PortControl
{
public:
	PortControl();
	void Init();
	void Exit();
	void Spin(bool full);
	bool Configure(GCodeBuffer& gb, StringRef& reply);

private:
	void UpdatePorts(IoBits_t newPortState);

	static const size_t MaxPorts = 16;		// the port bitmap is currently a 16-bit word
	static const uint16_t NoPort = 0xFFFF;

	struct PortMapEntry
	{
		uint16_t logicalPort;
		Pin pin;
		bool invert;
	};
	static_assert((sizeof(PortMapEntry) & (sizeof(PortMapEntry) - 1)) == 0, "PortMapEntry is not an efficient size for array inndexing");

	PortMapEntry portMap[MaxPorts];
	size_t numConfiguredPorts;
	unsigned int advanceMillis;
	uint32_t advanceClocks;
	IoBits_t currentPortState;
};

#endif

#endif /* SRC_PORTCONTROL_H_ */
