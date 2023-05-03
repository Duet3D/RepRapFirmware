/*
 * RemoteLedStrip.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_REMOTELEDSTRIP_H_
#define SRC_LEDSTRIPS_REMOTELEDSTRIP_H_

#include "LedStripBase.h"

#if SUPPORT_LED_STRIPS && SUPPORT_CAN_EXPANSION

class RemoteLedStrip : public LedStripBase
{
public:
	RemoteLedStrip(LedStripType p_type, size_t p_slotNumber, CanAddress p_boardNumber) noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException) override;
	GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) override;
	bool IsBitBanged() const noexcept override { return (remoteProperties & 0x01) == 0; }
	void DeleteRemote() noexcept override;

private:
	size_t slotNumber;
	CanAddress boardNumber;
	uint8_t remoteProperties;
};

#endif

#endif /* SRC_LEDSTRIPS_REMOTELEDSTRIP_H_ */
