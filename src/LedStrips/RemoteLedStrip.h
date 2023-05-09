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
	~RemoteLedStrip() override;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException) override;
	GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) override;
	bool MustStopMovement() const noexcept override { return (remoteProperties & 0x01) == 0; }

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept override { return GCodeResult::remoteInternalError; }
	GCodeResult HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept override { return GCodeResult::remoteInternalError; }
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	size_t slotNumber;
	AutoStringHandle pinNameString;
	CanAddress boardNumber;
	uint8_t remoteProperties;				// bit 0 means the remote driver uses DMA
};

#endif

#endif /* SRC_LEDSTRIPS_REMOTELEDSTRIP_H_ */
