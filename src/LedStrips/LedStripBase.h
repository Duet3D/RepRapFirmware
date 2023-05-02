/*
 * LedStripBase.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_LEDSTRIPBASE_H_
#define SRC_LEDSTRIPS_LEDSTRIPBASE_H_

#include <ObjectModel/ObjectModel.h>

#if SUPPORT_LED_STRIPS

#include <Hardware/IOPorts.h>
#include <General/NamedEnum.h>

// Define which types of LED strip this hardware supports
#define SUPPORT_DMA_NEOPIXEL		(defined(DUET3_MB6HC) || defined(DUET3_MB6XD) || defined(DUET3MINI) || defined(PCCB_10))
#define SUPPORT_DMA_DOTSTAR			(defined(DUET3_MB6HC) || defined(DUET3_MB6XD) || defined(PCCB_10))
#define SUPPORT_BITBANG_NEOPIXEL	(defined(DUET3MINI_V04) || defined(DUET_NG))

NamedEnum(LedStripType, uint8_t, DotStar, NeoPixel_RGB, NeoPixel_RGBW);

class LedStripBase INHERIT_OBJECT_MODEL
{
public:
	LedStripBase(LedStripType p_type) noexcept : type(p_type) { }

	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException) = 0;
	virtual GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) = 0;
	virtual bool IsBitBanged() const noexcept = 0;

#if SUPPORT_CAN_EXPANSION
	virtual void DeleteRemote() noexcept { }			// overridden in class RemoteLedStrip
#endif

	const char *_ecv_array GetTypeText() const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	LedStripType type;
};

#endif

#endif /* SRC_LEDSTRIPS_LEDSTRIPBASE_H_ */
