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

NamedEnum(LedStripType, uint8_t, DotStar, NeoPixel_RGB, NeoPixel_RGBW);

class LedStripBase INHERIT_OBJECT_MODEL
{
public:
	LedStripBase(LedStripType p_type) noexcept : type(p_type) { }
	virtual ~LedStripBase() { }

	// Configure or report on this LED strip. If pinName is not null then we are doing the initial configuration; else we are doing minor configuration or reporting.
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException) = 0;

	// Handle a M150 command addressed to this strip
	virtual GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) = 0;

	// Test whether this strip is bit-banged and therefore requires motion to be stopped before sending a command
	virtual bool MustStopMovement() const noexcept = 0;

	// Get the LED strip type
	LedStripType GetType() const noexcept { return type; }

	// Get the LED strip type as text
	const char *_ecv_array GetTypeText() const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	LedStripType type;
};

#endif

#endif /* SRC_LEDSTRIPS_LEDSTRIPBASE_H_ */
