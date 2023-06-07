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

#include <Hardware/IoPorts.h>
#include <General/NamedEnum.h>

#if SUPPORT_REMOTE_COMMANDS
class CanMessageGenericParser;
#endif

class LedStripBase INHERIT_OBJECT_MODEL
{
public:
	LedStripBase(LedStripType p_type) noexcept : type(p_type) { }
	virtual ~LedStripBase() { }

	// Configure or report on this LED strip. If pinName is not null then we are doing the initial configuration; else we are doing minor configuration or reporting.
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array pinName) THROWS(GCodeException) = 0;

	// Handle a M150 command addressed to this strip
	virtual GCodeResult HandleM150(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException) = 0;

#if SUPPORT_REMOTE_COMMANDS
	// Handle a configuration request received over CAN
	virtual GCodeResult Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept = 0;

	// Handle a M150 command addressed received over CAN
	virtual GCodeResult HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept = 0;
#endif

	// Test whether this strip is bit-banged and therefore requires motion to be stopped before sending a command
	virtual bool MustStopMovement() const noexcept = 0;

	// Get the LED strip type
	LedStripType GetType() const noexcept { return type; }

	// Return true if the LED strip type is NeoPixel
	bool IsNeoPixel() const noexcept { return type != LedStripType::DotStar; }

	// Get the LED strip type as text
	const char *_ecv_array GetTypeText() const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	LedStripType type;
};

#endif

#endif /* SRC_LEDSTRIPS_LEDSTRIPBASE_H_ */
