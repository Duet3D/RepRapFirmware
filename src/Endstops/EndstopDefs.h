/*
 * EndstopDefs.h
 *
 *  Created on: 5 Apr 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ENDSTOPDEFS_H_
#define SRC_ENDSTOPS_ENDSTOPDEFS_H_

#include <General/NamedEnum.h>

// Forward declarations
class EndstopOrZProbe;
class Endstop;
class ZProbe;

// Actions to take when an endstop is triggered. Note, these values are ordered!
enum class EndstopHitAction : uint8_t
{
	none = 0,						// don't stop anything
	stopDriver = 1,					// stop a single motor driver
	stopAxis = 2,					// stop all drivers for an axis
	stopAll = 3						// stop movement completely
};

// Struct to return info about what endstop has been triggered and what to do about it
struct EndstopHitDetails
{
	EndstopHitDetails() noexcept : action((uint32_t)EndstopHitAction::none), internalUse(0), axis(NO_AXIS), setAxisLow(false), setAxisHigh(false), isZProbe(false)
	{
	}

	void SetAction(EndstopHitAction a) noexcept { action = (uint32_t)a; }
	EndstopHitAction GetAction() const noexcept { return (EndstopHitAction)action; }

	uint16_t action : 2,			// an EndstopHitAction
			 internalUse : 4,		// used to pass the port index between CheckTriggered() and Acknowledge()
			 axis : 6,				// which axis to stop if the action is stopAxis, and which axis to set the position of if setAxisLow or SetAxisHigh is true
			 setAxisLow : 1,		// whether or not to set the axis position to its min
			 setAxisHigh : 1,		// whether or not to set the axis position to its max
			 isZProbe : 1;			// whether this is a Z probe
	DriverId driver;
};

// The values of the following enumeration must tally with the X,Y,... parameters for the M574 command
enum class EndStopPosition : unsigned int
{
	noEndStop = 0,
	lowEndStop = 1,
	highEndStop = 2,
	numPositions = 3
};

// Type of an endstop input - values must tally with the M574 command S parameter
NamedEnum
(	EndStopType, unsigned int,
	unused_wasActiveLow,
	inputPin,
	zProbeAsEndstop,
	motorStallAny,
	motorStallIndividual,
	numInputTypes
);

enum class ZProbeType : uint8_t
{
	none = 0,
	analog = 1,
	dumbModulated = 2,
	alternateAnalog = 3,
	endstopSwitch_obsolete = 4,
	digital = 5,
	e1Switch_obsolete = 6,
	zSwitch_obsolete = 7,
	unfilteredDigital = 8,
	blTouch = 9,
	zMotorStall = 10,
	numTypes = 11					// must be 1 higher than the last type
};

#endif /* SRC_ENDSTOPS_ENDSTOPDEFS_H_ */
