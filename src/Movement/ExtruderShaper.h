/*
 * PressureAdvanceShaper.h
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_EXTRUDERSHAPER_H_
#define SRC_MOVEMENT_EXTRUDERSHAPER_H_

#include <RepRapFirmware.h>

class DDA;
class BasicPrepParams;
class MoveSegment;

// This class implements MoveSegment generation for extruders with pressure advance.
// It also tracks extrusion that has be commanded but not implemented because less than one full step has been accumulated.
// Currently it only supports linear pressure advance.
class ExtruderShaper
{
public:
	ExtruderShaper() : k(0.0), extrusionPending(0.0) /*, lastSpeed(0.0)*/ { }

	// Temporary functions until we support more sophisticated pressure advance
	float GetKclocks() const noexcept { return k; }								// get pressure advance in step clocks
	float GetKseconds() const noexcept { return k * (1.0/StepClockRate); }
	void SetKseconds(float val) noexcept { k = val * StepClockRate; }			// set pressure advance in seconds
	float GetExtrusionPending() const noexcept { return extrusionPending; }
	void SetExtrusionPending(float ep) noexcept { extrusionPending = ep; }

private:
	float k;								// the pressure advance constant in step clocks
	float extrusionPending;					// extrusion we have been asked to do but haven't because it is less than one microstep, in mm
//	float lastSpeed;						// the speed we were moving at at the end of the last extrusion, needed to implement pressure advance
};

#endif /* SRC_MOVEMENT_EXTRUDERSHAPER_H_ */
