/*
 * PressureAdvanceShaper.h
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_EXTRUDERSHAPER_H_
#define SRC_MOVEMENT_EXTRUDERSHAPER_H_

class DDA;
class BasicPrepParams;
class MoveSegment;

class ExtruderShaper
{
public:
	ExtruderShaper() : k(0.0), extrusionPending(0.0), lastSpeed(0.0) { }

	// Temporary functions until we support more sophisticated pressure advance
	float GetK() const noexcept { return k; }
	void SetK(float val) noexcept { k = val; }

	MoveSegment *GetSegments(const DDA& dda, const BasicPrepParams& params, float extrusionAmount) const noexcept;

private:
	float k;								// the pressure advance constant
	float extrusionPending;					// extrusion we have been asked to do but haven't because it is less than one microstep
	float lastSpeed;						// the speed we were moving at at the end of the last extrusion
};

#endif /* SRC_MOVEMENT_EXTRUDERSHAPER_H_ */
