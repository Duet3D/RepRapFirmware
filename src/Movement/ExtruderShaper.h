/*
 * PressureAdvanceShaper.h
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_EXTRUDERSHAPER_H_
#define SRC_MOVEMENT_EXTRUDERSHAPER_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include <limits>

// This class implements MoveSegment generation for extruders with pressure advance.
// It also tracks extrusion that has been commanded but not implemented because less than one full step has been accumulated.
// Currently it only supports linear pressure advance.
class ExtruderShaper INHERIT_OBJECT_MODEL
{
public:
	ExtruderShaper() noexcept
		: k1(0.0), k2(0.0), dk(std::numeric_limits<float>::infinity()), vk(std::numeric_limits<float>::infinity())
	{ }

	// Temporary functions until we support more sophisticated pressure advance
	float GetK1Clocks() const noexcept { return k1; }								// get pressure advance in step clocks
	float GetK2Clocks() const noexcept { return k2; }								// get pressure advance in step clocks
	void SetParameters(const PressureAdvanceParameters& params) noexcept;

	bool IsActive() const noexcept { return k1 != 0.0; }
	float GetPressureAdvanceDistance(float speed) const noexcept;

	void AppendParameters(const StringRef& reply) const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	// Specified parameters
	float k1;								// the initial pressure advance constant in step clocks
	float k2;								// the slope of pressure advance distance vs. speed above distance d
	float dk;								// the pressure advance distance up to which k applies

	// Derived parameters
	float vk;								// the speed up to which k1 applies, equal to dk/k1
	float d0;								// the distance at which the k2 line intercepts the y-axis
};

#endif /* SRC_MOVEMENT_EXTRUDERSHAPER_H_ */
