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
		: k0(0.0), k1(0.0), dk(std::numeric_limits<float>::infinity()), vk(std::numeric_limits<float>::infinity())
	{ }

	// Temporary functions until we support more sophisticated pressure advance
	float GetK0Clocks() const noexcept { return k0; }								// get pressure advance in step clocks
	void SetParameters(const PressureAdvanceParameters& params) noexcept;

	bool IsActive() const noexcept { return k0 != 0.0; }
	motioncalc_t GetPressureAdvanceDistance(motioncalc_t speed) const noexcept;
	motioncalc_t GetAverageAdvanceClocks(motioncalc_t lowSpeed, motioncalc_t highSpeed, motioncalc_t steps) const noexcept
		pre(highSpeed > lowSpeed);

	void AppendParameters(const StringRef& reply) const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	// Specified parameters
	float k0;								// the initial pressure advance constant in step clocks
	float k1;								// the slope of pressure advance distance vs. speed above distance d
	float dk;								// the pressure advance distance up to which k applies

	// Derived parameters
	motioncalc_t vk;						// the speed up to which k1 applies, equal to dk/k1
	motioncalc_t d0;						// the distance at which the k2 line intercepts the y-axis
};

#endif /* SRC_MOVEMENT_EXTRUDERSHAPER_H_ */
