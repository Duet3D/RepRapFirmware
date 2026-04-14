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

// This class holds pressure advance parameters for an extruder.
class ExtruderShaper INHERIT_OBJECT_MODEL
{
public:
	ExtruderShaper() noexcept
	{
		SetParametersSimple(0.0);
	}

	float GetK0Clocks() const noexcept { return (float)k0; }								// get pressure advance in step clocks
	float GetK0Seconds() const noexcept { return (float)k0 * StepClocksToSeconds; }

	void SetParameters(const PressureAdvanceParameters& params) noexcept;
	void SetParametersSimple(float f) noexcept;

#if SUPPORT_REMOTE_COMMANDS
	void SetParameters(const ShortPressureAdvanceParameters& params) noexcept;
#endif

	bool IsActive() const noexcept { return k0 != (motioncalc_t)0.0; }
	motioncalc_t GetPressureAdvanceDistance(motioncalc_t speed) const noexcept;
	motioncalc_t GetAverageAdvanceClocks(motioncalc_t lowSpeed, motioncalc_t highSpeed, motioncalc_t steps) const noexcept
		pre(highSpeed > lowSpeed);

	void AppendParameters(const StringRef& reply) const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	// Specified parameters
	motioncalc_t k0;						// the initial pressure advance constant in step clocks
	motioncalc_t k1;						// the slope of pressure advance distance vs. speed above distance dk
	motioncalc_t dk;						// the pressure advance distance up to which k0 applies

	// Derived parameters
	motioncalc_t vk;						// the speed up to which k1 applies, equal to dk/k1
	motioncalc_t d0;						// the distance at which the k2 line intercepts the y-axis
};

#endif /* SRC_MOVEMENT_EXTRUDERSHAPER_H_ */
