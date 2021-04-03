/*
 * InputShaper.h
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_INPUTSHAPER_H_
#define SRC_MOVEMENT_INPUTSHAPER_H_

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>
#include <ObjectModel/ObjectModel.h>

class BasicPrepParams;

NamedEnum(InputShaperType, uint8_t,
	none,
	ZVD,
	ZVDD,
	EI2,
	EI3,
	DAA
);

class DDA;
class MoveSegment;

struct InputShaperPlan
{
	uint32_t accelSegments : 4,				// number of acceleration segments
			 decelSegments : 4;				// number of deceleration segments
};

class InputShaper INHERIT_OBJECT_MODEL
{
public:
	InputShaper() noexcept;

	float GetFrequency() const noexcept { return frequency; }
	float GetDamping() const noexcept { return zeta; }
	float GetDAAMinimumAcceleration() const noexcept { return minimumAcceleration; }
	InputShaperType GetType() const noexcept { return type; }
	InputShaperPlan PlanShaping(DDA& dda, BasicPrepParams& params, bool shapingEnabled) const noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M593

#if SUPPORT_REMOTE_COMMANDS
	void GetSegments(DDA& dda, const BasicPrepParams& params) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr float DefaultFrequency = 40.0;
	static constexpr float DefaultDamping = 0.1;
	static constexpr float DefaultDAAMinimumAcceleration = 10.0;

	float frequency;								// the undamped frequency
	float zeta;										// the damping ratio, see https://en.wikipedia.org/wiki/Damping. 0 = undamped, 1 = critically damped.
	float minimumAcceleration;						// the minimum value that we reduce acceleration to (DAA only)
	float halfPeriod;
	float coefficients[5];
	float times[3];
	float timeLost;
	InputShaperType type;
	uint8_t numImpulses;
};

#endif /* SRC_MOVEMENT_INPUTSHAPER_H_ */
