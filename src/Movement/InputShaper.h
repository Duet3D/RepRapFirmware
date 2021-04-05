/*
 * InputShaper.h
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_INPUTSHAPER_H_
#define SRC_MOVEMENT_INPUTSHAPER_H_

#define SUPPORT_DAA		(1)

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>
#include <ObjectModel/ObjectModel.h>

NamedEnum(InputShaperType, uint8_t,
	none,
	ZVD,
	ZVDD,
	EI2,
	EI3
#if SUPPORT_DAA
	, DAA
#endif
);

class DDA;
class BasicPrepParams;
class MoveSegment;

union InputShaperPlan
{
	struct
	{
		uint32_t shapeAccelStart : 1,
				 shapeAccelEnd : 1,
				 shapeDecelStart : 1,
				 shapeDecelEnd : 1,
				 accelSegments : 4,
				 decelSegments : 4;
	};
	uint32_t all;

	InputShaperPlan() noexcept : all(0) { }
};

class InputShaper INHERIT_OBJECT_MODEL
{
public:
	InputShaper() noexcept;

	float GetFrequency() const noexcept { return frequency; }
	float GetDamping() const noexcept { return zeta; }
	InputShaperType GetType() const noexcept { return type; }
	InputShaperPlan PlanShaping(DDA& dda, BasicPrepParams& params, bool shapingEnabled) const noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M593

#if SUPPORT_REMOTE_COMMANDS
	void GetSegments(DDA& dda, const BasicPrepParams& params) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	MoveSegment *GetAccelerationSegments(DDA& dda, BasicPrepParams& params, InputShaperPlan& plan) const noexcept;
	MoveSegment *GetDecelerationSegments(DDA& dda, BasicPrepParams& params, InputShaperPlan& plan) const noexcept;
	void FinishSegments(DDA& dda, BasicPrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept;
	float GetExtraAccelStartDistance(const DDA& dda) const noexcept;
	float GetExtraAccelEndDistance(const DDA& dda) const noexcept;
	float GetExtraDecelStartDistance(const DDA& dda) const noexcept;
	float GetExtraDecelEndDistance(const DDA& dda) const noexcept;

	static constexpr float DefaultFrequency = 40.0;
	static constexpr float DefaultDamping = 0.1;
#if SUPPORT_DAA
	static constexpr float DefaultDAAMinimumAcceleration = 10.0;
#endif

	float frequency;								// the undamped frequency
	float zeta;										// the damping ratio, see https://en.wikipedia.org/wiki/Damping. 0 = undamped, 1 = critically damped.
#if SUPPORT_DAA
	float daaMinimumAcceleration;					// the minimum value that we reduce acceleration to (DAA only)
#endif
	float coefficients[4];							// the coefficients of all the impulses, except the last which is 1
	float times[4];									// the time in seconds for the second and subsequent impulses
	float shapingTime;								// the time needed to send all the impulses, in step clocks
	float clocksLostAtStart, clocksLostAtEnd;		// the acceleration time lost due to input shaping. Multiply by 2 if shaping is used at both the start and end of acceleration.
	InputShaperType type;
	uint8_t numImpulses;
};

#endif /* SRC_MOVEMENT_INPUTSHAPER_H_ */
