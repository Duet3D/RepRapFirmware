/*
 * InputShaper.h
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_AXISSHAPER_H_
#define SRC_MOVEMENT_AXISSHAPER_H_

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>
#include <ObjectModel/ObjectModel.h>
#include <InputShaperPlan.h>

// These names must be in alphabetical order and lowercase
NamedEnum(InputShaperType, uint8_t,
	custom,
	ei2,
	ei3,
	mzv,
	none,
	zvd,
	zvdd,
	zvddd,
);

namespace InputShapingDebugFlags
{
	// Bit numbers in the input shaping debug bitmap
	constexpr unsigned int Errors = 0;
	constexpr unsigned int Retries = 1;
	constexpr unsigned int All = 2;
}

class DDA;
class PrepParams;
class MoveSegment;
struct AccelOrDecelPlan;

#if SUPPORT_REMOTE_COMMANDS
struct CanMessageSetInputShaping;
struct CanMessageMovementLinear;
#endif

class AxisShaper INHERIT_OBJECT_MODEL
{
public:
	AxisShaper() noexcept;

	// Configure input shaping
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M593

#if SUPPORT_REMOTE_COMMANDS
	// Handle a request from the master board to set input shaping parameters
	GCodeResult EutSetInputShaping(const CanMessageSetInputShaping& msg, size_t dataLength, const StringRef& reply) noexcept;

	// Calculate the shaped segments for a move
	void GetRemoteSegments(DDA& dda, PrepParams& params) const noexcept;
#endif

	// Plan input shaping for an individual move
	void PlanShaping(DDA& dda, PrepParams& params, bool shapingEnabled) noexcept;

	// Calculate the move segments when input shaping is not used
	static MoveSegment *GetUnshapedSegments(DDA& dda, const PrepParams& params) noexcept;

	void Diagnostics(MessageType mtype) noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	void CalculateDerivedParameters() noexcept;
	MoveSegment *GetAccelerationSegments(const DDA& dda, PrepParams& params) const noexcept;
	MoveSegment *GetDecelerationSegments(const DDA& dda, PrepParams& params) const noexcept;
	MoveSegment *FinishShapedSegments(const DDA& dda, const PrepParams& params, MoveSegment *accelSegs, MoveSegment *decelSegs) const noexcept;
	float GetExtraAccelStartDistance(float startSpeed, float acceleration) const noexcept;
	float GetExtraAccelEndDistance(float topSpeed, float acceleration) const noexcept;
	float GetExtraDecelStartDistance(float topSpeed, float deceleration) const noexcept;
	float GetExtraDecelEndDistance(float endSpeed, float deceleration) const noexcept;

	// New input shaping functions
	void ProposeShapeAccelEnd(const DDA& dda, const PrepParams& params, AccelOrDecelPlan& proposal) const noexcept;
	void ProposeShapeAccelBoth(const DDA& dda, const PrepParams& params, AccelOrDecelPlan& proposal) const noexcept;
	void ProposeShapeDecelStart(const DDA& dda, const PrepParams& params, AccelOrDecelPlan& proposal) const noexcept;
	void ProposeShapeDecelBoth(const DDA& dda, const PrepParams& params, AccelOrDecelPlan& proposal) const noexcept;
	void ImplementAccelShaping(const DDA& dda, PrepParams& params, const AccelOrDecelPlan& proposal) const noexcept;
	void ImplementDecelShaping(const DDA& dda, PrepParams& params, const AccelOrDecelPlan& proposal) const noexcept;

	bool TryReduceTopSpeedFullyShapeBoth(const DDA& dda, const PrepParams& params, float& newTopSpeed, InputShaperPlan& plan) const noexcept;
	bool TryReduceTopSpeedFullyShapeAccel(const DDA& dda, const PrepParams& params, float& newTopSpeed, InputShaperPlan& plan) const noexcept;
	bool TryReduceTopSpeedFullyShapeDecel(const DDA& dda, const PrepParams& params, float& newTopSpeed, InputShaperPlan& plan) const noexcept;
	bool TryReduceTopSpeedFullyShapeNeither(const DDA& dda, const PrepParams& params, float& newTopSpeed, InputShaperPlan& plan) const noexcept;

	static constexpr unsigned int MaxExtraImpulses = 4;
	static constexpr float DefaultFrequency = 40.0;
	static constexpr float DefaultDamping = 0.1;
	static constexpr float DefaultReductionLimit = 0.25;
	static constexpr float MinimumMiddleSegmentTime = 5.0/1000.0;	// minimum length of the segment between shaped start and shaped end of an acceleration or deceleration

	// Input shaping parameters input by the user
	InputShaperType type;								// the type of the input shaper, from which we can find its name
	float frequency;									// the undamped frequency in Hz
	float zeta;											// the damping ratio, see https://en.wikipedia.org/wiki/Damping. 0 = undamped, 1 = critically damped.
	float reductionLimit;								// the minimum amount that we reduce average acceleration or speed to as a fraction of the original value

	// Parameters that fully define the shaping
	unsigned int numExtraImpulses;						// the number of extra impulses
	float coefficients[MaxExtraImpulses];				// the coefficients of all the impulses
	float durations[MaxExtraImpulses];					// the duration in step clocks of each impulse

	// Secondary parameters, calculated from the primary ones
	float totalShapingClocks;							// the total time in step clocks for shaping the start or end of an acceleration or deceleration
	float minimumShapingStartOriginalClocks;			// the minimum acceleration/deceleration time for which we can shape the start, without changing the acceleration/deceleration
	float minimumShapingEndOriginalClocks;				// the minimum acceleration/deceleration time for which we can shape the start, without changing the acceleration/deceleration
	float minimumNonOverlappedOriginalClocks;			// the minimum original acceleration or deceleration time using non-overlapped start and end shaping
	float extraClocksAtStart;							// the extra time needed to shape the start of acceleration or deceleration
	float extraClocksAtEnd;								// the extra time needed to shape the end of acceleration or deceleration
	float extraDistanceAtStart;							// the extra distance per unit acceleration to shape the start of acceleration or deceleration, less the initial velocity contribution
	float extraDistanceAtEnd;							// the extra distance per unit acceleration to shape the end of acceleration or deceleration, less the final velocity contribution
	float overlappedDurations[2 * MaxExtraImpulses];	// the duration in step clocks of each impulse of an overlapped acceleration or deceleration
	float overlappedCoefficients[2 * MaxExtraImpulses];	// the coefficients if we use a shaped start immediately followed by a shaped end
	float overlappedShapingClocks;						// the acceleration or deceleration duration when we use overlapping, in step clocks
	float overlappedDeltaVPerA;							// the effective acceleration time (velocity change per unit acceleration) when we use overlapping, in step clocks
	float overlappedDistancePerA;						// the distance needed by an overlapped acceleration or deceleration, less the initial velocity contribution
	float overlappedDistancePerDv;						// the distance needed by an overlapped acceleration or deceleration per unit speed change, less the initial velocity contribution

	uint32_t movesShapedFirstTry = 0;
	uint32_t movesShapedOnRetry = 0;
	uint32_t movesTooShortToShape = 0;
	uint32_t movesWrongShapeToShape = 0;
	uint32_t movesCouldPossiblyShape = 0;
};

#endif /* SRC_MOVEMENT_AXISSHAPER_H_ */
