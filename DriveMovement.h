/*
 * DriveMovement.h
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#ifndef DRIVEMOVEMENT_H_
#define DRIVEMOVEMENT_H_

class DDA;

// Struct for passing parameters to the DriveMovement Prepare methods
struct PrepParams
{
	float decelStartDistance;
	uint32_t startSpeedTimesCdivA;
	uint32_t topSpeedTimesCdivA;
	uint32_t decelStartClocks;
	uint32_t topSpeedTimesCdivAPlusDecelStartClocks;
	uint32_t accelClocksMinusAccelDistanceTimesCdivTopSpeed;
	float compFactor;
};

enum class DMState : uint8_t
{
	idle = 0,
	moving = 1,
	stepError = 2
};

// This class describes a single movement of one drive
class DriveMovement
{
public:
	bool CalcNextStepTimeCartesian(const DDA &dda, size_t drive);
	bool CalcNextStepTimeDelta(const DDA &dda, size_t drive);
	void PrepareCartesianAxis(const DDA& dda, const PrepParams& params, size_t drive);
	void PrepareDeltaAxis(const DDA& dda, const PrepParams& params, size_t drive);
	void PrepareExtruder(const DDA& dda, const PrepParams& params, size_t drive);
	void ReduceSpeed(const DDA& dda, float inverseSpeedFactor);
	void DebugPrint(char c, bool withDelta) const;

	// Parameters common to Cartesian, delta and extruder moves

	// The following only need to be stored per-drive if we are supporting elasticity compensation
	uint64_t twoDistanceToStopTimesCsquaredDivA;
	uint32_t startSpeedTimesCdivA;
	int32_t accelClocksMinusAccelDistanceTimesCdivTopSpeed;		// this one can be negative
	uint32_t topSpeedTimesCdivAPlusDecelStartClocks;

	// These values don't depend on how the move is executed, so are set by Init()
	uint32_t totalSteps;								// total number of steps for this move
	uint8_t drive;										// the drive that this DM controls
	DMState state;										// whether this is active or not
	bool direction;										// true=forwards, false=backwards
	uint8_t stepsTillRecalc;							// how soon we need to recalculate

	// These values change as the step is executed
	uint32_t nextStep;									// number of steps already done
	uint32_t nextStepTime;								// how many clocks after the start of this move the next step is due
	uint32_t stepInterval;								// how many clocks between steps
	DriveMovement *nextDM;								// link to next DM that needs a step

	// Parameters unique to a style of move (Cartesian, delta or extruder). Currently, extruders and Cartesian moves use the same parameters.
	union MoveParams
	{
		struct CartesianParameters						// Parameters for Cartesian and extruder movement, including extruder pre-compensation
		{
			// The following don't depend on how the move is executed, so they could be set up in Init()
			uint64_t twoCsquaredTimesMmPerStepDivA;		// 2 * clock^2 * mmPerStepInHyperCuboidSpace / acceleration

			// The following depend on how the move is executed, so they must be set up in Prepare()
			uint32_t accelStopStep;						// the first step number at which we are no longer accelerating
			uint32_t decelStartStep;					// the first step number at which we are decelerating
			uint32_t reverseStartStep;					// the first step number for which we need to reverse direction to to elastic compensation
			uint32_t mmPerStepTimesCdivtopSpeed;		// mmPerStepInHyperCuboidSpace * clock / topSpeed

			// The following only need to be stored per-drive if we are supporting elasticity compensation
			int64_t fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA;		// this one can be negative
		} cart;

		struct DeltaParameters							// Parameters for delta movement
		{
			// The following don't depend on how the move is executed, so they can be set up in Init
			int64_t dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared;
			uint32_t reverseStartStep;
			int32_t hmz0sK;								// the starting step position less the starting Z height, multiplied by the Z movement fraction and K (can go negative)
			int32_t minusAaPlusBbTimesKs;
			uint32_t twoCsquaredTimesMmPerStepDivAK;	// this could be stored in the DDA if all towers use the same steps/mm

			// The following depend on how the move is executed, so they must be set up in Prepare()
			uint32_t accelStopDsK;
			uint32_t decelStartDsK;
			uint32_t mmPerStepTimesCdivtopSpeedK;
		} delta;
	} mp;

	static const uint32_t NoStepTime = 0xFFFFFFFF;		// value to indicate that no further steps are needed when calculating the next step time
	static const uint32_t K1 = 1024;					// a power of 2 used to multiply the value mmPerStepTimesCdivtopSpeed to reduce rounding errors
	static const uint32_t K2 = 512;						// a power of 2 used in delta calculations to reduce rounding errors (but too large makes things worse)
	static const int32_t Kc = 1024 * 1024;				// a power of 2 for scaling the Z movement fraction
};

#endif /* DRIVEMOVEMENT_H_ */
