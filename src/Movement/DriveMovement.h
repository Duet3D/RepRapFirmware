/*
 * DriveMovement.h
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#ifndef DRIVEMOVEMENT_H_
#define DRIVEMOVEMENT_H_

#include "RepRapFirmware.h"

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
	DriveMovement(DriveMovement *next);

	bool CalcNextStepTimeCartesian(const DDA &dda, bool live);
	bool CalcNextStepTimeDelta(const DDA &dda, bool live);
	void PrepareCartesianAxis(const DDA& dda, const PrepParams& params);
	void PrepareDeltaAxis(const DDA& dda, const PrepParams& params);
	void PrepareExtruder(const DDA& dda, const PrepParams& params, bool doCompensation);
	void ReduceSpeed(const DDA& dda, float inverseSpeedFactor);
	void DebugPrint(char c, bool withDelta) const;
	int32_t GetNetStepsLeft() const;
	int32_t GetNetStepsTaken() const;

	static void InitialAllocate(unsigned int num);
	static int NumFree() { return numFree; }
	static int MinFree() { return minFree; }
	static void ResetMinFree() { minFree = numFree; }
	static DriveMovement *Allocate(size_t drive, DMState st);
	static void Release(DriveMovement *item);

private:
	bool CalcNextStepTimeCartesianFull(const DDA &dda, bool live);
	bool CalcNextStepTimeDeltaFull(const DDA &dda, bool live);

	static DriveMovement *freeList;
	static int numFree;
	static int minFree;

public:
	// Parameters common to Cartesian, delta and extruder moves

	// The following only need to be stored per-drive if we are supporting pressure advance
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
	uint32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	uint32_t nextStepTime;								// how many clocks after the start of this move the next step is due
	uint32_t stepInterval;								// how many clocks between steps
	DriveMovement *nextDM;								// link to next DM that needs a step

	// Parameters unique to a style of move (Cartesian, delta or extruder). Currently, extruders and Cartesian moves use the same parameters.
	union MoveParams
	{
		struct CartesianParameters						// Parameters for Cartesian and extruder movement, including extruder pressure advance
		{
			// The following don't depend on how the move is executed, so they could be set up in Init()
			uint64_t twoCsquaredTimesMmPerStepDivA;		// 2 * clock^2 * mmPerStepInHyperCuboidSpace / acceleration

			// The following depend on how the move is executed, so they must be set up in Prepare()
			uint32_t accelStopStep;						// the first step number at which we are no longer accelerating
			uint32_t decelStartStep;					// the first step number at which we are decelerating
			uint32_t mmPerStepTimesCdivtopSpeed;		// mmPerStepInHyperCuboidSpace * clock / topSpeed

			// The following only need to be stored per-drive if we are supporting pressure advance
			int64_t fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivA;		// this one can be negative
		} cart;

		struct DeltaParameters							// Parameters for delta movement
		{
			// The following don't depend on how the move is executed, so they can be set up in Init
			int64_t dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared;
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

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1.
// This is also used for extruders on delta machines.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTimeCartesian(const DDA &dda, bool live)
{
	++nextStep;
	if (nextStep <= totalSteps)
	{
		if (stepsTillRecalc != 0)
		{
			--stepsTillRecalc;			// we are doing double/quad/octal stepping
			return true;
		}
		return CalcNextStepTimeCartesianFull(dda, live);
	}

	state = DMState::idle;
	return false;
}

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1.
inline bool DriveMovement::CalcNextStepTimeDelta(const DDA &dda, bool live)
{
	++nextStep;
	if (nextStep <= totalSteps)
	{
		if (stepsTillRecalc != 0)
		{
			--stepsTillRecalc;			// we are doing double or quad stepping
			return true;
		}
		else
		{
			return CalcNextStepTimeDeltaFull(dda, live);
		}
	}

	state = DMState::idle;
	return false;
}

// Return the number of net steps left for the move in the forwards direction.
// We have already taken nextSteps - 1 steps, unless nextStep is zero.
inline int32_t DriveMovement::GetNetStepsLeft() const
{
	int32_t netStepsLeft;
	if (reverseStartStep > totalSteps)		// if no reverse phase
	{
		netStepsLeft = (nextStep == 0) ? (int32_t)totalSteps : (int32_t)totalSteps - (int32_t)nextStep + 1;
	}
	else if (nextStep >= reverseStartStep)
	{
		netStepsLeft = (int32_t)totalSteps - (int32_t)nextStep + 1;
	}
	else
	{
		const int32_t totalNetSteps = (int32_t)(2 * reverseStartStep) - (int32_t)totalSteps - 2;
		netStepsLeft = (nextStep == 0) ? totalNetSteps : totalNetSteps - (int32_t)nextStep + 1;
	}
	return (direction) ? netStepsLeft : -netStepsLeft;
}

// Return the number of net steps already taken for the move in the forwards direction.
// We have already taken nextSteps - 1 steps, unless nextStep is zero.
inline int32_t DriveMovement::GetNetStepsTaken() const
{
	int32_t netStepsTaken;
	if (nextStep < reverseStartStep || reverseStartStep > totalSteps)				// if no reverse phase, or not started it yet
	{
		netStepsTaken = (nextStep == 0) ? 0 : (int32_t)nextStep - 1;
	}
	else
	{
		netStepsTaken = (int32_t)nextStep - (int32_t)(2 * reverseStartStep) + 2;	// allowing for direction having changed
	}
	return (direction) ? netStepsTaken : -netStepsTaken;
}

#endif /* DRIVEMOVEMENT_H_ */
