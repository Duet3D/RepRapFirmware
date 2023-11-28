/*
 * DDARing.h
 *
 *  Created on: 28 Feb 2019
 *      Author: David
 *
 *  This class represents a queue of moves, where for each move the movement is synchronised between all the motors involved.
 */

#ifndef SRC_MOVEMENT_DDARING_H_
#define SRC_MOVEMENT_DDARING_H_

#include "DDA.h"

class MovementState;

class DDARing INHERIT_OBJECT_MODEL
{
public:
	DDARing() noexcept;

	void Init1(unsigned int numDdas) noexcept;
	void Init2() noexcept;
	void Exit() noexcept;

	void RecycleDDAs() noexcept;
	bool CanAddMove() const noexcept;
	bool AddStandardMove(const RawMove &nextMove, bool doMotorMapping) noexcept SPEED_CRITICAL;	// Set up a new move, returning true if it represents real movement
	bool AddSpecialMove(float feedRate, const float coords[MaxDriversPerAxis]) noexcept;
#if SUPPORT_ASYNC_MOVES
	bool AddAsyncMove(const AsyncMove& nextMove) noexcept;
#endif

	uint32_t Spin(SimulationMode simulationMode, bool waitingForSpace, bool shouldStartMove) noexcept SPEED_CRITICAL;	// Try to process moves in the ring
	bool IsIdle() const noexcept;														// Return true if this DDA ring is idle
	uint32_t GetGracePeriod() const noexcept { return gracePeriod; }					// Return the minimum idle time, before we should start a move. Better to have a few moves in the queue so that we can do lookahead

	float PushBabyStepping(size_t axis, float amount) noexcept;							// Try to push some babystepping through the lookahead queue, returning the amount pushed

	uint32_t GetScheduledMoves() const noexcept { return scheduledMoves; }				// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const noexcept { return completedMoves; }				// How many moves have been completed?
	void ResetMoveCounters() noexcept { scheduledMoves = completedMoves = 0; }

	float GetSimulationTime() const noexcept { return simulationTime; }
	void ResetSimulationTime() noexcept { simulationTime = 0.0; }

	float GetRequestedSpeedMmPerSec() const noexcept { return InverseConvertSpeedToMmPerSec(lastRequestedSpeed); }
	float GetTopSpeedMmPerSec() const noexcept { return InverseConvertSpeedToMmPerSec(lastTopSpeed); }
	float GetAccelerationMmPerSecSquared() const noexcept { return InverseConvertAcceleration(lastAcceleration); }
	float GetDecelerationMmPerSecSquared() const noexcept { return InverseConvertAcceleration(lastDeceleration); }
	float GetTotalExtrusionRate() const noexcept { return InverseConvertSpeedToMmPerSec(lastExtrusionRate); }

	void GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const noexcept; // Get the position at the end of the last queued move in untransformed coords
#if SUPPORT_ASYNC_MOVES
	void GetPartialMachinePosition(float m[MaxAxes], AxesBitmap whichAxes) const noexcept;	// Return the machine coordinates of just some axes
#endif

	void SetPositions(const float move[MaxAxesPlusExtruders]) noexcept;					// Force the machine coordinates to be these
	void AdjustMotorPositions(const float adjustment[], size_t numMotors) noexcept;		// Perform motor endpoint adjustment
	void GetCurrentMotorPositions(int32_t pos[MaxAxesPlusExtruders]) const noexcept;	// Get the live motor positions
	void LiveCoordinates(float m[MaxAxesPlusExtruders]) noexcept;						// Fetch the last point at the end of the last completed DDA
	void ResetExtruderPositions() noexcept;												// Resets the extrusion amounts of the live coordinates

	bool PauseMoves(MovementState& ms) noexcept;										// Pause the print as soon as we can, returning true if we were able to skip any moves in the queue
#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT
	bool LowPowerOrStallPause(RestorePoint& rp) noexcept;								// Pause the print immediately, returning true if we were able to
#endif

#if SUPPORT_LASER
	uint32_t ManageLaserPower() const noexcept;											// Manage the laser power
#endif

	void RecordLookaheadError() noexcept { ++numLookaheadErrors; }						// Record a lookahead error
	void Diagnostics(MessageType mtype, unsigned int ringNumber) noexcept;

	bool SetWaitingToEmpty() noexcept;

	GCodeResult ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	void AddMoveFromRemote(const CanMessageMovementLinear& msg) noexcept;				// add a move from the ATE to the movement queue
	void AddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept;			// add a move from the ATE to the movement queue
#endif

#if SUPPORT_REMOTE_COMMANDS
	const volatile int32_t *GetLastMoveStepsTaken() const noexcept { return lastMoveStepsTaken; }
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	uint32_t PrepareMoves(DDA *firstUnpreparedMove, int32_t moveTimeLeft, unsigned int alreadyPrepared, SimulationMode simulationMode) noexcept;

	DDA* addPointer;
	DDA* volatile getPointer;
	DDA* checkPointer;

	volatile float liveCoordinates[MaxAxesPlusExtruders];						// The endpoint that the machine moved to in the last completed move

	unsigned int numDdasInRing;
	uint32_t gracePeriod;														// The minimum idle time in milliseconds, before we should start a move. Better to have a few moves in the queue so that we can do lookahead

	uint32_t scheduledMoves;													// Move counters for the code queue
	volatile uint32_t completedMoves;											// This one is modified by an ISR, hence volatile
	volatile int32_t numHiccups;												// Modified in the ISR

//	int32_t endPoint[MaxAxesPlusExtruders];								  		// Machine coordinates of the endpoint of the last dispatched move

	float lastRequestedSpeed = 0.0;												// the requested speed of the last move that was dispatched to the segment queue
	float lastTopSpeed = 0.0;													// the actual top speed of the last move that was dispatched to the segment queue
	float lastAcceleration = 0.0;												// the actual acceleration of the last move that was dispatched to the segment queue
	float lastDeceleration = 0.0;												// the actual deceleration of the last move that was dispatched to the segment queue
	float lastExtrusionRate = 0.0;												// the extrusion rate of the last move that was dispatched to the segment queue

	unsigned int numLookaheadUnderruns;											// How many times we have run out of moves to adjust during lookahead
	unsigned int numPrepareUnderruns;											// How many times we wanted a new move but there were only un-prepared moves in the queue
	unsigned int numNoMoveUnderruns;											// How many times we wanted a new move but there were none
	unsigned int numLookaheadErrors;											// How many times our lookahead algorithm failed
	unsigned int stepErrors;													// count of step errors, for diagnostics

	float simulationTime;														// Print time since we started simulating
#if SUPPORT_REMOTE_COMMANDS
	volatile int32_t lastMoveStepsTaken[NumDirectDrivers];						// how many steps were taken in the last move we did
#endif
	volatile uint32_t extrudersPrintingSince;									// The milliseconds clock time when extrudersPrinting was set to true

	volatile bool extrudersPrinting;											// Set whenever an extruder starts a printing move, cleared by a non-printing extruder move
	volatile bool liveCoordinatesValid;											// True if the XYZ live coordinates in liveCoordinates are reliable (the extruder ones always are)
	volatile bool waitingForRingToEmpty;										// True if Move has signalled that we are waiting for this ring to empty
};

// Start the next move. Return true if laser or IO bits need to be active
// Must be called with base priority greater than or equal to the step interrupt, to avoid a race with the step ISR.
inline bool DDARing::StartNextMove(Platform& p, uint32_t startTime) noexcept
pre(getPointer->GetState() == DDA::frozen)
{
	DDA * const cdda = getPointer;			// capture volatile variable
	if (cdda->IsNonPrintingExtruderMove())
	{
		extrudersPrinting = false;
	}
	else if (!extrudersPrinting)
	{
		extrudersPrintingSince = millis();
		extrudersPrinting = true;
	}
	currentDda = cdda;
	cdda->Start(p, startTime);
#if SUPPORT_LASER || SUPPORT_IOBITS
	return cdda->ControlLaser();
#else
	return false;
#endif
}

#endif /* SRC_MOVEMENT_DDARING_H_ */
