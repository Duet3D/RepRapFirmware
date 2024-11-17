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

	bool CanAddMove() const noexcept;
	bool AddStandardMove(const RawMove &nextMove, bool doMotorMapping) noexcept SPEED_CRITICAL;	// Set up a new move, returning true if it represents real movement
	bool AddSpecialMove(float feedRate, const float coords[MaxDriversPerAxis]) noexcept;
#if SUPPORT_ASYNC_MOVES
	bool AddAsyncMove(const AsyncMove& nextMove) noexcept;
#endif

	uint32_t Spin(SimulationMode simulationMode, bool signalMoveCompletion, bool shouldStartMove) noexcept SPEED_CRITICAL;	// Try to process moves in the ring
	bool IsIdle() const noexcept;														// Return true if this DDA ring is idle
	uint32_t GetGracePeriod() const noexcept { return gracePeriod; }					// Return the minimum idle time, before we should start a move. Better to have a few moves in the queue so that we can do lookahead

	DDA *GetCurrentDDA() const noexcept;												// If a move from this ring should be executing now, fetch its DDA
	float PushBabyStepping(size_t axis, float amount) noexcept;							// Try to push some babystepping through the lookahead queue, returning the amount pushed

	uint32_t GetScheduledMoves() const noexcept { return scheduledMoves; }				// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const noexcept { return completedMoves; }				// How many moves have been completed?
	void ResetMoveCounters() noexcept { scheduledMoves = completedMoves = 0; }

	float GetSimulationTime() const noexcept { return simulationTime; }
	void ResetSimulationTime() noexcept { simulationTime = 0.0; }

	float GetRequestedSpeedMmPerSec() const noexcept;
	float GetTopSpeedMmPerSec() const noexcept;
	float GetAccelerationMmPerSecSquared() const noexcept;								// Get the (peak) acceleration for reporting in the object model
	float GetDecelerationMmPerSecSquared() const noexcept;								// Get the (peak) deceleration for reporting in the object model
	float GetTotalExtrusionRate() const noexcept;

	void GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const noexcept; // Get the position at the end of the last queued move in untransformed coords
#if SUPPORT_ASYNC_MOVES
	void GetPartialMachinePosition(float m[MaxAxes], AxesBitmap whichAxes) const noexcept;	// Return the machine coordinates of just some axes
#endif

	void SetPositions(Move& move, const float positions[MaxAxesPlusExtruders], AxesBitmap axes) noexcept;	// Force the machine coordinates to be these
	void AdjustMotorPositions(Move& move, const float adjustment[], size_t numMotors) noexcept;		// Adjust the motor endpoints without moving the motors

	bool PauseMoves(MovementState& ms) noexcept;										// Pause the print as soon as we can, returning true if we were able to skip any moves in the queue
#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT
	bool LowPowerOrStallPause(RestorePoint& rp) noexcept;								// Pause the print immediately, returning true if we were able to
#endif

#if SUPPORT_LASER
	uint32_t ManageLaserPower() noexcept;												// Manage the laser power
#endif
	uint32_t ManageIOBitsAndFeedForward() noexcept;										// Manage the IOBITS (G1 P parameter) and extruder heater feedforward

	void RecordLookaheadError() noexcept { ++numLookaheadErrors; }						// Record a lookahead error
	void Diagnostics(MessageType mtype, unsigned int ringNumber) noexcept;

	bool SetWaitingToEmpty() noexcept;

	GCodeResult ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	void AddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept;			// add a move from the ATE to the movement queue
#endif

#if SUPPORT_REMOTE_COMMANDS
	const volatile int32_t *GetLastMoveStepsTaken() const noexcept { return lastMoveStepsTaken; }
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	uint32_t PrepareMoves(DDA *firstUnpreparedMove, uint32_t moveTimeLeft, unsigned int alreadyPrepared, SimulationMode simulationMode) noexcept;

	DDA* addPointer;															// Pointer to the next DDA that we can use to add a new move, if this DDA is free
	DDA* volatile getPointer;													// Pointer to the oldest committed or provisional move, if not equal to addPointer

	unsigned int numDdasInRing;													// The number of DDAs that this ring contains
	uint32_t gracePeriod;														// The minimum idle time in milliseconds, before we should start a move. Better to have a few moves in the queue so that we can do lookahead

	const Tool *_ecv_null lastFeedForwardTool;									// the tool we last applied heater feedforward to
	float lastAverageExtrusionSpeed;											// the extrusion speed we last set heater feedforward for

	uint32_t scheduledMoves;													// Number of moves scheduled in this ring
	uint32_t completedMoves;													// Number of moves completed in this ring

	unsigned int numLookaheadUnderruns;											// How many times we have run out of moves to adjust during lookahead
	unsigned int numPrepareUnderruns;											// How many times we wanted a new move but there were only un-prepared moves in the queue
	unsigned int numNoMoveUnderruns;											// How many times we wanted a new move but there were none
	unsigned int numLookaheadErrors;											// How many times our lookahead algorithm failed

	float simulationTime;														// Print time since we started simulating
#if SUPPORT_REMOTE_COMMANDS
	volatile int32_t lastMoveStepsTaken[NumDirectDrivers];						// how many steps were taken in the last move we did
#endif

	volatile bool waitingForRingToEmpty;										// True if Move has signalled that we are waiting for this ring to empty
};

#if 0	//TODO save this code for now to remind us how to start the laser, remove it when we have sorted that out
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
#endif

#endif /* SRC_MOVEMENT_DDARING_H_ */
