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

#if SUPPORT_S_CURVE
# include "MovementProfile.h"
#endif

class MovementState;

class DDARing final INHERIT_OBJECT_MODEL
{
public:
	DDARing() noexcept;

	void Init(unsigned int numDdas) noexcept;
	void Exit() noexcept;

	bool CanAddMove() const noexcept;
	MovementError AddStandardMove(const RawMove &nextMove, bool doMotorMapping) noexcept SPEED_CRITICAL;	// Set up a new move, returning true if it represents real movement
	bool AddSpecialMove(float feedRate, const float coords[MaxDriversPerAxis]) noexcept;
#if SUPPORT_ASYNC_MOVES
	bool AddAsyncMove(const AsyncMove& nextMove) noexcept;
#endif

	uint32_t Spin(uint32_t prepareAdvanceTime, SimulationMode simulationMode, bool signalMoveCompletion, bool shouldStartMove) noexcept SPEED_CRITICAL;	// Try to process moves in the ring
	bool IsIdle() const noexcept;														// Return true if this DDA ring is idle
	uint32_t GetGracePeriod() const noexcept { return gracePeriod; }					// Return the minimum idle time, before we should start a move. Better to have a few moves in the queue so that we can do lookahead

	DDA *_ecv_null GetCurrentDDA() const noexcept;										// If a move from this ring should be executing now, fetch its DDA
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
	float GetCurrentMoveDistance() const noexcept;
	float GetCurrentMoveDuration() const noexcept;
	FilePosition GetCurrentMoveFilePosition() const noexcept;							// Get the file position of the move being executed, or noFilePosition if there is none

	void GetCurrentMachinePosition(float m[MaxAxes]) const noexcept;					// Get the position at the end of the last queued move in untransformed coords
	void GetLastEndpoints(LogicalDrivesBitmap logicalDrives, int32_t returnedEndpoints[MaxAxesPlusExtruders]) const noexcept;
	int32_t GetLastEndpoint(size_t drive) const noexcept;
	void SetLastEndpoints(LogicalDrivesBitmap logicalDrives, const int32_t *_ecv_array ep) noexcept;
	void SetLastEndpoint(size_t drive, int32_t ep) noexcept;

	float GetStartCoordinate(size_t axis) const noexcept pre(axis < MaxAxes) { return startCoordinates[axis]; }
	void SetStartCoordinate(size_t axis, float pos) noexcept pre(axis < MaxAxes) { startCoordinates[axis] = pos; }
	void UpdateStartCoordinates(const float coords[MaxAxes]) noexcept;

	bool PauseMoves(MovementState& ms) noexcept;										// Pause the print as soon as we can, returning true if we were able to skip any moves in the queue
#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT
	bool LowPowerOrStallPause(MovementState& ms) noexcept;								// Pause the print immediately, returning true if we were able to
#endif

#if SUPPORT_LASER
	uint32_t ManageLaserPower(Platform& platform) noexcept;								// Manage the laser power
#endif
	uint32_t ManageIOBitsAndFeedForward(Platform& platform) noexcept;					// Manage the IOBITS (G1 P parameter) and extruder heater feedforward

	void RecordLookaheadError() noexcept { ++numLookaheadErrors; }						// Record a lookahead error
	void Diagnostics(const StringRef& reply, unsigned int ringNumber) noexcept;

	bool SetWaitingToEmpty() noexcept;

	GCodeResult ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

protected:
	DECLARE_OBJECT_MODEL

private:
	bool IsTimeToPrepareMove(uint32_t prepareAdvanceTime, uint32_t moveTimeLeft) const noexcept;
	uint32_t PrepareMoves(DDA *firstUnpreparedMove, uint32_t prepareAdvanceTime, uint32_t moveTimeLeft, SimulationMode simulationMode) noexcept;
#if SUPPORT_S_CURVE
	void PlanMoves(DDA *firstUnpreparedMove, bool stopping) noexcept;
	bool NeedNewPlan(DDA *moveToPrepare) const noexcept;
#endif

	DDA* addPointer;															// Pointer to the next DDA that we can use to add a new move, if this DDA is free
	DDA* volatile getPointer;													// Pointer to the oldest committed or provisional move, if not equal to addPointer

	unsigned int numDdasInRing;													// The number of DDAs that this ring contains
	uint32_t gracePeriod = DefaultGracePeriod;									// The minimum idle time in milliseconds, before we should start a move. Better to have a few moves in the queue so that we can do lookahead

#if SUPPORT_S_CURVE
	MovementProfile plannedProfile;												// the profile planned for a collection of moves
#endif

	const Tool *_ecv_null lastFeedForwardTool = nullptr;						// the tool we last applied heater feedforward to
	float lastAverageExtrusionSpeed = 0.0;										// the extrusion speed we last set heater feedforward for

	uint32_t scheduledMoves = 0;												// Number of moves scheduled in this ring
	uint32_t completedMoves = 0;												// Number of moves completed in this ring

	unsigned int numLookaheadUnderruns = 0;										// How many times we have run out of moves to adjust during lookahead
	unsigned int numNoMoveUnderruns = 0;										// How many times we wanted a new move but there were none
	unsigned int numLookaheadErrors = 0;										// How many times our lookahead algorithm failed

	float simulationTime = 0.0;													// Print time since we started simulating

	float startCoordinates[MaxAxes];											// the axis coordinates to start the next move from

	volatile bool waitingForRingToEmpty = false;								// True if Move has signalled that we are waiting for this ring to empty
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
