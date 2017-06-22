/*
 * Move.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef MOVE_H_
#define MOVE_H_

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "DDA.h"								// needed because of our inline functions
#include "BedProbing/RandomProbePointSet.h"
#include "BedProbing/Grid.h"
#include "Kinematics/Kinematics.h"
#include "GCodes/RestorePoint.h"

#ifdef DUET_NG
const unsigned int DdaRingLength = 30;
#else
// We are more memory-constrained on the SAM3X
const unsigned int DdaRingLength = 20;
#endif

/**
 * This is the master movement class.  It controls all movement in the machine.
 */
class Move
{
public:
	Move();
	void Init();													// Start me up
	void Spin();													// Called in a tight loop to keep the class going
	void Exit();													// Shut down

	void GetCurrentMachinePosition(float m[DRIVES], bool disableMotorMapping) const; // Get the current position in untransformed coords
	void GetCurrentUserPosition(float m[DRIVES], uint8_t moveType, uint32_t xAxes) const; // Return the position (after all queued moves have been executed) in transformed coords
	int32_t GetEndPoint(size_t drive) const { return liveEndPoints[drive]; } // Get the current position of a motor
	void LiveCoordinates(float m[DRIVES], uint32_t xAxes);			// Gives the last point at the end of the last complete DDA transformed to user coords
	void Interrupt();												// The hardware's (i.e. platform's)  interrupt should call this.
	void InterruptTime();											// Test function - not used
	bool AllMovesAreFinished();										// Is the look-ahead ring empty?  Stops more moves being added as well.
	void DoLookAhead();												// Run the look-ahead procedure
	void HitLowStop(size_t axis, DDA* hitDDA);						// What to do when a low endstop is hit
	void HitHighStop(size_t axis, DDA* hitDDA);						// What to do when a high endstop is hit
	void ZProbeTriggered(DDA* hitDDA);								// What to do when a the Z probe is triggered
	void SetNewPosition(const float positionNow[DRIVES], bool doBedCompensation); // Set the current position to be this
	void SetLiveCoordinates(const float coords[DRIVES]);			// Force the live coordinates (see above) to be these
	void ResetExtruderPositions();									// Resets the extrusion amounts of the live coordinates
	void SetXYBedProbePoint(size_t index, float x, float y);		// Record the X and Y coordinates of a probe point
	void SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError); // Record the Z coordinate of a probe point
	float GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const; // Get pre-recorded probe coordinates
	void FinishedBedProbing(int sParam, StringRef& reply);			// Calibrate or set the bed equation after probing
	void SetAxisCompensation(int8_t axis, float tangent);			// Set an axis-pair compensation angle
	float AxisCompensation(int8_t axis) const;						// The tangent value
	void SetIdentityTransform();									// Cancel the bed equation; does not reset axis angle compensation
	void AxisAndBedTransform(float move[], uint32_t xAxes, bool useBedCompensation) const; // Take a position and apply the bed and the axis-angle compensations
	void InverseAxisAndBedTransform(float move[], uint32_t xAxes) const;	// Go from a transformed point back to user coordinates
	float GetTaperHeight() const { return (useTaper) ? taperHeight : 0.0; }
	void SetTaperHeight(float h);
	bool UseMesh(bool b);											// Try to enable mesh bed compensation and report the final state
	bool IsUsingMesh() const { return usingMesh; }					// Return true if we are using mesh compensation
	float PushBabyStepping(float amount);							// Try to push some babystepping through the lookahead queue

	void Diagnostics(MessageType mtype);							// Report useful stuff

	// Kinematics and related functions
	Kinematics& GetKinematics() const { return *kinematics; }
	bool SetKinematics(KinematicsType k);											// Set kinematics, return true if successful
	bool CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes]) const;
																					// Convert Cartesian coordinates to delta motor coordinates, return true if successful
	void MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const;
																					// Convert motor coordinates to machine coordinates
	void EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const;
	void AdjustMotorPositions(const float_t adjustment[], size_t numMotors);		// Perform motor endpoint adjustment
	const char* GetGeometryString() const { return kinematics->GetName(true); }
	bool IsAccessibleProbePoint(float x, float y) const;

	// Temporary kinematics functions
	bool IsDeltaMode() const { return kinematics->GetKinematicsType() == KinematicsType::linearDelta; }
	// End temporary functions

	void CurrentMoveCompleted();													// Signal that the current move has just been completed
	bool TryStartNextMove(uint32_t startTime);										// Try to start another move, returning true if Step() needs to be called immediately
	float IdleTimeout() const { return idleTimeout; }								// Returns the idle timeout in seconds
	void SetIdleTimeout(float timeout) { idleTimeout = timeout; }					// Set the idle timeout in seconds

	void Simulate(uint8_t simMode);													// Enter or leave simulation mode
	float GetSimulationTime() const { return simulationTime; }						// Get the accumulated simulation time
	void PrintCurrentDda() const;													// For debugging

	FilePosition PausePrint(RestorePoint& rp, uint32_t xAxes);						// Pause the print as soon as we can
	bool NoLiveMovement() const;													// Is a move running, or are there any queued?

	bool IsExtruding() const;														// Is filament being extruded?

	uint32_t GetScheduledMoves() const { return scheduledMoves; }					// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const { return completedMoves; }					// How many moves have been completed?
	void ResetMoveCounters() { scheduledMoves = completedMoves = 0; }

	HeightMap& AccessBedProbeGrid() { return grid; }								// Access the bed probing grid

	const DDA *GetCurrentDDA() const { return currentDda; }							// Return the DDA of the currently-executing move

	static int32_t MotorEndPointToMachine(size_t drive, float coord);				// Convert a single motor position to number of steps
	static float MotorEndpointToPosition(int32_t endpoint, size_t drive);			// Convert number of motor steps to motor position

private:
	enum class IdleState : uint8_t { idle, busy, timing };

	bool StartNextMove(uint32_t startTime);											// start the next move, returning true if Step() needs to be called immediately
	void BedTransform(float move[MaxAxes], uint32_t xAxes) const;					// Take a position and apply the bed compensations
	void InverseBedTransform(float move[MaxAxes], uint32_t xAxes) const;			// Go from a bed-transformed point back to user coordinates
	void AxisTransform(float move[MaxAxes]) const;									// Take a position and apply the axis-angle compensations
	void InverseAxisTransform(float move[MaxAxes]) const;							// Go from an axis transformed point back to user coordinates
	void JustHomed(size_t axis, float hitPoint, DDA* hitDDA);						// Deal with setting positions after a drive has been homed
	void SetPositions(const float move[DRIVES]);									// Force the machine coordinates to be these

	bool DDARingAdd();									// Add a processed look-ahead entry to the DDA ring
	DDA* DDARingGet();									// Get the next DDA ring entry to be run
	bool DDARingEmpty() const;							// Anything there?

	DDA* volatile currentDda;
	DDA* ddaRingAddPointer;
	DDA* volatile ddaRingGetPointer;
	DDA* ddaRingCheckPointer;

	bool active;										// Are we live and running?
	uint8_t simulationMode;								// Are we simulating, or really printing?
	bool waitingForMove;								// True if we are waiting for a new move
	unsigned int numLookaheadUnderruns;					// How many times we have run out of moves to adjust during lookahead
	unsigned int numPrepareUnderruns;					// How many times we wanted a new move but there were only un-prepared moves in the queue
	unsigned int idleCount;								// The number of times Spin was called and had no new moves to process
	uint32_t longestGcodeWaitInterval;					// the longest we had to wait for a new gcode
	uint32_t gcodeWaitStartTime;						// When we last asked for a gcode and didn't get one
	float simulationTime;								// Print time since we started simulating
	volatile float liveCoordinates[DRIVES];				// The endpoint that the machine moved to in the last completed move
	volatile bool liveCoordinatesValid;					// True if the XYZ live coordinates are reliable (the extruder ones always are)
	volatile int32_t liveEndPoints[DRIVES];				// The XYZ endpoints of the last completed move in motor coordinates

	float tanXY, tanYZ, tanXZ; 							// Axis compensation - 90 degrees + angle gives angle between axes
	float recipTaperHeight;								// Reciprocal of the taper height
	bool useTaper;										// True to taper off the compensation

	HeightMap grid;    									// Grid definition and height map for G29 bed probing. The probe heights are stored in zBedProbePoints, see above.
	RandomProbePointSet probePoints;					// G30 bed probe points
	bool usingMesh;										// true if we are using the height map, false if we are using the random probe point set
	float taperHeight;									// Height over which we taper

	float idleTimeout;									// How long we wait with no activity before we reduce motor currents to idle
	float lastMoveTime;									// The approximate time at which the last move was completed, or 0
	float longWait;										// A long time for things that need to be done occasionally
	IdleState iState;									// whether the idle timer is active

	Kinematics *kinematics;								// What kinematics we are using

	unsigned int stepErrors;							// count of step errors, for diagnostics
	uint32_t scheduledMoves;							// Move counters for the code queue
	volatile uint32_t completedMoves;					// This one is modified by an ISR, hence volatile
};

//******************************************************************************************************

inline bool Move::DDARingEmpty() const
{
	return ddaRingGetPointer == ddaRingAddPointer;
}

inline bool Move::NoLiveMovement() const
{
	return DDARingEmpty() && currentDda == nullptr;		// must test currentDda and DDARingEmpty *in this order* !
}

// To wait until all the current moves in the buffers are complete, call this function repeatedly and wait for it to return true.
// Then do whatever you wanted to do after all current moves have finished.
// Then call ResumeMoving() otherwise nothing more will ever happen.
inline bool Move::AllMovesAreFinished()
{
	return NoLiveMovement();
}

// Start the next move. Must be called with interrupts disabled, to avoid a race with the step ISR.
inline bool Move::StartNextMove(uint32_t startTime)
pre(ddaRingGetPointer->GetState() == DDA::frozen)
{
	currentDda = ddaRingGetPointer;
	return currentDda->Start(startTime);
}

// This is the function that is called by the timer interrupt to step the motors.
inline void Move::Interrupt()
{
	if (currentDda != nullptr)
	{
		do
		{
		} while (currentDda->Step());
	}
}

#endif /* MOVE_H_ */
