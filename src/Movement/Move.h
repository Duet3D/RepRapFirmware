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

// Define the number of DDAs and DMs.
// A DDA represents a move in the queue.
// Each DDA needs one DM per drive that it moves, but only when it has been prepared and frozen

#if SAME70
const unsigned int DdaRingLength = 40;
const unsigned int NumDms = 20 * 12;								// allow enough for plenty of CAN expansion
#elif SAM4E || SAM4S
const unsigned int DdaRingLength = 40;
const unsigned int NumDms = 20 * 8;									// suitable for e.g. a delta + 5 input hot end
#else
// We are more memory-constrained on the SAM3X
const unsigned int DdaRingLength = 20;
const unsigned int NumDms = 20 * 5;									// suitable for e.g. a delta + 2-input hot end
#endif

constexpr uint32_t MovementStartDelayClocks = StepTimer::StepClockRate/100;		// 10ms delay between preparing the first move and starting it

enum EndstopHitAction
{
	noStop = 0,
	stopDriver = 1,
	stopAxis = 2,
	stopAll = 3
};

struct EndstopAction
{
	uint16_t driver : 4,			// which driver to stop if the action is stopDriver
			 axis : 4,				// which axis to stop if the action is stopAxis, and which axis to set the position of if setAxisPos is true
			 action : 2,			// the EndstopHitAction for this endstop
			 setAxisPos : 1;		// whether or not to set the axis position to its min or max
};

// This is the master movement class.  It controls all movement in the machine.
class Move INHERIT_OBJECT_MODEL
{
public:
	Move();
	void Init();													// Start me up
	void Spin();													// Called in a tight loop to keep the class going
	void Exit();													// Shut down

	void GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const; // Get the current position in untransformed coords
	void GetCurrentUserPosition(float m[MaxAxes], uint8_t moveType, AxesBitmap xAxes, AxesBitmap yAxes) const;
																	// Return the position (after all queued moves have been executed) in transformed coords
	int32_t GetEndPoint(size_t drive) const { return liveEndPoints[drive]; } 	// Get the current position of a motor
	void LiveCoordinates(float m[MaxTotalDrivers], AxesBitmap xAxes, AxesBitmap yAxes);	// Gives the last point at the end of the last complete DDA transformed to user coords
	void Interrupt() __attribute__ ((hot));							// The hardware's (i.e. platform's)  interrupt should call this.
	bool AllMovesAreFinished();										// Is the look-ahead ring empty?  Stops more moves being added as well.
	void DoLookAhead() __attribute__ ((hot));						// Run the look-ahead procedure
	void SetNewPosition(const float positionNow[MaxTotalDrivers], bool doBedCompensation); // Set the current position to be this
	void SetLiveCoordinates(const float coords[MaxTotalDrivers]);			// Force the live coordinates (see above) to be these
	void ResetExtruderPositions();									// Resets the extrusion amounts of the live coordinates
	void SetXYBedProbePoint(size_t index, float x, float y);		// Record the X and Y coordinates of a probe point
	void SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError); // Record the Z coordinate of a probe point
	float GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const; // Get pre-recorded probe coordinates
	bool FinishedBedProbing(int sParam, const StringRef& reply);	// Calibrate or set the bed equation after probing
	void SetAxisCompensation(unsigned int axis, float tangent);		// Set an axis-pair compensation angle
	float AxisCompensation(unsigned int axis) const;				// The tangent value
	void SetIdentityTransform();									// Cancel the bed equation; does not reset axis angle compensation
	void AxisAndBedTransform(float move[], AxesBitmap xAxes, AxesBitmap yAxes, bool useBedCompensation) const;
																	// Take a position and apply the bed and the axis-angle compensations
	void InverseAxisAndBedTransform(float move[], AxesBitmap xAxes, AxesBitmap yAxes) const;
																	// Go from a transformed point back to user coordinates
	void SetZeroHeightError(const float coords[MaxAxes]);			// Set zero height error at these coordinates
	float GetTaperHeight() const { return (useTaper) ? taperHeight : 0.0; }
	void SetTaperHeight(float h);
	bool UseMesh(bool b);											// Try to enable mesh bed compensation and report the final state
	bool IsUsingMesh() const { return usingMesh; }					// Return true if we are using mesh compensation
	unsigned int GetNumProbePoints() const;							// Return the number of currently used probe points
	float PushBabyStepping(float amount);							// Try to push some babystepping through the lookahead queue

	GCodeResult ConfigureAccelerations(GCodeBuffer&gb, const StringRef& reply);			// process M204
	GCodeResult ConfigureDynamicAcceleration(GCodeBuffer& gb, const StringRef& reply);	// process M593

	float GetMaxPrintingAcceleration() const { return maxPrintingAcceleration; }
	float GetMaxTravelAcceleration() const { return maxTravelAcceleration; }
	float GetDRCfreq() const { return 1.0/drcPeriod; }
	float GetDRCperiod() const { return drcPeriod; }
	float GetDRCminimumAcceleration() const { return drcMinimumAcceleration; }
	float IsDRCenabled() const { return drcEnabled; }

	void Diagnostics(MessageType mtype);							// Report useful stuff
	void RecordLookaheadError() { ++numLookaheadErrors; }			// Record a lookahead error

	// Kinematics and related functions
	Kinematics& GetKinematics() const { return *kinematics; }
	bool SetKinematics(KinematicsType k);											// Set kinematics, return true if successful
	bool CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const;
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

	bool IsRawMotorMove(uint8_t moveType) const;									// Return true if this is a raw motor move

	void CurrentMoveCompleted() __attribute__ ((hot));								// Signal that the current move has just been completed
	bool TryStartNextMove(uint32_t startTime) __attribute__ ((hot));				// Try to start another move, returning true if Step() needs to be called immediately
	float IdleTimeout() const;														// Returns the idle timeout in seconds
	void SetIdleTimeout(float timeout);												// Set the idle timeout in seconds

	void Simulate(uint8_t simMode);													// Enter or leave simulation mode
	float GetSimulationTime() const { return simulationTime; }						// Get the accumulated simulation time
	void PrintCurrentDda() const;													// For debugging

	bool PausePrint(RestorePoint& rp);												// Pause the print as soon as we can, returning true if we were able to
#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT
	bool LowPowerOrStallPause(RestorePoint& rp);									// Pause the print immediately, returning true if we were able to
#endif

	bool NoLiveMovement() const;													// Is a move running, or are there any queued?

	bool IsExtruding() const;														// Is filament being extruded?

	uint32_t GetScheduledMoves() const { return scheduledMoves; }					// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const { return completedMoves; }					// How many moves have been completed?
	void ResetMoveCounters() { scheduledMoves = completedMoves = 0; }

	HeightMap& AccessHeightMap() { return heightMap; }								// Access the bed probing grid
	const GridDefinition& GetGrid() const { return heightMap.GetGrid(); }			// Get the grid definition
	bool LoadHeightMapFromFile(FileStore *f, const StringRef& r);					// Load the height map from a file returning true if an error occurred
	bool SaveHeightMapToFile(FileStore *f) const;									// Save the height map to a file returning true if an error occurred

	const RandomProbePointSet& GetProbePoints() const { return probePoints; }		// Return the probe point set constructed from G30 commands

	const DDA *GetCurrentDDA() const { return currentDda; }							// Return the DDA of the currently-executing move

	float GetTopSpeed() const;
	float GetRequestedSpeed() const;

	void AdjustLeadscrews(const floatc_t corrections[]);							// Called by some Kinematics classes to adjust the leadscrews

	int32_t GetAccumulatedExtrusion(size_t extruder, bool& isPrinting);				// Return and reset the accumulated commanded extrusion amount

	bool WriteResumeSettings(FileStore *f) const;									// Write settings for resuming the print

	uint32_t ExtruderPrintingSince() const { return extrudersPrintingSince; }		// When we started doing normal moves after the most recent extruder-only move

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const;			// Get the current step interval for this axis or extruder
#endif

	static int32_t MotorMovementToSteps(size_t drive, float coord);					// Convert a single motor position to number of steps
	static float MotorStepsToMovement(size_t drive, int32_t endpoint);				// Convert number of motor steps to motor position

protected:
	DECLARE_OBJECT_MODEL

private:
	enum class MoveState : uint8_t
	{
		idle,			// no moves being executed or in queue, motors are at idle hold
		collecting,		// no moves currently being executed but we are collecting moves ready to execute them
		executing,		// we are executing moves
		timing			// no moves being executed or in queue, motors are at full current
	};

	bool StartNextMove(uint32_t startTime) __attribute__ ((hot));								// Start the next move, returning true if Step() needs to be called immediately
	void BedTransform(float move[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const;			// Take a position and apply the bed compensations
	void InverseBedTransform(float move[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const;	// Go from a bed-transformed point back to user coordinates
	void AxisTransform(float move[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const;			// Take a position and apply the axis-angle compensations
	void InverseAxisTransform(float move[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const;	// Go from an axis transformed point back to user coordinates
	void SetPositions(const float move[MaxTotalDrivers]);										// Force the machine coordinates to be these
	float GetInterpolatedHeightError(float xCoord, float yCoord) const;							// Get the height error at an XY position

	bool DDARingAdd();									// Add a processed look-ahead entry to the DDA ring
	DDA* DDARingGet();									// Get the next DDA ring entry to be run
	bool DDARingEmpty() const;							// Anything there?

	DDA* volatile currentDda;
	DDA* ddaRingAddPointer;
	DDA* volatile ddaRingGetPointer;
	DDA* ddaRingCheckPointer;

	bool active;										// Are we live and running?
	uint8_t simulationMode;								// Are we simulating, or really printing?
	MoveState moveState;								// whether the idle timer is active
	bool drcEnabled;

	float maxPrintingAcceleration;
	float maxTravelAcceleration;
	float drcPeriod;									// the period of ringing that we don't want to excite
	float drcMinimumAcceleration;						// the minimum value that we reduce acceleration to

	unsigned int numLookaheadUnderruns;					// How many times we have run out of moves to adjust during lookahead
	unsigned int numPrepareUnderruns;					// How many times we wanted a new move but there were only un-prepared moves in the queue
	unsigned int numLookaheadErrors;					// How many times our lookahead algorithm failed
	unsigned int idleCount;								// The number of times Spin was called and had no new moves to process
	uint32_t longestGcodeWaitInterval;					// the longest we had to wait for a new GCode
	float simulationTime;								// Print time since we started simulating

	float extrusionPending[MaxExtruders];				// Extrusion not done due to rounding to nearest step
	volatile float liveCoordinates[MaxTotalDrivers];	// The endpoint that the machine moved to in the last completed move
	volatile bool liveCoordinatesValid;					// True if the XYZ live coordinates are reliable (the extruder ones always are)
	volatile int32_t liveEndPoints[MaxTotalDrivers];	// The XYZ endpoints of the last completed move in motor coordinates
	volatile int32_t extrusionAccumulators[MaxExtruders]; // Accumulated extruder motor steps
	volatile uint32_t extrudersPrintingSince;			// The milliseconds clock time when extrudersPrinting was set to true
	volatile bool extrudersPrinting;					// Set whenever an extruder starts a printing move, cleared by a non-printing extruder move

	float tangents[3]; 									// Axis compensation - 90 degrees + angle gives angle between axes
	float& tanXY = tangents[0];
	float& tanYZ = tangents[1];
	float& tanXZ = tangents[2];

	HeightMap heightMap;    							// The grid definition in use and height map for G29 bed probing
	RandomProbePointSet probePoints;					// G30 bed probe points
	float taperHeight;									// Height over which we taper
	float recipTaperHeight;								// Reciprocal of the taper height
	float zShift;										// Height to add to the bed transform
	bool usingMesh;										// true if we are using the height map, false if we are using the random probe point set
	bool useTaper;										// True to taper off the compensation

	uint32_t idleTimeout;								// How long we wait with no activity before we reduce motor currents to idle, in milliseconds
	uint32_t lastStateChangeTime;						// The approximate time at which the state last changed, except we don't record timing->idle

	Kinematics *kinematics;								// What kinematics we are using

	unsigned int stepErrors;							// count of step errors, for diagnostics
	uint32_t scheduledMoves;							// Move counters for the code queue
	volatile uint32_t completedMoves;					// This one is modified by an ISR, hence volatile

	float specialMoveCoords[MaxTotalDrivers];			// Amounts by which to move individual motors (leadscrew adjustment move)
	bool specialMoveAvailable;							// True if a leadscrew adjustment move is pending

	EndstopAction endstopActions[NumEndstops];
#if HAS_STALL_DETECT
	EndstopAction stallActions[NumDirectDrivers];
#endif
};

//******************************************************************************************************

inline bool Move::DDARingEmpty() const
{
	return ddaRingGetPointer == ddaRingAddPointer		// by itself this means the ring is empty or full
		&& ddaRingAddPointer->GetState() == DDA::DDAState::empty;
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

// Start the next move. Return true if the
// Must be called with base priority greater than the step interrupt, to avoid a race with the step ISR.
inline bool Move::StartNextMove(uint32_t startTime)
pre(ddaRingGetPointer->GetState() == DDA::frozen)
{
	DDA * const cdda = ddaRingGetPointer;			// capture volatile variable
	if (cdda->IsNonPrintingExtruderMove())
	{
		extrudersPrinting = false;
	}
	else if (!extrudersPrinting)
	{
		extrudersPrinting = true;
		extrudersPrintingSince = millis();
	}
	currentDda = cdda;
	return cdda->Start(startTime);
}

// This is the function that is called by the timer interrupt to step the motors.
// This may occasionally get called prematurely.
inline void Move::Interrupt()
{
	if (currentDda != nullptr)
	{
		do
		{
		} while (currentDda->Step());
	}
}

#if HAS_SMART_DRIVERS

// Get the current step interval for this axis or extruder, or 0 if it is not moving
// This is called from the stepper drivers SPI interface ISR
inline uint32_t Move::GetStepInterval(size_t axis, uint32_t microstepShift) const
{
	const DDA * const cdda = currentDda;		// capture volatile variable
	return (cdda != nullptr && simulationMode == 0) ? cdda->GetStepInterval(axis, microstepShift) : 0;
}

#endif

#endif /* MOVE_H_ */
