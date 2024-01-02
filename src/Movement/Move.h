/*
 * Move.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef MOVE_H_
#define MOVE_H_

#include <RepRapFirmware.h>
#include "MoveTiming.h"
#include "AxisShaper.h"
#include "ExtruderShaper.h"
#include "DDARing.h"
#include "DDA.h"								// needed because of our inline functions
#include "BedProbing/RandomProbePointSet.h"
#include "BedProbing/Grid.h"
#include "Kinematics/Kinematics.h"
#include "MoveSegment.h"
#include "DriveMovement.h"
#include "StepTimer.h"
#include <GCodes/RestorePoint.h>
#include <Math/Deviation.h>

#if SUPPORT_ASYNC_MOVES
# include "HeightControl/HeightController.h"
#endif

// Define the number of DDAs and DMs.
// A DDA represents a move in the queue.
// Each DDA needs one DM per drive that it moves, but only when it has been prepared and frozen

#if SAME70

constexpr unsigned int InitialDdaRingLength = 60;
constexpr unsigned int AuxDdaRingLength = 5;
const unsigned int InitialNumDms = (InitialDdaRingLength/2 * 4) + AuxDdaRingLength;

#elif SAM4E || SAM4S || SAME5x

constexpr unsigned int InitialDdaRingLength = 40;
constexpr unsigned int AuxDdaRingLength = 3;
const unsigned int InitialNumDms = (InitialDdaRingLength/2 * 4) + AuxDdaRingLength;

#else

// We are more memory-constrained on the SAM3X and LPC
const unsigned int DdaRingLength = 20;
const unsigned int NumDms = 20 * 5;												// suitable for e.g. a delta + 2-input hot end

#endif

template<class T> class CanMessageMultipleDrivesRequest;
class CanMessageRevertPosition;

// This is the master movement class.  It controls all movement in the machine.
class Move INHERIT_OBJECT_MODEL
{
public:
	Move() noexcept;
	void Init() noexcept;													// Start me up
	void Exit() noexcept;													// Shut down

	[[noreturn]] void MoveLoop() noexcept;									// Main loop called by the Move task

	void GetCurrentMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber, bool disableMotorMapping) const noexcept; // Get the current position in untransformed coords
	void SetRawPosition(const float positions[MaxAxesPlusExtruders], MovementSystemNumber msNumber) noexcept
			pre(queueNumber < NumMovementSystems);							// Set the current position to be this without transforming them first
	void GetCurrentUserPosition(float m[MaxAxes], MovementSystemNumber msNumber, uint8_t moveType, const Tool *tool) const noexcept;
																			// Return the position (after all queued moves have been executed) in transformed coords
	int32_t GetLiveMotorPosition(size_t axis) const noexcept;
	void GetLiveCoordinates(unsigned int msNumber, const Tool *tool, float coordsOut[MaxAxesPlusExtruders]) noexcept;
																			// Gives the last point at the end of the last complete DDA
	void MoveAvailable() noexcept;											// Called from GCodes to tell the Move task that a move is available
	bool WaitingForAllMovesFinished(MovementSystemNumber msNumber) noexcept
		pre(queueNumber < rings.upb);										// Tell the lookahead ring we are waiting for it to empty and return true if it is
	void DoLookAhead() noexcept SPEED_CRITICAL;								// Run the look-ahead procedure
	void SetNewPosition(const float positionNow[MaxAxesPlusExtruders], MovementSystemNumber msNumber, bool doBedCompensation) noexcept
			pre(queueNumber < NumMovementSystems);							// Set the current position to be this
	void ResetExtruderPositions() noexcept;									// Resets the extrusion amounts of the live coordinates
	void SetXYBedProbePoint(size_t index, float x, float y) noexcept;		// Record the X and Y coordinates of a probe point
	void SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError) noexcept; // Record the Z coordinate of a probe point
	float GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const noexcept; // Get pre-recorded probe coordinates
	bool FinishedBedProbing(int sParam, const StringRef& reply) noexcept;	// Calibrate or set the bed equation after probing
	void SetAxisCompensation(unsigned int axis, float tangent) noexcept;	// Set an axis-pair compensation angle
	float AxisCompensation(unsigned int axis) const noexcept;				// The tangent value
	bool IsXYCompensated() const;											// Check if XY axis compensation applies to the X or Y axis
	void SetXYCompensation(bool xyCompensation);							// Define whether XY compensation applies to X (default) or to Y
	void SetIdentityTransform() noexcept;									// Cancel the bed equation; does not reset axis angle compensation
	void AxisAndBedTransform(float move[], const Tool *tool, bool useBedCompensation) const noexcept;
																			// Take a position and apply the bed and the axis-angle compensations
	void InverseAxisAndBedTransform(float move[], const Tool *tool) const noexcept;
																			// Go from a transformed point back to user coordinates
	void SetZeroHeightError(const float coords[MaxAxes]) noexcept;			// Set zero height error at these bed coordinates
	float GetTaperHeight() const noexcept { return (useTaper) ? taperHeight : 0.0; }
	void SetTaperHeight(float h) noexcept;
	bool UseMesh(bool b) noexcept;											// Try to enable mesh bed compensation and report the final state
	bool IsUsingMesh() const noexcept { return usingMesh; }					// Return true if we are using mesh compensation
	unsigned int GetNumProbedProbePoints() const noexcept;					// Return the number of actually probed probe points
	void SetLatestCalibrationDeviation(const Deviation& d, uint8_t numFactors) noexcept;
	void SetInitialCalibrationDeviation(const Deviation& d) noexcept;
	void SetLatestMeshDeviation(const Deviation& d) noexcept;
	void UpdateStepsPerMm() noexcept;										// called when steps/mm is set for any axis or extruder

	float PushBabyStepping(MovementSystemNumber msNumber, size_t axis, float amount) noexcept;				// Try to push some babystepping through the lookahead queue

	GCodeResult ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// process M595
	GCodeResult ConfigurePressureAdvance(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M572

	ExtruderShaper& GetExtruderShaperForExtruder(size_t extruder) noexcept;
	float GetPressureAdvanceClocksForLogicalDrive(size_t drive) const noexcept;
	float GetPressureAdvanceClocksForExtruder(size_t extruder) const noexcept;

#if SUPPORT_REMOTE_COMMANDS
	bool InitFromRemote(const CanMessageMovementLinearShaped& msg) noexcept;
	void StopDriversFromRemote(uint16_t whichDrives) noexcept;
	void RevertPosition(const CanMessageRevertPosition& msg) noexcept;

	void AddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept				// add a move to the movement queue when we are in expansion board mode
	{
		rings[0].AddMoveFromRemote(msg);
		MoveAvailable();
	}

	GCodeResult EutSetRemotePressureAdvance(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept;
	GCodeResult EutSetInputShaping(const CanMessageSetInputShaping& msg, size_t dataLength, const StringRef& reply) noexcept
	{
		return axisShaper.EutSetInputShaping(msg, dataLength, reply);
	}
#endif

	AxisShaper& GetAxisShaper() noexcept { return axisShaper; }

	// Functions called by DDA::Prepare to generate segments for executing DDAs
	void AddLinearSegments(const DDA& dda, size_t logicalDrive, uint32_t startTime, const PrepParams& params, int32_t steps, bool useInputShaping, bool usePressureAdvance) noexcept;
#if SUPPORT_LINEAR_DELTA
	void AddDeltaSegments(const DDA& dda, size_t logicalDrive, uint32_t startTime, const PrepParams& params, int32_t steps, bool useInputShaping) noexcept;
#endif

	void Diagnostics(MessageType mtype) noexcept;							// Report useful stuff

	// Kinematics and related functions
	Kinematics& GetKinematics() const noexcept { return *kinematics; }
	bool SetKinematics(KinematicsType k) noexcept;											// Set kinematics, return true if successful
	bool CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const noexcept;
																							// Convert Cartesian coordinates to delta motor coordinates, return true if successful
	void MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept;
																							// Convert motor coordinates to machine coordinates
	void AdjustMotorPositions(const float adjustment[], size_t numMotors) noexcept;			// Perform motor endpoint adjustment after auto calibration
	const char* GetGeometryString() const noexcept { return kinematics->GetName(true); }
	bool IsAccessibleProbePoint(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept;

	// Temporary kinematics functions
#if SUPPORT_LINEAR_DELTA
	bool IsDeltaMode() const noexcept { return kinematics->GetKinematicsType() == KinematicsType::linearDelta; }
#endif
	// End temporary functions

	bool IsRawMotorMove(uint8_t moveType) const noexcept;									// Return true if this is a raw motor move

	float IdleTimeout() const noexcept;														// Returns the idle timeout in seconds
	void SetIdleTimeout(float timeout) noexcept;											// Set the idle timeout in seconds

	void Simulate(SimulationMode simMode) noexcept;											// Enter or leave simulation mode
	float GetSimulationTime() const noexcept { return rings[0].GetSimulationTime(); }		// Get the accumulated simulation time

	bool PausePrint(MovementState& ms) noexcept;											// Pause the print as soon as we can, returning true if we were able to
#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT
	bool LowPowerOrStallPause(unsigned int queueNumber, RestorePoint& rp) noexcept;			// Pause the print immediately, returning true if we were able to
#endif

	bool NoLiveMovement() const noexcept { return rings[0].IsIdle(); }						// Is a move running, or are there any queued?

	uint32_t GetScheduledMoves() const noexcept { return rings[0].GetScheduledMoves(); }	// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const noexcept { return rings[0].GetCompletedMoves(); }	// How many moves have been completed?
	void ResetMoveCounters() noexcept { rings[0].ResetMoveCounters(); }
	void UpdateExtrusionPendingLimits(float extrusionPending) noexcept;

	HeightMap& AccessHeightMap() noexcept { return heightMap; }								// Access the bed probing grid
	const GridDefinition& GetGrid() const noexcept { return heightMap.GetGrid(); }			// Get the grid definition

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool LoadHeightMapFromFile(FileStore *f, const char *fname, const StringRef& r) noexcept;	// Load the height map from a file returning true if an error occurred
	bool SaveHeightMapToFile(FileStore *f, const char *fname) noexcept;						// Save the height map to a file returning true if an error occurred
# if SUPPORT_PROBE_POINTS_FILE
	bool LoadProbePointsFromFile(FileStore *f, const char *fname, const StringRef& r) noexcept;	// Load the probe points map from a file returning true if an error occurred
	void ClearProbePointsInvalid() noexcept;
# endif
#endif

#if SUPPORT_ASYNC_MOVES
	void GetPartialMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber, AxesBitmap whichAxes) const noexcept
			pre(queueNumber < NumMovementSystems);							// Get the current position of some axes from one of the rings
	AsyncMove *LockAuxMove() noexcept;														// Get and lock the aux move buffer
	void ReleaseAuxMove(bool hasNewMove) noexcept;											// Release the aux move buffer and optionally signal that it contains a move
	GCodeResult ConfigureHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Configure height following
	GCodeResult StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Start/stop height following
#endif

	const RandomProbePointSet& GetProbePoints() const noexcept { return probePoints; }		// Return the probe point set constructed from G30 commands

	DDARing& GetMainDDARing() noexcept { return rings[0]; }
	float GetTopSpeedMmPerSec() const noexcept { return rings[0].GetTopSpeedMmPerSec(); }
	float GetRequestedSpeedMmPerSec() const noexcept { return rings[0].GetRequestedSpeedMmPerSec(); }
	float GetAccelerationMmPerSecSquared() const noexcept { return rings[0].GetAccelerationMmPerSecSquared(); }
	float GetDecelerationMmPerSecSquared() const noexcept { return rings[0].GetDecelerationMmPerSecSquared(); }
	float GetTotalExtrusionRate() const noexcept { return rings[0].GetTotalExtrusionRate(); }

	const int32_t *GetMotorEndPositions() const noexcept { return motorPositionsAfterScheduledMoves; }
	void SetMotorEndPosition(size_t axis, int32_t val) noexcept { motorPositionsAfterScheduledMoves[axis] = val; }
	void SetAxisEndPosition(size_t axis, float pos) noexcept;
	float LiveMachineCoordinate(unsigned int axisOrExtruder) const noexcept;
	void ForceLiveCoordinatesUpdate() noexcept { forceLiveCoordinatesUpdate = true; }

	void AdjustLeadscrews(const floatc_t corrections[]) noexcept;							// Called by some Kinematics classes to adjust the leadscrews

	int32_t GetAccumulatedExtrusion(size_t logicalDrive, bool& isPrinting) noexcept;		// Return and reset the accumulated commanded extrusion amount

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteResumeSettings(FileStore *f) const noexcept;									// Write settings for resuming the print
#endif

	uint32_t ExtruderPrintingSince() const noexcept;										// When we started doing normal moves after the most recent extruder-only move

	unsigned int GetJerkPolicy() const noexcept { return jerkPolicy; }
	void SetJerkPolicy(unsigned int jp) noexcept { jerkPolicy = jp; }

	// Scanning Z probes
	void SetProbeReadingNeeded() noexcept { probeReadingNeeded = true; }

	int32_t GetStepsTaken(size_t logicalDrive) const noexcept;

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept;			// Get the current step interval for this axis or extruder
#endif

#if SUPPORT_CAN_EXPANSION
	void OnEndstopOrZProbeStatesChanged() noexcept;
#endif

	void Interrupt() noexcept;
	void CheckEndstops(Platform& platform, bool executingMove) noexcept;

	static int32_t MotorMovementToSteps(size_t drive, float coord) noexcept;				// Convert a single motor position to number of steps
	static float MotorStepsToMovement(size_t drive, int32_t endpoint) noexcept;				// Convert number of motor steps to motor position

	// We now use the laser task to take readings from scanning Z probes, so we always need it
	[[noreturn]] void LaserTaskRun() noexcept;

	static void CreateLaserTask() noexcept;													// create the laser task if we haven't already
	static void WakeLaserTask() noexcept;													// wake up the laser task, called at the start of a new move
	static void WakeLaserTaskFromISR() noexcept;											// wake up the laser task, called at the start of a new move

	static void WakeMoveTaskFromISR() noexcept;
	static const TaskBase *GetMoveTaskHandle() noexcept { return &moveTask; }

	static void TimerCallback(CallbackParameter p) noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	enum class MoveState : uint8_t
	{
		idle,			// no moves being executed or in queue, motors are at idle hold
		collecting,		// no moves currently being executed but we are collecting moves ready to execute them
		executing,		// we are executing moves
		timing			// no moves being executed or in queue, motors are at full current
	};

	void BedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;				// Take a position and apply the bed compensations
	void InverseBedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;			// Go from a bed-transformed point back to user coordinates
	void AxisTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;				// Take a position and apply the axis-angle compensations
	void InverseAxisTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;		// Go from an axis transformed point back to user coordinates
	float ComputeHeightCorrection(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;	// Compute the height correction needed at a point, ignoring taper

	const char *GetCompensationTypeString() const noexcept;

	float tanXY() const noexcept { return tangents[0]; }
	float tanYZ() const noexcept { return tangents[1]; }
	float tanXZ() const noexcept { return tangents[2]; }

	void DeactivateDM(size_t drive) noexcept;
	void StepDrivers(Platform& p, uint32_t now) noexcept SPEED_CRITICAL;			// Take one step of the DDA, called by timer interrupt.
	void SimulateSteppingDrivers(Platform& p) noexcept;								// For debugging use
	bool ScheduleNextStepInterrupt() noexcept SPEED_CRITICAL;						// Schedule the next interrupt, returning true if we can't because it is already due
	void StopAxisOrExtruder(bool executingMove, size_t logicalDrive) noexcept;		// stop movement of a drive and recalculate the endpoint
#if SUPPORT_REMOTE_COMMANDS
	void StopDriveFromRemote(size_t drive) noexcept;
#endif
	void StopAllDrivers(bool executingMove) noexcept;								// cancel the current isolated move
	void InsertDM(DriveMovement *dm) noexcept;										// insert a DM into the active list, keeping it in step time order
	void SetDirection(Platform& p, size_t axisOrExtruder, bool direction) noexcept;	// set the direction of a driver, observing timing requirements

#if SUPPORT_CAN_EXPANSION
	uint32_t InsertHiccup(uint32_t whenNextInterruptWanted) noexcept;
#else
	void InsertHiccup(uint32_t whenNextInterruptWanted) noexcept;
#endif

	// Move task stack size
	// 250 is not enough when Move and DDA debug are enabled
	// deckingman's system (MB6HC with CAN expansion) needs at least 365 in 3.3beta3
	static constexpr unsigned int MoveTaskStackWords = 450;
	static Task<MoveTaskStackWords> moveTask;

	static constexpr size_t LaserTaskStackWords = 300;	// stack size in dwords for the laser and IOBits task (increased to support scanning Z probes)
	static Task<LaserTaskStackWords> *laserTask;		// the task used to manage laser power or IOBits

	// Member data
	DDARing rings[NumMovementSystems];

	DriveMovement dms[MaxAxesPlusExtruders + NumDirectDrivers];		// One DriveMovement object per logical drive, plus an extra one for each local driver to support bed levelling moves
	volatile int32_t movementAccumulators[MaxAxesPlusExtruders]; 	// Accumulated motor steps, used by filament monitors
	int32_t motorPositionsAfterScheduledMoves[MaxAxesPlusExtruders];	// The motor positions that will result after all scheduled movement has completed normally

	mutable float latestLiveCoordinates[MaxAxesPlusExtruders];		// the most recent set of live coordinates that we fetched
	mutable uint32_t latestLiveCoordinatesFetchedAt = 0;			// when we fetched the live coordinates
	mutable bool forceLiveCoordinatesUpdate = true;					// true if we want to force latestLiveCoordinates to be updated
	mutable bool liveCoordinatesValid = false;						// true if the latestLiveCoordinates should be valid
	mutable volatile bool motionAdded = false;						// set when any move segments are added

#ifdef DUET3_MB6XD
	volatile uint32_t lastStepHighTime;								// when we last started a step pulse
#else
	volatile uint32_t lastStepLowTime;								// when we last completed a step pulse to a slow driver
#endif
	volatile uint32_t lastDirChangeTime;							// when we last changed the DIR signal to a slow driver

	StepTimer timer;												// Timer object to control getting step interrupts
	DriveMovement *activeDMs;

#if SUPPORT_ASYNC_MOVES
	AsyncMove auxMove;
	volatile bool auxMoveLocked;
	volatile bool auxMoveAvailable;
	HeightController *heightController;
#endif

	SimulationMode simulationMode;						// Are we simulating, or really printing?
	MoveState moveState;								// whether the idle timer is active

	unsigned int jerkPolicy;							// When we allow jerk
	unsigned int idleCount;								// The number of times Spin was called and had no new moves to process

	uint32_t whenLastMoveAdded;							// The time when we last added a move to any DDA ring
	uint32_t whenIdleTimerStarted;						// The approximate time at which the state last changed, except we don't record timing -> idle

	uint32_t idleTimeout;								// How long we wait with no activity before we reduce motor currents to idle, in milliseconds
	uint32_t longestGcodeWaitInterval;					// the longest we had to wait for a new GCode
	uint32_t numHiccups = 0;

	float tangents[3]; 									// Axis compensation - 90 degrees + angle gives angle between axes
	bool compensateXY;									// If true then we compensate for XY skew by adjusting the Y coordinate; else we adjust the X coordinate

	HeightMap heightMap;    							// The grid definition in use and height map for G29 bed probing
	RandomProbePointSet probePoints;					// G30 bed probe points
	float taperHeight;									// Height over which we taper
	float recipTaperHeight;								// Reciprocal of the taper height
	float zShift;										// Height to add to the bed transform

	Deviation latestCalibrationDeviation;
	Deviation initialCalibrationDeviation;
	Deviation latestMeshDeviation;

	Kinematics *kinematics;								// What kinematics we are using

	float minExtrusionPending = 0.0, maxExtrusionPending = 0.0;

	AxisShaper axisShaper;								// the input shaping that we use for axes - currently just one for all axes

	float specialMoveCoords[MaxDriversPerAxis];			// Amounts by which to move individual Z motors (leadscrew adjustment move)

	// Calibration and bed compensation
	uint8_t numCalibratedFactors;
	bool bedLevellingMoveAvailable;						// True if a leadscrew adjustment move is pending
	bool usingMesh;										// True if we are using the height map, false if we are using the random probe point set
	bool useTaper;										// True to taper off the compensation
	bool probeReadingNeeded = false;					// true if the laser task needs to take a Z probe reading
	bool checkingEndstops = false;						// true if we are doing an isolated move that checks endstops
};

//******************************************************************************************************

// Get the current position in untransformed coords
inline void Move::GetCurrentMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber, bool disableMotorMapping) const noexcept
{
	return rings[msNumber].GetCurrentMachinePosition(m, disableMotorMapping);
}

// Update the min and max extrusion pending values. These are reported by M122 to assist with debugging print quality issues.
// Inlined because this is only called from one place.
inline void Move::UpdateExtrusionPendingLimits(float extrusionPending) noexcept
{
	if (extrusionPending > maxExtrusionPending) { maxExtrusionPending = extrusionPending; }
	else if (extrusionPending < minExtrusionPending) { minExtrusionPending = extrusionPending; }
}

#if SUPPORT_ASYNC_MOVES

// Get the current position of some axes from one of the rings
inline void Move::GetPartialMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber, AxesBitmap whichAxes) const noexcept
{
	rings[msNumber].GetPartialMachinePosition(m, whichAxes);
}

#endif

// Set the current position to be this without transforming them first
inline void Move::SetRawPosition(const float positions[MaxAxesPlusExtruders], MovementSystemNumber msNumber) noexcept
{
	rings[msNumber].SetPositions(positions);
}

inline int32_t Move::GetLiveMotorPosition(size_t axis) const noexcept
{
	return dms[axis].GetCurrentMotorPosition();
}

ExtruderShaper& Move::GetExtruderShaperForExtruder(size_t extruder) noexcept
{
	return dms[ExtruderToLogicalDrive(extruder)].extruderShaper;
}

inline float Move::GetPressureAdvanceClocksForLogicalDrive(size_t drive) const noexcept
{
	return dms[drive].extruderShaper.GetKclocks();
}

inline float Move::GetPressureAdvanceClocksForExtruder(size_t extruder) const noexcept
{
	return (extruder < MaxExtruders) ? GetPressureAdvanceClocksForLogicalDrive(ExtruderToLogicalDrive(extruder)) : 0.0;
}

// Schedule the next interrupt, returning true if we can't because it is already due
// Base priority must be >= NvicPriorityStep when calling this
inline __attribute__((always_inline)) bool Move::ScheduleNextStepInterrupt() noexcept
{
	if (activeDMs != nullptr)
	{
		return timer.ScheduleCallbackFromIsr(activeDMs->nextStepTime);
	}
	return false;
}

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance. Now that we generate step pulses
// for multiple motors simultaneously, there is no need to preserve round-robin order.
inline void Move::InsertDM(DriveMovement *dm) noexcept
{
	DriveMovement **dmp = &activeDMs;
	while (*dmp != nullptr && (*dmp)->nextStepTime < dm->nextStepTime)
	{
		dmp = &((*dmp)->nextDM);
	}
	dm->nextDM = *dmp;
	*dmp = dm;
}

#if 0	//TODO remove or fix this code
// Force an end point
inline void Move::SetDriveCoordinate(int32_t a, size_t drive) noexcept
{
	endPoint[drive] = a;
	flags.endCoordinatesValid = false;
}

inline uint32_t Move::ExtruderPrintingSince() const noexcept
{
	return rings[0].ExtruderPrintingSince();	// When we started doing normal moves after the most recent extruder-only move
}

#if SUPPORT_CAN_EXPANSION

// Insert a hiccup, returning the amount of time inserted
// Note, clocksNeeded may be less than WakeupTime but that doesn't matter, the subtraction will wrap around and push the new moveStartTime on a little
inline __attribute__((always_inline)) uint32_t Move::InsertHiccup(uint32_t whenNextInterruptWanted) noexcept
{
	const uint32_t ticksDueAfterStart = (activeDMs != nullptr) ? activeDMs->nextStepTime : clocksNeeded - MoveTiming::WakeupTime;
	const uint32_t oldStartTime = afterPrepare.moveStartTime;
	afterPrepare.moveStartTime = whenNextInterruptWanted - ticksDueAfterStart;
	return afterPrepare.moveStartTime - oldStartTime;
}

#else

// Insert a hiccup
// Note, clocksNeeded may be less than WakeupTime but that doesn't matter, the subtraction will wrap around and push the new moveStartTime on a little
inline __attribute__((always_inline)) void Move::InsertHiccup(uint32_t whenNextInterruptWanted) noexcept
{
	const uint32_t ticksDueAfterStart = (activeDMs != nullptr) ? activeDMs->nextStepTime : clocksNeeded - MoveTiming::WakeupTime;
	afterPrepare.moveStartTime = whenNextInterruptWanted - ticksDueAfterStart;
}

#endif
#endif	//TODO

#if HAS_SMART_DRIVERS

// Get the current step interval for this axis or extruder, or 0 if it is not moving
// This is called from the stepper drivers SPI interface ISR
inline __attribute__((always_inline)) uint32_t Move::GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept
{
	if (simulationMode == SimulationMode::off) { return 0; }
	AtomicCriticalSectionLocker lock;
	return dms[axis].GetStepInterval(microstepShift);
}

#endif

#endif /* MOVE_H_ */
