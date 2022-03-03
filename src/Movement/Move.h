/*
 * Move.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef MOVE_H_
#define MOVE_H_

#include <RepRapFirmware.h>
#include "AxisShaper.h"
#include "ExtruderShaper.h"
#include "DDARing.h"
#include "DDA.h"								// needed because of our inline functions
#include "BedProbing/RandomProbePointSet.h"
#include "BedProbing/Grid.h"
#include "Kinematics/Kinematics.h"
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

// This is the master movement class.  It controls all movement in the machine.
class Move INHERIT_OBJECT_MODEL
{
public:
	Move() noexcept;
	void Init() noexcept;													// Start me up
	void Exit() noexcept;													// Shut down

	[[noreturn]] void MoveLoop() noexcept;									// Main loop called by the Move task

	void GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const noexcept; // Get the current position in untransformed coords
	void GetCurrentUserPosition(float m[MaxAxes], uint8_t moveType, const Tool *tool) const noexcept;
																			// Return the position (after all queued moves have been executed) in transformed coords
	int32_t GetEndPoint(size_t drive) const noexcept;					 	// Get the current position of a motor
	float LiveCoordinate(unsigned int axisOrExtruder, const Tool *tool) noexcept; // Gives the last point at the end of the last complete DDA
	void MoveAvailable() noexcept;											// Called from GCodes to tell the Move task that a move is available
	bool WaitingForAllMovesFinished() noexcept;								// Tell the lookahead ring we are waiting for it to empty and return true if it is
	void DoLookAhead() noexcept SPEED_CRITICAL;			// Run the look-ahead procedure
	void SetNewPosition(const float positionNow[MaxAxesPlusExtruders], bool doBedCompensation) noexcept; // Set the current position to be this
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

	float PushBabyStepping(size_t axis, float amount) noexcept;				// Try to push some babystepping through the lookahead queue

	GCodeResult ConfigureAccelerations(GCodeBuffer&gb, const StringRef& reply) THROWS(GCodeException);		// process M204
	GCodeResult ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// process M595
	GCodeResult ConfigurePressureAdvance(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M572

	float GetPressureAdvanceClocks(size_t extruder) const noexcept;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult EutSetRemotePressureAdvance(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept;
#endif

	float GetMaxPrintingAcceleration() const noexcept { return maxPrintingAcceleration; }
	float GetMaxTravelAcceleration() const noexcept { return maxTravelAcceleration; }
	AxisShaper& GetAxisShaper() noexcept { return axisShaper; }
	ExtruderShaper& GetExtruderShaper(size_t extruder) noexcept { return extruderShapers[extruder]; }

	void Diagnostics(MessageType mtype) noexcept;							// Report useful stuff

	// Kinematics and related functions
	Kinematics& GetKinematics() const noexcept { return *kinematics; }
	bool SetKinematics(KinematicsType k) noexcept;											// Set kinematics, return true if successful
	bool CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const noexcept;
																							// Convert Cartesian coordinates to delta motor coordinates, return true if successful
	void MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept;
																							// Convert motor coordinates to machine coordinates
	void AdjustMotorPositions(const float adjustment[], size_t numMotors) noexcept;			// Perform motor endpoint adjustment
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
	float GetSimulationTime() const noexcept { return mainDDARing.GetSimulationTime(); }	// Get the accumulated simulation time

	bool PausePrint(RestorePoint& rp) noexcept;												// Pause the print as soon as we can, returning true if we were able to
#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT
	bool LowPowerOrStallPause(RestorePoint& rp) noexcept;									// Pause the print immediately, returning true if we were able to
#endif

	bool NoLiveMovement() const noexcept { return mainDDARing.IsIdle(); }					// Is a move running, or are there any queued?

	uint32_t GetScheduledMoves() const noexcept { return mainDDARing.GetScheduledMoves(); }	// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const noexcept { return mainDDARing.GetCompletedMoves(); }	// How many moves have been completed?
	void ResetMoveCounters() noexcept { mainDDARing.ResetMoveCounters(); }

	HeightMap& AccessHeightMap() noexcept { return heightMap; }								// Access the bed probing grid
	const GridDefinition& GetGrid() const noexcept { return heightMap.GetGrid(); }			// Get the grid definition

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool LoadHeightMapFromFile(FileStore *f, const char *fname, const StringRef& r) noexcept;	// Load the height map from a file returning true if an error occurred
	bool SaveHeightMapToFile(FileStore *f, const char *fname) noexcept;						// Save the height map to a file returning true if an error occurred
#endif

	const RandomProbePointSet& GetProbePoints() const noexcept { return probePoints; }		// Return the probe point set constructed from G30 commands

	DDARing& GetMainDDARing() noexcept { return mainDDARing; }
	float GetTopSpeedMmPerSec() const noexcept { return mainDDARing.GetTopSpeedMmPerSec(); }
	float GetRequestedSpeedMmPerSec() const noexcept { return mainDDARing.GetRequestedSpeedMmPerSec(); }
	float GetAccelerationMmPerSecSquared() const noexcept { return mainDDARing.GetAccelerationMmPerSecSquared(); }
	float GetDecelerationMmPerSecSquared() const noexcept { return mainDDARing.GetDecelerationMmPerSecSquared(); }

	void AdjustLeadscrews(const floatc_t corrections[]) noexcept;							// Called by some Kinematics classes to adjust the leadscrews

	int32_t GetAccumulatedExtrusion(size_t drive, bool& isPrinting) noexcept;				// Return and reset the accumulated commanded extrusion amount

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteResumeSettings(FileStore *f) const noexcept;									// Write settings for resuming the print
#endif

	uint32_t ExtruderPrintingSince() const noexcept { return mainDDARing.ExtruderPrintingSince(); }	// When we started doing normal moves after the most recent extruder-only move

	unsigned int GetJerkPolicy() const noexcept { return jerkPolicy; }
	void SetJerkPolicy(unsigned int jp) noexcept { jerkPolicy = jp; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept;			// Get the current step interval for this axis or extruder
#endif

#if SUPPORT_ASYNC_MOVES
	AsyncMove *LockAuxMove() noexcept;														// Get and lock the aux move buffer
	void ReleaseAuxMove(bool hasNewMove) noexcept;											// Release the aux move buffer and optionally signal that it contains a move
	GCodeResult ConfigureHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Configure height following
	GCodeResult StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Start/stop height following
#endif

	static int32_t MotorMovementToSteps(size_t drive, float coord) noexcept;				// Convert a single motor position to number of steps
	static float MotorStepsToMovement(size_t drive, int32_t endpoint) noexcept;				// Convert number of motor steps to motor position

#if SUPPORT_LASER || SUPPORT_IOBITS
	[[noreturn]] void LaserTaskRun() noexcept;

	static void CreateLaserTask() noexcept;													// create the laser task if we haven't already
	static void WakeLaserTask() noexcept;													// wake up the laser task, called at the start of a new move
	static void WakeLaserTaskFromISR() noexcept;											// wake up the laser task, called at the start of a new move
#endif

	static void WakeMoveTaskFromISR() noexcept;

	static const TaskBase *GetMoveTaskHandle() noexcept { return &moveTask; }

#if SUPPORT_REMOTE_COMMANDS
# if USE_REMOTE_INPUT_SHAPING
	void AddShapeddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept		// add a move from the ATE to the movement queue
	{
		mainDDARing.AddMoveFromRemote(msg);
		MoveAvailable();
	}
# else
	void AddMoveFromRemote(const CanMessageMovementLinear& msg) noexcept					// add a move from the ATE to the movement queue
	{
		mainDDARing.AddMoveFromRemote(msg);
		MoveAvailable();
	}
# endif
#endif

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(queue)

#if SUPPORT_COORDINATE_ROTATION
	OBJECT_MODEL_ARRAY(rotationCentre)
#endif

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

	// Move task stack size
	// 250 is not enough when Move and DDA debug are enabled
	// deckingman's system (MB6HC with CAN expansion) needs at least 365 in 3.3beta3
	static constexpr unsigned int MoveTaskStackWords = 450;
	static Task<MoveTaskStackWords> moveTask;

#if SUPPORT_ASYNC_MOVES
	DDARing rings[2];
	DDARing& auxDDARing = rings[1];						// the DDA ring used for live babystepping, height following and other asynchronous moves
	AsyncMove auxMove;
	volatile bool auxMoveLocked;
	volatile bool auxMoveAvailable;
	HeightController *heightController;
#else
	DDARing rings[1];
#endif

	DDARing& mainDDARing = rings[0];					// The DDA ring used for regular moves

	SimulationMode simulationMode;						// Are we simulating, or really printing?
	MoveState moveState;								// whether the idle timer is active

	float maxPrintingAcceleration;
	float maxTravelAcceleration;

	unsigned int jerkPolicy;							// When we allow jerk
	unsigned int idleCount;								// The number of times Spin was called and had no new moves to process

	uint32_t whenLastMoveAdded;							// The time when we last added a move to the main DDA ring
	uint32_t whenIdleTimerStarted;						// The approximate time at which the state last changed, except we don't record timing->idle

	uint32_t idleTimeout;								// How long we wait with no activity before we reduce motor currents to idle, in milliseconds
	uint32_t longestGcodeWaitInterval;					// the longest we had to wait for a new GCode

	float tangents[3]; 									// Axis compensation - 90 degrees + angle gives angle between axes
	float& tanXY = tangents[0];
	float& tanYZ = tangents[1];
	float& tanXZ = tangents[2];
	bool compensateXY;

	HeightMap heightMap;    							// The grid definition in use and height map for G29 bed probing
	RandomProbePointSet probePoints;					// G30 bed probe points
	float taperHeight;									// Height over which we taper
	float recipTaperHeight;								// Reciprocal of the taper height
	float zShift;										// Height to add to the bed transform

	Deviation latestCalibrationDeviation;
	Deviation initialCalibrationDeviation;
	Deviation latestMeshDeviation;


	Kinematics *kinematics;								// What kinematics we are using

	AxisShaper axisShaper;
	ExtruderShaper extruderShapers[MaxExtruders];

	float latestLiveCoordinates[MaxAxesPlusExtruders];
	float specialMoveCoords[MaxDriversPerAxis];			// Amounts by which to move individual Z motors (leadscrew adjustment move)

	uint8_t numCalibratedFactors;
	bool bedLevellingMoveAvailable;						// True if a leadscrew adjustment move is pending
	bool usingMesh;										// True if we are using the height map, false if we are using the random probe point set
	bool useTaper;										// True to taper off the compensation

#if SUPPORT_LASER || SUPPORT_IOBITS
	static constexpr size_t LaserTaskStackWords = 100;	// stack size in dwords for the laser and IOBits task
	static Task<LaserTaskStackWords> *laserTask;		// the task used to manage laser power or IOBits
#endif

};

//******************************************************************************************************

// Get the current position in untransformed coords
inline void Move::GetCurrentMachinePosition(float m[MaxAxes], bool disableMotorMapping) const noexcept
{
	return mainDDARing.GetCurrentMachinePosition(m, disableMotorMapping);
}

// Get the current position of a motor
inline int32_t Move::GetEndPoint(size_t drive) const noexcept
{
	return mainDDARing.GetEndPoint(drive);
}

// Perform motor endpoint adjustment
inline void Move::AdjustMotorPositions(const float adjustment[], size_t numMotors) noexcept
{
	mainDDARing.AdjustMotorPositions(adjustment, numMotors);
}

inline void Move::ResetExtruderPositions() noexcept
{
	mainDDARing.ResetExtruderPositions();
}

inline float Move::GetPressureAdvanceClocks(size_t extruder) const noexcept
{
	return (extruder < MaxExtruders) ? extruderShapers[extruder].GetKclocks() : 0.0;
}

// Get the accumulated extruder motor steps taken by an extruder since the last call. Used by the filament monitoring code.
// Returns the number of motor steps moves since the last call, and sets isPrinting true unless we are currently executing an extruding but non-printing move
inline int32_t Move::GetAccumulatedExtrusion(size_t drive, bool& isPrinting) noexcept
{
	return mainDDARing.GetAccumulatedMovement(drive, isPrinting);
}

#if HAS_SMART_DRIVERS

// Get the current step interval for this axis or extruder, or 0 if it is not moving
// This is called from the stepper drivers SPI interface ISR
inline __attribute__((always_inline)) uint32_t Move::GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept
{
	return (simulationMode == SimulationMode::off) ? mainDDARing.GetStepInterval(axis, microstepShift) : 0;
}

#endif

#endif /* MOVE_H_ */
