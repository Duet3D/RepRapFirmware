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
#include <Hardware/IoPorts.h>
#include <Endstops/EndstopDefs.h>

#if SUPPORT_PHASE_STEPPING
#include <Movement/PhaseStep.h>
#endif

#if SUPPORT_ASYNC_MOVES
# include "HeightControl/HeightController.h"
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageFormats.h>
class CanMessageBuffer;
#endif

constexpr bool DirectionForwards = true;
constexpr bool DirectionBackwards = !DirectionForwards;

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

struct AxisDriversConfig
{
	AxisDriversConfig() noexcept { numDrivers = 0; }
	DriversBitmap GetDriversBitmap() const noexcept;

	uint8_t numDrivers;								// Number of drivers assigned to each axis
	DriverId driverNumbers[MaxDriversPerAxis];		// The driver numbers assigned - only the first numDrivers are meaningful
};

// Type of an axis. The values must correspond to values of the R parameter in the M584 command.
enum class AxisWrapType : uint8_t
{
	noWrap = 0,						// axis does not wrap
	wrapAt360,						// axis wraps, actual position are modulo 360deg
#if 0	// shortcut axes not implemented yet
	wrapWithShortcut,				// axis wraps, G0 moves are allowed to take the shortest direction
#endif
	undefined						// this one must be last
};

#if SUPPORT_NONLINEAR_EXTRUSION

struct NonlinearExtrusion
{
	float A;
	float B;
	float limit;
};

#endif

// Collection of data reported to help debug step errors
struct StepErrorDetails
{
	uint32_t executingStartTime;
	uint32_t executingDuration;
	uint32_t newSegmentStartTime;
	uint32_t timeNow;
	uint8_t stepErrorType;
};

// This is the master movement class.  It controls all movement in the machine.
class Move final INHERIT_OBJECT_MODEL
{
public:
	// Enumeration to describe the status of a drive
	enum class DriverStatus : uint8_t { disabled, idle, enabled };

	Move() noexcept;
	void Init() noexcept;													// Start me up
	void Exit() noexcept;													// Shut down

	[[noreturn]] void MoveLoop() noexcept;									// Main loop called by the Move task

	// Drivers configuration
	size_t GetNumActualDirectDrivers() const noexcept;
	void SetDriversDirection(size_t axisOrExtruder, bool direction) noexcept;
	void SetDirectionValue(size_t driver, bool dVal) noexcept;
	bool GetDirectionValue(size_t driver) const noexcept;
	void SetOneDriverAbsoluteDirection(size_t driver, bool dVal) noexcept;
	void SetEnableValue(size_t driver, int8_t eVal) noexcept;
	int8_t GetEnableValue(size_t driver) const noexcept;
	void EnableDrivers(size_t axisOrExtruder, bool unconditional) noexcept;
	void EnableOneLocalDriver(size_t driver, float requiredCurrent) noexcept;
	void DisableAllDrivers() noexcept;
	void DisableDrivers(size_t axisOrExtruder) noexcept;
	void DisableOneLocalDriver(size_t driver) noexcept;
	void EmergencyDisableDrivers() noexcept;
	void SetDriversIdle() noexcept;

	GCodeResult ConfigureDriverBrakePort(GCodeBuffer& gb, const StringRef& reply, size_t driver) noexcept
		pre(driver < GetNumActualDirectDrivers());
	GCodeResult SetMotorCurrent(size_t axisOrExtruder, float current, int code, const StringRef& reply) noexcept;

	int GetMotorCurrent(size_t axisOrExtruder, int code) const noexcept;
	void SetIdleCurrentFactor(float f) noexcept;
	float GetIdleCurrentFactor() const noexcept { return idleCurrentFactor; }
	uint32_t GetIdleTimeout() const noexcept { return idleTimeout; }
	bool SetDriverMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept;
	bool SetDriversMicrostepping(size_t axisOrExtruder, int microsteps, bool interpolate, const StringRef& reply) noexcept;
	void SetDriverStepTiming(size_t driver, const float microseconds[4]) noexcept;
	bool GetDriverStepTiming(size_t driver, float microseconds[4]) const noexcept;

#if HAS_SMART_DRIVERS
	void SetNumSmartDrivers(size_t p_numSmartDrivers) noexcept { numSmartDrivers = p_numSmartDrivers; }
	unsigned int GetNumSmartDrivers() const noexcept { return numSmartDrivers; }
	static void SpinSmartDrivers(bool driversPowered) noexcept;
	static StandardDriverStatus GetSmartDriverStatus(size_t driver, bool accumulated, bool clearAccumulated) noexcept;
	float GetTmcDriversTemperature(unsigned int boardNumber) const noexcept;
	void DriversJustPoweredUp() noexcept;
	void TurnSmartDriversOff() noexcept;
#endif

#ifdef DUET3_MB6XD
	void GetActualDriverTimings(float timings[4]) noexcept;
#endif

	float NormalAcceleration(size_t axisOrExtruder) const noexcept;
	float Acceleration(size_t axisOrExtruder, bool reduced) const noexcept;
	void SetAcceleration(size_t axisOrExtruder, float value, bool reduced) noexcept;
	float MaxFeedrate(size_t axisOrExtruder) const noexcept;
	const float *_ecv_array MaxFeedrates() const noexcept { return maxFeedrates; }
	void SetMaxFeedrate(size_t axisOrExtruder, float value) noexcept;
	float MinMovementSpeed() const noexcept { return minimumMovementSpeed; }
	void SetMinMovementSpeed(float value) noexcept;
	float GetPrintingInstantDv(size_t axis) const noexcept;
	float GetMaxInstantDv(size_t axis) const noexcept;
	void SetInstantDv(size_t axis, float value, bool includingMax) noexcept;
	float AxisMaximum(size_t axis) const noexcept;
	void SetAxisMaximum(size_t axis, float value, bool byProbing) noexcept;
	float AxisMinimum(size_t axis) const noexcept;
	void SetAxisMinimum(size_t axis, float value, bool byProbing) noexcept;
	float AxisTotalLength(size_t axis) const noexcept;

	GCodeResult ConfigureBacklashCompensation(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M425
	void UpdateBacklashSteps() noexcept;
	int32_t ApplyBacklashCompensation(size_t drive, int32_t delta) noexcept;
	uint32_t GetBacklashCorrectionDistanceFactor() const noexcept { return backlashCorrectionDistanceFactor; }

	inline AxesBitmap GetLinearAxes() const noexcept { return linearAxes; }
	inline AxesBitmap GetRotationalAxes() const noexcept { return rotationalAxes; }
	inline bool IsAxisLinear(size_t axis) const noexcept { return linearAxes.IsBitSet(axis); }
	inline bool IsAxisRotational(size_t axis) const noexcept { return rotationalAxes.IsBitSet(axis); }
	inline bool IsAxisContinuous(size_t axis) const noexcept { return continuousAxes.IsBitSet(axis); }
#if 0	// shortcut axes not implemented yet
	inline bool IsAxisShortcutAllowed(size_t axis) const noexcept { return shortcutAxes.IsBitSet(axis); }
#endif

#if HAS_STALL_DETECT || SUPPORT_CAN_EXPANSION
	GCodeResult ConfigureStallDetection(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf) THROWS(GCodeException);
#endif

	void SetAxisType(size_t axis, AxisWrapType wrapType, bool isNistRotational) noexcept;

	const AxisDriversConfig& GetAxisDriversConfig(size_t axis) const noexcept
		pre(axis < MaxAxes)
		{ return axisDrivers[axis]; }
	void SetAxisDriversConfig(size_t axis, size_t numValues, const DriverId driverNumbers[]) noexcept
		pre(axis < MaxAxes);
	DriverId GetExtruderDriver(size_t extruder) const noexcept
		pre(extruder < MaxExtruders)
		{ return extruderDrivers[extruder]; }
	void SetExtruderDriver(size_t extruder, DriverId driver) noexcept
		pre(extruder < MaxExtruders);

#ifdef DUET3_MB6XD		// the first element has a special meaning when we use a TC to generate the steps
	uint32_t GetSlowDriverStepPeriodClocks() { return stepPulseMinimumPeriodClocks; }
	uint32_t GetSlowDriverDirHoldClocksFromLeadingEdge() { return directionHoldClocksFromLeadingEdge; }
	uint32_t GetSlowDriverDirSetupClocks() const noexcept { return directionSetupClocks; }
#else
	uint32_t GetSlowDriverStepHighClocks() const noexcept { return slowDriverStepTimingClocks[0]; }
	uint32_t GetSlowDriverStepLowClocks() const noexcept { return slowDriverStepTimingClocks[1]; }
	uint32_t GetSlowDriverDirHoldClocksFromTrailingEdge() const noexcept { return slowDriverStepTimingClocks[3]; }
	uint32_t GetSlowDriverDirSetupClocks() const noexcept { return slowDriverStepTimingClocks[2]; }
#endif

	void PollOneDriver(size_t driver) noexcept pre(driver < NumDirectDrivers);

#if VARIABLE_NUM_DRIVERS
	void AdjustNumDrivers(size_t numDriversNotAvailable) noexcept;
#endif

#ifdef DUET3_MB6XD
	bool HasDriverError(size_t driver) const noexcept;
#endif

#if SUPPORT_NONLINEAR_EXTRUSION
	const NonlinearExtrusion& GetExtrusionCoefficients(size_t extruder) const noexcept pre(extruder < MaxExtruders) { return nonlinearExtrusion[extruder]; }
#endif

	float DriveStepsPerMm(size_t axisOrExtruder) const noexcept pre(axisOrExtruder < MaxAxesPlusExtruders) { return driveStepsPerMm[axisOrExtruder]; }
	void SetDriveStepsPerMm(size_t axisOrExtruder, float value, uint32_t requestedMicrostepping) noexcept pre(axisOrExtruder < MaxAxesPlusExtruders);

	void SetAsExtruder(size_t drive, bool isExtruder) noexcept pre(drive < MaxAxesPlusExtruders) { dms[drive].SetAsExtruder(isExtruder); }

	bool SetMicrostepping(size_t axisOrExtruder, int microsteps, bool mode, const StringRef& reply) noexcept pre(axisOrExtruder < MaxAxesPlusExtruders);
	unsigned int GetMicrostepping(size_t axisOrExtruder, bool& interpolation) const noexcept pre(axisOrExtruder < MaxAxesPlusExtruders);
	unsigned int GetMicrostepping(size_t axisOrExtruder) const noexcept pre(axisOrExtruder < MaxAxesPlusExtruders) { return microstepping[axisOrExtruder] & 0x7FFF; }
	bool GetMicrostepInterpolation(size_t axisOrExtruder) const noexcept pre(axisOrExtruder < MaxAxesPlusExtruders) { return (microstepping[axisOrExtruder] & 0x8000) != 0; }
	uint16_t GetRawMicrostepping(size_t axisOrExtruder) const noexcept pre(axisOrExtruder < MaxAxesPlusExtruders) { return microstepping[axisOrExtruder]; }

#if SUPPORT_CAN_EXPANSION
	GCodeResult UpdateRemoteStepsPerMmAndMicrostepping(AxesBitmap axesAndExtruders, const StringRef& reply) noexcept;
#endif

	// Various functions called from GCodes module
	void GetCurrentMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber, bool disableMotorMapping) const noexcept; // Get the current position in untransformed coords
#if SUPPORT_ASYNC_MOVES
	void GetPartialMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber, AxesBitmap whichAxes) const noexcept
			pre(queueNumber < NumMovementSystems);							// Get the current position of some axes from one of the rings
#endif
	void SetRawPosition(const float positions[MaxAxes], MovementSystemNumber msNumber, AxesBitmap axes) noexcept
			pre(msNumber < NumMovementSystems);							// Set the current position to be this without transforming them first
	void AdjustMotorPositions(const float adjustment[], size_t numMotors) noexcept;		// Perform motor endpoint adjustment after auto calibration
	void GetCurrentUserPosition(float m[MaxAxes], MovementSystemNumber msNumber, uint8_t moveType, const Tool *tool) const noexcept;
																			// Return the position (after all queued moves have been executed) in transformed coords
	int32_t GetLiveMotorPosition(size_t driver) const noexcept pre(driver < MaxAxesPlusExtruders);
	void SetMotorPosition(size_t drive, int32_t pos) noexcept pre(drive < MaxAxesPlusExtruders);

	void MoveAvailable() noexcept;											// Called from GCodes to tell the Move task that a move is available
	bool WaitingForAllMovesFinished(MovementSystemNumber msNumber
#if SUPPORT_ASYNC_MOVES
									, AxesBitmap axesAndExtrudersOwned
#endif
								   ) noexcept
		pre(msNumber < rings.upb);										// Tell the lookahead ring we are waiting for it to empty and return true if it is
	void DoLookAhead() noexcept SPEED_CRITICAL;								// Run the look-ahead procedure
	void SetNewPositionOfOwnedAxes(const MovementState& ms, bool doBedCompensation) noexcept;	// Set the current position to be this
	void SetNewPositionOfAllAxes(const MovementState& ms, bool doBedCompensation) noexcept;		// Set the current position to be this
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

	float PushBabyStepping(MovementSystemNumber msNumber, size_t axis, float amount) noexcept;				// Try to push some babystepping through the lookahead queue

	GCodeResult ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);				// process M595
	GCodeResult ConfigurePressureAdvance(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);			// process M572
	GCodeResult ConfigureAxisLimits(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array axisLetters, size_t numTotalAxes, bool inM501) THROWS(GCodeException);	// process M208
	GCodeResult ConfigureNonlinearExtrusion(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// process M592

	ExtruderShaper& GetExtruderShaperForExtruder(size_t extruder) noexcept;
	void ClearExtruderMovementPending(size_t extruder) noexcept;
	float GetPressureAdvanceClocksForLogicalDrive(size_t drive) const noexcept;
	float GetPressureAdvanceClocksForExtruder(size_t extruder) const noexcept;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult EutSetMotorCurrents(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept;
	GCodeResult EutSetStepsPerMmAndMicrostepping(const CanMessageMultipleDrivesRequest<StepsPerUnitAndMicrostepping>& msg, size_t dataLength, const StringRef& reply) noexcept;
	GCodeResult EutHandleSetDriverStates(const CanMessageMultipleDrivesRequest<DriverStateControl>& msg, const StringRef& reply) noexcept;
	GCodeResult EutProcessM569(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult EutProcessM569Point2(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult EutProcessM569Point7(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult EutProcessM915(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	void SendDriversStatus(CanMessageBuffer& buf) noexcept;

	bool InitFromRemote(const CanMessageMovementLinearShaped& msg) noexcept;
	void StopDriversFromRemote(uint16_t whichDrives) noexcept;
	void RevertPosition(const CanMessageRevertPosition& msg) noexcept;

	void AddMoveFromRemote(const CanMessageMovementLinearShaped& msg) noexcept				// add a move to the movement queue when we are in expansion board mode
	{
		rings[0].AddMoveFromRemote(msg);
		MoveAvailable();
	}

	GCodeResult EutSetRemotePressureAdvance(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept;
	GCodeResult EutSetInputShaping(const CanMessageSetInputShapingNew& msg, size_t dataLength, const StringRef& reply) noexcept
	{
		return axisShaper.EutSetInputShaping(msg, dataLength, reply);
	}

	void AppendDiagnostics(const StringRef& reply) noexcept;
#endif

	AxisShaper& GetAxisShaper() noexcept { return axisShaper; }

	// Functions called by DDA::Prepare to generate segments for executing DDAs
	void AddLinearSegments(const DDA& dda, size_t logicalDrive, uint32_t startTime, const PrepParams& params, motioncalc_t steps, MovementFlags moveFlags) noexcept;
	void SetHomingDda(size_t drive, DDA *dda) noexcept pre(drive < MaxAxesPlusExtruders);

	bool AreDrivesStopped(AxesBitmap drives) const noexcept;								// return true if none of the drives passed has any movement pending

	void Diagnostics(MessageType mtype) noexcept;											// Report useful stuff

	// Kinematics and related functions
	Kinematics &_ecv_from GetKinematics() const noexcept { return *kinematics; }
	bool SetKinematics(KinematicsType k) noexcept;											// Set kinematics, return true if successful
	bool CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const noexcept;
																							// Convert Cartesian coordinates to delta motor coordinates, return true if successful
	void MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept;
																							// Convert motor coordinates to machine coordinates
	const char *_ecv_array GetGeometryString() const noexcept { return kinematics->GetName(true); }
	bool IsAccessibleProbePoint(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept;

	bool IsRawMotorMove(uint8_t moveType) const noexcept;									// Return true if this is a raw motor move

	float IdleTimeout() const noexcept;														// Returns the idle timeout in seconds
	void SetIdleTimeout(float timeout) noexcept;											// Set the idle timeout in seconds

	void Simulate(SimulationMode simMode) noexcept;											// Enter or leave simulation mode
	float GetSimulationTime() const noexcept { return rings[0].GetSimulationTime(); }		// Get the accumulated simulation time

	bool PausePrint(MovementState& ms) noexcept;											// Pause the print as soon as we can, returning true if we were able to
#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT
	bool LowPowerOrStallPause(unsigned int queueNumber, RestorePoint& rp) noexcept;			// Pause the print immediately, returning true if we were able to
	void CancelStepping() noexcept;															// Stop generating steps
#endif

	bool NoLiveMovement() const noexcept { return rings[0].IsIdle(); }						// Is a move running, or are there any queued?

	uint32_t GetScheduledMoves() const noexcept { return rings[0].GetScheduledMoves(); }	// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const noexcept { return rings[0].GetCompletedMoves(); }	// How many moves have been completed?
	void ResetMoveCounters() noexcept { rings[0].ResetMoveCounters(); }
	void UpdateExtrusionPendingLimits(float extrusionPending) noexcept;

	HeightMap& AccessHeightMap() noexcept { return heightMap; }								// Access the bed probing grid
	const GridDefinition& GetGrid() const noexcept { return heightMap.GetGrid(); }			// Get the grid definition

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool LoadHeightMapFromFile(FileStore *f, const char *_ecv_array fname, const StringRef& r) noexcept;	// Load the height map from a file returning true if an error occurred
	bool SaveHeightMapToFile(FileStore *f, const char *_ecv_array fname) noexcept;						// Save the height map to a file returning true if an error occurred
# if SUPPORT_PROBE_POINTS_FILE
	bool LoadProbePointsFromFile(FileStore *f, const char *_ecv_array fname, const StringRef& r) noexcept;	// Load the probe points map from a file returning true if an error occurred
	void ClearProbePointsInvalid() noexcept;
# endif
#endif

#if SUPPORT_ASYNC_MOVES
	AsyncMove *LockAuxMove() noexcept;														// Get and lock the aux move buffer
	void ReleaseAuxMove(bool hasNewMove) noexcept;											// Release the aux move buffer and optionally signal that it contains a move
	GCodeResult ConfigureHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// Configure height following
	GCodeResult StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// Start/stop height following
#endif

	const RandomProbePointSet& GetProbePoints() const noexcept { return probePoints; }		// Return the probe point set constructed from G30 commands

	DDARing& GetMainDDARing() noexcept { return rings[0]; }
	float GetTopSpeedMmPerSec() const noexcept { return rings[0].GetTopSpeedMmPerSec(); }
	float GetRequestedSpeedMmPerSec() const noexcept { return rings[0].GetRequestedSpeedMmPerSec(); }
	float GetAccelerationMmPerSecSquared() const noexcept { return rings[0].GetAccelerationMmPerSecSquared(); }		// Get the (peak) acceleration for reporting in the object model
	float GetDecelerationMmPerSecSquared() const noexcept { return rings[0].GetDecelerationMmPerSecSquared(); }		// Get the (peak) deceleration for reporting in the object model
	float GetTotalExtrusionRate() const noexcept { return rings[0].GetTotalExtrusionRate(); }

	float LiveMachineCoordinate(unsigned int axisOrExtruder) const noexcept;				// Get a single coordinate for reporting e.g.in the OM
	void ForceLiveCoordinatesUpdate() noexcept { forceLiveCoordinatesUpdate = true; }		// Force the stored coordinates to be updated next time LiveMachineCoordinate is called

	bool GetLiveMachineCoordinates(float coords[MaxAxes]) const noexcept;					// Get the current machine coordinates, independently of the above functions, so not affected by other tasks calling them

	void AdjustLeadscrews(const floatc_t corrections[]) noexcept;							// Called by some Kinematics classes to adjust the leadscrews

	// Filament monitor support
	int32_t GetAccumulatedExtrusion(size_t logicalDrive, bool& isPrinting) noexcept;		// Return and reset the accumulated commanded extrusion amount
	uint32_t ExtruderPrintingSince(size_t logicalDrive) const noexcept;						// When we started doing normal moves after the most recent extruder-only move

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteResumeSettings(FileStore *f) const noexcept;									// Write settings for resuming the print
	bool WriteMoveParameters(FileStore *f) const noexcept;
#endif

	unsigned int GetJerkPolicy() const noexcept { return jerkPolicy; }
	void SetJerkPolicy(unsigned int jp) noexcept { jerkPolicy = jp; }

#if SUPPORT_SCANNING_PROBES
	void PrepareScanningProbeDataCollection(const DDA& dda, const PrepParams& params) noexcept;
	void ScanningProbeTimerCallback() noexcept;
#endif

	int32_t GetStepsTaken(size_t logicalDrive) const noexcept;

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t drive, uint32_t microstepShift) const noexcept;			// Get the current step interval for this axis or extruder
#endif

#if SUPPORT_CLOSED_LOOP
	bool EnableIfIdle(size_t driver) noexcept;										// if the driver is idle, enable it; return true if driver enabled on return
	void InvertCurrentMotorSteps(size_t driver) noexcept;
#endif

#if SUPPORT_PHASE_STEPPING
	void ConfigurePhaseStepping(size_t axisOrExtruder, float value, PhaseStepConfig config);							// configure Ka & Kv parameters for phase stepping
	PhaseStepParams GetPhaseStepParams(size_t axisOrExtruder);
	bool GetCurrentMotion(size_t driver, uint32_t when, MotionParameters& mParams) noexcept;	// get the net full steps taken, including in the current move so far, also speed and acceleration; return true if moving
	bool SetStepMode(size_t axisOrExtruder, StepMode mode, const StringRef& reply) noexcept;
	StepMode GetStepMode(size_t axisOrExtruder) noexcept;
	void ResetPhaseStepMonitoringVariables() noexcept;

	void PhaseStepControlLoop() noexcept;
#endif

#if SUPPORT_S_CURVE
	void UseSCurve(bool enable) noexcept { usingSCurve = enable; }
	bool IsUsingSCurve() noexcept { return usingSCurve; }
#endif

	void Interrupt() noexcept;

#if SUPPORT_CAN_EXPANSION
	void OnEndstopOrZProbeStatesChanged() noexcept;
	bool CheckEndstops(bool executingMove) noexcept;
#else
	void CheckEndstops(bool executingMove) noexcept;
#endif

	int32_t MotorMovementToSteps(size_t drive, float coord) const noexcept;					// Convert a single motor position to number of steps
	float MotorStepsToMovement(size_t drive, int32_t endpoint) const noexcept;				// Convert number of motor steps to motor position

	void DeactivateDM(DriveMovement *dmToRemove) noexcept;									// remove a DM from the active list

#if HAS_STALL_DETECT
	EndstopValidationResult CheckStallDetectionEnabled(uint8_t axisOrExtruder, float speed, uint8_t& failingDriver) noexcept;	// check that stall detection will work at this speed
#endif

	// Movement error handling
	void LogStepError(uint8_t type) noexcept;												// stop all movement because of a step error
	StepErrorDetails GetStepErrorDetails() const noexcept { return stepErrorDetails; }
	bool HasMovementError() const noexcept;
	void ResetAfterError() noexcept;
	void GenerateMovementErrorDebug() noexcept;
	void AddPrepareHiccup() noexcept { ++numPrepareHiccups; }

	// We now use the laser task to take readings from scanning Z probes, so we always need it
	[[noreturn]] void LaserTaskRun() noexcept;

	static void CreateLaserTask() noexcept;													// create the laser task if we haven't already
	static void WakeLaserTask() noexcept;													// wake up the laser task, called at the start of a new move

	static void WakeMoveTaskFromISR() noexcept;
	static const TaskBase *_ecv_from GetMoveTaskHandle() noexcept { return &moveTask; }

	static void TimerCallback(CallbackParameter p) noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	enum class MoveState : uint8_t
	{
		idle = 0,		// no moves being executed or in queue, motors are at idle hold
		collecting,		// no moves currently being executed but we are collecting moves ready to execute them
		executing,		// we are executing moves
		timing			// no moves being executed or in queue, motors are at full current
	};

	enum class StepErrorState : uint8_t
	{
		noError = 0,	// no error
		haveError,		// had an error, movement is stopped
		resetting		// had an error, ready to reset it
	};

#if SUPPORT_SCANNING_PROBES
	struct ScanningProbeControl
	{
		StepTimer timer;
		size_t numReadingsNeeded = 0;
		size_t nextReadingNeeded = 1;
		uint32_t startTime;
		float distancePerReading;
		float accelDistance;
		float decelStartDistance;
		float acceleration;
		float deceleration;
		float initialSpeed;
		float topSpeed;
		uint32_t accelClocks;
		uint32_t steadyClocks;
		bool readingNeeded = false;
	};
#endif

	MoveSegment *AddSegment(MoveSegment *list, uint32_t startTime, uint32_t duration, motioncalc_t distance, motioncalc_t a J_FORMAL_PARAMETER(j), MovementFlags moveFlags, motioncalc_t pressureAdvance) noexcept;

	void BedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;				// Take a position and apply the bed compensations
	void InverseBedTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;			// Go from a bed-transformed point back to user coordinates
	void AxisTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;				// Take a position and apply the axis-angle compensations
	void InverseAxisTransform(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;		// Go from an axis transformed point back to user coordinates
	float ComputeHeightCorrection(float xyzPoint[MaxAxes], const Tool *tool) const noexcept;	// Compute the height correction needed at a point, ignoring taper
	void UpdateLiveMachineCoordinates() const noexcept;											// force an update of the live machine coordinates
	void SetNewPositionOfSomeAxes(const MovementState& ms, bool doBedCompensation, AxesBitmap axes) noexcept;	// Set the current position to be this

	const char *_ecv_array GetCompensationTypeString() const noexcept;

	float tanXY() const noexcept { return tangents[0]; }
	float tanYZ() const noexcept { return tangents[1]; }
	float tanXZ() const noexcept { return tangents[2]; }

	void StepDrivers(uint32_t now) noexcept SPEED_CRITICAL;							// Take one step of the DDA, called by timer interrupt.
	void PrepareForNextSteps(DriveMovement *stopDm, MovementFlags flags, uint32_t now) noexcept SPEED_CRITICAL;
	void SimulateSteppingDrivers(Platform& p) noexcept;								// For debugging use
	bool ScheduleNextStepInterrupt() noexcept SPEED_CRITICAL;						// Schedule the next interrupt, returning true if we can't because it is already due
	bool StopAxisOrExtruder(bool executingMove, size_t logicalDrive) noexcept;		// stop movement of a drive and recalculate the endpoint
#if SUPPORT_REMOTE_COMMANDS
	void StopDriveFromRemote(size_t drive) noexcept;
#endif
	bool StopAllDrivers(bool executingMove) noexcept;								// cancel the current isolated move
	void InsertDM(DriveMovement *dm) noexcept;										// insert a DM into the active list, keeping it in step time order
	void SetDirection(size_t axisOrExtruder, bool direction) noexcept;				// set the direction of a driver, observing timing requirements

#if SUPPORT_CAN_EXPANSION
	void IterateDrivers(size_t axisOrExtruder, function_ref_noexcept<void(uint8_t) noexcept> localFunc, function_ref_noexcept<void(DriverId) noexcept> remoteFunc) noexcept;
	void IterateLocalDrivers(size_t axisOrExtruder, function_ref_noexcept<void(uint8_t) noexcept> func) noexcept { IterateDrivers(axisOrExtruder, func, [](DriverId) noexcept {}); }
	void IterateRemoteDrivers(size_t axisOrExtruder, function_ref_noexcept<void(DriverId) noexcept> func) noexcept { IterateDrivers(axisOrExtruder, [](uint8_t) noexcept {}, func); }
#else
	void IterateDrivers(size_t axisOrExtruder, function_ref_noexcept<void(uint8_t) noexcept> localFunc) noexcept;
	void IterateLocalDrivers(size_t axisOrExtruder, function_ref_noexcept<void(uint8_t) noexcept> func) noexcept { IterateDrivers(axisOrExtruder, func); }
#endif

	void InternalDisableDriver(size_t driver) noexcept;
	void EngageBrake(size_t driver) noexcept;
	void DisengageBrake(size_t driver) noexcept;

	void UpdateMotorCurrent(size_t driver, float current) noexcept;
	void SetOneDriverDirection(uint8_t driver, bool direction) noexcept pre(driver < NumDirectDrivers);

	StandardDriverStatus GetLocalDriverStatus(size_t driver) const noexcept;

#if defined(DUET3_MB6XD)
	void UpdateDriverTimings() noexcept;
#endif

#if SUPPORT_SCANNING_PROBES
	void SetupNextScanningProbeReading() noexcept;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	static bool WriteAxisLimits(FileStore *f, AxesBitmap axesProbed, const float limits[MaxAxes], int sParam) noexcept;
#endif

	// Move task stack size
	// 250 is not enough when Move and DDA debug are enabled
	// deckingman's system (MB6HC with CAN expansion) needs at least 365 in 3.3beta3
	static constexpr unsigned int MoveTaskStackWords = 450;
	static Task<MoveTaskStackWords> moveTask;

	static constexpr size_t LaserTaskStackWords = 300;				// stack size in dwords for the laser and IOBits task (increased to support scanning Z probes)
	static Task<LaserTaskStackWords> *laserTask;					// the task used to manage laser power or IOBits

	// Member data
	DDARing rings[NumMovementSystems];

	DriveMovement dms[MaxAxesPlusExtruders + NumDirectDrivers];		// One DriveMovement object per logical drive, plus an extra one for each local driver to support bed levelling moves

	float driveStepsPerMm[MaxAxesPlusExtruders];
	uint16_t microstepping[MaxAxesPlusExtruders];					// the microstepping used for each axis or extruder, top bit is set if interpolation enabled

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
	DriveMovement *_ecv_null activeDMs;
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	DriveMovement *_ecv_null phaseStepDMs;

	// These variables monitor how fast the phase stepping control loop is running etc.
	StepTimer::Ticks prevPSControlLoopCallTime;			// The last time the control loop was called
	StepTimer::Ticks minPSControlLoopRuntime;				// The minimum time the control loop has taken to run
	StepTimer::Ticks maxPSControlLoopRuntime;				// The maximum time the control loop has taken to run
	StepTimer::Ticks minPSControlLoopCallInterval;		// The minimum interval between the control loop being called
	StepTimer::Ticks maxPSControlLoopCallInterval;		// The maximum interval between the control loop being called
#endif

#if SUPPORT_S_CURVE
	bool usingSCurve = false;
#endif

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
	unsigned int numInterruptHiccups;					// The number of hiccups inserted by the ISR
	unsigned int numPrepareHiccups;						// The number of hiccups inserted when preparing the move

	uint32_t whenLastMoveAdded[NumMovementSystems];		// The time when we last added a move to each DDA ring
	uint32_t whenIdleTimerStarted;						// The approximate time at which the state last changed, except we don't record timing -> idle

	uint32_t idleTimeout;								// How long we wait with no activity before we reduce motor currents to idle, in milliseconds
	uint32_t longestGcodeWaitInterval;					// the longest we had to wait for a new GCode

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

	Kinematics *_ecv_from kinematics;					// What kinematics we are using

	float minExtrusionPending = 0.0, maxExtrusionPending = 0.0;

	AxisShaper axisShaper;								// the input shaping that we use for axes - currently just one for all axes

	float specialMoveCoords[MaxDriversPerAxis];			// Amounts by which to move individual Z motors (leadscrew adjustment move)

	// Drives
#if VARIABLE_NUM_DRIVERS && SUPPORT_DIRECT_LCD
	size_t numActualDirectDrivers;
#endif


#if HAS_SMART_DRIVERS
	size_t numSmartDrivers;											// the number of TMC drivers we have, the remaining are simple enable/step/dir drivers
	DriversBitmap temperatureShutdownDrivers, temperatureWarningDrivers, shortToGroundDrivers;
# if HAS_STALL_DETECT
	DriversBitmap logOnStallDrivers, eventOnStallDrivers;
# endif
	MillisTimer openLoadTimers[MaxSmartDrivers];
#endif

	StandardDriverStatus lastEventStatus[NumDirectDrivers];
	bool directions[NumDirectDrivers];
	int8_t enableValues[NumDirectDrivers];

#ifdef DUET3_MB6XD
	bool driverErrPinsActiveLow;
#endif

	// Stepper motor brake control
#if SUPPORT_BRAKE_PWM
	PwmPort brakePorts[NumDirectDrivers];					// the brake ports for each driver
	float brakeVoltages[NumDirectDrivers];
	static constexpr float FullyOnBrakeVoltage = 100.0;		// this value means always use full voltage (don't PWM)
	float currentBrakePwm[NumDirectDrivers];
#else
	IoPort brakePorts[NumDirectDrivers];					// the brake ports for each driver
#endif
	MillisTimer brakeOffTimers[NumDirectDrivers];
	MillisTimer motorOffTimers[NumDirectDrivers];
	uint16_t brakeOffDelays[NumDirectDrivers];				// how many milliseconds we wait between energising the driver and energising the brake
	uint16_t motorOffDelays[NumDirectDrivers];				// how many milliseconds we wait between de-energising the brake (to turn it on) and de-energising the driver

	float motorCurrents[MaxAxesPlusExtruders];				// the normal motor current for each stepper driver
	float motorCurrentFraction[MaxAxesPlusExtruders];		// the percentages of normal motor current that each driver is set to
	float standstillCurrentPercent[MaxAxesPlusExtruders];	// the percentages of normal motor current that each driver uses when in standstill

	volatile DriverStatus driverState[MaxAxesPlusExtruders];
	float maxFeedrates[MaxAxesPlusExtruders];				// max feed rates in mm per step clock
	float normalAccelerations[MaxAxesPlusExtruders];		// max accelerations in mm per step clock squared for normal moves
	float reducedAccelerations[MaxAxesPlusExtruders];		// max accelerations in mm per step clock squared for probing and stall detection moves
	float printingInstantDvs[MaxAxesPlusExtruders];			// current max jerk in mm per step clock (changed by M205 and M206)
	float maxInstantDvs[MaxAxesPlusExtruders];				// max jerk in mm per step clock (changed by M206 only)

	AxisDriversConfig axisDrivers[MaxAxes];					// the driver numbers assigned to each axis
	AxesBitmap linearAxes;									// axes that behave like linear axes w.r.t. feedrate handling
	AxesBitmap rotationalAxes;								// axes that behave like rotational axes w.r.t. feedrate handling
	AxesBitmap continuousAxes;								// axes that wrap modulo 360
#if 0	// shortcut axes not implemented yet
	AxesBitmap shortcutAxes;								// axes that wrap modulo 360 and for which G0 may choose the shortest direction
#endif

	// Axes and endstops
	float axisMaxima[MaxAxes];
	float axisMinima[MaxAxes];
	AxesBitmap axisMinimaProbed, axisMaximaProbed;

	// Backlash compensation user configuration
	float backlashMm[MaxAxes];								// amount of backlash in mm for each axis motor
	uint32_t backlashCorrectionDistanceFactor;				// what multiple of the backlash we apply the correction over

	// Backlash compensation system variables
	uint32_t backlashSteps[MaxAxes];						// the backlash converted to microsteps
	int32_t backlashStepsDue[MaxAxes];						// how many backlash compensation microsteps are due for each axis
	AxesBitmap lastDirections;								// each bit is set if the corresponding axes motor last moved backwards

#if SUPPORT_NONLINEAR_EXTRUSION
	NonlinearExtrusion nonlinearExtrusion[MaxExtruders];	// nonlinear extrusion coefficients
#endif

	DriverId extruderDrivers[MaxExtruders];					// the driver number assigned to each extruder
#ifdef DUET3_MB6XD
	float driverTimingMicroseconds[NumDirectDrivers][4];	// step high time, step low time, direction setup time to step high, direction hold time from step low (1 set per driver)
	uint32_t stepPulseMinimumPeriodClocks;					// minimum period between leading edges of step pulses, in step clocks
	uint32_t directionSetupClocks;							// minimum direction change to step high time, in step clocks
	uint32_t directionHoldClocksFromLeadingEdge;			// minimum step high to direction low step clocks, calculated from the step low to direction change hold time
	const Pin *ENABLE_PINS;									// 6XD version 0.1 uses different enable pins from version 1.0 and later
#else
	uint32_t slowDriversBitmap;								// bitmap of driver port bits that need extended step pulse timing
	uint32_t slowDriverStepTimingClocks[4];					// minimum step high, step low, dir setup and dir hold timing for slow drivers
#endif

	float idleCurrentFactor;
	float minimumMovementSpeed;								// minimum allowed movement speed in mm per step clock

#if SUPPORT_SCANNING_PROBES
	ScanningProbeControl probeControl;
#endif

	// Calibration and bed compensation
	uint8_t numCalibratedFactors;
	bool bedLevellingMoveAvailable;						// True if a leadscrew adjustment move is pending
	bool usingMesh;										// True if we are using the height map, false if we are using the random probe point set
	bool useTaper;										// True to taper off the compensation

	// Reporting of step errors
	volatile StepErrorState stepErrorState = StepErrorState::noError;
	StepErrorDetails stepErrorDetails;
};

//******************************************************************************************************

inline float Move::NormalAcceleration(size_t drive) const noexcept
{
	return normalAccelerations[drive];
}

inline float Move::Acceleration(size_t drive, bool useReduced) const noexcept
{
	return (useReduced) ? min<float>(reducedAccelerations[drive], normalAccelerations[drive]) : normalAccelerations[drive];
}

inline void Move::SetAcceleration(size_t drive, float value, bool reduced) noexcept
{
	((reduced) ? reducedAccelerations : normalAccelerations)[drive] = max<float>(value, ConvertAcceleration(MinimumAcceleration));	// don't allow zero or negative
}

inline float Move::MaxFeedrate(size_t drive) const noexcept
{
	return maxFeedrates[drive];
}

inline void Move::SetMaxFeedrate(size_t drive, float value) noexcept
{
	maxFeedrates[drive] = max<float>(value, minimumMovementSpeed);						// don't allow zero or negative, but do allow small values
}

inline void Move::SetMinMovementSpeed(float value) noexcept
{
	minimumMovementSpeed = max<float>(value, ConvertSpeedFromMmPerSec(AbsoluteMinFeedrate));
}

inline float Move::GetPrintingInstantDv(size_t drive) const noexcept
{
	return printingInstantDvs[drive];
}

inline float Move::GetMaxInstantDv(size_t drive) const noexcept
{
	return maxInstantDvs[drive];
}

inline size_t Move::GetNumActualDirectDrivers() const noexcept
{
#if VARIABLE_NUM_DRIVERS
	return numActualDirectDrivers;
#else
	return NumDirectDrivers;
#endif
}

#if VARIABLE_NUM_DRIVERS

inline void Move::AdjustNumDrivers(size_t numDriversNotAvailable) noexcept
{
	numActualDirectDrivers = NumDirectDrivers - numDriversNotAvailable;
}

#endif

inline void Move::SetDirectionValue(size_t drive, bool dVal) noexcept
{
#if SUPPORT_PHASE_STEPPING
	// We must prevent the tmc task loop fetching the current position while we are changing the direction
	if (directions[drive] != dVal)
	{
		TaskCriticalSectionLocker lock;
		directions[drive] = dVal;
	}
#else
	directions[drive] = dVal;
#endif
}

inline bool Move::GetDirectionValue(size_t drive) const noexcept
{
	return directions[drive];
}

inline void Move::SetOneDriverDirection(uint8_t driver, bool direction) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
		const bool d = (direction == DirectionForwards) ? directions[driver] : !directions[driver];
#if SAME5x
		IoPort::WriteDigital(DIRECTION_PINS[driver], d);
#else
		digitalWrite(DIRECTION_PINS[driver], d);
#endif
	}
}

inline void Move::SetOneDriverAbsoluteDirection(size_t driver, bool direction) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
#if SAME5x
		IoPort::WriteDigital(DIRECTION_PINS[driver], direction);
#else
		digitalWrite(DIRECTION_PINS[driver], direction);
#endif
	}
}

inline int8_t Move::GetEnableValue(size_t driver) const noexcept
{
	return enableValues[driver];
}

inline float Move::AxisMaximum(size_t axis) const noexcept
{
	return axisMaxima[axis];
}

inline float Move::AxisMinimum(size_t axis) const noexcept
{
	return axisMinima[axis];
}

inline float Move::AxisTotalLength(size_t axis) const noexcept
{
	return axisMaxima[axis] - axisMinima[axis];
}

// Get the current position in untransformed coords
inline void Move::GetCurrentMachinePosition(float m[MaxAxes], MovementSystemNumber msNumber, bool disableMotorMapping) const noexcept
{
	rings[msNumber].GetCurrentMachinePosition(m, disableMotorMapping);
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
inline void Move::SetRawPosition(const float positions[MaxAxes], MovementSystemNumber msNumber, AxesBitmap axes) noexcept
{
	rings[msNumber].SetPositions(*this, positions, axes);
	liveCoordinatesValid = false;											// force the live XYZ position to be recalculated
}

// Adjust the motor endpoints without moving the motors. Called after auto-calibrating a linear delta or rotary delta machine.
// There must be no pending movement when calling this!
inline void Move::AdjustMotorPositions(const float adjustment[], size_t numMotors) noexcept
{
	rings[0].AdjustMotorPositions(*this, adjustment, numMotors);
	liveCoordinatesValid = false;											// force the live XYZ position to be recalculated
}

inline int32_t Move::GetLiveMotorPosition(size_t driver) const noexcept
{
	return dms[driver].currentMotorPosition;
}

inline ExtruderShaper& Move::GetExtruderShaperForExtruder(size_t extruder) noexcept
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
		return timer.ScheduleMovementCallbackFromIsr(activeDMs->nextStepTime);
	}
	return false;
}

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance.
// Now that we generate step pulses for multiple motors simultaneously, there is no need to preserve round-robin order.
// Base priority must be >= NvicPriorityStep when calling this, unless we are simulating.
inline void Move::InsertDM(DriveMovement *dm) noexcept
{
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	DriveMovement *_ecv_null *dmp = dm->state == DMState::phaseStepping ? &phaseStepDMs : &activeDMs;
#else
	DriveMovement *_ecv_null *dmp = &activeDMs;
#endif
	while (*dmp != nullptr && (int32_t)((*dmp)->nextStepTime - dm->nextStepTime) < 0)
	{
		dmp = &((*dmp)->nextDM);
	}
	dm->nextDM = *dmp;
	*dmp = dm;
}

inline void Move::LogStepError(uint8_t type) noexcept
{
	stepErrorDetails.stepErrorType = type;
	stepErrorState = StepErrorState::haveError;
}

inline bool Move::HasMovementError() const noexcept
{
	return stepErrorState == StepErrorState::haveError;
}

inline void Move::ResetAfterError() noexcept
{
	if (HasMovementError())
	{
		stepErrorState = StepErrorState::resetting;
	}
}

inline void Move::SetNewPositionOfOwnedAxes(const MovementState& ms, bool doBedCompensation) noexcept
{
#if SUPPORT_ASYNC_MOVES
	SetNewPositionOfSomeAxes(ms, doBedCompensation, ms.GetAxesAndExtrudersOwned());
#else
	SetNewPositionOfAllAxes(ms, doBedCompensation);
#endif
}

#if HAS_SMART_DRIVERS

// Get the current step interval for this axis or extruder, or 0 if it is not moving
// This is called from the stepper drivers SPI interface ISR
inline __attribute__((always_inline)) uint32_t Move::GetStepInterval(size_t drive, uint32_t microstepShift) const noexcept
{
	if (likely(simulationMode == SimulationMode::off))
	{
		AtomicCriticalSectionLocker lock;
		return dms[drive].GetStepInterval(microstepShift);
	}
	return 0;
}

#endif

#if SUPPORT_CLOSED_LOOP

// Invert the current number of microsteps taken. Called when the driver direction control is changed.
inline void Move::InvertCurrentMotorSteps(size_t driver) noexcept
{
	dms[driver].currentMotorPosition = -dms[driver].currentMotorPosition;
}

#endif

#endif /* MOVE_H_ */
