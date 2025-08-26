/*
 * DDA.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef DDA_H_
#define DDA_H_

#include <RepRapFirmware.h>
#include "DriveMovement.h"
#include "StepTimer.h"
#include "MoveSegment.h"
#include "MovementError.h"
#include <Platform/Tasks.h>
#include "RawMove.h"
#include <GCodes/SimulationMode.h>

# define DDA_LOG_PROBE_CHANGES	0
# define DDA_DEBUG_STEP_COUNT	0

class DDARing;
class CanMessageMovementLinearShaped;

struct MoveProfile
{
	float startSpeed;												// the speed at the start of the move. Valid for the first un-commited move in the queue.
	float topSpeed;													// top speed of the move. Valid???
	float endSpeed;													// end speed of the move. Valid (and zero) for the last move in the queue
#if SUPPORT_S_CURVE
	float startAcceleration;										// the acceleration or deceleration at the start of this move, may be positive or negative. Valid for the first un-commited move in the queue.
	float peakAcceleration;											// the acceleration in the steady acceleration phase, if any. Valid if phase1Distance != 0.
    float peakDeceleration;											// the deceleration in the steady deceleration phase, if any. This is negative if there is a peak deceleration phase. Valid if phase5Distance != 0.
    float endAcceleration;											// the acceleration or deceleration at the end of the move. Valid (and zero) for the last move in the queue.
    float distances[7];												// the distances of each phase
#endif
};

// Struct for passing parameters to the DriveMovement Prepare methods, also accessed by the input shaper
struct PrepParams
{
#if SUPPORT_S_CURVE
	uint32_t phase0Clocks, phase1Clocks, phase2Clocks, steadyClocks, phase4Clocks, phase5Clocks, phase6Clocks;
    float initialAcceleration, peakAcceleration;	// the accelerations, always positive
    float initialDeceleration, peakDeceleration;	// the decelerations, always negative
    float distances[7];								// the distances of each phase
	float jerk;										// the magnitude of the rate of change of acceleration or deceleration, always positive; or zero if not using S-curve acceleration
#else
	uint32_t accelClocks, steadyClocks, decelClocks;
	float acceleration;								// the acceleration to use, always positive
	float deceleration;								// the deceleration to use, always negative
# define peakAcceleration	acceleration
# define peakDeceleration	deceleration
	float accelDistance;
	float decelStartDistance;
	float totalDistance;
#endif
	float topSpeed;									// the top speed reached
	bool useInputShaping;

#if SUPPORT_S_CURVE
	uint32_t TotalAccelClocks() const noexcept { return phase0Clocks + phase1Clocks + phase2Clocks; }
	uint32_t TotalDecelClocks() const noexcept { return phase4Clocks + phase5Clocks + phase6Clocks; }
	float TotalAccelDistance() const noexcept { return distances[0] + distances[1] + distances[2]; }
	float TotalDecelDistance() const noexcept { return distances[4] + distances[5] + distances[6]; }
#else
	uint32_t TotalAccelClocks() const noexcept { return accelClocks; }
	uint32_t TotalDecelClocks() const noexcept { return decelClocks; }
	float TotalAccelDistance() const noexcept { return accelDistance; }
#endif

	// Get the total clocks needed
	uint32_t TotalClocks() const noexcept { return TotalAccelClocks() + steadyClocks + TotalDecelClocks(); }

	// Set up the parameters from the DDA, excluding steadyClocks because that may be affected by input shaping
	void SetFromDDA(const DDA& dda) noexcept;

	void DebugPrint() const noexcept;
};

#if SUPPORT_S_CURVE
struct MultipleMoveParameters;
#endif

// This defines a single coordinated movement of one or several motors
class DDA final
{
	friend class DriveMovement;
	friend class ExtruderShaper;
	friend class PrepParams;
	friend class Move;

public:

	enum DDAState : uint8_t
	{
		empty,				// empty or being filled in
#if SUPPORT_S_CURVE
		created,			// filled in but not yet planned
#endif
		planned,			// ready, but could be subject to modifications
		committed			// has been converted into move segments already
	};

	explicit DDA(DDA *_ecv_null n) noexcept;

	void* operator new(size_t count) { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	MovementError InitStandardMove(DDARing& ring, const RawMove &nextMove, bool doMotorMapping) noexcept SPEED_CRITICAL;	// Set up a new move, returning true if it represents real movement
	bool InitLeadscrewMove(DDARing& ring, float feedrate, const float amounts[MaxDriversPerAxis]) noexcept;		// Set up a leadscrew motor move
#if SUPPORT_ASYNC_MOVES
	bool InitAsyncMove(DDARing& ring, const AsyncMove& nextMove) noexcept;							// Set up an async move
#endif

	void SetNext(DDA *n) noexcept { next = n; }
	void SetPrevious(DDA *p) noexcept { prev = p; }
	bool Free() noexcept;
	void Prepare(DDARing& ring, uint32_t prepareAdvanceTime, SimulationMode simMode) noexcept SPEED_CRITICAL;	// Calculate all the values and freeze this DDA
	bool CanPauseAfter() const noexcept;
	bool IsPrintingMove() const noexcept { return flags.isPrintingMove; }							// Return true if this involves both XY movement and extrusion
	bool UsingStandardFeedrate() const noexcept { return flags.usingStandardFeedrate; }
	bool IsCheckingEndstops() const noexcept { return flags.checkEndstops; }
	bool IsIsolatedMove() const noexcept { return flags.isolatedMove; }
	bool NoShaping() const noexcept { return flags.isolatedMove; }

#if SUPPORT_SCANNING_PROBES
	bool IsScanningProbeMove() const noexcept { return flags.scanningProbeMove; }
#endif

	DDAState GetState() const noexcept { return state; }
	bool IsCommitted() const noexcept { return state == DDA::committed; }
	bool IsProvisional() const noexcept;
	DDA* GetNext() const noexcept { return next; }
	DDA* GetPrevious() const noexcept { return prev; }
	uint32_t GetTimeLeft() const noexcept;

	const int32_t *_ecv_array DriveCoordinates() const noexcept { return endPoint; }				// Get endpoints of a move in machine coordinates
	void SetDriveCoordinate(size_t drive, int32_t ep) noexcept;										// Force an end point
	void SetFeedRate(float rate) noexcept { requestedSpeed = rate; }
	void GetEndCoordinates(float returnedCoords[MaxAxes]) noexcept;					// Calculate the machine axis coordinates (after bed and skew correction) at the end of this move

	FilePosition GetFilePosition() const noexcept { return filePos; }
	float GetRequestedSpeedMmPerClock() const noexcept { return requestedSpeed; }
	float GetRequestedSpeedMmPerSec() const noexcept { return InverseConvertSpeedToMmPerSec(requestedSpeed); }
	float GetTopSpeedMmPerSec() const noexcept { return InverseConvertSpeedToMmPerSec(profile.topSpeed); }
	float GetAccelerationMmPerSecSquared() const noexcept							// Get the (peak) acceleration for reporting in the object model
#if SUPPORT_S_CURVE
		{ return InverseConvertAcceleration(profile.peakAcceleration); }
#else
		{ return InverseConvertAcceleration(maxAcceleration); }
#endif
	float GetDecelerationMmPerSecSquared() const noexcept							// Get the (peak) acceleration for reporting in the object model
#if SUPPORT_S_CURVE
		{ return InverseConvertAcceleration(profile.peakDeceleration); }
#else
		{ return InverseConvertAcceleration(maxAcceleration); }
#endif
	float GetVirtualExtruderPosition() const noexcept { return virtualExtruderPosition; }
	float GetTotalExtrusionRate() const noexcept;

#if SUPPORT_S_CURVE
	bool IsSCurveMove() const noexcept { return flags.useScurve; }
	bool IsFullyPlanned() const noexcept { return flags.fullyPlanned; }
#endif

	float AdvanceBabyStepping(DDARing& ring, size_t axis, float amount) noexcept;	// Try to push babystepping earlier in the move queue
	const Tool *_ecv_null GetTool() const noexcept { return tool; }
	float GetTotalDistance() const noexcept { return totalDistance; }
	void LimitSpeedAndAcceleration(float maxSpeed, float maxAllowedAcceleration) noexcept;	// Limit the speed an acceleration of this move

	float GetProportionDone() const noexcept;										// Return the proportion of extrusion for the complete multi-segment move already done
	float GetInitialUserC0() const noexcept { return initialUserC0; }
	float GetInitialUserC1() const noexcept { return initialUserC1; }

	uint32_t GetClocksNeeded() const noexcept { return clocksNeeded; }
	bool HasExpired() const noexcept pre(IsCommitted());
	bool IsNonPrintingExtruderMove() const noexcept { return flags.isNonPrintingExtruderMove; }
	void UpdateMovementAccumulators(volatile int32_t *accumulators) const noexcept;
	uint32_t GetMoveStartTime() const noexcept { return afterPrepare.moveStartTime; }
	uint32_t GetMoveFinishTime() const noexcept { return afterPrepare.moveStartTime + clocksNeeded; }

	float GetAverageExtrusionSpeed() const noexcept pre(IsCommitted()) { return afterPrepare.averageExtrusionSpeed; }
	bool HaveDoneIoBits() const noexcept { return flags.doneIoBits; }
	bool HaveDoneFeedForward() const noexcept { return flags.doneFeedForward; }
	bool HaveDoneOutputOnExtrude() const noexcept { return flags.doneOutputOnExtrude; }
	void SetDoneIoBits() noexcept { flags.doneIoBits = true; }
	void SetDoneFeedForward() noexcept { flags.doneFeedForward = true; }
	void SetDoneOutputOnExtrude() noexcept { flags.doneOutputOnExtrude = true; }

#if SUPPORT_S_CURVE
	void SetSpeedRatioForPrintingMoves(const Move& move) noexcept;
	static void PlanMoves(DDA *firstUnpreparedMove, bool stopping) noexcept;
#endif

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits GetLaserPwmOrIoBits() const noexcept { return laserPwmOrIoBits; }
#endif

#if SUPPORT_LASER
	uint32_t ManageLaserPower(Platform& p) const noexcept;					// Manage the laser power
#endif

#if SUPPORT_IOBITS
	IoBits_t GetIoBits() const noexcept { return laserPwmOrIoBits.ioBits; }
#endif

	void DebugPrint(const char *_ecv_array tag) const noexcept;				// print the DDA only

	static void PrintMoves() noexcept;										// print saved moves for debugging

#if DDA_LOG_PROBE_CHANGES
	static const size_t MaxLoggedProbePositions = 40;
	static size_t numLoggedProbePositions;
	static int32_t loggedProbePositions[XYZ_AXES * MaxLoggedProbePositions];
#endif

private:
	static constexpr float MinimumAccelOrDecelClocks = 10.0;				// Minimum number of acceleration or deceleration clocks we try to ensure

	MovementError RecalculateMove(DDARing& ring) noexcept SPEED_CRITICAL;
	static void DoLookahead(DDARing& ring, DDA *laDDA) noexcept SPEED_CRITICAL;	// Try to smooth out moves in the queue

#if SUPPORT_S_CURVE
	void CalculateIsolatedSCurveMove() noexcept SPEED_CRITICAL pre(endSpeed == 0.0; endDeceleration == 0.0);
	static void DistributePlanOverMoves(DDA *startMove, DDA *endMove, float distanceToPlan, const MultipleMoveParameters& accelParams, const MultipleMoveParameters& decelParams) noexcept;
#endif

	void MatchSpeeds() noexcept SPEED_CRITICAL;
	bool IsDecelerationMove() const noexcept;								// return true if this move is or have been might have been intended to be a deceleration-only move
	bool IsAccelerationMove() const noexcept;								// return true if this move is or have been might have been intended to be an acceleration-only move
	void DebugPrintVector(const char *_ecv_array name, const float *_ecv_array vec, size_t len) const noexcept;

#if SUPPORT_CAN_EXPANSION
	int32_t PrepareRemoteExtruder(size_t drive, float& extrusionPending, float speedChange) const noexcept;
#endif

    static float Normalise(float v[], AxesBitmap unitLengthAxes) noexcept;  // Normalise a vector to unit length over the specified axes
    static float Normalise(float v[]) noexcept; 							// Normalise a vector to unit length over all axes
	float NormaliseLinearMotion(AxesBitmap linearAxes) noexcept;			// Make the direction vector unit-normal in XYZ
    static void Absolute(float v[], size_t dimensions) noexcept;			// Put a vector in the positive hyperquadrant

    static float Magnitude(const float v[]) noexcept;						// Get the magnitude measured over all axes and extruders
    static float Magnitude(const float v[], AxesBitmap axes) noexcept;  	// Return the length of a vector over the specified orthogonal axes
    static void Scale(float v[], float scale) noexcept;						// Multiply a vector by a scalar
    static float VectorBoxIntersection(const float v[], const float box[]) noexcept;	// Compute the length that a vector would have to have to just touch the surface of a hyperbox of MaxAxesPlusExtruders dimensions.

    DDA *_ecv_null next;							// The next one in the ring
	DDA *_ecv_null prev;							// The previous one in the ring

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;				// laser PWM required or port state required during this move (here because it is currently 16 bits)
#endif

	volatile DDAState state;						// What state this DDA is in

	union
	{
		struct
		{
			uint32_t canPauseAfter : 1,				// True if we can pause at the end of this move
					 isPrintingMove : 1,			// True if this move includes XY movement and extrusion
					 usePressureAdvance : 1,		// True if pressure advance should be applied to any forward extrusion
					 hadLookaheadUnderrun : 1,		// True if the lookahead queue was not long enough to optimise this move
					 xyMoving : 1,					// True if movement along an X axis or a Y axis was requested, even if it's too small to do
					 isLeadscrewAdjustmentMove : 1,	// True if this is a leadscrews adjustment move
					 usingStandardFeedrate : 1,		// True if this move uses the standard feed rate
					 isNonPrintingExtruderMove : 1,	// True if this move is an extruder-only move, or involves reverse extrusion (and possibly axis movement too)
					 continuousRotationShortcut : 1, // True if continuous rotation axes take shortcuts
					 checkEndstops : 1,				// True if this move monitors endstops or Z probe
					 controlLaserOrIoBits : 1,		// True if this move controls the laser or iobits
					 isolatedMove : 1,				// set if we disable input shaping for this move and wait for it to finish e.g. for a G1 H2 move
					 doneIoBits : 1,				// set if we have written the IOBITS ports for this move
					 doneFeedForward : 1,			// set if we have commanded feedforward for this move
					 doneOutputOnExtrude: 1			// set if we have set/cleared output on extrude for this move
#if SUPPORT_SCANNING_PROBES
					 , scanningProbeMove : 1 	 	// True if this is a scanning Z probe move
#endif
#if SUPPORT_S_CURVE
					 , useScurve : 1,				// set if this move uses S-curve acceleration
					 fullyPlanned : 1				// set if this move can't be made to go any faster even if we add more moves to the ring
#endif
					 ;
		};
		uint32_t all;								// so that we can print all the flags at once for debugging
	} flags;

	const Tool *_ecv_null tool;						// which tool (if any) is active

    FilePosition filePos;							// The position in the SD card file after this move was read, or zero if not read from SD card

	int32_t endPoint[MaxAxesPlusExtruders];  		// Machine coordinates of the endpoint
	float directionVector[MaxAxesPlusExtruders];	// The normalised direction vector - first 3 are XYZ Cartesian coordinates even on a delta
    float totalDistance;							// How long is the move in hypercuboid space
    float maxAcceleration;							// The maximum acceleration and deceleration to use, always positive
#if SUPPORT_S_CURVE
	float jerk;										// The magnitude of the rate of change of acceleration or deceleration, always positive
#endif
    float requestedSpeed;							// The speed that the user asked for
    float virtualExtruderPosition;					// the virtual extruder position at the end of this move, used for pause/resume

    // These vary depending on how we connect the move with its predecessor and successor, but remain constant while the move is being executed
    MoveProfile profile;

	float proportionDone;							// what proportion of the extrusion in the G1 or G0 move of which this is a part has been done after this segment is complete
	float initialUserC0, initialUserC1;				// if this is a segment of an arc move, the user X and Y coordinates at the start
	uint32_t clocksNeeded;

#if SUPPORT_ASYNC_MOVES
	LogicalDrivesBitmap ownedDrives;				// logical drives we are allowed to move
#endif

	union
	{
		// Values that are needed only before Prepare is called and in the first few lines of Prepare
		struct
		{
			float accelDistance;
			float decelDistance;
			float targetNextSpeed;					// The speed that the next move would like to start at, used to keep track of the lookahead without making recursive calls
#if SUPPORT_S_CURVE
			float startSpeedRatio;					// the ratio of start speed of this move to the end speed of the previous move needed to maintain the same extrusion speed across the boundary
			float maxPrevEndSpeed;					// the maximum end speed we can have for the previous move to remain within the instantaneous speed change limits
#endif
		} beforePrepare;

		// Values that are not set or accessed before Prepare is called
		struct
		{
			// These are calculated from the above and used in the ISR, so they are set up by Prepare()
			uint32_t moveStartTime;					// clock count at which the move is due to start (before execution) or was started (during execution)
			float averageExtrusionSpeed;			// the average extrusion speed in mm/sec, for applying heater feedforward
			LogicalDrivesBitmap drivesMoving;		// bitmap of logical drives moving - needed to keep track of whether remote drives are moving and to determine when a move that checks endstops has terminated
		} afterPrepare;
	};

#if DDA_LOG_PROBE_CHANGES
	static bool probeTriggered;

	void LogProbePosition() noexcept;
#endif
};

inline bool DDA::CanPauseAfter() const noexcept
{
	return flags.canPauseAfter
#if SUPPORT_CAN_EXPANSION
		// We can't easily cancel moves that have already been sent to CAN expansion boards
		&& next->state != committed
#endif
		;
}

inline bool DDA::IsProvisional() const noexcept
{
#if SUPPORT_S_CURVE
	return state == created || state == planned;
#else
	return state == planned;
#endif
}

#endif /* DDA_H_ */
