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
#include <Platform/Tasks.h>
#include <GCodes/GCodes.h>			// for class RawMove

# define DDA_LOG_PROBE_CHANGES	0

class DDARing;

// Struct for passing parameters to the DriveMovement Prepare methods, also accessed by the input shaper
struct PrepParams
{
	float totalDistance;
	float accelDistance;
	float decelStartDistance;
	float accelClocks, steadyClocks, decelClocks;
	float acceleration, deceleration;				// the acceleration and deceleration to use, both positive

#if SUPPORT_LINEAR_DELTA
	// Parameters used only for delta moves
	float initialX, initialY;
# if SUPPORT_CAN_EXPANSION
	float finalX, finalY;
	float zMovement;
# endif
	const LinearDeltaKinematics *dparams;
	float a2plusb2;									// sum of the squares of the X and Y movement fractions
#endif

	bool useInputShaping;

	// Calculate and set the steady clocks
	void Finalise(float topSpeed) noexcept;

	// Get the total clocks needed
	float TotalClocks() const noexcept { return accelClocks + steadyClocks + decelClocks; }

	// Set up the parameters from the DDA, excluding steadyClocks because that may be affected by input shaping
	void SetFromDDA(const DDA& dda) noexcept;
};

// This defines a single coordinated movement of one or several motors
class DDA
{
	friend class DriveMovement;
	friend class AxisShaper;
	friend class ExtruderShaper;
	friend class PrepParams;

public:

	enum DDAState : uint8_t
	{
		empty,				// empty or being filled in
		provisional,		// ready, but could be subject to modifications
		committed,			// has been converted into move segments already
		completed			// this move has been completed e.g. because an endstop or probe has stopped it
	};

	DDA(DDA* n) noexcept;

	void* operator new(size_t count) { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	bool InitStandardMove(DDARing& ring, const RawMove &nextMove, bool doMotorMapping) noexcept SPEED_CRITICAL;	// Set up a new move, returning true if it represents real movement
	bool InitLeadscrewMove(DDARing& ring, float feedrate, const float amounts[MaxDriversPerAxis]) noexcept;		// Set up a leadscrew motor move
#if SUPPORT_ASYNC_MOVES
	bool InitAsyncMove(DDARing& ring, const AsyncMove& nextMove) noexcept;			// Set up an async move
#endif

	void Start(Platform& p, uint32_t tim) noexcept SPEED_CRITICAL;					// Start executing the DDA, i.e. move the move.

	void SetNext(DDA *n) noexcept { next = n; }
	void SetPrevious(DDA *p) noexcept { prev = p; }
	bool Free() noexcept;
	void Prepare(DDARing& ring, SimulationMode simMode) noexcept SPEED_CRITICAL;					// Calculate all the values and freeze this DDA
	bool CanPauseAfter() const noexcept;
	bool IsPrintingMove() const noexcept { return flags.isPrintingMove; }			// Return true if this involves both XY movement and extrusion
	bool UsingStandardFeedrate() const noexcept { return flags.usingStandardFeedrate; }
	bool IsCheckingEndstops() const noexcept { return flags.checkEndstops; }
	bool IsScanningProbeMove() const noexcept { return flags.scanningProbeMove; }

	DDAState GetState() const noexcept { return state; }
	DDA* GetNext() const noexcept { return next; }
	DDA* GetPrevious() const noexcept { return prev; }
	uint32_t GetTimeLeft() const noexcept;

#if SUPPORT_REMOTE_COMMANDS
	bool InitFromRemote(const CanMessageMovementLinear& msg) noexcept;
	bool InitFromRemote(const CanMessageMovementLinearShaped& msg) noexcept;
	void StopDrivers(uint16_t whichDrives) noexcept;
#endif

	const int32_t *DriveCoordinates() const noexcept { return endPoint; }			// Get endpoints of a move in machine coordinates
	void SetDriveCoordinate(int32_t a, size_t drive) noexcept;						// Force an end point
	void SetFeedRate(float rate) noexcept { requestedSpeed = rate; }
	float GetEndCoordinate(size_t drive, bool disableMotorMapping) noexcept;
	float GetRawEndCoordinate(size_t drive) const noexcept { return endCoordinates[drive]; }
	void FetchEndPoints(int32_t ep[MaxAxesPlusExtruders]) const noexcept;
	void FetchCurrentPositions(int32_t ep[MaxAxesPlusExtruders]) const noexcept;
	void SetPositions(const float move[]) noexcept;									// Force the endpoints to be these
	FilePosition GetFilePosition() const noexcept { return filePos; }
	float GetRequestedSpeedMmPerClock() const noexcept;
	float GetRequestedSpeedMmPerSec() const noexcept;
	float GetTopSpeedMmPerSec() const noexcept;
	float GetAccelerationMmPerSecSquared() const noexcept;
	float GetDecelerationMmPerSecSquared() const noexcept;
	float GetVirtualExtruderPosition() const noexcept { return virtualExtruderPosition; }
	float GetTotalExtrusionRate() const noexcept;

	float AdvanceBabyStepping(DDARing& ring, size_t axis, float amount) noexcept;	// Try to push babystepping earlier in the move queue
	const Tool *GetTool() const noexcept { return tool; }
	float GetTotalDistance() const noexcept { return totalDistance; }
	void LimitSpeedAndAcceleration(float maxSpeed, float maxAcceleration) noexcept;	// Limit the speed an acceleration of this move

	float GetProportionDone(bool moveWasAborted) const noexcept;					// Return the proportion of extrusion for the complete multi-segment move already done
	float GetInitialUserC0() const noexcept { return initialUserC0; }
	float GetInitialUserC1() const noexcept { return initialUserC1; }

	uint32_t GetClocksNeeded() const noexcept { return clocksNeeded; }
	bool HasExpired() const noexcept;
	bool IsGoodToPrepare() const noexcept;
	bool IsNonPrintingExtruderMove() const noexcept { return flags.isNonPrintingExtruderMove; }
	void UpdateMovementAccumulators(volatile int32_t *accumulators) const noexcept;

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits GetLaserPwmOrIoBits() const noexcept { return laserPwmOrIoBits; }
	bool ControlLaser() const noexcept { return flags.controlLaser; }
#endif

#if SUPPORT_LASER
	uint32_t ManageLaserPower() const noexcept;										// Manage the laser power
#endif

#if SUPPORT_IOBITS
	uint32_t GetMoveStartTime() const noexcept { return afterPrepare.moveStartTime; }
	IoBits_t GetIoBits() const noexcept { return laserPwmOrIoBits.ioBits; }
#endif

	uint32_t GetMoveFinishTime() const noexcept { return afterPrepare.moveStartTime + clocksNeeded; }

	void DebugPrint(const char *tag) const noexcept;								// print the DDA only

	static void PrintMoves() noexcept;												// print saved moves for debugging

#if DDA_LOG_PROBE_CHANGES
	static const size_t MaxLoggedProbePositions = 40;
	static size_t numLoggedProbePositions;
	static int32_t loggedProbePositions[XYZ_AXES * MaxLoggedProbePositions];
#endif

#if 0	// debug only
	static uint32_t stepsRequested[NumDirectDrivers], stepsDone[NumDirectDrivers];
#endif

private:
	void RecalculateMove(DDARing& ring) noexcept SPEED_CRITICAL;
	void MatchSpeeds() noexcept SPEED_CRITICAL;
	bool IsDecelerationMove() const noexcept;								// return true if this move is or have been might have been intended to be a deceleration-only move
	bool IsAccelerationMove() const noexcept;								// return true if this move is or have been might have been intended to be an acceleration-only move
	void DebugPrintVector(const char *name, const float *vec, size_t len) const noexcept;

#if SUPPORT_CAN_EXPANSION
	int32_t PrepareRemoteExtruder(size_t drive, float& extrusionPending, float speedChange) const noexcept;
#endif

	static void DoLookahead(DDARing& ring, DDA *laDDA) noexcept SPEED_CRITICAL;	// Try to smooth out moves in the queue
    static float Normalise(float v[], AxesBitmap unitLengthAxes) noexcept;  // Normalise a vector to unit length over the specified axes
    static float Normalise(float v[]) noexcept; 							// Normalise a vector to unit length over all axes
	float NormaliseLinearMotion(AxesBitmap linearAxes) noexcept;			// Make the direction vector unit-normal in XYZ
    static void Absolute(float v[], size_t dimensions) noexcept;			// Put a vector in the positive hyperquadrant

    static float Magnitude(const float v[]) noexcept;						// Get the magnitude measured over all axes and extruders
    static float Magnitude(const float v[], AxesBitmap axes) noexcept;  	// Return the length of a vector over the specified orthogonal axes
    static void Scale(float v[], float scale) noexcept;						// Multiply a vector by a scalar
    static float VectorBoxIntersection(const float v[], const float box[]) noexcept;	// Compute the length that a vector would have to have to just touch the surface of a hyperbox of MaxAxesPlusExtruders dimensions.

    DDA *next;										// The next one in the ring
	DDA *prev;										// The previous one in the ring

	volatile DDAState state;						// What state this DDA is in

	union
	{
		struct
		{
			uint16_t endCoordinatesValid : 1,		// True if endCoordinates can be relied
#if SUPPORT_LINEAR_DELTA
					 isDeltaMovement : 1,			// True if this is a delta printer movement
#endif
					 canPauseAfter : 1,				// True if we can pause at the end of this move
					 isPrintingMove : 1,			// True if this move includes XY movement and extrusion
					 usePressureAdvance : 1,		// True if pressure advance should be applied to any forward extrusion
					 hadLookaheadUnderrun : 1,		// True if the lookahead queue was not long enough to optimise this move
					 xyMoving : 1,					// True if movement along an X axis or a Y axis was requested, even it if's too small to do
					 isLeadscrewAdjustmentMove : 1,	// True if this is a leadscrews adjustment move
					 usingStandardFeedrate : 1,		// True if this move uses the standard feed rate
					 isNonPrintingExtruderMove : 1,	// True if this move is an extruder-only move, or involves reverse extrusion (and possibly axis movement too)
					 continuousRotationShortcut : 1, // True if continuous rotation axes take shortcuts
					 checkEndstops : 1,				// True if this move monitors endstops or Z probe
					 controlLaser : 1,				// True if this move controls the laser or iobits
					 scanningProbeMove : 1,	 	 	// True if this is a scanning Z probe move
					 isRemote : 1,					// True if this move was commanded from a remote
					 wasAccelOnlyMove : 1;			// set by Prepare if this was an acceleration-only move, for the next move to look at
		};
		uint16_t all;								// so that we can print all the flags at once for debugging
	} flags;

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;				// laser PWM required or port state required during this move (here because it is currently 16 bits)
#endif

	const Tool *tool;								// which tool (if any) is active

    FilePosition filePos;							// The position in the SD card file after this move was read, or zero if not read from SD card

	int32_t endPoint[MaxAxesPlusExtruders];  		// Machine coordinates of the endpoint
	float endCoordinates[MaxAxesPlusExtruders];		// The Cartesian coordinates at the end of the move plus extrusion amounts
	float directionVector[MaxAxesPlusExtruders];	// The normalised direction vector - first 3 are XYZ Cartesian coordinates even on a delta
    float totalDistance;							// How long is the move in hypercuboid space
	float acceleration;								// The acceleration to use, always positive
	float deceleration;								// The deceleration to use, always positive
    float requestedSpeed;							// The speed that the user asked for
    float virtualExtruderPosition;					// the virtual extruder position at the end of this move, used for pause/resume

    // These vary depending on how we connect the move with its predecessor and successor, but remain constant while the move is being executed
	float startSpeed;
	float endSpeed;
	float topSpeed;

	float proportionDone;							// what proportion of the extrusion in the G1 or G0 move of which this is a part has been done after this segment is complete
	float initialUserC0, initialUserC1;				// if this is a segment of an arc move, the user X and Y coordinates at the start
	uint32_t clocksNeeded;

	union
	{
		// Values that are needed only before Prepare is called and in the first few lines of Prepare
		struct
		{
			float accelDistance;
			float decelDistance;
			float targetNextSpeed;					// The speed that the next move would like to start at, used to keep track of the lookahead without making recursive calls
		} beforePrepare;

		// Values that are not set or accessed before Prepare is called
		struct
		{
			// These are calculated from the above and used in the ISR, so they are set up by Prepare()
			uint32_t moveStartTime;					// clock count at which the move is due to start (before execution) or was started (during execution)
			float averageExtrusionSpeed;			// the average extrusion speed in mm/sec, for applying heater feedforward

#if SUPPORT_CAN_EXPANSION
			AxesBitmap drivesMoving;				// bitmap of logical drives moving - needed to keep track of whether remote drives are moving
#endif
		} afterPrepare;
	};

#if DDA_LOG_PROBE_CHANGES
	static bool probeTriggered;

	void LogProbePosition() noexcept;
#endif
};

// Return true if there is no reason to delay preparing this move
inline bool DDA::IsGoodToPrepare() const noexcept
{
	return endSpeed >= topSpeed;							// if it never decelerates, we can't improve it
}

inline bool DDA::CanPauseAfter() const noexcept
{
	return flags.canPauseAfter
#if SUPPORT_CAN_EXPANSION
		// We can't easily cancel moves that have already been sent to CAN expansion boards
		&& next->state == DDAState::provisional
#endif
		;
}

// This is called by DDARing::LiveCoordinates to get the endpoints of a move that has been completed
inline void DDA::FetchEndPoints(int32_t ep[MaxAxesPlusExtruders]) const noexcept
{
	memcpyi32(ep, endPoint, MaxAxesPlusExtruders);
}

// This is called from DDARing only
inline bool DDA::HasExpired() const noexcept
{
	return state == committed && (int32_t)(StepTimer::GetTimerTicks() - (afterPrepare.moveStartTime + clocksNeeded)) >= 0;
}

#endif /* DDA_H_ */
