/*
 * DDA.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef DDA_H_
#define DDA_H_

#include "RepRapFirmware.h"
#include "DriveMovement.h"
#include "StepTimer.h"
#include "GCodes/GCodes.h"			// for class RawMove

#include <optional>

#ifdef DUET_NG
# define DDA_LOG_PROBE_CHANGES	0
#else
# define DDA_LOG_PROBE_CHANGES	0	// save memory on the wired Duet
#endif

class DDARing;

// This defines a single coordinated movement of one or several motors
class DDA
{
	friend class DriveMovement;

public:

	enum DDAState : uint8_t
	{
		empty,				// empty or being filled in
		provisional,		// ready, but could be subject to modifications
		frozen,				// ready, no further modifications allowed
		executing,			// steps are currently being generated for this DDA
		completed			// move has been completed or aborted
	};

	DDA(DDA* n);

	bool InitStandardMove(DDARing& ring, GCodes::RawMove &nextMove, bool doMotorMapping) __attribute__ ((hot));	// Set up a new move, returning true if it represents real movement
	bool InitLeadscrewMove(DDARing& ring, float feedrate, const float amounts[MaxTotalDrivers]);		// Set up a leadscrew motor move

	void Start(Platform& p, uint32_t tim) __attribute__ ((hot));			// Start executing the DDA, i.e. move the move.
	void StepDrivers(Platform& p) __attribute__ ((hot));					// Take one step of the DDA, called by timed interrupt.
	std::optional<uint32_t> GetNextInterruptTime() const;					// Return the time that the next interrupt is needed

	void SetNext(DDA *n) { next = n; }
	void SetPrevious(DDA *p) { prev = p; }
	void Complete() { state = completed; }
	bool Free();
	void Prepare(uint8_t simMode, float extrusionPending[]) __attribute__ ((hot));	// Calculate all the values and freeze this DDA
	bool HasStepError() const;
	bool CanPauseAfter() const { return flags.canPauseAfter; }
	bool IsPrintingMove() const { return flags.isPrintingMove; }			// Return true if this involves both XY movement and extrusion
	bool UsingStandardFeedrate() const { return flags.usingStandardFeedrate; }

	DDAState GetState() const { return state; }
	DDA* GetNext() const { return next; }
	DDA* GetPrevious() const { return prev; }
	int32_t GetTimeLeft() const;
	void InsertHiccup(uint32_t delayClocks) { afterPrepare.moveStartTime += delayClocks; }
	const int32_t *DriveCoordinates() const { return endPoint; }			// Get endpoints of a move in machine coordinates
	void SetDriveCoordinate(int32_t a, size_t drive);						// Force an end point
	void SetFeedRate(float rate) { requestedSpeed = rate; }
	float GetEndCoordinate(size_t drive, bool disableMotorMapping);
	bool FetchEndPosition(volatile int32_t ep[MaxTotalDrivers], volatile float endCoords[MaxTotalDrivers]);
    void SetPositions(const float move[], size_t numDrives);				// Force the endpoints to be these
    FilePosition GetFilePosition() const { return filePos; }
    float GetRequestedSpeed() const { return requestedSpeed; }
    float GetTopSpeed() const { return topSpeed; }
    float GetVirtualExtruderPosition() const { return virtualExtruderPosition; }
	float AdvanceBabyStepping(DDARing& ring, size_t axis, float amount);					// Try to push babystepping earlier in the move queue
	bool IsHomingAxes() const { return (endStopsToCheck & HomeAxes) != 0; }
	uint32_t GetXAxes() const { return xAxes; }
	uint32_t GetYAxes() const { return yAxes; }
	float GetTotalDistance() const { return totalDistance; }
	void LimitSpeedAndAcceleration(float maxSpeed, float maxAcceleration);	// Limit the speed an acceleration of this move

	// Filament monitor support
	int32_t GetStepsTaken(size_t drive) const;

	float GetProportionDone(bool moveWasAborted) const;						// Return the proportion of extrusion for the complete multi-segment move already done

	void MoveAborted();

	uint32_t GetClocksNeeded() const { return clocksNeeded; }
	bool IsGoodToPrepare() const;
	bool IsNonPrintingExtruderMove() const { return flags.isNonPrintingExtruderMove; }

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits GetLaserPwmOrIoBits() const { return laserPwmOrIoBits; }
#endif

#if SUPPORT_IOBITS
	uint32_t GetMoveStartTime() const { return afterPrepare.moveStartTime; }
	IoBits_t GetIoBits() const { return laserPwmOrIoBits.ioBits; }
#endif

	uint32_t GetMoveFinishTime() const { return afterPrepare.moveStartTime + clocksNeeded; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const;	// Get the current full step interval for this axis or extruder
#endif

	void DebugPrint(const char *tag) const;									// print the DDA only
	void DebugPrintAll(const char *tag) const;								// print the DDA and active DMs

	// Note on the following constant:
	// If we calculate the step interval on every clock, we reach a point where the calculation time exceeds the step interval.
	// The worst case is pure Z movement on a delta. On a Mini Kossel with 80 steps/mm with this firmware running on a Duet (84MHx SAM3X8 processor),
	// the calculation can just be managed in time at speeds of 15000mm/min (step interval 50us), but not at 20000mm/min (step interval 37.5us).
	// Therefore, where the step interval falls below 60us, we don't calculate on every step.
	// Note: the above measurements were taken some time ago, before some firmware optimisations.
#if SAME70
	// The system clock of the SAME70 is running at 150MHz. Use the same defaults as for the SAM4E for now.
	static constexpr uint32_t MinCalcIntervalDelta = (40 * StepTimer::StepClockRate)/1000000; 		// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (40 * StepTimer::StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t MinInterruptInterval = 6;									// about 6us minimum interval between interrupts, in step clocks
	static constexpr uint32_t HiccupTime = 10;											// how long we hiccup for
#elif SAM4E || SAM4S
	static constexpr uint32_t MinCalcIntervalDelta = (40 * StepTimer::StepClockRate)/1000000; 		// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (40 * StepTimer::StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t MinInterruptInterval = 6;									// about 6us minimum interval between interrupts, in step clocks
	static constexpr uint32_t HiccupTime = 10;											// how long we hiccup for
#elif __LPC17xx__
    static constexpr uint32_t MinCalcIntervalDelta = (40 * StepTimer::StepClockRate)/1000000;		// the smallest sensible interval between calculations (40us) in step timer clocks
    static constexpr uint32_t MinCalcIntervalCartesian = (40 * StepTimer::StepClockRate)/1000000;	// same as delta for now, but could be lower
    static constexpr uint32_t MinInterruptInterval = 6;									// about 6us minimum interval between interrupts, in step clocks
	static constexpr uint32_t HiccupTime = 10;											// how long we hiccup for
#else
	static constexpr uint32_t MinCalcIntervalDelta = (60 * StepTimer::StepClockRate)/1000000; 		// the smallest sensible interval between calculations (60us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (60 * StepTimer::StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t MinInterruptInterval = 4;									// about 6us minimum interval between interrupts, in step clocks
	static constexpr uint32_t HiccupTime = 8;											// how long we hiccup for
#endif
	static constexpr uint32_t MaxStepInterruptTime = 10 * MinInterruptInterval;			// the maximum time we spend looping in the ISR , in step clocks
	static constexpr uint32_t WakeupTime = StepTimer::StepClockRate/10000;				// stop resting 100us before the move is due to end

	static void PrintMoves();										// print saved moves for debugging

#if DDA_LOG_PROBE_CHANGES
	static const size_t MaxLoggedProbePositions = 40;
	static size_t numLoggedProbePositions;
	static int32_t loggedProbePositions[XYZ_AXES * MaxLoggedProbePositions];
#endif

	static uint32_t lastStepLowTime;								// when we last completed a step pulse to a slow driver
	static uint32_t lastDirChangeTime;								// when we last change the DIR signal to a slow driver

private:
	DriveMovement *FindDM(size_t drive) const;						// find the DM for a drive if there is one even if it is completed
	DriveMovement *FindActiveDM(size_t drive) const;				// find the DM for a drive if there is one but only if it is active
	void RecalculateMove(DDARing& ring) __attribute__ ((hot));
	void MatchSpeeds() __attribute__ ((hot));
	void ReduceHomingSpeed();										// called to reduce homing speed when a near-endstop is triggered
	void StopDrive(size_t drive);									// stop movement of a drive and recalculate the endpoint
	void InsertDM(DriveMovement *dm) __attribute__ ((hot));
	void DeactivateDM(size_t drive);
	void ReleaseDMs();
	bool IsDecelerationMove() const;								// return true if this move is or have been might have been intended to be a deceleration-only move
	bool IsAccelerationMove() const;								// return true if this move is or have been might have been intended to be an acceleration-only move
	void DebugPrintVector(const char *name, const float *vec, size_t len) const;
	void CheckEndstops(Platform& platform);
	float NormaliseXYZ();											// Make the direction vector unit-normal in XYZ
	void AdjustAcceleration();										// Adjust the acceleration and deceleration to reduce ringing

	static void DoLookahead(DDARing& ring, DDA *laDDA) __attribute__ ((hot));	// Try to smooth out moves in the queue
    static float Normalise(float v[], size_t dim1, size_t dim2);  	// Normalise a vector of dim1 dimensions to unit length in the first dim1 dimensions
    static void Absolute(float v[], size_t dimensions);				// Put a vector in the positive hyperquadrant
    static float Magnitude(const float v[], size_t dimensions);  	// Return the length of a vector
    static void Scale(float v[], float scale, size_t dimensions);	// Multiply a vector by a scalar
    static float VectorBoxIntersection(const float v[], 			// Compute the length that a vector would have to have to...
    		const float box[], size_t dimensions);					// ...just touch the surface of a hyperbox.

    DDA *next;								// The next one in the ring
	DDA *prev;								// The previous one in the ring

	volatile DDAState state;				// What state this DDA is in

	union
	{
		struct
		{
			uint16_t endCoordinatesValid : 1,		// True if endCoordinates can be relied on
					 isDeltaMovement : 1,			// True if this is a delta printer movement
					 canPauseAfter : 1,				// True if we can pause at the end of this move
					 isPrintingMove : 1,			// True if this move includes XY movement and extrusion
					 usePressureAdvance : 1,		// True if pressure advance should be applied to any forward extrusion
					 hadLookaheadUnderrun : 1,		// True if the lookahead queue was not long enough to optimise this move
					 xyMoving : 1,					// True if movement along an X axis or the Y axis was requested, even it if's too small to do
					 goingSlow : 1,					// True if we have slowed the movement because the Z probe is approaching its threshold
					 isLeadscrewAdjustmentMove : 1,	// True if this is a leadscrews adjustment move
					 usingStandardFeedrate : 1,		// True if this move uses the standard feed rate
					 isNonPrintingExtruderMove : 1,	// True if this move is a fast extruder-only move, probably a retract/re-prime
					 continuousRotationShortcut : 1, // True if continuous rotation axes take shortcuts
					 usesEndstops : 1,				// True if this move monitors endstops of Z probe
					 controlLaser : 1;				// True if this move controls the laser or iobits
		};
		uint16_t all;								// so that we can print all the flags at once for debugging
	} flags;

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;		// laser PWM required or port state required during this move (here because it is currently 16 bits)
#endif

    EndstopChecks endStopsToCheck;			// Which endstops we are checking on this move
    AxesBitmap xAxes;						// Which axes are behaving as X axes
    AxesBitmap yAxes;						// Which axes are behaving as Y axes

    FilePosition filePos;					// The position in the SD card file after this move was read, or zero if not read from SD card

	int32_t endPoint[MaxTotalDrivers];  	// Machine coordinates of the endpoint
	float endCoordinates[MaxTotalDrivers];	// The Cartesian coordinates at the end of the move plus extrusion amounts
	float directionVector[MaxTotalDrivers];	// The normalised direction vector - first 3 are XYZ Cartesian coordinates even on a delta
    float totalDistance;					// How long is the move in hypercuboid space
	float acceleration;						// The acceleration to use
	float deceleration;						// The deceleration to use
    float requestedSpeed;					// The speed that the user asked for
    float virtualExtruderPosition;			// the virtual extruder position at the end of this move, used for pause/resume

    // These vary depending on how we connect the move with its predecessor and successor, but remain constant while the move is being executed
	float startSpeed;
	float endSpeed;
	float topSpeed;

	float proportionLeft;					// what proportion of the extrusion in the G1 or G0 move of which this is a part remains to be done after this segment is complete
	uint32_t clocksNeeded;

	union
	{
		// Values that are needed only before Prepare is called
		struct
		{
			float accelDistance;
			float decelDistance;
			float targetNextSpeed;				// The speed that the next move would like to start at, used to keep track of the lookahead without making recursive calls
			float maxAcceleration;				// the maximum allowed acceleration for this move according to the limits set by M201
		} beforePrepare;

		// Values that are not set or accessed before Prepare is called
		struct
		{
			// These are calculated from the above and used in the ISR, so they are set up by Prepare()
			uint32_t moveStartTime;				// clock count at which the move is due to start (before execution) or was started (during execution)
			uint32_t startSpeedTimesCdivA;		// the number of clocks it would have taken to reach the start speed from rest
			uint32_t topSpeedTimesCdivDPlusDecelStartClocks;
			int32_t extraAccelerationClocks;	// the additional number of clocks needed because we started the move at less than topSpeed. Negative after ReduceHomingSpeed has been called.

			// These are used only in delta calculations
		    int32_t cKc;						// The Z movement fraction multiplied by Kc and converted to integer
		} afterPrepare;
	};

#if DDA_LOG_PROBE_CHANGES
	static bool probeTriggered;

	void LogProbePosition();
#endif

    DriveMovement* activeDMs;					// list of associated DMs that need steps, in step time order
    DriveMovement* completedDMs;				// list of associated DMs that don't need any more steps
};

// Find the DriveMovement record for a given drive even if it is completed, or return nullptr if there isn't one
inline DriveMovement *DDA::FindDM(size_t drive) const
{
	for (DriveMovement* dm = activeDMs; dm != nullptr; dm = dm->nextDM)
	{
		if (dm->drive == drive)
		{
			return dm;
		}
	}
	for (DriveMovement* dm = completedDMs; dm != nullptr; dm = dm->nextDM)
	{
		if (dm->drive == drive)
		{
			return dm;
		}
	}
	return nullptr;
}

// Find the active DriveMovement record for a given drive, or return nullptr if there isn't one
inline DriveMovement *DDA::FindActiveDM(size_t drive) const
{
	for (DriveMovement* dm = activeDMs; dm != nullptr; dm = dm->nextDM)
	{
		if (dm->drive == drive)
		{
			return dm;
		}
	}
	return nullptr;
}

// Force an end point
inline void DDA::SetDriveCoordinate(int32_t a, size_t drive)
{
	endPoint[drive] = a;
	flags.endCoordinatesValid = false;
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder
inline uint32_t DDA::GetStepInterval(size_t axis, uint32_t microstepShift) const
{
	const DriveMovement * const dm = FindActiveDM(axis);
	return (dm != nullptr) ? dm->GetStepInterval(microstepShift) : 0;
}

#endif

#endif /* DDA_H_ */
