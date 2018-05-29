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
#include "GCodes/GCodes.h"			// for class RawMove

#ifdef DUET_NG
#define DDA_LOG_PROBE_CHANGES	0
#else
#define DDA_LOG_PROBE_CHANGES	0		// save memory on the wired Duet
#endif

/**
 * This defines a single linear movement of the print head
 */
class DDA
{
	friend class DriveMovement;

public:

	enum DDAState : unsigned char
	{
		empty,				// empty or being filled in
		provisional,		// ready, but could be subject to modifications
		frozen,				// ready, no further modifications allowed
		executing,			// steps are currently being generated for this DDA
		completed			// move has been completed or aborted
	};

	DDA(DDA* n);

	bool Init(GCodes::RawMove &nextMove, bool doMotorMapping) __attribute__ ((hot));	// Set up a new move, returning true if it represents real movement
	bool Init(const float_t steps[DRIVES]);							// Set up a raw (unmapped) motor move
	void Init();													// Set up initial positions for machine startup
	bool Start(uint32_t tim) __attribute__ ((hot));					// Start executing the DDA, i.e. move the move.
	bool Step() __attribute__ ((hot));								// Take one step of the DDA, called by timed interrupt.
	void SetNext(DDA *n) { next = n; }
	void SetPrevious(DDA *p) { prev = p; }
	void Complete() { state = completed; }
	bool Free();
	void Prepare(uint8_t simMode) __attribute__ ((hot));			// Calculate all the values and freeze this DDA
	bool HasStepError() const;
	bool CanPauseAfter() const { return canPauseAfter; }
	bool IsPrintingMove() const { return isPrintingMove; }			// Return true if this involves both XY movement and extrusion
	bool UsingStandardFeedrate() const { return usingStandardFeedrate; }

	DDAState GetState() const { return state; }
	DDA* GetNext() const { return next; }
	DDA* GetPrevious() const { return prev; }
	int32_t GetTimeLeft() const;
	const int32_t *DriveCoordinates() const { return endPoint; }	// Get endpoints of a move in machine coordinates
	void SetDriveCoordinate(int32_t a, size_t drive);				// Force an end point
	void SetFeedRate(float rate) { requestedSpeed = rate; }
	float GetEndCoordinate(size_t drive, bool disableMotorMapping);
	bool FetchEndPosition(volatile int32_t ep[DRIVES], volatile float endCoords[DRIVES]);
    void SetPositions(const float move[], size_t numDrives);		// Force the endpoints to be these
    FilePosition GetFilePosition() const { return filePos; }
    float GetRequestedSpeed() const { return requestedSpeed; }
    float GetVirtualExtruderPosition() const { return virtualExtruderPosition; }
	float AdvanceBabyStepping(float amount);						// Try to push babystepping earlier in the move queue
	bool IsHomingAxes() const { return (endStopsToCheck & HomeAxes) != 0; }
	uint32_t GetXAxes() const { return xAxes; }
	uint32_t GetYAxes() const { return yAxes; }
	float GetTotalDistance() const { return totalDistance; }
	void LimitSpeedAndAcceleration(float maxSpeed, float maxAcceleration);	// Limit the speed an acceleration of this move

	int32_t GetStepsTaken(size_t drive) const;
	bool IsNonPrintingExtruderMove(size_t drive) const;

	float GetProportionDone(bool moveWasAborted) const;						// Return the proportion of extrusion for the complete multi-segment move already done

	void MoveAborted();

	uint32_t GetClocksNeeded() const { return clocksNeeded; }
	bool IsGoodToPrepare() const;

#if SUPPORT_IOBITS
	uint32_t GetMoveStartTime() const { return moveStartTime; }
	IoBits_t GetIoBits() const { return ioBits; }
#endif

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const;	// Get the current full step interval for this axis or extruder
#endif

	void DebugPrint() const;												// print the DDA only
	void DebugPrintAll() const;												// print the DDA and active DMs

	// Note on the following constant:
	// If we calculate the step interval on every clock, we reach a point where the calculation time exceeds the step interval.
	// The worst case is pure Z movement on a delta. On a Mini Kossel with 80 steps/mm with this firmware running on a Duet (84MHx SAM3X8 processor),
	// the calculation can just be managed in time at speeds of 15000mm/min (step interval 50us), but not at 20000mm/min (step interval 37.5us).
	// Therefore, where the step interval falls below 60us, we don't calculate on every step.
	// Note: the above measurements were taken some time ago, before some firmware optimisations.
#if SAME70
	// The system clock of the SAME70 is running at 150MHz. Use the same defaults as for the SAM4E for now.
	static constexpr uint32_t MinCalcIntervalDelta = (40 * StepClockRate)/1000000; 		// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (40 * StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t MinInterruptInterval = 6;									// about 6us minimum interval between interrupts, in step clocks
#elif SAM4E || SAM4S
	static constexpr uint32_t MinCalcIntervalDelta = (40 * StepClockRate)/1000000; 		// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (40 * StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t MinInterruptInterval = 6;									// about 6us minimum interval between interrupts, in step clocks
#else
	static constexpr uint32_t MinCalcIntervalDelta = (60 * StepClockRate)/1000000; 		// the smallest sensible interval between calculations (60us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (60 * StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t MinInterruptInterval = 4;									// about 6us minimum interval between interrupts, in step clocks
#endif
	static constexpr uint32_t MaxStepInterruptTime = 10 * MinInterruptInterval;			// the maximum time we spend looping in the ISR , in step clocks

	static void PrintMoves();										// print saved moves for debugging

#if DDA_LOG_PROBE_CHANGES
	static const size_t MaxLoggedProbePositions = 40;
	static size_t numLoggedProbePositions;
	static int32_t loggedProbePositions[XYZ_AXES * MaxLoggedProbePositions];
#endif

	static uint32_t numHiccups;										// how many times we delayed an interrupt to avoid using too much CPU time in interrupts
	static uint32_t lastStepLowTime;								// when we last completed a step pulse to a slow driver
	static uint32_t lastDirChangeTime;								// when we last change the DIR signal to a slow driver

private:
	DriveMovement *FindDM(size_t drive) const;
	void RecalculateMove() __attribute__ ((hot));
	void MatchSpeeds() __attribute__ ((hot));
	void ReduceHomingSpeed();										// called to reduce homing speed when a near-endstop is triggered
	void StopDrive(size_t drive);									// stop movement of a drive and recalculate the endpoint
	void InsertDM(DriveMovement *dm) __attribute__ ((hot));
	void RemoveDM(size_t drive);
	void ReleaseDMs();
	bool IsDecelerationMove() const;								// return true if this move is or have been might have been intended to be a deceleration-only move
	void DebugPrintVector(const char *name, const float *vec, size_t len) const;
	void CheckEndstops(Platform& platform);
	float NormaliseXYZ();											// Make the direction vector unit-normal in XYZ

	static void DoLookahead(DDA *laDDA) __attribute__ ((hot));		// Try to smooth out moves in the queue
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
			uint8_t endCoordinatesValid : 1;		// True if endCoordinates can be relied on
			uint8_t isDeltaMovement : 1;			// True if this is a delta printer movement
			uint8_t canPauseAfter : 1;				// True if we can pause at the end of this move
			uint8_t isPrintingMove : 1;				// True if this move includes XY movement and extrusion
			uint8_t usePressureAdvance : 1;			// True if pressure advance should be applied to any forward extrusion
			uint8_t hadLookaheadUnderrun : 1;		// True if the lookahead queue was not long enough to optimise this move
			uint8_t xyMoving : 1;					// True if movement along an X axis or the Y axis was requested, even it if's too small to do
			uint8_t goingSlow : 1;					// True if we have slowed the movement because the Z probe is approaching its threshold
			uint8_t isLeadscrewAdjustmentMove : 1;	// True if this is a leadscrews adjustment move
			uint8_t usingStandardFeedrate : 1;		// True if this move uses the standard feed rate
		};
		uint16_t flags;								// so that we can print all the flags at once for debugging
	};

    EndstopChecks endStopsToCheck;			// Which endstops we are checking on this move
    AxesBitmap xAxes;						// Which axes are behaving as X axes
    AxesBitmap yAxes;						// Which axes are behaving as Y axes

    FilePosition filePos;					// The position in the SD card file after this move was read, or zero if not read from SD card

	int32_t endPoint[DRIVES];  				// Machine coordinates of the endpoint
	float endCoordinates[DRIVES];			// The Cartesian coordinates at the end of the move plus extrusion amounts
	float directionVector[DRIVES];			// The normalised direction vector - first 3 are XYZ Cartesian coordinates even on a delta
    float totalDistance;					// How long is the move in hypercuboid space
	float acceleration;						// The acceleration to use
    float requestedSpeed;					// The speed that the user asked for
    float virtualExtruderPosition;			// the virtual extruder position at the end of this move, used for pause/resume

    // These are used only in delta calculations
    int32_t cKc;							// The Z movement fraction multiplied by Kc and converted to integer

    // These vary depending on how we connect the move with its predecessor and successor, but remain constant while the move is being executed
	float startSpeed;
	float endSpeed;
	float topSpeed;
	float accelDistance;
	float decelDistance;

	// This is a temporary, used to keep track of the lookahead to avoid making recursive calls
	float targetNextSpeed;					// The speed that the next move would like to start at

	// These are calculated from the above and used in the ISR, so they are set up by Prepare()
	uint32_t clocksNeeded;					// in clocks
	uint32_t moveStartTime;					// clock count at which the move was started
	uint32_t startSpeedTimesCdivA;			// the number of clocks it would have taken to reach the start speed from rest
	uint32_t topSpeedTimesCdivAPlusDecelStartClocks;
	int32_t extraAccelerationClocks;		// the additional number of clocks needed because we started the move at less than topSpeed. Negative after ReduceHomingSpeed has been called.

	float proportionLeft;					// what proportion of the extrusion in the G1 or G0 move of which this is a part remains to be done after this segment is complete

#if SUPPORT_IOBITS
	IoBits_t ioBits;						// port state required during this move
#endif

#if DDA_LOG_PROBE_CHANGES
	static bool probeTriggered;

	void LogProbePosition();
#endif

    DriveMovement* firstDM;					// list of contained DMs that need steps, in step time order
	DriveMovement *pddm[DRIVES];			// These describe the state of each drive movement
};

// Find the DriveMovement record for a given drive, or return nullptr if there isn't one
inline DriveMovement *DDA::FindDM(size_t drive) const
{
	return pddm[drive];
}

// Force an end point
inline void DDA::SetDriveCoordinate(int32_t a, size_t drive)
{
	endPoint[drive] = a;
	endCoordinatesValid = false;
}

#if HAS_STALL_DETECT

// Get the current full step interval for this axis or extruder
inline uint32_t DDA::GetStepInterval(size_t axis, uint32_t microstepShift) const
{
	const DriveMovement * const dm = FindDM(axis);
	return (dm != nullptr) ? dm->GetStepInterval(microstepShift) : 0;
}

#endif

#endif /* DDA_H_ */
