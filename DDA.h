/*
 * DDA.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef DDA_H_
#define DDA_H_

#include "DriveMovement.h"

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

	bool Init(const float nextMove[], EndstopChecks ce,
			bool doMotorMapping, FilePosition fPos);				// Set up a new move, returning true if it represents real movement
	void Init();													// Set up initial positions for machine startup
	bool Start(uint32_t tim);										// Start executing the DDA, i.e. move the move.
	bool Step();													// Take one step of the DDA, called by timed interrupt.
	void SetNext(DDA *n) { next = n; }
	void SetPrevious(DDA *p) { prev = p; }
	void Release() { state = empty; }
	void Prepare();													// Calculate all the values and freeze this DDA
	float CalcTime() const;											// Calculate the time needed for this move (used for simulation)
	void PrintIfHasStepError();
	bool CanPause() const { return canPause; }

	DDAState GetState() const { return state; }
	DDA* GetNext() const { return next; }
	DDA* GetPrevious() const { return prev; }
	int32_t GetTimeLeft() const;
	float GetMotorPosition(size_t drive) const;						// Get the real mm position of a motor at the planned endpoint of this move
	const int32_t *DriveCoordinates() const { return endPoint; }	// Get endpoints of a move in machine coordinates
	void SetDriveCoordinate(int32_t a, size_t drive);				// Force an end point
	void SetFeedRate(float rate) { requestedSpeed = rate; }
	float GetEndCoordinate(size_t drive, bool disableDeltaMapping);
	bool FetchEndPosition(volatile int32_t ep[DRIVES], volatile float endCoords[AXES]);
    void SetPositions(const float move[]);							// Force the endpoints to be these
    FilePosition GetFilePosition() const { return filePos; }
    float GetRequestedSpeed() const { return requestedSpeed; }

	void DebugPrint() const;

	static const uint32_t stepClockRate = VARIANT_MCK/32;			// the frequency of the clock used for stepper pulse timing (using TIMER_CLOCK3), about 0.38us resolution
	static const uint64_t stepClockRateSquared = (uint64_t)stepClockRate * stepClockRate;
	static const int32_t MinStepInterval = (4 * stepClockRate)/1000000; // the smallest sensible interval between steps (10us) in step timer clocks

	// Note on the following constant:
	// If we calculate the step interval on every clock, we reach a point where the calculation time exceeds the step interval.
	// The worst case is pure Z movement on a delta. On a Mini Kossel with 80 steps/mm witt this formware runnig on a Duet (84MHx SAM3X8 processor),
	// the calculation can just be managed in time at speeds of 15000mm/min (step interval 50us), but not at 20000mm/min (step interval 37.5us).
	// Therefore, where the step interval falls below 70us, we don't calculate on every step.
	static const uint32_t MinCalcInterval = (70 * stepClockRate)/1000000; // the smallest sensible interval between calculations (70us) in step timer clocks

private:
	static const uint32_t minInterruptInterval = 6;					// about 2us minimum interval between interrupts, in clocks

	void RecalculateMove();
	void CalcNewSpeeds();
	void ReduceHomingSpeed();										// called to reduce homing speed when a near-endstop is triggered
	void StopDrive(size_t drive);									// stop movement of a drive and recalculate the endpoint
	void MoveAborted(uint32_t clocksFromStart);
	void DebugPrintVector(const char *name, const float *vec, size_t len) const;

	static void DoLookahead(DDA *laDDA);							// called by AdjustEndSpeed to do the real work
    static float Normalise(float v[], size_t dim1, size_t dim2);  	// Normalise a vector of dim1 dimensions to unit length in the first dim1 dimensions
    static void Absolute(float v[], size_t dimensions);				// Put a vector in the positive hyperquadrant
    static float Magnitude(const float v[], size_t dimensions);  	// Return the length of a vector
    static void Scale(float v[], float scale, size_t dimensions);	// Multiply a vector by a scalar
    static float VectorBoxIntersection(const float v[], 			// Compute the length that a vector would have to have to...
    		const float box[], size_t dimensions);					// ...just touch the surface of a hyperbox.

    DDA* next;								// The next one in the ring
	DDA *prev;								// The previous one in the ring

	volatile DDAState state;				// what state this DDA is in
	bool endCoordinatesValid;				// True if endCoordinates can be relied on
    bool isDeltaMovement;					// True if this is a delta printer movement
    bool canPause;							// True if we can pause at the end of this move
    bool goingSlow;							// True if we have reduced speed during homing

    EndstopChecks endStopsToCheck;			// Which endstops we are checking on this move
    // We are on a half-word boundary here, so expect 2 bytes of padding to be inserted at this point

    FilePosition filePos;					// The position in the SD card file after this move was read, or zero if not read fro SD card

	int32_t endPoint[DRIVES];  				// Machine coordinates of the endpoint
	float endCoordinates[AXES];				// The Cartesian coordinates at the end of the move
	float directionVector[DRIVES];			// The normalised direction vector - first 3 are XYZ Cartesian coordinates even on a delta
    float totalDistance;					// How long is the move in hypercuboid space
	float acceleration;						// The acceleration to use
    float requestedSpeed;					// The speed that the user asked for

    // These are used only in delta calculations
    float a2plusb2;							// Sum of the squares of the X and Y movement fractions
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
	uint32_t firstStepTime;					// in clocks, relative to the start of the move

	DriveMovement ddm[DRIVES];				// These describe the state of each drive movement
};

// Force an end point
inline void DDA::SetDriveCoordinate(int32_t a, size_t drive)
{
	endPoint[drive] = a;
	endCoordinatesValid = false;
}

#endif /* DDA_H_ */
