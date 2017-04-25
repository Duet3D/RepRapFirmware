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
#include "Libraries/Math/Matrix.h"

#ifdef DUET_NG
const unsigned int DdaRingLength = 30;
#else
// We are more memory-constrained on the SAM3X
const unsigned int DdaRingLength = 20;
#endif

#include "DeltaKinematics.h"
#include "DeltaProbe.h"
#include "Grid.h"

enum PointCoordinateSet
{
	unset = 0,
	xSet = 1,
	ySet = 2,
	zSet = 4,
	xyCorrected = 8,
	probeError = 16
};

/**
 * This is the master movement class.  It controls all movement in the machine.
 */
class Move
{
public:

    Move(Platform* p, GCodes* g);
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
    void SetPositions(const float move[DRIVES]);					// Force the coordinates to be these
    void SetLiveCoordinates(const float coords[DRIVES]);			// Force the live coordinates (see above) to be these
	void ResetExtruderPositions();									// Resets the extrusion amounts of the live coordinates
    void SetXBedProbePoint(size_t index, float x);					// Record the X coordinate of a probe point
    void SetYBedProbePoint(size_t index, float y);					// Record the Y coordinate of a probe point
    void SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError); // Record the Z coordinate of a probe point
    float XBedProbePoint(size_t index) const;						// Get the X coordinate of a probe point
    float YBedProbePoint(size_t index) const;						// Get the Y coordinate of a probe point
    float ZBedProbePoint(size_t index) const;						// Get the Z coordinate of a probe point
    size_t NumberOfProbePoints() const;								// How many points to probe have been set?  0 if incomplete
    size_t NumberOfXYProbePoints() const;							// How many XY coordinates of probe points have been set (Zs may not have been probed yet)
    bool AllProbeCoordinatesSet(int index) const;					// XY, and Z all set for this one?
    bool XYProbeCoordinatesSet(int index) const;					// Just XY set for this one?
    float GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const; // Get pre-recorded probe coordinates
    void FinishedBedProbing(int sParam, StringRef& reply);			// Calibrate or set the bed equation after probing
    float SecondDegreeTransformZ(float x, float y) const;			// Used for second degree bed equation
    void SetAxisCompensation(int8_t axis, float tangent);			// Set an axis-pair compensation angle
    float AxisCompensation(int8_t axis) const;						// The tangent value
    void SetIdentityTransform();									// Cancel the bed equation; does not reset axis angle compensation
    void Transform(float move[], uint32_t xAxes, bool useBedCompensation) const; // Take a position and apply the bed and the axis-angle compensations
    void InverseTransform(float move[], uint32_t xAxes) const;		// Go from a transformed point back to user coordinates
	float GetTaperHeight() const { return (useTaper) ? taperHeight : 0.0; }
	void SetTaperHeight(float h);

    void Diagnostics(MessageType mtype);							// Report useful stuff

    Kinematics& GetKinematics() { return deltaParams; }				// temporary definition
    const DeltaKinematics& GetDeltaParams() const { return deltaParams; }
    DeltaKinematics& AccessDeltaParams() { return deltaParams; }
    bool IsDeltaMode() const { return deltaParams.IsDeltaMode(); }
    const char* GetGeometryString() const;

    int GetCoreXYMode() const { return coreXYMode; }
    void SetCoreXYMode(int mode) { coreXYMode = mode; }
    float GetCoreAxisFactor(size_t axis) const { return axisFactors[axis]; }
    void SetCoreAxisFactor(size_t axis, float f) { axisFactors[axis] = f; }
    bool IsCoreXYAxis(size_t axis) const;											// Return true if the specified axis shares its motors with another

    void CurrentMoveCompleted();													// Signal that the current move has just been completed
    bool TryStartNextMove(uint32_t startTime);										// Try to start another move, returning true if Step() needs to be called immediately
    void MotorTransform(const float machinePos[MAX_AXES], int32_t motorPos[MAX_AXES]) const;		// Convert Cartesian coordinates to delta motor coordinates
    float MotorFactor(size_t drive, const float directionVector[]) const;							// Calculate the movement fraction for a single axis motor of a Cartesian or CoreXY printer
    void MachineToEndPoint(const int32_t motorPos[], float machinePos[], size_t numDrives) const;	// Convert motor coordinates to machine coordinates
    void EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const;

	float IdleTimeout() const { return idleTimeout; }								// Returns the idle timeout in seconds
	void SetIdleTimeout(float timeout) { idleTimeout = timeout; }					// Set the idle timeout in seconds

    void Simulate(uint8_t simMode);													// Enter or leave simulation mode
    float GetSimulationTime() const { return simulationTime; }						// Get the accumulated simulation time
    void PrintCurrentDda() const;													// For debugging

    FilePosition PausePrint(float positions[DRIVES], float& pausedFeedRate, uint32_t xAxes); // Pause the print as soon as we can
    bool NoLiveMovement() const;													// Is a move running, or are there any queued?

    int DoDeltaProbe(float frequency, float amplitude, float rate, float distance);

    static int32_t MotorEndPointToMachine(size_t drive, float coord);				// Convert a single motor position to number of steps
    static float MotorEndpointToPosition(int32_t endpoint, size_t drive);			// Convert number of motor steps to motor position

	bool IsExtruding() const;														// Is filament being extruded?

	uint32_t GetScheduledMoves() const { return scheduledMoves; }					// How many moves have been scheduled?
	uint32_t GetCompletedMoves() const { return completedMoves; }					// How many moves have been completed?
	void ResetMoveCounters() { scheduledMoves = completedMoves = 0; }

	HeightMap& AccessBedProbeGrid() { return grid; }								// Access the bed probing grid

private:

	enum class IdleState : uint8_t { idle, busy, timing };

	bool StartNextMove(uint32_t startTime);											// start the next move, returning true if Step() needs to be called immediately
	bool GoodProbePointOrdering(size_t numPoints) const;							// Check that the probe points are in the right order
	void SetProbedBedEquation(size_t numPoints, StringRef& reply);					// When we have a full set of probed points, work out the bed's equation
	void DoDeltaCalibration(size_t numPoints, StringRef& reply);
	void BedTransform(float move[MAX_AXES], uint32_t xAxes) const;					// Take a position and apply the bed compensations
	void InverseBedTransform(float move[MAX_AXES], uint32_t xAxes) const;			// Go from a bed-transformed point back to user coordinates
	void AxisTransform(float move[MAX_AXES]) const;									// Take a position and apply the axis-angle compensations
	void InverseAxisTransform(float move[MAX_AXES]) const;							// Go from an axis transformed point back to user coordinates
	void BarycentricCoordinates(size_t p0, size_t p1,   							// Compute the barycentric coordinates of a point in a triangle
			size_t p2, float x, float y, float& l1,     							// (see http://en.wikipedia.org/wiki/Barycentric_coordinate_system).
			float& l2, float& l3) const;
	float TriangleZ(float x, float y) const;										// Interpolate onto a triangular grid
	void AdjustDeltaParameters(const floatc_t v[], size_t numFactors);				// Perform delta adjustment
	void JustHomed(size_t axis, float hitPoint, DDA* hitDDA);						// Deal with setting positions after a drive has been homed
	void DeltaProbeInterrupt();														// Step ISR when using the experimental delta probe

	static void PrintMatrix(const char* s, const MathMatrix<floatc_t>& m, size_t numRows = 0, size_t maxCols = 0);	// for debugging
	static void PrintVector(const char *s, const floatc_t *v, size_t numElems);		// for debugging

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

	// Variable for G32 bed probing, for bed compensation and delta calibration
	float xBedProbePoints[MaxProbePoints];				// The X coordinates of the points on the bed at which to probe
	float yBedProbePoints[MaxProbePoints];				// The Y coordinates of the points on the bed at which to probe
	float zBedProbePoints[MaxGridProbePoints];			// The Z coordinates of the points on the bed that were probed
	float baryXBedProbePoints[5];						// The X coordinates of the triangle corner points
	float baryYBedProbePoints[5];						// The Y coordinates of the triangle corner points
	float baryZBedProbePoints[5];						// The Z coordinates of the triangle corner points
	uint8_t probePointSet[MaxProbePoints];				// Has the XY of this point been set?  Has the Z been probed?
	float aX, aY, aC; 									// Bed plane explicit equation z' = z + aX*x + aY*y + aC
	float tanXY, tanYZ, tanXZ; 							// Axis compensation - 90 degrees + angle gives angle between axes
	int numBedCompensationPoints;						// The number of points we are actually using for bed compensation, 0 means identity bed transform
	float xRectangle, yRectangle;						// The side lengths of the rectangle used for second-degree bed compensation
	float taperHeight;									// Height over which we taper
	float recipTaperHeight;								// Reciprocal of the taper height
	bool useTaper;										// True to taper off the compensation

	HeightMap grid;    									// Grid definition and height map for G29 bed probing. The probe heights are stored in zBedProbePoints, see above.

	float idleTimeout;									// How long we wait with no activity before we reduce motor currents to idle
	float lastMoveTime;									// The approximate time at which the last move was completed, or 0
	float longWait;										// A long time for things that need to be done occasionally
	IdleState iState;									// whether the idle timer is active

	DeltaKinematics deltaParams;						// Information about the delta parameters of this machine
	DeltaProbe deltaProbe;								// Delta probing state
	uint32_t deltaProbingStartTime;
	bool deltaProbing;
	int coreXYMode;										// 0 = Cartesian, 1 = CoreXY, 2 = CoreXZ, 3 = CoreYZ
	float axisFactors[MAX_AXES];						// How much further the motors need to move for each axis movement, on a CoreXY/CoreXZ/CoreYZ machine
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
	else if (deltaProbing)
	{
		DeltaProbeInterrupt();
	}
}

#endif /* MOVE_H_ */
