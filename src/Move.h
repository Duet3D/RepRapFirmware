/*
 * Move.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef MOVE_H_
#define MOVE_H_

#include "DDA.h"
#include "Matrix.h"
#include "DeltaParameters.h"
#include "DeltaProbe.h"

const unsigned int DdaRingLength = 20;

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
    void Init();										// Start me up
    void Spin();										// Called in a tight loop to keep the class going
    void Exit();										// Shut down
    void GetCurrentUserPosition(float m[DRIVES], uint8_t moveType) const;	// Return the position (after all queued moves have been executed) in transformed coords
    int32_t GetEndPoint(size_t drive) const { return liveEndPoints[drive]; }	// Get the current position of a motor
    void LiveCoordinates(float m[DRIVES]);				// Gives the last point at the end of the last complete DDA transformed to user coords
    void Interrupt();									// The hardware's (i.e. platform's)  interrupt should call this.
    void InterruptTime();								// Test function - not used
    bool AllMovesAreFinished();							// Is the look-ahead ring empty?  Stops more moves being added as well.
    void ResumeMoving();								// Allow moves to be added after a call to AllMovesAreFinished()
    void DoLookAhead();									// Run the look-ahead procedure
    void HitLowStop(size_t axis, DDA* hitDDA);			// What to do when a low endstop is hit
    void HitHighStop(size_t axis, DDA* hitDDA);			// What to do when a high endstop is hit
    void ZProbeTriggered(DDA* hitDDA);					// What to do when a the Z probe is triggered
    void SetPositions(const float move[DRIVES]);		// Force the coordinates to be these
    void SetLiveCoordinates(const float coords[DRIVES]); // Force the live coordinates (see above) to be these
    void SetXBedProbePoint(size_t index, float x);		// Record the X coordinate of a probe point
    void SetYBedProbePoint(size_t index, float y);		// Record the Y coordinate of a probe point
    void SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError);	// Record the Z coordinate of a probe point
    float XBedProbePoint(size_t index) const;			// Get the X coordinate of a probe point
    float YBedProbePoint(size_t index) const;			// Get the Y coordinate of a probe point
    float ZBedProbePoint(size_t index)const ;			// Get the Z coordinate of a probe point
    size_t NumberOfProbePoints() const;					// How many points to probe have been set?  0 if incomplete
    size_t NumberOfXYProbePoints() const;				// How many XY coordinates of probe points have been set (Zs may not have been probed yet)
    bool AllProbeCoordinatesSet(int index) const;		// XY, and Z all set for this one?
    bool XYProbeCoordinatesSet(int index) const;		// Just XY set for this one?
    void FinishedBedProbing(int sParam, StringRef& reply); // Calibrate or set the bed equation after probing
    float SecondDegreeTransformZ(float x, float y) const; // Used for second degree bed equation
    void SetAxisCompensation(int8_t axis, float tangent); // Set an axis-pair compensation angle
    float AxisCompensation(int8_t axis) const;			// The tangent value
    void SetIdentityTransform();						// Cancel the bed equation; does not reset axis angle compensation
    void Transform(float move[]) const;					// Take a position and apply the bed and the axis-angle compensations
    void InverseTransform(float move[]) const;			// Go from a transformed point back to user coordinates
    void Diagnostics();									// Report useful stuff

    const DeltaParameters& GetDeltaParams() const { return deltaParams; }
    DeltaParameters& AccessDeltaParams() { return deltaParams; }
    bool IsDeltaMode() const { return deltaParams.IsDeltaMode(); }
    const char* GetGeometryString() const;

    int GetCoreXYMode() const { return coreXYMode; }
    void SetCoreXYMode(int mode) { coreXYMode = mode; }
    float GetCoreAxisFactor(size_t axis) const { return axisFactors[axis]; }
    void setCoreAxisFactor(size_t axis, float f) { axisFactors[axis] = f; }
    bool IsCoreXYAxis(size_t axis) const;											// return true if the specified axis shares its motors with another

    void CurrentMoveCompleted();													// signals that the current move has just been completed
    bool StartNextMove(uint32_t startTime);											// start the next move, returning true if Step() needs to be called immediately
    void MotorTransform(const float machinePos[AXES], int32_t motorPos[AXES]) const;				// Convert Cartesian coordinates to delta motor coordinates
    float MotorFactor(size_t drive, const float directionVector[]) const;							// Calculate the movement fraction for a single axis motor of a Cartesian or CoreXY printer
    void MachineToEndPoint(const int32_t motorPos[], float machinePos[], size_t numDrives) const;	// Convert motor coordinates to machine coordinates
    void EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const;

	float IdleTimeout() const { return idleTimeout; }								// Returns the idle timeout in seconds
	void SetIdleTimeout(float timeout) { idleTimeout = timeout; }					// Set the idle timeout in seconds

    void Simulate(bool sim);														// Enter or leave simulation mode
    float GetSimulationTime() const { return simulationTime; }						// Get the accumulated simulation time
    void PrintCurrentDda() const;													// For debugging

    FilePosition PausePrint(float positions[DRIVES+1]);								// Pause the print as soon as we can
    bool NoLiveMovement() const;													// Is a move running, or are there any queued?

    int DoDeltaProbe(float frequency, float amplitude, float rate, float distance);

    static int32_t MotorEndPointToMachine(size_t drive, float coord);				// Convert a single motor position to number of steps
    static float MotorEndpointToPosition(int32_t endpoint, size_t drive);			// Convert number of motor steps to motor position

	bool IsExtruding() const;														// Is filament being extruded?

private:

    enum class IdleState : uint8_t { idle, busy, timing };

    void SetProbedBedEquation(size_t numPoints, StringRef& reply);					// When we have a full set of probed points, work out the bed's equation
    void DoDeltaCalibration(size_t numPoints, StringRef& reply);
    void BedTransform(float move[AXES]) const;										// Take a position and apply the bed compensations
    void GetCurrentMachinePosition(float m[DRIVES], bool disableMotorMapping) const;	// Get the current position in untransformed coords
    void InverseBedTransform(float move[AXES]) const;								// Go from a bed-transformed point back to user coordinates
    void AxisTransform(float move[AXES]) const;										// Take a position and apply the axis-angle compensations
    void InverseAxisTransform(float move[AXES]) const;								// Go from an axis transformed point back to user coordinates
    void BarycentricCoordinates(size_t p0, size_t p1,   							// Compute the barycentric coordinates of a point in a triangle
    		size_t p2, float x, float y, float& l1,     							// (see http://en.wikipedia.org/wiki/Barycentric_coordinate_system).
    		float& l2, float& l3) const;
    float TriangleZ(float x, float y) const;										// Interpolate onto a triangular grid
    void AdjustDeltaParameters(const float v[], size_t numFactors);					// Perform delta adjustment
    void JustHomed(size_t axis, float hitPoint, DDA* hitDDA);						// deal with setting positions after a drive has been homed

    static void PrintMatrix(const char* s, const MathMatrix<float>& m, size_t numRows = 0, size_t maxCols = 0);	// for debugging
    static void PrintVector(const char *s, const float *v, size_t numElems);		// for debugging

    bool DDARingAdd();									// Add a processed look-ahead entry to the DDA ring
    DDA* DDARingGet();									// Get the next DDA ring entry to be run
    bool DDARingEmpty() const;							// Anything there?

    DDA* volatile currentDda;
    DDA* ddaRingAddPointer;
    DDA* volatile ddaRingGetPointer;
    DDA* ddaRingCheckPointer;

    bool addNoMoreMoves;								// If true, allow no more moves to be added to the look-ahead
    bool active;										// Are we live and running?
    bool simulating;									// Are we simulating, or really printing?
    unsigned int idleCount;								// The number of times Spin was called and had no new moves to process
    float simulationTime;								// Print time since we started simulating
    volatile float liveCoordinates[DRIVES];				// The endpoint that the machine moved to in the last completed move
    volatile bool liveCoordinatesValid;					// True if the XYZ live coordinates are reliable (the extruder ones always are)
    volatile int32_t liveEndPoints[DRIVES];				// The XYZ endpoints of the last completed move in motor coordinates

    float xBedProbePoints[MAX_PROBE_POINTS];			// The X coordinates of the points on the bed at which to probe
    float yBedProbePoints[MAX_PROBE_POINTS];			// The Y coordinates of the points on the bed at which to probe
    float zBedProbePoints[MAX_PROBE_POINTS];			// The Z coordinates of the points on the bed at which to probe
    float baryXBedProbePoints[5];						// The X coordinates of the triangle corner points
    float baryYBedProbePoints[5];						// The Y coordinates of the triangle corner points
    float baryZBedProbePoints[5];						// The Z coordinates of the triangle corner points
    uint8_t probePointSet[MAX_PROBE_POINTS];			// Has the XY of this point been set?  Has the Z been probed?
    float aX, aY, aC; 									// Bed plane explicit equation z' = z + aX*x + aY*y + aC
    float tanXY, tanYZ, tanXZ; 							// Axis compensation - 90 degrees + angle gives angle between axes
    int numBedCompensationPoints;						// The number of points we are actually using for bed compensation, 0 means identity bed transform
    float xRectangle, yRectangle;						// The side lengths of the rectangle used for second-degree bed compensation

    float idleTimeout;									// How long we wait with no activity before we reduce motor currents to idle
    float lastMoveTime;									// The approximate time at which the last move was completed, or 0
    float longWait;										// A long time for things that need to be done occasionally
    IdleState iState;									// whether the idle timer is active

    DeltaParameters deltaParams;						// Information about the delta parameters of this machine
    DeltaProbe deltaProbe;								// Delta probing state
    uint32_t deltaProbingStartTime;
    bool deltaProbing;
    int coreXYMode;										// 0 = Cartesian, 1 = CoreXY, 2 = CoreXZ, 3 = CoreYZ
    float axisFactors[AXES];							// How much further the motors need to move for each axis movement, on a CoreXY/CoreXZ/CoreYZ machine
    unsigned int stepErrors;							// count of step errors, for diagnostics
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

// To wait until all the current moves in the buffers are
// complete, call this function repeatedly and wait for it to
// return true.  Then do whatever you wanted to do after all
// current moves have finished.  THEN CALL THE ResumeMoving() FUNCTION
// OTHERWISE NOTHING MORE WILL EVER HAPPEN.
inline bool Move::AllMovesAreFinished()
{
	addNoMoreMoves = true;
	return NoLiveMovement();
}

inline void Move::ResumeMoving()
{
	addNoMoreMoves = false;
}

#endif /* MOVE_H_ */
