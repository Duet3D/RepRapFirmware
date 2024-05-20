/*
 * RawMove.h
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#ifndef SRC_GCODES_RAWMOVE_H_
#define SRC_GCODES_RAWMOVE_H_

#include <RepRapFirmware.h>
#include <GCodes/RestorePoint.h>

// Details of a move that are copied from GCodes to Move
struct RawMove
{
	float coords[MaxAxesPlusExtruders];								// new positions for the axes, amount of movement for the extruders
	float initialUserC0, initialUserC1;								// if this is a segment of an arc move, the user XYZ coordinates at the start
	float feedRate;													// feed rate of this move
	float moveStartVirtualExtruderPosition;							// the virtual extruder position at the start of this move, for normal moves
	FilePosition filePos;											// offset in the file being printed at the start of reading this move
	float proportionDone;											// what proportion of the entire move has been done when this segment is complete
	float cosXyAngle;												// the cosine of the change in XY angle between the previous move and this move
	float maxPrintingAcceleration;
	float maxTravelAcceleration;

	const Tool *movementTool;										// which tool (if any) is being used by this move

	uint16_t moveType : 3,											// the S parameter from the G0 or G1 command, 0 for a normal move
			applyM220M221 : 1,										// true if this move is affected by M220 and M221 (this could be moved to ExtendedRawMove)
			usePressureAdvance : 1,									// true if we want to us extruder pressure advance, if there is any extrusion
			canPauseAfter : 1,										// true if we can pause just after this move and successfully restart
			hasPositiveExtrusion : 1,								// true if the move includes extrusion; only valid if the move was set up by SetupMove
			isCoordinated : 1,										// true if this is a coordinated move
			usingStandardFeedrate : 1,								// true if this move uses the standard feed rate
			checkEndstops : 1,										// true if any endstops or the Z probe can terminate the move
			reduceAcceleration : 1,									// true if Z probing so we should limit the Z acceleration
			inverseTimeMode : 1,									// true if executing the move in inverse time mode
			linearAxesMentioned : 1,								// true if any linear axes were mentioned in the movement command
			rotationalAxesMentioned: 1								// true if any rotational axes were mentioned in the movement command
#if SUPPORT_SCANNING_PROBES
			, scanningProbeMove : 1									// true if the laser task should be woken at the end of each segment to capture a height reading
#endif
			;

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;								// the laser PWM or port bit settings required
# if !defined(DUET3) && !defined(DUET3MINI)
	uint16_t padding;												// pad to make the length a multiple of 4 bytes
# endif
#elif defined(DUET3) || defined(DUET3MINI)
	uint16_t padding;												// pad to make the length a multiple of 4 bytes
#endif

	// If adding any more fields, keep the total size a multiple of 4 bytes so that we can use our optimised assignment operator

	// GCC normally calls memcpy to assign objects of this class. We can do better because we know they must be 32-bit aligned.
	RawMove& operator=(const RawMove& arg) noexcept
	{
		memcpyu32(reinterpret_cast<uint32_t*>(this), reinterpret_cast<const uint32_t*>(&arg), sizeof(*this)/4);
		return *this;
	}
};

enum class SegmentedMoveState : uint8_t
{
	inactive = 0,
	active,
	aborted
};

constexpr size_t PauseRestorePointNumber = 1;
constexpr size_t ToolChangeRestorePointNumber = 2;

constexpr size_t NumTotalRestorePoints = NumVisibleRestorePoints + 2;			// The total number of visible + invisible restore points
constexpr size_t SimulationRestorePointNumber = NumVisibleRestorePoints;
constexpr size_t ResumeObjectRestorePointNumber = NumVisibleRestorePoints + 1;

// Details of a move that are needed only by GCodes
// CAUTION: segmentsLeft should ONLY be changed from 0 to not 0 by calling NewMoveAvailable()!
class MovementState : public RawMove
{
public:

#if SUPPORT_ASYNC_MOVES
	static void GlobalInit(size_t numVisibleAxes) noexcept;
	static const float *GetLastKnownMachinePositions() noexcept { return lastKnownMachinePositions; }
	static AxesBitmap GetAxesAndExtrudersMoved() noexcept { return axesAndExtrudersMoved; }

	AxesBitmap GetAxesAndExtrudersOwned() const noexcept { return axesAndExtrudersOwned; }	// Get the axes and extruders that this movement system owns
	ParameterLettersBitmap GetOwnedAxisLetters() const noexcept { return ownedAxisLetters; } // Get the letters denoting axes that this movement system owns
	AxesBitmap AllocateAxes(AxesBitmap axes, ParameterLettersBitmap axisLetters) noexcept;	// try to allocate the requested axes, if we can't then return the axes we can't allocate
	void ReleaseAllOwnedAxesAndExtruders() noexcept;
	void ReleaseNonToolAxesAndExtruders() noexcept;
	void ReleaseAxesAndExtruders(AxesBitmap axesToRelease) noexcept;
	void ReleaseAxisLetter(char letter) noexcept;											// stop claiming that we own an axis letter (if we do) but don't release the associated axis
	void SaveOwnAxisCoordinates() noexcept;													// fetch and save the coordinates of axes we own to lastKnownMachinePositions
	void OwnedAxisCoordinatesUpdated(AxesBitmap axesIncluded) noexcept;						// update changed coordinates of some owned axes - called after G92
	void OwnedAxisCoordinateUpdated(size_t axis) noexcept;									// update the machine coordinate of an axis we own - called after Z probing
#endif

	MovementSystemNumber GetMsNumber() const noexcept { return msNumber; }
	float GetProportionDone() const noexcept;												// get the proportion of this whole move that has been completed, based on segmentsLeft and totalSegments
	void Init(MovementSystemNumber p_msNumber) noexcept;
	void ResetLaser() noexcept;																// reset the laser parameters
	void ChangeExtrusionFactor(unsigned int extruder, float multiplier) noexcept;			// change the extrusion factor of an extruder
	const RestorePoint& GetRestorePoint(size_t n) const pre(n < NumTotalRestorePoints) { return restorePoints[n]; }
	void SetDefaults(size_t firstDriveToZero) noexcept;										// set up default values
	void ClearMove() noexcept;
	void SavePosition(unsigned int restorePointNumber, size_t numAxes, float p_feedRate, FilePosition p_filePos) noexcept
		pre(restorePointNumber < NumTotalRestorePoints);
	void ResumeAfterPause() noexcept;

	// Tool management
	void SelectTool(int toolNumber, bool simulating) noexcept;
	ReadLockedPointer<Tool> GetLockedCurrentTool() const noexcept;
	ReadLockedPointer<Tool> GetLockedCurrentOrDefaultTool() const noexcept;
	int GetCurrentToolNumber() const noexcept;
	void SetPreviousToolNumber() noexcept;
	AxesBitmap GetCurrentXAxes() const noexcept;											// Get the current axes used as X axes
	AxesBitmap GetCurrentYAxes() const noexcept;											// Get the current axes used as Y axes
	AxesBitmap GetCurrentZAxes() const noexcept;											// Get the current axes used as Y axes
	AxesBitmap GetCurrentAxisMapping(unsigned int axis) const noexcept;
	float GetCurrentToolOffset(size_t axis) const noexcept;									// Get an axis offset of the current tool

	// Object cancellation support
	void InitObjectCancellation() noexcept;
	bool IsCurrentObjectCancelled() const noexcept { return currentObjectCancelled; }
	bool IsFirstMoveSincePrintingResumed() const noexcept { return printingJustResumed; }
	void DoneMoveSincePrintingResumed() noexcept { printingJustResumed = false; }
	void StopPrinting(GCodeBuffer& gb) noexcept;
	void ResumePrinting(GCodeBuffer& gb) noexcept;

	float LiveCoordinate(unsigned int axisOrExtruder) const noexcept;

	void Diagnostics(MessageType mtype) noexcept;

	// These variables are currently all public, but we ought to make most of them private
	Tool *currentTool;												// the current tool of this movement system

	// The current user position now holds the requested user position after applying workplace coordinate offsets.
	// So we must subtract the workplace coordinate offsets when we want to display them.
	// We have chosen this approach because it allows us to switch workplace coordinates systems or turn off applying workplace offsets without having to update currentUserPosition.
	float currentUserPosition[MaxAxes];								// the current position of the axes as commanded by the input gcode, after accounting for workplace offset,
																	// before accounting for tool offset and Z hop
	float latestVirtualExtruderPosition;							// the virtual extruder position of this movement system after completing pending moves
	float virtualFanSpeed;											// the last speed given in a M106 command with no fan number
	float initialCoords[MaxAxes];									// the initial positions of the axes
	float previousX, previousY;										// the initial X and Y coordinates in user space of the previous move
	float previousXYDistance;										// the XY length of that previous move
	unsigned int currentCoordinateSystem;							// this is zero-based, where as the P parameter in the G10 command is 1-based
	unsigned int segmentsLeft;										// the number of segments left to do in the current move, or 0 if no move available
	unsigned int totalSegments;										// the total number of segments left in the complete move
	unsigned int arcAxis0, arcAxis1;								// the axis numbers of the arc before we apply axis mapping
	float arcCentre[MaxAxes];										// the arc centres coordinates of those axes that are moving in arcs
	float arcRadius;												// the arc radius before we apply scaling factors
	float arcCurrentAngle;											// the current angle of the arc relative to the +arcAxis0 direction
	float currentAngleSine, currentAngleCosine;						// the sine and cosine of the current angle
	float arcAngleIncrement;										// the amount by which we increment the arc angle in each segment
	float angleIncrementSine, angleIncrementCosine;					// the sine and cosine of the increment
	float speedFactor;												// speed factor as a fraction (normally 1.0)
	unsigned int segmentsTillNextFullCalc;							// how may more segments we can do before we need to do the full calculation instead of the quicker one
	GCodeQueue *codeQueue;											// stores certain codes for deferred execution

	GCodeBuffer *null updateUserPositionGb;							// if this is non-null then we need to update the user position from the machine position

	unsigned int segmentsLeftToStartAt;
	float moveFractionToSkip;
	float firstSegmentFractionToSkip;

	float restartMoveFractionDone;									// how much of the next move was printed before the pause or power failure (from M26)
	float restartInitialUserC0;										// if the print was paused during an arc move, the user X coordinate at the start of that move (from M26)
	float restartInitialUserC1;										// if the print was paused during an arc move, the user Y coordinate at the start of that move (from M26)

	mutable float latestLiveCoordinates[MaxAxesPlusExtruders];		// the most recent set of live coordinates that we fetched
	mutable uint32_t latestLiveCoordinatesFetchedAt = 0;			// when we fetched the live coordinates

	RestorePoint restorePoints[NumTotalRestorePoints];

	RestorePoint& GetPauseRestorePoint() noexcept { return restorePoints[PauseRestorePointNumber]; }				// The position and feed rate when we paused the print
	const RestorePoint& GetPauseRestorePoint() const noexcept { return restorePoints[PauseRestorePointNumber]; }	// The position and feed rate when we paused the print
	RestorePoint& GetToolChangeRestorePoint() noexcept { return restorePoints[ToolChangeRestorePointNumber]; }		// The position and feed rate when we freed a tool
	RestorePoint& GetSimulationRestorePoint() noexcept { return restorePoints[SimulationRestorePointNumber]; }		// The position and feed rate when we started simulating
	RestorePoint& GetResumeObjectRestorePoint() noexcept { return restorePoints[ResumeObjectRestorePointNumber]; }	// The position and feed rate when we resumed printing objects

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	FilePosition fileOffsetToPrint;									// the offset to start printing from
# if SUPPORT_ASYNC_MOVES
	FilePosition fileOffsetToSkipTo;								// when resuming in single-reader mode, skip commands until this file offset
# endif
#endif

#if SUPPORT_LASER
	LaserPixelData laserPixelData;									// when in laser mode, the number of S parameters in the G1 move  and their values converted to laser PWM
#endif

	// Tool change. These variables can be global because movement is locked while doing a tool change, so only one per movement system can take place at a time.
	int16_t newToolNumber;											// the tool number we are switching to, or the tool number we were supposed to switch to but didn't because the current object has been cancelled
	int16_t previousToolNumber;										// the tool number we were using before the last tool change, or -1 if we weren't using a tool
	uint8_t toolChangeParam;

	// Object cancellation variables
	int currentObjectNumber;										// the current object number, or a negative value if it isn't an object
	bool currentObjectCancelled;									// true if the current object should not be printed
	bool printingJustResumed;										// true if we have just restarted printing

	// Misc
	bool doingArcMove;												// true if we are doing an arc move
	bool xyPlane;													// true if the G17/G18/G19 selected plane of the arc move is XY in the original user coordinates
	SegmentedMoveState segMoveState;
	bool pausedInMacro;												// if we are paused then this is true if we paused while fileGCode was executing a macro
	bool forceLiveCoordinatesUpdate = true;							// true if we want to force latestLiveCoordinates to be updated

private:
	MovementSystemNumber msNumber;

#if SUPPORT_ASYNC_MOVES
	AxesBitmap axesAndExtrudersOwned;								// axes and extruders that this movement system has moved since the last sync
	ParameterLettersBitmap ownedAxisLetters;						// cache of letters denoting user axes for which the corresponding machine axes for the current tool are definitely owned

	static AxesBitmap axesAndExtrudersMoved;						// axes and extruders that are owned by any movement system
	static float lastKnownMachinePositions[MaxAxesPlusExtruders];	// the last stored machine position of the axes
#endif
};

#if SUPPORT_ASYNC_MOVES

// Stop claiming that we own an axis letter (if we do) but don't release the associated axis
inline void MovementState::ReleaseAxisLetter(char letter) noexcept
{
	ownedAxisLetters.ClearBit(ParameterLetterToBitNumber(letter));
}

struct AsyncMove
{
	float movements[MaxAxesPlusExtruders];
	float startSpeed, endSpeed, requestedSpeed;
	float acceleration, deceleration;

	void SetDefaults() noexcept;
};

#endif

#endif /* SRC_GCODES_RAWMOVE_H_ */
