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

	Tool *currentTool;												// which tool (if any) is being used
#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;								// the laser PWM or port bit settings required
#else
	uint16_t padding;
#endif
	uint8_t moveType;												// the S parameter from the G0 or G1 command, 0 for a normal move

	uint8_t applyM220M221 : 1,										// true if this move is affected by M220 and M221 (this could be moved to ExtendedRawMove)
			usePressureAdvance : 1,									// true if we want to us extruder pressure advance, if there is any extrusion
			canPauseAfter : 1,										// true if we can pause just after this move and successfully restart
			hasPositiveExtrusion : 1,								// true if the move includes extrusion; only valid if the move was set up by SetupMove
			isCoordinated : 1,										// true if this is a coordinated move
			usingStandardFeedrate : 1,								// true if this move uses the standard feed rate
			checkEndstops : 1,										// true if any endstops or the Z probe can terminate the move
			reduceAcceleration : 1;									// true if Z probing so we should limit the Z acceleration
	// If adding any more fields, keep the total size a multiple of 4 bytes so that we can use our optimised assignment operator

	void SetDefaults(size_t firstDriveToZero) noexcept;				// set up default values

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
struct MovementState : public RawMove
{
	// The current user position now holds the requested user position after applying workplace coordinate offsets.
	// So we must subtract the workplace coordinate offsets when we want to display them.
	// We have chosen this approach because it allows us to switch workplace coordinates systems or turn off applying workplace offsets without having to update currentUserPosition.
	float currentUserPosition[MaxAxes];								// The current position of the axes as commanded by the input gcode, after accounting for workplace offset,
																	// before accounting for tool offset and Z hop
	float latestVirtualExtruderPosition;							// The virtual extruder position of this movement system after completing pending moves
	float virtualFanSpeed;											// Last speed given in a M106 command with no fan number
	float currentZHop;												// The amount of Z hop that is currently applied
	float initialCoords[MaxAxes];									// the initial positions of the axes
	float previousX, previousY;										// the initial X and Y coordinates in user space of the previous move
	float previousXYDistance;										// the XY length of that previous move
	unsigned int currentCoordinateSystem;							// This is zero-based, where as the P parameter in the G10 command is 1-based
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
	GCodeQueue *codeQueue;											// Stores certain codes for deferred execution

	GCodeBuffer *null updateUserPositionGb;							// if this is non-null then we need to update the user position from the machine position

	unsigned int segmentsLeftToStartAt;
	float moveFractionToSkip;
	float firstSegmentFractionToSkip;

	float restartMoveFractionDone;									// how much of the next move was printed before the pause or power failure (from M26)
	float restartInitialUserC0;										// if the print was paused during an arc move, the user X coordinate at the start of that move (from M26)
	float restartInitialUserC1;										// if the print was paused during an arc move, the user Y coordinate at the start of that move (from M26)

	RestorePoint restorePoints[NumTotalRestorePoints];
	RestorePoint& pauseRestorePoint = restorePoints[PauseRestorePointNumber];				// The position and feed rate when we paused the print
	RestorePoint& toolChangeRestorePoint = restorePoints[ToolChangeRestorePointNumber];		// The position and feed rate when we freed a tool
	RestorePoint& simulationRestorePoint = restorePoints[SimulationRestorePointNumber];		// The position and feed rate when we started simulating
	RestorePoint& resumeObjectRestorePoint = restorePoints[ResumeObjectRestorePointNumber];	// The position and feed rate when we resumed printing objects

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
	FilePosition fileOffsetToPrint;									// The offset to print from
#endif

	// Tool change. These variables can be global because movement is locked while doing a tool change, so only one can take place at a time.
	int16_t newToolNumber;
	int16_t previousToolNumber;										// the tool number we were using before the last tool change, or -1 if we weren't using a tool
	uint8_t toolChangeParam;

	bool doingArcMove;												// true if we are doing an arc move
	bool xyPlane;													// true if the G17/G18/G19 selected plane of the arc move is XY in the original user coordinates
	SegmentedMoveState segMoveState;
	bool pausedInMacro;												// if we are paused then this is true if we paused while fileGCode was executing a macro

	// Object cancellation variables
	int currentObjectNumber;										// the current object number, or a negative value if it isn't an object
	int virtualToolNumber;											// the number of the tool that was active when we cancelled an object
	bool currentObjectCancelled;									// true if the current object should not be printed
	bool printingJustResumed;										// true if we have just restarted printing

	float GetProportionDone() const noexcept;						// get the proportion of this whole move that has been completed, based on segmentsLeft and totalSegments
	void Reset() noexcept;
	void ChangeExtrusionFactor(unsigned int extruder, float multiplier) noexcept;	// change the extrusion factor of an extruder
	const RestorePoint& GetRestorePoint(size_t n) const pre(n < NumTotalRestorePoints) { return restorePoints[n]; }
	void ClearMove() noexcept;
	void SavePosition(unsigned int restorePointNumber, size_t numAxes, float p_feedRate, FilePosition p_filePos) noexcept
		pre(restorePointNumber < NumTotalRestorePoints);

	// Tool management
	void SelectTool(int toolNumber, bool simulating) noexcept;
	ReadLockedPointer<Tool> GetLockedCurrentTool() const noexcept;
	ReadLockedPointer<Tool> GetLockedCurrentOrDefaultTool() const noexcept;
	int GetCurrentToolNumber() const noexcept;
	void SetPreviousToolNumber() noexcept;
	AxesBitmap GetCurrentXAxes() const noexcept;											// Get the current axes used as X axes
	AxesBitmap GetCurrentYAxes() const noexcept;											// Get the current axes used as Y axes
	AxesBitmap GetCurrentAxisMapping(unsigned int axis) const noexcept;
	float GetCurrentToolOffset(size_t axis) const noexcept;									// Get an axis offset of the current tool

	// Object cancellation support
	void InitObjectCancellation() noexcept;
	bool IsCurrentObjectCancelled() const noexcept { return currentObjectCancelled; }
	bool IsFirstMoveSincePrintingResumed() const noexcept { return printingJustResumed; }
	void DoneMoveSincePrintingResumed() noexcept { printingJustResumed = false; }
	void SetVirtualTool(int toolNum) noexcept { virtualToolNumber = toolNum; }
	void StopPrinting(GCodeBuffer& gb) noexcept;
	void ResumePrinting(GCodeBuffer& gb) noexcept;

	void Diagnostics(MessageType mtype, unsigned int moveSystemNumber) noexcept;
};

#if SUPPORT_ASYNC_MOVES

struct AsyncMove
{
	float movements[MaxAxesPlusExtruders];
	float startSpeed, endSpeed, requestedSpeed;
	float acceleration, deceleration;

	void SetDefaults() noexcept;
};

#endif

#endif /* SRC_GCODES_RAWMOVE_H_ */
