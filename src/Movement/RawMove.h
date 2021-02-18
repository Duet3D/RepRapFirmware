/*
 * RawMove.h
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#ifndef SRC_GCODES_RAWMOVE_H_
#define SRC_GCODES_RAWMOVE_H_

#include "RepRapFirmware.h"

// Details of a move that are passed from GCodes to Move
struct RawMove
{
	float coords[MaxAxesPlusExtruders];								// new positions for the axes, amount of movement for the extruders
	float initialUserC0, initialUserC1;								// if this is a segment of an arc move, the user XYZ coordinates at the start
	float feedRate;													// feed rate of this move
	float virtualExtruderPosition;									// the virtual extruder position at the start of this move, for normal moves
	FilePosition filePos;											// offset in the file being printed at the start of reading this move
	float proportionDone;											// what proportion of the entire move has been done when this segment is complete
	float cosXyAngle;												// the cosine of the change in XY angle between the previous move and this move
	const Tool *tool;												// which tool (if any) is being used
#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;								// the laser PWM or port bit settings required
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

	void SetDefaults(size_t firstDriveToZero) noexcept;				// set up default values
};

enum class SegmentedMoveState : uint8_t
{
	inactive = 0,
	active,
	aborted
};

// Details of a move that are needed only by GCodes
struct ExtendedRawMove : public RawMove
{
	float initialCoords[MaxAxes];									// the initial positions of the axes
	unsigned int segmentsLeft;										// the number of segments left to do in the current move, or 0 if no move available
	unsigned int totalSegments;										// the total number of segments left in the complete move
	float arcCentre[MaxAxes];
	float arcRadius;
	float arcCurrentAngle;
	float arcAngleIncrement;
	unsigned int arcAxis0, arcAxis1;
	bool doingArcMove;
	SegmentedMoveState segMoveState;

	float GetProportionDone() const noexcept;
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
