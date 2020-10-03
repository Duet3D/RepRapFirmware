/*
 * RawMove.h
 *
 *  Created on: 18 Apr 2019
 *      Author: David
 */

#ifndef SRC_GCODES_RAWMOVE_H_
#define SRC_GCODES_RAWMOVE_H_

#include "RepRapFirmware.h"

struct RawMove
{
	float coords[MaxAxesPlusExtruders];								// new positions for the axes, amount of movement for the extruders
	float initialCoords[MaxAxes];									// the initial positions of the axes
	float feedRate;													// feed rate of this move
	float virtualExtruderPosition;									// the virtual extruder position at the start of this move, for normal moves
	FilePosition filePos;											// offset in the file being printed at the start of reading this move
	float proportionDone;											// what proportion of the entire move has been done when this segment is complete
	float initialUserX, initialUserY;								// if this is a segment of an arc move, the user X and Y coordinates at the start
	const Tool *tool;												// which tool (if any) is being used
#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;								// the laser PWM or port bit settings required
#endif
	uint8_t moveType;												// the S parameter from the G0 or G1 command, 0 for a normal move

	uint8_t applyM220M221 : 1,										// true if this move is affected my M220 and M221
			usePressureAdvance : 1,									// true if we want to us extruder pressure advance, if there is any extrusion
			canPauseAfter : 1,										// true if we can pause just after this move and successfully restart
			hasPositiveExtrusion : 1,								// true if the move includes extrusion; only valid if the move was set up by SetupMove
			isCoordinated : 1,										// true if this is a coordinated move
			usingStandardFeedrate : 1,								// true if this move uses the standard feed rate
			checkEndstops : 1,										// true if any endstops or the Z probe can terminate the move
			reduceAcceleration : 1;									// true if Z probing so we should limit the Z acceleration

	void SetDefaults(size_t firstDriveToZero) noexcept;				// set up default values
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
