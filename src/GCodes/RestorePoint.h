/*
 * RestorePoint.h
 *
 *  Created on: 14 Jun 2017
 *      Author: David
 */

#ifndef SRC_GCODES_RESTOREPOINT_H_
#define SRC_GCODES_RESTOREPOINT_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>

class RestorePoint INHERIT_OBJECT_MODEL
{
public:
	float moveCoords[MaxAxes];				// The axis locations when we paused
	float feedRate;							// The feed rate for the current move
	float virtualExtruderPosition;			// The virtual extruder position at the start of this move
	float proportionDone;					// How much of this move we have already done (zero unless we interrupted a move)
	FilePosition filePos;					// The file position that this move was read from
	float initialUserX, initialUserY;		// If we paused during an arc move and proportionDone is nonzero, the X and Y user coordinates at the start of the move
	int toolNumber;							// The tool number that was active
	int32_t spindleSpeeds[MaxSpindles];		// The spindle RPMs that were set, negative if anticlockwise direction

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;		// The output port bits setting for this move, or the laser power
#endif

	RestorePoint() noexcept;
	void Init() noexcept;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(coordinates)
	OBJECT_MODEL_ARRAY(spindleSpeeds)
};

#endif /* SRC_GCODES_RESTOREPOINT_H_ */
